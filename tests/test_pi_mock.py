"""Test cases for the PIControllerBase class in pi_controller module."""
import json
from unittest.mock import MagicMock, patch
# pylint: disable=import-error, no-name-in-module
from pi import PIControllerBase


@patch("pi.pi_controller.GCSDevice")
def test_connect_tcpip_daisy_chain(mock_gcs_device_cls):
    """Test connecting to a TCP/IP daisy chain."""
    mock_device = MagicMock()
    mock_device.OpenTCPIPDaisyChain.return_value = [
        "PI Device 1",
        "PI Device 2",
        "<device 3 not connected>",
    ]
    mock_device.dcid = 1
    mock_gcs_device_cls.return_value = mock_device

    controller = PIControllerBase(log=False)
    controller.connect_tcpip_daisy_chain("192.168.29.100", 10003)

    mock_device.OpenTCPIPDaisyChain.assert_called_once_with("192.168.29.100", 10003)
    assert controller.is_connected()
    assert controller.daisy_chains[("192.168.29.100", 10003)] == [
        (1, "PI Device 1"),
        (2, "PI Device 2"),
    ]
    assert (1, "PI Device 1") in controller.daisy_chains[("192.168.29.100", 10003)]
    assert (2, "PI Device 2") in controller.daisy_chains[("192.168.29.100", 10003)]
    assert ("192.168.29.100", 10003, 1) in controller.devices
    assert ("192.168.29.100", 10003, 2) in controller.devices


def test_list_devices_on_chain():
    """Test listing devices on a daisy chain."""
    controller = PIControllerBase(log=False)
    ip_port = ("192.168.29.100", 10003)
    controller.daisy_chains[ip_port] = [(1, "PI Device 1"), (2, "PI Device 2")]

    devices = controller.list_devices_on_chain(*ip_port)
    assert devices == [(1, "PI Device 1"), (2, "PI Device 2")]


@patch("pi.pi_controller.GCSDevice")
def test_connect_disconnect_device(mock_gcs_device_cls):
    """Test connecting and disconnecting a device."""
    mock_device = MagicMock()
    mock_gcs_device_cls.return_value = mock_device

    controller = PIControllerBase(log=False)
    controller.connect_tcp("127.0.0.1", 50000)
    device_key = ("127.0.0.1", 50000, 1)

    mock_device.ConnectTCPIP.assert_called_once_with("127.0.0.1", 50000)
    assert controller.is_connected()

    controller.disconnect_device(device_key)
    mock_device.CloseConnection.assert_called_once()
    assert not controller.is_connected()


def test_disconnect_all():
    """Test disconnecting all devices."""
    mock_device1 = MagicMock()
    mock_device2 = MagicMock()
    controller = PIControllerBase(log=False)
    controller.devices[("ip", 1, 1)] = mock_device1
    controller.devices[("ip", 1, 2)] = mock_device2
    controller._set_connected(True)

    controller.disconnect_all()
    mock_device1.CloseConnection.assert_called_once()
    mock_device2.CloseConnection.assert_called_once()
    assert not controller.devices
    assert not controller.is_connected()


def test_get_serial_number():
    """Test getting the serial number of a device."""
    controller = PIControllerBase(log=False)
    controller._set_connected(True)
    device_key = ("ip", 1, 1)
    device = MagicMock()
    device.qIDN.return_value = "PI,Model,123456,1.0.0"
    controller.devices[device_key] = device

    serial = controller.get_serial_number(device_key)
    assert serial == "123456"


@patch("pi.pi_controller.GCSDevice")
def test_get_pos(mock_gcs_device_cls):
    """Test getting the position of an axis."""
    mock_device = MagicMock()
    mock_gcs_device_cls.return_value = mock_device

    controller = PIControllerBase(log=False)
    controller._set_connected(True)
    device_key = ("ip", 1, 1)
    mock_device.axes = ["1", "2"]
    mock_device.qPOS.return_value = {"1": 42.0}
    controller.devices[device_key] = mock_device

    pos = controller.get_pos(device_key, "1")
    assert pos == 42.0
    mock_device.qPOS.assert_called_once_with("1")


def test_set_named_position(tmp_path):
    """Test setting a named position."""
    controller = PIControllerBase(log=False)
    controller.named_position_file = tmp_path / "positions.json"
    controller._set_connected(True)
    device_key = ("ip", 1, 1)
    device = MagicMock()
    device.axes = ["1"]
    # Mock qMOV to return a real float value
    device.qMOV.return_value = {"1": 10.0}
    controller.devices[device_key] = device
    controller.get_serial_number = MagicMock(return_value="123456")

    controller.set_named_position(device_key, "1", "home")

    with open(controller.named_position_file) as file:
        data = json.load(file)

    assert "123456" in data
    assert "home" in data["123456"]
    assert data["123456"]["home"][1] == 10.0


def test_go_to_named_position(tmp_path):
    """Test going to a named position."""
    controller = PIControllerBase(log=False)
    controller.named_position_file = tmp_path / "positions.json"
    controller._set_connected(True)
    device_key = ("ip", 1, 1)
    device = MagicMock()
    controller.devices[device_key] = device
    controller.get_serial_number = MagicMock(return_value="123456")
    controller.set_pos = MagicMock()

    # Prepare a named position file
    named_positions = {"123456": {"home": ["1", 42.0]}}
    with open(controller.named_position_file, "w") as file:
        json.dump(named_positions, file)

    controller.go_to_named_position(device_key, "home", blocking=False)
    controller.set_pos.assert_called_once_with(42.0, device_key, "1", False, 20)


@patch("pi.pi_controller.GCSDevice")
def test_home_success(mock_gcs_device_cls):
    """Test successful home (reference move)."""
    mock_device = MagicMock()
    mock_device.IsMoving.return_value = {"1": False}
    mock_gcs_device_cls.return_value = mock_device

    controller = PIControllerBase(log=False)
    controller._set_connected(True)
    device_key = ("ip", 1, 1)
    controller.devices[device_key] = mock_device

    # Test allowed method
    for method in ["FRF", "FNL", "FPL"]:
        getattr(mock_device, method).reset_mock()
        result = controller.home(
            device_key, "1", method=method, blocking=True, timeout=1
        )
        assert result is True
        getattr(mock_device, method).assert_called_once_with("1")


@patch("pi.pi_controller.GCSDevice")
def test_home_invalid_method(mock_gcs_device_cls):
    """Test home with an invalid method."""
    mock_device = MagicMock()
    mock_gcs_device_cls.return_value = mock_device

    controller = PIControllerBase(log=False)
    controller._set_connected(True)
    device_key = ("ip", 1, 1)
    controller.devices[device_key] = mock_device

    # Test disallowed method
    result = controller.home(device_key, "1", method="INVALID", blocking=True)
    assert result is False


@patch("pi.pi_controller.GCSDevice")
def test_home_timeout(mock_gcs_device_cls):
    """Test home with a timeout."""
    mock_device = MagicMock()
    # Simulate IsMoving always True
    mock_device.IsMoving.return_value = {"1": True}
    mock_gcs_device_cls.return_value = mock_device

    controller = PIControllerBase(log=False)
    controller._set_connected(True)
    device_key = ("ip", 1, 1)
    controller.devices[device_key] = mock_device

    result = controller.home(
        device_key, "1", method="FRF", blocking=True, timeout=0.1
    )
    assert result is False
