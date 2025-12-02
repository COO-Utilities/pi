"""
This module provides a base class for communicating with PI (Physik Instrumente) motion controllers
"""
import json
import os
import time
from pipython import GCSDevice, GCSError
from hardware_device_base import HardwareMotionBase


# Status codes
class PIStatus:
    """Status codes for PI controller operations."""
    OK = 0
    CONNECTED = 1
    MOVING = 2
    HOMED = 3

    # Error codes (negative)
    ERROR_NOT_CONNECTED = -1
    ERROR_CONNECTION_FAILED = -2
    ERROR_DEVICE_NOT_FOUND = -3
    ERROR_POSITION = -4
    ERROR_SERVO = -5
    ERROR_REFERENCE = -6
    ERROR_LIMITS = -7
    ERROR_TIMEOUT = -8
    ERROR_COMMAND = -9
    ERROR_INVALID_METHOD = -10


class PIControllerBase(HardwareMotionBase):
    """
    Base class for communicating with PI (Physik Instrumente) motion controllers daisy-chained
    over TCP/IP via a terminal server.
    """

    def __init__(self, log: bool = True, logfile: str = None):
        """
        Initialize the controller and set up logging.
        """
        # set up logging
        if logfile is None:
            logfile = self.__class__.__module__.rsplit(".", 1)[-1]
        super().__init__(log, logfile)

        self.devices = {}  # {(ip, port, device_id): GCSDevice instance}
        self.daisy_chains = {}  # {(ip, port): [(device_id, desc)]}
        self.named_position_file = "config/pi_named_positions.json"

    def _require_connection(self):
        """
        Raise an error if not connected to any device.
        """
        if not self.is_connected():
            self.report_error("Controller is not connected", PIStatus.ERROR_NOT_CONNECTED)
            raise RuntimeError("Controller is not connected")

    def connect_tcp(self, ip_address, port=50000):
        """
        Connect to a single PI controller via TCP/IP (non-daisy-chain).
        """
        device = GCSDevice()
        device.ConnectTCPIP(ip_address, port)
        self.devices[(ip_address, port, 1)] = device
        self._set_connected(True)
        self.report_info(f"Connected to single PI controller at {ip_address}:{port}", PIStatus.CONNECTED)

    def connect_tcpip_daisy_chain(self, ip_address, port, blocking=True):
        """
        Connect to all available devices on a daisy-chained set of PI controllers via TCP/IP.
        Each device is a separate GCSDevice instance.
        """
        main_device = GCSDevice()
        devices = main_device.OpenTCPIPDaisyChain(ip_address, port)
        dcid = main_device.dcid

        available = []
        for index, desc in enumerate(devices, start=1):
            if "not connected" not in desc.lower():
                available.append((index, desc))

        if not available:
            self.report_error(f"No connected devices found at {ip_address}:{port}", PIStatus.ERROR_DEVICE_NOT_FOUND)
            raise RuntimeError(f"No connected devices found at {ip_address}:{port}")

        self.daisy_chains[(ip_address, port)] = available

        for device_id, desc in available:
            if device_id == 1:
                dev = main_device
            else:
                dev = GCSDevice()

            dev.ConnectDaisyChainDevice(device_id, dcid)
            self.devices[(ip_address, port, device_id)] = dev
            self.report_info(f"[{ip_address}:{port}] Connected to device {device_id}: {desc}", PIStatus.CONNECTED)

        self._set_connected(True)

        if blocking:
            # Wait until all devices are ready
            while not all(dev.IsControllerReady() for dev in self.devices.values()):
                time.sleep(0.1)

    def disconnect_device(self, device_key):
        """
        Disconnect from a single device specified by device_key.
        """
        if device_key in self.devices:
            self.devices[device_key].CloseConnection()
            del self.devices[device_key]
            self.report_info(f"Disconnected device {device_key}", PIStatus.OK)
        if not self.devices:
            self._set_connected(False)

    def disconnect_all(self):
        """
        Disconnect from all devices (e.g., the whole daisychain).
        """
        for device_key in list(self.devices.keys()):
            self.devices[device_key].CloseConnection()
            self.report_info(f"Disconnected device {device_key}", PIStatus.OK)
        self.devices.clear()
        self._set_connected(False)
        self.report_info("Disconnected from all PI controllers", PIStatus.OK)

    def list_devices_on_chain(self, ip_address, port):
        """
        Return the list of available (device_id, description) tuples for the given daisy chain.
        """
        if (ip_address, port) not in self.daisy_chains:
            raise ValueError(f"No daisy chain found at {ip_address}:{port}")
        return self.daisy_chains[(ip_address, port)]

    def get_idn(self, device_key) -> str:
        """
        Return the identification string for the specified device.
        """
        self._require_connection()
        return self.devices[device_key].qIDN()

    def get_serial_number(self, device_key) -> str:
        """
        Return the serial number for the specified device.
        """
        idn = self.get_idn(device_key)
        return idn.split(",")[-2].strip()

    def get_axes(self, device_key):
        """
        Return the list of axes for the specified device.
        """
        self._require_connection()
        return self.devices[device_key].axes

    def get_pos(self, device_key=None, axis=None):
        """
        Get the position of the hardware motion device.

        Args:
            device_key: Tuple (ip, port, device_id) identifying the device
            axis: Axis to get position for

        Returns:
            Position value or None if error
        """
        if device_key is None or axis is None:
            self.report_error("device_key and axis are required for get_pos", PIStatus.ERROR_POSITION)
            return None

        self._require_connection()
        device = self.devices[device_key]
        try:
            return device.qPOS(axis)[axis]
        except (GCSError, IndexError) as ex:
            self.report_error(f"Error getting position: {ex}", PIStatus.ERROR_POSITION)
            return None

    def is_loop_closed(self, device_key=None, axis=None) -> bool:
        """
        Check if the hardware motion device servo is enabled (closed loop).

        Args:
            device_key: Tuple (ip, port, device_id) identifying the device
            axis: Axis to check

        Returns:
            True if servo is enabled, False otherwise
        """
        if device_key is None or axis is None:
            self.report_error("device_key and axis are required for is_loop_closed", PIStatus.ERROR_SERVO)
            return False

        self._require_connection()
        try:
            return bool(self.devices[device_key].qSVO(axis)[axis])
        except GCSError as ex:
            self.report_error(f"Error checking servo status: {ex}", PIStatus.ERROR_SERVO)
            return False

    def get_error_code(self, device_key):
        """
        Return the error code for the specified device, or None if an error occurs.
        """
        self._require_connection()
        try:
            return self.devices[device_key].qERR()
        except GCSError as ex:
            self.report_error(f"Error getting error code: {ex}", PIStatus.ERROR_COMMAND)
            return None

    def halt_motion(self, device_key):
        """
        Halt all motion for the specified device.
        """
        self._require_connection()
        try:
            self.devices[device_key].HLT()
        except GCSError as ex:
            self.report_error(f"Error halting motion: {ex}", PIStatus.ERROR_POSITION)

    def set_pos(self, pos, device_key=None, axis=None, blocking=True, timeout=20) -> bool:
        """
        Set the position.

        Args:
            pos: Target position
            device_key: Tuple (ip, port, device_id) identifying the device
            axis: Axis to move
            blocking: If True, wait until move is complete
            timeout: Timeout in seconds for blocking operation

        Returns:
            True if successful, False otherwise
        """
        if device_key is None or axis is None:
            self.report_error("device_key and axis are required for set_pos", PIStatus.ERROR_POSITION)
            return False

        self._require_connection()
        try:
            self.devices[device_key].MOV(axis, pos)
            if blocking:
                start_time = time.time()
                while self.is_moving(device_key, axis):
                    if time.time() - start_time > timeout:
                        self.report_error(
                            f"Move to position {pos} timed out after {timeout} seconds on axis {axis}",
                            PIStatus.ERROR_TIMEOUT
                        )
                        return False
                    time.sleep(0.1)
            return True
        except GCSError as ex:
            self.report_error(f"Error setting position: {ex}", PIStatus.ERROR_POSITION)
            return False

    def set_named_position(self, device_key, axis, name):
        """
        Save the current position of the axis under a named label, scoped to
        the controller serial number.
        """
        device = self.devices[device_key]
        try:
            pos = device.qMOV(axis)[axis]
        except (GCSError, OSError, ValueError):
            pos = self.get_pos(device_key, axis)

        if pos is None:
            self.report_warning(f"Could not get position for axis {axis}", PIStatus.ERROR_POSITION)
            return

        serial = self.get_serial_number(device_key)
        positions = {}

        if os.path.exists(self.named_position_file):
            with open(self.named_position_file, "r") as file:
                try:
                    positions = json.load(file)
                except json.JSONDecodeError:
                    self.report_warning(
                        f"Could not parse JSON from {self.named_position_file}", PIStatus.ERROR_POSITION
                    )

        if serial not in positions:
            positions[serial] = {}

        positions[serial][name] = [axis, pos]

        with open(self.named_position_file, "w") as file:
            json.dump(positions, file, indent=2)

        self.report_info(
            f"Saved position '{name}' for controller {serial}, axis {axis}: {pos}", PIStatus.OK
        )

    def go_to_named_position(self, device_key, name, blocking=True, timeout=20):
        """
        Move the specified device's axis to a previously saved named position.

        Args:
            device_key: Tuple (ip, port, device_id) identifying the device
            name: Name of the saved position
            blocking: If True, wait until move is complete
            timeout: Timeout in seconds for blocking operation
        """
        serial = self.get_serial_number(device_key)

        if not os.path.exists(self.named_position_file):
            self.report_warning(
                f"Named positions file not found: {self.named_position_file}", PIStatus.ERROR_POSITION
            )
            return

        try:
            with open(self.named_position_file, "r") as file:
                positions = json.load(file)
        except json.JSONDecodeError:
            self.report_warning(
                f"Failed to read positions from {self.named_position_file}", PIStatus.ERROR_POSITION
            )
            return

        if serial not in positions:
            self.report_warning(f"No named positions found for controller {serial}", PIStatus.ERROR_POSITION)
            return

        if name not in positions[serial]:
            self.report_warning(
                f"Named position '{name}' not found for controller {serial}", PIStatus.ERROR_POSITION
            )
            return

        axis, pos = positions[serial][name]
        self.set_pos(pos, device_key, axis, blocking, timeout)
        self.report_info(
            f"Moved axis {axis} to named position '{name}' for controller {serial}: {pos}", PIStatus.OK
        )

    def is_moving(self, device_key, axis):
        """Check if stage/axis is moving."""
        self._require_connection()
        return self.devices[device_key].IsMoving(axis)[axis]

    def close_loop(self, device_key=None, axis=None, enable=True) -> bool:
        """
        Close the loop (enable servo) for the hardware motion device.
        Can also be used to open the loop (disable servo) by setting enable=False.

        Args:
            device_key: Tuple (ip, port, device_id) identifying the device
            axis: Axis to enable servo for
            enable: True to close loop (enable servo), False to open loop (disable servo)

        Returns:
            True if successful, False otherwise
        """
        if device_key is None or axis is None:
            self.report_error("device_key and axis are required for close_loop", PIStatus.ERROR_SERVO)
            return False

        self._require_connection()
        try:
            self.devices[device_key].SVO(axis, int(enable))
            self.report_info(f"{'Closed (enabled servo)' if enable else 'Opened (disabled servo)'} loop for device {device_key}, axis {axis}", PIStatus.OK)
            return True
        except (GCSError, RuntimeError) as ex:
            self.report_error(f"Error {'closing' if enable else 'opening'} loop: {ex}", PIStatus.ERROR_SERVO)
            return False

    def get_limits(self, device_key=None, axis=None):
        """
        Get the limits of the hardware motion device.

        Args:
            device_key: Tuple (ip, port, device_id) identifying the device
            axis: Axis to get limits for

        Returns:
            Dictionary with axis as key and (min, max) tuple as value, or None if error
        """
        if device_key is None or axis is None:
            self.report_error("device_key and axis are required for get_limits", PIStatus.ERROR_LIMITS)
            return None

        self._require_connection()
        try:
            min_limit = self.devices[device_key].qTMN(axis)[axis]
            max_limit = self.devices[device_key].qTMX(axis)[axis]
            return {axis: (min_limit, max_limit)}
        except (GCSError, RuntimeError) as ex:
            self.report_error(f"Error getting limits: {ex}", PIStatus.ERROR_LIMITS)
            return None

    def is_controller_ready(self, device_key):
        """Check if stage/controller is ready."""
        self._require_connection()
        return self.devices[device_key].IsControllerReady()

    def is_homed(self, device_key=None, axis=None) -> bool:
        """
        Check if the hardware motion device is homed (referenced).

        Args:
            device_key: Tuple (ip, port, device_id) identifying the device
            axis: Axis to check

        Returns:
            True if axis is referenced, False otherwise
        """
        if device_key is None or axis is None:
            self.report_error("device_key and axis are required for is_homed", PIStatus.ERROR_REFERENCE)
            return False

        self._require_connection()
        return self.devices[device_key].qFRF(axis)[axis]

    def home(self, device_key=None, axis=None, method="FRF", blocking=True, timeout=20) -> bool: # pylint:disable=too-many-arguments
        """
        Home the hardware motion device.

        Args:
            device_key: Tuple (ip, port, device_id) identifying the device
            axis: Axis to home
            method: Reference method to use ("FRF", "FNL", "FPL")
            blocking: If True, wait until homing is complete
            timeout: Timeout in seconds for blocking operation

        Returns:
            True if successful, False otherwise
        """
        if device_key is None or axis is None:
            self.report_error("device_key and axis are required for home", PIStatus.ERROR_REFERENCE)
            return False

        self._require_connection()
        allowed_methods = {"FRF", "FNL", "FPL"}
        if method not in allowed_methods:
            self.report_error(
                f"Invalid reference method: {method}. Must be one of {allowed_methods}", PIStatus.ERROR_INVALID_METHOD
            )
            return False

        device = self.devices[device_key]

        # Check if the device supports the specified method
        if not getattr(device, "Has%s", method)():
            self.report_error(f"Device {device_key} does not support method '{method}'", PIStatus.ERROR_INVALID_METHOD)
            return False

        try:
            getattr(device, method)(axis)
            self.report_info(
                f"Started reference move '{method}' on axis {axis} (device {device_key})", PIStatus.HOMED
            )
            if blocking:
                start_time = time.time()
                while self.is_moving(device_key, axis):
                    if time.time() - start_time > timeout:
                        self.report_error(
                            f"Reference move timed out after {timeout} seconds on axis {axis}", PIStatus.ERROR_TIMEOUT
                        )
                        return False
                    time.sleep(0.1)

            return True
        except (GCSError, OSError, ValueError) as ex:
            self.report_error(
                f"Error during reference move '{method}' on axis {axis}: {ex}", PIStatus.ERROR_REFERENCE
            )
            return False

    def connect(self, *args, **kwargs) -> None:
        """
        Establish a connection to PI controller(s).

        This method supports multiple connection modes:
        1. TCP single device: connect(ip_address, port)
        2. TCP daisy chain: connect(ip_address, port, daisy_chain=True, blocking=True)

        Examples:
            controller.connect("192.168.1.100", 50000)
            controller.connect("192.168.1.100", 50000, daisy_chain=True)
        """
        if len(args) < 2:
            self.report_error("connect requires at least ip_address and port", PIStatus.ERROR_CONNECTION_FAILED)
            return

        ip_address = args[0]
        port = args[1]
        daisy_chain = kwargs.get('daisy_chain', False)
        blocking = kwargs.get('blocking', True)

        try:
            if daisy_chain:
                self.connect_tcpip_daisy_chain(ip_address, port, blocking=blocking)
            else:
                self.connect_tcp(ip_address, port)

            self._set_connected(True)
            self.report_info(f"Successfully connected to PI controller at {ip_address}:{port}", PIStatus.CONNECTED)
        except (GCSError, RuntimeError) as ex:
            self.report_error(f"Failed to connect: {ex}", PIStatus.ERROR_CONNECTION_FAILED)
            self._set_connected(False)

    def disconnect(self) -> None:
        """
        Disconnect from all PI controller devices.
        """
        try:
            self.disconnect_all()
            self._set_connected(False)
        except Exception as ex:
            self.report_error(f"Error during disconnect: {ex}", PIStatus.ERROR_CONNECTION_FAILED)

    def _send_command(self, command, device_key=None) -> bool:
        """
        Send a raw GCS command to the specified device.

        Args:
            command: GCS command string to send
            device_key: Tuple (ip, port, device_id) identifying the device

        Returns:
            True if command was sent successfully, False otherwise
        """
        if device_key is None:
            self.report_error("device_key is required for _send_command", PIStatus.ERROR_COMMAND)
            return False

        self._require_connection()

        try:
            with self.lock:
                device = self.devices[device_key]
                device.send(command)
            self.logger.debug(f"Sent command to device {device_key}: {command}")
            return True
        except (GCSError, KeyError) as ex:
            self.report_error(f"Error sending command: {ex}", PIStatus.ERROR_COMMAND)
            return False

    def _read_reply(self, device_key=None):
        """
        Read a reply from the specified device.

        Args:
            device_key: Tuple (ip, port, device_id) identifying the device

        Returns:
            Reply string or None if error
        """
        if device_key is None:
            self.report_error("device_key is required for _read_reply", PIStatus.ERROR_COMMAND)
            return None

        self._require_connection()

        try:
            device = self.devices[device_key]
            reply = device.read()
            self.logger.debug(f"Read reply from device {device_key}: {reply}")
            return reply
        except (GCSError, KeyError) as ex:
            self.report_error(f"Error reading reply: {ex}", PIStatus.ERROR_COMMAND)
            return None
