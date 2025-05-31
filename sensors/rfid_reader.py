import serial
import time
from utils.logger import get_logger

logger = get_logger(__name__)

class RFIDReader:
    """
    Handles RFID card reading with RDM6300 via UART on Raspberry Pi.

    Attributes:
        serial_port (str): Serial port device path.
        baudrate (int): Baud rate for serial communication.
        authorized_uids (list): List of authorized card UIDs as strings.
    """

    def __init__(self, serial_port: str = "/dev/serial0", baudrate: int = 9600, authorized_uids: list = None):
        """
        Initialize RFID reader over UART and authorized UID list.

        Args:
            serial_port (str): Path to serial port device (default /dev/serial0).
            baudrate (int): Baud rate (default 9600).
            authorized_uids (list): List of authorized UID strings.
        """
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.authorized_uids = authorized_uids if authorized_uids else []
        self.last_uid = None  # Last read UID
        self.last_read_time = 0  # Last read timestamp
        self.debounce_interval = 2.0  # Debounce interval in seconds

        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            logger.info(f"RDM6300 initialized on {self.serial_port} at {self.baudrate} baud")
        except Exception as e:
            logger.error(f"Failed to open serial port {self.serial_port}: {e}")
            raise

    def read_card(self) -> tuple[bool, str]:
        """
        Wait for RFID card presence and read its UID.

        Returns:
            tuple (bool, str): 
                - bool: True if UID is authorized, False otherwise.
                - str: UID string from the card, or None if read fails or debounced.
        """
        try:
            # Read byte by byte until start byte (0x02) is found
            while True:
                byte = self.ser.read(1)
                if byte == b'\x02':
                    # Read the remaining 13 bytes of the frame (total 14 bytes)
                    frame = self.ser.read(13)
                    if len(frame) == 13 and frame[-1] == 0x03:
                        # Extract UID from the first 10 bytes (ASCII decoded)
                        tag_hex = frame[:10].decode('ascii')
                        
                        # Debounce check
                        current_time = time.time()
                        if tag_hex == self.last_uid and (current_time - self.last_read_time) < self.debounce_interval:
                            logger.debug(f"UID {tag_hex} debounced (skipped repeated read)")
                            return False, None
                        
                        # New UID or debounce interval passed
                        logger.info(f"RFID card detected with UID: {tag_hex}")
                        self.last_uid = tag_hex
                        self.last_read_time = current_time
                        is_authorized = tag_hex in self.authorized_uids
                        return is_authorized, tag_hex
                    else:
                        logger.warning("Invalid frame: Incomplete frame or end byte is not 0x03.")
                        return False, None
                elif not byte:
                    # Return None on timeout
                    return False, None

        except Exception as e:
            logger.error(f"Exception during RFID card reading: {e}")
            return False, None

    def cleanup(self):
        """
        Clean up serial connection.
        """
        try:
            if self.ser.is_open:
                self.ser.close()
                logger.info(f"Serial port {self.serial_port} closed")
        except Exception as e:
            logger.error(f"Failed to cleanup serial port: {e}")
            raise