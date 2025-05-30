import RPi.GPIO as GPIO
from pirc522 import RFID
import time
from utils.logger import get_logger

logger = get_logger(__name__)

class RFIDReader:
    """
    Handles RFID card reading with MFRC522 reader via Raspberry Pi GPIO.


    Attributes:
        rst_pin (int): BCM GPIO pin number for RFID reset (RST).
        authorized_uids (list): List of authorized card UIDs as strings.
    """

    def __init__(self, rst_pin: int = 25, authorized_uids: list = None):
        """
        Initialize RFID reader hardware and authorized cards list.

        Args:
            rst_pin (int): BCM GPIO pin for RFID reset (default 25).
            authorized_uids (list): List of authorized UID strings.
        """
        self.rst_pin = rst_pin
        self.authorized_uids = authorized_uids if authorized_uids else []

        try:
            GPIO.setmode(GPIO.BCM)
            self.reader = RFID(pin_rst=self.rst_pin)
            logger.info(f"RFID reader initialized on RST pin {self.rst_pin}")
        except Exception as e:
            logger.error(f"Failed to initialize RFID reader on pin {self.rst_pin}: {e}")
            raise

    def read_card(self) -> tuple[bool, str]:
        """
        Wait for RFID card presence and read its UID.

        Returns:
            tuple (bool, str):
                - bool: True if UID is authorized, False otherwise.
                - str: UID string of the detected card, or None if no card.
        """
        try:
            self.reader.wait_for_tag()
            (error, tag_type) = self.reader.request()
            if error:
                logger.debug("No RFID card detected")
                return False, None

            (error, uid) = self.reader.anticoll()
            if error:
                logger.error("Failed to read RFID card UID")
                return False, None

            uid_str = "".join(str(x) for x in uid)
            logger.info(f"RFID card detected with UID: {uid_str}")

            is_authorized = uid_str in self.authorized_uids
            return is_authorized, uid_str

        except Exception as e:
            logger.error(f"Exception during RFID card reading: {e}")
            return False, None

    def cleanup(self):
        """
        Clean up RFID reader and GPIO resources.
        """
        try:
            self.reader.cleanup()
            GPIO.cleanup(self.rst_pin)
            logger.info(f"Cleaned up RFID reader on pin {self.rst_pin}")
        except Exception as e:
            logger.error(f"Failed to cleanup RFID reader: {e}")
            raise

# Note:
# BCM GPIO pin used for RFID reset: define explicitly, e.g. GPIO25.
