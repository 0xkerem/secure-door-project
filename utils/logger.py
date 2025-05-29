# utils/logger.py
import logging
import sys
from logging.handlers import RotatingFileHandler

def get_logger(name: str = __name__) -> logging.Logger:
    """
    Returns a configured logger instance.

    :param name: Name of the logger (usually __name__ of the module).
    :return: logging.Logger object
    """
    logger = logging.getLogger(name)

    # Prevent adding handlers multiple times
    if logger.hasHandlers():
        return logger

    logger.setLevel(logging.DEBUG)

    # Console handler: outputs INFO level and above to stdout
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)
    console_formatter = logging.Formatter(
        "%(asctime)s [%(levelname)-5s] %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )
    console_handler.setFormatter(console_formatter)
    logger.addHandler(console_handler)

    # File handler: rotating file for DEBUG level and above
    file_handler = RotatingFileHandler(
        filename="logs/app.log",
        maxBytes=5 * 1024 * 1024,
        backupCount=3,
        encoding="utf-8"
    )
    file_handler.setLevel(logging.DEBUG)
    file_formatter = logging.Formatter(
        "%(asctime)s [%(levelname)-5s] %(name)s (%(filename)s:%(lineno)d): %(message)s"
    )
    file_handler.setFormatter(file_formatter)
    logger.addHandler(file_handler)

    return logger
