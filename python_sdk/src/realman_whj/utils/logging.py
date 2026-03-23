"""
Logging utilities.
"""

import logging
import sys
from typing import Optional

# Default logger name
_LOGGER_NAME = "realman_whj"
_default_level = logging.INFO
_default_handler: Optional[logging.Handler] = None


def get_logger(name: str = None) -> logging.Logger:
    """
    Get logger instance.
    
    Args:
        name: Logger name (defaults to 'realman_whj')
        
    Returns:
        Logger instance
    """
    logger_name = f"{_LOGGER_NAME}.{name}" if name else _LOGGER_NAME
    logger = logging.getLogger(logger_name)
    
    # Setup default handler if not already set
    global _default_handler
    if _default_handler is None and not logger.handlers:
        _setup_default_handler()
    
    return logger


def _setup_default_handler():
    """Setup default logging handler."""
    global _default_handler
    
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    _default_handler = logging.StreamHandler(sys.stdout)
    _default_handler.setFormatter(formatter)
    
    logger = logging.getLogger(_LOGGER_NAME)
    logger.setLevel(_default_level)
    logger.addHandler(_default_handler)


def set_log_level(level: str or int):
    """
    Set logging level.
    
    Args:
        level: Logging level ('DEBUG', 'INFO', 'WARNING', 'ERROR') or logging constant
    """
    global _default_level
    
    if isinstance(level, str):
        level = getattr(logging, level.upper(), logging.INFO)
    
    _default_level = level
    logger = logging.getLogger(_LOGGER_NAME)
    logger.setLevel(level)


def add_file_handler(filepath: str, level: str or int = logging.DEBUG):
    """
    Add file logging handler.
    
    Args:
        filepath: Log file path
        level: Logging level for file handler
    """
    if isinstance(level, str):
        level = getattr(logging, level.upper(), logging.DEBUG)
    
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    handler = logging.FileHandler(filepath)
    handler.setLevel(level)
    handler.setFormatter(formatter)
    
    logger = logging.getLogger(_LOGGER_NAME)
    logger.addHandler(handler)
