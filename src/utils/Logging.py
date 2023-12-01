import logging

def CreateLogger(name, level=logging.DEBUG):
    logger = logging.getLogger(name)
    logger.setLevel(level)

    ch = logging.StreamHandler()
    ch.setLevel(level)

    ch.setFormatter(_CustomFormatter())

    logger.addHandler(ch)

    return logger

class _CustomFormatter(logging.Formatter):
    format_i = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    format_wec = "%(asctime)s - %(name)s - %(levelname)s - %(message)s (%(filename)s:%(lineno)d)"

    FORMATS = {
        logging.DEBUG: format_i,
        logging.INFO: format_i,
        logging.WARNING: format_wec,
        logging.ERROR: format_wec,
        logging.CRITICAL: format_wec
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)

if __name__ == "__main__":
    logger = logging.getLogger("Test_Logger")
    logger.setLevel(logging.DEBUG)

    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)

    ch.setFormatter(_CustomFormatter())

    logger.addHandler(ch)

    logger.debug("debug message")
    logger.info("info message")
    logger.warning("warning message")
    logger.error("error message")
    logger.critical("critical message")