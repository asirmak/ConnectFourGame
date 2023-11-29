import logging

class ProjectLogger():
    pass

class _CustomFormatter(logging.Formatter):
    grey = "\x1b[38;20m"
    yellow = "\x1b[33;20m"
    red = "\x1b[31;20m"
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"
    format_i = "%(asctime)s - %(levelname)s - %(message)s"
    format_wec = "%(asctime)s - %(levelname)s - %(message)s (%(filename)s:%(lineno)d)"

    FORMATS = {
        logging.DEBUG: grey + format_i + reset,
        logging.INFO: grey + format_i + reset,
        logging.WARNING: yellow + format_wec + reset,
        logging.ERROR: red + format_wec + reset,
        logging.CRITICAL: bold_red + format_wec + reset
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)

if __name__ == "__main__":
    # create logger with 'spam_application'
    logger = logging.getLogger("My_app")
    logger.setLevel(logging.DEBUG)

    # create console handler with a higher log level
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)

    ch.setFormatter(_CustomFormatter())

    logger.addHandler(ch)

    logger.debug("debug message")
    logger.info("info message")
    logger.warning("warning message")
    logger.error("error message")
    logger.critical("critical message")