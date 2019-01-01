import logging
from tornado.options import options

application_log = logging.getLogger("tornado.application")

class MavDatabase(object):
    """Abstraction layer used to communicate with selected database"""
    def __init__(self):
        self.backend = options.database_backend
        application_log.info(f"Database backend seleceted: {self.backend}")