import logging
from tornado.options import options

application_log = logging.getLogger("tornado.application")


class MavDatabase(object):
    """Abstraction layer used to communicate with selected database"""

    def __init__(self):
        self.backend = options.database_backend
        application_log.info(f"Database backend seleceted: {self.backend}")

        if self.backend == "tinydb":  # lightweight non-async json storage
            import tinydb  # noqa: F401
        elif self.backend == "mongo":  # async access to mongo database
            import motor  # noqa: F401
        elif self.backend == "sqlite":  # async access to sqlite3 database
            import aiosqlite  # noqa: F401
        else:
            pass
