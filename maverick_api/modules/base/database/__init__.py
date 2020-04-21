import logging

# from pathlib import Path
from tornado.options import options
import tornado.ioloop
from tortoise import Tortoise

# from maverick_api.modules.base.util import functions

application_log = logging.getLogger("tornado.application")


class MavDatabase(object):
    """Abstraction layer used to communicate with selected database"""

    def __init__(self):
        self.backend = options.database_backend
        application_log.info(f"Database backend seleceted: {self.backend}")
        ioloop = tornado.ioloop.IOLoop.current()
        ioloop.run_sync(self.init)

    async def init(self):
        # Here we create a SQLite DB using file "db.sqlite3"
        #  also specify the app name of "models"
        #  which contain models from "app.models"
        await Tortoise.init(db_url="sqlite://db.sqlite3", modules={"models": []})
        # Generate the schema
        await Tortoise.generate_schemas()

    async def shutdown(self):
        await Tortoise.close_connections()
