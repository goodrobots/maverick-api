#!/usr/bin/python3

"""
Tornado server for maverick-api
Samuel Dudley
Feb 2018
https://github.com/goodrobots/maverick-api
"""
__version__ = "0.2"

# TODO: setup tests and flake8

from tornado.options import options
from pathlib import Path, PurePath

from maverick_api.modules.base.database import MavDatabase
from maverick_api.modules.base.setup.config import MavConfig
from maverick_api.modules.base.setup.logging import MavLogging

if __name__ == "__main__":
    # Obtain basedir path (must be done from this script)
    basedir = Path(__file__).resolve().parent.joinpath("maverick_api")

    # Setup config
    MavConfig(PurePath(basedir).joinpath("config", "maverick-api.conf"))

    # Define basedir in options
    options.basedir = str(basedir)

    # Setup logging
    MavLogging()

    # Setup database
    MavDatabase()

    # Instantiate and start api server
    # Import at this point to allow config settings to take place
    from maverick_api.modules.base.apiserver import ApiServer

    api = ApiServer()
    api.initialize()
    api.serve()
