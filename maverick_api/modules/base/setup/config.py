import os
import sys
import functools
import logging

from tornado import ioloop
from tornado.options import define, options

application_log = logging.getLogger("tornado.application")


class MavConfig(object):
    def __init__(self, config_file):
        self.config_file = config_file
        self.define_options()
        self.modify_time = None
        self.load_options()
        self.autoreload_config_file()

    # Define, config options
    def define_options(self):
        define(
            "generate_schema_and_exit",
            default=False,
            type=bool,
            help="Generate graphql schema and exit",
        )
        define("app_prefix", default="maverick-api/", type=str, help="URL prefix")
        define("app_secretkey", default="super_s3cret", type=str, help="Secret Key")
        define(
            "basedir",
            default=None,
            type=str,
            help="Base Directory, defaults to directory of maverick_api.py script if not set",
        )
        define(
            "config_file",
            default=self.config_file,
            type=str,
            help="Path to config file",
        )
        define(
            "database_backend",
            default="sqlite",
            type=str,
            help="Used to select the database backend",
        )
        define("datadir", default="data/", type=str, help="Data directory")
        define("debug", default=False, type=bool, help="Turn on debug mode")
        define(
            "development", default=False, type=bool, help="Turn on development options"
        )
        define("logdir", default="logs/", type=str, help="Log directory")
        define(
            "server_interface",
            default="0.0.0.0",
            type=str,
            help="Interface to listen on: 0.0.0.0 represents all interfaces",
        )
        define("server_port", default=6795, type=int, help="Port to listen on"),
        define(
            "disable_ssl",
            default=True,
            type=bool,
            help="Avoid loading ssl_keyfile and ssl_certfile",
        )
        define(
            "ssl_keyfile",
            default="",
            type=str,
            help="Path to valid SSL keyfile (not used if disable_ssl is set to True)",
        )
        define(
            "ssl_certfile",
            default="",
            type=str,
            help="Path to valid SSL certfile (not used if disable_ssl is set to True)",
        )
        define(
            "json_errors",
            default=True,
            type=bool,
            help="Return graphql errors in JSON format",
        )
        define(
            "apirate",
            default=0,
            type=float,
            help="Limit the API data rate in Hz, set to 0 to remove rate limits",
        )
        define(
            "module_allow_list",
            default=[],
            type=list,
            help="List of modules to allow loading. Takes precedence over deny list",
        )
        define(
            "module_deny_list",
            default=["mavros"],
            type=list,
            help="List of modules to deny loading",
        )
        define(
            "service_definition_path",
            default="~/software/maverick/bin/status.d",
            type=str,
            help="Path to search for the service definition file that is used to populate maverick_service",
        )
        define(
            "use_dbus", default=True, type=bool, help="Use DBUS for service monitoring",
        )

    # Parse and load config options
    def load_options(self):
        options.logging = None
        options.parse_command_line(final=False)
        try:
            options.parse_config_file(options.config_file)
        except FileNotFoundError as e:
            application_log.critical(
                f"Error, config file {options.config_file} not found"
            )
            sys.exit(1)

    def autoreload_config_file(self):
        application_log.debug("Starting autoreload_config_file")

        def reload_options_on_update(config_file):
            modified = os.stat(config_file).st_mtime
            if not self.modify_time:
                self.modify_time = modified
                return
            if self.modify_time != modified:
                application_log.info("Modified config file, reloading options")
                self.modify_time = modified
                self.load_options()

        config_callback = functools.partial(
            reload_options_on_update, options.config_file
        )
        ioloop.PeriodicCallback(config_callback, 2000).start()
