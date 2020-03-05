import logging
import copy

from maverick_api.modules import schemaBase
from maverick_api.modules.base.util.process_runner import ProcessRunner

# graphql imports
from graphql import (
    GraphQLField,
    GraphQLObjectType,
    GraphQLString,
    GraphQLBoolean,
    GraphQLInt,
)
from graphql.pyutils.event_emitter import EventEmitterAsyncIterator

application_log = logging.getLogger("tornado.application")


class MaverickShutdownSchema(schemaBase):
    def __init__(self):
        super().__init__(self)
        self.name = "MaverickShutdown"
        self.shutdown_command_defaults = {
            "running": False,
            "uptime": None,
            "stdout": None,
            "stderror": None,
            "returncode": None,
        }
        self.shutdown_command = copy.deepcopy(self.shutdown_command_defaults)
        self.shutdown_proc = None

        self.shutdown_command_type = GraphQLObjectType(
            self.name,
            lambda: {
                "running": GraphQLField(GraphQLBoolean, description=""),
                "uptime": GraphQLField(
                    GraphQLInt,
                    description="Number of seconds the process has been running for",
                ),
                "stdout": GraphQLField(GraphQLString, description=""),
                "stderror": GraphQLField(GraphQLString, description=""),
                "returncode": GraphQLField(GraphQLInt, description=""),
            },
            description="Maverick shutdown interface",
        )

        self.q = {
            self.name: GraphQLField(
                self.shutdown_command_type, resolve=self.get_shutdown_command_status
            )
        }

        self.m = {
            self.name: GraphQLField(
                self.shutdown_command_type,
                args=self.get_mutation_args(self.shutdown_command_type),
                resolve=self.run_shutdown_command,
            )
        }

        self.s = {
            self.name: GraphQLField(
                self.shutdown_command_type,
                subscribe=self.sub_shutdown_command_status,
                resolve=None,
            )
        }

    async def run_shutdown_command(self, root, info, **kwargs):
        application_log.debug(f"run_shutdown_command {kwargs}")
        cmd = "sudo shutdown -a now"

        if self.shutdown_proc:
            # already running?
            if self.shutdown_proc.complete:
                self.shutdown_proc = None
        if not self.shutdown_proc:
            # try to run the command
            self.shutdown_proc = ProcessRunner(
                cmd,
                started_callback=self.process_callback,
                output_callback=self.process_callback,
                complete_callback=self.process_callback,
            )
            self.shutdown_proc.start()
        return self.shutdown_command

    def process_callback(self, *args, **kwargs):
        self.shutdown_command["running"] = self.shutdown_proc.running
        self.shutdown_command["uptime"] = self.shutdown_proc.uptime
        self.shutdown_command["stdout"] = self.shutdown_proc.stdout
        self.shutdown_command["stderror"] = self.shutdown_proc.stderror
        self.shutdown_command["returncode"] = self.shutdown_proc.returncode
        self.subscriptions.emit(
            self.subscription_string + self.name, {self.name: self.shutdown_command},
        )
        return self.shutdown_command

    def sub_shutdown_command_status(self, root, info):
        return EventEmitterAsyncIterator(
            self.subscriptions, self.subscription_string + self.name,
        )

    def get_shutdown_command_status(self, root, info):
        return self.shutdown_command
