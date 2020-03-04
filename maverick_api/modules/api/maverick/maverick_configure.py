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
    GraphQLFloat,
)
from graphql.pyutils.event_emitter import EventEmitterAsyncIterator

application_log = logging.getLogger("tornado.application")


class MaverickConfigureSchema(schemaBase):
    def __init__(self):
        super().__init__(self)
        self.name = "MaverickConfigure"
        self.configure_command_defaults = {
            "environment": None,
            "module": None,
            "debug": False,
            "profile": False,
            "dryrun": False,
            "running": False,
            "uptime": None,
            "stdout": None,
            "stderror": None,
            "returncode": None,
        }
        self.configure_command = copy.deepcopy(self.configure_command_defaults)
        self.configure_proc = None

        self.configure_command_type = GraphQLObjectType(
            self.name,
            lambda: {
                "environment": GraphQLField(
                    GraphQLString,
                    description="Environment to configure: bootstrap, dev, flight or minimal",
                ),
                "dryrun": GraphQLField(
                    GraphQLBoolean,
                    description="Configure dry run - just show what would be done",
                ),
                "debug": GraphQLField(GraphQLBoolean, description="Debug output"),
                "profile": GraphQLField(
                    GraphQLBoolean, description="Profile performance data"
                ),
                "module": GraphQLField(
                    GraphQLString, description="Only make changes to module <module>"
                ),
                "running": GraphQLField(GraphQLBoolean, description=""),
                "uptime": GraphQLField(
                    GraphQLFloat,
                    description="Number of seconds the process has been running for",
                ),
                "terminate": GraphQLField(GraphQLBoolean, description=""),
                "stdout": GraphQLField(GraphQLString, description=""),
                "stderror": GraphQLField(GraphQLString, description=""),
                "returncode": GraphQLField(GraphQLInt, description=""),
            },
            description="Maverick configure interface",
        )

        self.q = {
            self.name: GraphQLField(
                self.configure_command_type, resolve=self.get_configure_command_status
            )
        }

        self.m = {
            self.name: GraphQLField(
                self.configure_command_type,
                args=self.get_mutation_args(self.configure_command_type),
                resolve=self.run_configure_command,
            )
        }

        self.s = {
            self.name: GraphQLField(
                self.configure_command_type,
                subscribe=self.sub_configure_command_status,
                resolve=None,
            )
        }

    async def run_configure_command(self, root, info, **kwargs):
        application_log.debug(f"run_configure_command {kwargs}")

        self.configure_command["dryrun"] = kwargs.get("dryrun", False)
        self.configure_command["debug"] = kwargs.get("debug", False)
        self.configure_command["profile"] = kwargs.get("profile", False)
        self.configure_command["module"] = kwargs.get("module", None)
        self.configure_command["environment"] = kwargs.get("environment", None)
        self.configure_command["terminate"] = kwargs.get("terminate", False)

        cmd = "maverick configure"
        if self.configure_command["environment"] is not None:
            cmd += f' --env={self.configure_command["environment"]}'
        if self.configure_command["dryrun"]:
            cmd += " --dryrun"
        if self.configure_command["debug"]:
            cmd += " --debug"
        if self.configure_command["profile"]:
            cmd += " --profile"
        if self.configure_command["module"] is not None:
            cmd += f' --module={self.configure_command["module"]}'
        application_log.info(f"Running configure command: {cmd}")

        if self.configure_command["terminate"]:
            # try to terminate a running command
            if self.configure_proc:
                self.configure_proc.terminate()
        if self.configure_proc:
            # already running?
            if self.configure_proc.complete:
                self.configure_proc = None
        if not self.configure_proc:
            # try to run the command
            self.configure_proc = ProcessRunner(
                cmd,
                started_callback=self.process_callback,
                output_callback=self.process_callback,
                complete_callback=self.process_callback,
            )
            self.configure_proc.start()
        return self.configure_command

    def process_callback(self, *args, **kwargs):
        self.configure_command["running"] = self.configure_proc.running
        self.configure_command["uptime"] = self.configure_proc.uptime
        self.configure_command["stdout"] = self.configure_proc.stdout
        self.configure_command["stderror"] = self.configure_proc.stderror
        self.configure_command["returncode"] = self.configure_proc.returncode
        application_log.debug(self.configure_proc.stdout_log)
        application_log.debug(self.configure_proc.stderror_log)
        self.subscriptions.emit(
            self.subscription_string + self.name, {self.name: self.configure_command},
        )
        return self.configure_command

    def sub_configure_command_status(self, root, info):
        return EventEmitterAsyncIterator(
            self.subscriptions, self.subscription_string + self.name,
        )

    def get_configure_command_status(self, root, info):
        return self.configure_command
