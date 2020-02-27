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


class MaverickShellSchema(schemaBase):
    def __init__(self):
        super().__init__()
        self.name = "MaverickShell"
        self.subscription_string = f"{__package__}.{self.__class__.__name__}"
        self.shell_command_defaults = {
            "command": "",
            "running": False,
            "uptime": None,
            "stdout": None,
            "stderror": None,
            "returncode": None,
        }
        self.shell_command = copy.deepcopy(self.shell_command_defaults)
        self.shell_proc = None

        self.shell_command_type = GraphQLObjectType(
            self.name,
            lambda: {
                "command": GraphQLField(
                    GraphQLString, description="The command to run in the shell",
                ),
                "running": GraphQLField(
                    GraphQLBoolean, description="Is a process running"
                ),
                "uptime": GraphQLField(
                    GraphQLFloat,
                    description="Number of seconds the process has been running for",
                ),
                "terminate": GraphQLField(GraphQLBoolean, description=""),
                "stdout": GraphQLField(GraphQLString, description=""),
                "stderror": GraphQLField(GraphQLString, description=""),
                "returncode": GraphQLField(GraphQLInt, description=""),
            },
            description="Maverick shell interface",
        )

        self.q = {
            self.name: GraphQLField(
                self.shell_command_type, resolve=self.get_shell_command_status
            )
        }

        self.m = {
            self.name: GraphQLField(
                self.shell_command_type,
                args=self.get_mutation_args(self.shell_command_type),
                resolve=self.run_shell_command,
            )
        }

        self.s = {
            self.name: GraphQLField(
                self.shell_command_type,
                subscribe=self.sub_shell_command_status,
                resolve=None,
            )
        }

    async def run_shell_command(self, root, info, **kwargs):
        application_log.debug(f"run_shell_command {kwargs}")

        self.shell_command["command"] = kwargs.get("command", "")
        self.shell_command["terminate"] = kwargs.get("terminate", False)
        cmd = self.shell_command["command"]
        application_log.info(f"Running shell command: {cmd}")

        if self.shell_command["terminate"]:
            # try to terminate a running command
            if self.shell_proc:
                self.shell_proc.terminate()
        if self.shell_proc:
            # already running?
            if self.shell_proc.complete:
                self.shell_proc = None
            else:
                application_log.debug("process is still running")
                cmd += "\n"
                self.shell_proc.process.stdin.write(cmd.encode())
                await self.shell_proc.process.stdin.drain()
        if not self.shell_proc:
            # try to run the command
            self.shell_proc = ProcessRunner(
                cmd,
                started_callback=self.start_process_callback,
                output_callback=self.output_process_callback,
                complete_callback=self.complete_process_callback,
            )
            self.shell_proc.start()
        return self.shell_command

    def start_process_callback(self, *args, **kwargs):
        ret = {}
        ret["command"] = kwargs["command"]
        ret["running"] = kwargs["running"]
        ret["uptime"] = kwargs["uptime"]
        ret["stdout"] = kwargs["stdout"]
        ret["stderror"] = kwargs["stderror"]
        ret["returncode"] = kwargs["returncode"]

    def output_process_callback(self, *args, **kwargs):
        ret = {}
        ret["command"] = kwargs["command"]
        ret["running"] = kwargs["running"]
        ret["uptime"] = kwargs["uptime"]
        ret["stdout"] = kwargs["stdout"]
        ret["stderror"] = kwargs["stderror"]
        ret["returncode"] = kwargs["returncode"]
        # if self.shell_proc.complete:
        self.subscriptions.emit(
            self.subscription_string + self.name, {self.name: ret},
        )

    def complete_process_callback(self, *args, **kwargs):
        ret = {}
        ret["command"] = kwargs["command"]
        ret["running"] = kwargs["running"]
        ret["uptime"] = kwargs["uptime"]
        ret["stdout"] = ""
        ret["stderror"] = ""
        ret["returncode"] = kwargs["returncode"]
        application_log.debug(self.shell_proc.stdout_log)
        application_log.debug(self.shell_proc.stderror_log)
        self.subscriptions.emit(
            self.subscription_string + self.name, {self.name: ret},
        )

    def sub_shell_command_status(self, root, info):
        return EventEmitterAsyncIterator(
            self.subscriptions, self.subscription_string + self.name,
        )

    def get_shell_command_status(self, root, info):
        return self.shell_command
