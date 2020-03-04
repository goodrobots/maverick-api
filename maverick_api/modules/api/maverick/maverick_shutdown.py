import logging
import asyncio
import time
import re

from maverick_api.modules import schemaBase

import tornado.ioloop

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
        self.configure_proc = None

        self.shutdown_command = {
            "running": False,
            "uptime": None,
            "stdout": None,
            "stderror": None,
            "returncode": None,
        }

        self.shutdown_command_type = GraphQLObjectType(
            "MaverickShutdown",
            lambda: {
                "running": GraphQLField(GraphQLBoolean, description=""),
                "uptime": GraphQLField(
                    GraphQLInt,
                    description="Number of seconds the process has been running for",
                ),
                "terminate": GraphQLField(GraphQLBoolean, description=""),
                "stdout": GraphQLField(GraphQLString, description=""),
                "stderror": GraphQLField(GraphQLString, description=""),
                "returncode": GraphQLField(GraphQLInt, description=""),
            },
            description="Maverick shutdown interface",
        )

        self.q = {
            "MaverickShutdown": GraphQLField(
                self.shutdown_command_type, resolve=self.get_shutdown_command_status
            )
        }

        self.m = {
            "MaverickShutdown": GraphQLField(
                self.shutdown_command_type,
                args=self.get_mutation_args(self.shutdown_command_type),
                resolve=self.run_shutdown_command,
            )
        }

        self.s = {
            "MaverickShutdown": GraphQLField(
                self.shutdown_command_type,
                subscribe=self.sub_shutdown_command_status,
                resolve=None,
            )
        }

    async def run_shutdown_command(self, root, info, **kwargs):
        application_log.debug(f"run_shutdown_command {kwargs}")
        # cmd = "sudo shutdown"
        cmd = "pwd"

        loop = tornado.ioloop.IOLoop.current()
        loop.add_callback(self.run, cmd)
        self.shutdown_command["running"] = True

        self.subscriptions.emit(
            "maverick_api.modules.api.maverick.MaverickShutdownSchema"
            + "MaverickShutdown",
            {"MaverickShutdown": self.shutdown_command},
        )
        return self.shutdown_command

    def sub_shutdown_command_status(self, root, info):
        return EventEmitterAsyncIterator(
            self.subscriptions,
            "maverick_api.modules.api.maverick.MaverickShutdownSchema"
            + "MaverickShutdown",
        )

    def get_shutdown_command_status(self, root, info):
        return self.shutdown_command

    async def run(self, cmd):
        start_time = time.time()
        self.configure_proc = await asyncio.create_subprocess_shell(
            cmd, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE
        )

        done = None
        pending = None
        while self.configure_proc.returncode is None:
            tasks = [
                MaverickShutdownSchema.read_from("stdout", self.configure_proc.stdout),
                MaverickShutdownSchema.read_from(
                    "stderror", self.configure_proc.stderr
                ),
            ]
            out = {"stdout": "", "stderror": ""}
            done, pending = await asyncio.wait(
                tasks, timeout=2.0, return_when=asyncio.FIRST_COMPLETED
            )
            for task in pending:
                task.cancel()
            for task in done:
                application_log.debug(f"{task.result()}")
                if task.result():
                    (out_name, out_string) = task.result()
                    out[out_name] = out[out_name] + out_string
            # out_string += line
            self.shutdown_command["uptime"] = int(time.time() - start_time)
            for key, val in out.items():
                if val:
                    self.shutdown_command[key] = val
            self.subscriptions.emit(
                "maverick_api.modules.api.maverick.MaverickShutdownSchema"
                + "MaverickShutdown",
                {"MaverickShutdown": self.shutdown_command},
            )
        stdout, stderr = await self.configure_proc.communicate()
        application_log.info(f"{self.configure_proc.returncode}")
        self.shutdown_command["returncode"] = self.configure_proc.returncode
        self.configure_proc = None
        self.shutdown_command["running"] = False
        self.shutdown_command["stdout"] = None
        self.shutdown_command["stderror"] = None
        self.shutdown_command["uptime"] = None
        application_log.info(f"{pending}")
        for task in pending:
            task.cancel()
        await asyncio.sleep(2)
        self.subscriptions.emit(
            "maverick_api.modules.api.maverick.MaverickShutdownSchema"
            + "MaverickShutdown",
            {"MaverickShutdown": self.shutdown_command},
        )

        application_log.debug(
            f'[{cmd} exited with {self.shutdown_command["returncode"]}]'
        )

    @staticmethod
    async def read_from(name, source):
        stddata = await source.readline()
        line = stddata.decode("ascii").strip()
        # FIXME: this does not quite strip the colour codes from all text
        #   for the moment it does a pretty good job...
        line = re.sub(r"\x1B\[[0-?]*[ -/]*[@-~]", "", line, flags=re.IGNORECASE)
        return (name, line)
