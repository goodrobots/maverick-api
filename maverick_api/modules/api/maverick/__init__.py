import logging
import asyncio
import copy
import threading
import time
import re

from modules.api import moduleBase
from modules.api import schemaBase
from modules.api import api_callback

import tornado.ioloop
from tornado.options import options

# graphql imports
from graphql import (
    GraphQLArgument,
    GraphQLEnumType,
    GraphQLEnumValue,
    GraphQLField,
    GraphQLInterfaceType,
    GraphQLList,
    GraphQLNonNull,
    GraphQLObjectType,
    GraphQLSchema,
    GraphQLString,
    GraphQLBoolean,
    GraphQLInt,
    GraphQLFloat,
)
from graphql.pyutils.event_emitter import EventEmitter, EventEmitterAsyncIterator

application_log = logging.getLogger("tornado.application")


class MaverickSchema(schemaBase):
    def __init__(self):
        super().__init__()
        self.configure_command_defaults = {
            "enviroment": None,
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
            "MaverickConfigure",
            lambda: {
                "enviroment": GraphQLField(
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
                    GraphQLInt,
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
            "MaverickConfigure": GraphQLField(
                self.configure_command_type, resolve=self.get_configure_command_status
            )
        }

        self.m = {
            "MaverickConfigure": GraphQLField(
                self.configure_command_type,
                args=self.get_mutation_args(self.configure_command_type),
                resolve=self.run_configure_command,
            )
        }

        self.s = {
            "MaverickConfigure": GraphQLField(
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
        self.configure_command["enviroment"] = kwargs.get("enviroment", None)
        self.configure_command["terminate"] = kwargs.get("terminate", False)

        # TODO: remove this line
        self.configure_command["dryrun"] = True

        cmd = "maverick configure"
        if self.configure_command["enviroment"] is not None:
            cmd += f' --env={self.configure_command["enviroment"]}'
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
            else:
                pass
            
        elif self.configure_proc:
            # already running
            pass
        
        else:
            # try to run the command
            loop = tornado.ioloop.IOLoop.current()
            loop.add_callback(self.run, cmd)
            self.configure_command["running"] = True

        self.subscriptions.emit(
            "modules.api.maverick.MaverickSchema" + "MaverickConfigure",
            {"MaverickConfigure": self.configure_command},
        )
        return self.configure_command

    def sub_configure_command_status(self, root, info):
        return EventEmitterAsyncIterator(
            self.subscriptions,
            "modules.api.maverick.MaverickSchema" + "MaverickConfigure",
        )

    def get_configure_command_status(self, root, info):
        return self.configure_command

    async def run(self, cmd):
        start_time = time.time()
        self.configure_proc = await asyncio.create_subprocess_shell(
            cmd, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE
        )

        done = None
        pending = None
        while self.configure_proc.returncode is None:
            tasks = [
                MaverickSchema.read_from("stdout", self.configure_proc.stdout),
                MaverickSchema.read_from("stderror", self.configure_proc.stderr),
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
            self.configure_command["uptime"] = int(time.time() - start_time)
            for key, val in out.items():
                if val:
                    self.configure_command[key] = val
            self.subscriptions.emit(
                "modules.api.maverick.MaverickSchema" + "MaverickConfigure",
                {"MaverickConfigure": self.configure_command},
            )
        stdout, stderr = await self.configure_proc.communicate()
        application_log.info(f"{self.configure_proc.returncode}")
        self.configure_command["returncode"] = self.configure_proc.returncode
        self.configure_proc = None
        self.configure_command["running"] = False
        self.configure_command["stdout"] = None
        self.configure_command["stderror"] = None
        self.configure_command["uptime"] = None
        application_log.info(f"{pending}")
        for task in pending:
            task.cancel()
        await asyncio.sleep(2)
        self.subscriptions.emit(
            "modules.api.maverick.MaverickSchema" + "MaverickConfigure",
            {"MaverickConfigure": self.configure_command},
        )

        application_log.debug(
            f'[{cmd!r} exited with {self.configure_command["returncode"]}]'
        )

    @staticmethod
    async def read_from(name, source):
        stddata = await source.readline()
        line = stddata.decode("ascii").strip()
        # FIXME: this does not quite strip the colour codes from all text
        #   for the moment it does a pretty good job...
        line = re.sub(r'\x1B\[[0-?]*[ -/]*[@-~]', "", line, flags=re.IGNORECASE)
        return (name, line)
