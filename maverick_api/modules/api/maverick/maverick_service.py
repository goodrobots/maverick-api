import logging
import copy
import os

from maverick_api.modules import schemaBase
from maverick_api.modules.base.util.process_runner import ProcessRunner


# graphql imports
from graphql import (
    GraphQLArgument,
    GraphQLField,
    GraphQLNonNull,
    GraphQLObjectType,
    GraphQLString,
    GraphQLBoolean,
)
from graphql.pyutils.event_emitter import EventEmitterAsyncIterator

application_log = logging.getLogger("tornado.application")


class MaverickServiceSchema(schemaBase):
    def __init__(self):
        super().__init__(self)
        self.name = "MaverickService"
        self.service_path = "~/software/maverick/bin/status.d"
        self.services = self.read_services()
        self.service_command_defaults = {"name": "", "enabled": None, "running": None}
        self.service_command = copy.deepcopy(self.service_command_defaults)
        self.service_proc = None

        self.service_command_type = GraphQLObjectType(
            self.name,
            lambda: {
                "name": GraphQLField(GraphQLString, description="Service name",),
                "enabled": GraphQLField(
                    GraphQLBoolean, description="If the service is enabled at boot",
                ),
                "running": GraphQLField(
                    GraphQLBoolean, description="If the service is currently running"
                ),
            },
            description="Maverick service interface",
        )

        self.q = {
            self.name: GraphQLField(
                self.service_command_type,
                args={"name": GraphQLArgument(GraphQLNonNull(GraphQLString)),},
                resolve=self.get_service_status,
            )
        }

        self.m = {
            self.name: GraphQLField(
                self.service_command_type,
                args=self.get_mutation_args(self.service_command_type),
                resolve=self.set_service_status,
            )
        }

        self.s = {
            self.name: GraphQLField(
                self.service_command_type,
                subscribe=self.sub_service_status,
                resolve=None,
            )
        }

    def read_services(self):
        # TODO: finish this
        try:
            service_folders = [
                f.name for f in os.scandir(self.service_path) if f.is_dir()
            ]
            for service_folder in service_folders:
                category = service_folder.split(".")[-1].strip()  # e.g.: dev
        except FileNotFoundError:
            application_log.warning(
                f"Could not find service folder at: {self.service_path}"
            )

        return {
            "maverick-apsitl@dev": {},
        }

    async def set_service_status(self, root, info, **kwargs):
        application_log.debug(f"set_service_status {kwargs}")

        self.service_command["name"] = kwargs.get("name", "").lower()
        self.service_command["enabled"] = kwargs.get("enabled", None)
        self.service_command["running"] = kwargs.get("running", None)

        services = []
        if self.service_command["name"] == "all":
            services = self.services.keys()
        else:
            services.append(self.service_command["name"])

        for service in services:
            self.service_command["name"] = service

            if self.service_command["enabled"] is None:
                pass
            elif self.service_command["enabled"]:
                cmd = f"sudo systemctl enable {service}"
                ret = await self.run_command(cmd)
                if ret:
                    self.service_command["enabled"] = True
                else:
                    self.service_command["enabled"] = None
                self.emit_subscription()

            else:
                cmd = f"sudo systemctl disable {service}"
                ret = await self.run_command(cmd)
                if ret:
                    self.service_command["enabled"] = False
                else:
                    self.service_command["enabled"] = None
                self.emit_subscription()

            if self.service_command["running"] is None:
                pass
            elif self.service_command["running"]:
                cmd = f"sudo systemctl start {service}"
                ret = await self.run_command(cmd)
                if ret:
                    self.service_command["running"] = True
                else:
                    self.service_command["running"] = None
                self.emit_subscription()
            else:
                cmd = f"sudo systemctl stop {service}"
                ret = await self.run_command(cmd)
                if ret:
                    self.service_command["running"] = False
                else:
                    self.service_command["running"] = None
                self.emit_subscription()

            await self._get_service_status()
        return self.service_command

    async def get_service_status(self, root, info, **kwargs):
        application_log.debug(f"get_service_status {kwargs}")

        self.service_command["name"] = kwargs.get("name", "all").lower()
        await self._get_service_status()
        return self.service_command

    async def _get_service_status(self):
        services = []
        if self.service_command["name"] == "all":
            services = self.services.keys()
        else:
            services.append(self.service_command["name"])

        for service in services:
            self.service_command["name"] = service
            cmd = f"systemctl --no-pager status {service}"
            ret = await self.run_command(cmd)
            if ret:
                self.service_command["running"] = True
            else:
                self.service_command["running"] = False
            self.emit_subscription()

            cmd = f"systemctl is-enabled {service}"
            ret = await self.run_command(cmd)
            if ret:
                self.service_command["enabled"] = True
            else:
                self.service_command["enabled"] = False
            self.emit_subscription()

    async def run_command(self, cmd):
        if cmd and self.okay_to_run():
            self.service_proc = ProcessRunner(cmd)
            await self.service_proc.run()
            if self.service_proc.returncode == 0:
                return True
            else:
                return False
        else:
            application_log.warning(f"Not able to run service command: {cmd}")

    def okay_to_run(self):
        if self.service_proc:
            # already running?
            if self.service_proc.complete:
                self.service_proc = None
            else:
                return False
        if not self.service_proc:
            return True

    def emit_subscription(self):
        self.subscriptions.emit(
            self.subscription_string + self.name, {self.name: self.service_command},
        )

    def sub_service_status(self, root, info):
        return EventEmitterAsyncIterator(
            self.subscriptions, self.subscription_string + self.name,
        )
