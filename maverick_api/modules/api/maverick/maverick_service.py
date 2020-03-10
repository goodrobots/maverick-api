import logging
import copy
import os
import pathlib

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

from tornado.options import options


class MaverickServiceSchema(schemaBase):
    def __init__(self):
        super().__init__(self)
        self.service_command_name = "MaverickService"
        self.service_command_list_name = self.service_command_name + "List"
        self.service_definition_path = pathlib.Path(options.service_definition_path).expanduser().resolve()
        self.meta_info_file = "__init__"
        self.services = self.read_services()
        self.service_command_defaults = {"name": "", "category":"", "enabled": None, "running": None}
        self.service_command = copy.deepcopy(self.service_command_defaults)
        self.service_proc = None

        self.service_command_type = GraphQLObjectType(
            self.service_command_name,
            lambda: {
                "name": GraphQLField(GraphQLString, description="Service name",),
                "category": GraphQLField(GraphQLString, description="Service category",),
                "enabled": GraphQLField(
                    GraphQLBoolean, description="If the service is enabled at boot",
                ),
                "running": GraphQLField(
                    GraphQLBoolean, description="If the service is currently running"
                ),
            },
            description="Maverick service interface",
        )


        # self.service_command_list_type = GraphQLObjectType(
            
        # )

        self.q = {
            self.service_command_name: GraphQLField(
                self.service_command_type,
                args={"name": GraphQLArgument(GraphQLNonNull(GraphQLString)),},
                resolve=self.get_service_status,
            )
        }

        self.m = {
            self.service_command_name: GraphQLField(
                self.service_command_type,
                args=self.get_mutation_args(self.service_command_type),
                resolve=self.set_service_status,
            )
        }

        self.s = {
            self.service_command_name: GraphQLField(
                self.service_command_type,
                subscribe=self.sub_service_status,
                resolve=None,
            )
        }

    def read_services(self):
        services = []
        try:
            service_folders = [
                f.name for f in os.scandir(self.service_definition_path) if f.is_dir()
            ]
            service_folders.sort()
            for idx, service_folder in enumerate(service_folders):
                service_folder_path = self.service_definition_path.joinpath(service_folder)
                category = service_folder.split(".")[-1].strip().lower()  # e.g.: dev
                
                service_files = [
                    f.name for f in os.scandir(service_folder_path) if f.is_file()
                ]
                service_files.sort()
                sub_idx = 0
                
                if self.meta_info_file in service_files:
                    meta_info = ""
                    with open(service_folder_path.joinpath(self.meta_info_file), "r+") as fid:
                        meta_info = fid.read().strip()

                for service_file in [ x for x in service_files if x!= self.meta_info_file]:
                    with open(service_folder_path.joinpath(service_file), "r+") as fid:
                        service_info = fid.readlines()
                        for line in [x for x in service_info if x.strip()]:
                            line = line.split(",")
                            service_name = line[0].strip().lower()
                            display_name = line[1].strip()
                            services.append({"category_name":category, "category_display_name":meta_info, "command":f"maverick-{service_name}","service_name": service_name, "service_display_name":display_name, "category_index":idx, "service_index":sub_idx})
                            sub_idx += 1

            for service in services:
                application_log.debug(f"Adding service: {service}")

            
        except FileNotFoundError:
            application_log.warning(
                f"Could not find service folder at: {self.service_definition_path}"
            )

        return services

    async def set_service_status(self, root, info, **kwargs):
        application_log.debug(f"set_service_status {kwargs}")

        service_command = {}
        service_command["name"] = kwargs.get("name", "").lower()
        service_command["enabled"] = kwargs.get("enabled", None)
        service_command["category"] = kwargs.get("category", "").lower()
        service_command["running"] = kwargs.get("running", None)

        services = copy(self.services)

        # Both or one of category and name must be provided with the mutation
        if not service_command["category"] and not service_command["name"]:
            application_log.warining("No service category or service name was provided. No action taken.")
            return service_command

        # Filter the avalble services by category, if provided
        if service_command["category"]:
            services = [x for x in services if x["category_name"] == service_command["category"]]

        # Filter the avalble services by name, if provided
        if service_command["name"]:
            services = [x for x in services if x["service_name"] == service_command["name"]]
        
        # Check to ensure we have a service left over after the filtering
        if len(services) < 1:
            application_log.warining(f"No services were selected using service category = {service_command['category']} and service name = {service_command['name']}.")


        for service in services:
            # TODO: refactor duplicated code segments
            service_command["name"] = service["service_name"]

            if service_command["enabled"] is None:
                pass
            elif service_command["enabled"]:
                cmd = f"sudo systemctl enable {service}"
                ret = await self.run_command(cmd)
                if ret:
                    service_command["enabled"] = True
                else:
                    service_command["enabled"] = None
                self.emit_subscription()

            else:
                cmd = f"sudo systemctl disable {service}"
                ret = await self.run_command(cmd)
                if ret:
                    service_command["enabled"] = False
                else:
                    service_command["enabled"] = None
                self.emit_subscription()

            if service_command["running"] is None:
                pass
            elif service_command["running"]:
                cmd = f"sudo systemctl start {service}"
                ret = await self.run_command(cmd)
                if ret:
                    service_command["running"] = True
                else:
                    service_command["running"] = None
                self.emit_subscription()
            else:
                cmd = f"sudo systemctl stop {service}"
                ret = await self.run_command(cmd)
                if ret:
                    service_command["running"] = False
                else:
                    service_command["running"] = None
                self.emit_subscription()

            if len(services) == 1:
                await self._get_service_status(services)
        if len(services) >= 1:
            await self._get_service_status()

        return service_command

    async def get_service_status(self, root, info, **kwargs):
        application_log.debug(f"get_service_status {kwargs}")

        self.service_command["name"] = kwargs.get("name", "all").lower()
        await self._get_service_status()
        return self.service_command

    async def _get_service_status(self, services):
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
            if await self.service_proc.run() == 0:
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
        # re-check the service_proc after the above has run
        if not self.service_proc:
            return True

    def emit_subscription(self):
        self.subscriptions.emit(
            self.subscription_string + self.service_command_name, {self.service_command_name: self.service_command},
        )

    def sub_service_status(self, root, info):
        return EventEmitterAsyncIterator(
            self.subscriptions, self.subscription_string + self.service_command_name,
        )
