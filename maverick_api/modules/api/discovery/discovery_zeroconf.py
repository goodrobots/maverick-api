from zeroconf import IPVersion, ServiceInfo, Zeroconf
import logging
import socket
import functools
from tornado.ioloop import PeriodicCallback
from tornado.options import options
from maverick_api.modules import moduleBase

application_log = logging.getLogger("tornado.application")
from maverick_api.modules.api.status import api_instance_uuid


class DiscoveryZeroconfModule(moduleBase):
    def __init__(self):
        super().__init__()
        logging.getLogger("zeroconf").setLevel(logging.DEBUG)
        self.periodic_callbacks = []
        self.discovered_api_instances = {}
        self.ip_version = IPVersion.V4Only  # IPVersion.All
        self.secure = False
        self.network = f"{socket.getfqdn()}:{options.server_port}"
        desc = {
            "httpEndpoint": f"{self.http_protocol}://{self.network}/graphql",
            "wsEndpoint": f"{self.ws_protocol}://{self.network}/subscriptions",
            "schemaEndpoint": f"{self.http_protocol}://{self.network}/schema",
            "websocketsOnly": False,
        }
        self.service_info = ServiceInfo(
            # TODO: FIXME come up with useful values for the info below
            "_http._tcp.local.",
            f"Maverick API {api_instance_uuid}._http._tcp.local.",
            addresses=[socket.inet_aton(options.server_interface)],
            port=options.server_port,
            properties=desc,
            server=f"{socket.getfqdn()}.",
        )

    def start(self):
        try:
            self.zeroconf = Zeroconf(ip_version=self.ip_version)
            self.zeroconf.register_service(self.service_info)
            self.install_periodic_callbacks()
            self.start_periodic_callbacks()
        except OSError as e:
            # the port was blocked
            application_log.info(
                f"Unable to start zeroconf server due to {e}, attempting to register with avahi"
            )
            # TODO: register by shell command or find bindings

    def shutdown(self):
        self.stop_periodic_callbacks()
        if self.zeroconf:
            self.zeroconf.unregister_service(self.service_info)
            self.zeroconf.close()

    @property
    def http_protocol(self):
        if self.secure:
            return "https"
        else:
            return "http"

    @property
    def ws_protocol(self):
        if self.secure:
            return "wss"
        else:
            return "ws"

    def scan_for_api_instances(self):
        queried_info = self.zeroconf.get_service_info(
            "_http._tcp.local.", "Maverick API._http._tcp.local."
        )
        # application_log.debug(f"Discovered Maverick API instances: {queried_info}")

    def install_periodic_callbacks(self):
        callback = functools.partial(self.scan_for_api_instances)
        self.periodic_callbacks.append(
            PeriodicCallback(callback, callback_time=5000, jitter=0.1)
        )

    def start_periodic_callbacks(self):
        for periodic_callback in self.periodic_callbacks:
            periodic_callback.start()

    def stop_periodic_callbacks(self):
        for periodic_callback in self.periodic_callbacks:
            periodic_callback.stop()
