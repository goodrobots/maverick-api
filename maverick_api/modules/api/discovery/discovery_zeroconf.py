from zeroconf import IPVersion, ServiceInfo, Zeroconf
import logging
import socket
import functools
from tornado.ioloop import PeriodicCallback
from tornado.options import options
from maverick_api.modules import moduleBase

application_log = logging.getLogger("tornado.application")


class DiscoveryZeroconfModule(moduleBase):
    def __init__(self, loop, module):
        super().__init__(loop, module)
        logging.getLogger("zeroconf").setLevel(logging.DEBUG)
        self.periodic_callbacks = []
        self.discovered_api_instances = {}
        ip_version = IPVersion.V4Only  # IPVersion.All
        self.network = f"{options.server_interface}:{options.server_port}"
        desc = {
            "httpEndpoint": f"http://{self.network}/graphql",
            "wsEndpoint": f"ws://{self.network}/subscriptions",
            "introspectionEndpoint": f"http://{self.network}/schema",
            "websocketsOnly": False,
        }
        self.service_info = ServiceInfo(
            "_http._tcp.local.",
            "Maverick API._http._tcp.local.",
            addresses=[socket.inet_aton(options.server_interface)],
            port=options.server_port,
            properties=desc,
            server="maverick-api.local.",
        )
        self.zeroconf = Zeroconf(ip_version=ip_version)
        self.zeroconf.register_service(self.service_info)
        self.install_periodic_callbacks()
        self.start_periodic_callbacks()

        # TODO: on exit call:
        # zeroconf.unregister_service(self.service_info)
        # zeroconf.close()

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
