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
        self.secure = not options.disable_ssl
        self.network = f"{socket.getfqdn()}"
        self.zeroconf = None
        subdesc = "{}:{}".format(
            socket.gethostname(),
            options.name if options.name else options.server_port_nonssl,
        )
        desc = {
            "httpEndpoint": f"http://{socket.getfqdn()}:{options.server_port_nonssl}/graphql",
            "wsEndpoint": f"ws://{socket.getfqdn()}:{options.server_port_nonssl}/subscriptions",
            "schemaEndpoint": f"http://{socket.getfqdn()}:{options.server_port_nonssl}/schema",
            "websocketsOnly": False,
            "uuid": api_instance_uuid,
            "service_type": "maverick-api",
            "name": subdesc,
            "hostname": socket.getfqdn(),
        }
        if self.secure:
            desc[
                "httpsEndpoint"
            ] = f"https://{socket.getfqdn()}:{options.server_port_ssl}/graphql"
            desc[
                "wssEndpoint"
            ] = f"wss://{socket.getfqdn()}:{options.server_port_ssl}/subscriptions"
            desc[
                "schemasEndpoint"
            ] = f"https://{socket.getfqdn()}:{options.server_port_ssl}/schema"
        self.service_info = ServiceInfo(
            "_api._tcp.local.",
            "maverick-api ({})._api._tcp.local.".format(subdesc),
            addresses=[socket.inet_aton(options.server_interface)],
            properties=desc,
            port=int(options.server_port_nonssl),
        )
        application_log.info("Zeroconf Service Info: {}".format(self.service_info))

    def start(self):
        try:
            self.zeroconf = Zeroconf(ip_version=self.ip_version)
            self.zeroconf.register_service(
                info=self.service_info, cooperating_responders=True
            )
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
            "_api._tcp.local.", "Maverick API._api._tcp.local."
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
