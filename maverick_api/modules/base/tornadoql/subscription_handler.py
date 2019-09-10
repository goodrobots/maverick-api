import logging
import time
from urllib.parse import urlparse

import tornado.ioloop
from tornado import websocket
from tornado.escape import json_decode
from tornado.options import options

from graphql import format_error
from graphql.subscription import subscribe
from graphql.language import parse
from graphql.execution import ExecutionResult
from graphql.subscription.map_async_iterator import MapAsyncIterator

from .session_control import GraphQLSession
from modules.api import api_schema

GRAPHQL_WS = "graphql-ws"
WS_PROTOCOL = GRAPHQL_WS

GQL_CONNECTION_INIT = "connection_init"  # Client -> Server
GQL_CONNECTION_ACK = "connection_ack"  # Server -> Client
GQL_CONNECTION_ERROR = "connection_error"  # Server -> Client

# NOTE: This does not follow the standard due to connection optimization
GQL_CONNECTION_TERMINATE = "connection_terminate"  # Client -> Server
GQL_CONNECTION_KEEP_ALIVE = "ka"  # Server -> Client
GQL_START = "start"  # Client -> Server
GQL_DATA = "data"  # Server -> Client
GQL_ERROR = "error"  # Server -> Client
GQL_COMPLETE = "complete"  # Server -> Client
GQL_STOP = "stop"  # Client -> Server

websocket_log = logging.getLogger("tornado.websocket")
application_log = logging.getLogger("tornado.application")


class SubscriptionObserver(object):
    def __init__(
        self, op_id, subscription, send_execution_result, send_error, on_close
    ):
        self.op_id = op_id
        self.subscription = subscription
        self.send_execution_result = send_execution_result
        self.send_error = send_error
        self.on_close = on_close
        self.completed = False

    async def main(self):
        """loop and return subscription results"""
        while not self.completed:
            try:
                result = await self.subscription.__anext__()
            except StopAsyncIteration:
                break
            try:
                await self.send_execution_result(self.op_id, result)
            except tornado.websocket.WebSocketClosedError:
                break

    async def dispose(self):
        self.completed = True
        await self.subscription.aclose()

    async def on_completed(self):
        await self.dispose()
        self.on_close()

    def on_error(self, error):
        self.send_error(self.op_id, error)


class GraphQLSubscriptionHandler(websocket.WebSocketHandler):
    def initialize(self):

        try:
            if options.apirate:
                self.rate_limit_data = True
                self.message_interval_seconds = 1.0 / options.apirate
                application_log.info(f"Setting API rate to {options.apirate}Hz")
            else:
                self.rate_limit_data = False
                self.message_interval_seconds = 1.0
                application_log.info(f"An API rate has not been set")
        except Exception as e:
            application_log.info(
                f"Error setting API rate, no rate will be enforced: {e}"
            )
            self.rate_limit_data = False
            self.message_interval_seconds = 1.0
        self.last_data_send_time = {}

        self.handler_sockets = []
        self.handler_subscriptions = {}
        self.start_time = time.time()

        self.remote_ip = ""
        # TODO: provide DB object via options to context
        self.context = {
            "authorization": None,
            "session": self.current_user,
            "db_access": None,
        }

    @property
    def uptime(self):
        return time.time() - self.start_time

    @property
    def schema(self):
        return api_schema

    @property
    def sockets(self):
        return self.handler_sockets

    @property
    def subscriptions(self):
        return self.handler_subscriptions.get(self, {})

    @subscriptions.setter
    def subscriptions(self, subscriptions):
        self.handler_subscriptions[self] = subscriptions

    def select_subprotocol(self, subprotocols):
        return WS_PROTOCOL

    def check_origin(self, origin):
        # TODO FIXME check origin rather than just returning True
        self.CORS_ORIGINS = ["localhost", "dev.maverick.one"]
        parsed_origin = urlparse(origin)
        return True

    async def send_message(self, op_id=None, op_type=None, payload=None):
        message = {}
        if op_id is not None:
            message["id"] = op_id
        if op_type is not None:
            message["type"] = op_type
            if op_type == GQL_DATA and self.rate_limit_data:
                current_time = time.time()
                # check to make sure we are within sending rate
                if (
                    current_time - self.last_data_send_time.get(op_id, 0)
                ) < self.message_interval_seconds:
                    return None
                else:
                    self.last_data_send_time[op_id] = current_time
        if payload is not None:
            message["payload"] = payload
        assert message, "You need to send at least one thing"
        websocket_log.debug(
            f"Websocket Send  {self.remote_ip} {self.uptime:.2f} {message}"
        )
        return await self.write_message(message)

    def send_error(self, op_id, error, error_type=None):
        if error_type is None:
            error_type = GQL_ERROR
        assert error_type in [GQL_CONNECTION_ERROR, GQL_ERROR], (
            "error_type should be one of the allowed error messages"
            " GQL_CONNECTION_ERROR or GQL_ERROR"
        )
        error_payload = {"message": str(error)}
        return self.send_message(op_id, error_type, error_payload)

    async def send_execution_result(self, op_id, execution_result):
        result = self.execution_result_to_dict(execution_result)
        return await self.send_message(op_id, GQL_DATA, result)

    def execution_result_to_dict(self, execution_result):
        result = {}
        if execution_result.data:
            result["data"] = execution_result.data
        if execution_result.errors:
            result["errors"] = [
                format_error(error) for error in execution_result.errors
            ]
        return result

    def get_graphql_params(self, payload):
        # application_log.debug(f"Subscription payload: {payload}")
        provided_context = payload.get("context", {})

        params = {
            "request_string": payload.get("query"),
            "variable_values": payload.get("variables"),
            "operation_name": payload.get("operationName"),
            "context_value": {**provided_context, **self.context},
        }
        return params

    # @Session.ensure_active_session
    async def open(self):
        # application_log.info("open socket %s", self)
        self.sockets.append(self)
        self.subscriptions = {}

    def on_close(self):
        tornado.ioloop.IOLoop.current().spawn_callback(self.close_subscriptions)

    async def close_subscriptions(self):
        # application_log.info("close socket %s", self)
        for sub in self.subscriptions:
            await self.subscriptions[sub].dispose()
        try:
            self.sockets.remove(self)
        except ValueError as e:
            # socket could not be found in list
            application_log.info(
                "Subscription socket was unable to be found during subscription close: {0}".format(
                    e
                )
            )
        self.subscriptions = {}

    def on_message(self, message):
        websocket_log.debug(
            f"Websocket Receive Raw {self.remote_ip} {self.uptime:.2f} {message}"
        )
        parsed_message = json_decode(message)
        # websocket_log.debug(
        #     f"Websocket Receive Parsed {self.remote_ip} {self.uptime:.2f} {parsed_message}"
        # )
        op_id = parsed_message.get("id")
        op_type = parsed_message.get("type")
        payload = parsed_message.get("payload")

        if op_type == GQL_CONNECTION_INIT:
            return self.on_connection_init(op_id, payload)

        elif op_type == GQL_CONNECTION_TERMINATE:
            return self.on_connection_terminate(op_id)

        elif op_type == GQL_START:
            assert isinstance(payload, dict), "The payload must be a dict"

            params = self.get_graphql_params(payload)
            if not isinstance(params, dict):
                error = Exception(
                    """Invalid params returned from get_graphql_params!
                    return values must be a dict."""
                )
                return self.send_error(op_id, error)
            return self.on_start(op_id, params)

        elif op_type == GQL_STOP:
            return self.on_stop(op_id)

        else:
            return self.send_error(
                op_id, Exception("Invalid message type: {}.".format(op_type))
            )

    def on_connection_init(self, op_id, payload):
        self.remote_ip = self.request.remote_ip
        # application_log.debug(f"Subscription connection init payload: {payload}")
        auth = payload.get("authorization", "")
        if "Bearer " in auth:
            auth = auth.lstrip("Bearer ")
        else:
            auth = None
        self.context["authorization"] = auth
        tornado.ioloop.IOLoop.current().spawn_callback(
            self.send_message, op_type=GQL_CONNECTION_ACK
        )

    def on_connection_terminate(self, op_id):
        self.close(code=1011)

    async def on_start(self, op_id, params):
        """Setup a subscription"""
        # application_log.debug(f"Attempting subscription: {op_id} {params}")
        subscription = await subscribe(
            schema=self.schema,
            document=parse(params["request_string"]),
            root_value=None,
            context_value=params["context_value"],
            variable_values=params["variable_values"],
            operation_name=params["operation_name"],
        )
        if isinstance(subscription, ExecutionResult):
            # There was an error during the subscription graphql call
            # TODO: log send errors back to client
            application_log.warn(
                f"Subscription failed: {op_id} {subscription.errors} {params}"
            )
            error = Exception(subscription.errors)
            return self.send_error(op_id, error)
        else:
            # The subscription graphql call successfully created a MapAsyncIterator
            # application_log.debug(f"Subscription successful: {op_id} {params}")
            await self.subscribe(op_id, subscription)

    async def on_stop(self, op_id):
        await self.unsubscribe(op_id)

    async def subscribe(self, op_id, subscription):
        if op_id in self.subscriptions:
            await self.subscriptions[op_id].dispose()
            del self.subscriptions[op_id]
        self.subscriptions[op_id] = SubscriptionObserver(
            op_id,
            subscription,
            self.send_execution_result,
            self.send_error,
            self.on_close,
        )
        # application_log.debug("subscriptions: %s", self.subscriptions)
        tornado.ioloop.IOLoop.current().spawn_callback(self.subscriptions[op_id].main)

    async def unsubscribe(self, op_id):
        # application_log.info("subscrption end: op_id=%s", op_id)
        await self.subscriptions[op_id].dispose()
        self.subscriptions = {n: s for n, s in self.subscriptions.items() if s != op_id}
        # application_log.debug("subscriptions: %s", self.subscriptions)
