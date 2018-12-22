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
)
from graphql.pyutils.event_emitter import EventEmitter, EventEmitterAsyncIterator

from modules.api import schemaBase
from modules.base.tornadoql.session_control import Session

user1 = dict(id="1", userName="sam", password="password1")
user2 = dict(id="2", userName="ben", password="password2")
auth_data = {user1["id"]: user1, user2["id"]: user2}


class AuthenticationSchema(schemaBase):
    def __init__(self):
        super().__init__()
        self.auth_type = GraphQLObjectType(
            "Authentication",
            lambda: {
                "id": GraphQLField(
                    GraphQLNonNull(GraphQLString), description="The id of user."
                ),
                "userName": GraphQLField(
                    GraphQLString,
                    description="The user name associated with the account.",
                ),
                "password": GraphQLField(
                    GraphQLString,
                    description="The hashed password associated with the account.",
                ),
            },
        )

        self.q = {
            "Authentication": GraphQLField(
                self.auth_type,
                args={
                    "id": GraphQLArgument(
                        GraphQLNonNull(GraphQLString), description="id of the user"
                    )
                },
                resolve=self.get_auth,
            )
        }

        self.m = {
            "Authentication": GraphQLField(
                self.auth_type,
                args=self.get_mutation_args(self.auth_type),
                resolve=self.set_auth,
            )
        }

        self.s = {
            "Authentication": GraphQLField(
                self.auth_type, subscribe=self.sub_auth, resolve=None
            )
        }

    def get_auth(self, root, info, id):
        """Authentication query handler"""
        return auth_data.get(id)

    @Session.authenticated
    async def set_auth(self, root, info, **kwargs):
        """Authentication mutation handler"""
        usr = auth_data.get(kwargs["id"])
        updated_dict = {**usr, **kwargs}
        self.subscriptions.emit(__name__, {"Authentication": updated_dict})
        auth_data[kwargs["id"]] = updated_dict
        return updated_dict

    def sub_auth(self, root, info, **kwargs):
        """Authentication subscription handler"""
        return EventEmitterAsyncIterator(self.subscriptions, __name__)
