from graphql import (
    GraphQLArgument,
    GraphQLField,
    GraphQLNonNull,
    GraphQLObjectType,
    GraphQLString,
)
from graphql.pyutils.simple_pub_sub import SimplePubSubIterator

from maverick_api.modules import schemaBase
from maverick_api.modules.base.tornadoql.session_control import GraphQLSession

user1 = dict(id="1", userName="bob", password="password1")
user2 = dict(id="2", userName="ben", password="password2")
auth_data = {user1["id"]: user1, user2["id"]: user2}


class AuthenticationSchema(schemaBase):
    def __init__(self):
        super().__init__(self)

        self.auth_request_type = GraphQLObjectType(
            "AuthenticationRequest",
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

        self.auth_status_type = GraphQLObjectType(
            "AuthenticationStatus",
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
            "AuthenticationRequest": GraphQLField(
                self.auth_request_type,
                args={
                    "id": GraphQLArgument(
                        GraphQLNonNull(GraphQLString), description="id of the user"
                    )
                },
                resolve=self.get_auth,
            ),
            "AuthenticationStatus": GraphQLField(
                self.auth_status_type,
                args={
                    "id": GraphQLArgument(
                        GraphQLNonNull(GraphQLString), description="id of the user"
                    )
                },
                resolve=self.get_auth,
            ),
        }

        self.m = {
            "AuthenticationRequest": GraphQLField(
                self.auth_request_type,
                args=self.get_mutation_args(self.auth_request_type),
                resolve=self.set_auth,
            ),
            "AuthenticationStatus": GraphQLField(
                self.auth_status_type,
                args=self.get_mutation_args(self.auth_request_type),
                resolve=self.set_auth,
            ),
        }

        self.s = {
            "AuthenticationRequest": GraphQLField(
                self.auth_request_type, subscribe=self.sub_auth, resolve=None
            ),
            "AuthenticationStatus": GraphQLField(
                self.auth_status_type, subscribe=self.sub_auth, resolve=None
            ),
        }

    @GraphQLSession.authenticated(RBAC="FIXME")
    def get_auth(self, root, info, id):
        """AuthenticationRequest query handler"""
        return auth_data.get(id)

    def set_auth(self, root, info, **kwargs):
        """AuthenticationRequest mutation handler"""
        usr = auth_data.get(kwargs["id"])
        updated_dict = {**usr, **kwargs}
        self.subscriptions.emit(__name__, {"AuthenticationRequest": updated_dict})
        auth_data[kwargs["id"]] = updated_dict
        return updated_dict

    def sub_auth(self, root, info, **kwargs):
        """AuthenticationRequest subscription handler"""
        return SimplePubSubIterator(self.subscriptions, __name__)
