from functools import wraps
from uuid import uuid4
from graphql.error import GraphQLError
import hashlib
import binascii
import os
import motor  # async access to mongo database
import logging

# FIXME: remove
import random


class GraphQLSession(object):
    def __init__(self, session_id):
        self.session_id = session_id
        self.user_authenticated = True
        self.RBAC = {}  # TODO: define RBAC 'levels'

    def create_document(self):
        return {
            "session_id": self.session_id,
            "user_authenticated": self.user_authenticated,
            "RBAC": self.RBAC,
        }

    def set_authenticated(self, val):
        self.authenticated = val

    async def is_authenticated(self, client):
        """Check to see if this session is authenticated"""
        # TODO: Expand and include LDAP
        # TODO: Make database layer general to mongo, sqlite, etc...
        db = client.session_database
        print(self.session_id.decode("utf-8"))
        document = await db.session_collection.find_one(
            {"session_id": self.session_id.decode("utf-8")}
        )
        return document["user_authenticated"]

    def verify_RBAC(self, *, RBAC=None):
        """Check to see if current user meets role based
            access control requirements"""
        print("supplied RBAC", RBAC)
        if RBAC is None:
            return True
        else:
            # TODO: perform check against user RBAC
            # and return True / False
            return False

    @staticmethod
    def hash_password(password):
        """Hash a password for storing."""
        salt = hashlib.sha256(os.urandom(60)).hexdigest().encode("ascii")
        pwdhash = hashlib.pbkdf2_hmac("sha512", password.encode("utf-8"), salt, 100000)
        pwdhash = binascii.hexlify(pwdhash)
        return (salt + pwdhash).decode("ascii")

    @staticmethod
    def verify_password(stored_password, provided_password):
        """Verify a stored password against one provided by user"""
        salt = stored_password[:64]
        stored_password = stored_password[64:]
        pwdhash = hashlib.pbkdf2_hmac(
            "sha512", provided_password.encode("utf-8"), salt.encode("ascii"), 100000
        )
        pwdhash = binascii.hexlify(pwdhash).decode("ascii")
        return pwdhash == stored_password

    @staticmethod
    def authenticated(original_function=None, *, RBAC=None):
        def _decorate(func):
            @wraps(func)
            async def wrapper_authenticated(self, *args, **kwargs):
                (root, info) = args
                if (info is None) and (root is None):
                    # the call is being made directly from the api
                    # authentication is not required
                    return await func(self, *args, **kwargs)
                # self.session = info.context.get("session")
                # client = info.context.get("db_client")
                # is_authenticated = await self.session.is_authenticated(client)
                
                # FIXME: a quick hack to return an error 50% of the time
                is_authenticated = random.choice([True, False])
                if not is_authenticated:
                    logging.warn("Un-authenticated access attempt")
                    # FIXME: need a default behavior for non-auth'd session
                    return AuthError(message="Current session is not authenticated", status_code = 401)
                    # return GraphQLError(message="Current session is not authenticated")
                if RBAC is not None:
                    # FIXME: a quick hack to return an error
                    return AuthError(message="Current user is not authorized", status_code = 403)
                    # RBAC requirements have been passed to the function
                    if self.session.verify_RBAC(RBAC=RBAC):
                        # The RBAC requirements have been met
                        return await func(self, *args, **kwargs)
                    else:
                        return AuthError(message="Current user is not authorized", status_code = 403)
                else:
                    return await func(self, *args, **kwargs)

            return wrapper_authenticated

        if original_function:
            return _decorate(original_function)
        return _decorate

    @staticmethod
    def ensure_active_session(func):
        """Either get an active session or create a new one"""

        @wraps(func)
        async def wrapper_active_session(self, *args, **kwargs):
            # check to see if the session is still valid
            session_id = self.get_secure_cookie("session")
            if not session_id:
                # this session is not yet setup
                # create a new session
                session_id = uuid4().__str__()
                # use current_user alias as it is baked into tornado
                self.current_user = GraphQLSession(session_id)
                # connect to the session db
                # db = self.opts["db_client"].session_database
                try:
                    self.set_secure_cookie("session", session_id)
                    print("setting secure cookie")
                    result = await db.session_collection.insert_one(
                        self.current_user.create_document()
                    )
                    print("Create session document:", result)
                    print("result %s" % repr(result.acknowledged))
                    print(dir(result))
                except RuntimeError as e:
                    # cannot set secure cookie from a websocket
                    print(e)
                    return
                except Exception as e:
                    return
            else:
                # To save db access we assume the session document exists
                # FIXME: lookup and load session if required
                self.current_user = GraphQLSession(session_id)
            return await func(self, *args, **kwargs)

        return wrapper_active_session

class AuthError(GraphQLError):
    """Authentication / Authorization error"""
    def __init__(self, message, status_code):
        super().__init__(message)
        self.status_code = status_code
        
    def __str__(self):
        return self.message
        