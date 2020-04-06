from functools import wraps
from uuid import uuid4
import hashlib
import binascii
import os
import logging
import datetime
import jwt
import json
import bcrypt
from pathlib import Path

from graphql.error import GraphQLError
from tornado.options import options
import tornado.web

application_log = logging.getLogger("tornado.application")

# FIXME: remove
user_records = {
    "bob": {"id": 1234, "authenticated": True, "RBAC": "*", "token": "callsignviper"},
    "jim": {"id": 5678, "authenticated": False, "RBAC": None, "token": None},
}

user1 = dict(id="1", userName="bob", password="password1")
user2 = dict(id="2", userName="ben", password="password2")
auth_data = {user1["id"]: user1, user2["id"]: user2}

private_key = ""
public_key = ""

with open(
    Path(options.basedir).joinpath("data", "keys", "private_key.pem"), "r+"
) as fid:
    private_key = fid.read()

with open(
    Path(options.basedir).joinpath("data", "keys", "public_key.pem"), "r+"
) as fid:
    public_key = fid.read()


def create_refresh_jwt(ttl, code):
    refresh_jwt = jwt.encode(
        {
            "iss": "maverick-api",
            "aud": "maverick-web",
            "iat": datetime.utcnow(),
            "nbf": datetime.utcnow(),
            "exp": datetime.datetime.utcnow() + datetime.timedelta(minutes=ttl),
            "refresh": code,
        },
        private_key,
        algorithm="RS256",
    )
    return refresh_jwt


def refresh_database_lookup(refresh_code):
    # TODO FIXME do database lookup
    # refresh code could be None.
    return refresh_code


def invalidate_user_refresh_tokens(user):
    # lookup the user
    # remove refresh tokens from the user database
    pass


def get_latest_refresh_token(current_refresh_token):
    new_refresh_token = None
    # lookup the value in the database for the current token and see if there is a newer one
    # if there is, return a refresh token with the new value
    # if database lookup for current value returns new value, create a new jwt with the value
    current_refresh_code = decode_jwt(current_refresh_token).get("refresh", None)
    new_refresh_code = refresh_database_lookup(current_refresh_code)
    if current_refresh_code != new_refresh_code:
        new_refresh_token = create_refresh_jwt(code=new_refresh_code)
    return new_refresh_token


def create_access_jwt(ttl, code):
    access_jwt = jwt.encode(
        {
            "iss": "maverick-api",
            "aud": "maverick-web",
            "iat": datetime.utcnow(),
            "nbf": datetime.utcnow(),
            "exp": datetime.datetime.utcnow() + datetime.timedelta(minutes=ttl),
            "access": code,
        },
        private_key,
        algorithm="RS256",
    )
    return access_jwt


def decode_jwt(jwt_payload):
    decoded = jwt.decode(
        jwt_payload,
        public_key,
        issuer="maverick-api",
        audience="maverick-web",
        leeway=10,
        algorithms="RS256",
    )
    return decoded


def verify_jwt(jwt_payload):
    error = True
    try:
        decode_jwt(jwt_payload)
        error = False

    except jwt.ExpiredSignatureError:
        # Signature has expired
        pass
    except jwt.InvalidIssuerError:
        # issuer does not match
        pass
    except jwt.InvalidAudienceError:
        # Audience is incorrect
        pass
    except jwt.InvalidIssuedAtError:
        # iat was not correct
        pass
    return error


class LoginHandler(tornado.web.RequestHandler):
    """login user with provided credentials"""

    def set_default_headers(self):
        self.set_header("Access-Control-Allow-Origin", "*")
        self.set_header("Access-Control-Allow-Credentials", "true")
        self.set_header(
            "Access-Control-Allow-Headers",
            "Origin, X-Requested-With, Content-Type, Accept, Authorization",
        )
        self.set_header("Access-Control-Allow-Methods", "POST")

    async def post(self):
        # check if the user is already logged in via the presence of an access token
        #  if one exists, don't check it, just return
        auth = self.request.headers.get("Authorization", "")
        if auth:
            self.finish()

        # get the username and password sent via the request and
        #  check this against the db records. If the user exists:
        #  get the current refresh token for this user, set the cookie
        #  and provide an access token
        #  otherwise don't allow login
        username = self.get_argument("user")
        password = self.get_argument("password")
        # TODO return user from DB
        user = None  # self.application.syncdb["users"].find_one({"user": email})

        if (
            user
            and user["password"]
            and bcrypt.hashpw(password, user["password"]) == user["password"]
        ):
            # passwords match for this user
            # TODO: get the current refresh token for this user and set it as a secure cookie
            pass

        self.finish()


class LogoutHandler(tornado.web.RequestHandler):
    """logout current user"""

    def set_default_headers(self):
        self.set_header("Access-Control-Allow-Origin", "*")
        self.set_header("Access-Control-Allow-Credentials", "true")
        self.set_header(
            "Access-Control-Allow-Headers",
            "Origin, X-Requested-With, Content-Type, Accept, Authorization",
        )
        self.set_header("Access-Control-Allow-Methods", "POST")

    async def post(self):
        # check if the user is already logged in via the presence of an access token
        auth = self.request.headers.get("Authorization", "")
        # if the access token is valid...
        if auth and verify_jwt(auth):

            # check to see if they want to log out all sessions
            all_sessions = self.get_argument("allSessions", False)
            if all_sessions:
                user = None
                # TODO: get user from access token
                # TODO: invalidate all refresh tokens stored for this user.
                invalidate_user_refresh_tokens(user)

            # log the current user out...

            # set the secure cookie to a null string so further refresh_token
            #  attempts will fail, requiring a login
            # note: the webapp needs to clear the access token in memory
            # set a logout flag to the current time
            # and propergate a logout request via local storage (logging out all open tabs)
            self.set_secure_cookie(
                "refresh_token",
                "",
                secure=True,
                httponly=True,
                expires_days=1,
                path="/refresh_token",
            )  # we can also set domain="maverick.one"
        self.finish()


class RefreshTokenHandler(tornado.web.RequestHandler):
    """given a refresh token, provide an access token"""

    def set_default_headers(self):
        self.set_header("Access-Control-Allow-Origin", "*")
        self.set_header("Access-Control-Allow-Credentials", "true")
        self.set_header(
            "Access-Control-Allow-Headers",
            "Origin, X-Requested-With, Content-Type, Accept, Authorization",
        )
        self.set_header("Access-Control-Allow-Methods", "GET")

    async def get(self):
        send_access_token = False
        set_refresh_cookie = False
        latest_refresh_token = None
        json_output = None
        current_refresh_token = self.get_secure_cookie(name="refresh_token")
        # if we don't have a refresh token, ask the user to login
        if not current_refresh_token:
            # infom the user they need to login
            json_output = {
                "accessToken": None,
                "error": "No refresh token, login required",
            }
        else:
            if verify_jwt(current_refresh_token):
                # the refresh token is valid
                # send an access token
                send_access_token = True
                # check to see if there is a newer refresh token that needs to be set
                latest_refresh_token = get_latest_refresh_token(current_refresh_token)
                if latest_refresh_token:
                    set_refresh_cookie = True

                if set_refresh_cookie:
                    self.set_secure_cookie(
                        "refresh_token",
                        latest_refresh_token,
                        secure=True,
                        httponly=True,
                        expires_days=365,
                        path="/refresh_token",
                    )  # we can also set domain="maverick.one"

                if send_access_token:
                    # TODO FIXME
                    # here we would encode the access rights of the user, not a code
                    # note all data is simply base64 encoded so we cannot encode secrets
                    access_token = create_access_jwt(code="callsignviper")
                    json_output = {"accessToken": access_token, "error": None}

            else:
                # the refresh token was out of date or otherwise invalid
                # ask the user to login
                json_output = {
                    "accessToken": None,
                    "error": "Refresh token error, login required",
                }

        self.write(json.dumps(json_output))
        self.finish()


class GraphQLSession(object):
    def __init__(self, session_id):
        self.session_id = session_id
        self.user_authenticated = False
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
        document = await db.session_collection.find_one(
            {"session_id": self.session_id.decode("utf-8")}
        )
        return document["user_authenticated"]

    @staticmethod
    def verify_RBAC(self, *, API_RBAC=None, user_RBAC=None):
        """Check to see if current user meets role based
            access control requirements"""
        application_log.debug(f"supplied RBAC: {user_RBAC}")
        if API_RBAC is None:
            # no RBAC check required
            return True
        else:
            if user_RBAC is None:
                # no access
                return False
            elif user_RBAC == "*":
                # full access
                return True
            else:
                # TODO: perform check against user RBAC
                # FIXME: return True for now
                return True

    @staticmethod
    def hash_password(password):
        """Hash a password for storing."""
        salt = hashlib.sha256(os.urandom(60)).hexdigest().encode("ascii")
        pwdhash = hashlib.pbkdf2_hmac("sha512", password.encode("utf-8"), salt, 100_000)
        pwdhash = binascii.hexlify(pwdhash)
        return (salt + pwdhash).decode("ascii")

    @staticmethod
    def verify_password(stored_password, provided_password):
        """Verify a stored password against one provided by user"""
        salt = stored_password[:64]
        stored_password = stored_password[64:]
        pwdhash = hashlib.pbkdf2_hmac(
            "sha512", provided_password.encode("utf-8"), salt.encode("ascii"), 100_000
        )
        pwdhash = binascii.hexlify(pwdhash).decode("ascii")
        return pwdhash == stored_password

    @staticmethod
    def authenticated(original_function=None, *, RBAC=None):
        def _decorate(func):
            @wraps(func)
            async def wrapper_authenticated(self, *args, **kwargs):
                user_record = None
                user_is_authenticated = False
                user_RBAC = None
                (root, info) = args
                if (info is None) and (root is None):
                    # the call is being made directly from the api
                    # authentication is not required
                    return await func(self, *args, **kwargs)

                # self.session = info.context.get("session")
                # client = info.context.get("db_client")
                # is_authenticated = await self.session.is_authenticated(client)

                token = info.context.get("authorization", None)
                if token:
                    # TODO search database via token
                    # TODO: perform database lookup against token to obtain user auth & RBAC levels
                    for user in user_records:
                        if user_records[user]["token"] == token:
                            user_record = user_records[user]
                            user_is_authenticated = user_record["authenticated"]
                            user_RBAC = user_record["RBAC"]
                            break

                if not user_is_authenticated:
                    logging.warn("Un-authenticated API access attempt")
                    return AuthError(
                        message="Current session is not authenticated", status_code=401
                    )
                logging.info(f"{user_record[id]} is authenticated")
                if RBAC:
                    # a RBAC check is required
                    RBAC_result = GraphQLSession.verify_RBAC(
                        API_RBAC=RBAC, user_RBAC=user_RBAC
                    )
                    if not RBAC_result:
                        logging.warn("Un-authorized API usage attempt")
                        return AuthError(
                            message="Current user is not authorized", status_code=403
                        )
                    else:
                        # The RBAC requirements have been met by the user
                        logging.info(f"RBAC requirements met by {user_record[id]}")
                        return await func(self, *args, **kwargs)
                else:
                    # There are no RBAC requirements to check
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
                db = self.opts["db_client"].session_database
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
