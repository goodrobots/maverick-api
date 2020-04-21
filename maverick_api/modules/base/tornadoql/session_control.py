from functools import wraps
from uuid import uuid4
import hashlib
import binascii
import os
import base64
import logging
import datetime
import jwt
import json
import bcrypt

# from pathlib import Path

from graphql.error import GraphQLError

# from tornado.options import options
import tornado.web

application_log = logging.getLogger("tornado.application")

secure = True  # TODO: replace with option
cookies_allowed = True  # TODO: allow users to disable cookies for the login


# FIXME: replace with user database
user_records = {
    b"bob": {
        "name": "bob",
        "id": 1234,
        "authenticated": False,
        "roles": ["user", "admin"],
        "token": "callsignviper",
        "password": bcrypt.hashpw(b"pas$w0rd", bcrypt.gensalt()),
        "refresh_token_invalid": {},
        "refresh_token_current": {},
        "refresh_token_unused": {},
        "access_token_invalid": {},
        "access_token_current": {},
        "access_token_unused": {},
    }
}

private_key = ""
public_key = ""
refresh_token_life = datetime.timedelta(days=365)
access_token_life = datetime.timedelta(minutes=15)

# with open(
#     Path(options.basedir).joinpath("data", "keys", "private_key.pem"), "r+"
# ) as fid:
#     private_key = fid.read()

# with open(
#     Path(options.basedir).joinpath("data", "keys", "public_key.pem"), "r+"
# ) as fid:
#     public_key = fid.read()


def create_token_code():
    return str(base64.urlsafe_b64encode(os.urandom(30)))


def create_token(user, access, ttl=None):
    code = create_token_code()
    user_data = {}
    jwt_store = ""
    if access:
        # TODO replace with database lookup

        user_data = {"name": user.get("name", ""), "roles": user.get("roles", [])}
        jwt_store = "access_token_unused"
        if not ttl:
            ttl = access_token_life
    else:
        user_data = {"name": user.get("name", "")}
        jwt_store = "refresh_token_unused"
        if not ttl:
            ttl = refresh_token_life

    encoded_jwt = jwt.encode(
        {
            "iss": "maverick-api",
            "aud": "maverick-web",
            "iat": datetime.datetime.utcnow(),
            "nbf": datetime.datetime.utcnow(),
            "exp": datetime.datetime.utcnow() + ttl,
            "code": code,
            "user": user_data,
        },
        private_key,
        algorithm="RS256",
    )
    user[jwt_store][code] = {
        "jwt": encoded_jwt,
        "create_time": datetime.datetime.utcnow(),
    }
    return encoded_jwt


def get_current_refresh_token(user):
    # check to see if there is an unused refresh token
    refresh_token_current = user.get("refresh_token_current")
    if not refresh_token_current:
        refresh_token_current = create_token(user, access=False)
    return refresh_token_current


def get_current_access_token(user):
    # check to see if there is an unused access token
    access_token_current = user.get("access_token_current")
    if not access_token_current:
        access_token_current = create_token(user, access=True)
    return access_token_current


def invalidate_user_tokens(user_name, token_codes=None):
    """allow forced logout of all devices"""
    # TODO: refactor
    global user_records
    if not token_codes:
        token_codes = []
    user = user_records[user_name]
    refresh_token_groups = ["refresh_token_unused", "refresh_token_current"]
    access_token_groups = ["access_token_unused", "access_token_current"]
    for refresh_token_group in refresh_token_groups:
        tokens = []
        if not token_codes:
            tokens = [t for t in user[refresh_token_group]]
        else:
            tokens = [t for t in user[refresh_token_group] if t in tokens]
        for token in tokens:
            user["refresh_token_invalid"][token] = {
                **user[refresh_token_group][token],
                **{
                    "invalid_meta": "forced logout",
                    "invalid_time": datetime.datetime.utcnow(),
                },
            }
        user[refresh_token_group] = {}

    for access_token_group in access_token_groups:
        tokens = []
        if not token_codes:
            tokens = [t for t in user[access_token_group]]
        else:
            tokens = [t for t in user[access_token_group] if t in tokens]
        for token in tokens:
            user["access_token_invalid"][token] = {
                **user[access_token_group][token],
                **{
                    "invalid_meta": "forced logout",
                    "invalid_time": datetime.datetime.utcnow(),
                },
            }
        user[access_token_group] = {}


# def get_latest_refresh_token(current_refresh_token):
#     new_refresh_token = None
#     # lookup the value in the database for the current token and see if there is a newer one
#     # if there is, return a refresh token with the new value
#     # if database lookup for current value returns new value, create a new jwt with the value
#     current_refresh_code = decode_jwt(current_refresh_token).get("refresh", None)
#     new_refresh_code = refresh_database_lookup(current_refresh_code)
#     if current_refresh_code != new_refresh_code:
#         new_refresh_token = create_refresh_jwt(
#             datetime.timedelta(minutes=20), code=new_refresh_code
#         )
#     return new_refresh_token


def decode_jwt(jwt_payload):
    decoded = {}
    try:
        decoded = jwt.decode(
            jwt_payload,
            public_key,
            issuer="maverick-api",
            audience="maverick-web",
            leeway=datetime.timedelta(seconds=10),
            algorithms="RS256",
        )
        print(decoded)
    except jwt.ExpiredSignatureError:
        application_log.info("jwt signature has expired")
    except jwt.InvalidIssuerError:
        application_log.info("jwt issuer does not match")
    except jwt.InvalidAudienceError:
        application_log.info("jwt audience is incorrect")
    except jwt.InvalidIssuedAtError:
        application_log.info("jwt iat was not correct")
    return decoded


def decode_token(token):
    decoded_token = decode_jwt(token)
    return (decoded_token["code"], decoded_token["user"])


def verify_access_token(auth):
    # TODO: needs cleanup
    token = ""
    if auth.startswith("Bearer "):
        token = auth[7:]
    if not token:
        return False
    try:
        code, user = decode_token(token)
    except KeyError:
        return False
    if not code or not user:
        return False
    return (code, user)


class LoginHandler(tornado.web.RequestHandler):
    """login user with provided credentials, return a refresh token on success"""

    def set_default_headers(self):
        self.set_header("Access-Control-Allow-Origin", "*")
        self.set_header("Access-Control-Allow-Credentials", "true")
        self.set_header(
            "Access-Control-Allow-Headers",
            "Origin, X-Requested-With, Content-Type, Accept, Authorization",
        )
        self.set_header("Access-Control-Allow-Methods", "POST, OPTIONS")

    def options(self):
        self.set_status(204)

    async def post(self):
        # check if the user is already logged in via the presence of an access token
        #  if one exists, don't check it, just return without data
        auth = self.request.headers.get("Authorization", "")
        if auth:
            self.set_status(204)
            self.finish()

        # get the username and password sent via the request and
        #  check this against the db records. If the user exists:
        #  get the current refresh token for this user, set the cookie
        #  and provide an access token
        #  otherwise don't allow login
        username = self.get_argument("user").encode("utf-8")
        password = self.get_argument("password").encode("utf-8")
        # TODO return user from DB
        # username = b"bob"
        # password = b"pas$w0rd"
        user = user_records.get(username, {})
        if user and user["password"] and bcrypt.checkpw(password, user["password"]):
            # passwords match for this user
            refresh_token = get_current_refresh_token(user)
            self.set_cookie(
                "refresh_token",
                refresh_token,
                secure=secure,
                httponly=True,
                expires_days=365,
                path="/refresh_token",  # this is the only path the token will be sent to
            )  # we can also set domain="maverick.one"
            self.set_status(202)  # Accepted
        else:
            self.set_status(401)  # Bad request
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
        self.set_header("Access-Control-Allow-Methods", "POST, OPTIONS")

    def options(self):
        self.set_status(204)

    async def post(self):
        # check if the user is already logged in via the presence of an access token
        auth = self.request.headers.get("Authorization", "")
        # if the access token is valid...
        if auth:
            try:
                ret = verify_access_token(auth)
            except:
                ret = None
            if not ret:
                self.finish()
            (code, user) = ret
            user_name = user.get("name")

            # check to see if they want to log out all sessions
            if self.get_argument("allSessions", False):
                # TODO: get user from access token
                # TODO: invalidate all refresh tokens stored for this user.
                invalidate_user_tokens(user_name)

            # log the current user out...

            # set the secure cookie to a null string so further refresh_token
            #  attempts will fail, requiring a login
            # note: the webapp needs to clear the access token in memory
            # set a logout flag to the current time
            # and propergate a logout request via local storage (logging out all open tabs)
            self.set_cookie(
                "refresh_token",
                "",
                secure=secure,
                httponly=True,
                expires_days=365,
                path="/refresh_token",
            )  # we can also set domain="maverick.one"

            # invalidate this refresh token, a new one will be generated on next login
            invalidate_user_tokens(user_name, token_codes=[code])
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
        self.set_header("Access-Control-Allow-Methods", "GET, OPTIONS")

    def options(self):
        self.set_status(204)

    async def get(self):
        send_access_token = False
        set_refresh_cookie = False
        latest_refresh_token = None
        json_output = None
        current_refresh_token = self.get_cookie(name="refresh_token")
        # print("!!")
        # token = self.get_cookie("refresh_token")
        # print(token)
        # print(decode_token(token))
        # print("!!")

        # if we don't have a refresh token, ask the user to login
        if not current_refresh_token:
            # inform the user they need to login
            json_output = {
                "accessToken": None,
                "error": "No refresh token, login required",
            }
        else:
            try:
                ret = decode_token(current_refresh_token)
            except:
                ret = None
            if ret:
                (code, user) = ret

                # new plan... every time we make a new access token via the refresh, we also create
                #  a new refresh token
                # the backend should invalidate the previous tokens only when the frontend uses the new tokens — confirming its successful receipt.
                # the new refresh token is stored as new and any further requests to this endpoint will continue to set it
                # until the new refresh token is used we will continue to use the old refresh token.
                # once the new refresh token is seen we will invalidate the old one.

                # the refresh token is valid
                # send an access token
                send_access_token = True
                # check to see if there is a newer refresh token that needs to be set

                if latest_refresh_token:
                    set_refresh_cookie = True

                if set_refresh_cookie:
                    self.set_cookie(
                        "refresh_token",
                        latest_refresh_token,
                        secure=secure,
                        httponly=True,
                        expires_days=365,
                        path="/refresh_token",
                    )  # we can also set domain="maverick.one"

                if send_access_token:
                    # encode the access rights of the user
                    # note all data is simply base64 encoded so we cannot encode secrets
                    access_token = create_token(user, access=True)
                    json_output = {
                        "accessToken": access_token,
                        "error": 0,
                        "errorString": "",
                    }

            else:
                # the refresh token was out of date or otherwise invalid
                # ask the user to login
                json_output = {
                    "accessToken": None,
                    "error": 1,
                    "errorString": "Refresh token error, login required",
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
