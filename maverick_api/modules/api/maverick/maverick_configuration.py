# expose the api configuration file for the -api server
# expose ~/config/maverick/localconf.json

import jstyleson as jjson
from jsondiff import diff as jdiff

from modules.base.util.functions import mkdirs

import logging
import asyncio
import copy
import threading
import time
import re

from modules.api import moduleBase
from modules.api import schemaBase
from modules.api import api_callback

import tornado.ioloop
from tornado.options import options

# graphql imports
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
    GraphQLFloat,
)
from graphql.pyutils.event_emitter import EventEmitter, EventEmitterAsyncIterator

application_log = logging.getLogger("tornado.application")


class MaverickConfigurationSchema(schemaBase):
    def __init__(self):
        super().__init__()
        self.config = {}
        config = ""
        with open("/srv/maverick/config/maverick/localconf.json", "r+") as fid:
            config = fid.read()

        self.config = jjson.loads(config)
        self.make_backup_dir()
        # self.modify_config({"testing":123})

    def modify_config(self, data):
        """apply a change to the config"""
        new_config = {**self.config, **data}
        print(jdiff(self.config, new_config, syntax="explicit"))

    def make_backup_dir(self):
        """make a .backup folder if it does not exist"""
        mkdirs("/srv/maverick/config/maverick/.backup")

    def wirte_backup(self):
        """write a backup of the configfile with a timestamp"""
