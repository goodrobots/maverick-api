# expose the api configuration file for the -api server
# expose ~/config/maverick/localconf.json

from jsoncomment import JsonComment
from jsondiff import diff as jdiff

from maverick_api.modules.base.util.functions import mkdirs

import logging

from maverick_api.modules import schemaBase

application_log = logging.getLogger("tornado.application")


class MaverickConfigurationSchema(schemaBase):
    def __init__(self):
        super().__init__(self)
        pass

    def load_config_from_file(
        self, filepath="/srv/maverick/config/maverick/localconf.json"
    ):
        self.config = {}
        config = ""
        with open(filepath, "r+") as fid:
            config = fid.read()
        jjson = JsonComment()
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
