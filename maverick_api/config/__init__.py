import json, os


class Configuration(object):
    def __init__(self, config_name):
        self.read_config(config_name)

    def read_config(self, config_name):
        try:
            with open(os.path.join(os.path.dirname(__file__), config_name), "r") as fid:
                self.config = json.loads(fid.read())
        except Exception as e:
            # TODO: catch the correct error here...
            # and raise an exception + log
            self.config = None
            print("ERROR: {0}".format(e))
            raise SystemExit()

    def get_config(self):
        return self.config
