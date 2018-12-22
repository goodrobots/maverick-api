import json, os

class Configuration(object):
    def __init__(self, config_name, config_file):
        self.config_file = config_file
        self.config_name = config_name
        self.read_config()
        
            
    def read_config(self):
        try:
            with open(self.config_file, 'r') as fid:
                self.config = json.loads(fid.read())
        except Exception as e:
            # TODO: catch the correct error here...
            # and raise an exception + log
            self.config = None
            print("ERROR: {0}".format(e))
            raise SystemExit()
    
    def get_config(self):
        return self.config