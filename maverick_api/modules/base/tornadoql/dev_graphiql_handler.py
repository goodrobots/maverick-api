import os
import tornado.web
from tornado.options import options


class GraphiQLHandler(tornado.web.RequestHandler):
    def get(self):
        if options.development:
            self.render(
                os.path.join(
                    self.application.settings.get("static_path"), "graphiql.html"
                )
            )
        else:
            self.set_status(403)
        self.finish()
