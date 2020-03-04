from pathlib import Path, PurePath

basedir = Path.cwd()

from maverick_api.modules.base.setup.config import MavConfig

MavConfig(PurePath(basedir).joinpath("maverick_api", "config", "maverick-api.conf"))

from maverick_api.modules.api.maverick import MaverickConfigureSchema  # noqa: F401
