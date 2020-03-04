import asyncio
import logging
import time
import os, pty
import termios
import fcntl

import tornado.ioloop

application_log = logging.getLogger("tornado.application")


class ProcessRunner(object):
    def __init__(
        self,
        cmd,
        width=50,
        height=50,
        loop=tornado.ioloop.IOLoop.current(),
        started_callback=None,
        output_callback=None,
        complete_callback=None,
        read_timeout=2.0,
        post_timeout=0,
    ):
        self.cmd = cmd
        self.width = width
        self.height = height
        self.loop = loop
        self.started_callback = started_callback
        self.output_callback = output_callback
        self.complete_callback = complete_callback
        self.read_timeout = read_timeout
        self.post_timeout = post_timeout
        self.process = None
        self.complete = False
        self.returncode = None
        self.running = False
        self.start_time = None
        self.stdout = ""
        self.stderror = ""
        self.stdout_log = []
        self.stderror_log = []

    def to_dict(self):
        return {
            "command": self.cmd,
            "complete": self.complete,
            "returncode": self.returncode,
            "running": self.running,
            "uptime": self.uptime,
            "stdout": self.stdout,
            "stderror": self.stderror,
        }

    def start(self):
        if self.process:
            # already running...
            return False
        else:
            self.loop.add_callback(self.run)
            return True

    def terminate(self):
        if self.process:
            application_log.info("Attempting to terminate shell process")
            self.process.terminate()
            self.process.kill()
            self.complete = True
            return True
        else:
            return False

    @property
    def uptime(self):
        if self.start_time:
            return time.time() - self.start_time
        else:
            return None

    def fileCallback(self, fd, event, **kwargs):
        o = os.read(fd, 10240)
        self.append_to_output_log("stdout", o)

    async def run(self):
        # https://github.com/QubesOS/qubes-core-admin/blob/master/qubes/backup.py
        """Internal method, do not call directly. Use start()"""
        my_env = os.environ.copy()
        my_env["TERM"] = "xterm-256color"
        my_env[
            "LS_COLORS"
        ] = "rs=0:di=01;34:ln=01;36:mh=00:pi=40;33:so=01;35:do=01;35:bd=40;33;01:cd=40;33;01:or=40;31;01:mi=00:su=37;41:sg=30;43:ca=30;41:tw=30;42:ow=34;42:st=37;44:ex=01;32:*.tar=01;31:*.tgz=01;31:*.arc=01;31:*.arj=01;31:*.taz=01;31:*.lha=01;31:*.lz4=01;31:*.lzh=01;31:*.lzma=01;31:*.tlz=01;31:*.txz=01;31:*.tzo=01;31:*.t7z=01;31:*.zip=01;31:*.z=01;31:*.Z=01;31:*.dz=01;31:*.gz=01;31:*.lrz=01;31:*.lz=01;31:*.lzo=01;31:*.xz=01;31:*.zst=01;31:*.tzst=01;31:*.bz2=01;31:*.bz=01;31:*.tbz=01;31:*.tbz2=01;31:*.tz=01;31:*.deb=01;31:*.rpm=01;31:*.jar=01;31:*.war=01;31:*.ear=01;31:*.sar=01;31:*.rar=01;31:*.alz=01;31:*.ace=01;31:*.zoo=01;31:*.cpio=01;31:*.7z=01;31:*.rz=01;31:*.cab=01;31:*.wim=01;31:*.swm=01;31:*.dwm=01;31:*.esd=01;31:*.jpg=01;35:*.jpeg=01;35:*.mjpg=01;35:*.mjpeg=01;35:*.gif=01;35:*.bmp=01;35:*.pbm=01;35:*.pgm=01;35:*.ppm=01;35:*.tga=01;35:*.xbm=01;35:*.xpm=01;35:*.tif=01;35:*.tiff=01;35:*.png=01;35:*.svg=01;35:*.svgz=01;35:*.mng=01;35:*.pcx=01;35:*.mov=01;35:*.mpg=01;35:*.mpeg=01;35:*.m2v=01;35:*.mkv=01;35:*.webm=01;35:*.ogm=01;35:*.mp4=01;35:*.m4v=01;35:*.mp4v=01;35:*.vob=01;35:*.qt=01;35:*.nuv=01;35:*.wmv=01;35:*.asf=01;35:*.rm=01;35:*.rmvb=01;35:*.flc=01;35:*.avi=01;35:*.fli=01;35:*.flv=01;35:*.gl=01;35:*.dl=01;35:*.xcf=01;35:*.xwd=01;35:*.yuv=01;35:*.cgm=01;35:*.emf=01;35:*.ogv=01;35:*.ogx=01;35:*.aac=00;36:*.au=00;36:*.flac=00;36:*.m4a=00;36:*.mid=00;36:*.midi=00;36:*.mka=00;36:*.mp3=00;36:*.mpc=00;36:*.ogg=00;36:*.ra=00;36:*.wav=00;36:*.oga=00;36:*.opus=00;36:*.spx=00;36:*.xspf=00;36:"
        my_env["COLORTERM"] = "truecolor"
        # my_env["COLUMNS"] = str(int(self.width))
        # my_env["LINES"] = str(int(self.height))

        def set_ctty(ctty_fd, master_fd):
            os.setsid()
            os.close(master_fd)
            fcntl.ioctl(ctty_fd, termios.TIOCSCTTY, 0)

            echo = False

            if not echo:
                termios_p = termios.tcgetattr(ctty_fd)
                # termios_p.c_lflags
                termios_p[3] &= ~termios.ECHO
                termios.tcsetattr(ctty_fd, termios.TCSANOW, termios_p)

        self.start_time = time.time()
        self.running = True

        (self.pty_master, pty_slave) = pty.openpty()

        # application_log.debug(my_env)
        self.process = await asyncio.create_subprocess_shell(  #  create_subprocess_exec(
            self.cmd,
            env=my_env,
            bufsize=0,
            stdin=pty_slave,
            stdout=pty_slave,
            stderr=pty_slave,
            preexec_fn=lambda: set_ctty(pty_slave, self.pty_master),
        )
        if self.started_callback:
            # let them know we are running...
            self.started_callback(**self.to_dict())
        done = None
        pending = None
        self.loop.add_handler(self.pty_master, self.fileCallback, self.loop.READ)
        while self.process.returncode is None:
            await asyncio.sleep(0.2)
        self.loop.remove_handler(self.pty_master)
        os.close(pty_slave)
        os.close(self.pty_master)

        self.running = False
        self.complete = True
        if pending:
            for task in pending:
                task.cancel()
        if self.post_timeout:
            await asyncio.sleep(self.post_timeout)
        self.returncode = self.process.returncode
        application_log.debug(f"[{self.cmd!r} exited with {self.returncode}]")
        if self.complete_callback:
            # let the caller that the process is complete...
            self.complete_callback(**self.to_dict())
        self.process = None
        return True

    def append_to_output_log(self, name, output):
        should_callback = False
        try:
            output = output.decode("ascii")
        except:
            pass
        application_log.debug(f"{name}:{output}")
        if name == "stdout":  # and output:
            self.stdout = output
            self.stderror = ""
            self.stdout_log.append((self.uptime, output))
            should_callback = True
        elif name == "stderror":  # and output:
            self.stderror = output
            self.stdout = ""
            self.stderror_log.append((self.uptime, output))
            should_callback = True

        if self.output_callback and should_callback:
            # let them know we have output running...
            # print(self.to_dict())
            self.output_callback(**self.to_dict())

    async def read_from(self, name, source):
        stddata = await source.readline()
        line = stddata.decode("ascii").replace("\t", "")
        # FIXME: this does not quite strip the colour codes from all text
        #   for the moment it does a pretty good job...
        # line = re.sub(r"\x1B\[[0-?]*[ -/]*[@-~]", "", line, flags=re.IGNORECASE)
        return (name, line)
