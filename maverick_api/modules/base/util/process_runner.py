import asyncio
import logging
import time
import os, pty
import termios
import fcntl
import re
import functools

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
        echo=False,
        strip_output=False,
        shell=True,
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
        # FIXME: this does not quite strip the colour & control codes from all text
        #   for the moment it does a pretty good job...
        self.re_ctrl_format_matcher = re.compile(
            r"\x1B\[[0-?]*[ -/]*[@-~]", flags=re.IGNORECASE
        )
        self.echo = echo
        self.strip_output = strip_output
        self.shell = shell

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
            self.loop.add_callback(functools.partial(self.run, self.shell))
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
        self.append_to_output_log("stdout", os.read(fd, 10240))

    async def run(self, shell=True):
        # https://github.com/QubesOS/qubes-core-admin/blob/master/qubes/backup.py
        my_env = os.environ.copy()
        my_env["TERM"] = "xterm-256color"
        my_env["COLORTERM"] = "truecolor"
        # my_env["COLUMNS"] = str(int(self.width))
        # my_env["LINES"] = str(int(self.height))

        def set_ctty(ctty_fd, master_fd):
            os.setsid()
            os.close(master_fd)
            fcntl.ioctl(ctty_fd, termios.TIOCSCTTY, 0)

            if not self.echo:
                termios_p = termios.tcgetattr(ctty_fd)
                termios_p[3] &= ~termios.ECHO
                termios.tcsetattr(ctty_fd, termios.TCSANOW, termios_p)

        self.start_time = time.time()
        self.running = True

        (self.pty_master, pty_slave) = pty.openpty()

        # application_log.debug(my_env)
        if shell:
            self.process = await asyncio.create_subprocess_shell(
                self.cmd,
                env=my_env,
                bufsize=0,
                stdin=pty_slave,
                stdout=pty_slave,
                stderr=pty_slave,
                preexec_fn=lambda: set_ctty(pty_slave, self.pty_master),
            )
        else:
            # TODO: trim down the operation if a shell is not required
            self.process = await asyncio.create_subprocess_exec(
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
        self.returncode = await self.process.wait()
        self.loop.remove_handler(self.pty_master)
        os.close(pty_slave)
        os.close(self.pty_master)

        self.running = False
        self.complete = True
        if self.post_timeout:
            await asyncio.sleep(self.post_timeout)
        application_log.debug(f"[{self.cmd!r} exited with {self.returncode}]")
        if self.complete_callback:
            # let the caller that the process is complete...
            self.complete_callback(**self.to_dict())
        self.process = None
        return self.returncode

    def append_to_output_log(self, name, output):
        should_callback = False
        try:
            output = output.decode("ascii")
        except:
            pass
        if self.strip_output:
            output = self.re_ctrl_format_matcher.sub("", output)
        application_log.debug(f"{name}:{output}")
        if name == "stdout" and output:
            self.stdout = output
            self.stderror = ""
            self.stdout_log.append((self.uptime, output))
            should_callback = True
        elif name == "stderror" and output:
            self.stderror = output
            self.stdout = ""
            self.stderror_log.append((self.uptime, output))
            should_callback = True

        if self.output_callback and should_callback:
            # let them know we have output running...
            self.output_callback(**self.to_dict())
