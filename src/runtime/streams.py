# SteerPy author, 2026.

import sys


class JSStream:
    def __init__(self, callback):
        self._callback = callback

    def write(self, text):
        if text:
            self._callback(str(text))
        return len(text)

    def flush(self):
        pass

    def isatty(self):
        return False


class StreamBridge:
    def __init__(self, stdout_callback, stderr_callback):
        self.stdout_callback = stdout_callback
        self.stderr_callback = stderr_callback

    def install(self):
        sys.stdout = JSStream(self.stdout_callback)
        sys.stderr = JSStream(self.stderr_callback)


_stream_bridge = StreamBridge(_cb_stdout, _cb_stderr)
_stream_bridge.install()
