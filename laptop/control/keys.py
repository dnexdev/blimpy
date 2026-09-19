"""Non-blocking single-key reader for the terminal (Windows msvcrt / POSIX termios)."""
import os, sys

ESC = "\x1b"


class KeyPoller:
    def __init__(self):
        self.win = os.name == "nt"
        if not self.win:
            import termios, tty
            self.fd = sys.stdin.fileno()
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)

    def poll(self):
        """Return one pressed key (str) or None."""
        if self.win:
            import msvcrt
            if not msvcrt.kbhit():
                return None
            ch = msvcrt.getwch()
            if ch in ("\x00", "\xe0"):      # arrow / function key prefix: swallow the second code
                msvcrt.getwch()
                return None
            return ch
        import select
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def close(self):
        if not self.win:
            import termios
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
