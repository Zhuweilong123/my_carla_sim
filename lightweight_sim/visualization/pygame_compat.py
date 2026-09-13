"""Small pygame compatibility helpers shared by GUI entry points."""

import pygame


def patch_sysfont_for_python314() -> None:
    """Make pygame-ce font discovery tolerant of Python 3.14 Win32 values."""

    try:
        import pygame.sysfont as sysfont

        original_init = sysfont.initsysfonts_win32

        def safe_init():
            fonts = {}
            try:
                result = original_init()
                for name, path in result.items():
                    if isinstance(name, str) and isinstance(path, str):
                        fonts[name] = path
            except Exception:
                pass
            return fonts

        sysfont.initsysfonts_win32 = safe_init
        original_sysfont_init = sysfont.SysFont.__init__

        def safe_sysfont_init(self, name, size, bold=False, italic=False):
            try:
                original_sysfont_init(self, name, size, bold, italic)
            except TypeError:
                self.__dict__.clear()
                pygame.font.Font.__init__(self, None, size)

        sysfont.SysFont.__init__ = safe_sysfont_init
    except Exception:
        pass
