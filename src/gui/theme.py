"""Visual language of the operator screen, after ISA-101 "high performance HMI".

The rules the palette follows:

  * Normal is grey. A screen where nothing needs attention is calm and
    colourless, so that colour, when it appears, is noticed at once.
  * Colour carries meaning and only that: red = alarm (act now),
    amber = warning (look soon), blue = the operator's own selection.
    Green is used only for "connected / running", never as decoration.
  * Every coloured indicator also carries text, so the screen still reads
    correctly for a colour-blind operator.
"""

import tkinter.font as tkfont

# Neutral greys -- the whole screen at rest.
BACKGROUND = "#D4D7DB"
PANEL = "#E9EBED"
PANEL_BORDER = "#B7BCC2"
HEADER = "#3A3F45"
HEADER_TEXT = "#F2F3F4"
TEXT = "#1D2125"
TEXT_MUTED = "#5E656D"
VIEW_BACKGROUND = "#2B2F33"

# Meaningful colours, each paired with a text label wherever it is used.
ALARM = "#C8102E"
ALARM_TEXT = "#FFFFFF"
WARNING = "#F2A900"
WARNING_TEXT = "#1D2125"
SELECTED = "#1F4E79"
SELECTED_TEXT = "#FFFFFF"
RUNNING = "#2E7D32"
RUNNING_TEXT = "#FFFFFF"
INACTIVE = "#9AA0A6"

# Overlay colours on the camera image, as OpenCV BGR tuples.
OVERLAY_OBJECT_BGR = (230, 230, 230)
# The operator's selection, in a lighter tone of SELECTED so it stands out
# on the dark camera image.
OVERLAY_TARGET_BGR = (255, 170, 60)
OVERLAY_MARKER_X_BGR = (46, 16, 200)        # ALARM red: the X axis
OVERLAY_MARKER_Y_BGR = (200, 120, 30)       # blue: the Y axis

PAD_PX = 8
BORDER_PX = 1
BUTTON_PAD_Y_PX = 7

# Families tried in order; DejaVu Sans ships with Raspberry Pi OS.
FONT_FAMILIES = ("Segoe UI", "DejaVu Sans", "Helvetica", "Arial")
FONT_SIZE_SMALL = 9
FONT_SIZE_NORMAL = 10
FONT_SIZE_LARGE = 13
FONT_SIZE_VALUE = 20
FONT_SIZE_STATE = 17
FONT_SIZE_TITLE = 14


def pick_font_family() -> str:
    """The first preferred family that this machine actually has."""
    available = set(tkfont.families())
    for family in FONT_FAMILIES:
        if family in available:
            return family
    return "TkDefaultFont"


class Fonts:
    """Named fonts, created once the Tk root exists."""

    def __init__(self) -> None:
        family = pick_font_family()
        self.small = (family, FONT_SIZE_SMALL)
        self.normal = (family, FONT_SIZE_NORMAL)
        self.bold = (family, FONT_SIZE_NORMAL, "bold")
        self.large = (family, FONT_SIZE_LARGE, "bold")
        self.value = (family, FONT_SIZE_VALUE, "bold")
        self.state = (family, FONT_SIZE_STATE, "bold")
        self.title = (family, FONT_SIZE_TITLE, "bold")
        self.mono = ("DejaVu Sans Mono", FONT_SIZE_SMALL) if "DejaVu Sans Mono" in tkfont.families() \
            else ("Consolas", FONT_SIZE_SMALL)
