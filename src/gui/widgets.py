"""Panels of the operator screen. Each class draws one thing and knows nothing
about threads: the window (src/gui/hmi.py) feeds them from the UI queue.
"""

import math
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from tkinter import ttk
from typing import Callable, Optional

import cv2
import numpy as np

from src.app.messages import ANY_COLOR, CycleState, CycleStatus, LogEvent, Scene, Severity
from src.gui import theme

MILLISECONDS_PER_SECOND = 1000.0
DEGREES_TO_RADIANS = math.pi / 180.0


def panel(parent, title: str, fonts: theme.Fonts) -> tk.Frame:
    """A titled, bordered panel; returns the frame to fill."""
    outer = tk.Frame(parent, bg=theme.PANEL, highlightbackground=theme.PANEL_BORDER,
                     highlightthickness=theme.BORDER_PX)
    tk.Label(outer, text=title.upper(), bg=theme.PANEL, fg=theme.TEXT_MUTED, font=fonts.bold,
             anchor="w").pack(fill="x", padx=theme.PAD_PX, pady=(theme.PAD_PX // 2, 0))
    body = tk.Frame(outer, bg=theme.PANEL)
    body.pack(fill="both", expand=True, padx=theme.PAD_PX, pady=theme.PAD_PX // 2)
    outer.body = body
    return outer


# ----------------------------------------------------------------------
# Header
# ----------------------------------------------------------------------

class StatusIndicator(tk.Frame):
    """A labelled lamp: grey text when normal, coloured only when not."""

    LEVEL_COLORS = {
        "ok": (theme.RUNNING, theme.RUNNING_TEXT),
        "warning": (theme.WARNING, theme.WARNING_TEXT),
        "alarm": (theme.ALARM, theme.ALARM_TEXT),
        "off": (theme.INACTIVE, theme.TEXT),
    }

    def __init__(self, parent, name: str, fonts: theme.Fonts) -> None:
        super().__init__(parent, bg=theme.HEADER)
        tk.Label(self, text=name, bg=theme.HEADER, fg=theme.HEADER_TEXT, font=fonts.small).pack(side="left")
        self._value = tk.Label(self, text="-", width=11, font=fonts.bold)
        self._value.pack(side="left", padx=(4, 0))
        self.set("off", "WAIT")

    def set(self, level: str, text: str) -> None:
        background, foreground = self.LEVEL_COLORS[level]
        self._value.configure(text=text, bg=background, fg=foreground)


class HeaderBar(tk.Frame):
    """Station name, cycle mode, subsystem lamps and the clock."""

    def __init__(self, parent, station_name: str, fonts: theme.Fonts) -> None:
        super().__init__(parent, bg=theme.HEADER)
        tk.Label(self, text=station_name, bg=theme.HEADER, fg=theme.HEADER_TEXT,
                 font=fonts.title).pack(side="left", padx=theme.PAD_PX * 2, pady=theme.PAD_PX)
        self._mode = tk.Label(self, text="READY", width=14, font=fonts.large)
        self._mode.pack(side="left", padx=theme.PAD_PX)
        self._clock = tk.Label(self, bg=theme.HEADER, fg=theme.HEADER_TEXT, font=fonts.large)
        self._clock.pack(side="right", padx=theme.PAD_PX * 2)
        self.indicators = {}
        for name in ("ROBOT", "MARKER", "VISION", "CAMERA"):
            indicator = StatusIndicator(self, name, fonts)
            indicator.pack(side="right", padx=theme.PAD_PX)
            self.indicators[name] = indicator
        self.set_mode(CycleState.READY)

    def set_mode(self, state: CycleState) -> None:
        colors = {
            CycleState.FAULT: (theme.ALARM, theme.ALARM_TEXT),
            CycleState.STOPPED: (theme.WARNING, theme.WARNING_TEXT),
            CycleState.READY: (theme.INACTIVE, theme.TEXT),
        }
        background, foreground = colors.get(state, (theme.RUNNING, theme.RUNNING_TEXT))
        self._mode.configure(text=state.value, bg=background, fg=foreground)

    def tick(self) -> None:
        self._clock.configure(text=datetime.now().strftime("%H:%M:%S"))


# ----------------------------------------------------------------------
# Alarms
# ----------------------------------------------------------------------

class AlarmBanner(tk.Frame):
    """The newest unacknowledged warning or alarm, with an ACK button.

    Grey and quiet while nothing is pending, the same bar turns amber or red
    the moment something needs the operator.
    """

    def __init__(self, parent, fonts: theme.Fonts) -> None:
        super().__init__(parent, bg=theme.PANEL)
        self._pending: list[tuple[str, LogEvent]] = []
        self._text = tk.Label(self, anchor="w", font=fonts.bold, padx=theme.PAD_PX)
        self._text.pack(side="left", fill="both", expand=True)
        self._count = tk.Label(self, font=fonts.bold, width=12)
        self._count.pack(side="left")
        self._ack = tk.Button(self, text="ACKNOWLEDGE", font=fonts.bold, command=self.acknowledge_all,
                              relief="flat", bg=theme.PANEL, activebackground=theme.PANEL_BORDER)
        self._ack.pack(side="right", padx=theme.PAD_PX // 2, pady=2)
        self._refresh()

    def raise_event(self, event: LogEvent) -> None:
        if event.severity is Severity.INFO:
            return
        stamp = datetime.now().strftime("%H:%M:%S")
        self._pending = [(when, pending) for when, pending in self._pending
                         if not event.alarm_key or pending.alarm_key != event.alarm_key]
        self._pending.append((stamp, event))
        self._refresh()

    def clear(self, alarm_key: str) -> None:
        """Drop a condition that has gone away by itself (e.g. marker found)."""
        self._pending = [(when, event) for when, event in self._pending if event.alarm_key != alarm_key]
        self._refresh()

    def acknowledge_all(self) -> None:
        self._pending.clear()
        self._refresh()

    def _refresh(self) -> None:
        if not self._pending:
            self._show("No active alarms", theme.PANEL, theme.TEXT_MUTED, "")
            return
        # The newest ALARM outranks any warning; otherwise the newest warning.
        alarms = [item for item in self._pending if item[1].severity is Severity.ALARM]
        stamp, event = (alarms or self._pending)[-1]
        background, foreground = ((theme.ALARM, theme.ALARM_TEXT) if event.severity is Severity.ALARM
                                  else (theme.WARNING, theme.WARNING_TEXT))
        self._show(f"{stamp}  {event.severity.value}  {event.source}: {event.text}",
                   background, foreground, f"{len(self._pending)} pending")

    def _show(self, text: str, background: str, foreground: str, count: str) -> None:
        for widget in (self, self._text, self._count):
            widget.configure(bg=background)
        self._text.configure(text=text, fg=foreground)
        self._count.configure(text=count, fg=foreground)
        self._ack.configure(state="normal" if count else "disabled")


# ----------------------------------------------------------------------
# Camera view
# ----------------------------------------------------------------------

class CameraView(tk.Label):
    """The latest frame, scaled to fit, with what the system understood drawn on it."""

    def __init__(self, parent, fonts: theme.Fonts) -> None:
        super().__init__(parent, bg=theme.VIEW_BACKGROUND, fg=theme.HEADER_TEXT,
                         text="Waiting for the first image...", font=fonts.large,
                         borderwidth=0, highlightthickness=0, padx=0, pady=0)
        # The image is scaled to this label's size; any border or padding would
        # make the label request a few pixels more each frame and grow the window.
        self._photo = None

    def show(self, scene: Scene, selected_shape: str, selected_color: str) -> None:
        width_px, height_px = max(self.winfo_width(), 1), max(self.winfo_height(), 1)
        frame_height_px, frame_width_px = scene.frame.shape[:2]
        scale = min(width_px / frame_width_px, height_px / frame_height_px)
        view_size_px = (int(frame_width_px * scale), int(frame_height_px * scale))
        # Before Tk lays the window out the label is 1 px wide, which rounds to a zero size.
        if min(view_size_px) <= 0:
            return
        view = cv2.resize(scene.frame, view_size_px, interpolation=cv2.INTER_AREA)
        self._draw_marker(view, scene, scale)
        for scene_object in scene.objects:
            is_target = scene_object.matches(selected_shape, selected_color)
            self._draw_object(view, scene_object, scale, is_target)
        # PPM is the one format Tk reads natively and OpenCV writes without
        # compression, so no Pillow is needed on the Pi.
        _encoded, ppm = cv2.imencode(".ppm", view)
        self._photo = tk.PhotoImage(data=ppm.tobytes(), format="PPM")
        self.configure(image=self._photo, text="")

    @staticmethod
    def _draw_object(view, scene_object, scale: float, is_target: bool) -> None:
        detection = scene_object.detection
        center = (int(detection.center_x_px * scale), int(detection.center_y_px * scale))
        radius_px = int(max(detection.width_px, detection.height_px) * scale / 2)
        color = theme.OVERLAY_TARGET_BGR if is_target else theme.OVERLAY_OBJECT_BGR
        thickness_px = 3 if is_target else 1
        cv2.circle(view, center, radius_px, color, thickness_px, cv2.LINE_AA)
        angle_rad = detection.angle_deg * DEGREES_TO_RADIANS
        tip = (int(center[0] + radius_px * math.cos(angle_rad)), int(center[1] + radius_px * math.sin(angle_rad)))
        cv2.arrowedLine(view, center, tip, color, 2, cv2.LINE_AA, tipLength=0.25)
        cv2.putText(view, f"#{scene_object.object_id}", (center[0] - radius_px, center[1] - radius_px - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    @staticmethod
    def _draw_marker(view, scene: Scene, scale: float) -> None:
        if scene.marker_pose is None:
            cv2.putText(view, "NO MARKER", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        theme.OVERLAY_MARKER_X_BGR, 2, cv2.LINE_AA)
            return
        top_left, top_right, _bottom_right, bottom_left = scene.marker_pose.corners_px * scale
        center = np.array([scene.marker_pose.center_x_px, scene.marker_pose.center_y_px]) * scale
        for corner, color in ((top_right, theme.OVERLAY_MARKER_X_BGR), (bottom_left, theme.OVERLAY_MARKER_Y_BGR)):
            direction = (corner - top_left) / max(np.linalg.norm(corner - top_left), 1e-6)
            tip = center + direction * np.linalg.norm(top_right - top_left)
            cv2.arrowedLine(view, tuple(center.astype(int)), tuple(tip.astype(int)), color, 2, cv2.LINE_AA)


# ----------------------------------------------------------------------
# Operator controls
# ----------------------------------------------------------------------

class JobPanel:
    """Shape and colour selection plus the three cycle buttons."""

    def __init__(self, body: tk.Frame, shapes: list, fonts: theme.Fonts,
                 on_pick: Callable, on_stop: Callable, on_reset: Callable,
                 on_selection_changed: Callable) -> None:
        self._fonts = fonts
        self._on_selection_changed = on_selection_changed
        self.shape = tk.StringVar(value=shapes[0])
        self.color = tk.StringVar(value=ANY_COLOR.upper())

        tk.Label(body, text="Shape", bg=theme.PANEL, fg=theme.TEXT_MUTED, font=fonts.small,
                 anchor="w").pack(fill="x")
        shape_row = tk.Frame(body, bg=theme.PANEL)
        shape_row.pack(fill="x", pady=(0, theme.PAD_PX))
        self._shape_buttons = {}
        for shape in shapes:
            button = tk.Button(shape_row, text=shape.upper(), font=fonts.bold, relief="flat",
                               pady=theme.BUTTON_PAD_Y_PX, command=lambda chosen=shape: self.select(chosen, None))
            button.pack(side="left", fill="x", expand=True, padx=(0, 4))
            self._shape_buttons[shape] = button

        tk.Label(body, text="Colour", bg=theme.PANEL, fg=theme.TEXT_MUTED, font=fonts.small,
                 anchor="w").pack(fill="x")
        self._color_box = ttk.Combobox(body, textvariable=self.color, state="readonly", font=fonts.large)
        self._color_box.pack(fill="x", pady=(0, theme.PAD_PX))
        self._color_box.bind("<<ComboboxSelected>>", lambda _event: self._changed())

        self._matching = tk.Label(body, text="", bg=theme.PANEL, fg=theme.TEXT, font=fonts.normal, anchor="w")
        self._matching.pack(fill="x")

        self.pick_button = self._command_button(body, "▶  PICK SELECTED", theme.RUNNING, theme.RUNNING_TEXT, on_pick)
        buttons = tk.Frame(body, bg=theme.PANEL)
        buttons.pack(fill="x")
        self._command_button(buttons, "■  CYCLE STOP", theme.ALARM, theme.ALARM_TEXT, on_stop, side="left")
        self._command_button(buttons, "↺  RESET", theme.INACTIVE, theme.TEXT, on_reset, side="left")
        self._paint_shapes()
        self.set_colors([])

    def _command_button(self, parent, text, background, foreground, command, side=None) -> tk.Button:
        button = tk.Button(parent, text=text, font=self._fonts.large, bg=background, fg=foreground,
                           activebackground=background, activeforeground=foreground, relief="flat",
                           pady=theme.BUTTON_PAD_Y_PX, command=command, disabledforeground=theme.TEXT_MUTED)
        if side:
            button.pack(side=side, fill="x", expand=True, padx=(0, 4), pady=(theme.PAD_PX // 2, 0))
        else:
            button.pack(fill="x", pady=(theme.PAD_PX, 0))
        return button

    def select(self, shape: Optional[str], color: Optional[str]) -> None:
        if shape is not None:
            self.shape.set(shape)
        if color is not None:
            self.color.set(color.upper())
        self._paint_shapes()
        self._changed()

    def selected_color(self) -> str:
        return self.color.get().lower()

    def set_colors(self, seen_colors: list) -> None:
        """Offer ANY plus every colour on the paper right now."""
        values = [ANY_COLOR.upper()] + sorted({color.upper() for color in seen_colors})
        if self.color.get() not in values:
            values.append(self.color.get())
        self._color_box.configure(values=values)

    def set_matching(self, count: int, can_pick: bool) -> None:
        self._matching.configure(text=f"Matching objects on the paper: {count}",
                                 fg=theme.TEXT if count else theme.ALARM)
        self.pick_button.configure(state="normal" if can_pick and count else "disabled",
                                   bg=theme.RUNNING if can_pick and count else theme.INACTIVE)

    def _paint_shapes(self) -> None:
        for shape, button in self._shape_buttons.items():
            chosen = shape == self.shape.get()
            button.configure(bg=theme.SELECTED if chosen else theme.PANEL_BORDER,
                             fg=theme.SELECTED_TEXT if chosen else theme.TEXT,
                             activebackground=theme.SELECTED if chosen else theme.PANEL_BORDER)

    def _changed(self) -> None:
        self._on_selection_changed()


# ----------------------------------------------------------------------
# Data displays
# ----------------------------------------------------------------------

class ObjectTable:
    """Every detected plate with its robot pose -- the GUI requirement of the brief."""

    COLUMNS = (("id", "#", 40), ("shape", "Shape", 90), ("color", "Colour", 80),
               ("x", "X [mm]", 80), ("y", "Y [mm]", 80), ("rz", "Rz [deg]", 80), ("conf", "Conf", 60))

    def __init__(self, body: tk.Frame, on_row_selected: Callable) -> None:
        self._tree = ttk.Treeview(body, columns=[key for key, _, _ in self.COLUMNS], show="headings",
                                  height=6, selectmode="browse")
        for key, title, width_px in self.COLUMNS:
            self._tree.heading(key, text=title)
            self._tree.column(key, width=width_px, anchor="e" if key in ("x", "y", "rz", "conf") else "w")
        self._tree.tag_configure("target", background=theme.SELECTED, foreground=theme.SELECTED_TEXT)
        self._tree.pack(fill="both", expand=True)
        self._tree.bind("<<TreeviewSelect>>", lambda _event: self._row_selected(on_row_selected))

    def show(self, scene: Scene, selected_shape: str, selected_color: str) -> None:
        self._tree.delete(*self._tree.get_children())
        for scene_object in scene.objects:
            pose = scene_object.world_pose
            values = (scene_object.object_id, scene_object.shape.upper(), scene_object.color.upper(),
                      f"{pose.x_mm:.1f}" if pose else "-", f"{pose.y_mm:.1f}" if pose else "-",
                      f"{pose.rz_deg:.1f}" if pose else "-", f"{scene_object.detection.confidence:.2f}")
            tags = ("target",) if scene_object.matches(selected_shape, selected_color) else ()
            self._tree.insert("", "end", values=values, tags=tags)

    def _row_selected(self, on_row_selected: Callable) -> None:
        selection = self._tree.selection()
        if selection:
            values = self._tree.item(selection[0], "values")
            on_row_selected(values[1].lower(), values[2].lower())


class CyclePanel:
    """Where the cycle is, which attempt, and the production counters."""

    PAIRS_PER_ROW = 2
    STATE_COLORS = {
        CycleState.READY: (theme.PANEL_BORDER, theme.TEXT),
        CycleState.STOPPED: (theme.WARNING, theme.WARNING_TEXT),
        CycleState.FAULT: (theme.ALARM, theme.ALARM_TEXT),
    }

    def __init__(self, body: tk.Frame, fonts: theme.Fonts) -> None:
        self._state = tk.Label(body, text="READY", font=fonts.state)
        self._state.pack(fill="x")
        self._target = tk.Label(body, text="-", bg=theme.PANEL, fg=theme.TEXT, font=fonts.bold)
        self._target.pack(fill="x")
        grid = tk.Frame(body, bg=theme.PANEL)
        grid.pack(fill="x", pady=(theme.PAD_PX // 2, 0))
        self._values = {}
        # Two label/value pairs per row keeps the panel short on a 720p screen.
        fields = (("attempt", "Attempt"), ("cycle", "This cycle"), ("cycles", "Cycles done"),
                  ("time", "Last pick"), ("good", "Placed"), ("failed", "Failed"))
        for index, (key, title) in enumerate(fields):
            row, column = divmod(index, self.PAIRS_PER_ROW)
            tk.Label(grid, text=title, bg=theme.PANEL, fg=theme.TEXT_MUTED, font=fonts.normal,
                     anchor="w").grid(row=row, column=column * 2, sticky="w", padx=(0, theme.PAD_PX))
            value = tk.Label(grid, text="-", bg=theme.PANEL, fg=theme.TEXT, font=fonts.bold, anchor="e")
            value.grid(row=row, column=column * 2 + 1, sticky="e", padx=(0, theme.PAD_PX * 2))
            self._values[key] = value
        for column in (1, 3):
            grid.columnconfigure(column, weight=1)

    def show(self, status: CycleStatus) -> None:
        background, foreground = self.STATE_COLORS.get(status.state, (theme.RUNNING, theme.RUNNING_TEXT))
        self._state.configure(text=status.state.value, bg=background, fg=foreground)
        self._target.configure(text=f"Target: {status.target}")
        self._values["attempt"].configure(text=f"{status.attempt} / {status.max_attempts}" if status.attempt else "-")
        self._values["cycle"].configure(text=f"{status.picks_in_cycle} / {status.objects_per_cycle}")
        self._values["cycles"].configure(text=str(status.completed_cycles))
        self._values["good"].configure(text=str(status.good_picks))
        self._values["failed"].configure(text=str(status.failed_picks),
                                         fg=theme.ALARM if status.failed_picks else theme.TEXT)
        self._values["time"].configure(text=f"{status.last_pick_time_sec:.1f} s" if status.last_pick_time_sec else "-")


class KpiPanel:
    """Inference time against the 3 s limit (T5), as number and as bar."""

    BAR_HEIGHT_PX = 14

    def __init__(self, body: tk.Frame, fonts: theme.Fonts, limit_sec: float, average_window: int) -> None:
        self._limit_sec = float(limit_sec)
        self._samples = deque(maxlen=int(average_window))
        self._frame_times = deque(maxlen=int(average_window))
        self._last = self._value(body, "INFERENCE", fonts, fonts.large)
        self._average = self._value(body, "AVG", fonts, fonts.bold)
        self._rate = self._value(body, "FRAMES/S", fonts, fonts.bold)
        tk.Label(body, text=f"LIMIT {self._limit_sec:.1f} s", bg=theme.PANEL, fg=theme.TEXT_MUTED,
                 font=fonts.small).pack(side="right")
        self._bar = tk.Canvas(body, height=self.BAR_HEIGHT_PX, bg=theme.PANEL_BORDER, highlightthickness=0)
        self._bar.pack(side="right", fill="x", expand=True, padx=theme.PAD_PX)

    @staticmethod
    def _value(parent, title: str, fonts: theme.Fonts, value_font) -> tk.Label:
        tk.Label(parent, text=title, bg=theme.PANEL, fg=theme.TEXT_MUTED, font=fonts.small).pack(side="left")
        value = tk.Label(parent, text="-", bg=theme.PANEL, fg=theme.TEXT, font=value_font)
        value.pack(side="left", padx=(4, theme.PAD_PX * 2))
        return value

    def add(self, inference_time_sec: float) -> None:
        self._samples.append(inference_time_sec)
        self._frame_times.append(time.monotonic())
        over = inference_time_sec > self._limit_sec
        self._last.configure(text=f"{inference_time_sec * MILLISECONDS_PER_SECOND:.0f} ms",
                             fg=theme.ALARM if over else theme.TEXT)
        self._average.configure(text=f"{np.mean(self._samples) * MILLISECONDS_PER_SECOND:.0f} ms")
        if len(self._frame_times) > 1:
            span_sec = self._frame_times[-1] - self._frame_times[0]
            self._rate.configure(text=f"{(len(self._frame_times) - 1) / max(span_sec, 1e-6):.1f}")
        self._draw_bar(inference_time_sec)

    def _draw_bar(self, inference_time_sec: float) -> None:
        width_px = self._bar.winfo_width()
        fill_px = int(width_px * min(inference_time_sec / self._limit_sec, 1.0))
        self._bar.delete("all")
        self._bar.create_rectangle(0, 0, fill_px, self.BAR_HEIGHT_PX, width=0,
                                   fill=theme.ALARM if inference_time_sec > self._limit_sec else theme.SELECTED)


class EventLog:
    """Timestamped history of everything that happened, newest on top."""

    def __init__(self, body: tk.Frame, max_lines: int) -> None:
        self._max_lines = int(max_lines)
        self._tree = ttk.Treeview(body, columns=("time", "level", "source", "text"), show="headings", height=6)
        for key, title, width_px in (("time", "Time", 70), ("level", "Level", 85),
                                     ("source", "Source", 70), ("text", "Message", 420)):
            self._tree.heading(key, text=title)
            self._tree.column(key, width=width_px, anchor="w", stretch=key == "text")
        self._tree.tag_configure(Severity.ALARM.value, background=theme.ALARM, foreground=theme.ALARM_TEXT)
        self._tree.tag_configure(Severity.WARNING.value, background=theme.WARNING, foreground=theme.WARNING_TEXT)
        self._tree.pack(fill="both", expand=True)

    def add(self, event: LogEvent) -> None:
        self._tree.insert("", 0, values=(datetime.now().strftime("%H:%M:%S"), event.severity.value,
                                         event.source, event.text), tags=(event.severity.value,))
        rows = self._tree.get_children()
        if len(rows) > self._max_lines:
            self._tree.delete(*rows[self._max_lines:])
