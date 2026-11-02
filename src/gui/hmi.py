"""The operator screen. It runs on the main thread because Tk is not thread-safe.

The window never calls into a worker. It only

  * drains ui_queue on a timer (root.after) and updates its panels, and
  * puts operator commands on command_queue for the cycle controller.

So a slow robot or a slow model can never freeze the screen, and no Tk call
ever happens from a second thread.
"""

import queue
import time
import tkinter as tk
from tkinter import ttk
from typing import Callable, Optional

from src.app.calibration import MarkerOffsetCalibrator, PlaceOffsetCalibrator
from src.app.messages import (
    CycleState, CycleStatus, LinkState, LinkStatus, LogEvent, OperatorCommand,
    PickRequest, Scene, Severity,
)
from src.gui import theme
from src.gui.widgets import (
    AlarmBanner, CameraView, CyclePanel, EventLog, HeaderBar, JobPanel, KpiPanel,
    MarkerCalibrationPanel, ObjectTable, PlaceAnglePanel, panel, scrollable,
)

RIGHT_COLUMN_WIDTH_PX = 430
# Laid out for a 1280 x 720 panel, the smallest screen the station expects.
MIN_WINDOW_SIZE_PX = (1100, 700)
BOTTOM_ROW_HEIGHT_PX = 170
# Alarm keys (LogEvent.alarm_key) that clear themselves once the condition is
# gone: the marker is back, images arrive, the robot has connected.
MARKER_ALARM = "marker"
CAMERA_ALARM = "camera"
INFERENCE_ALARM = "inference"
LINK_ALARMS = ("link", "bind")
# Calibration may only change while no pick is in flight. FAULT counts:
# the cycle has been aborted and the robot is not moving.
IDLE_STATES = (CycleState.READY, CycleState.STOPPED, CycleState.FAULT)


class HmiSettings:
    """Numbers for the screen, read once from settings.yaml (section hmi)."""

    def __init__(self, station_name: str, refresh_ms: int, max_events_per_refresh: int,
                 image_stale_sec: float, event_log_lines: int, kpi_average_window: int,
                 inference_limit_sec: float) -> None:
        self.station_name = station_name
        self.refresh_ms = int(refresh_ms)
        self.max_events_per_refresh = int(max_events_per_refresh)
        self.image_stale_sec = float(image_stale_sec)
        self.event_log_lines = int(event_log_lines)
        self.kpi_average_window = int(kpi_average_window)
        self.inference_limit_sec = float(inference_limit_sec)


class HmiWindow:
    """Builds the screen and keeps it in step with the pipeline."""

    def __init__(self, root: tk.Tk, settings: HmiSettings, shapes: list,
                 ui_queue: queue.Queue, command_queue: queue.Queue, on_close: Callable,
                 calibrator: Optional[MarkerOffsetCalibrator] = None,
                 angle_calibrator: Optional[PlaceOffsetCalibrator] = None) -> None:
        self._root = root
        self._calibrator = calibrator
        self._calibration: Optional[MarkerCalibrationPanel] = None
        self._angle_calibrator = angle_calibrator
        self._angles: Optional[PlaceAnglePanel] = None
        self._settings = settings
        self._ui_queue = ui_queue
        self._command_queue = command_queue
        self._latest_scene: Optional[Scene] = None
        self._scene_changed = False
        self._last_scene_at_sec = 0.0
        self._cycle_state = CycleState.READY
        self._robot_connected = False

        self._fonts = theme.Fonts()
        self._configure_root(on_close)
        self._build(shapes)
        self._root.after(self._settings.refresh_ms, self._refresh)

    # ------------------------------------------------------------------
    # Layout
    # ------------------------------------------------------------------

    def _configure_root(self, on_close: Callable) -> None:
        self._root.title(self._settings.station_name)
        self._root.configure(bg=theme.BACKGROUND)
        self._root.minsize(*MIN_WINDOW_SIZE_PX)
        self._root.protocol("WM_DELETE_WINDOW", on_close)
        style = ttk.Style(self._root)
        style.theme_use("clam")
        style.configure("Treeview", background=theme.PANEL, fieldbackground=theme.PANEL,
                        foreground=theme.TEXT, font=self._fonts.normal, rowheight=22)
        style.configure("Treeview.Heading", font=self._fonts.bold, background=theme.PANEL_BORDER)
        style.map("Treeview", background=[("selected", theme.SELECTED)])
        style.configure("TNotebook", background=theme.BACKGROUND, borderwidth=0)
        style.configure("TNotebook.Tab", font=self._fonts.bold, padding=(theme.PAD_PX * 2, theme.PAD_PX // 2))
        style.map("TNotebook.Tab", background=[("selected", theme.PANEL)])

    def _build(self, shapes: list) -> None:
        self._header = HeaderBar(self._root, self._settings.station_name, self._fonts)
        self._header.pack(fill="x")
        self._banner = AlarmBanner(self._root, self._fonts)
        self._banner.pack(fill="x", padx=theme.PAD_PX, pady=(theme.PAD_PX, 0))

        # Bottom row first, so it keeps its height and the image takes the rest.
        bottom = tk.Frame(self._root, bg=theme.BACKGROUND, height=BOTTOM_ROW_HEIGHT_PX)
        bottom.pack(side="bottom", fill="x", padx=theme.PAD_PX, pady=(0, theme.PAD_PX))
        bottom.pack_propagate(False)
        self._build_bottom_row(bottom)

        top = tk.Frame(self._root, bg=theme.BACKGROUND)
        top.pack(fill="both", expand=True, padx=theme.PAD_PX, pady=theme.PAD_PX)
        right = tk.Frame(top, bg=theme.BACKGROUND, width=RIGHT_COLUMN_WIDTH_PX)
        right.pack(side="right", fill="y")
        right.pack_propagate(False)
        self._build_right_column(right, shapes)

        view_panel = panel(top, "Picking area - live", self._fonts)
        view_panel.pack(side="left", fill="both", expand=True, padx=(0, theme.PAD_PX))
        # Vision performance sits under the image it belongs to (the
        # inference time is always in view).
        kpi_strip = tk.Frame(view_panel.body, bg=theme.PANEL)
        kpi_strip.pack(side="bottom", fill="x", pady=(theme.PAD_PX // 2, 0))
        self._kpi = KpiPanel(kpi_strip, self._fonts, self._settings.inference_limit_sec,
                             self._settings.kpi_average_window)
        self._camera = CameraView(view_panel.body, self._fonts)
        self._camera.pack(fill="both", expand=True)

    def _build_bottom_row(self, bottom: tk.Frame) -> None:
        objects_panel = panel(bottom, "Detected objects (wobj2)", self._fonts)
        objects_panel.pack(side="left", fill="both", expand=True, padx=(0, theme.PAD_PX))
        self._objects = ObjectTable(objects_panel.body, self._select_from_table)
        log_panel = panel(bottom, "Event log", self._fonts)
        log_panel.pack(side="left", fill="both", expand=True)
        self._log = EventLog(log_panel.body, self._settings.event_log_lines)

    def _build_right_column(self, right: tk.Frame, shapes: list) -> None:
        tabs = ttk.Notebook(right)
        tabs.pack(fill="both", expand=True)
        operation = tk.Frame(tabs, bg=theme.BACKGROUND)
        tabs.add(operation, text="OPERATION")

        job_panel = panel(operation, "Job selection", self._fonts)
        job_panel.pack(fill="x", pady=(theme.PAD_PX // 2, theme.PAD_PX))
        self._job = JobPanel(job_panel.body, shapes, self._fonts, self._pick, self._cycle_stop,
                             self._reset, self._selection_changed)

        cycle_panel = panel(operation, "Cycle", self._fonts)
        cycle_panel.pack(fill="both", expand=True)
        self._cycle = CyclePanel(cycle_panel.body, self._fonts)

        if self._calibrator is not None:
            calibration_tab = tk.Frame(tabs, bg=theme.BACKGROUND)
            tabs.add(calibration_tab, text="CALIBRATION")
            # Scrolls when the window is made smaller than the two panels.
            calibration = scrollable(calibration_tab)
            marker_panel = panel(calibration, "Marker centre in wobj2", self._fonts)
            marker_panel.pack(fill="x", pady=(theme.PAD_PX // 2, 0))
            self._calibration = MarkerCalibrationPanel(marker_panel.body, self._fonts, self._calibrator.offset_mm,
                                                       self._apply_marker_offset, self._save_marker_offset)
            if self._angle_calibrator is not None:
                angle_panel = panel(calibration, "Place angle (cavity rz)", self._fonts)
                angle_panel.pack(fill="x", pady=(theme.PAD_PX, 0))
                self._angles = PlaceAnglePanel(angle_panel.body, self._fonts,
                                               self._angle_calibrator.adjustable_shapes(shapes),
                                               self._add_place_reading, self._save_place_offset,
                                               self._angle_shape_changed)
                self._angle_shape_changed()

    # ------------------------------------------------------------------
    # Operator commands -> command_queue
    # ------------------------------------------------------------------

    def _pick(self) -> None:
        request = PickRequest(self._job.shape.get(), self._job.selected_color())
        self._command_queue.put(request)
        self._log.add(LogEvent(Severity.INFO, "Operator", str(request)))

    def _cycle_stop(self) -> None:
        self._command_queue.put(OperatorCommand.CYCLE_STOP)
        self._log.add(LogEvent(Severity.INFO, "Operator", "cycle stop"))

    def _reset(self) -> None:
        self._command_queue.put(OperatorCommand.RESET)
        self._banner.acknowledge_all()

    def _apply_marker_offset(self, x_text: str, y_text: str) -> None:
        if self._cycle_state not in IDLE_STATES:
            self._log.add(LogEvent(Severity.WARNING, "Calib", "stop the cycle before changing the marker offset"))
            return
        try:
            text = self._calibrator.apply(float(x_text.replace(",", ".")), float(y_text.replace(",", ".")))
        except ValueError as error:
            self._log.add(LogEvent(Severity.WARNING, "Calib", f"not applied: {error}"))
            return
        self._log.add(LogEvent(Severity.INFO, "Calib", text))
        self._calibration.show(self._calibrator.offset_mm, self._calibrator.has_unsaved_change)

    def _save_marker_offset(self) -> None:
        try:
            text = self._calibrator.save()
        except (OSError, KeyError) as error:
            self._log.add(LogEvent(Severity.ALARM, "Calib", f"save failed: {error}"))
            return
        self._log.add(LogEvent(Severity.INFO, "Calib", text))
        self._calibration.show(self._calibrator.offset_mm, unsaved=False)

    def _add_place_reading(self, shape: str, reading_text: str) -> None:
        if self._cycle_state not in IDLE_STATES:
            self._log.add(LogEvent(Severity.WARNING, "Calib", "stop the cycle before changing a place angle"))
            return
        if not reading_text.strip():
            self._log.add(LogEvent(Severity.WARNING, "Calib", "type the rz reading or press USE LIVE first"))
            return
        try:
            text = self._angle_calibrator.add_reading(shape, float(reading_text.replace(",", ".")))
        except ValueError as error:
            self._log.add(LogEvent(Severity.WARNING, "Calib", f"not applied: {error}"))
            return
        self._log.add(LogEvent(Severity.INFO, "Calib", text))
        self._angles.clear_reading()
        self._angle_shape_changed()

    def _save_place_offset(self, shape: str) -> None:
        try:
            text = self._angle_calibrator.save(shape)
        except (OSError, KeyError) as error:
            self._log.add(LogEvent(Severity.ALARM, "Calib", f"save failed: {error}"))
            return
        self._log.add(LogEvent(Severity.INFO, "Calib", text))
        self._angle_shape_changed()

    def _angle_shape_changed(self) -> None:
        shape = self._angles.shape.get()
        self._angles.show_offset(self._angle_calibrator.offset_deg(shape),
                                 self._angle_calibrator.has_unsaved_change(shape))
        self._scene_changed = True

    def _show_live_angle(self, scene: Scene) -> None:
        shape = self._angles.shape.get()
        plates = [item for item in scene.objects if item.shape == shape and item.world_pose is not None]
        self._angles.show_live(plates[0].world_pose.rz_deg if plates else None, len(plates))

    def _select_from_table(self, shape: str, color: str) -> None:
        self._job.select(shape, color)

    def _selection_changed(self) -> None:
        self._scene_changed = True

    # ------------------------------------------------------------------
    # ui_queue -> panels
    # ------------------------------------------------------------------

    def _refresh(self) -> None:
        """Drain what the workers sent, then redraw the image at most once."""
        for _ in range(self._settings.max_events_per_refresh):
            try:
                message = self._ui_queue.get_nowait()
            except queue.Empty:
                break
            self._dispatch(message)
        if self._scene_changed and self._latest_scene is not None:
            self._redraw_scene()
        self._update_camera_lamp()
        self._header.tick()
        self._root.after(self._settings.refresh_ms, self._refresh)

    def _dispatch(self, message) -> None:
        if isinstance(message, Scene):
            self._on_scene(message)
        elif isinstance(message, LogEvent):
            self._log.add(message)
            self._banner.raise_event(message)
        elif isinstance(message, LinkStatus):
            self._on_link(message)
        elif isinstance(message, CycleStatus):
            self._on_cycle(message)

    def _on_scene(self, scene: Scene) -> None:
        self._latest_scene = scene
        self._scene_changed = True
        self._last_scene_at_sec = time.monotonic()
        self._kpi.add(scene.inference_time_sec)
        self._banner.clear(CAMERA_ALARM)
        over_limit = scene.inference_time_sec > self._settings.inference_limit_sec
        self._header.indicators["VISION"].set("warning" if over_limit else "ok",
                                              "SLOW" if over_limit else "RUNNING")
        self._header.indicators["MARKER"].set("ok" if scene.has_marker else "warning",
                                              "FOUND" if scene.has_marker else "MISSING")
        if scene.has_marker:
            self._banner.clear(MARKER_ALARM)
        if not over_limit:
            self._banner.clear(INFERENCE_ALARM)
        self._job.set_colors([scene_object.color for scene_object in scene.objects])

    def _on_link(self, status: LinkStatus) -> None:
        self._robot_connected = status.state is LinkState.CONNECTED
        levels = {LinkState.CONNECTED: ("ok", "CONNECTED"), LinkState.LISTENING: ("warning", "WAITING"),
                  LinkState.OFFLINE: ("alarm", "OFFLINE")}
        self._header.indicators["ROBOT"].set(*levels[status.state])
        if self._robot_connected:
            for alarm_key in LINK_ALARMS:
                self._banner.clear(alarm_key)
        self._scene_changed = True

    def _on_cycle(self, status: CycleStatus) -> None:
        self._cycle_state = status.state
        self._cycle.show(status)
        if self._calibration is not None:
            self._calibration.set_editable(status.state in IDLE_STATES)
        if self._angles is not None:
            self._angles.set_editable(status.state in IDLE_STATES)
        self._header.set_mode(status.state)
        self._scene_changed = True

    def _redraw_scene(self) -> None:
        scene = self._latest_scene
        shape, color = self._job.shape.get(), self._job.selected_color()
        self._camera.show(scene, shape, color)
        self._objects.show(scene, shape, color)
        can_pick = self._robot_connected and self._cycle_state in (CycleState.READY, CycleState.STOPPED)
        self._job.set_matching(len(scene.matching(shape, color)), can_pick)
        if self._angles is not None:
            self._show_live_angle(scene)
        self._scene_changed = False

    def _update_camera_lamp(self) -> None:
        if self._last_scene_at_sec == 0.0:
            return
        stale = time.monotonic() - self._last_scene_at_sec > self._settings.image_stale_sec
        self._header.indicators["CAMERA"].set("alarm" if stale else "ok", "NO IMAGE" if stale else "LIVE")
