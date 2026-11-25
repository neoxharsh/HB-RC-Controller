"""
HB RC Tkinter Desktop UI
========================

Frontend for hb_rc_controller.py.

Features:
* BLE connect/disconnect and continuous packet streaming.
* Throttle, steering and speed controls.
* Independent throttle and steering acceleration/deceleration durations.
* Independent acceleration and deceleration curve shapes for each axis:
      Linear, Ease In, Ease Out, Ease In-Out
* BLE device discovery plus editable/manual Bluetooth address entry.
* Manual throttle/steering sliders return to centre when released.
* Keyboard controls ramp toward their targets, then smoothly return to centre
  after keys are released:
      W / Up       forward
      S / Down     reverse
      A / Left     steer left
      D / Right    steer right
      Space        immediate neutral
* Three-state lights with remembered cyclic state.

Install:
    python -m pip install bleak

Run from the directory containing both files:
    python hb_rc_ui.py

Curve settings are application-side output shaping. They do not change the
car's firmware or BLE packet format. The drive field mapping remains
experimental; test with wheels raised and clear of people/objects.
"""

import asyncio
import math
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

from bleak import BleakScanner
from hb_rc_controller import HBRCController, LightMode, LIGHT_MODE_NAMES


CURVE_NAMES = ("Linear", "Ease In", "Ease Out", "Ease In-Out")


def apply_curve(progress: float, curve_name: str) -> float:
    """Map normalised progress 0..1 through the selected easing curve."""
    x = max(0.0, min(1.0, progress))
    if curve_name == "Ease In":
        return x * x
    if curve_name == "Ease Out":
        return 1.0 - (1.0 - x) * (1.0 - x)
    if curve_name == "Ease In-Out":
        return 0.5 - 0.5 * math.cos(math.pi * x)
    return x


class MotionAxis:
    """
    One independently configurable axis.

    Acceleration duration is used when moving away from neutral/increasing
    magnitude. Deceleration duration is used when returning toward neutral.
    Reversing direction first uses the deceleration profile to reach zero,
    then the acceleration profile in the opposite direction.
    """

    def __init__(
        self, accel_time=0.45, decel_time=0.30,
        accel_curve="Ease In-Out", decel_curve="Ease In-Out"
    ):
        self.value = 0.0
        self.target = 0.0
        self.start_value = 0.0
        self.segment_started = time.monotonic()
        self.segment_duration = 0.0
        self.segment_curve = accel_curve
        self.accel_time = float(accel_time)
        self.decel_time = float(decel_time)
        self.accel_curve = accel_curve
        self.decel_curve = decel_curve
        self.pending_target = None

    def _start_segment(self, target, duration, curve):
        self.start_value = self.value
        self.target = float(target)
        self.segment_started = time.monotonic()
        self.segment_curve = curve
        self.segment_duration = max(0.0, duration) * (
            abs(self.target - self.start_value) / 100.0
        )
        if self.segment_duration <= 0:
            self.value = self.target

    def set_target(self, target):
        target = max(-100.0, min(100.0, float(target)))
        self.value = self.current_value()
        if abs(target - self.target) < 0.001 and self.pending_target is None:
            return

        # Direction reversal is deliberately split into two phases:
        # decelerate to neutral, then accelerate in the opposite direction.
        if self.value * target < 0:
            self.pending_target = target
            self._start_segment(0.0, self.decel_time, self.decel_curve)
            return

        self.pending_target = None
        accelerating = (
            target != 0 and (
                self.value == 0 or
                (self.value * target > 0 and abs(target) > abs(self.value))
            )
        )
        duration = self.accel_time if accelerating else self.decel_time
        curve = self.accel_curve if accelerating else self.decel_curve
        self._start_segment(target, duration, curve)

    def current_value(self):
        if self.segment_duration <= 0:
            self.value = self.target
        else:
            elapsed = time.monotonic() - self.segment_started
            progress = min(1.0, elapsed / self.segment_duration)
            eased = apply_curve(progress, self.segment_curve)
            self.value = self.start_value + (
                self.target - self.start_value
            ) * eased
            if progress >= 1.0:
                self.value = self.target

        if self.pending_target is not None and abs(self.value) < 0.001:
            next_target = self.pending_target
            self.pending_target = None
            self.value = 0.0
            self._start_segment(next_target, self.accel_time, self.accel_curve)
        return self.value

    def update_settings(
        self, accel_time=None, decel_time=None,
        accel_curve=None, decel_curve=None
    ):
        current = self.current_value()
        old_target = self.pending_target if self.pending_target is not None else self.target
        if accel_time is not None:
            self.accel_time = float(accel_time)
        if decel_time is not None:
            self.decel_time = float(decel_time)
        if accel_curve is not None:
            self.accel_curve = accel_curve
        if decel_curve is not None:
            self.decel_curve = decel_curve
        self.value = current
        self.target = current
        self.pending_target = None
        self.set_target(old_target)


class HBRCTkUI:
    BG = "#101820"
    PANEL = "#1b2833"
    EDGE = "#304352"
    TEXT = "#e8f0f5"
    MUTED = "#9fb0bd"
    ACCENT = "#35b6a5"
    WARNING = "#e4b44c"
    RED = "#e05b5b"

    UPDATE_MS = 20

    def __init__(self, root):
        self.root = root
        self.root.title("HB RC — Desktop Controller")
        self.root.geometry("1080x930")
        self.root.minsize(900, 820)
        self.root.configure(bg=self.BG)

        self.controller = HBRCController()
        self.connected = False
        self.scanning = False
        self.scan_thread = None
        self.device_choices = {}
        self.closing = False

        self.throttle_axis = MotionAxis()
        self.steering_axis = MotionAxis(
            accel_time=0.30, decel_time=0.22,
            accel_curve="Ease In-Out", decel_curve="Ease In-Out"
        )
        self.pressed_keys = set()
        self.keyboard_active = False
        self.slider_dragging = {"throttle": False, "steering": False}
        self.last_sent = (None, None)
        self._updating_controls = False

        self.throttle_var = tk.IntVar(value=0)
        self.steering_var = tk.IntVar(value=0)
        self.speed_var = tk.IntVar(value=1)

        self.throttle_accel_var = tk.DoubleVar(value=0.45)
        self.throttle_decel_var = tk.DoubleVar(value=0.30)
        self.throttle_accel_curve_var = tk.StringVar(value="Ease In-Out")
        self.throttle_decel_curve_var = tk.StringVar(value="Ease In-Out")

        self.steering_accel_var = tk.DoubleVar(value=0.30)
        self.steering_decel_var = tk.DoubleVar(value=0.22)
        self.steering_accel_curve_var = tk.StringVar(value="Ease In-Out")
        self.steering_decel_curve_var = tk.StringVar(value="Ease In-Out")

        self._configure_styles()
        self._build_ui()
        self._bind_events()

        self.root.after(self.UPDATE_MS, self._motion_tick)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _configure_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TButton", font=("Segoe UI", 9, "bold"), padding=7)
        style.configure("Connect.TButton", background=self.ACCENT, foreground="#061411")
        style.configure("Stop.TButton", background=self.RED, foreground="#ffffff")
        style.configure("TCombobox", padding=4)

    def _panel(self, parent, title):
        frame = tk.Frame(
            parent, bg=self.PANEL, highlightthickness=1,
            highlightbackground=self.EDGE
        )
        tk.Label(
            frame, text=title, bg=self.PANEL, fg=self.MUTED,
            font=("Segoe UI", 10, "bold")
        ).pack(anchor="w", padx=14, pady=(11, 7))
        return frame

    def _build_ui(self):
        header = tk.Frame(self.root, bg=self.BG)
        header.pack(fill="x", padx=18, pady=(15, 10))

        tk.Label(
            header, text="HB RC", bg=self.BG, fg=self.TEXT,
            font=("Segoe UI", 24, "bold")
        ).pack(side="left")
        tk.Label(
            header, text="Bluetooth vehicle controller",
            bg=self.BG, fg=self.MUTED, font=("Segoe UI", 10)
        ).pack(side="left", padx=12, pady=(9, 0))

        self.status_var = tk.StringVar(value="Disconnected")
        self.status_dot = tk.Label(
            header, text="●", bg=self.BG, fg=self.MUTED,
            font=("Segoe UI", 14)
        )
        self.status_dot.pack(side="right", padx=(8, 0))
        tk.Label(
            header, textvariable=self.status_var,
            bg=self.BG, fg=self.MUTED, font=("Segoe UI", 10, "bold")
        ).pack(side="right")

        connection = tk.Frame(self.root, bg=self.PANEL, padx=13, pady=10)
        connection.pack(fill="x", padx=18, pady=(0, 12))

        tk.Label(
            connection, text="BLE device address",
            bg=self.PANEL, fg=self.MUTED, font=("Segoe UI", 8, "bold")
        ).grid(row=0, column=0, sticky="w")
        self.address_var = tk.StringVar(value="E0:97:00:00:00:00")
        self.address_entry = ttk.Entry(
            connection, textvariable=self.address_var, width=25
        )
        self.address_entry.grid(row=1, column=0, sticky="ew", padx=(0, 8))

        self.device_var = tk.StringVar(value="Select a scanned car (optional)")
        self.device_combo = ttk.Combobox(
            connection, textvariable=self.device_var, state="readonly", width=34
        )
        self.device_combo.grid(row=1, column=1, sticky="ew", padx=4)
        self.device_combo.bind("<<ComboboxSelected>>", self._device_selected)

        self.scan_button = ttk.Button(
            connection, text="Scan for cars", command=self._scan_devices
        )
        self.scan_button.grid(row=1, column=2, padx=5)
        self.connect_button = ttk.Button(
            connection, text="Connect", style="Connect.TButton",
            command=self._toggle_connection
        )
        self.connect_button.grid(row=1, column=3, padx=(5, 0))
        connection.grid_columnconfigure(0, weight=1)
        connection.grid_columnconfigure(1, weight=1)

        content = tk.Frame(self.root, bg=self.BG)
        content.pack(fill="both", expand=True, padx=18, pady=(0, 12))

        left = tk.Frame(content, bg=self.BG)
        left.pack(side="left", fill="both", expand=True, padx=(0, 7))
        right = tk.Frame(content, bg=self.BG)
        right.pack(side="right", fill="both", expand=True, padx=(7, 0))

        # Drive controls
        throttle_panel = self._panel(left, "THROTTLE")
        throttle_panel.pack(fill="both", expand=True)
        tk.Label(
            throttle_panel, text="FORWARD", bg=self.PANEL, fg=self.ACCENT,
            font=("Segoe UI", 9, "bold")
        ).pack(pady=(2, 0))
        self.throttle_scale = tk.Scale(
            throttle_panel, from_=100, to=-100, orient="vertical",
            variable=self.throttle_var, length=225, width=23,
            showvalue=False, resolution=1, bg=self.PANEL, fg=self.TEXT,
            troughcolor="#0d141b", highlightthickness=0, bd=0,
            activebackground=self.ACCENT, command=self._throttle_slider,
            state="disabled"
        )
        self.throttle_scale.pack()
        tk.Label(
            throttle_panel, text="REVERSE", bg=self.PANEL, fg=self.WARNING,
            font=("Segoe UI", 9, "bold")
        ).pack()
        self.throttle_readout = tk.StringVar(value="Output: 0%")
        tk.Label(
            throttle_panel, textvariable=self.throttle_readout,
            bg=self.PANEL, fg=self.TEXT, font=("Consolas", 10, "bold")
        ).pack(pady=(5, 11))

        steering_panel = self._panel(left, "STEERING")
        steering_panel.pack(fill="x", pady=(10, 0))
        steering_row = tk.Frame(steering_panel, bg=self.PANEL)
        steering_row.pack(fill="x", padx=12, pady=(2, 4))
        tk.Label(
            steering_row, text="LEFT", bg=self.PANEL, fg=self.MUTED,
            font=("Segoe UI", 8, "bold")
        ).pack(side="left")
        self.steering_scale = tk.Scale(
            steering_row, from_=-100, to=100, orient="horizontal",
            variable=self.steering_var, showvalue=False, resolution=1,
            bg=self.PANEL, fg=self.TEXT, troughcolor="#0d141b",
            highlightthickness=0, bd=0, activebackground=self.ACCENT,
            command=self._steering_slider, state="disabled"
        )
        self.steering_scale.pack(side="left", fill="x", expand=True, padx=8)
        tk.Label(
            steering_row, text="RIGHT", bg=self.PANEL, fg=self.MUTED,
            font=("Segoe UI", 8, "bold")
        ).pack(side="right")
        self.steering_readout = tk.StringVar(value="Output: 0%")
        tk.Label(
            steering_panel, textvariable=self.steering_readout,
            bg=self.PANEL, fg=self.TEXT, font=("Consolas", 10, "bold")
        ).pack(pady=(0, 9))

        # Speed control
        speed_panel = self._panel(right, "SPEED LIMIT")
        speed_panel.pack(fill="x")
        self.speed_var_label = tk.StringVar(value="1 / 127")
        tk.Label(
            speed_panel, textvariable=self.speed_var_label,
            bg=self.PANEL, fg=self.TEXT, font=("Consolas", 12, "bold")
        ).pack(anchor="e", padx=14)
        self.speed_scale = tk.Scale(
            speed_panel, from_=1, to=127, orient="horizontal",
            variable=self.speed_var, showvalue=False, bg=self.PANEL,
            fg=self.TEXT, troughcolor="#0d141b", highlightthickness=0,
            bd=0, activebackground=self.ACCENT, command=self._speed_changed,
            state="disabled"
        )
        self.speed_scale.pack(fill="x", padx=14, pady=(0, 10))

        # Light modes
        light_panel = self._panel(right, "LIGHT MODES")
        light_panel.pack(fill="x", pady=(10, 0))
        self.light_var = tk.StringVar(value="Constant ON")
        self.light_label = tk.Label(
            light_panel, textvariable=self.light_var, bg=self.PANEL,
            fg=self.ACCENT, font=("Segoe UI", 14, "bold")
        )
        self.light_label.pack(pady=(7, 10))
        light_buttons = tk.Frame(light_panel, bg=self.PANEL)
        light_buttons.pack(fill="x", padx=10, pady=(0, 8))
        for label, mode in (
            ("ON", LightMode.CONSTANT_ON),
            ("BLINK", LightMode.RAPID_BLINK),
            ("OFF", LightMode.OFF),
        ):
            ttk.Button(
                light_buttons, text=label,
                command=lambda selected=mode: self._select_light(selected)
            ).pack(side="left", expand=True, fill="x", padx=3)
        ttk.Button(
            light_panel, text="Next light mode",
            command=self._next_light
        ).pack(fill="x", padx=13, pady=(0, 11))

        # Curve settings
        curves = self._panel(right, "ACCELERATION / DECELERATION CURVES")
        curves.pack(fill="both", expand=True, pady=(10, 0))
        self._build_axis_curve_settings(
            curves, "THROTTLE",
            self.throttle_accel_var, self.throttle_decel_var,
            self.throttle_accel_curve_var, self.throttle_decel_curve_var,
            self._update_throttle_profile
        )
        self._build_axis_curve_settings(
            curves, "STEERING",
            self.steering_accel_var, self.steering_decel_var,
            self.steering_accel_curve_var, self.steering_decel_curve_var,
            self._update_steering_profile
        )

        # Keyboard and stop
        keyboard = self._panel(right, "KEYBOARD")
        keyboard.pack(fill="x", pady=(10, 0))
        tk.Label(
            keyboard,
            text="W / ↑  Forward     S / ↓  Reverse\n"
                 "A / ←  Left        D / →  Right\n"
                 "Release keys to return smoothly to centre",
            bg=self.PANEL, fg=self.TEXT, justify="left",
            font=("Consolas", 9), padx=13, pady=8
        ).pack(anchor="w")

        self.stop_button = ttk.Button(
            right, text="EMERGENCY STOP — NEUTRAL",
            style="Stop.TButton", command=self._emergency_stop,
            state="disabled"
        )
        self.stop_button.pack(fill="x", pady=(10, 0))

        footer = tk.Frame(self.root, bg=self.BG)
        footer.pack(fill="x", padx=18, pady=(0, 11))
        self.packet_status = tk.StringVar(value="BLE stream stopped")
        tk.Label(
            footer, textvariable=self.packet_status,
            bg=self.BG, fg=self.MUTED, font=("Consolas", 9)
        ).pack(side="left")
        tk.Label(
            footer, text="Drive mapping experimental • test with wheels raised",
            bg=self.BG, fg=self.MUTED, font=("Segoe UI", 8)
        ).pack(side="right")

    def _build_axis_curve_settings(
        self, parent, title, accel_var, decel_var,
        accel_curve_var, decel_curve_var, callback
    ):
        section = tk.Frame(parent, bg=self.PANEL)
        section.pack(fill="x", padx=12, pady=(3, 7))

        tk.Label(
            section, text=title, bg=self.PANEL, fg=self.TEXT,
            font=("Segoe UI", 9, "bold")
        ).grid(row=0, column=0, columnspan=3, sticky="w", pady=(2, 4))

        tk.Label(
            section, text="Acceleration time",
            bg=self.PANEL, fg=self.MUTED, font=("Segoe UI", 8)
        ).grid(row=1, column=0, sticky="w")
        accel_value = tk.Label(
            section, text=f"{accel_var.get():.2f}s",
            bg=self.PANEL, fg=self.TEXT, font=("Consolas", 8)
        )
        accel_value.grid(row=1, column=2, sticky="e")
        tk.Scale(
            section, from_=0.05, to=2.0, resolution=0.05,
            orient="horizontal", variable=accel_var, showvalue=False,
            bg=self.PANEL, fg=self.TEXT, troughcolor="#0d141b",
            highlightthickness=0, bd=0, activebackground=self.ACCENT,
            command=lambda _v: (
                accel_value.configure(text=f"{accel_var.get():.2f}s"),
                callback()
            )
        ).grid(row=2, column=0, columnspan=3, sticky="ew")

        tk.Label(
            section, text="Deceleration time",
            bg=self.PANEL, fg=self.MUTED, font=("Segoe UI", 8)
        ).grid(row=3, column=0, sticky="w")
        decel_value = tk.Label(
            section, text=f"{decel_var.get():.2f}s",
            bg=self.PANEL, fg=self.TEXT, font=("Consolas", 8)
        )
        decel_value.grid(row=3, column=2, sticky="e")
        tk.Scale(
            section, from_=0.05, to=2.0, resolution=0.05,
            orient="horizontal", variable=decel_var, showvalue=False,
            bg=self.PANEL, fg=self.TEXT, troughcolor="#0d141b",
            highlightthickness=0, bd=0, activebackground=self.ACCENT,
            command=lambda _v: (
                decel_value.configure(text=f"{decel_var.get():.2f}s"),
                callback()
            )
        ).grid(row=4, column=0, columnspan=3, sticky="ew")

        for row, label, variable in (
            (5, "Acceleration curve", accel_curve_var),
            (6, "Deceleration curve", decel_curve_var),
        ):
            tk.Label(
                section, text=label, bg=self.PANEL, fg=self.MUTED,
                font=("Segoe UI", 8)
            ).grid(row=row, column=0, sticky="w", pady=(3, 2))
            combo = ttk.Combobox(
                section, textvariable=variable, values=CURVE_NAMES,
                state="readonly", width=16
            )
            combo.grid(row=row, column=1, columnspan=2, sticky="e", pady=(3, 2))
            combo.bind("<<ComboboxSelected>>", lambda _e: callback())
        section.grid_columnconfigure(1, weight=1)

    def _bind_events(self):
        self.root.bind_all("<KeyPress>", self._key_down)
        self.root.bind_all("<KeyRelease>", self._key_up)
        self.root.bind("<FocusOut>", self._focus_lost)

        self.throttle_scale.bind("<ButtonPress-1>", lambda _e: self._slider_press("throttle"))
        self.throttle_scale.bind("<ButtonRelease-1>", self._slider_release_throttle)
        self.steering_scale.bind("<ButtonPress-1>", lambda _e: self._slider_press("steering"))
        self.steering_scale.bind("<ButtonRelease-1>", self._slider_release_steering)

    def _update_throttle_profile(self):
        self.throttle_axis.update_settings(
            accel_time=self.throttle_accel_var.get(),
            decel_time=self.throttle_decel_var.get(),
            accel_curve=self.throttle_accel_curve_var.get(),
            decel_curve=self.throttle_decel_curve_var.get(),
        )

    def _update_steering_profile(self):
        self.steering_axis.update_settings(
            accel_time=self.steering_accel_var.get(),
            decel_time=self.steering_decel_var.get(),
            accel_curve=self.steering_accel_curve_var.get(),
            decel_curve=self.steering_decel_curve_var.get(),
        )

    def _slider_press(self, axis):
        self.slider_dragging[axis] = True
        self.keyboard_active = False
        self.pressed_keys.clear()

    def _slider_release_throttle(self, _event=None):
        self.slider_dragging["throttle"] = False
        self.throttle_axis.set_target(0)
        return "break"

    def _slider_release_steering(self, _event=None):
        self.slider_dragging["steering"] = False
        self.steering_axis.set_target(0)
        return "break"

    def _throttle_slider(self, value):
        if not self.connected or self._updating_controls:
            return
        self.throttle_axis.set_target(int(float(value)))

    def _steering_slider(self, value):
        if not self.connected or self._updating_controls:
            return
        self.steering_axis.set_target(int(float(value)))

    def _speed_changed(self, value):
        amount = int(float(value))
        self.speed_var_label.set(f"{amount} / 127")
        if self.connected:
            self.controller.set_drive(speed=amount)

    def _scan_devices(self):
        if self.scanning or self.connected:
            return

        self.scanning = True
        self.scan_button.configure(state="disabled", text="Scanning...")
        self.packet_status.set("Scanning nearby Bluetooth LE devices...")
        self.device_choices.clear()

        def worker():
            try:
                devices = asyncio.run(BleakScanner.discover(timeout=6.0))
                found = []
                for device in devices:
                    address = getattr(device, "address", "") or ""
                    name = getattr(device, "name", None) or "Unknown device"
                    if address:
                        found.append((name, address))
                self.root.after(
                    0, lambda results=found: self._scan_finished(results, None)
                )
            except Exception as exc:
                error = str(exc)
                self.root.after(
                    0, lambda message=error: self._scan_finished([], message)
                )

        self.scan_thread = threading.Thread(target=worker, daemon=True)
        self.scan_thread.start()

    def _scan_finished(self, devices, error):
        self.scanning = False
        self.scan_button.configure(state="normal", text="Scan for cars")

        if error:
            self.packet_status.set("Bluetooth scan failed")
            messagebox.showerror("Bluetooth scan", error)
            return

        # Prioritise likely HB/LGMT names while retaining all discovered devices.
        devices.sort(key=lambda item: (
            0 if ("LGMT" in item[0].upper() or "HB" in item[0].upper()) else 1,
            item[0].lower()
        ))

        labels = []
        for name, address in devices:
            label = f"{name}  |  {address}"
            labels.append(label)
            self.device_choices[label] = address

        self.device_combo.configure(values=labels)
        if labels:
            self.device_combo.current(0)
            self._device_selected()
            self.packet_status.set(f"Found {len(labels)} BLE device(s)")
        else:
            self.device_var.set("No BLE devices found")
            self.packet_status.set(
                "No BLE devices found; enter the address manually"
            )

    def _device_selected(self, _event=None):
        address = self.device_choices.get(self.device_var.get())
        if address:
            self.address_var.set(address)

    def _get_selected_address(self):
        address = self.address_var.get().strip()
        if not address:
            raise ValueError("Enter or select a Bluetooth device address.")
        return address

    def _toggle_connection(self):
        if not self.connected:
            self.connect_button.configure(state="disabled")
            self.scan_button.configure(state="disabled")
            self.status_var.set("Connecting...")
            self.status_dot.configure(fg=self.WARNING)
            self.root.update_idletasks()
            try:
                self.controller.address = self._get_selected_address()
                self.controller.connect()
            except Exception as exc:
                self.connect_button.configure(state="normal")
                self.scan_button.configure(state="normal")
                self.status_var.set("Connection failed")
                self.status_dot.configure(fg=self.RED)
                messagebox.showerror("BLE connection failed", str(exc))
                return
            self.connected = True
            self.status_var.set("Connected — streaming")
            self.status_dot.configure(fg=self.ACCENT)
            self.connect_button.configure(text="Disconnect", state="normal")
            self.scan_button.configure(state="disabled")
            self.address_entry.configure(state="disabled")
            self.device_combo.configure(state="disabled")
            self.packet_status.set("Continuous BLE stream active (~23 ms)")
            for widget in (
                self.throttle_scale, self.steering_scale,
                self.speed_scale, self.stop_button
            ):
                widget.configure(state="normal")
            self._reset_motion()
        else:
            self._emergency_stop()
            try:
                self.controller.disconnect()
            except Exception as exc:
                messagebox.showwarning("Disconnect", str(exc))
            self.connected = False
            self.status_var.set("Disconnected")
            self.status_dot.configure(fg=self.MUTED)
            self.connect_button.configure(text="Connect", state="normal")
            self.scan_button.configure(state="normal")
            self.address_entry.configure(state="normal")
            self.device_combo.configure(state="readonly")
            self.packet_status.set("BLE stream stopped")
            for widget in (
                self.throttle_scale, self.steering_scale,
                self.speed_scale, self.stop_button
            ):
                widget.configure(state="disabled")
            self._reset_motion()

    def _select_light(self, mode):
        if not self.connected:
            return
        try:
            self.controller.set_light_mode(mode)
            self._refresh_light()
        except Exception as exc:
            messagebox.showerror("Light control", str(exc))

    def _next_light(self):
        if not self.connected:
            return
        try:
            self.controller.next_light_mode()
            self._refresh_light()
        except Exception as exc:
            messagebox.showerror("Light control", str(exc))

    def _refresh_light(self):
        mode = self.controller.current_light_mode
        self.light_var.set(LIGHT_MODE_NAMES[mode])
        colour = {
            LightMode.CONSTANT_ON: self.ACCENT,
            LightMode.RAPID_BLINK: self.WARNING,
            LightMode.OFF: self.MUTED,
        }[mode]
        self.light_label.configure(fg=colour)

    def _key_down(self, event):
        if not self.connected:
            return
        key = event.keysym.lower()
        if key == "space":
            self._emergency_stop()
            return
        if key in {"w", "a", "s", "d", "up", "down", "left", "right"}:
            self.keyboard_active = True
            self.pressed_keys.add(key)

    def _key_up(self, event):
        self.pressed_keys.discard(event.keysym.lower())

    def _focus_lost(self, _event=None):
        self.pressed_keys.clear()
        self.throttle_axis.set_target(0)
        self.steering_axis.set_target(0)

    def _emergency_stop(self):
        self.pressed_keys.clear()
        self.keyboard_active = False
        self.throttle_axis.value = 0.0
        self.throttle_axis.target = 0.0
        self.throttle_axis.start_value = 0.0
        self.throttle_axis.segment_duration = 0.0
        self.steering_axis.value = 0.0
        self.steering_axis.target = 0.0
        self.steering_axis.start_value = 0.0
        self.steering_axis.segment_duration = 0.0
        self.throttle_var.set(0)
        self.steering_var.set(0)
        self.throttle_readout.set("Output: 0%")
        self.steering_readout.set("Output: 0%")
        if self.connected:
            self.controller.stop_drive()

    def _reset_motion(self):
        self._emergency_stop()
        self.throttle_axis = MotionAxis(
            self.throttle_accel_var.get(),
            self.throttle_decel_var.get(),
            self.throttle_accel_curve_var.get(),
            self.throttle_decel_curve_var.get(),
        )
        self.steering_axis = MotionAxis(
            self.steering_accel_var.get(),
            self.steering_decel_var.get(),
            self.steering_accel_curve_var.get(),
            self.steering_decel_curve_var.get(),
        )

    def _motion_tick(self):
        if self.closing:
            return

        if self.connected:
            if self.keyboard_active:
                keys = self.pressed_keys
                forward = bool({"w", "up"} & keys)
                reverse = bool({"s", "down"} & keys)
                left = bool({"a", "left"} & keys)
                right = bool({"d", "right"} & keys)

                throttle_target = 100 if forward and not reverse else (
                    -100 if reverse and not forward else 0
                )
                steering_target = -100 if left and not right else (
                    100 if right and not left else 0
                )
                if not self.slider_dragging["throttle"]:
                    self.throttle_axis.set_target(throttle_target)
                if not self.slider_dragging["steering"]:
                    self.steering_axis.set_target(steering_target)

            throttle_out = int(round(self.throttle_axis.current_value()))
            steering_out = int(round(self.steering_axis.current_value()))

            self.throttle_readout.set(f"Output: {throttle_out:+d}%")
            self.steering_readout.set(f"Output: {steering_out:+d}%")

            self._updating_controls = True
            try:
                if not self.slider_dragging["throttle"]:
                    self.throttle_var.set(throttle_out)
                if not self.slider_dragging["steering"]:
                    self.steering_var.set(steering_out)
            finally:
                self._updating_controls = False

            current = (throttle_out, steering_out)
            if current != self.last_sent:
                self.controller.set_drive(
                    throttle=throttle_out, steering=steering_out
                )
                self.last_sent = current

        self.root.after(self.UPDATE_MS, self._motion_tick)

    def _on_close(self):
        self.closing = True
        try:
            self.controller.close()
        except Exception:
            pass
        self.root.destroy()


def main():
    root = tk.Tk()
    HBRCTkUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
