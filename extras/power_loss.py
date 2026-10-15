# power_loss.py
#
# Hardware-backed power loss recovery for the Elegoo Centauri Carbon
# running COSMOS. Writes state to disk via direct os.write()+fsync(), bypassing
# high-level buffers, to minimize the time between "outage detected" and
# "file safely on disk".
#
# Based on reverse engineering of:
#   - elegoo/extras/por.cpp (CentauriCarbon2, Elegoo, GPL-3.0)
#   - elegoo/extras/virtual_sdcard.cpp (CentauriCarbon2, Elegoo, GPL-3.0)
#   - firmware/core/klippy/extras/break_save.cpp + firmware/devices/gpio.h
#     (CentauriCarbon, Elegoo, GPL-3.0) -- source of these pin numbers:
#       BREAK_DETECTION_GPIO = PG0  -> Linux GPIO 192
#       BREAK_CONTROL_GPIO   = PG11 -> Linux GPIO 203  (NOT used here)
#       CC_EN_GPIO           = PC1  -> Linux GPIO 65
# Also borrows one key idea from the community project KlipperPLR
# (github.com/bigtreetech/KlipperPLR, forked from Yumi-Lab/Yumi_PLR):
# deriving the extrude mode (M82/M83) and last E value by replaying the
# actual gcode file up to the resume point, instead of trusting
# Klipper's live gcode_move status -- see _derive_extrude_state_from_file.
#
# printer.cfg usage:
#
#   [power_loss]
#   sensor_gpio: 192
#   charge_gpio: 65
#   poll_interval: 0.01
#   charge_time: 30
#   save_path: /data/power_loss_state.json
#   z_hop: 10.0        ; safety clearance before homing X/Y on resume
#   travel_speed: 3000 ; mm/min for hop/travel moves on resume
#   homing_override_z_lift: 5.0  ; must match the "G0 Z<N>" in this
#                                 ; printer's [homing_override] macro
#   purge_mm: 15.0     ; purged at the tray before resuming, to clear
#                       ; ooze from sitting hot/stationary
#   move_temp: 200.0   ; nozzle-only threshold before ANY movement
#                       ; (bed and full target keep heating in the
#                       ; background while homing/traveling)
#   #invert_sensor: False   ; uncomment if polarity turns out inverted
#
# On resume, two options are offered (via prompt or manually):
#   POWER_LOSS_RESUME       -- precise instant-of-outage position
#   POWER_LOSS_RESUME_SAFE  -- last completed layer (repeats part of a
#                              layer, but avoids resuming mid-move,
#                              e.g. mid-purge/wipe -- default/primary
#                              choice in the prompt)
# Both accept optional overrides:
#   FILENAME=<file.gcode>   -- if the saved file path came back empty
#   EXTRUDER_TEMP=<C> BED_TEMP=<C> -- force the resume temperatures.
#
# Resume temperatures come from the gcode file itself: the last
# PRINT_START EXTRUDER=/BED= or M104/M109/M140/M190 the slicer wrote
# before the resume point. The heater targets saved at the outage are
# only a fallback if the file can't be read.
#
# DELIBERATELY never declared here: BREAK_CONTROL_GPIO (203). That pin
# enables the supercap's OUTPUT to the power rail -- touching it by
# software without first confirming with a multimeter what's on the other
# side is the experiment we decided NOT to automate blindly.
#
# CONFIRMED WITH A REAL POWER CUT (mains fully unplugged): with the
# supercap charged, the SoC/logic stayed alive for 5-15 seconds with the
# touchscreen still live-updating and responsive to touch. Motors/heaters
# were NOT confirmed to have any residual power -- treat that part as
# still unknown.

import os
import re
import json
import time
import fcntl
import struct
import logging

try:
    import gpiod
    HAVE_GPIOD = True
except ImportError:
    HAVE_GPIOD = False

GPIO_SYSFS = "/sys/class/gpio"
GPIO_CHIP_NAME = "gpiochip0"
GPIO_CHIP_PATH = "/dev/" + GPIO_CHIP_NAME
GCODES_DIR = "/user-resource/gcodes"

# ---- raw GPIO character-device ioctl fallback (no gpiod dependency) ----
# Confirmed working on the Centauri Carbon 1 kernel -- targets the v1 GPIO uAPI
# (struct gpioevent_request / GPIO_GET_LINEEVENT_IOCTL from
# <linux/gpio.h>). Newer kernels (~5.10+) may only support the v2 API
# instead, which uses a different struct/ioctl entirely and is NOT
# implemented here. If this fails, _setup_ioctl_event() catches it and
# the module falls back to polling -- it never leaves detection with
# no working path.
_GPIOHANDLE_REQUEST_INPUT = 1 << 0
_GPIOEVENT_REQUEST_RISING_EDGE = 1 << 0
_GPIOEVENT_REQUEST_FALLING_EDGE = 1 << 1
_GPIOEVENT_REQUEST_BOTH_EDGES = (_GPIOEVENT_REQUEST_RISING_EDGE
                                  | _GPIOEVENT_REQUEST_FALLING_EDGE)
_GPIOEVENT_EVENT_RISING_EDGE = 0x01
_GPIOEVENT_EVENT_FALLING_EDGE = 0x02

# struct gpioevent_request { u32 lineoffset; u32 handleflags;
#   u32 eventflags; char consumer_label[32]; int fd; } -- 48 bytes
_GPIOEVENT_REQUEST_FMT = "<3I32si"
_GPIOEVENT_REQUEST_SIZE = struct.calcsize(_GPIOEVENT_REQUEST_FMT)

# struct gpioevent_data { u64 timestamp; u32 id; } -- kernel pads to 16
_GPIOEVENT_DATA_FMT = "<QI"
_GPIOEVENT_DATA_READ_SIZE = 16


def _ioc(direction, ioc_type, nr, size):
    # Mirrors the standard Linux _IOC()/_IOWR() macro layout
    # (dir<<30 | size<<16 | type<<8 | nr) used across architectures
    # that matter here (ARM included).
    return (direction << 30) | (size << 16) | (ioc_type << 8) | nr


_IOC_WRITE = 1
_IOC_READ = 2
_GPIO_GET_LINEEVENT_IOCTL = _ioc(_IOC_READ | _IOC_WRITE, 0xB4, 0x04,
                                  _GPIOEVENT_REQUEST_SIZE)


class SysfsGpio:
    """Minimal sysfs GPIO wrapper. Does not use Klipper's pin system
    because Klipper only knows how to talk to pins declared under an
    [mcu] -- these pins live on the host (SoC), not on the serial MCU."""

    def __init__(self, line):
        self.line = line
        self.path = "%s/gpio%d" % (GPIO_SYSFS, line)
        if not os.path.exists(self.path):
            try:
                with open("%s/export" % GPIO_SYSFS, "w") as f:
                    f.write(str(line))
            except OSError as e:
                # EBUSY = already exported by another process; not fatal
                if e.errno != 16:
                    raise

    def set_direction(self, direction):
        with open("%s/direction" % self.path, "w") as f:
            f.write(direction)

    def read(self):
        with open("%s/value" % self.path, "r") as f:
            return int(f.read().strip())

    def write(self, value):
        with open("%s/value" % self.path, "w") as f:
            f.write("1" if value else "0")


class PowerLoss:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")

        self.sensor_line = config.getint("sensor_gpio", 192)
        self.charge_line = config.getint("charge_gpio", 65)
        self.poll_interval = config.getfloat("poll_interval", 0.01, above=0.0)
        self.charge_time = config.getfloat("charge_time", 30.0, minval=0.0)
        self.invert_sensor = config.getboolean("invert_sensor", False)
        self.z_hop = config.getfloat("z_hop", 10.0, minval=0.0)
        self.travel_speed = config.getfloat("travel_speed", 3000.0, above=0.0)
        # This printer's [homing_override] macro forces set_position_z=0
        # on every G28 call, then does its own real physical "G0 Z5"
        # lift before homing X/Y -- silently invalidating whatever Z we
        # declared beforehand. After G28 X Y returns, we re-declare Z
        # accounting for that extra known lift, instead of trusting
        # Klipper's own (now wrong) idea of where Z is.
        # FRAGILE: this hardcodes knowledge of that specific macro's
        # "G0 Z5" constant. If COSMOS/the user ever changes that macro,
        # this offset needs to change too.
        self.homing_override_z_lift = config.getfloat(
            "homing_override_z_lift", 5.0
        )
        self.purge_mm = config.getfloat("purge_mm", 15.0, minval=0.0)
        self.move_temp = config.getfloat("move_temp", 200.0, minval=0.0)
        self.save_path = config.get(
            "save_path", "/data/power_loss_state.json"
        )

        self.charge_pin = SysfsGpio(self.charge_line)
        self.charge_pin.set_direction("out")
        self.charge_pin.write(0)  # starts off; enabled in handle_ready

        self.charge_done = False
        self.saved_already = False
        self.start_time = None
        self.read_history = []  # ring buffer, filled in _poll (non-blocking)
        self.read_history_max = 200
        self.prompt_attempts_left = 0
        self.last_checkpoint = None  # updated on every layer change
        self.last_seen_layer = None

        # Three-tier edge-event detection, each falling back to the
        # next if unavailable/failing: gpiod bindings -> raw ioctl on
        # the gpio character device -> polling (the always-available,
        # already-validated-with-real-hardware baseline in _poll).
        # Real interrupt-driven detection means lower latency and the
        # reactor stays idle (via register_fd) instead of us asking
        # "did it change?" every poll_interval. On the Centauri Carbon 1
        # there are no gpiod bindings installed, and the raw ioctl tier
        # (v1 GPIO uAPI) is the one confirmed working. A log line always
        # says which tier ended up active, including plain "polling" if
        # both fail.
        #
        # IMPORTANT: this must run BEFORE any sysfs export of the
        # sensor line. A GPIO line has a single exclusive owner in the
        # kernel -- if sysfs (the polling fallback) claims it first,
        # the character-device ioctl below will always fail with
        # EBUSY, regardless of whether the kernel actually supports it
        # (confirmed the hard way: our own earlier code exported the
        # line unconditionally in __init__, so the ioctl tier could
        # never succeed even when the kernel understood it fine).
        self._gpiod_line = None  # only set/used for the gpiod tier
        self.event_fd = None
        self.detection_method = "polling"

        if HAVE_GPIOD:
            self.event_fd = self._setup_gpiod_event()
            if self.event_fd is not None:
                self.detection_method = "gpiod"

        if self.event_fd is None:
            self.event_fd = self._setup_ioctl_event()
            if self.event_fd is not None:
                self.detection_method = "ioctl"

        if self.event_fd is not None:
            self.reactor.register_fd(self.event_fd, self._handle_gpio_event)
            logging.info("power_loss: edge-event detection active via "
                          "%s (fd=%d)", self.detection_method,
                          self.event_fd)
            self.sensor_pin = None
        else:
            logging.info("power_loss: no edge-event method available -- "
                          "using polling for outage detection")
            self.sensor_pin = SysfsGpio(self.sensor_line)
            self.sensor_pin.set_direction("in")

        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler(
            "klippy:disconnect", self._handle_disconnect
        )

        self.gcode.register_command(
            "POWER_LOSS_STATUS", self.cmd_POWER_LOSS_STATUS,
            desc="Shows the current status of the power loss monitor",
        )
        self.gcode.register_command(
            "POWER_LOSS_RESUME", self.cmd_POWER_LOSS_RESUME,
            desc="Resumes a print interrupted by a power outage, using "
                 "the precise instant-of-outage position",
        )
        self.gcode.register_command(
            "POWER_LOSS_RESUME_SAFE", self.cmd_POWER_LOSS_RESUME_SAFE,
            desc="Resumes from the last completed layer instead of the "
                 "precise instant-of-outage position -- repeats part of "
                 "a layer but avoids resuming mid-move artifacts",
        )
        self.gcode.register_command(
            "POWER_LOSS_DISCARD", self.cmd_POWER_LOSS_DISCARD,
            desc="Discards the saved state without resuming",
        )
        self.gcode.register_command(
            "POWER_LOSS_TEST_READ", self.cmd_POWER_LOSS_TEST_READ,
            desc="Shows the latest pin readings already collected by the "
                 "normal poll timer -- non-blocking, safe to run while "
                 "printing",
        )

        logging.info("power_loss: init OK (sensor=GPIO%d charge=GPIO%d)",
                      self.sensor_line, self.charge_line)

    # ---- edge-event detection setup (tiers 1 and 2) ----

    def _setup_gpiod_event(self):
        """Tier 1: gpiod Python bindings. Returns an fd or None."""
        try:
            chip = gpiod.Chip(GPIO_CHIP_NAME)
            line = chip.get_line(self.sensor_line)
            edge = (gpiod.LINE_REQ_EV_FALLING_EDGE if self.invert_sensor
                    else gpiod.LINE_REQ_EV_RISING_EDGE)
            line.request(consumer="power_loss", type=edge)
            self._gpiod_line = line
            return line.event_get_fd()
        except Exception as e:
            logging.warning("power_loss: gpiod edge-event setup failed "
                             "(%s)", e)
            self._gpiod_line = None
            return None

    def _setup_ioctl_event(self):
        """Tier 2: raw ioctl on the gpio character device, no external
        dependency. v1 GPIO uAPI only -- see the module-level comment.
        Returns an fd or None."""
        try:
            chip_fd = os.open(GPIO_CHIP_PATH, os.O_RDONLY)
        except OSError as e:
            logging.warning("power_loss: couldn't open %s (%s)",
                             GPIO_CHIP_PATH, e)
            return None

        try:
            consumer = b"power_loss"
            consumer = consumer + b"\x00" * (32 - len(consumer))
            req = struct.pack(
                _GPIOEVENT_REQUEST_FMT,
                self.sensor_line,
                _GPIOHANDLE_REQUEST_INPUT,
                _GPIOEVENT_REQUEST_BOTH_EDGES,
                consumer,
                0,
            )
            result = fcntl.ioctl(chip_fd, _GPIO_GET_LINEEVENT_IOCTL, req)
            # last field of the struct is the kernel-assigned line fd
            line_fd = struct.unpack(_GPIOEVENT_REQUEST_FMT, result)[-1]
            return line_fd
        except Exception as e:
            logging.warning("power_loss: raw ioctl edge-event setup "
                             "failed (%s) -- this kernel may only "
                             "support the newer GPIO v2 uAPI, which "
                             "isn't implemented here", e)
            return None
        finally:
            os.close(chip_fd)

    # ---- lifecycle ----

    def _handle_ready(self):
        self.charge_pin.write(1)
        self.start_time = self.reactor.monotonic()
        self.timer = self.reactor.register_timer(
            self._poll, self.reactor.NOW
        )
        logging.info("power_loss: polling started, charging supercap "
                      "(%.0fs)", self.charge_time)

        if os.path.exists(self.save_path):
            # No clean API for an extra to know "a UI client is
            # connected" -- a single fixed delay is a guess that can
            # still miss the window (confirmed: Moonraker was still
            # registering its webhooks connection 5s after ready in one
            # test). Instead, resend the (idempotent) prompt a few times
            # over the first ~20s -- one of the attempts is very likely
            # to land once Mainsail/Moonraker are actually connected.
            self.prompt_attempts_left = 6
            self.reactor.register_timer(
                self._show_resume_prompt,
                self.reactor.monotonic() + 2.0,
            )

    def _show_resume_prompt(self, eventtime):
        if self.prompt_attempts_left <= 0 or not os.path.exists(self.save_path):
            return self.reactor.NEVER

        self.prompt_attempts_left -= 1
        logging.info("power_loss: showing pending resume prompt "
                      "(%d attempts left after this one)",
                      self.prompt_attempts_left)
        try:
            self.gcode.run_script_from_command(
                'RESPOND TYPE=command MSG="action:prompt_begin '
                'Interrupted Print"'
            )
            self.gcode.run_script_from_command(
                'RESPOND TYPE=command MSG="action:prompt_text A power '
                'outage was detected during a print. Resume precisely, '
                'resume from the last completed layer (safer, repeats '
                'part of a layer), or discard?"'
            )
            self.gcode.run_script_from_command(
                'RESPOND TYPE=command MSG="action:prompt_footer_button '
                'Resume Precise|POWER_LOSS_RESUME"'
            )
            self.gcode.run_script_from_command(
                'RESPOND TYPE=command MSG="action:prompt_footer_button '
                'Resume Safe|POWER_LOSS_RESUME_SAFE|primary"'
            )
            self.gcode.run_script_from_command(
                'RESPOND TYPE=command MSG="action:prompt_footer_button '
                'Discard|POWER_LOSS_DISCARD"'
            )
            self.gcode.run_script_from_command(
                'RESPOND TYPE=command MSG="action:prompt_show"'
            )
        except Exception as e:
            logging.error("power_loss: failed to show resume prompt: %s", e)

        if self.prompt_attempts_left <= 0:
            return self.reactor.NEVER
        return eventtime + 3.0

    def _handle_disconnect(self):
        # Leave everything in a safe state if Klipper shuts down for
        # some other reason
        try:
            self.charge_pin.write(0)
        except Exception:
            pass

        if self.event_fd is not None:
            try:
                self.reactor.unregister_fd(self.event_fd)
            except Exception:
                pass
            if self.detection_method == "gpiod" and self._gpiod_line is not None:
                try:
                    self._gpiod_line.release()
                except Exception:
                    pass
                self._gpiod_line = None
            elif self.detection_method == "ioctl":
                try:
                    os.close(self.event_fd)
                except Exception:
                    pass
            self.event_fd = None

    def _poll(self, eventtime):
        if not self.charge_done and self.start_time is not None:
            if eventtime - self.start_time > self.charge_time:
                self.charge_done = True
                logging.info("power_loss: supercap charge complete")

        self._check_layer_change()

        # Only reads/checks the pin ourselves when polling is the
        # active detection method -- when an edge-event tier owns the
        # line (gpiod/ioctl), self.sensor_pin is None (the sysfs export
        # was skipped on purpose, see __init__: a line can only have
        # one exclusive owner in the kernel) and read_history gets
        # populated from actual events instead, in _handle_gpio_event.
        if self.sensor_pin is not None:
            try:
                raw = self.sensor_pin.read()
            except OSError as e:
                # Don't let a transient sysfs read failure (e.g. the
                # line got unexported from under us by something else)
                # take down the whole timer/Klipper -- skip this cycle
                # and try again next poll.
                logging.error("power_loss: sensor read failed: %s", e)
                return eventtime + self.poll_interval
            self.read_history.append(raw)
            if len(self.read_history) > self.read_history_max:
                self.read_history.pop(0)

            detected = bool(raw) != self.invert_sensor
            # NOTE: no debounce/jitter yet (Elegoo's por.cpp has one,
            # with a configurable "jitter_time"). In manual SSH tests
            # the pin came back stable with no noise, but that was one
            # sample at a time -- before trusting this in production
            # it would be worth porting that debounce.
            if detected and not self.saved_already:
                self._on_power_loss()

        return eventtime + self.poll_interval

    def _handle_gpio_event(self, eventtime):
        raw = None
        try:
            if self.detection_method == "gpiod":
                event = self._gpiod_line.event_read()
                raw = 1 if event.type == gpiod.LineEvent.RISING_EDGE else 0
            elif self.detection_method == "ioctl":
                data = os.read(self.event_fd, _GPIOEVENT_DATA_READ_SIZE)
                _, ev_id = struct.unpack(_GPIOEVENT_DATA_FMT, data[:12])
                raw = 1 if ev_id == _GPIOEVENT_EVENT_RISING_EDGE else 0
        except Exception as e:
            logging.error("power_loss: failed to read gpio event: %s", e)
            return

        if raw is not None:
            self.read_history.append(raw)
            if len(self.read_history) > self.read_history_max:
                self.read_history.pop(0)

            detected = bool(raw) != self.invert_sensor
            if detected and not self.saved_already:
                self._on_power_loss()

    # ---- the part that actually matters ----

    def _derive_extrude_state_from_file(self, file_path, file_position):
        """Ground truth for extrude mode (M82/M83) and last E value,
        derived directly from the file's own commands instead of
        trusting Klipper's live gcode_move status (which is exactly
        where we had a real bug, confusing absolute_coordinates with
        absolute_extrude). Borrowed from the technique used by the
        community project KlipperPLR (bigtreetech/KlipperPLR).

        IMPORTANT: bounded reads only, on purpose. An earlier version
        of this function read the ENTIRE file from byte 0 up to
        file_position in a single synchronous Python loop -- on a
        large file at a large offset (confirmed: 32MB file, ~26MB
        offset) that blocked the reactor long enough for the MCU to
        starve and shut down with "Timer too close", crashing an
        in-progress real recovery. Never remove these bounds without
        making the scan yield to the reactor periodically instead.

        Two bounded reads instead: the file's HEADER (mode is almost
        always declared once, near the top, by the slicer) for
        absolute_extrude, and a small WINDOW immediately before
        file_position for the last E value.

        Returns (last_e, absolute_extrude), either of which may be
        None if the file couldn't be read or nothing was found."""
        if not file_path or file_position is None:
            return None, None

        HEADER_BYTES = 65536
        WINDOW_BYTES = 200000
        e_re = re.compile(r'E(-?[0-9.]+)', re.IGNORECASE)

        absolute_extrude = True  # G-code spec default
        last_e = None

        def _scan_lines(text, track_mode, track_e):
            nonlocal absolute_extrude, last_e
            for line in text.splitlines():
                stripped = line.strip()
                if not stripped or stripped.startswith(";"):
                    continue
                upper = stripped.upper()
                if track_mode and upper.startswith("M83"):
                    absolute_extrude = False
                elif track_mode and upper.startswith("M82"):
                    absolute_extrude = True
                elif track_e and (upper.startswith("G0")
                                   or upper.startswith("G1")
                                   or upper.startswith("G92")):
                    m = e_re.search(upper)
                    if m:
                        if upper.startswith("G92"):
                            last_e = float(m.group(1))
                        elif absolute_extrude:
                            last_e = float(m.group(1))
                        # relative mode: deltas aren't meaningful to
                        # accumulate here -- see _do_resume, which
                        # just zeroes E instead

        try:
            with open(file_path, "r", errors="ignore") as f:
                # Pass 1: header, for the mode declaration.
                header = f.read(HEADER_BYTES)
                _scan_lines(header, track_mode=True, track_e=False)

                # Pass 2: a small window right before the resume
                # point, for the last E value and any mode change
                # that isn't caught by the header (rare, but a
                # slicer-inserted M83 mid-file would only show up
                # here).
                window_start = max(HEADER_BYTES, file_position - WINDOW_BYTES)
                f.seek(window_start)
                window = f.read(min(WINDOW_BYTES, file_position - window_start))
                _scan_lines(window, track_mode=True, track_e=True)
        except OSError as e:
            logging.error("power_loss: failed to read gcode file for "
                           "extrude state: %s", e)
            return None, None

        return last_e, absolute_extrude

    # Temperature commands the slicer wrote into the file. COSMOS's slicer
    # profile starts prints with "PRINT_START EXTRUDER=<t> BED=<t>"; mid-
    # print temperature changes (e.g. first layer -> other layers) come as
    # plain M104/M109/M140/M190. Matched only at the start of a line, so
    # commented-out lines (";M104 ...") are ignored.
    _TEMP_CMD_RE = re.compile(
        rb'^[ \t]*(M104|M109|M140|M190)\b[^\n;]*?\b[SR](\d+(?:\.\d+)?)',
        re.IGNORECASE | re.MULTILINE)
    _PRINT_START_RE = re.compile(
        rb'^[ \t]*(?:PRINT_START|START_PRINT)\b([^\n;]*)',
        re.IGNORECASE | re.MULTILINE)
    _PS_EXTRUDER_RE = re.compile(
        rb'\b(?:EXTRUDER|EXTRUDER_TEMP|HOTEND|HOTEND_TEMP|NOZZLE|NOZZLE_TEMP)'
        rb'=(\d+(?:\.\d+)?)', re.IGNORECASE)
    _PS_BED_RE = re.compile(rb'\b(?:BED|BED_TEMP)=(\d+(?:\.\d+)?)',
                            re.IGNORECASE)

    def _derive_temps_from_file(self, file_path, file_position):
        """The temperatures the slicer had set for this print at
        file_position: the last PRINT_START / M104 / M109 / M140 / M190
        found in the file before that point. Zero values (heater-off
        commands) are ignored.

        Unlike _derive_extrude_state_from_file this reads the WHOLE file
        up to file_position, because a temperature change can be
        anywhere (typically right after the first layer). It's done in
        chunks with a C-level regex, and yields to the reactor between
        chunks via reactor.pause() -- a single blocking pass over a large
        file is exactly what once starved the MCU ("Timer too close").

        Returns (extruder_temp, bed_temp); either may be None."""
        if not file_path or file_position is None:
            return None, None

        CHUNK_BYTES = 262144
        extruder = None
        bed = None

        def scan(buf):
            nonlocal extruder, bed
            # Collect (offset, kind, value) so commands of both forms are
            # applied in file order -- the last one wins.
            hits = []
            for m in self._TEMP_CMD_RE.finditer(buf):
                hits.append((m.start(), m.group(1).upper(),
                             float(m.group(2))))
            for m in self._PRINT_START_RE.finditer(buf):
                args = m.group(1)
                e = self._PS_EXTRUDER_RE.search(args)
                b = self._PS_BED_RE.search(args)
                if e:
                    hits.append((m.start(), b"M104", float(e.group(1))))
                if b:
                    hits.append((m.start(), b"M140", float(b.group(1))))
            hits.sort(key=lambda h: h[0])
            for _, cmd, value in hits:
                if value <= 0.0:
                    continue
                if cmd in (b"M104", b"M109"):
                    extruder = value
                else:
                    bed = value

        try:
            with open(file_path, "rb") as f:
                remaining = file_position
                carry = b""
                while remaining > 0:
                    data = f.read(min(CHUNK_BYTES, remaining))
                    if not data:
                        break
                    remaining -= len(data)
                    data = carry + data
                    if remaining > 0:
                        # Keep the partial last line for the next chunk.
                        cut = data.rfind(b"\n") + 1
                        carry = data[cut:]
                        data = data[:cut]
                    scan(data)
                    self.reactor.pause(self.reactor.monotonic() + 0.001)
        except OSError as e:
            logging.error("power_loss: failed to read gcode file for "
                           "temperatures: %s", e)
            return None, None

        return extruder, bed

    def _check_layer_change(self):
        try:
            print_stats = self.printer.lookup_object("print_stats")
            info = print_stats.get_status(self.reactor.monotonic()).get(
                "info", {}
            )
            current_layer = info.get("current_layer")
        except Exception:
            return

        if current_layer is None or current_layer == self.last_seen_layer:
            return

        self.last_seen_layer = current_layer
        snapshot = self._snapshot_position()
        if snapshot is not None:
            snapshot["layer"] = current_layer
            # Print temperatures as of this layer. Second source for the
            # resume, in case the top-level targets captured at the
            # outage are unusable (e.g. the outage hit during a PAUSE,
            # which lowers the hotend to its probing temperature).
            snapshot.update(self._read_heater_targets())
            self.last_checkpoint = snapshot

    def _read_heater_targets(self):
        """Current *target* temperatures, keyed 'extruder_target' and
        'heater_bed_target'. Missing heaters are simply omitted."""
        targets = {}
        try:
            heaters = self.printer.lookup_object("heaters")
        except Exception:
            return targets
        now = self.reactor.monotonic()
        for name in ("extruder", "heater_bed"):
            try:
                targets[name + "_target"] = (
                    heaters.lookup_heater(name).get_temp(now)[1])
            except Exception:
                pass
        return targets

    def _snapshot_position(self):
        """Cheap position/file/speed snapshot, no timestamp/temps --
        used both for the instant-of-outage capture and for the
        per-layer checkpoint."""
        snap = {}
        try:
            toolhead = self.printer.lookup_object("toolhead")
            snap["gcode_position"] = list(toolhead.get_position())
        except Exception:
            return None

        try:
            gcode_move = self.printer.lookup_object("gcode_move")
            gm_status = gcode_move.get_status(self.reactor.monotonic())
            snap["speed_factor"] = gm_status.get("speed_factor")
            snap["absolute_extrude"] = gm_status.get("absolute_extrude")
        except Exception:
            pass

        try:
            virtual_sdcard = self.printer.lookup_object("virtual_sdcard")
            vsd_status = virtual_sdcard.get_status(self.reactor.monotonic())
            snap["file_path"] = vsd_status.get("file_path")
            snap["file_position"] = virtual_sdcard.file_position
        except Exception:
            return None

        return snap

    def _on_power_loss(self):
        self.saved_already = True
        logging.warning("power_loss: OUTAGE DETECTED -- saving state")

        # Electrically, a manual power-off looks identical to a real
        # outage -- the sensor can't tell them apart. But if nothing
        # was actually printing, there's nothing meaningful to resume;
        # writing the state file anyway just produces a confusing
        # "interrupted print" prompt on the next boot for what was a
        # normal, intentional shutdown. Only save state if a print was
        # genuinely active.
        printing = False
        ps_state = None
        try:
            print_stats = self.printer.lookup_object("print_stats")
            ps_state = print_stats.get_status(self.reactor.monotonic()).get("state")
            printing = ps_state in ("printing", "paused")
        except Exception as e:
            logging.error("power_loss: failed to check print_stats state: "
                           "%s -- assuming a print WAS active, to be safe",
                           e)
            printing = True

        # State MUST be captured BEFORE TURN_OFF_HEATERS. _collect_state
        # reads each heater's *target*, and TURN_OFF_HEATERS sets those
        # targets to 0 immediately -- capturing afterwards saved
        # extruder_target=0 / heater_bed_target=0, so the resume skipped
        # all heating and moved/extruded cold (real bug: resume hopped Z
        # with a cold nozzle and bed, then failed on the purge with
        # "Extrude below minimum temp"). Capture is in-memory only (no
        # I/O), so this costs essentially nothing before heaters go off.
        state = self._collect_state() if printing else None

        # Heaters off regardless of whether a print is active -- pure
        # safety, costs nothing to do even if nothing was printing.
        try:
            self.gcode.run_script_from_command("TURN_OFF_HEATERS")
        except Exception as e:
            logging.error("power_loss: failed to turn off heaters: %s", e)

        if not printing:
            logging.info("power_loss: no active print (state=%s) -- "
                          "skipping state save, nothing to resume", ps_state)
            self._protect_filesystems()
            return

        try:
            fd = os.open(
                self.save_path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644
            )
            os.write(fd, json.dumps(state).encode("utf-8"))
            os.fsync(fd)
            os.close(fd)
        except OSError as e:
            logging.error("power_loss: failed to write state: %s", e)
            return

        # Ordered filesystem protection. Confirmed with real-world
        # testing: there are 5-15s of backup power (with EVERYTHING
        # unplugged from the wall) for the SoC/logic -- enough to flush
        # and remount filesystems read-only, instead of letting a real
        # outage kill everything mid-write (that's how a FAT partition
        # got corrupted in an earlier test without this).
        #
        # Deliberately NOT a full "shutdown -h now": that leaves the
        # board needing a physical unplug/replug to boot again on many
        # embedded PMICs (confirmed on this printer), which defeats the
        # point if power comes back while nobody's around. Instead:
        # sync, then remount every writable ext4/vfat mount read-only.
        # Klipper itself keeps running; whatever happens next (power
        # returns, or the board eventually dies) the filesystems are
        # already safe.
        logging.warning("power_loss: protecting filesystems (sync + "
                         "remount read-only)")
        self._protect_filesystems()

    def _protect_filesystems(self):
        try:
            os.system("sync")
        except Exception as e:
            logging.error("power_loss: sync failed: %s", e)

        try:
            with open("/proc/mounts", "r") as f:
                mounts = f.readlines()
        except OSError as e:
            logging.error("power_loss: failed to read /proc/mounts: %s", e)
            return

        for line in mounts:
            parts = line.split()
            if len(parts) < 4:
                continue
            _, mountpoint, fstype, options = parts[0:4]
            if fstype not in ("ext2", "ext3", "ext4", "vfat", "fat", "exfat"):
                continue
            if "rw" not in options.split(","):
                continue  # already read-only, nothing to do
            try:
                os.system('mount -o remount,ro "%s"' % mountpoint)
                logging.warning("power_loss: remounted %s (%s) read-only",
                                 mountpoint, fstype)
            except Exception as e:
                logging.error("power_loss: failed to remount %s: %s",
                               mountpoint, e)

    def _collect_state(self):
        """Top-level fields (temps, timing) plus two position packages:
        'instant' (captured right now, at the moment of the outage) and
        'checkpoint' (the last layer-change snapshot, if any -- see
        _check_layer_change). Each package's fields must be used
        together, never mixed between packages: position and
        file_position only make sense paired from the same instant."""
        state = {"timestamp": time.time()}
        state.update(self._read_heater_targets())

        try:
            print_stats = self.printer.lookup_object("print_stats")
            ps_status = print_stats.get_status(self.reactor.monotonic())
            state["print_duration"] = ps_status.get("print_duration")
        except Exception:
            pass

        try:
            fan = self.printer.lookup_object("fan")
            state["fan_speed"] = fan.get_status(self.reactor.monotonic()).get("speed")
        except Exception:
            pass

        state["instant"] = self._snapshot_position()
        state["checkpoint"] = self.last_checkpoint

        return state

    # ---- diagnostic commands ----

    def cmd_POWER_LOSS_STATUS(self, gcmd):
        gcmd.respond_info(
            "sensor=GPIO%d charge=GPIO%d charge_done=%s saved=%s "
            "pending_resume=%s detection=%s"
            % (self.sensor_line, self.charge_line,
               self.charge_done, self.saved_already,
               os.path.exists(self.save_path),
               self.detection_method)
        )

    def cmd_POWER_LOSS_DISCARD(self, gcmd):
        self.prompt_attempts_left = 0
        self.gcode.run_script_from_command(
            'RESPOND TYPE=command MSG="action:prompt_end"'
        )
        if os.path.exists(self.save_path):
            os.remove(self.save_path)
            gcmd.respond_info("power_loss: state discarded")
        else:
            gcmd.respond_info("power_loss: no saved state")

    def cmd_POWER_LOSS_RESUME(self, gcmd):
        self._do_resume(gcmd, "instant")

    def cmd_POWER_LOSS_RESUME_SAFE(self, gcmd):
        self._do_resume(gcmd, "checkpoint")

    def _min_extrude_temp(self):
        try:
            heater = self.printer.lookup_object("extruder").get_heater()
            return float(getattr(heater, "min_extrude_temp", 170.0))
        except Exception:
            return 170.0

    def _resolve_resume_temps(self, gcmd, full_state, pkg):
        """Pick the temperatures to resume at. Order of preference for
        each heater (first usable value wins):
          1. EXTRUDER_TEMP= / BED_TEMP= passed to the resume command
          2. the gcode file being printed: the last temperature the
             slicer set before the resume point (_derive_temps_from_file)
          3. the heater target captured at the instant of the outage
          4. the heater target recorded at the last layer checkpoint
        A nozzle value below min_extrude_temp is never "usable" (e.g. the
        probing temperature COSMOS's PAUSE lowers to). Only if no source
        at all yields one does the resume refuse to start -- before
        anything moves. Returns (extruder_target, bed_target)."""
        min_extrude = self._min_extrude_temp()
        checkpoint = full_state.get("checkpoint") or {}

        gcmd.respond_info("power_loss: reading print temperatures from %s..."
                          % pkg["file_path"])
        file_ext, file_bed = self._derive_temps_from_file(
            pkg["file_path"], pkg["file_position"])

        def pick(override_param, file_value, key, minimum):
            override = gcmd.get_float(override_param, None, minval=0.0)
            if override is not None:
                return override, override_param
            candidates = (("gcode file", file_value),
                          ("outage capture", full_state.get(key)),
                          ("layer checkpoint", checkpoint.get(key)))
            for source_name, value in candidates:
                if value is not None and float(value) >= minimum \
                        and float(value) > 0.0:
                    return float(value), source_name
            return 0.0, None

        extruder_target, ext_src = pick("EXTRUDER_TEMP", file_ext,
                                        "extruder_target", min_extrude)
        bed_target, bed_src = pick("BED_TEMP", file_bed,
                                   "heater_bed_target", 0.0)

        if ext_src is None:
            raise gcmd.error(
                "power_loss: no usable nozzle temperature found (gcode file "
                "unreadable or without temperature commands, and no valid "
                "saved value) -- refusing to resume, nothing has moved. "
                "Re-run with the print temperatures: "
                "POWER_LOSS_RESUME_SAFE EXTRUDER_TEMP=<temp> BED_TEMP=<temp>")

        if bed_src is None:
            gcmd.respond_info(
                "power_loss: WARNING no bed temperature found -- resuming "
                "with the bed unheated. Pass BED_TEMP= if that's wrong.")

        gcmd.respond_info(
            "power_loss: temperatures -> extruder %.0fC (from %s), "
            "bed %.0fC (from %s)"
            % (extruder_target, ext_src, bed_target, bed_src or "none"))
        return extruder_target, bed_target

    def _do_resume(self, gcmd, package_name):
        if not os.path.exists(self.save_path):
            raise gcmd.error("power_loss: no saved state at %s"
                              % self.save_path)

        with open(self.save_path, "r") as f:
            full_state = json.load(f)

        pkg = full_state.get(package_name)
        if package_name == "checkpoint" and not pkg:
            gcmd.respond_info(
                "power_loss: no layer checkpoint available (outage "
                "happened before the first layer change) -- falling "
                "back to the precise instant capture instead"
            )
            pkg = full_state.get("instant")
            package_name = "instant"

        if not pkg:
            raise gcmd.error(
                "power_loss: no usable position data in saved state"
            )

        # Manual override for cases where virtual_sdcard's own status
        # came back with file_path=None (observed in real testing --
        # the key existed but its value was empty, presumably because
        # the outage happened before that field got populated). You
        # know which file you printed; this lets you supply it instead
        # of losing the whole recovery over one missing string.
        # Usage: POWER_LOSS_RESUME_SAFE FILENAME=my_part.gcode
        filename_override = gcmd.get("FILENAME", None)
        if filename_override:
            pkg = dict(pkg)  # don't mutate the loaded state in place
            if "/" in filename_override:
                pkg["file_path"] = filename_override
            else:
                pkg["file_path"] = os.path.join(
                    GCODES_DIR, filename_override
                )
            gcmd.respond_info(
                "power_loss: using manually supplied file: %s"
                % pkg["file_path"]
            )

        required = ("gcode_position", "file_path", "file_position")
        missing = [k for k in required if pkg.get(k) is None]
        if missing:
            raise gcmd.error(
                "power_loss: incomplete '%s' package, missing: %s -- "
                "cannot resume safely"
                % (package_name, ", ".join(missing))
            )

        extruder_target, bed_target = self._resolve_resume_temps(
            gcmd, full_state, pkg)

        gcmd.respond_info(
            "power_loss: resuming using '%s' position data "
            "(extruder=%.0fC bed=%.0fC)..."
            % (package_name, extruder_target, bed_target)
        )

        self.prompt_attempts_left = 0

        self.gcode.run_script_from_command(
            'RESPOND TYPE=command MSG="action:prompt_end"'
        )

        x, y, z = pkg["gcode_position"][0:3]

        # Ground truth for E / extrude mode comes from replaying the
        # actual gcode file up to this point, not from Klipper's live
        # gcode_move status (that live path is exactly where we had a
        # real bug -- see the header comment). Falls back to the
        # captured live values only if the file can't be read.
        file_e, file_absolute_extrude = self._derive_extrude_state_from_file(
            pkg["file_path"], pkg["file_position"]
        )
        if file_absolute_extrude is None:
            gcmd.respond_info(
                "power_loss: couldn't replay gcode file for extrude "
                "state, falling back to the captured live value"
            )
            absolute_extrude = pkg.get("absolute_extrude", True)
            e = pkg["gcode_position"][3] if len(pkg["gcode_position"]) > 3 else 0.0
        else:
            absolute_extrude = file_absolute_extrude
            e = file_e if file_e is not None else 0.0

        # Heat the NOZZLE to a movement-safe threshold before ANY motion
        # -- enough to release it from the part if it cooled down stuck.
        # TEMPERATURE_WAIT waits for a threshold without changing the
        # heater's actual target, so full heating of both nozzle and bed
        # keeps going in the background while homing/traveling below.
        # Unconditional on purpose: _resolve_resume_temps already
        # guaranteed a usable extruder_target, so there is no path that
        # reaches the first move without waiting here.
        gcmd.respond_info("power_loss: heating nozzle to %.0fC before "
                           "any movement (in case it's stuck to the "
                           "part)..." % min(self.move_temp, extruder_target))
        if bed_target > 0.0:
            self.gcode.run_script_from_command("M140 S%.1f" % bed_target)
        self.gcode.run_script_from_command("M104 S%.1f" % extruder_target)
        self.gcode.run_script_from_command(
            "TEMPERATURE_WAIT SENSOR=extruder MINIMUM=%.1f"
            % min(self.move_temp, extruder_target)
        )

        # Real physical homing, not a blind position guess. A saved
        # position taken while the toolhead was actively moving (e.g.
        # mid-purge-wipe) can be wrong by several mm due to Klipper's
        # motion lookahead buffer -- the commanded position isn't
        # necessarily where the steppers physically were at the instant
        # of the outage. Real endstop homing doesn't have that problem.
        #
        # Sequence (now that it's safely hot): mark Z as homed at the
        # last known height (Z wasn't actively moving during a typical
        # purge wipe, so it's the one coordinate reasonably
        # trustworthy) -> hop up for clearance -> real G28 X Y ->
        # travel to the saved X/Y (still elevated) -> lower to the
        # saved Z -> load file position and STOP, waiting for a
        # manual M24.
        gcmd.respond_info("power_loss: marking Z, hopping up for clearance...")
        self.gcode.run_script_from_command(
            "SET_KINEMATIC_POSITION Z=%.3f" % z
        )
        self.gcode.run_script_from_command("G91")
        self.gcode.run_script_from_command(
            "G1 Z%.1f F%.0f" % (self.z_hop, self.travel_speed)
        )
        self.gcode.run_script_from_command("G90")

        gcmd.respond_info("power_loss: homing X Y...")
        self.gcode.run_script_from_command("G28 X Y")

        # Undo the Z bookkeeping corruption caused by this printer's
        # homing_override (set_position_z=0 + its own real "G0 Z5" lift
        # on every G28). Physically the nozzle is now at
        # (z + z_hop + homing_override_z_lift) -- but Klipper currently
        # thinks it's at homing_override_z_lift, since the override
        # zeroed its own reference right before that move. Re-declare
        # the true height so the rest of this sequence (and the final
        # lower-to-print-height move) is accurate again.
        z_after_override = z + self.z_hop + self.homing_override_z_lift
        self.gcode.run_script_from_command(
            "SET_KINEMATIC_POSITION Z=%.3f" % z_after_override
        )
        gcmd.respond_info(
            "power_loss: saved_z=%.3f z_hop=%.1f override_lift=%.1f -> "
            "declared Z=%.3f (compare against what you physically see)"
            % (z, self.z_hop, self.homing_override_z_lift, z_after_override)
        )

        # Purge and clean at the tray, not over the part -- after
        # sitting hot/stationary and being reheated, there's almost
        # certainly some ooze on the tip that would otherwise land on
        # the print the moment M24 resumes. Wait for full temps here
        # (not before) so the homing/travel above overlaps with the
        # rest of the heating time.
        gcmd.respond_info("power_loss: waiting for full temperatures "
                           "before purging...")
        if bed_target > 0.0:
            self.gcode.run_script_from_command("M190 S%.1f" % bed_target)
        self.gcode.run_script_from_command("M109 S%.1f" % extruder_target)

        gcmd.respond_info("power_loss: purging and wiping at the tray...")
        self.gcode.run_script_from_command("MOVE_TO_TRAY")
        self.gcode.run_script_from_command("M83")
        self.gcode.run_script_from_command("G1 E%.1f F300" % self.purge_mm)
        self.gcode.run_script_from_command("M400")

        # Wipe motion -- same coordinates as this printer's own
        # CLEAN_NOZZLE macro (not calling that macro directly, since it
        # has its own heating side effects we don't want here).
        wipe_lines = ["G1 F9000"]
        for _wipe in range(4):
            for i in range(8):
                wx = 187 - (i * 2)
                wy = 265 if i % 2 == 0 else 263
                wipe_lines.append("G1 X%d Y%d" % (wx, wy))
            wipe_lines.append("G1 X187")
        wipe_lines.append("G1 Y263")
        wipe_lines.append("M400")
        self.gcode.run_script_from_command("\n".join(wipe_lines))

        gcmd.respond_info("power_loss: traveling to saved X=%.2f Y=%.2f "
                           "(still elevated)..." % (x, y))
        self.gcode.run_script_from_command(
            "G1 X%.3f Y%.3f F%.0f" % (x, y, self.travel_speed)
        )

        gcmd.respond_info("power_loss: lowering to saved Z=%.3f..." % z)
        self.gcode.run_script_from_command(
            "G1 Z%.3f F%.0f" % (z, self.travel_speed / 4.0)
        )

        if absolute_extrude:
            self.gcode.run_script_from_command("G92 E%.5f" % e)
            self.gcode.run_script_from_command("M82")
        else:
            # Relative mode: the absolute E number isn't meaningful to
            # carry forward -- zero it and switch to M83, same as
            # KlipperPLR does. Everything from here on is deltas.
            self.gcode.run_script_from_command("G92 E0")
            self.gcode.run_script_from_command("M83")

        fan_speed = full_state.get("fan_speed")
        if fan_speed is not None:
            self.gcode.run_script_from_command("M106 S%d" % round(fan_speed * 255))

        speed_factor = pkg.get("speed_factor")
        if speed_factor:
            self.gcode.run_script_from_command(
                "M220 S%.1f" % (speed_factor * 100.0)
            )

        filename = os.path.basename(pkg["file_path"])
        self.gcode.run_script_from_command("M23 %s" % filename)
        self.gcode.run_script_from_command(
            "M26 S%d" % pkg["file_position"]
        )

        gcmd.respond_info(
            "power_loss: resuming print automatically."
        )
        self.gcode.run_script_from_command("M24")

        os.remove(self.save_path)
        self.saved_already = False

    def cmd_POWER_LOSS_TEST_READ(self, gcmd):
        # Non-blocking: only shows what the normal poll timer (_poll)
        # already collected in the background. Safe to run at any time,
        # including mid-print.
        n = gcmd.get_int("N", 50, minval=1, maxval=self.read_history_max)
        vals = self.read_history[-n:]
        ones = sum(vals)
        gcmd.respond_info(
            "last %d readings: %s  (ones=%d of %d)"
            % (len(vals), vals, ones, len(vals))
        )


def load_config(config):
    return PowerLoss(config)
