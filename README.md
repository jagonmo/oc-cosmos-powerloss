# oc-cosmos-powerloss

Hardware-backed power loss recovery for the **Elegoo Centauri Carbon 1**
running **OpenCentauri COSMOS**.

The Centauri Carbon board has a supercapacitor and a dedicated
power-fail detection line that Elegoo's stock firmware uses for its own
power loss recovery. COSMOS does not use them by default (the supercap
is not even charged). This Klipper extra brings that hardware back:

- Charges the supercap and watches the power-fail line from the SoC.
- On an outage, while the supercap keeps the SoC alive for a few
  seconds, it saves the print state to disk, turns the heaters off,
  and remounts writable filesystems read-only so nothing gets corrupted.
- On the next boot it offers to resume the print from Mainsail/Fluidd.

## Hardware

Pin numbers come from Elegoo's GPL-3.0 firmware sources
(`elegooofficial/CentauriCarbon`, `elegooofficial/CentauriCarbon2`):

| Signal | SoC pin | Linux GPIO | Use |
|---|---|---|---|
| `BREAK_DETECTION_GPIO` | PG0 | 192 | Power-fail sense (input) |
| `CC_EN_GPIO` | PC1 | 65 | Supercap charge enable (output) |
| `BREAK_CONTROL_GPIO` | PG11 | 203 | **Never touched** by this module |

Measured on a real unit with mains fully unplugged: the supercap keeps
the SoC alive for roughly 5–15 seconds. Motors and heaters are not
assumed to have any backup power.

Outage detection uses the first method that works, in this order:
`gpiod` Python bindings → raw `ioctl` on `/dev/gpiochip0` (v1 GPIO uAPI,
the one in use on the Centauri Carbon 1) → sysfs polling. The active
method is shown by `POWER_LOSS_STATUS` and logged in `klippy.log`.

## Files

```
extras/
├── __init__.py      # empty, marks the package
└── power_loss.py    # the Klipper extra ([power_loss] section)
```

## Installation

### Included in COSMOS

If your COSMOS build ships this module, `power_loss.py` is already in
`/usr/share/klipper/klippy/extras/`. Skip to [Configuration](#configuration).
The module is inactive until you add the `[power_loss]` section.

### Manual install (COSMOS builds without the module)

COSMOS's root filesystem is a read-only squashfs, so the file can't be
copied straight into `klippy/extras/`. Instead, overlay a writable
directory on top of it.

1. Copy the module to a writable location on the printer:

   ```sh
   mkdir -p /data/klipper-extras-upper /data/klipper-extras-work
   cp power_loss.py /data/klipper-extras-upper/power_loss.py
   ```

2. Edit `/etc/init.d/klipper` and add this inside the `start)` block,
   right **before** the `start-stop-daemon -S ...` line, so the overlay
   is mounted before Klipper starts:

   ```sh
   grep -q " /usr/share/klipper/klippy/extras overlay " /proc/mounts || \
   mount -t overlay klipper-extras-overlay \
       -o lowerdir=/usr/share/klipper/klippy/extras,upperdir=/data/klipper-extras-upper,workdir=/data/klipper-extras-work \
       /usr/share/klipper/klippy/extras
   ```

3. Restart Klipper (`service klipper restart`) and confirm the file is
   visible:

   ```sh
   ls /usr/share/klipper/klippy/extras/power_loss.py
   ```

A COSMOS update may overwrite `/etc/init.d/klipper`; if the module stops
loading after an update, repeat step 2.

## Configuration

Add to `printer.cfg` (`/etc/klipper/config/printer.cfg`):

```ini
[power_loss]
sensor_gpio: 192
charge_gpio: 65
save_path: /data/power_loss_state.json
```

Then restart Klipper and run `POWER_LOSS_STATUS`. After `charge_time`
seconds it should report `charge_done=True`.

### Options

| Option | Default | Description |
|---|---|---|
| `sensor_gpio` | `192` | Linux GPIO number of the power-fail sense line. |
| `charge_gpio` | `65` | Linux GPIO number of the supercap charge enable. |
| `invert_sensor` | `False` | Set to `True` if the sense line's polarity is inverted. |
| `poll_interval` | `0.01` | Seconds between reads when falling back to polling. |
| `charge_time` | `30` | Seconds after startup before the supercap is considered charged. |
| `save_path` | `/data/power_loss_state.json` | Where the saved state is written. Must be on a writable filesystem. |
| `z_hop` | `10.0` | mm lifted on resume before homing X/Y. |
| `travel_speed` | `3000` | mm/min for hop and travel moves on resume. |
| `homing_override_z_lift` | `5.0` | Must match the `G0 Z<N>` lift in the printer's `[homing_override]`. Used to correct Z after `G28 X Y`. |
| `purge_mm` | `15.0` | Filament purged at the tray before resuming. |
| `move_temp` | `200.0` | Nozzle temperature reached before **any** movement, to free a nozzle stuck to the part. Full temperatures keep heating in the background. |

## Usage

When the printer boots with a saved state, a prompt appears in
Mainsail/Fluidd with three options. They can also be run manually:

| Command | Description |
|---|---|
| `POWER_LOSS_RESUME_SAFE` | Resume from the start of the last completed layer. Repeats part of a layer but avoids resuming mid-move. **Recommended.** |
| `POWER_LOSS_RESUME` | Resume from the exact position at the moment of the outage. |
| `POWER_LOSS_DISCARD` | Delete the saved state without resuming. |
| `POWER_LOSS_STATUS` | Show charge state, detection method and whether a resume is pending. |
| `POWER_LOSS_TEST_READ [N=50]` | Show the last sense-line readings. Safe to run while printing. |

Both resume commands accept optional overrides:

- `FILENAME=<file.gcode>` if the saved file path is empty.
- `EXTRUDER_TEMP=<C>` and `BED_TEMP=<C>` to force the resume
  temperatures.

### What the resume does

1. Reads the print temperatures from the G-code file being printed:
   the last `PRINT_START EXTRUDER=… BED=…` or `M104`/`M109`/`M140`/`M190`
   before the resume point, so per-layer temperature changes are
   respected. If the file can't be read, it falls back to the heater
   targets saved at the outage, then to those of the last completed
   layer. Only if none of these gives a nozzle temperature of at least
   `min_extrude_temp` does it stop with an error, before anything
   moves.
2. Starts heating bed and nozzle, and waits for the nozzle to reach
   `move_temp` before any movement.
3. Lifts Z by `z_hop`, homes X and Y, and corrects Z for the
   `[homing_override]` lift.
4. Waits for full bed and nozzle temperatures.
5. Moves to the tray, purges `purge_mm` and wipes the nozzle.
6. Travels to the saved X/Y, lowers to the saved Z, restores extrusion
   mode, fan speed and speed factor, and resumes the file with `M24`.

The extrusion mode (M82/M83) and last E value are read from the G-code
file itself rather than from Klipper's live state (technique borrowed
from [KlipperPLR](https://github.com/bigtreetech/KlipperPLR)).

## Requirements and limitations

- Written for the **Centauri Carbon 1**. The resume uses COSMOS's
  `MOVE_TO_TRAY` macro and the tray wipe coordinates of this printer.
- Z is not re-homed: the resume trusts the saved Z height, since the bed
  can't be probed with a part on it. Don't move the bed or Z by hand
  between the outage and the resume.
- The state is only saved if a print was active (printing or paused).
  Turning the printer off while idle does not create a resume prompt.
- No debounce is applied to the sense line yet. In testing it read
  cleanly with no noise during printing.

## Credits and license

Based on reverse engineering of Elegoo's GPL-3.0 firmware
(`elegooofficial/CentauriCarbon`, `elegooofficial/CentauriCarbon2`).
Extrusion state recovery idea from
[KlipperPLR](https://github.com/bigtreetech/KlipperPLR).

Licensed under the GNU General Public License v3.0. See [LICENSE](LICENSE).
