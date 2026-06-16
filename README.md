# APM
Code for an RC car-based autonomous pacemaker (APM). 

Forked from the 2024 NTNU master's thesis ["Development of an Autonomous Rabbit for Running on a Track"](https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/3152310).

## Hardware

- **ZED Box Orin NX 16GB** - runs the control logic, GNSS, and ZED SDK vision processing
- **2× ZED X stereo cameras** - front camera for lane detection/steering, back camera for runner tracking
- **Arrma Infraction 6S RC car** - ESC for main motor, steering servo, brake servo
- **Arduino** - controls ESC and servos via PWM over an Ethernet connection to the ZED Box

## Software dependencies

- [ZED SDK](https://www.stereolabs.com/developers/release) (required for camera drivers)
- [ZED Python API](github.com/stereolabs/zed-python-api) - Recommended to install in a [virtual environment like conda](https://www.stereolabs.com/docs/development/api-languages/python/virtual-environment)
- Get required Python packages via conda:

```bash
conda install ...  # (To be automated... but just install any packages you need. If you can't find the package, they're usually on conda-forge)
```

## Running manually

```bash
conda run python -m apm
```

The web app starts on port `8080`. Open `http://<device-ip>:8080` to configure and start a run.

## Configuration

Settings can be found in `config/`:

- `default.toml` - baseline config (camera serials, PID gains, Arduino IP/port, etc.)
- `settings.toml` - local overrides; edit this file rather than `default.toml`

Key things to check before a first run:

Check that `gpsd` is running as a systemctl service.
On a fresh install of JetPack/Ubuntu, remove the service. It tends to start before the GNSS receiver has been detected, so it just sits there doing nothing.
Instead, set it up according to [Stereolabs guidelines](https://www.stereolabs.com/docs/development/zed-sdk/modules/global-localization/setting-up-gnss-rtk)

| Setting | Location | Description |
|---|---|---|
| `camera.front.serial_number` | `settings.toml` | Serial number of the front ZED X camera |
| `camera.back.serial_number` | `settings.toml` | Serial number of the back ZED X camera |
| `arduino.socket.host` | `settings.toml` | IP of the Arduino on the LAN interface (`192.168.56.1` by default) |
| `webapp.port` | `settings.toml` | Port for the NiceGUI web app (default `8080`) |

## Arduino firmware

The Arduino firmware lives in `arduino/`. Flash it with [PlatformIO](https://platformio.org/):

```bash
cd arduino
pio run --target upload
```

**NB!** PlatformIO can also be run as a VScode extension.

## Run at startup (Ubuntu / systemd)

To start the APM automatically on boot, create a systemd user service.

**1. Create the service file** at `~/.config/systemd/user/apm.service`:

```ini
[Unit]
Description=APM
After=network.target

[Service]
ExecStart=conda run -n base python -m apm
WorkingDirectory=/home/APM
Restart=on-failure
RestartSec=5

[Install]
WantedBy=default.target
```

**2. Enable and start it:**

```bash
systemctl --user enable apm.service
systemctl --user start apm.service
```

**3. Allow it to run without a login session** (needed on headless boot):

```bash
sudo loginctl enable-linger $USER
```

**Check status and logs:**

```bash
systemctl --user status apm.service
journalctl --user -u apm.service -f
```

**Stop the service** (e.g. for an update)
```bash
systemctl --user stop apm.service
```