# unicore_rtk_driver

## Introduction
ROS 2 driver for Unicore UM98x GNSS receivers (UM982 dual antenna heading) and other receivers
that output NMEA and RTCM 3. The driver publishes position, heading, velocity and GNSS time,
and operates as an RTK rover. RTK base stations are handled by the separate `rtk_base` package
(https://github.com/GSO-soslab/rtk_base).

- ROS 2 Jazzy
- Ubuntu 24.04
- Receivers in use: Holybro H-RTK UM982 (ALPHA RISE, WAM-V)
- Other UM98x boards (for example MJRTK UM982) use the same `unicore` receiver type
- Receivers configured with their own tools (for example SparkFun RTK Surveyor) use the `generic`
  receiver type

RTCM corrections are received over UDP, TCP, serial radio or NTRIP and written to the receiver
directly from the network threads. They are not routed through ROS topics. The corrections come
from a base running `rtk_base`, or from an NTRIP caster.

## Directory structure
- `unicore_rtk_driver/driver_node.py`: ROS node. Declares parameters, creates topics and services,
  and connects the components below.
- `unicore_rtk_driver/serial_link.py`: receiver serial port. Reader thread, automatic reconnect,
  serialized writes.
- `unicore_rtk_driver/processor.py`: converts validated NMEA and Unicore sentences into ROS messages.
  Has no I/O and is used directly by the tests.
- `unicore_rtk_driver/protocol/`: stream framing (NMEA, Unicore ASCII, RTCM 3), checksums
  (NMEA XOR, Unicore CRC-32, RTCM CRC-24Q) and sentence parsers.
- `unicore_rtk_driver/receivers/`: receiver specific configuration commands. `unicore.py` for UM98x,
  `generic.py` for receivers that are configured externally.
- `unicore_rtk_driver/rtcm/`: RTCM input transports (`transport.py`), routing with per source
  framing and failover (`router.py`), RTCM 3 helpers (`rtcm3.py`).
- `unicore_rtk_driver/tools/probe.py`: command line tool that identifies what a receiver or an RTCM
  stream outputs.
- `unicore_rtk_driver/tools/common.py`: logging for the command line tools.
- `unicore_rtk_driver/tools/rover.py`: rover without ROS, sends corrections to the receiver and
  prints its solution and heading (`unicore_rtk_rover`).
- `config/`: example parameter files.
- `launch/start.launch.py`: launch file with `namespace`, `node_name` and `params_file` arguments.
- `udev/`, `scripts/`: udev rule template and installer for stable device names.
- `test/`: unit tests, regression test against recorded data, fixture extraction tool.

Data flow:
```
receiver -> SerialLink -> StreamFramer -+-> NMEA / Unicore -> GnssProcessor -> ROS topics
                                        |                  \-> ~/nmea, gpsd (UDP)
                                        +-> RTCM (ignored)

RTCM inputs (UDP, TCP, NTRIP, serial) -> RtcmRouter -> SerialLink (written to the receiver)
```

## Installation

### Dependencies
All dependencies are declared in `package.xml` and installed with `rosdep`, including `nmea_msgs`,
`gps_msgs` and `tf_transformations`. Update the package indexes first; an outdated apt index makes the ROS
package downloads fail with `404 Not Found`.
```bash
cd ~/ros2_ws/src
git clone -b jazzy-devel https://github.com/GSO-soslab/unicore_rtk_driver.git
cd ~/ros2_ws
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
```

### Compile the code
```bash
colcon build --packages-select unicore_rtk_driver
```

### Serial port access
The user running the driver must be in the `dialout` group. The installer below adds the user to
the group and creates a stable device name from the adapter serial number, or from the USB port
when the adapter has no serial number (CH340 based boards such as the MJRTK UM982).
```bash
src/unicore_rtk_driver/scripts/install_udev_rules.sh --dry-run /dev/ttyUSB0 mjrtk_um982
src/unicore_rtk_driver/scripts/install_udev_rules.sh /dev/ttyUSB0 mjrtk_um982
```
Log out and back in after the user is added to `dialout`. `udev/99-rtk-gnss.rules` contains
commented examples for the Holybro, MJRTK and SparkFun units.

### Receiver on a Raspberry Pi UART
A receiver wired to the Raspberry Pi header (GPIO 14 TX, GPIO 15 RX) shares the UART with the
serial console that Ubuntu Server enables by default. The console reads the receiver output as
login attempts (`FAILED LOGIN on '/dev/ttyS0'` in the journal) and writes login prompts to the
receiver. Disable the console and move GPIO 14/15 to the PL011 UART, which has a larger FIFO and a
baud rate independent of the core clock (Bluetooth is disabled):
```bash
sudo sed -i 's/console=serial0,115200 //' /boot/firmware/cmdline.txt
sudo systemctl mask serial-getty@ttyS0.service serial-getty@ttyAMA0.service
echo 'dtoverlay=disable-bt' | sudo tee -a /boot/firmware/config.txt
sudo reboot
```
The receiver is then `/dev/ttyAMA0`. On Raspberry Pi OS, disable the console with `raspi-config`
(Interface Options, Serial Port: login shell disabled, serial hardware enabled) and add the same
`dtoverlay=disable-bt` line. Tested on a Raspberry Pi 4 with Ubuntu Server 24.04.

## Quick test

- Identify what the receiver outputs and at which baud rate. The tool only listens unless
  `--query-version` is given, which sends the read only `VERSIONA` query.
```bash
ros2 run unicore_rtk_driver unicore_rtk_probe /dev/ttyUSB0
ros2 run unicore_rtk_driver unicore_rtk_probe /dev/ttyUSB0 --query-version
```

- Check an RTCM stream before using it as a rover input: any `rtcm.inputs` URI (`udp://`, `tcp://`,
  `tcpserver://`, `ntrip://`, `serial://`). The tool reports the message types and rates and the
  base station position, listening for 10 s by default (`--duration`).
```bash
ros2 run unicore_rtk_driver unicore_rtk_probe udp://:2101
ros2 run unicore_rtk_driver unicore_rtk_probe tcp://<base_ip>:2101
```
```
RTCM stream udp://:2101 for 10 s
  bytes read     : 4098 (0 not part of any frame)
  RTCM messages  : 1005 x10 (1.0/s), 1074 x10 (1.0/s), 1084 x10 (1.0/s), 1124 x10 (1.0/s), 1230 x1 (0.1/s)
  base station   : id 0: <lat>, <lon>, <height> m (ellipsoid)
```

- Start the driver as a rover with the default parameters.
```bash
ros2 launch unicore_rtk_driver start.launch.py namespace:=my_robot
```

- Start the driver with a parameter file from `config/`.
```bash
ros2 launch unicore_rtk_driver start.launch.py params_file:=/path/to/params.yaml
```

| File | Purpose |
|---|---|
| `param.yaml` | Rover, receiver configuration left unchanged. Same behavior as previous versions. |
| `rover_rtk.yaml` | UM982 rover. Enables the required outputs and reads corrections from a base over TCP. |

- Send a raw command to the receiver. With `save_config_after_tx_cmd: true` (default) the driver
  also sends `saveconfig`, which writes the command to the receiver flash. Set it to false to try a
  command without saving it.
```bash
ros2 param set /my_robot/unicore_rtk_driver save_config_after_tx_cmd false
ros2 topic pub /my_robot/unicore_rtk_driver/tx_cmd std_msgs/msg/String "data: 'UNIHEADINGA COM3 0.2'" --once
```

- Resend the receiver configuration after changing parameters at runtime.
```bash
ros2 service call /my_robot/unicore_rtk_driver/configure_receiver std_srvs/srv/Trigger
```

## Receiver configuration
With `configure_receiver: true` the driver sends the following commands every time the serial port
is opened. Nothing is written to receiver flash unless `save_receiver_config: true`.

1. Mode command: `rover.mode_command` (default `MODE ROVER`).
2. One `<LOG> [<receiver_port>] <period>` per entry of `receiver_logs`. The period is in seconds,
   so `GPGGA:0.1` outputs GGA at 10 Hz.
3. Each entry of `receiver_commands`, verbatim.
4. `saveconfig` if `save_receiver_config` is true.

With `configure_receiver: false` (default) only `receiver_commands` are sent, which keeps receivers
that were configured by hand unchanged. The `generic` receiver type never generates commands.

Any receiver setting goes in `receiver_commands`. The receiver's reply to each command is logged
(`receiver replied: ...`). Commonly used UM982 settings, from the Unicore Reference Commands
Manual R1.13:

| Command | Section | Description |
|---|---|---|
| `CONFIG RTK TIMEOUT <s>` | 4.41 | Oldest corrections used for an RTK solution, 1 to 1800 s |
| `CONFIG DGPS TIMEOUT <s>` | 4.12 | Oldest corrections used for a DGPS solution, 1 to 1800 s, UM982 default 600 s |
| `CONFIG HEADING LENGTH <cm> <tolerance cm>` | 4.17 | Distance between the master and slave antennas |
| `CONFIG` | 4.1 | Report the configuration, one `$CONFIG` line per setting, logged as `receiver config: ...` |

```yaml
    receiver_commands: ['CONFIG RTK TIMEOUT 30', 'CONFIG DGPS TIMEOUT 60', 'CONFIG']
```

## RTK rover
```
base computer                                              vehicle
receiver -> rtk_base -- tcpserver / udp --> unicore_rtk_driver (rtcm.inputs) -> receiver
                     -- serial radio ----->
NTRIP caster ------------------------------> unicore_rtk_driver (ntrip://...)
```
Base stations (receiver set-up, survey-in, RTCM forwarding, relay without ROS) are described in
the `rtk_base` package.

- Data from each entry of `rtcm.inputs` is framed per source and only complete frames that pass the
  CRC are written to the receiver. When several inputs are listed, the first listed input that
  delivered a frame within `rtcm.failover_timeout_s` is used.
- The base station position from RTCM 1005/1006 in the incoming corrections is logged and reported
  in `/diagnostics`.
- The measured latency from the base receiver port to the rover receiver port, both nodes on one
  host over TCP loopback, is about 2.3 ms. This excludes radio and network transmission time. It
  was measured with the previous combined driver; the RTCM transports are unchanged.

RTK solution quality:
- `sensor_msgs/NavSatFix` reports RTK fixed and RTK float with the same status (`STATUS_GBAS_FIX`).
  `~/gpsfix` reports them as `STATUS_RTK_FIX` and `STATUS_RTK_FLOAT`; use it where the solution type
  matters (monitoring, mission logic). `/diagnostics` shows the same information as text.
- Estimators should weight the position by its covariance. On a vehicle that uses RTK, set
  `fix_covariance_mode: gst` so the covariance comes directly from the receiver's GST error
  estimate (centimeter level with an RTK fixed solution).

When corrections stop:
- The receiver decides the solution type; the driver reports it. When corrections stop arriving,
  the receiver keeps its RTK solution with the last corrections it received, and the error grows
  as they age. GGA keeps RTK fixed or float until the correction age passes the receiver's RTK
  timeout, then reports DGPS until the DGPS timeout, then single.
- `CONFIG RTK TIMEOUT` and `CONFIG DGPS TIMEOUT` in `receiver_commands` limit how long old
  corrections are used. A short RTK timeout makes the solution type follow the link; a long one
  rides out short dropouts.
- The receiver's correction age (GGA field 13) is in `/diagnostics` as
  `receiver correction age s`. `rtcm in age s` is the time since the driver last received a frame;
  after `rtcm.correction_timeout_s` the driver reports a warning (`no RTCM corrections`).

Using RTK in navigation:
- Gate on the accuracy and on the correction age, not on the solution type alone. With
  `fix_covariance_mode: gst` the covariance of `~/fix` follows the receiver's own error estimate.
- RTK positions are relative to the base position. An error in the base position (for example a
  short survey-in) shifts every RTK position by the same amount. When the solution falls back to
  DGPS or single, that offset is lost and the position can jump by up to the base position error.
- The dual antenna heading is solved between the two rover antennas and does not use the
  corrections; it stays valid when they stop.

GGA quality to `gps_msgs/GPSStatus`:

| GGA quality | Solution | `status.status` |
|---|---|---|
| 0 | no fix | `STATUS_NO_FIX` |
| 1 | single | `STATUS_FIX` |
| 2 | DGPS | `STATUS_DGPS_FIX` |
| 3 | PPS | `STATUS_FIX` |
| 4 | RTK fixed | `STATUS_RTK_FIX` |
| 5 | RTK float | `STATUS_RTK_FLOAT` |
| 6 | dead reckoning | `STATUS_NO_FIX` |
| 7 | manual position | `STATUS_FIX` |
| 8 | simulation | `STATUS_NO_FIX` |
| 9 | SBAS | `STATUS_WAAS_FIX` |

RTCM endpoints:

| URI | Input |
|---|---|
| `udp://HOST:PORT` | Listen on `HOST:PORT`. `udp://:2101` listens on all interfaces. |
| `tcpserver://BIND:PORT` | A base connects to this port. |
| `tcp://HOST:PORT` | Connect to a base `tcpserver`. Reconnects automatically. |
| `serial:///dev/ttyX?baud=57600` | Radio modem. |
| `ntrip://[USER:PASS@]HOST:PORT/MOUNT` | NTRIP v1/v2 caster. The latest GGA is sent periodically for VRS mountpoints. |

Endpoint options are given as a query string, for example `tcp://<base_ip>:2101?reconnect_s=0.5`.

| Option | Default | Applies to | Description |
|---|---|---|---|
| `reconnect_s` | 2.0 | tcp, serial, ntrip | Wait between connection attempts |
| `baud` | 57600 | serial | Baud rate |
| `gga_period_s` | 10 | ntrip | Period of the GGA sent to the caster |

### Rover without ROS
`unicore_rtk_rover` runs a rover receiver from a terminal: it takes corrections from the same
inputs as `rtcm.inputs`, writes them to the receiver and prints one line per GGA. It needs Python 3
and pyserial only and runs from the cloned repository without building it.
```bash
cd unicore_rtk_driver
python3 -m unicore_rtk_driver.tools.rover --device /dev/ttyUSB0 --configure \
    --input tcp://<base_ip>:2101
```
```
17:32:30  position RTK float   sats 26  acc 0.084 m  hdop 0.6  corr age 1.0 s  base 0000  rtcm in 402 B/s  heading 113.84 deg (antennas RTK fixed)
```
- `--configure` sends `MODE ROVER`, `GPGGA 1`, `GPGST 1` and `GPHPR 1` after every connect (receiver working
  memory only; nothing is saved). Without it the receiver is not changed.
- `--command` sends a raw command after every connect, as `receiver_commands` does, and can be
  repeated, for example `--command 'CONFIG RTK TIMEOUT 30' --command CONFIG`. Nothing is saved.
- `--log GPGGA:0.1 --log GPGST:0.1 --log GPHPR:0.1` with `--configure` outputs at 10 Hz.
- `position`: `no fix`, `single`, `DGPS`, `RTK float`, `RTK fixed`, relative to the base.
  `acc` is the receiver's horizontal standard deviation from GST, sqrt(lat_std^2 + lon_std^2);
  `hdop` describes the satellite geometry only (below 1 is excellent), not the accuracy.
  `corr age` is the age of the corrections the receiver uses; `-` means it uses none. `base` is
  the station ID they come from.
- `heading` comes from HPR: the direction from the master antenna to the slave antenna, in
  degrees from true north. It needs both antennas with a clear sky view and is solved between
  the two antennas, without the base. `antennas RTK fixed` means the heading is valid, also when
  the position is still `RTK float` or `single`.
- Without `--input` the tool only prints the solution. In a ROS workspace it is also
  `ros2 run unicore_rtk_driver unicore_rtk_rover`.

## Node: unicore_rtk_driver_node

### Published topics
Topic names are relative to the node and can be changed with the `*_topic` parameters.

| Topic | Type | Source | Description |
|---|---|---|---|
| `~/fix` | `sensor_msgs/NavSatFix` | GGA, GST | Position. Published when GST arrives, using the most recent GGA. |
| `~/gpsfix` | `gps_msgs/GPSFix` | GGA, GST | Same position and covariance as `~/fix`, with the solution type in `status.status` (`STATUS_RTK_FIX`, `STATUS_RTK_FLOAT`, ...), satellites used and HDOP. Fields without a source are NaN. |
| `~/heading_odometry` | `nav_msgs/Odometry` | UNIHEADINGA | True heading as yaw in `heading_frame_id` (NED), child frame `gps_frame_id`. |
| `~/heading_enu_pose` | `geometry_msgs/PoseWithCovarianceStamped` | UNIHEADINGA | Heading counter clockwise from east, `frame_id` = `heading_enu_frame_id`. |
| `~/heading_enu_pose/imu` | `geometry_msgs/PoseWithCovarianceStamped` | UNIHEADINGA, IMU, TF | IMU roll and pitch with GNSS heading, in the IMU frame. Requires `imu_topic`. |
| `~/heading_enu_pose/imu/manual` | `geometry_msgs/PoseWithCovarianceStamped` | UNIHEADINGA | ENU heading plus `heading_correction_angle_gps_to_imu`, in the IMU frame. |
| `~/simple_heading` | `std_msgs/Float32` | UNIHEADINGA | True heading in radians. |
| `~/unicore_velocity` | `geometry_msgs/TwistWithCovarianceStamped` | AGRICA | East, north, up velocity with covariance, `frame_id` = `velocity_frame_id`. |
| `~/global_velocity` | `geometry_msgs/TwistStamped` | RMC | East and north velocity from speed and course, `frame_id` = `velocity_frame_id`. |
| `~/time_ref` | `sensor_msgs/TimeReference` | RMC | GNSS UTC time. |
| `~/nmea` | `nmea_msgs/Sentence` | all lines | Raw receiver output. Published only when there is a subscriber. |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | | Serial link, fix status, solution type, satellites used, horizontal std (receiver estimate from GST, 1 sigma, m), HDOP, receiver correction age (GGA), RTCM input rate and age, transport status, base position from the incoming RTCM. |

### Subscribed topics

| Topic | Type | Description |
|---|---|---|
| `~/tx_cmd` | `std_msgs/String` | Raw command written to the receiver, followed by `saveconfig` when `save_config_after_tx_cmd` is true. |
| `imu_topic` | `sensor_msgs/Imu` | IMU orientation used for `~/heading_enu_pose/imu*`. |

### Services

| Service | Type | Description |
|---|---|---|
| `~/configure_receiver` | `std_srvs/Trigger` | Send the receiver configuration built from the current parameter values, independent of `configure_receiver`. |

### Parameters

Serial port and output:

| Parameter | Default | Description |
|---|---|---|
| `device` | `/dev/ttyUSB0` | Receiver serial device |
| `baudrate` | `115200` | Receiver baud rate |
| `serial_reconnect_s` | `1.0` | Wait between attempts to open the serial port |
| `use_gnss_time` | `false` | Stamp `fix`, `time_ref` and velocities with GNSS time instead of arrival time |
| `use_gpsd_udp` | `false` | Forward NMEA lines to gpsd over UDP |
| `gpsd_udp_host` | `127.0.0.1` | gpsd address |
| `virtual_port` | `5005` | gpsd UDP port |
| `gpsd_udp_patterns` | `['$GN']` | Lines containing any of these strings are forwarded to gpsd |
| `debug` | `false` | Log every sentence at debug level |
| `diagnostics_period` | `1.0` | Period of `/diagnostics` in seconds |

Frames:

| Parameter | Default | Description |
|---|---|---|
| `gps_frame_id` | `unicore_gps` | Frame of the receiver. Used for `fix`, `time_ref`, `nmea` and as child frame of `heading_odometry`. |
| `heading_frame_id` | `world_ned` | NED world frame of `heading_odometry` |
| `heading_enu_frame_id` | `world_enu` | `frame_id` of `heading_enu_pose` |
| `velocity_frame_id` | `world` | ENU frame of `unicore_velocity` and `global_velocity` |
| `imu_topic` | `imu/data` | IMU used for the IMU frame heading outputs |
| `heading_correction_angle_gps_to_imu` | `-3.0` | Fixed heading offset in degrees for `heading_enu_pose/imu/manual` |

Receiver:

| Parameter | Default | Description |
|---|---|---|
| `receiver` | `unicore` | `unicore` or `generic` |
| `receiver_port` | `''` | Receiver port named in output commands, for example `COM1`. Empty uses the port the command is received on. |
| `configure_receiver` | `false` | Send the receiver configuration each time the port is opened |
| `save_receiver_config` | `false` | Append `saveconfig` to the configuration |
| `rover.mode_command` | `MODE ROVER` | Rover mode command, for example `MODE ROVER UAV`. Empty sends no mode command. |
| `receiver_logs` | `['GPGGA:1', 'GPGST:1', 'GPRMC:1', 'GPHPR:1', 'UNIHEADINGA:0.5', 'AGRICA:1']` | Outputs to enable, `LOG:period_s` |
| `receiver_commands` | `['']` | Additional raw commands sent after the configuration |
| `save_config_after_tx_cmd` | `true` | Send `saveconfig` after each `~/tx_cmd` command |

RTCM:

| Parameter | Default | Description |
|---|---|---|
| `rtcm.inputs` | `['']` | RTCM inputs in priority order, see RTCM endpoints |
| `rtcm.failover_timeout_s` | `5.0` | Time without frames after which the next input is used |
| `rtcm.correction_timeout_s` | `5.0` | Correction age at which `/diagnostics` reports a warning |

Validation and covariance. The defaults reproduce the output of previous versions.

| Parameter | Default | Description |
|---|---|---|
| `fix_covariance_mode` | `hdop_scaled` | `hdop_scaled`: variance = (HDOP x GST std)^2, height uses 2 x HDOP. `gst`: variance = GST std^2. |
| `strict_validation` | `false` | Reject UNIHEADINGA without a solution or with a bad solution status, and publish `fix` only after a new valid GGA |
| `heading_use_gnss_time` | `false` | Stamp heading outputs with the GPS time in the UNIHEADINGA header |
| `epe_quality0` ... `epe_quality9` | | Default position error per GGA fix quality, kept for compatibility |

Topic name parameters: `fix_topic`, `gpsfix_topic`, `heading_enu_pose_topic`, `time_ref_topic`, `nmea_topic`,
`global_vel_topic`, `simple_heading_topic`, `unicore_heading_topic`, `unicore_velocity_topic`,
`tx_cmd_topic`.

## Frame conventions
The conventions below match how `alpha_rise_auv` and `wamv_rise` use the driver.

- `<robot>/world` is ENU. `<robot>/world_ned` is a static transform of it (yaw pi/2, roll pi).
- `heading_odometry`: true heading (clockwise from north) as yaw about the NED down axis,
  `frame_id` = `heading_frame_id`, `child_frame_id` = `gps_frame_id`. Fused by `robot_localization`
  (yaw only) with `pose_use_child_frame`.
- `heading_enu_pose*`: heading counter clockwise from east. `frame_id` identifies the sensor the
  heading belongs to. The vehicles set `heading_enu_frame_id` to the GNSS frame. The `/imu` outputs
  use the IMU frame because the heading is already rotated into it (MicroStrain `ext/heading_enu`).
- `fix`: `frame_id` = `gps_frame_id`, used by `world_odom_transform` as child frame of
  `gps/odometry`. Altitude is height above the WGS84 ellipsoid (GGA altitude plus geoid separation).
  The covariance is compared against `acceptable_var` by `world_odom_transform`.
- `unicore_velocity`, `global_velocity`: east, north, up in `velocity_frame_id`. Set it to
  `<robot>/world` on a namespaced robot.

## Shared code
The base station package [`rtk_base`](https://github.com/GSO-soslab/rtk_base) contains copies of
these modules, so that each package can be installed and maintained on its own:
`protocol/checksums.py`, `protocol/framer.py`, `protocol/nmea_parser.py`, `serial_link.py` and
`rtcm/rtcm3.py`. Its `fix_processor.py` is derived from `processor.py`. A fix in one of these
files belongs in both packages.

## Testing
```bash
colcon test --packages-select unicore_rtk_driver
colcon test-result --verbose
```
`test/test_golden_regression.py` replays 60 s of receiver output recorded on ALPHA RISE
(2025-11-20) and compares the result with the messages published by the previous driver version in
the same recording. New fixtures can be created from any bag that contains `~/nmea` with
`test/tools/extract_golden.py`.

## Known limitations
The following have not been verified on hardware yet:
- RTCM input on the same UM982 port that is used for the host connection.
- SparkFun RTK Surveyor as rover with `receiver: generic`.

## Acknowledgment
The NMEA parser is derived from [nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver) (BSD license).

## Message reference


<details>

<summary>GNGGA: Time, position, and fix related data with multiple antennas</summary>

Raw:
`$GNGGA,185340.00,4129.47587847,N,07125.31160943,W,1,16,0.8,16.2800,M,-30.3918,M,,*72`

Parsed:
`{'GGA': {'fix_type': 1, 'latitude': 41.491264641166666, 'latitude_direction': 'N', 'longitude': 71.42186015716666, 'longitude_direction': 'W', 'altitude': 16.28, 'mean_sea_level': -30.3918, 'hdop': 0.8, 'num_satellites': 16, 'utc_time': (1732128820, 0)}}`

<pre>
$GNGGA

NMEA 0183 standard Time, position, and fix related data with multiple antennas

Syntax
$GNGGA,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,<11>,<12>,<13>,<14>,<15>*hh<CR><LF>

Fields
<1> UTC of position fix
<2> Latitude
<3> Direction of latitude:
        N: North
        S: South
<4> Longitude
<5> Direction of longitude:
        E: East
        W: West
<6> GPS Quality indicator:
        0: Fix not valid
        1: GPS fix
        2: Differential GPS fix (DGNSS), SBAS, OmniSTAR VBS, Beacon, RTX in GVBS mode
        3: Not applicable
        4: RTK Fixed, xFill
        5: RTK Float, OmniSTAR XP/HP, Location RTK, RTX
        6: INS Dead reckoning
<7> Number of SVs in use, range from 00 through to 24+
<8> Horizontal Dilution of Precision (HDOP)
<9> Orthometric height (MSL reference)
<10> M: unit of measure for orthometric height is meters
<11> Geoid separation
<12> M: geoid separation measured in meters
<13> Age of differential GPS data record, Type 1 or Type 9. Null field when DGPS is not used.
<14> Reference station ID, range 0000 to 4095. A null field when any reference station ID is selected and no corrections are received. See table below for a description of the field values.

</pre>

</details>

<details>

<summary>GNRMC: time, date, position, velocity, etc with multiple antennas</summary>

Raw:
`$GNRMC,185340.00,A,4129.47587847,N,07125.31160943,W,0.003,180.7,201124,13.9,W,A,C*7B`

Parsed:
`{'RMC': {'fix_valid': True, 'latitude': 41.491264641166666, 'latitude_direction': 'N', 'longitude': 71.42186015716666, 'longitude_direction': 'W', 'speed': 0.0015433333333319998, 'true_course': 3.153809958353753, 'utc_time': (1732128820, 0)}}`

<pre>
$GNRMC

NMEA 0183 output time, date, position, velocity, etc with multiple antennas.

Syntax
$GNRMC,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,<11>,<12>,<13>*hh<CR><LF>

Fields
<1> UTC of position fix: hhmmss.ss
<2> Pos Status:
        A: Data valid
        V: Navigation receiver warning
<3> Latitude
<4> Latitude direction 
        N: North 
        S: South
<5> Longitude
<6> Longitude direction
        E: East
        W: West
<7> Speed over ground, knots
<8> Track true: Course over ground, degrees True, measured clockwise from the North
<9> Date: ddmmyy 
<10> Magnetic variation, degrees
<11> Magnetic variation direction
<12> Mode indicator:
        A: Autonomous mode
        D: Differential mode
        F = RTK Float
        M: Manual input mode
        N: No fix
        P = High precision mode
        R = RTK Int
        S: Simulator mode
        V = Mode invalid (except for A and D)
<13> Navigation Status:
        S = Safe
        C = Caution
        U = Unsafe
        V = Navigational status not valid
</pre>

</details>

<details>

<summary>GNGST: pseudorange measurement error statistics with multiple antennas</summary>

Raw:
`$GNGST,185340.00,3.48,2.52,2.04,173.3083,2.468,2.010,2.757*4F`

Parsed:
`{'GST': {'utc_time': (1732128820, 0), 'ranges_std_dev': 3.48, 'semi_major_ellipse_std_dev': 2.52, 'semi_minor_ellipse_std_dev': 2.04, 'semi_major_orientation': 173.3083, 'lat_std_dev': 2.468, 'lon_std_dev': 2.01, 'alt_std_dev': 2.757}}`

<pre>
$GNGST

NMEA 0183 output pseudorange measurement error statistics with multiple antennas

Syntax
$GNGST,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>*hh<CR><LF>

Fields
<1> UTC (hhmmss.ss): UTC time of the GGA/GNS fix associated with this message
<2> RMS: Standard deviation of pseudoranges and DGNSS corrections (RMS value)
<3> SMJR std: Standard deviation of semi-major axis of error ellipse (m)
<4> SMNR std: Standard deviation of semi-minor axis of error ellipse (m)
<5> Orientation: Orientation of semi-major axis of error ellipse (degrees from true north)
<6> Lat std: Standard deviation of latitude error (m)
<7> Lon std: Standard deviation of longitude error (m)
<8> Alt std: Standard deviation of altitude error (m)
</pre>

</details>


<details>

<summary>GNVTG: actual track made good and speed over ground with multiple antennas</summary>

Raw:
`$GNVTG,180.683,T,194.562,M,0.00335,N,0.00621,K,A*34`

Parsed:
`{'VTG': {'true_course': 180.683, 'speed': 0.0017233888888873998}}`

<pre>
$GNVTG

NMEA 0183 standard actual track made good and speed over ground

Syntax
$GNVTG,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>*hh<CR><LF>

Fields
<1> Track made good (degrees true)
<2> T: track made good is relative to true north
<3> Track made good (degrees magnetic)
<4> M: track made good is relative to magnetic north
<5> Speed, in knots
<6> N: speed is measured in knots
<7> Speed over ground in kilometers/hour (kph)
<8> K: speed over ground is measured in kph
<9> Mode indicator:
    A: Autonomous mode
    D: Differential mode
    E: Estimated (dead reckoning) mode
    M: Manual Input mode
    S: Simulator mode
    N: Data not valid
</pre>

</details>

<details>

<summary>GNHPR: heading, pitch, and roll angles for dual-antenna carriers</summary>

Raw:
`$GNHPR,185340.00,113.84,-01.36,000.00,4,16,0.00,0999*4A`

Parsed:
`{'HPR': {'utc_time': (1732128820, 0), 'heading': 113.84, 'pitch': -1.36, 'roll': 0.0, 'fix_type': 4, 'num_satellites': 16, 'age': 0.0, 'station_id': 999}}`

<pre>
$GNHPR

NMEA 0183 standard heading, pitch, and roll angles for dual-antenna carriers

Syntax
$GNHPR,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>*hh<CR><LF>

Fields
<1> UTC: hhmmss.ss
<2> Heading: hhh.hhhh, 0~360°
<3> Pitch: ppp.pppp, -90~90°
<4> Roll: rrr.rrrr, -90~90°
<5> Solution quality indicator:
        0 = Fix invalid
        1 = Single point positioning
        2 = Differential GPS
        4 = RTK fix
        5 = RTK float
        6 = Dead reckoning mode
        7 = Manual input mode (fixed value)
        8 = Extra wide-lane
        9 = SBAS
<6> Satellite Number
<7> Age of differential data: dd.dd
<8> Base station ID: xxxx
</pre>

</details>
