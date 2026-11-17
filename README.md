# unicore_rtk_driver

## Introduction
ROS 2 driver for Unicore UM98x GNSS receivers (UM982 dual antenna heading) and other receivers
that output NMEA and RTCM 3. The driver publishes position, heading, velocity and GNSS time,
and supports operation as an RTK rover or as an RTK base station.

- ROS 2 Jazzy
- Ubuntu 24.04
- Receivers in use: Holybro H-RTK UM982 (ALPHA RISE, WAM-V)
- Other UM98x boards (for example MJRTK UM982) use the same `unicore` receiver type
- Receivers configured with their own tools (for example SparkFun RTK Surveyor) use the `generic`
  receiver type

RTCM corrections are exchanged over UDP, TCP, serial radio or NTRIP directly from the serial
reader and network threads. They are not routed through ROS topics.
A base station computer without ROS can forward its receiver's RTCM with the included relay
(see RTCM relay without ROS).

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
- `unicore_rtk_driver/rtcm/`: RTCM transports (`transport.py`), routing with per source framing and
  failover (`router.py`), RTCM 3 helpers (`rtcm3.py`).
- `unicore_rtk_driver/tools/probe.py`: command line tool that identifies what a receiver outputs.
- `unicore_rtk_driver/tools/relay.py`: RTCM relay for a base station computer without ROS.
- `config/`: example parameter files.
- `launch/start.launch.py`: launch file with `namespace`, `node_name` and `params_file` arguments.
- `udev/`, `scripts/`: udev rule template and installer for stable device names.
- `systemd/`: service file for the RTCM relay.
- `test/`: unit tests, regression test against recorded data, fixture extraction tool.

Data flow:
```
receiver -> SerialLink -> StreamFramer -+-> NMEA / Unicore -> GnssProcessor -> ROS topics
                                        |                  \-> ~/nmea, gpsd (UDP)
                                        +-> RTCM ------------> RtcmRouter -> RTCM outputs

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

## Quick test

- Identify what the receiver outputs and at which baud rate. The tool only listens unless
  `--query-version` is given, which sends the read only `VERSIONA` query.
```bash
ros2 run unicore_rtk_driver unicore_rtk_probe /dev/ttyUSB0
ros2 run unicore_rtk_driver unicore_rtk_probe /dev/ttyUSB0 --query-version
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
| `base_um982.yaml` | UM982 base. Survey-in or fixed position, RTCM served over TCP and UDP. |
| `base_generic.yaml` | Externally configured base (for example SparkFun RTK Surveyor). RTCM forwarded. |
| `relay.conf.example` | Arguments for the RTCM relay (no ROS), see RTCM relay without ROS. |

- Send a raw command to the receiver.
```bash
ros2 topic pub /my_robot/unicore_rtk_driver/tx_cmd std_msgs/msg/String "data: 'UNIHEADINGA COM3 0.2'" --once
```

- Resend the receiver configuration after changing parameters at runtime.
```bash
ros2 service call /my_robot/unicore_rtk_driver/configure_receiver std_srvs/srv/Trigger
```

## Receiver configuration
With `configure_receiver: true` the driver sends the following commands every time the serial port
is opened. Nothing is written to receiver flash unless `save_receiver_config: true`.

1. Mode command. Rover: `rover.mode_command` (default `MODE ROVER`). Base: `base.mode_command` if
   set, otherwise `MODE BASE TIME <base.survey_in_min_duration_s>` or
   `MODE BASE <lat> <lon> <height>` when `base.use_fixed_position` is true.
2. Base only: one `RTCM<number> [<receiver_port>] <period>` per entry of `base.rtcm_messages`.
3. One `<LOG> [<receiver_port>] <period>` per entry of `receiver_logs`.
4. Each entry of `receiver_commands`, verbatim.
5. `saveconfig` if `save_receiver_config` is true.

With `configure_receiver: false` (default) only `receiver_commands` are sent, which keeps receivers
that were configured by hand unchanged. The `generic` receiver type never generates commands.

## RTK base and rover
```
base computer                                         vehicle
receiver -> driver (mode: base) -- tcpserver / udp --> driver (mode: rover) -> receiver
                                -- serial radio ----->
NTRIP caster ------------------------------------------> driver (mode: rover, ntrip://...)
```

- Base: RTCM frames read from the receiver are written to every entry of `rtcm.outputs`. The base
  station position from RTCM 1005/1006 is logged and reported in `/diagnostics`.
- Rover: data from each entry of `rtcm.inputs` is framed per source and only complete frames that
  pass the CRC are written to the receiver. When several inputs are listed, the first listed input
  that delivered a frame within `rtcm.failover_timeout_s` is used.
- The measured latency from the base receiver port to the rover receiver port, both nodes on one
  host over TCP loopback, is about 2.3 ms. This excludes radio and network transmission time.

RTK solution quality:
- `sensor_msgs/NavSatFix` reports RTK fixed and RTK float with the same status (`STATUS_GBAS_FIX`).
  `~/gpsfix` reports them as `STATUS_RTK_FIX` and `STATUS_RTK_FLOAT`; use it where the solution type
  matters (monitoring, mission logic). `/diagnostics` shows the same information as text.
- Estimators should weight the position by its covariance. On a vehicle that uses RTK, set
  `fix_covariance_mode: gst` so the covariance comes directly from the receiver's GST error
  estimate (centimeter level with an RTK fixed solution).

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

| URI | Output (base) | Input (rover) |
|---|---|---|
| `udp://HOST:PORT` | One datagram per frame. Broadcast addresses are allowed. | Listen on `HOST:PORT`. `udp://:2101` listens on all interfaces. |
| `tcpserver://BIND:PORT` | Rovers connect. A client that cannot keep up is disconnected. | A base connects to this port. |
| `tcp://HOST:PORT` | Connect to a relay or rover. Reconnects automatically. | Connect to a base `tcpserver`. Reconnects automatically. |
| `serial:///dev/ttyX?baud=57600` | Radio modem. | Radio modem. |
| `ntrip://[USER:PASS@]HOST:PORT/MOUNT` | Not supported. | NTRIP v1/v2 caster. The latest GGA is sent periodically for VRS mountpoints. |

Endpoint options are given as a query string, for example `tcp://192.168.1.10:2101?reconnect_s=0.5`.

| Option | Default | Applies to | Description |
|---|---|---|---|
| `reconnect_s` | 2.0 | tcp, serial, ntrip | Wait between connection attempts |
| `send_timeout_s` | 0.05 | tcp, tcpserver, serial outputs | Maximum time a send may block |
| `baud` | 57600 | serial | Baud rate |
| `gga_period_s` | 10 | ntrip | Period of the GGA sent to the caster |

### RTCM relay without ROS
A base station computer that only has to forward corrections does not need ROS. The relay reads
the receiver's serial port and writes the RTCM frames to the same outputs as `rtcm.outputs`, using
the same serial link, framing and transports as the node. It needs Python 3 and pyserial only
(`sudo apt install python3-serial`) and runs from the cloned repository without building it.
The receiver is not configured by the relay; set up base mode and RTCM output with the receiver's
own tools.

```bash
git clone -b jazzy-devel https://github.com/GSO-soslab/unicore_rtk_driver.git
cd unicore_rtk_driver
python3 -m unicore_rtk_driver.tools.relay --device /dev/ttyACM0 --baud 115200 \
    --output tcpserver://0.0.0.0:2101 --output udp://192.168.1.255:2101
```

Arguments can be kept in a file, one per line (see `config/relay.conf.example`):
```bash
python3 -m unicore_rtk_driver.tools.relay @$HOME/rtk/relay.conf
```

The relay prints a status line every `--status-period` seconds (serial state, RTCM rate, message
types, output state, base position from RTCM 1005/1006) and warns when no RTCM arrives for
`--no-rtcm-warning-s` seconds. `systemd/unicore_rtk_relay.service` starts it at boot; edit the user
and paths in the file before installing it. In a ROS workspace the same tool is available as
`ros2 run unicore_rtk_driver unicore_rtk_relay`.

| Option | Default | Description |
|---|---|---|
| `--device` | required | Receiver serial device |
| `--baud` | `115200` | Receiver baud rate |
| `--output` | required | RTCM output URI, repeatable |
| `--status-period` | `10` | Seconds between status lines, 0 disables them |
| `--serial-reconnect-s` | `1.0` | Wait between attempts to open the serial port |
| `--no-rtcm-warning-s` | `10` | Warn when no RTCM arrives for this long |
| `--log-level` | `INFO` | `DEBUG`, `INFO`, `WARNING` or `ERROR` |

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
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | | Serial link, fix status, solution type, satellites used, RTCM rates and age, transport status, base position. |

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
| `mode` | `rover` | `rover` or `base` |
| `receiver` | `unicore` | `unicore` or `generic` |
| `receiver_port` | `''` | Receiver port named in output commands, for example `COM1`. Empty uses the port the command is received on. |
| `configure_receiver` | `false` | Send the receiver configuration each time the port is opened |
| `save_receiver_config` | `false` | Append `saveconfig` to the configuration |
| `rover.mode_command` | `MODE ROVER` | Rover mode command, for example `MODE ROVER UAV`. Empty sends no mode command. |
| `receiver_logs` | `['GPGGA:1', 'GPGST:1', 'GPRMC:1', 'GPHPR:1', 'UNIHEADINGA:0.5', 'AGRICA:1']` | Outputs to enable, `LOG:period_s` |
| `receiver_commands` | `['']` | Additional raw commands sent after the configuration |
| `save_config_after_tx_cmd` | `true` | Send `saveconfig` after each `~/tx_cmd` command |

Base station:

| Parameter | Default | Description |
|---|---|---|
| `base.mode_command` | `''` | Base mode command sent verbatim. Empty generates it from the parameters below. |
| `base.survey_in_min_duration_s` | `120` | Survey-in duration |
| `base.use_fixed_position` | `false` | Use the fixed position instead of survey-in |
| `base.fixed_latitude_deg` | `0.0` | Fixed base latitude |
| `base.fixed_longitude_deg` | `0.0` | Fixed base longitude |
| `base.fixed_altitude_m` | `0.0` | Fixed base height, in the reference expected by the receiver |
| `base.rtcm_messages` | `['1006:10', '1033:10', '1074:1', '1084:1', '1094:1', '1124:1']` | RTCM messages to enable, `number:period_s` |

RTCM:

| Parameter | Default | Description |
|---|---|---|
| `rtcm.outputs` | `['']` | RTCM outputs, see RTCM endpoints |
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
- UM982 base commands: `MODE BASE TIME`, `MODE BASE <lat> <lon> <height>` (height reference to be
  confirmed for the firmware in use) and `RTCM<number> [port] <period>`.
- RTCM input on the same UM982 port that is used for the host connection.
- SparkFun RTK Surveyor as base or rover with `receiver: generic`.

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
