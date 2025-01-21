# Unicore GPS ROS1 driver

This is ROS1 driver for [H-RTK Unicore UM982 module](https://holybro.com/products/h-rtk-unicore-um982?srsltid=AfmBOor9B8UnQe8gZG0i5H9SFrWDkjocAbjg1wBmmKDetLk7dQbwZMtK).

## TODO:

- publish msg from UNIHEADINGA: ok
        - heading and pitch with std
- publish msg from AGRICA
        - ENU velocity with STD
        - rover station lat/lon/alt with std
                - is this the primary antenna ?
                - but no HDOP, what is the `proper way to calculate covariance of gps measurements`
- publish the msg from GPS NMEA ?
        - GGA: ok
                - lat/lon/alt, with default std, `IO time is the actual IO time`, GPS time is the time from data['time'], 
        - GST: ok
                - `lat/lon/alt std`, use the std here
        - RMC: ok
                - time_ref:GNSS/data['time]
                - velocity for test
        - HPR: ok
                - publish the HPR heading and simple heading, for test

AGRICA: 450 characters + 2 more for '\r\n', total will be 452
UNIHEADINGA: 143 characters + 2 more for '\r\n', total will be 145

## Test:
- check all the data paring is good
- check if all the data vaild before publish

## published message:
- heading message:
    - info: the true north heading, unit: degree
    - type: std_msgs/Float32
    - topic: "unicore/heading"
    - usage: for checking heading information

- heading odometry message: 
    - info: the true north heading, represent in orientation
    - type: nav_msgs/Odometry
    - topic: "unicore/heading_odom" 
    - usage: feed to robot_localization for heading update
- nmea message:
    - info: GGA / RMC / GST / HDT / VTG
    - type: nmea_msgs/Sentence 
    - topic: "unicore/nmea" 
    - usage: feed to nmea_navsat_driver/nmea_topic_driver for gps fix update


## Acknowledgment

https://github.com/ros-drivers/nmea_navsat_driver

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
