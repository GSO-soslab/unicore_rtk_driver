# Unicore GPS ROS1 driver

This is ROS1 driver for [H-RTK Unicore UM982 module](https://holybro.com/products/h-rtk-unicore-um982?srsltid=AfmBOor9B8UnQe8gZG0i5H9SFrWDkjocAbjg1wBmmKDetLk7dQbwZMtK).

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

## issue:
- the nav_fix timestamp is based on nmea_topic_driver, not based on nmea_sentence sent, it has almost 1 seconds delay
    - for example: 
        - nmea_sentence: secs: 1731691980
                         nsecs:  63486576
        - fix: secs: 1731691981
               nsecs:  63377618

## Todo:

- publish the heading in pose

- publish NavFix message:
    std_msgs/Header header: ok
    sensor_msgs/NavSatStatus: ok
        status: GGA=based on fix type, has more information, RMC=fixed or not fixed
        service: NavSatStatus.SERVICE_GPS
    float64 latitude: GGA/RMC:direct :ok
    float64 longitude: GGA/RMC:direct :ok
    float64 altitude: GGA: direct, RMC=nan :ok
    float64[9] position_covariance: need test, from GST msg :ok
    uint8 position_covariance_types 

- publish TimeReference
- publish vel

$GNGGA,185340.00,4129.47587847,N,07125.31160943,W,1,16,0.8,16.2800,M,-30.3918,M,,*72
{'GGA': {'fix_type': 1, 'latitude': 41.491264641166666, 'latitude_direction': 'N', 'longitude': 71.42186015716666, 'longitude_direction': 'W', 'altitude': 16.28, 'mean_sea_level': -30.3918, 'hdop': 0.8, 'num_satellites': 16, 'utc_time': (1732128820, 0)}}

$GNGLL,4129.47587847,N,-7125.31160943,W,185340.00,A,A*75

$GNRMC,185340.00,A,4129.47587847,N,07125.31160943,W,0.003,180.7,201124,13.9,W,A,C*7B
{'RMC': {'fix_valid': True, 'latitude': 41.491264641166666, 'latitude_direction': 'N', 'longitude': 71.42186015716666, 'longitude_direction': 'W', 'speed': 0.0015433333333319998, 'true_course': 3.153809958353753, 'utc_time': (1732128820, 0)}}

$GNGST,185340.00,3.48,2.52,2.04,173.3083,2.468,2.010,2.757*4F
{'GST': {'utc_time': (1732128820, 0), 'ranges_std_dev': 3.48, 'semi_major_ellipse_std_dev': 2.52, 'semi_minor_ellipse_std_dev': 2.04, 'semi_major_orientation': 173.3083, 'lat_std_dev': 2.468, 'lon_std_dev': 2.01, 'alt_std_dev': 2.757}}

$GNVTG,180.683,T,194.562,M,0.00335,N,0.00621,K,A*34
{'VTG': {'true_course': 180.683, 'speed': 0.0017233888888873998}}

$GNTHS,113.8408,A*1E

$GNHPR,185340.00,113.84,-01.36,000.00,4,16,0.00,0999*4A
{'HPR': {'utc_time': (1732128820, 0), 'heading': 113.84, 'pitch': -1.36, 'roll': 0.0, 'fix_type': 4, 'num_satellites': 16, 'age': 0.0, 'station_id': 999}},

- Unicore NMEA sentence data structure
$GNTHS,113.8408,A*1E
Header,Heading,Mode_indicator,checksum,sentence_terminator

$GNHPR,185340.00,113.84,-01.36,000.00,4,16,0.00,0999*4A
Header,UTC_time,heading,pitch,roll,quality_indicator,satellite_number,age,base_station_id,checksum,sentence_terminator



