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
- Unicore NMEA sentence data structure
    - $GNTHS
    - $GNHPR