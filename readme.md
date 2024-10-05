- driver for unicore rtk UM9802 Module
- The driver will dump all nmea string to nmea_msgs/Sentence topic so `nmea_topic_driver` in `nmea_navsat_driver` can use it.
- the unicore message for true north reference heading will be published to `unicore/heading` and an odometry message `unicore/heading/odometry`
-  `unicore/heading/odometry` will be used in robot localization for heading update


## Unicore setup commands
- 