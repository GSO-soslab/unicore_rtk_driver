from unicore_rtk_driver.protocol import nmea_parser, unicore_parser


def test_malformed_nmea_returns_false_instead_of_raising():
    assert nmea_parser.parse_nmea_sentence('$GNGGA,1,2*00') is False
    assert nmea_parser.parse_nmea_sentence('$GNRMC,123519,A*00') is False


def test_gga_empty_fix_type_is_no_fix():
    gga = nmea_parser.parse_nmea_sentence('$GNGGA,173242.00,,,,,,00,9999.0,,,,,,*49')
    assert gga['GGA']['fix_type'] == 0


def test_rmc_time_without_decimals_and_century():
    now = 1763659904.0  # 2025-11-20
    secs, nsecs = nmea_parser.convert_time_rmc('201125', '173144', now)
    assert (secs, nsecs) == (1763659904, 0)
    secs, _ = nmea_parser.convert_time_rmc('311299', '235959', now)
    assert secs == 946684799  # 1999-12-31, not 2099


def test_agric_speed_flag():
    assert unicore_parser.convert_agric_speed_type('0') is True
    assert unicore_parser.convert_agric_speed_type('1') is False
    assert unicore_parser.convert_agric_speed_type(0) is True
    assert unicore_parser.convert_agric_speed_type('') is False


def test_agric_year():
    assert unicore_parser.convert_time_argica(25, 11, 20, 17, 31, 44) == 1763659904


def test_gps_time():
    # header of a recorded UNIHEADINGA: week 2393, 408722000 ms, 18 leap seconds
    assert unicore_parser.gps_week_ms_to_unix(2393, 408722000, 18) == (1763659904, 0)
    assert unicore_parser.gps_week_ms_to_unix(0, 0, 18) is None


def test_short_unicore_sentence():
    assert unicore_parser.parse_unicore_sentence('#AGRICA,1,GPS;GNSS,232*00') is False
    assert unicore_parser.parse_unicore_sentence('#AGRICA,no-separator*00') is False
