"""Tests for the ~/gpsfix output (gps_msgs/GPSFix) and its RTK status."""
import math

import pytest
from builtin_interfaces.msg import Time
from gps_msgs.msg import GPSStatus

from test_golden_regression import FIXTURE, Log, alpha_rise_config, replay
from unicore_rtk_driver.processor import GnssProcessor
from unicore_rtk_driver.protocol.checksums import nmea_checksum


def nmea(body):
    return f'${body}*{nmea_checksum(body):02X}'


def gga(quality, satellites=24):
    return nmea(f'GNGGA,173144.00,4129.47587847,N,07125.31160943,W,{quality},{satellites},'
                '0.6,16.2800,M,-30.3918,M,1.0,0001')


GST = nmea('GNGST,173144.00,1.49,0.012,0.009,156.0,0.010,0.011,0.020')


def run(lines):
    out = {}
    proc = GnssProcessor(alpha_rise_config(), lambda name, msg: out.setdefault(name, []).append(msg), Log())
    stamp = Time(sec=1763659904, nanosec=0)
    for line in lines:
        assert proc.handle_nmea(line, stamp) is not False, line
    return out


@pytest.mark.parametrize('quality, status', [
    (4, GPSStatus.STATUS_RTK_FIX),
    (5, GPSStatus.STATUS_RTK_FLOAT),
    (2, GPSStatus.STATUS_DGPS_FIX),
    (1, GPSStatus.STATUS_FIX),
])
def test_status_from_gga_quality(quality, status):
    out = run([gga(quality), GST])
    msg, fix = out['gpsfix'][0], out['fix'][0]
    assert msg.status.status == status
    assert msg.status.satellites_used == 24
    assert msg.status.position_source == GPSStatus.SOURCE_GPS
    assert (msg.latitude, msg.longitude, msg.altitude) == (fix.latitude, fix.longitude, fix.altitude)
    assert list(msg.position_covariance) == list(fix.position_covariance)
    assert msg.position_covariance_type == fix.position_covariance_type
    assert msg.header == fix.header
    assert msg.hdop == pytest.approx(0.6)
    assert math.isnan(msg.speed) and math.isnan(msg.err_horz)


def test_horizontal_std_from_gst():
    proc = GnssProcessor(alpha_rise_config(), lambda name, msg: None, Log())
    assert math.isnan(proc.horizontal_std_m)
    proc.handle_nmea(gga(4), Time(sec=1763659904, nanosec=0))
    proc.handle_nmea(GST, Time(sec=1763659904, nanosec=0))
    assert proc.horizontal_std_m == pytest.approx(math.hypot(0.010, 0.011))


def test_rtk_fixed_and_float_share_navsatfix_status():
    fixed, floating = run([gga(4), GST]), run([gga(5), GST])
    # NavSatFix cannot tell them apart; GPSFix can
    assert fixed['fix'][0].status.status == floating['fix'][0].status.status
    assert fixed['gpsfix'][0].status.status != floating['gpsfix'][0].status.status


def test_recorded_data_one_gpsfix_per_fix():
    assert FIXTURE
    _, out, _ = replay(alpha_rise_config())
    assert len(out['gpsfix']) == len(out['fix']) > 0
    for fix, msg in zip(out['fix'], out['gpsfix']):
        both_nan = math.isnan(msg['latitude']) and math.isnan(fix['latitude'])
        assert msg['latitude'] == fix['latitude'] or both_nan
        expected = GPSStatus.STATUS_NO_FIX if fix['status']['status'] < 0 else GPSStatus.STATUS_FIX
        assert msg['status']['status'] == expected


def test_every_mapped_status_exists_in_installed_gps_msgs():
    from unicore_rtk_driver.processor import GGA_TO_GPS_STATUS
    defined = {getattr(GPSStatus, n) for n in dir(GPSStatus) if n.startswith('STATUS_')}
    assert set(GGA_TO_GPS_STATUS.values()) <= defined
