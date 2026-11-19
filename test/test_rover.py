"""Tests for the rover tool: GGA/GST/HPR summaries, configuration, RTCM input, status lines."""
import os
import pty
import select
import socket
import threading
import time
import tty

from rtcm_helpers import message, station_1005
from unicore_rtk_driver.protocol.checksums import nmea_checksum
from unicore_rtk_driver.protocol.solution import parse_gga, parse_gst, parse_hpr
from unicore_rtk_driver.tools import rover
from unicore_rtk_driver.tools.common import Logger


def nmea(body):
    return f'${body}*{nmea_checksum(body):02X}\r\n'


GGA_FIXED = nmea('GNGGA,173230.00,4129.4758,N,07125.3116,W,4,26,0.6,16.28,M,-30.39,M,1.0,0000')
GGA_SINGLE = nmea('GNGGA,173145.00,4129.4758,N,07125.3116,W,1,24,0.7,16.30,M,-30.39,M,,')
HPR = nmea('GNHPR,173230.00,113.84,-01.36,000.00,4,16,0.00,0999')
GST = nmea('GNGST,173230.00,0.02,0.010,0.008,156.0,0.009,0.012,0.020')


def test_gga_summary():
    fixed, single = parse_gga(GGA_FIXED), parse_gga(GGA_SINGLE)
    assert (fixed.solution, fixed.satellites, fixed.hdop) == ('RTK fixed', 26, 0.6)
    assert (fixed.correction_age_s, fixed.station_id) == (1.0, '0000')
    assert (single.solution, single.correction_age_s, single.station_id) == ('single', None, '')
    assert parse_gga(HPR) is None


def test_gst_summary():
    gst = parse_gst(GST)
    assert (gst.lat_std_m, gst.lon_std_m, gst.alt_std_m) == (0.009, 0.012, 0.020)
    assert abs(gst.horizontal_std_m - 0.015) < 1e-9
    assert parse_gst(GGA_FIXED) is None


def test_hpr_summary():
    hpr = parse_hpr(HPR)
    assert (hpr.heading_deg, hpr.pitch_deg, hpr.solution) == (113.84, -1.36, 'RTK fixed')
    assert parse_hpr(GGA_FIXED) is None


def test_invalid_settings_are_rejected():
    assert rover.main(['--device', '/dev/null_no_receiver', '--rtk-timeout-s', '5000']) == 2


def test_status_line_without_inputs():
    r = rover.Rover('/dev/null_no_receiver', 115200, [], Logger('test'))
    line = r.status_line(parse_gga(GGA_SINGLE))
    assert line.startswith('17:31:45  position single')
    assert 'acc -' in line and 'corr age -' in line and 'base -' in line
    assert 'rtcm in' not in line


def test_rover_configures_receiver_forwards_rtcm_and_prints_heading():
    master, slave = pty.openpty()
    tty.setraw(slave)
    base = socket.socket()
    base.bind(('127.0.0.1', 0))
    base.listen(1)
    lines, stop = [], threading.Event()
    argv = ['--device', os.ttyname(slave), '--configure', '--rtk-timeout-s', '30',
            '--command', 'CONFIG', '--input', f'tcp://127.0.0.1:{base.getsockname()[1]}']
    t = threading.Thread(target=rover.main, args=(argv, stop, lines.append), daemon=True)
    t.start()
    try:
        base.settimeout(10)
        conn, _ = base.accept()
        frames = [station_1005(7, 1.0e6, -4.6e6, 4.2e6), message(1074)]
        conn.sendall(b'junk' + b''.join(frames))

        received, end = b'', time.time() + 5
        while time.time() < end and not all(f in received for f in frames):
            if select.select([master], [], [], 0.1)[0]:
                received += os.read(master, 4096)
        # the inputs start after the configuration: commands first, then the frames
        commands = (b'MODE ROVER\r\nGPGGA 1\r\nGPGST 1\r\nGPHPR 1\r\n'
                    b'CONFIG RTK TIMEOUT 30\r\nCONFIG\r\n')
        assert received == commands + b''.join(frames)

        os.write(master, (HPR + GST + GGA_FIXED).encode())
        end = time.time() + 5
        while time.time() < end and not lines:
            time.sleep(0.05)
        assert lines and lines[0].startswith('17:32:30  position RTK fixed')
        assert 'acc 0.015 m' in lines[0] and 'base 0000' in lines[0]
        assert 'heading 113.84 deg (antennas RTK fixed)' in lines[0]
    finally:
        stop.set()
        t.join(timeout=5)
        base.close()
        os.close(master)
        os.close(slave)
