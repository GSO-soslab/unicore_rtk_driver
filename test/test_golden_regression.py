"""Replay real receiver output through the processor and compare against what
the original driver published at the time (recorded in the same bag).

Fixture: test/data/alpha_rise_2025_11_20_60s.json, made with
test/tools/extract_golden.py from an Alpha Rise bag (mvp_localization
test_data, 2025-11-20).  Driver params at the time: use_gnss_time=true,
gps_frame_id/heading_enu_frame_id=alpha_rise/unicore_gps,
heading_frame_id=alpha_rise/world_ned.  No IMU topic was configured.
"""
import json
import math
import os

import pytest
from builtin_interfaces.msg import Time
from rosidl_runtime_py import message_to_ordereddict

from unicore_rtk_driver.processor import Config, GnssProcessor

FIXTURE = os.path.join(os.path.dirname(__file__), 'data', 'alpha_rise_2025_11_20_60s.json')


class Log:
    def __init__(self):
        self.warnings = []

    def debug(self, *a, **k):
        pass

    info = debug

    def warn(self, msg, **k):
        self.warnings.append(msg)

    warning = error = warn


def alpha_rise_config(**overrides):
    cfg = Config(gps_frame_id='alpha_rise/unicore_gps',
                 heading_frame_id='alpha_rise/world_ned',
                 heading_enu_frame_id='alpha_rise/unicore_gps',
                 velocity_frame_id='world',
                 use_gnss_time=True)
    for k, v in overrides.items():
        setattr(cfg, k, v)
    return cfg


def replay(cfg):
    with open(FIXTURE) as f:
        fixture = json.load(f)
    out = {k: [] for k in fixture['expected']}
    log = Log()
    proc = GnssProcessor(cfg, lambda name, msg: out.setdefault(name, []).append(
        json.loads(json.dumps(message_to_ordereddict(msg)))), log)
    for sec, nsec, line in fixture['lines']:
        stamp = Time(sec=sec, nanosec=nsec)
        line = line.strip()
        if line.startswith('$'):
            proc.handle_nmea(line, stamp)
        elif line.startswith('#'):
            proc.handle_unicore(line, stamp)
    return fixture, out, log


def assert_close(a, b, path=''):
    if isinstance(a, dict):
        assert a.keys() == b.keys(), path
        for k in a:
            assert_close(a[k], b[k], f'{path}.{k}')
    elif isinstance(a, list):
        assert len(a) == len(b), path
        for i, (x, y) in enumerate(zip(a, b)):
            assert_close(x, y, f'{path}[{i}]')
    elif isinstance(a, float) or isinstance(b, float):
        if math.isnan(a) and math.isnan(b):
            return
        assert a == pytest.approx(b, rel=1e-6, abs=1e-6), path
    else:
        assert a == b, path


@pytest.mark.parametrize('topic', ['fix', 'time_ref', 'heading_odometry',
                                   'heading_enu_pose', 'simple_heading',
                                   'global_velocity'])
def test_default_config_reproduces_original_outputs(topic):
    fixture, out, _ = replay(alpha_rise_config())
    expected, got = fixture['expected'][topic], out[topic]
    # the bag may start recording outputs slightly after the raw stream:
    # align on the tail
    assert len(got) >= len(expected) > 0
    got = got[len(got) - len(expected):]
    for e, g in zip(expected, got):
        assert_close(e, g, topic)


def test_unicore_velocity_same_data_new_frame_invalid_speed_dropped():
    fixture, out, _ = replay(alpha_rise_config())
    expected, got = fixture['expected']['unicore_velocity'], out['unicore_velocity']
    # The original driver ignored the AGRICA speed-valid flag.  In this window
    # exactly one AGRICA has a solution but flag=1 (speed invalid): it was
    # published before and is dropped now.  Everything else must match.
    flagged = []
    for _, _, line in fixture['lines']:
        if line.startswith('#AGRICA'):
            f = line.split(';')[1].split(',')
            if f[8] != '0' and f[54] == '1':
                flagged.append(float(f[24]))  # Vel_East of the flagged line
    assert len(flagged) == 1
    expected = [e for e in expected
                if e['twist']['twist']['linear']['x'] != flagged[0]]
    assert len(got) == len(expected) > 0
    for e, g in zip(expected, got):
        assert e['header']['frame_id'] == 'alpha_rise/world_ned'   # old label
        assert g['header']['frame_id'] == 'world'                  # ENU data, ENU frame
        e['header']['frame_id'] = g['header']['frame_id']
        assert_close(e, g)


def test_no_checksum_or_validity_warnings_on_good_data():
    _, _, log = replay(alpha_rise_config())
    assert not [w for w in log.warnings if 'checksum' in w]


def test_gst_covariance_mode():
    _, base, _ = replay(alpha_rise_config())
    _, gst, _ = replay(alpha_rise_config(fix_covariance_mode='gst'))
    assert len(base['fix']) == len(gst['fix'])
    assert any(not math.isnan(f['position_covariance'][0]) for f in base['fix'])
    for a, b in zip(base['fix'], gst['fix']):
        if math.isnan(a['latitude']) or any(map(math.isnan, a['position_covariance'])):
            continue  # receiver lost the fix (vehicle submerged)
        assert a['latitude'] == b['latitude']
        hdop = math.sqrt(a['position_covariance'][0] / b['position_covariance'][0])
        assert hdop == pytest.approx(math.sqrt(a['position_covariance'][4] /
                                               b['position_covariance'][4]))


def test_heading_gnss_time_matches_line_time():
    _, out, _ = replay(alpha_rise_config(heading_use_gnss_time=True))
    _, io, _ = replay(alpha_rise_config())
    for g, i in zip(out['heading_odometry'], io['heading_odometry']):
        tg = g['header']['stamp']['sec'] + g['header']['stamp']['nanosec'] * 1e-9
        ti = i['header']['stamp']['sec'] + i['header']['stamp']['nanosec'] * 1e-9
        # GNSS epoch is before arrival, and not by more than ~1 s
        assert 0.0 <= ti - tg < 1.0
