import json
import os

from rtcm_helpers import message
from unicore_rtk_driver.tools.probe import analyse, report, sentence_name

FIXTURE = os.path.join(os.path.dirname(__file__), 'data', 'alpha_rise_2025_11_20_60s.json')


def recorded_bytes(n=40):
    with open(FIXTURE) as f:
        return b''.join(line.encode() for _, _, line in json.load(f)['lines'][:n])


def test_sentence_name():
    assert sentence_name('$GNGGA,1,2*00') == 'GGA'
    assert sentence_name('#UNIHEADINGA,93,GPS;SOL*00') == 'UNIHEADINGA'
    assert sentence_name('$command,VERSIONA,response: OK*00') == 'command'


def test_recorded_um982_output_is_recognised():
    r = analyse(recorded_bytes(), 115200)
    assert {'GGA', 'RMC', 'HPR'} <= set(r.nmea)
    assert {'UNIHEADINGA', 'AGRICA'} <= set(r.unicore)
    assert r.bad_checksums == 0 and r.looks_unicore and r.has_heading
    text = report(r, '/dev/ttyUSB0')
    assert 'receiver: unicore' in text and 'baudrate: 115200' in text


def test_generic_base_with_rtcm():
    data = b'$GNGGA,173144.00,,,,,0,00,9999.0,,,,,,*7E\r\n' + message(1005) + message(1074)
    r = analyse(data, 57600)
    assert not r.looks_unicore and not r.has_heading
    assert r.rtcm == {1005: 1, 1074: 1}
    assert 'configured as a base' in report(r, '/dev/x')


def test_wrong_baud_is_noise():
    r = analyse(bytes(range(256)) * 4, 9600)
    assert r.valid_frames == 0
