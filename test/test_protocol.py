import json
import os

import pytest

from rtcm_helpers import frame, message
from unicore_rtk_driver.protocol.checksums import (crc24q, nmea_checksum, nmea_is_valid,
                                                   unicore_crc32, unicore_is_valid)
from unicore_rtk_driver.protocol.framer import FrameType, StreamFramer

FIXTURE = os.path.join(os.path.dirname(__file__), 'data', 'alpha_rise_2025_11_20_60s.json')


def real_lines():
    with open(FIXTURE) as f:
        return [line.strip() for _, _, line in json.load(f)['lines']]


def table_crc32(text):
    """Reference implementation from the Unicore manual (table driven, init 0)."""
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ 0xEDB88320 if c & 1 else c >> 1
        table.append(c)
    crc = 0
    for b in text.encode():
        crc = table[(crc ^ b) & 0xFF] ^ (crc >> 8)
    return crc


def test_all_recorded_lines_pass_their_checksum():
    lines = real_lines()
    assert len(lines) > 400
    for line in lines:
        if line.startswith('$'):
            assert nmea_is_valid(line), line
        else:
            assert unicore_is_valid(line), line


def test_unicore_crc_matches_manual_algorithm():
    for line in real_lines():
        if line.startswith('#'):
            body = line[1:].rpartition('*')[0]
            assert unicore_crc32(body) == table_crc32(body)


def test_checksum_rejects_corruption():
    good = '$GNGGA,173144.00,4134.50833626,N,07147.29336922,W,1,28,0.5,81.8631,M,-30.6392,M,,*72'
    assert nmea_is_valid(good)
    assert not nmea_is_valid(good.replace('4134', '4135'))
    assert not nmea_is_valid(good.replace('*72', '*7'))
    assert not nmea_is_valid('$GNGGA,1*2*72')      # two '*' used to raise
    assert not nmea_is_valid('GNGGA,1*00')
    assert not unicore_is_valid('#UNIHEADINGA,1;2*zzzzzzzz')
    assert nmea_checksum('GPGGA') == ord('G') ^ ord('P') ^ ord('G') ^ ord('G') ^ ord('A')


def test_crc24q_known_value():
    # CRC-24Q of "123456789" is 0xCDE703
    assert crc24q(b'123456789') == 0xCDE703


def test_framer_splits_mixed_stream_in_any_chunking():
    lines = real_lines()[:20]
    rtcm = [message(1074, 30), message(1006, 15)]
    stream = b''
    for i, line in enumerate(lines):
        stream += line.encode() + b'\r\n'
        if i in (3, 11):
            stream += rtcm[i == 11]
    for chunk in (1, 7, 64, len(stream)):
        f = StreamFramer()
        frames = []
        for i in range(0, len(stream), chunk):
            frames += f.feed(stream[i:i + chunk])
        texts = [fr.text for fr in frames if fr.type is not FrameType.RTCM]
        bins = [fr.data for fr in frames if fr.type is FrameType.RTCM]
        assert texts == lines
        assert bins == rtcm
        assert f.dropped_bytes == 0


def test_framer_resyncs_after_garbage_and_bad_rtcm():
    good = message(1084, 20)
    bad = bytearray(message(1094, 20))
    bad[10] ^= 0xFF
    stream = b'\x00\xffnoise' + bytes(bad) + b'$GNVTG,1*00\r\n' + b'\xd3' + good
    frames = StreamFramer().feed(stream)
    assert [fr.type for fr in frames] == [FrameType.NMEA, FrameType.RTCM]
    assert frames[1].data == good


def test_framer_drops_overlong_line_and_binary_text():
    f = StreamFramer(max_line_length=50)
    frames = f.feed(b'$' + b'A' * 200 + b'\r\n$GNGGA,ok*00\r\n#\xff\xfe\r\n')
    assert [fr.text for fr in frames] == ['$GNGGA,ok*00']


def test_framer_can_ignore_rtcm():
    f = StreamFramer(parse_rtcm=False)
    assert f.feed(message(1005) + b'$GNGGA,x*00\r\n')[0].text == '$GNGGA,x*00'


@pytest.mark.parametrize('payload_len', [0, 1, 1023])
def test_rtcm_lengths(payload_len):
    fr = frame(bytes(payload_len))
    assert StreamFramer().feed(fr)[0].data == fr
