import json
import os
import socket
import threading
import time

from rtcm_helpers import lla_to_ecef, message, station_1005
from unicore_rtk_driver.tools.probe import (analyse, listen_stream, main, report,
                                            sentence_name, stream_report)

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


def test_missing_gst_hint_matches_receiver_type():
    hint = 'no GST: ~/fix is published on GST; enable it '
    generic = analyse(message(1005), 115200)
    assert hint + "with the receiver's own tools" in report(generic, '/dev/x')
    unicore = analyse(recorded_bytes(), 115200)
    unicore.nmea.pop('GST', None)
    assert hint + '(receiver_logs)' in report(unicore, '/dev/x')


def test_wrong_baud_is_noise():
    r = analyse(bytes(range(256)) * 4, 9600)
    assert r.valid_frames == 0


def _free_port(kind):
    with socket.socket(socket.AF_INET, kind) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def _send_while_listening(uri, send, duration=1.5):
    out = {}
    t = threading.Thread(target=lambda: out.update(data=listen_stream(uri, duration)))
    t.start()
    time.sleep(0.3)
    send()
    t.join()
    return out['data']


def test_stream_over_udp_reports_messages_and_base_position():
    port = _free_port(socket.SOCK_DGRAM)
    frames = [station_1005(5, *lla_to_ecef(41.5, -71.4, 10.0)), message(1074), message(1084)]

    def send():
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx:
            for f in frames:
                tx.sendto(f, ('127.0.0.1', port))

    data = _send_while_listening(f'udp://127.0.0.1:{port}', send)
    text = stream_report(f'udp://127.0.0.1:{port}', data, 1.5)
    assert '1005 x1' in text and '1074 x1' in text and '1084 x1' in text
    assert 'id 5: 41.50000000, -71.40000000' in text
    assert f"rtcm.inputs: ['udp://127.0.0.1:{port}']" in text


def test_stream_over_tcp_from_a_base_server():
    server = socket.socket()
    server.bind(('127.0.0.1', 0))
    server.listen(1)
    port = server.getsockname()[1]

    def send():
        server.settimeout(5)
        conn, _ = server.accept()
        conn.sendall(message(1074) + message(1124))
        time.sleep(0.2)
        conn.close()

    data = _send_while_listening(f'tcp://127.0.0.1:{port}', send)
    server.close()
    text = stream_report(f'tcp://127.0.0.1:{port}', data, 1.5)
    assert '1074 x1' in text and '1124 x1' in text and 'no RTCM 1005/1006' in text


def test_stream_without_data_exits_with_hint(capsys):
    port = _free_port(socket.SOCK_DGRAM)
    assert main([f'udp://127.0.0.1:{port}', '--duration', '0.5']) == 1
    assert 'No data from' in capsys.readouterr().out
