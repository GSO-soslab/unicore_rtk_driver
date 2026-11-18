"""Tests that the node exits cleanly on the stop signals it receives from a terminal, launch and systemd."""
import signal
import socket
import subprocess
import sys
import time

import pytest


def free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def wait_for_port(port, timeout_s=20.0):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        try:
            socket.create_connection(('127.0.0.1', port), timeout=0.2).close()
            return
        except OSError:
            time.sleep(0.1)
    raise TimeoutError(f'node did not open port {port}')


@pytest.mark.parametrize('signals', [
    (signal.SIGINT,),                   # Ctrl-C on ros2 run
    (signal.SIGINT, signal.SIGINT),     # Ctrl-C under launch: terminal and launch both signal
    (signal.SIGTERM,),                  # systemctl stop
    (signal.SIGTERM, signal.SIGINT),    # systemctl stop of ros2 launch: systemd, then launch
])
def test_node_exits_cleanly(signals):
    port = free_port()
    node = subprocess.Popen(
        [sys.executable, '-m', 'unicore_rtk_driver.driver_node', '--ros-args',
         '-r', '__ns:=/shutdown_test', '-p', 'device:=/dev/null_no_receiver', '-p', 'mode:=base',
         '-p', f'rtcm.outputs:=["tcpserver://127.0.0.1:{port}"]'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        wait_for_port(port)
        time.sleep(0.5)  # node is spinning
        for sig in signals:
            node.send_signal(sig)
            time.sleep(0.005)
        output = node.communicate(timeout=20)[0]
    finally:
        node.kill()
    assert node.returncode == 0, output
    assert 'Traceback' not in output, output
