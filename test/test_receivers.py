import pytest

from unicore_rtk_driver.receivers import (LogRequest, RoverSettings, create_receiver,
                                          parse_log_list)
from unicore_rtk_driver.receivers.unicore import DEFAULT_LOGS

LOGS = [LogRequest('GPGGA', 1), LogRequest('UNIHEADINGA', 0.5)]
ROVER = RoverSettings('MODE ROVER')


def test_unicore_rover_and_replies():
    rx = create_receiver('unicore')
    assert rx.configuration_commands(ROVER, LOGS) == ['MODE ROVER', 'GPGGA 1', 'UNIHEADINGA 0.5']
    assert rx.is_command_response('$command,MODE ROVER,response: OK*7C')
    assert not rx.is_command_response('$GNGGA,...')


def test_unicore_logs_on_named_port():
    rx = create_receiver('unicore', 'COM1')
    assert rx.configuration_commands(ROVER, LOGS[:1]) == ['MODE ROVER', 'GPGGA COM1 1']


def test_generic_sends_nothing():
    rx = create_receiver('generic')
    assert rx.configuration_commands(ROVER, LOGS) == []
    assert rx.save_configuration_command() == ''


def test_unknown_receiver():
    with pytest.raises(ValueError):
        create_receiver('ublox')


def test_log_list():
    assert parse_log_list(['gpgga:1', 'UNIHEADINGA:0.5', '', 'AGRICA']) == [
        LogRequest('GPGGA', 1.0), LogRequest('UNIHEADINGA', 0.5), LogRequest('AGRICA', 1.0)]
    with pytest.raises(ValueError):
        parse_log_list(['GPGGA 1; RESET:1'])


def test_default_logs_cover_every_published_topic():
    names = {log.name for log in DEFAULT_LOGS}
    assert {'GPGGA', 'GPGST', 'GPRMC', 'UNIHEADINGA', 'AGRICA'} <= names


def test_mode_commands_are_configurable():
    rx = create_receiver('unicore')
    assert rx.configuration_commands(RoverSettings('MODE ROVER UAV'), LOGS[:1]) == [
        'MODE ROVER UAV', 'GPGGA 1']
    assert rx.configuration_commands(RoverSettings(''), LOGS[:1]) == ['GPGGA 1']
