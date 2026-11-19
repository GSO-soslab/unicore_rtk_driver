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


def test_unicore_rover_settings_only_when_set():
    rx = create_receiver('unicore')
    assert rx.settings_commands(RoverSettings('MODE ROVER')) == []
    settings = RoverSettings('MODE ROVER', rtk_timeout_s=30, dgps_timeout_s=60,
                             heading_baseline_cm=100, heading_baseline_tolerance_cm=3)
    assert rx.settings_commands(settings) == [
        'CONFIG RTK TIMEOUT 30', 'CONFIG DGPS TIMEOUT 60', 'CONFIG HEADING LENGTH 100 3']
    assert rx.settings_commands(RoverSettings(heading_baseline_cm=100)) == [
        'CONFIG HEADING LENGTH 100']
    # the settings never change the mode and log commands
    assert rx.configuration_commands(settings, LOGS[:1]) == ['MODE ROVER', 'GPGGA 1']


@pytest.mark.parametrize('kwargs', [
    {'rtk_timeout_s': 1801}, {'dgps_timeout_s': -1}, {'heading_baseline_cm': -5},
    {'heading_baseline_tolerance_cm': 3},
])
def test_invalid_rover_settings(kwargs):
    with pytest.raises(ValueError):
        RoverSettings('MODE ROVER', **kwargs)


def test_configuration_report():
    rx = create_receiver('unicore')
    assert rx.config_query_command() == 'CONFIG'
    assert rx.is_config_report('$CONFIG,COM1,CONFIG COM1 460800*65')
    assert not rx.is_config_report('$command,CONFIG,response: OK*54')
    generic = create_receiver('generic')
    assert generic.settings_commands(RoverSettings(rtk_timeout_s=30)) == []
    assert generic.config_query_command() == ''
