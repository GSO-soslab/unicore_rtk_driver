import pytest

from unicore_rtk_driver.receivers import (BaseStationSettings, LogRequest, OperatingMode,
                                          RoverSettings, RtcmMessage, create_receiver, parse_log_list,
                                          parse_rtcm_message_list)
from unicore_rtk_driver.receivers.unicore import DEFAULT_LOGS

LOGS = [LogRequest('GPGGA', 1), LogRequest('UNIHEADINGA', 0.5)]
ROVER = RoverSettings('MODE ROVER')


def test_unicore_survey_in():
    rx = create_receiver('unicore')
    base = BaseStationSettings(survey_in_min_duration_s=300,
                               rtcm_messages=[RtcmMessage(1006, 10), RtcmMessage(1074, 1)])
    assert rx.configuration_commands(OperatingMode.BASE, ROVER, base, LOGS) == [
        'MODE BASE TIME 300', 'RTCM1006 10', 'RTCM1074 1', 'GPGGA 1', 'UNIHEADINGA 0.5']


def test_unicore_fixed_base_on_named_port():
    rx = create_receiver('unicore', 'COM1')
    base = BaseStationSettings(use_fixed_position=True, fixed_latitude_deg=41.49126464,
                               fixed_longitude_deg=-71.42186016, fixed_altitude_m=-14.11,
                               rtcm_messages=[RtcmMessage(1006, 10)])
    assert rx.configuration_commands(OperatingMode.BASE, ROVER, base, LOGS[:1]) == [
        'MODE BASE 41.4912646400 -71.4218601600 -14.1100', 'RTCM1006 COM1 10', 'GPGGA COM1 1']


def test_unicore_rover_and_replies():
    rx = create_receiver('unicore')
    assert rx.configuration_commands(OperatingMode.ROVER, ROVER, BaseStationSettings(), LOGS) == [
        'MODE ROVER', 'GPGGA 1', 'UNIHEADINGA 0.5']
    assert rx.is_command_response('$command,MODE ROVER,response: OK*7C')
    assert not rx.is_command_response('$GNGGA,...')


def test_generic_sends_nothing():
    rx = create_receiver('generic')
    assert rx.configuration_commands(OperatingMode.BASE, ROVER, BaseStationSettings(), LOGS) == []
    assert rx.save_configuration_command() == ''


def test_unknown_receiver():
    with pytest.raises(ValueError):
        create_receiver('ublox')


def test_rtcm_message_list():
    assert parse_rtcm_message_list(['1006:10', ' 1074 ', '']) == [
        RtcmMessage(1006, 10.0), RtcmMessage(1074, 1.0)]


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
    assert rx.configuration_commands(OperatingMode.ROVER, RoverSettings('MODE ROVER UAV'),
                                     BaseStationSettings(), LOGS[:1]) == ['MODE ROVER UAV', 'GPGGA 1']
    assert rx.configuration_commands(OperatingMode.ROVER, RoverSettings(''),
                                     BaseStationSettings(), LOGS[:1]) == ['GPGGA 1']
    base = BaseStationSettings(mode_command='MODE BASE 1 TIME 60 1.5', rtcm_messages=[])
    assert rx.configuration_commands(OperatingMode.BASE, ROVER, base, []) == ['MODE BASE 1 TIME 60 1.5']
