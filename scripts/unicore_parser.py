"""Parsing functions for UNICORE sentence strings."""

import re
import logging
import math

logger = logging.getLogger('rosout')
field_delimiter_regex = re.compile(r'[,*]')

def safe_float(field):
    """Convert  field to a float.

    Args:
        field: The field (usually a str) to convert to float.

    Returns:
        The float value represented by field or NaN if float conversion throws a ValueError.
    """
    try:
        return float(field)
    except ValueError:
        return float('NaN')

def safe_int(field):
    """Convert field to an int.

    Args:
        field: The field (usually a str) to convert to int.

    Returns:
        The int value represented by field or 0 if int conversion throws a ValueError.
    """
    try:
        return int(field)
    except ValueError:
        return 0
    
def safe_str(field):
    """Convert field to an str.

    Args:
        field: The field (usually a str) to convert to str, 
               some data not available will display as Quotation Marks("")

    Returns:
        empty str or actual str
    """
    if field == "\"\"":
        return ""
    else:
        return field

def convert_solution_status(field):
    """Convert a UNICORE Solution Status into Enum

    Args:
        field (str): check Table 0-5 Solution Status

    Returns:
        0,1,2,4
    """

    if field == "SOL_COMPUTED": # Solution computed
        return 0
    elif field == "INSUFFICIENT_OBS": # Insufficient observation
        return 1
    elif field == "NO_CONVERGENCE": # No convergence, invalid solution
        return 2
    elif field == "COV_TRACE": # Covariance matrix trace exceeds maximum (trace > 1000 m)
        return 4
    else:
        return -1 # something wrong

def convert_pos_type(field):
    """Convert a UNICORE Position or Velocity type into Enum

    Args:
        field (str): Table 0-4 Position or Velocity Type

    Returns:
        0,1,2,8
    """

    if field == "NONE": # No solution
        return 0
    elif field == "FIXEDPOS": # Position fixed by the FIX POSITION command
        return 1
    elif field == "FIXEDHEIGHT": # Not supported currently
        return 2
    elif field == "DOPPLER_VELOCITY": # Velocity computed using instantaneous Doppler
        return 8    
    else:
        return -1 # something wrong
    
def convert_mask(field):
    """Convert a UNICORE Galileo & BDS-3 Signal Mask to Enum

    Args:
        field (str): 
            check Table 7-86 Galileo & BDS-3 Signal Mask or 
            Table 7-85 GPS/GLONASS/BDS-2 Signal Mask

    Returns:
        each bit from 0~7 order
    """

    bits = []
    for i in range(8):
        bits.append((safe_int(field) >> i) & 1)
    return bits

parse_maps = {
    "HEAD": [
        # ID:1 '#', Sync character. The ASCII message always starts with a “#”.
        # ID:2 The ASCII name of the log or command in this manual.
        # Both ID:1 and ID:2 are sticked together
        # ("Message", str, 0),
        # ID:3 The minimum percentage of time that the processor is idle, 
        #      calculated once per second.
        ("CPUIdle", safe_int, 1),
        # ID:4 Reference time (GPST or BDST)
        ("TimeRef", str, 2),
        # ID:5 GPS time quality indicator. The value is Unknown or
        #      Fine, of which the former indicates that the receiver
        #      has not yet calculated the precise GPS time.
        ("TimeStatus", str, 3),
        # ID:6 GPS week number
        ("WeekNum", safe_int, 4),
        # ID:7 GPS seconds of week (ms)
        ("WeekSec", safe_int, 5),
        # ID:8 reserved
        # ("reserved", safe_int, 6),
        # ID:9 Reserved for the Unicore format version number
        ("Version", safe_int, 7),
        # ID:10 Leap second
        ("LeapSec", safe_int, 8),
        # ID:11 Output delay (Difference between the data output time
        #       and GNSS signal receiving time), ms
        ("OutputDelay", safe_int, 9),
    ],
    "UNIHEADINGA": [
        # ID:1 header
        # ID:2 Solution status, refer to Table 0-5 Solution Status
        ("SolStat", convert_solution_status, 0),
        # ID:3 Position type, refer to Table 0-4, Position or Velocity Type
        ("PosType", convert_pos_type, 1),
        # ID:4 Baseline length
        ("BaseLen", safe_float, 2),
        # ID:5 Heading (0 to 360.0 degrees)
        ("Heading", safe_float, 3),
        # ID:6 Pitch (± 90 degrees)
        ("Pitch", safe_float, 4),
        # ID:7 Reserved
        # ("Reserved", safe_float, 5),
        # ID:8 Standard deviation of heading
        ("HeadingStd", safe_float, 6),
        # ID:9 Standard deviation of pitch
        ("PitchStd", safe_float, 7),
        # ID:10 Base station ID
        ("StationId", safe_str, 8),
        # ID:11 Number of satellites tracked
        ("SatNum", safe_int, 9),
        # ID:12 Number of satellites used in solution
        ("SatNum", safe_int, 10),
        # ID:13 Number of satellites above the elevation mask angle
        ("ObsNum", safe_int, 11),
        # ID:14 Number of satellites with L2 signal above the 
        #       elevation mask angle
        ("MultiNum", safe_int, 12),
        # ID:15 Reserved
        # ("Reserved", safe_float, 13),
        # ID:16 Extended solution status, refer to Table 7-87 
        #       Extended Solution Status
        # FIXME simple convert to int, should do more test if real data exist
        ("ExtSolStat", safe_int, 14),
        # ID:17 Galileo and BDS-3 signal mask refer to Table 7-86 
        #       Galileo & BDS-3 Signal Mask
        # FIXME: what is the mask(0x01, 0x02... 0x80) means
        ("Mask1", convert_mask, 15),
        # ID:18 GPS, GLONASS and BDS-2 signal mask 
        #       (see Table 7-85 GPS/GLONASS/BDS-2 Signal Mask)
        ("Mask2", convert_mask, 16),
    ]    
}
"""A dictionary that maps from sentence identifier string (e.g. "GGA") to a list of tuples.
Each tuple is a three-tuple of (str: field name, callable: conversion function, int: field index).
The parser splits the sentence into comma-delimited fields. The string value of each field is passed
to the appropriate conversion function based on the field index."""

def parse_unicore_sentence(unicore_sentence):
    """Parse a UNICORE sentence string into a dictionary.

    Args:
        unicore_sentence (str): A single unicore sentence of one of the types, 
        which include a header part, a data part and CRC part

    Returns:
        a header and a data ?
    """

    # Cut possible carriage return or new line of NMEA Sentence
    unicore_sentence = unicore_sentence.strip()  

    # Get the header and data parts
    header, data = unicore_sentence.split(';')
    # DEBUG:
    # print(header)
    # print(data)

    # Get the separated header and data
    header_fields = [field for field in field_delimiter_regex.split(header)]
    data_fields = [field for field in field_delimiter_regex.split(data)]

    # Get the sentence type 
    sentence_type = header_fields[0][1:]
    if sentence_type not in parse_maps:
        logger.debug("Sentence type %s not in parse map, ignoring."
                     % repr(sentence_type))
        return False

    # parse the header part
    head_type= "HEAD"
    parse_head_map = parse_maps[head_type]
    parsed_head_sentence = {}
    for entry in parse_head_map:
        parsed_head_sentence[entry[0]] = entry[1](header_fields[entry[2]])

    # parse the data part
    parse_data_map = parse_maps[sentence_type]
    parsed_data_sentence = {}
    for entry in parse_data_map:
        parsed_data_sentence[entry[0]] = entry[1](data_fields[entry[2]])

    return {head_type: parsed_head_sentence, sentence_type: parsed_data_sentence}

