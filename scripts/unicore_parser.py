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

def convert_deg_to_rads(degs):
    """Convert an angle in degrees to radians.

    This wrapper is needed because math.radians doesn't accept non-numeric inputs.

    Args:
        degs (float, int, or str): Angle in degrees

    Returns:
        The value of safe_float(degs) converted from degrees to radians.
    """
    return math.radians(safe_float(degs))

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

pose_type = {
    'NONE': 0, # No solution
    'FIXEDPOS': 1, # Position fixed by the FIX POSITION command
    'FIXEDHEIGHT': 2, # Not supported currently
    'DOPPLER_VELOCITY': 8, # Velocity computed using instantaneous Doppler
    'SINGLE' : 16,  # Single point positioning
    'PSRDIFF' : 17, # Pseudorange differential solution
    'SBAS' : 18, # SBAS positioning
    'L1_FLOAT' : 32, # L1 float solution
    'IONOFREE_FLOAT' : 33, # Ionosphere-free float solution
    'NARROW_FLOAT' :34, # Narrow-lane float solution
    'L1_INT' : 48, # L1 fixed solution
    'WIDE_INT' : 49, # Wide-lane fixed solution
    'NARROW_INT' : 50, # Narrow-lane fixed solution
    'INS' : 52, # Inertial navigation solution
    'INS_PSRSP' : 53, # Integrated solution of INS and single point positioning
    'INS_PSRDIFF' : 54, # Integrated solution of INS and pseudorange differential positioning
    'INS_RTKFLOAT' : 55, # Integrated solution of INS and RTK float
    'INS_RTKFIXED' : 56, # Integrated solution of INS and RTK fix
    'PPP_CONVERGING' : 68, # PPP solution converging
    'PPP' : 69 # Precise Point Positioning
} 

def convert_pose_type(field):
    """Convert a UNICORE Position or Velocity type into Enum

    Args:
        field (str): Table 0-4 Position or Velocity Type

    Returns:
        0,1,2,8...
    """

    return pose_type(field, -1)
    
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
        ("CPU_Idle", safe_int, 1),
        # ID:4 Reference time (GPST or BDST)
        ("Time_Ref", str, 2),
        # ID:5 GPS time quality indicator. The value is Unknown or
        #      Fine, of which the former indicates that the receiver
        #      has not yet calculated the precise GPS time.
        ("Time_Status", str, 3),
        # ID:6 GPS week number
        ("Week_Num", safe_int, 4),
        # ID:7 GPS seconds of week (ms)
        ("Week_MS", safe_int, 5),
        # ID:8 reserved
        # ("reserved", safe_int, 6),
        # ID:9 Reserved for the Unicore format version number
        ("Version", safe_int, 7),
        # ID:10 Leap second
        ("Leap_Seccond", safe_int, 8),
        # ID:11 Output delay (Difference between the data output time
        #       and GNSS signal receiving time), ms
        ("Output_Delay", safe_int, 9),
    ],
    "UNIHEADINGA": [
        # ID:1 header
        # ID:2 Solution status, refer to Table 0-5 Solution Status
        #      0: Solution computed
        #      1: Insufficient observation
        #      2: No convergence, invalid solution
        #      3: Covariance matrix trace exceeds maximum (trace > 1000 m)
        ("Sol_Stat", convert_solution_status, 0),
        # ID:3 Position type, refer to Table 0-4, Position or Velocity Type
        #      0: No solution
        #      1: Position fixed by the FIX POSITION command
        #      2: Not supported currently
        #      8: Velocity computed using instantaneous Doppler       
        #      16: Single point positioning
        #      ...
        ("Pose_Type", convert_pose_type, 1),
        # ID:4 Baseline length
        ("Base_Len", safe_float, 2),
        # ID:5 Heading (0 to 360.0 degrees)
        ("Heading", convert_deg_to_rads, 3),
        # ID:6 Pitch (± 90 degrees)
        ("Pitch", convert_deg_to_rads, 4),
        # ID:7 Reserved
        # ("Reserved", safe_float, 5),
        # ID:8 Standard deviation of heading
        ("Heading_Std", convert_deg_to_rads, 6),
        # ID:9 Standard deviation of pitch
        ("Pitch_Std", convert_deg_to_rads, 7),
        # ID:10 Base station ID
        ("Station_Id", safe_str, 8),
        # ID:11 Number of satellites tracked
        ("Num_Sat", safe_int, 9),
        # ID:12 Number of satellites used in solution
        ("Sol_Num_Sat", safe_int, 10),
        # ID:13 Number of satellites above the elevation mask angle
        ("Num_Obs", safe_int, 11),
        # ID:14 Number of satellites with L2 signal above the 
        #       elevation mask angle
        ("Num_Multi", safe_int, 12),
        # ID:15 Reserved
        # ("Reserved", safe_float, 13),
        # ID:16 Extended solution status, refer to Table 7-87 
        #       Extended Solution Status
        # FIXME simple convert to int, should do more test if real data exist
        ("Ext_Sol_Stat", safe_int, 14),
        # ID:17 Galileo and BDS-3 signal mask refer to Table 7-86 
        #       Galileo & BDS-3 Signal Mask
        # FIXME: what is the mask(0x01, 0x02... 0x80) means
        #       0, 0x01, GALILEO E1 used in solution
        #       1, 0x02, GALILEO E5b used in solution
        #       2, 0x04, GALILEO E5A used in solution
        #       3, 0x08, BDS3 B2b used in solution
        #       4, 0x10, BDS-3 B1I used in solution
        #       5, 0x20, BDS-3 B3I used in solution
        #       6, 0x40, BDS-3 B2a used in solution
        #       7, 0x80, BDS-3 B1C used in solution        
        ("Mask1", convert_mask, 15),
        # ID:18 GPS, GLONASS and BDS-2 signal mask 
        #       0, 0x01, GPS L1 used in solution
        #       1, 0x02, GPS L2 used in solution
        #       2, 0x04, GPS L5 used in solution
        #       3, 0x08, BDS-2 B3I used in solution
        #       4, 0x10, GLONASS L1 used in solution
        #       5, 0x20, GLONASS L2 used in solution
        #       6, 0x40, BDS-2 B1I used in solution
        #       7, 0x80, BDS-2 B2I used in solution        
        ("Mask2", convert_mask, 16),
    ],
    "AGRICA": [
        # ID:1 header
        # ID:2 GNSS
        ("GNSS", safe_str, 0),
        # Id:3 Command length, from GNSS to CRC, 232 bytes in total, 
        #      a fixed value of 0XE8
        ("Length", safe_int, 1),
        # ID:4 UTC-year, for example: 2016-> 16; 2116-> 116
        ("Year", safe_int, 2),
        # ID:5 UTC-month
        ("Month", safe_int, 3),
        # ID:6 UTC-day
        ("Day", safe_int, 4),
        # ID:7 UTC-hour
        ("Hour", safe_int, 5),
        # ID:8 UTC-minute
        ("Minute", safe_int, 6),
        # ID:9 UTC-second
        ("Second", safe_int, 7),
        # ID:10 Rover position status:
        #       0: Invalid solution;
        #       1: Single point solution;
        #       2: Pseudorange differential
        #       solution;
        #       4: Fixed solution;
        #       5: Float solution
        #       7: Input a fixed position (only supported by specific versions)
        ("Pose_Type", safe_int, 8),
        # ID:11 Heading solution status of master and slave antennas
        #       0: Invalid solution;
        #       4: Fixed solution;
        #       5: Float solution
        ("Heading_Status", safe_int, 9),
        # ID:12 Number of GPS satellites used in the solution
        ("Num_GPS", safe_int, 10),
        # ID:13 Number of BeiDou System (BDS) satellites used in the solution
        ("Num_BDS", safe_int, 11),
        # ID:14 Number of GLONASS satellites used in the solution
        ("Num_GLO", safe_int, 12),
        # ID:15 Baseline vector from the base station to the rover station, 
        #       northern component
        ("Trans_North", safe_float, 13),
        # ID:16 Baseline vector from the base station to the rover station, 
        #       eastern component
        ("Trans_East", safe_float, 14),
        # ID:17 Baseline vector from the base station to the rover station, 
        #       vertical component
        ("Trans_Up", safe_float, 15),
        # ID:18 Baseline vector from the base station to the rover station, 
        #       northern component standard deviation
        ("Trans_Std_North", safe_float, 16),
        # ID:19 Baseline vector from the base station to the rover station, 
        #       eastern component standard deviation
        ("Trans_Std_East", safe_float, 17),
        # ID:20 Baseline vector from the base station to the rover station, 
        #       vertical component standard deviation
        ("Trans_Std_Up", safe_float, 18),
        # ID:21 Heading
        ("Heading", safe_float, 19),
        # ID:22 Pitch
        ("Pitch", safe_float, 20),
        # ID:23 Roll
        ("Roll", safe_float, 21),
        # ID:24 Speed, scalar
        # FIXME what is unit?
        ("Speed", safe_float, 22),
        # ID:25 North velocity
        ("Vel_North", safe_float, 23),
        # ID:26 East velocity
        ("Vel_East", safe_float, 24),
        # ID:27 Up velocity
        ("Vel_Up", safe_float, 25),
        # ID:28 North velocity standard deviation
        ("Vel_Std_North", safe_float, 26),
        # ID:29 East velocity standard deviation
        ("Vel_Std_East", safe_float, 27),
        # ID:30 Up velocity standard deviation
        ("Vel_Std_Up", safe_float, 28),
        # ID:31 Latitude of the rover station: -90~90 degrees, 
        #       positive for the North and negative for the South
        ("Lat", safe_float, 29),
        # ID:32 Longitude of the rover station: -180~180 degrees, 
        #       positive for the East and negative for the West
        ("Lon", safe_float, 30),
        # ID:33 Height of the rover station
        ("Alt", safe_float, 31),
        # ID:34 X axis of the ECEF coordinate system
        ("ECEF_X", safe_float, 32),
        # ID:35 Y axis of the ECEF coordinate system
        ("ECEF_Y", safe_float, 33),
        # ID:36 Z axis of the ECEF coordinate system
        ("ECEF_Z", safe_float, 34),
        # ID:37 Latitude standard deviation
        ("Lat_Std", safe_float, 35),
        # ID:38 Longitude standard deviation
        ("Lon_Std", safe_float, 36),
        # ID:39 Height standard deviation
        ("Alt_Std", safe_float, 37),
        # ID:40 ECEF_X standard deviation
        ("ECEF_X_Std", safe_float, 38),
        # ID:41 ECEF_Y standard deviation
        ("ECEF_Y_Std", safe_float, 39),
        # ID:42 ECEF_Z standard deviation
        ("ECEF_Z_Std", safe_float, 40),
        # ID:43 Latitude of the base station: -90~90 degrees
        ("Base_Lat", safe_float, 41),
        # ID:44 Longitude of the base station: -180~180 degrees
        ("Base_Lon", safe_float, 42),
        # ID:45 Height of the base station
        ("Base_Alt", safe_float, 43),
        # ID:46 Latitude of the slave antenna: -90~90 degrees
        ("Secondary_Lat", safe_float, 44),
        # ID:47 Longitude of the slave antenna: -180~180 degrees
        ("Secondary_Lon", safe_float, 45),
        # ID:48 Height of the slave antenna
        ("Secondary_Alt", safe_float, 46),
        # ID:49 Milliseconds of GPS week
        ("Week_MS", safe_int, 47),
        # ID:50 Differential age
        ("Diff_Age", safe_float, 48),
        # ID:51 Direction of velocity
        ("Speed_Heading", safe_float, 49),
        # ID:52 Undulation
        ("Undulation", safe_float, 50),
        # ID:53 Reserved
        # ("Reserved1", safe_int, 51),
        # ID:54 Reserved
        # ("Reserved2", safe_int, 52),
        # ID:55 Number of Galileo satellites
        ("Num_GAL", safe_float, 53),
        # ID:56 0: speed solution status valid
        #       1: speed solution status invalid
        ("Speed_Vaild", safe_float, 54),
        # ID:57 Reserved
        # ("Reserved3", safe_int, 55),
        # ID:58 Reserved
        # ("Reserved4", safe_int, 56),
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

