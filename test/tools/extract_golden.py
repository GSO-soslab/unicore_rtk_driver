#!/usr/bin/env python3
"""Build a regression fixture from a rosbag recorded with the driver running.

The bag must contain the driver's raw `nmea` topic (every line the receiver sent,
NMEA and Unicore) plus the output topics.  The fixture stores the raw lines with
their arrival stamps and the outputs the driver *actually published* at the time,
so tests can replay the lines through the current code and compare.

Usage:
    python3 extract_golden.py <bag_dir> <namespace/node> <seconds> <out.json>
    e.g. extract_golden.py rosbag2_2025_11_20 /alpha_rise/unicore_rtk_driver 60 golden.json

nmea_msgs/Sentence messages are decoded directly from their CDR encoding.
"""
import json
import os
import struct
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

OUTPUTS = ['fix', 'heading_odometry', 'heading_enu_pose', 'simple_heading',
           'time_ref', 'global_velocity', 'unicore_velocity']


def decode_sentence(data):
    """Decode a CDR nmea_msgs/Sentence without needing the package."""
    def rstr(o):
        o = (o + 3) & ~3
        n = struct.unpack_from('<I', data, o)[0]
        o += 4
        return data[o:o + n - 1].decode('ascii', errors='replace'), o + n
    sec, nsec = struct.unpack_from('<iI', data, 4)
    _, o = rstr(12)
    sentence, _ = rstr(o)
    return sec, nsec, sentence


def main(bag, ns, seconds, out):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = {f'{ns}/{o}': o for o in OUTPUTS if f'{ns}/{o}' in types}
    wanted[f'{ns}/nmea'] = 'nmea'
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(wanted)))

    lines, expected, t0 = [], {o: [] for o in OUTPUTS}, None
    while reader.has_next():
        topic, data, t = reader.read_next()
        t0 = t0 if t0 is not None else t
        if (t - t0) * 1e-9 > seconds:
            break
        name = wanted[topic]
        if name == 'nmea':
            lines.append(decode_sentence(data))
        else:
            msg = deserialize_message(data, get_message(types[topic]))
            expected[name].append(json.loads(json.dumps(message_to_ordereddict(msg))))

    with open(out, 'w') as f:
        json.dump({'source_bag': os.path.basename(os.path.normpath(bag)), 'namespace': ns, 'lines': lines,
                   'expected': expected}, f, separators=(',', ':'))
    print(f'{len(lines)} lines, ' + ', '.join(f'{k}={len(v)}' for k, v in expected.items()))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], float(sys.argv[3]), sys.argv[4])
