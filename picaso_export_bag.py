#!/usr/bin/env python3
"""Export a ROS 2 PiCASo capture topic to raw files and a recorder-compatible CSV."""

import argparse
from array import array
import csv
from pathlib import Path
import sys


CSV_FIELDS = (
    'capture', 'file', 'mode', 'events', 'bytes',
    't_cmd_unix', 't_done_unix', 't_cmd_mono', 't_done_mono', 't_cmd_utc',
    'capture_id', 'header_stamp_ns', 'frame_id', 'bag_stamp_ns', 'image_stamp_ns',
)


def seconds_text(nanoseconds: int) -> str:
    """Format integer nanoseconds as seconds without float precision loss.

    :param nanoseconds: Nonnegative clock reading in nanoseconds.
    :return: Decimal seconds with nine fractional digits.
    """
    seconds, fraction = divmod(nanoseconds, 1_000_000_000)
    return f'{seconds}.{fraction:09d}'


def export_bag(bag: Path, topic: str, output: Path) -> int:
    """Stream recorded captures into a new directory without replaying the bag.

    :param bag: ROS 2 bag directory, including its metadata.yaml.
    :param topic: Recorded topic with type picaso_msgs/msg/Capture.
    :param output: Directory to create; must not already exist.
    :return: Number of exported captures.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from picaso_msgs.msg import Capture

    if not (bag / 'metadata.yaml').is_file():
        raise ValueError(f'Not a ROS 2 bag directory (missing metadata.yaml): {bag}')
    if output.exists():
        raise FileExistsError(f'Output already exists; choose a new directory: {output}')

    # Read the recorded storage plugin instead of assuming SQLite. Other bag
    # formats require their rosbag2 storage plugin in the running environment.
    metadata = rosbag2_py.Info().read_metadata(str(bag), '')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=metadata.storage_identifier),
                rosbag2_py.ConverterOptions('', ''))
    topics = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    if topic not in topics:
        raise ValueError(f'Topic {topic!r} not found. Recorded topics: {", ".join(sorted(topics))}')
    if topics[topic] != 'picaso_msgs/msg/Capture':
        raise ValueError(f'Topic {topic!r} has type {topics[topic]}, expected picaso_msgs/msg/Capture')
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    if not reader.has_next():
        raise ValueError(f'Topic {topic!r} has no recorded captures')

    output.mkdir(parents=True, exist_ok=False)
    count = 0
    with (output / 'timestamps.csv').open('x', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(CSV_FIELDS)
        while reader.has_next():
            _, serialized, bag_stamp = reader.read_next()
            message = deserialize_message(serialized, Capture)
            words = array('H', message.pdm_words)
            if message.byte_count != len(words) * 2 or len(words) != message.events * message.samples_per_event:
                raise ValueError(f'Invalid payload length in capture {message.capture_id}; '
                                 f'partial export retained in {output}')
            if sys.byteorder != 'little':
                words.byteswap()
            # Number files in bag order: capture_id can reset after a driver
            # restart. Preserve that original ID separately in the CSV.
            index = count + 1
            filename = f'capture_{index:04d}.bin'
            with (output / filename).open('xb') as binary:
                binary.write(words.tobytes())
            header_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            image_ns = (message.image_stamp.sec * 1_000_000_000 + message.image_stamp.nanosec
                        if message.has_image else '')
            writer.writerow((index, filename, message.mode, message.events, message.byte_count,
                             seconds_text(message.t_cmd_unix_ns), seconds_text(message.t_done_unix_ns),
                             seconds_text(message.t_cmd_mono_ns), seconds_text(message.t_done_mono_ns),
                             message.t_cmd_utc, message.capture_id, header_ns, message.header.frame_id,
                             bag_stamp, image_ns))
            stream.flush()
            count = index
    return count


def main() -> None:
    """Parse paths and topic, export offline, and report actionable errors."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path, help='ROS 2 bag directory containing metadata.yaml')
    parser.add_argument('--topic', required=True, help='Recorded picaso_msgs/msg/Capture topic')
    parser.add_argument('--output', type=Path, help='New output directory (default: <bag>_export beside bag)')
    args = parser.parse_args()
    output = args.output or args.bag.resolve().with_name(f'{args.bag.resolve().name}_export')
    try:
        count = export_bag(args.bag, args.topic, output)
    except ImportError as error:
        parser.exit(1, f'ROS dependencies unavailable: {error}. Source ROS 2 and the sonar workspace first.\n')
    except (OSError, ValueError, RuntimeError) as error:
        parser.exit(1, f'Export failed: {error}\n')
    print(f'Exported {count} captures to {output.resolve()}')


if __name__ == '__main__':
    main()
