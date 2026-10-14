#!/usr/bin/env python3
"""Export a ROS 1 mission folder or ROS 2 PiCASo bag to raw captures and CSV."""

import argparse
from array import array
import csv
from itertools import chain
from pathlib import Path
import sys


CSV_FIELDS = (
    'capture', 'file', 'mode', 'events', 'bytes',
    't_cmd_unix', 't_done_unix', 't_cmd_mono', 't_done_mono', 't_cmd_utc',
    'capture_id', 'header_stamp_ns', 'frame_id', 'bag_stamp_ns', 'image_stamp_ns', 'tag',
)


def seconds_text(nanoseconds: int) -> str:
    """Format integer nanoseconds as seconds without float precision loss.

    :param nanoseconds: Nonnegative clock reading in nanoseconds.
    :return: Decimal seconds with nine fractional digits.
    """
    seconds, fraction = divmod(nanoseconds, 1_000_000_000)
    return f'{seconds}.{fraction:09d}'


def write_captures(captures, output: Path) -> int:
    """Write a stream of (Capture, bag timestamp) pairs without overwriting files.

    :param captures: Deserialized messages in recording timestamp order.
    :param output: New combined export directory.
    :return: Number of exported captures.
    """
    if output.exists():
        raise FileExistsError(f'Output already exists; choose a new directory: {output}')
    captures = iter(captures)
    try:
        first = next(captures)
    except StopIteration:
        raise ValueError('The selected topic has no recorded captures') from None

    output.mkdir(parents=True, exist_ok=False)
    count = 0
    with (output / 'timestamps.csv').open('x', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(CSV_FIELDS)
        for message, bag_stamp in chain((first,), captures):
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
                             bag_stamp, image_ns, message.tag))
            stream.flush()
            count = index
    return count


def export_bag(bag: Path, topic: str, output: Path) -> int:
    """Read a ROS 2 bag with its recorded storage plugin.

    :param bag: ROS 2 bag directory, including its metadata.yaml.
    :param topic: Recorded picaso_msgs/msg/Capture topic.
    :param output: New export directory.
    :return: Number of exported captures.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from picaso_msgs.msg import Capture

    if not (bag / 'metadata.yaml').is_file():
        raise ValueError(f'Not a ROS 2 bag directory (missing metadata.yaml): {bag}')
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

    def captures():
        while reader.has_next():
            _, serialized, bag_stamp = reader.read_next()
            yield deserialize_message(serialized, Capture), bag_stamp

    return write_captures(captures(), output)


def ros1_bag_paths(mission: Path) -> list[Path]:
    """Find completed bags directly in a mission directory, or accept one .bag.

    :param mission: Mission directory or single ROS 1 .bag file.
    :return: Sorted input paths; file order does not control capture numbering.
    """
    if mission.is_file() and mission.suffix == '.bag':
        return [mission]
    if not mission.is_dir():
        raise ValueError(f'Expected a mission directory or ROS 1 .bag file: {mission}')
    active = sorted(mission.glob('*.bag.active'))
    if active:
        raise ValueError(f'Unfinished recording(s) in {mission}: '
                         f'{", ".join(path.name for path in active)}. Finish recording first.')
    bags = sorted(path for path in mission.glob('*.bag') if path.is_file())
    if not bags:
        raise ValueError(f'No ROS 1 .bag files found directly in {mission}')
    return bags


def export_ros1_mission(mission: Path, topic: str, output: Path) -> int:
    """Merge split ROS 1 bags into one export using their embedded message types.

    :param mission: Folder containing one mission's .bag files, or a single bag.
    :param topic: Recorded picaso_msgs/Capture topic (ROS 1 spelling).
    :param output: New combined export directory.
    :return: Number of exported captures across all bags.
    """
    bags = ros1_bag_paths(mission)
    from rosbags.highlevel import AnyReader

    try:
        # AnyReader registers embedded custom definitions and merges messages by
        # bag timestamp, including split indices such as _2.bag and _10.bag.
        with AnyReader(bags) as reader:
            connections = [entry for entry in reader.connections if entry.topic == topic]
            if not connections:
                topics = sorted({entry.topic for entry in reader.connections})
                raise ValueError(f'Topic {topic!r} not found in the mission. '
                                 f'Recorded topics: {", ".join(topics)}')
            # rosbags normalizes ROS 1 names to ROS 2-style package/msg/Type.
            wrong_types = sorted({entry.msgtype for entry in connections
                                  if entry.msgtype != 'picaso_msgs/msg/Capture'})
            if wrong_types:
                raise ValueError(f'Topic {topic!r} has unexpected type(s): '
                                 f'{", ".join(wrong_types)}; expected picaso_msgs/Capture')
            print(f'Reading {len(bags)} ROS 1 bag file(s) from {mission}', flush=True)

            def captures():
                for connection, bag_stamp, serialized in reader.messages(connections=connections):
                    yield reader.deserialize(serialized, connection.msgtype), bag_stamp

            return write_captures(captures(), output)
    except (OSError, ValueError):
        raise
    except Exception as error:
        # Library reader/type errors are not all subclasses of RuntimeError.
        # Preserve their cause while giving the CLI one consistent error type.
        partial = f'; partial export retained in {output}' if output.exists() else ''
        raise ValueError(f'Cannot read ROS 1 mission {mission}: {error}{partial}') from error


def main() -> None:
    """Parse paths and topic, export offline, and report actionable errors."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path,
                        help='Mission folder of ROS 1 .bag files, a .bag file, or a ROS 2 bag directory')
    parser.add_argument('--topic', default='/air_sonar/capture',
                        help='Recorded capture topic (default: /air_sonar/capture)')
    parser.add_argument('--output', type=Path, help='New output directory (default: <bag>_export beside bag)')
    args = parser.parse_args()
    bag = args.bag.expanduser().resolve()
    is_ros2 = (bag / 'metadata.yaml').is_file()
    default_name = bag.stem if bag.is_file() else bag.name
    output = args.output.expanduser() if args.output else bag.with_name(f'{default_name}_export')
    try:
        if is_ros2:
            count = export_bag(bag, args.topic, output)
        else:
            count = export_ros1_mission(bag, args.topic, output)
    except ImportError as error:
        hint = ('Source ROS 2 and the sonar workspace first.' if is_ros2 else
                'Install the offline reader: python3 -m pip install -r requirements-bag-export.txt')
        parser.exit(1, f'Export dependencies unavailable: {error}. {hint}\n')
    except (OSError, ValueError, RuntimeError) as error:
        parser.exit(1, f'Export failed: {error}\n')
    print(f'Exported {count} captures to {output.resolve()}')


if __name__ == '__main__':
    main()
