#!/usr/bin/env python3
"""
Convert ROS 2 Jazzy bag metadata to Humble-compatible format.

Usage:
    python3 convert_bag_metadata.py /path/to/bag_folder
"""

import sys
import os
import yaml


def convert_metadata(bag_path):
    """Convert Jazzy metadata to Humble format."""
    metadata_path = os.path.join(bag_path, 'metadata.yaml')
    
    if not os.path.exists(metadata_path):
        print(f"Error: metadata.yaml not found in {bag_path}")
        return False
    
    # Read original metadata
    with open(metadata_path, 'r') as f:
        data = yaml.safe_load(f)
    
    info = data.get('rosbag2_bagfile_information', {})
    
    # Convert to Humble format (version 5)
    humble_info = {
        'version': 5,
        'storage_identifier': info.get('storage_identifier', 'sqlite3'),
        'duration': info.get('duration', {'nanoseconds': 0}),
        'starting_time': info.get('starting_time', {'nanoseconds_since_epoch': 0}),
        'message_count': info.get('message_count', 0),
        'topics_with_message_count': [],
        'compression_format': info.get('compression_format', ''),
        'compression_mode': info.get('compression_mode', ''),
        'relative_file_paths': info.get('relative_file_paths', []),
    }
    
    # Convert topics - simplify QoS profiles
    for topic in info.get('topics_with_message_count', []):
        topic_meta = topic.get('topic_metadata', {})
        
        # Create simplified QoS profile string
        qos_profiles = """- history: 1
  depth: 10
  reliability: 1
  durability: 2
  deadline:
    sec: 2147483647
    nsec: 4294967295
  lifespan:
    sec: 2147483647
    nsec: 4294967295
  liveliness: 0
  liveliness_lease_duration:
    sec: 2147483647
    nsec: 4294967295
  avoid_ros_namespace_conventions: false"""
        
        humble_topic = {
            'topic_metadata': {
                'name': topic_meta.get('name', ''),
                'type': topic_meta.get('type', ''),
                'serialization_format': topic_meta.get('serialization_format', 'cdr'),
                'offered_qos_profiles': qos_profiles,
            },
            'message_count': topic.get('message_count', 0),
        }
        humble_info['topics_with_message_count'].append(humble_topic)
    
    # Create output
    output = {'rosbag2_bagfile_information': humble_info}
    
    # Backup original
    backup_path = os.path.join(bag_path, 'metadata_jazzy_backup.yaml')
    os.rename(metadata_path, backup_path)
    print(f"Backed up original to: {backup_path}")
    
    # Write new metadata
    with open(metadata_path, 'w') as f:
        yaml.dump(output, f, default_flow_style=False, sort_keys=False)
    
    print(f"Created Humble-compatible metadata: {metadata_path}")
    return True


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 convert_bag_metadata.py /path/to/bag_folder")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    if convert_metadata(bag_path):
        print("Conversion successful!")
    else:
        print("Conversion failed!")
        sys.exit(1)
