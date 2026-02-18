#!/usr/bin/env python3
"""
Clean a ROS 2 bag metadata file to be Humble-compatible.

This script removes Jazzy-specific fields from metadata.yaml and
the SQLite3 database to make bags recorded on Jazzy playable on Humble.
"""

import argparse
import shutil
import sqlite3
import yaml
from pathlib import Path


def clean_metadata(metadata_path: Path) -> dict:
    """Remove Jazzy-specific fields from metadata."""
    with open(metadata_path, 'r') as f:
        data = yaml.safe_load(f)
    
    info = data['rosbag2_bagfile_information']
    
    # Force version 5 (Humble)
    info['version'] = 5
    
    # Remove 'files' field (Jazzy-specific)
    if 'files' in info:
        del info['files']
    
    # Remove 'custom_data' field if null
    if 'custom_data' in info and info['custom_data'] is None:
        del info['custom_data']
    
    # Set ros_distro to humble
    info['ros_distro'] = 'humble'
    
    # Clean topic metadata - remove type_description_hash
    for topic in info.get('topics_with_message_count', []):
        topic_meta = topic.get('topic_metadata', {})
        if 'type_description_hash' in topic_meta:
            del topic_meta['type_description_hash']
        
        # Convert QoS profiles to string format expected by Humble
        # The rosbags library already does this in version 5
    
    return data


def clean_embedded_metadata(metadata_str: str) -> str:
    """Clean embedded metadata in the database."""
    data = yaml.safe_load(metadata_str)
    
    # Remove files field
    if 'files' in data:
        del data['files']
    
    # Remove custom_data if null
    if 'custom_data' in data and data['custom_data'] is None:
        del data['custom_data']
    
    # Set ros_distro to humble
    data['ros_distro'] = 'humble'
    
    # Force version 5
    data['version'] = 5
    
    # Clean topic metadata - remove type_description_hash
    for topic in data.get('topics_with_message_count', []):
        topic_meta = topic.get('topic_metadata', {})
        if 'type_description_hash' in topic_meta:
            del topic_meta['type_description_hash']
    
    return yaml.dump(data, default_flow_style=False, sort_keys=False)


def clean_database(db_path: Path, output_path: Path) -> None:
    """
    Create a new Humble-compatible SQLite3 database.
    
    Humble's rosbag2 expects a simpler schema without type_description_hash.
    """
    # Copy the original database
    shutil.copy2(db_path, output_path)
    
    conn = sqlite3.connect(output_path)
    cursor = conn.cursor()
    
    # Check if type_description_hash column exists
    cursor.execute("PRAGMA table_info(topics)")
    columns = [row[1] for row in cursor.fetchall()]
    
    if 'type_description_hash' in columns:
        print("Removing type_description_hash from database schema...")
        
        # Create a new topics table without type_description_hash
        cursor.execute("""
            CREATE TABLE topics_new (
                id INTEGER PRIMARY KEY,
                name TEXT NOT NULL,
                type TEXT NOT NULL,
                serialization_format TEXT NOT NULL,
                offered_qos_profiles TEXT NOT NULL
            )
        """)
        
        # Copy data (excluding type_description_hash)
        cursor.execute("""
            INSERT INTO topics_new (id, name, type, serialization_format, offered_qos_profiles)
            SELECT id, name, type, serialization_format, offered_qos_profiles FROM topics
        """)
        
        # Replace old table
        cursor.execute("DROP TABLE topics")
        cursor.execute("ALTER TABLE topics_new RENAME TO topics")
    
    # Remove message_definitions table if it exists (Jazzy-specific)
    cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='message_definitions'")
    if cursor.fetchone():
        print("Removing message_definitions table...")
        cursor.execute("DROP TABLE message_definitions")
    
    # Clean embedded metadata in the database
    cursor.execute("SELECT id, metadata FROM metadata")
    rows = cursor.fetchall()
    for row_id, metadata_str in rows:
        print("Cleaning embedded metadata in database...")
        clean_meta = clean_embedded_metadata(metadata_str)
        cursor.execute("UPDATE metadata SET metadata = ?, metadata_version = 5 WHERE id = ?", 
                      (clean_meta, row_id))
    
    # Update schema table if it exists
    cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='schema'")
    if cursor.fetchone():
        cursor.execute("UPDATE schema SET schema_version = 3 WHERE schema_version > 3")
    
    conn.commit()
    conn.close()


def convert_bag_for_humble(input_bag: Path, output_bag: Path) -> None:
    """Convert a Jazzy bag to Humble-compatible format."""
    
    # Create output directory
    output_bag.mkdir(parents=True, exist_ok=True)
    
    # Find input files
    input_metadata = input_bag / 'metadata.yaml'
    if not input_metadata.exists():
        raise FileNotFoundError(f"No metadata.yaml found in {input_bag}")
    
    # Clean metadata
    print(f"Cleaning metadata from {input_metadata}...")
    clean_data = clean_metadata(input_metadata)
    
    # Write cleaned metadata
    output_metadata = output_bag / 'metadata.yaml'
    with open(output_metadata, 'w') as f:
        yaml.dump(clean_data, f, default_flow_style=False, sort_keys=False)
    
    # Process each database file
    for db_file in input_bag.glob('*.db3'):
        print(f"Processing database: {db_file.name}")
        output_db = output_bag / db_file.name
        clean_database(db_file, output_db)
    
    print(f"\nConversion complete! Humble-compatible bag saved to: {output_bag}")
    print("\nTo transfer to Jetson:")
    print(f"  scp -r {output_bag} f1tenth@192.168.1.83:~/f1tenth_bags/")


def main():
    parser = argparse.ArgumentParser(
        description='Convert a ROS 2 bag from Jazzy format to Humble-compatible format'
    )
    parser.add_argument(
        'input_bag',
        type=Path,
        help='Path to the input bag directory (Jazzy format)'
    )
    parser.add_argument(
        '--output', '-o',
        type=Path,
        default=None,
        help='Path for output bag directory (default: <input>_humble_clean)'
    )
    
    args = parser.parse_args()
    
    input_bag = args.input_bag.resolve()
    if not input_bag.is_dir():
        raise ValueError(f"Input must be a bag directory: {input_bag}")
    
    if args.output:
        output_bag = args.output.resolve()
    else:
        output_bag = input_bag.parent / f"{input_bag.name}_humble_clean"
    
    if output_bag.exists():
        print(f"Output directory already exists: {output_bag}")
        response = input("Overwrite? [y/N] ")
        if response.lower() != 'y':
            print("Aborted.")
            return
        shutil.rmtree(output_bag)
    
    convert_bag_for_humble(input_bag, output_bag)


if __name__ == '__main__':
    main()
