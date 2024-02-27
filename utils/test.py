import os
import json
from datetime import datetime, timedelta

def process_json_files(source_folder, destination_folder):
    # Get a list of JSON files in the source folder
    json_files = [f for f in os.listdir(source_folder) if f.endswith('.json')]

    # Sort JSON files by timestamp
    json_files.sort(key=lambda x: get_timestamp(os.path.join(source_folder, x)))

    # Read and process each JSON file
    for idx, json_file in enumerate(json_files):
        source_path = os.path.join(source_folder, json_file)
        destination_path = os.path.join(destination_folder, json_file)

        with open(source_path, 'r') as file:
            data = json.load(file)

        if idx == 0:
            # For the first file, set its timestamp as the new start time
            start_time = datetime.strptime(data['timestamp'], "%Y.%m.%d.%H:%M:%S.%f")
            new_start_time = datetime(2023, 9, 11, 13, 58, 12, 711000)
            time_difference = new_start_time - start_time
        else:
            # For subsequent files, adjust the timestamp based on the time difference
            timestamp = datetime.strptime(data['timestamp'], "%Y.%m.%d.%H:%M:%S.%f")
            adjusted_timestamp = timestamp + time_difference
            data['timestamp'] = adjusted_timestamp.strftime("%Y.%m.%d.%H:%M:%S.%f")

        # Write modified data to the destination folder
        with open(destination_path, 'w') as file:
            json.dump(data, file, indent=2)

        print(f"Processed file {idx + 1}: {json_file}")

def get_timestamp(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    timestamp_str = data['timestamp']
    return datetime.strptime(timestamp_str, "%Y.%m.%d.%H:%M:%S.%f")

# Example usage:
source_folder = "/home/kana/Documents/Dataset/OBIGO/SIM/0823/hlv"
destination_folder = "/home/kana/Documents/Dataset/OBIGO/SIM/0911/hlv"

process_json_files(source_folder, destination_folder)
