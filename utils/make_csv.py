import json
import os
from tqdm import tqdm
import csv
from datetime import datetime, timedelta

def calculate_elapsed_time(start_timestamp, end_timestamp):
    start_time = datetime.strptime(start_timestamp, "%Y.%m.%d.%H:%M:%S.%f")
    end_time = datetime.strptime(end_timestamp, "%Y.%m.%d.%H:%M:%S.%f")
    return end_time - start_time  # Return timedelta object

def get_timestamp(path):
    with open(path, 'r', encoding='utf-8') as file:
        data = json.load(file)
    return data['timestamp']

top_dir = '/home/kana/Documents/Dataset/OBIGO/SIM'
data = [["date", "car", "start", "last", "elapsed"]]
total_elapsed = timedelta()  # Initialize total elapsed time

dirs = [f for f in os.listdir(top_dir) if os.path.isdir(os.path.join(top_dir, f))]
dirs.sort()

for date in tqdm(dirs):
    date_path = f'{top_dir}/{date}'
    for v in ['hlv', 'tlv']:
        v_path = f'{date_path}/{v}'
        json_files = [f for f in os.listdir(v_path) if f.endswith('.json')]
        json_files.sort()  # Sort file names
        first_file_path = os.path.join(v_path, json_files[0])
        last_file_path = os.path.join(v_path, json_files[-1])

        first_ts = get_timestamp(first_file_path)
        last_ts = get_timestamp(last_file_path)
        elapsed_ts = calculate_elapsed_time(first_ts, last_ts)
        total_elapsed += elapsed_ts  # Update total elapsed time

        # Formatting elapsed time for the row
        formatted_elapsed = str(elapsed_ts)
        data.append([date, v, first_ts, last_ts, formatted_elapsed])

with open('/home/kana/Documents/Dataset/OBIGO/log.csv', mode='w', newline='') as file:
    csv_writer = csv.writer(file)
    csv_writer.writerows(data)

# Print total elapsed time
print("Total elapsed time:", str(total_elapsed))
