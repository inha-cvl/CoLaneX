import json

# JSON 파일 경로
json_file_path = "../planning/KCity.json"

# JSON 파일 읽기
with open(json_file_path, 'r') as file:
    data = json.load(file)

# 삭제할 키 목록
keys_to_keep = ["304", "303", "302", "305", "301", "306", "319", "318", "309", "308", "307", "313", "312", "311", "315", "299", "298", "118", "117", "101", "102", "100"]

# "lanelets"에서 지정된 키 이외의 모든 키 삭제
data["lanelets"] = {key: data["lanelets"][key] for key in keys_to_keep}

# 새로운 JSON 파일로 저장
new_json_file_path = "./test_bed.json"
with open(new_json_file_path, 'w') as new_file:
    json.dump(data, new_file, indent=2)

print(f"새로운 JSON 파일이 {new_json_file_path}에 저장되었습니다.")
