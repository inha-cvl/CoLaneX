import glob
import re


date = '1012'
car = 'hlv'

# 결과를 저장할 파일
output_file = open(f"./log_result/{date}_{car}_result.txt", "w")

# 특정 패턴의 파일 목록 가져오기
file_list = glob.glob(f"./log/{date}_{car}_*.txt")

# 데이터 저장을 위한 변수 초기화
average_values = {}
count_values = {}
cnt_values = {}

# 파일별로 데이터 처리
for file_name in file_list:
    with open(file_name, "r") as file:
        for line in file:
            match = re.search(r'Average rate for (\d+)m to (\d+)m distance: ([\d.]+) cnt=\s*\[(\d+)\]', line)
            if match:
                start_range, end_range, average, count = map(float, match.groups())
                print(start_range, end_range, average, count)
                range_str = f"{start_range}m to {end_range}m"
                if range_str in average_values:
                    average_values[range_str] += average
                    count_values[range_str] += count
                    cnt_values[range_str] += 1
                else:
                    average_values[range_str] = average
                    count_values[range_str] = count
                    cnt_values[range_str] = 1

# 결과 파일에 각 범위에 대한 평균과 카운트를 추가
for range_str in average_values:
    average = average_values[range_str]
    count = count_values[range_str]
    cnt = cnt_values[range_str]
    avg = average / cnt if cnt != 0 else 0
    output_file.write(f"{range_str}: {avg:.2f} cnt=[{count}]\n")

# 결과 파일 닫기
output_file.close()