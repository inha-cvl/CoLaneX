import os
import shutil
from datetime import datetime, timedelta

# 현재 날짜와 어제 날짜 계산
today = datetime.now()
yesterday = today - timedelta(days=1)

# 대상 폴더와 목적 폴더 설정
source_folder = "/home/kana/Documents/Dataset/OBIGO/SIM/0907/tlv"
destination_folder = "/home/kana/Documents/Dataset/OBIGO/SIM/0922/tlv"

# 대상 폴더 내의 파일 목록 가져오기
file_list = os.listdir(source_folder)

# 어제에 수정된 파일을 찾아 목적 폴더로 이동
for file_name in file_list:
    file_path = os.path.join(source_folder, file_name)
    
    # 파일의 최종 수정 날짜 가져오기
    modified_time = datetime.fromtimestamp(os.path.getmtime(file_path))
    
    # 어제에 수정된 파일인지 확인
    if modified_time.date() == today.date():
        # 파일을 목적 폴더로 복사
        # destination_path = os.path.join(destination_folder, file_name)
        # shutil.copy2(file_path, destination_path)
        
        # 복사 후에 원본 파일 삭제 (선택사항)
        os.remove(file_path)

print("작업 완료")
