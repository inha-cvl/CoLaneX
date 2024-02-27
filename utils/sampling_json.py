import os
import glob
import json
top = '/home/kana/Documents/Dataset/OBIGO/SIM'


def modify_json_path(file_path, x):
    with open(file_path, 'r', encoding='utf-8', errors='ignore') as file:
        data = json.load(file)
    if x == 'rx':
        data["Ext_V2X_Rx_PDU"]["v2x_msg"]["data"]["path"] = []
    else:
        data["Ext_V2X_Tx_PDU"]["v2x_msg"]["data"]["path"] = []
    
    with open(file_path, 'w', encoding='utf-8') as file:
        json.dump(data, file, indent=2, ensure_ascii=False)
    

#for d in ['0725','0728', '0731', '0804','0809','0811','0814','0822','0823','0828']:
#d_folder = f'{top}/{d}/temp'

for v in ['hlv', 'tlv']:
    s_folder = f'{top}/0922/{v}'
    for x in ['rx', 'tx']:
        files = glob.glob(os.path.join(s_folder, f'{x}*.json'))
        for i, file in enumerate(files):
            if i % 5 != 0:
                os.remove(file)
                
        #for file_path in files:
                #modify_json_path(file_path, x)



'''
shutil.move(file, os.path.join(d_folder, os.path.basename(file)))
'''