### Build
  gcc -o sample sample.c -lpthread -I ./include/ ./include/j2735/ -L ./include/j2735/ -lasncodec
  gcc -o tlv tlv.c -lpthread -I ./include/j2735/ -I ./include/ -L ./include/j2735/ -lasncodec && ./tlv

### Run
  ./sample -c 1000000 -m 100000 

### Reference Standard
SAE J2945/1_201603/202004 On-Board System Requirements for V2V Safety Communications
SAE J2735_201603 Dedicated Short Range Communications (DSRC) Message Set Dictionary 
ITSK-00100-3:2021v2  C-ITS 규격 - 제3부:C2X 정보연계 C-ITS Specification Part3.C2X Information Linking
IEEE 1609.3 Networking services
