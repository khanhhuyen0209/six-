Policy:
Yêu cầu: PHẢI DEFINE SIZE MAP - 4 cái limit ở đầu ( cho limit nhỏ hơn size khoảng 0.8-1m cho chắc ăn )
=> Chọn random goal thuộc map size thỏa mãn điều kiện:
 + test_1.py : Phải là điểm chưa quét qua ( unexplored ): self.map.data[idx] == -1
 + test_2.py : Là điểm free ( self.map.data[idx] == 0 ) và phải cách goal trước đó- điểm hiện tại ít nhất 3m

Bug:
test_1: Hơi lag, nhiều lúc bị trượt vì update map bị chậm [ WARN] [1745838890.815485065, 277.321000000]: Map update loop missed its desired rate of 5.0000Hz... the loop actually took 1.0270 seconds
test_2: Chạy oke nhưng đủ kiểu lỗi:
[ERROR] [1745838078.791977813, 133.267000000]: Extrapolation Error: Lookup would require extrapolation 0.005000000s into the future.  Requested time 133.267000000 but the latest data is at time 133.262000000, when looking up transform from frame [odom] to frame [map]
[ERROR] [1745838078.792069212, 133.267000000]: Global Frame: odom Plan Frame size 80: map
( hai cái trên cần fix nma đang cố )
[ WARN] [1745838172.272870830, 203.269000000]: Control loop missed its desired rate of 20.0000Hz... the loop actually took 0.0800 seconds ( kệ nó )
Đôi khi nếu gặp lỗi Teb không tạo được path hay lỗi WARN như ở test_1 thì move_base sẽ chạy cái này: [ WARN] [1745838332.817993826, 303.072000000]: Rotate recovery behavior started.
nhiều lúc xoay vòng liên tục nma yên tâm nó vẫn thoát được vòng => đi tiếp 

NHỚ SAVE MAP
AI SỬA HỘ T CÁI ERROR VISUAL Ở ĐẦU T LUỜI 
