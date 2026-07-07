#!/bin/bash
# 双击本文件（或终端运行）即可实时查看 INA219 读数，Ctrl+C 退出
PORT=/dev/cu.usbserial-0001
echo "连接 $PORT @115200 ... Ctrl+C 退出"
python3 - <<'PY'
import serial, time, sys
p = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=1)
p.setDTR(False); p.setRTS(True); time.sleep(0.1)
p.setRTS(False); time.sleep(0.3)
p.reset_input_buffer()
try:
    while True:
        line = p.readline().decode('utf-8', 'replace').rstrip()
        if line:
            print(line)
except KeyboardInterrupt:
    p.close()
    sys.exit(0)
PY
