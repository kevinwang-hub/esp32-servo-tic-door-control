import serial, time

p = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=0.3)
# Toggle DTR/RTS to hardware-reset the ESP32, then read boot output
p.setDTR(False); p.setRTS(True); time.sleep(0.1)
p.setRTS(False); time.sleep(0.1)
p.reset_input_buffer()
deadline = time.time() + 5
while time.time() < deadline:
    line = p.readline().decode(errors='replace').rstrip()
    if line:
        print(line)
p.close()
