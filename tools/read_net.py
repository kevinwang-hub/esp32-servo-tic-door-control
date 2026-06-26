import serial, time

p = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=0.3)
p.setDTR(False); p.setRTS(True); time.sleep(0.1)
p.setRTS(False); time.sleep(0.1)
p.reset_input_buffer()
deadline = time.time() + 18   # allow time for Wi-Fi join attempt + fallback
while time.time() < deadline:
    line = p.readline().decode(errors='replace').rstrip()
    if line:
        print(line)
p.close()
