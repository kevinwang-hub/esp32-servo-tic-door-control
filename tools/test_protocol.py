import serial, time

p = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=0.3)
# ESP32 resets on port open; wait for boot
time.sleep(1.5)
p.reset_input_buffer()

def send(cmd):
    p.write((cmd + "\n").encode())
    print(">", cmd)

def drain(secs=1.0):
    end = time.time() + secs
    while time.time() < end:
        line = p.readline().decode(errors="replace").strip()
        if line:
            print("<", line)

send("PING");   drain(0.6)
send("STATUS"); drain(0.6)
send("SERVO 100"); drain(0.4)
send("STATUS"); drain(0.6)
send("STEP 5");  drain(0.4)
send("STATUS"); drain(0.6)
p.close()
