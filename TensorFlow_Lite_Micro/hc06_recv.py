import serial

PORT = "COM7"          # 네 HC-06 COM포트로 바꿔
BAUD = 115200
OUT_FILE = "lidar_log.csv"

ser = serial.Serial(PORT, BAUD, timeout=1)

print(f"open: {PORT}, {BAUD}")
print(f"save: {OUT_FILE}")

with open(OUT_FILE, "w", encoding="utf-8", newline="") as f:
    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            print(line)
            f.write(line + "\n")
            f.flush()

    except KeyboardInterrupt:
        print("stop")

ser.close()
