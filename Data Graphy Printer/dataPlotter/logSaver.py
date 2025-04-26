import serial
import serial.tools.list_ports
ser = serial.Serial('COM3', 115200)
with open("log.txt", "w", encoding="utf-8") as log:
    try:
        command = "LOGSDCARD=1\n"
        ser.write(command.encode('utf-8'))
        while True:
            data = ser.readline()
            if not data:
                continue
            log.write(data.decode('utf-8', errors='replace'))
    except KeyboardInterrupt:
        print("Logging stopped by user.")
    finally:
        ser.close()