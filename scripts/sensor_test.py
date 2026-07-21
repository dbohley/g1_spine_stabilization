import serial
import time
import re

PORT = '/dev/ttyUSB1'   # try /dev/ttyUSB1 if this one doesn't respond
BAUD = 115200

def parse_reading(line):
    """Extract Fx, Fy, Fz, Tx, Ty, Tz from a line like:
    '34.928 N 10.234 N -0.370 N -0.1196 Nm -0.0787 Nm -0.9156 Nm'"""
    nums = re.findall(r'-?\d+\.\d+', line)
    if len(nums) >= 6:
        fx, fy, fz, tx, ty, tz = map(float, nums[:6])
        return {'Fx': fx, 'Fy': fy, 'Fz': fz, 'Tx': tx, 'Ty': ty, 'Tz': tz}
    return None



def main():


    # --- Open the connection ---
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(0.2)  # let the port settle

    # --- Optional: zero the sensor before reading ---
    ser.reset_input_buffer()
    ser.write(b'bias on\r\n')
    time.sleep(0.1)
    print(ser.read(ser.in_waiting or 200).decode(errors='ignore'))

    # --- Start continuous streaming ---
    ser.reset_input_buffer()
    ser.write(b'c\r\n')

    print("Streaming... press Ctrl+C to stop")

    try:
        while True:
            line = ser.readline().decode(errors='ignore').strip()
            if not line:
                continue
            reading = parse_reading(line)
            if reading:
                print(reading)
                # e.g. do something with reading['Fz'] here in real time

    except KeyboardInterrupt:
        print("\nStopping stream...")
        ser.write(b'e')       # tell the sensor to stop streaming
        time.sleep(0.1)
        ser.reset_input_buffer()

    finally:
        ser.close()

if __name__== "__main__":
    main()