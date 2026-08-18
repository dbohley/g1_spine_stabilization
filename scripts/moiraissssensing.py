import socket
import struct
import time

SENSOR_IP = '192.168.1.1'   # update to your sensor's actual IP
PORT = 49152

# From Step 1 - replace with your sensor's actual values
CPF = 1000000   # counts per force (N)
CPT = 1000000   # counts per torque (Nm)

HEADER = 0x1234
CMD_START_STREAMING = 0x0002
SAMPLE_COUNT = 0  # 0 = stream indefinitely


def start_streaming(sock):
    packet = struct.pack('!HHI', HEADER, CMD_START_STREAMING, SAMPLE_COUNT)
    sock.sendto(packet, (SENSOR_IP, PORT))


def stop_streaming(sock):
    packet = struct.pack('!HHI', HEADER, 0x0000, 0)  # command 0 = stop
    sock.sendto(packet, (SENSOR_IP, PORT))


def parse_response(data):
    rdt_seq, ft_seq, status, fx, fy, fz, tx, ty, tz = struct.unpack('!IIIiiiiii', data)
    return {
        'rdt_seq': rdt_seq,
        'ft_seq': ft_seq,
        'status': status,
        'Fx': fx / CPF, 'Fy': fy / CPF, 'Fz': fz / CPF,
        'Tx': tx / CPT, 'Ty': ty / CPT, 'Tz': tz / CPT,
    }


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)

    start_streaming(sock)
    print("Streaming... press Ctrl+C to stop")

    try:
        while True:
            data, _ = sock.recvfrom(36)
            reading = parse_response(data)
            if reading['status'] != 0:
                print(f"WARNING: status code {reading['status']:#010x}")
            print(reading)

    except KeyboardInterrupt:
        print("\nStopping stream...")

    finally:
        stop_streaming(sock)
        sock.close()


if __name__ == '__main__':
    main()