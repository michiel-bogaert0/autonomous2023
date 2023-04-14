# import socket
# import serial
# import base64
# import time

# # NTRIP connection settings
# caster_host = '192.168.50.36'
# caster_port = 2101
# mountpoint = 'pygnssutils'
# username = 'anon'
# password = 'password'

# # Serial connection settings
# serial_port = '/dev/ttybase'
# serial_baudrate = 460800

# def create_ntrip_request():
#     ntrip_request = f'GET /{mountpoint} HTTP/1.0\r\n'
#     ntrip_request += f'User-Agent: NTRIP PythonClient/1.0\r\n'
#     ntrip_request += f'Authorization: Basic {base64.b64encode(f"{username}:{password}".encode()).decode()}\r\n'
#     ntrip_request += '\r\n'
#     return ntrip_request

# def connect_ntrip_caster(host, port, request):
#     sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     sock.connect((host, port))
#     sock.sendall(request.encode())
#     return sock

# def main():
#     ntrip_request = create_ntrip_request()
#     ntrip_socket = connect_ntrip_caster(caster_host, caster_port, ntrip_request)

#     # Check NTRIP response
#     response = ntrip_socket.recv(4096).decode(errors='ignore')
#     print(response)
#     if 'ICY 200 OK' not in response:
#         print(f'Error connecting to NTRIP caster: {response}')
#         return

#     # Set up serial connection
#     ser = serial.Serial(serial_port, serial_baudrate, timeout=0.5)

#     try:
#         while True:
#             data = ntrip_socket.recv(4096)
#             if not data:
#                 break
#             ser.write(data)
#             time.sleep(0.01)
#     except KeyboardInterrupt:
#         print('Closing connections...')
#     finally:
#         ntrip_socket.close()
#         ser.close()

# if __name__ == '__main__':
#     main()


import socket
import serial

# Define the server address and port
SERVER_ADDRESS = ('192.168.50.36', 50010)

# Define the serial port and baud rate
SERIAL_PORT = '/dev/ttybase'
BAUD_RATE = 460800

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Create a serial object
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

# Connect the socket to the server address and port
sock.connect(SERVER_ADDRESS)
print('Connecting to {} port {}'.format(*SERVER_ADDRESS))

# Read data from the socket and write it to the serial port
while True:
  data = sock.recv(1024)
  print('Received {!r}'.format(data))
  ser.write(data)

# Close the serial port and socket
ser.close()
sock.close()
