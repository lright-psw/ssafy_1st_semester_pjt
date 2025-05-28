import socket
import time
import threading

HOST = '192.168.110.107'
PORT = 20002

def connect_to_server():
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((HOST, PORT))
            print(f"Connected to {HOST}:{PORT}")
            return s
        except socket.error as e:
            print(f"Connection failed: {e}. Retrying in 5 seconds...")
            s.close()
            time.sleep(5)

try:
    while True:
        # Try to connect to the server
        s = connect_to_server()

        while True:
            try:
                data = s.recv(1024)
                if not data:
                    break

                command = data.decode('utf-8')

                if command == '1':
                    print("command 1 recieved")

                elif command == '2':
                    print("command 2 recieved")

                else:
                    print("Unknown command received.")

                    time.sleep(0.1)

            except socket.error as e:
                    print(f"Socket error: {e}. Reconnecting...")
                    break

        s.close()

except KeyboardInterrupt:
    print("Program terminated")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    print("Resources released and motor stopped.")