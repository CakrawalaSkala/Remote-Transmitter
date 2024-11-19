import socket
import threading

# Define IP and port to listen on (should match the C code settings)
HOST_IP_ADDR = "192.168.137.1"  # Listen on all available network interfaces
PORT = 3333  # Replace with the same PORT used in the C code

# Variable to control the server loop
running = True

def udp_server():
    global running
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST_IP_ADDR, PORT))

    print(f"Listening for UDP packets on {HOST_IP_ADDR}:{PORT}...")

    while running:
        try:
            # Set a timeout so the loop can check for the 'running' flag regularly
            sock.settimeout(1.0)
            data, address = sock.recvfrom(128)  # Buffer size is 128 bytes
            print(f"Received message: {data.decode()} from {address}")
        except socket.timeout:
            continue
        except KeyboardInterrupt:
            break

    # Close the socket when done
    sock.close()
    print("Server closed.")

def listen_for_exit():
    global running
    while running:
        user_input = input("Press 'q' to quit:\n")
        if user_input.lower() == 'q':
            running = False
            print("Shutting down...")

if __name__ == "__main__":
    # Run the server and exit listener in parallel threads
    server_thread = threading.Thread(target=udp_server)
    exit_thread = threading.Thread(target=listen_for_exit)

    server_thread.start()
    exit_thread.start()

    # Wait for both threads to complete
    server_thread.join()
    exit_thread.join()
