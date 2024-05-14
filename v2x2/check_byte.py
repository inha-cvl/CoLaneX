import socket

def hex_to_host_byte_order(hex_value):
    if len(hex_value) == 8:
        iv = int(hex_value, 16)
        host_val = socket.ntohl(iv)
        return socket.ntohl(host_val)
    elif len(hex_value) == 4:
        iv = int(hex_value, 16)
        host_val = socket.ntohs(iv)
        return socket.ntohs(host_val)
    else:
        iv = int(hex_value, 16)
        host_val = socket.ntohs(iv)
        return socket.ntohs(host_val)

while True:
    hex_input = input("Enter the hexadecimal value (type 'exit' to quit): ")
    if hex_input.lower() == "exit":
        print("Exiting...")
        break
    try:
        host_byte_order_value = hex_to_host_byte_order(hex_input)
        print("Host byte order value:", host_byte_order_value)
    except ValueError:
        print("Invalid hexadecimal value. Please enter a valid hexadecimal value.")
