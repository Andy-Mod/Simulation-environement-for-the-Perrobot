import can
import time

def send_message(bus, message_id, data):
    """Function to send a CAN message."""
    # Ensure data is a list of bytes
    data_bytes = data.to_bytes(8, 'big')  # Convert integer to 8 bytes big endian
    msg = can.Message(arbitration_id=message_id, data=data_bytes, is_extended_id=False)
    try:
        bus.send(msg)
        print(f"Message sent on {bus.channel_info} {msg}")
    except can.CanError:
        print("Message NOT sent")

def receive_message(bus):
    """Function to receive a CAN message."""
    message = bus.recv(10)  # Timeout can be added as an argument (in seconds)
    if message is not None:
        print(f"Received message: {message}")
    else:
        print("No message received.")

if __name__ == "__main__":
    try:
        # Configure the CAN interface
        can_interface = 'can0'  # Name of the CAN interface
        bus = can.interface.Bus(can_interface, bustype='socketcan')

        # Send a message
        message_id = 0x005
        
        data2 = 0x0100000000000000
        data3 = 0x0000000000000000
        
        
        send_message(bus, message_id, data2)
        # 
        time.sleep(10)
        send_message(bus, message_id, data3)

        # Try to receive any message
        print("Listening for CAN messages...")
        while True:
            receive_message(bus)
            
    finally:
        # Ensure the bus is properly shut down on exit
        bus.shutdown()
