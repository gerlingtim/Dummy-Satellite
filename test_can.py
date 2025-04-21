import can

SCALE = 1000.0

def decode_vector_int16(payload: bytes) -> tuple[float, float, float]:
    """Decode a 6-byte payload into three float values."""
    x = int.from_bytes(payload[0:2], byteorder='big', signed=True) / SCALE
    y = int.from_bytes(payload[2:4], byteorder='big', signed=True) / SCALE
    z = int.from_bytes(payload[4:6], byteorder='big', signed=True) / SCALE
    return x, y, z

def decode_single_int16(payload: bytes) -> float:
    """Decode a 2-byte payload into a single float value."""
    return int.from_bytes(payload[0:2], byteorder='big', signed=True) / SCALE

def main():
    # Open the SLCAN-Interface at COM5
    bus = can.interface.Bus(
        interface="slcan",
        channel="COM5",
        bitrate=500000  # Fits MCP2515 configuration
    )

    print("Wait for CAN-message at SLCAN (COM5)...")

    try:
        while True:
            message = bus.recv(1.0)  # Wait for a message with a timeout of 1 second
            if message:
                if message.dlc == 6:
                    print(f"Received: ID=0x{message.arbitration_id:X}, DLC={message.dlc}, Daten={decode_vector_int16(message.data)}")
                else:
                    print(f"Received: ID=0x{message.arbitration_id:X}, DLC={message.dlc}, Daten={decode_single_int16(message.data)}")
    except KeyboardInterrupt:
        print("Closed.")

if __name__ == "__main__":
    main()