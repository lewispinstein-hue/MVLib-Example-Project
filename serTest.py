import serial
import struct

# --- CONFIGURATION ---
USER_PORT = '/dev/cu.usbmodem102' # Update this if it changes!
BAUD_RATE = 115200

# ---------------------------------------------------------
# COBS DECODER
# Reverses the byte-stuffing to restore the original packet
# ---------------------------------------------------------
def decode_cobs(encoded_bytes):
    decoded = bytearray()
    i = 0
    while i < len(encoded_bytes):
        code = encoded_bytes[i]
        if code == 0:  # Hit the delimiter, we are done
            break
        i += 1
        
        # Copy the data bytes
        for _ in range(code - 1):
            if i < len(encoded_bytes):
                decoded.append(encoded_bytes[i])
                i += 1
                
        # If the code wasn't 0xFF, it means there was originally a 0x00 here
        if code != 0xFF and i < len(encoded_bytes):
            decoded.append(0)
            
    return bytes(decoded)


# ---------------------------------------------------------
# MAIN LISTEN LOOP
# ---------------------------------------------------------
try:
    ser = serial.Serial(USER_PORT, BAUD_RATE, timeout=1)
    
    # Assert DTR and RTS to wake up the V5 Brain
    ser.dtr = True 
    ser.rts = True
    
    print(f"Connected to {USER_PORT}. Waiting for telemetry...")
    
    while True:
        # 1. Read until we hit the 0x00 delimiter
        raw_packet = ser.read_until(b'\x00')
        if raw_packet:
            # 2. STRIP THE HEADERS SAFELY
            # Find the exact location of 'sout' in the byte array
            idx = raw_packet.find(b'sout')
            
            if idx != -1:
                # Slice off everything BEFORE and INCLUDING 'sout', up to the trailing 0x00
                encoded_payload = raw_packet[idx + 4 : -1]
            else:
                # If no header is found, just strip the trailing 0x00
                encoded_payload = raw_packet[:-1]
            
            try:
                # 3. Decode the COBS framing
                decoded_data = decode_cobs(encoded_payload)
                # 4. Check length and unpack!
                if len(decoded_data) == 16:
                    timestamp, x, y, theta = struct.unpack('<Ifff', decoded_data)
                    print(f"Time: {timestamp}ms | X: {x:6.2f} | Y: {y:6.2f} | Theta: {theta:6.2f}")
                else:
                  print(f"Mismatch! Got {len(decoded_data)} decoded bytes.")
                  print(f"-> Decoded Hex: {decoded_data.hex()}")
                  print(f"-> Raw Packet: {raw_packet.hex()}")
            except Exception as e:
                print(f"Decoding error: {e}")

except serial.SerialException as e:
    print(f"Serial error: {e}")
    print("-> Did you make sure to KILL the PROS terminal before running this?")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        
print("Goodbye!")