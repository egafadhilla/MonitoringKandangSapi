import serial
import sys
 
try:
    # Open serial port
    ser = serial.Serial(
        port="COM5",
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1.0
    )
except serial.SerialException as e:
    print(f"Error: Tidak dapat membuka port COM5. {e}")
    sys.exit()
 
print(f"Mendengarkan data dari {ser.name}...")
 
message_buffer = bytearray()
try:
    while True:
        # Baca satu byte yang masuk
        byte = ser.read(1)
        if byte:
            message_buffer.extend(byte)
            # Jika byte yang diterima adalah '}', berarti pesan selesai
            if byte == b'}':
                try:
                    text = message_buffer.decode('utf-8')
                    print(f"Diterima: {text}")
                except UnicodeDecodeError:
                    print(f"Menerima data yang tidak bisa di-decode: {message_buffer}")
                finally:
                    # Kosongkan buffer untuk pesan berikutnya
                    message_buffer.clear()
except KeyboardInterrupt:
    print("\nProgram dihentikan.")
finally:
    ser.close()