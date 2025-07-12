import serial
import sys
import threading
import time

 
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
 
# Event untuk sinkronisasi antara thread utama (pengirim) dan thread pembaca
response_received = threading.Event()

def read_from_port(ser, event_handler):
    """Fungsi ini berjalan di thread terpisah untuk terus membaca dari port serial."""
    message_buffer = bytearray()
    while True:
        try:
            # Baca satu byte yang masuk
            byte = ser.read(1)
            if byte:
                message_buffer.extend(byte)
                # Jika byte yang diterima adalah '}', berarti pesan selesai
                if byte == b'}':
                    try:
                        text = message_buffer.decode('utf-8')
                        # \r (carriage return) digunakan untuk menimpa baris input saat ini
                        print(f"\rDiterima: {text}\n> ", end="")
                    except UnicodeDecodeError:
                        print(f"\rMenerima data yang tidak bisa di-decode: {message_buffer}\n> ", end="")
                    finally:
                        # Kosongkan buffer untuk pesan berikutnya
                        message_buffer.clear()
                        # Beri tahu thread utama bahwa respons telah diterima
                        event_handler.set()
        except serial.SerialException:
            print("\nKoneksi serial terputus.")
            break
 
# Mulai thread untuk membaca data
# [FIX] Ganti 'spiner' dengan variabel serial port yang benar, yaitu 'ser'
read_thread = threading.Thread(target=read_from_port, args=(ser, response_received))
read_thread.daemon = True # Thread akan berhenti saat program utama berhenti
read_thread.start()

print(f"Terhubung ke {ser.name}. Ketik 'ping', 'req', atau 'exit' untuk keluar.")

try:
    while True:
        # Thread utama menunggu input dari pengguna
        command = input("> ").strip().lower()

        # Kosongkan event sebelum mengirim perintah baru
        response_received.clear()

        if command == "exit":
            break
        elif command == "ping":
            print("Mengirim PING...")
            ser.write(b'{PING_GAS}')
        elif command == "req":
            print("Meminta data sensor...")
            ser.write(b'{REQ_GAS}')
        else:
            print("Perintah tidak dikenal. Gunakan 'ping', 'req', atau 'exit'.")
            continue # Langsung ke iterasi berikutnya jika perintah tidak valid
        
        # Tunggu respons dari slave selama maksimal 2 detik
        # Event akan di-set oleh thread pembaca jika ada data masuk
        received = response_received.wait(timeout=2.0)
        if not received:
            print("Timeout: Tidak ada respons dari slave.")

except KeyboardInterrupt:
    print("\nProgram dihentikan.")
finally:
    ser.close()