import serial 
import sys
import threading
import time
import serial.serialutil

# --- Polling Control ---
polling_thread = None
stop_polling_event = threading.Event()
# -----------------------

PORT = "/dev/ttyUSB1" # Ubah jika perlu
BAUDRATE = 115200

def connect_serial(port, baudrate):
    """Mencoba terhubung ke port serial secara terus-menerus hingga berhasil."""
    while True:
        try:
            ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0  # Timeout untuk operasi read
            )
            print(f"\nBerhasil terhubung ke {port}.")
            return ser
        except serial.SerialException as e:
            print(f"\rGagal terhubung ke {port}: {e}. Mencoba lagi dalam 5 detik...", end="")
            time.sleep(5)


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
            # Biarkan loop utama yang menangani proses reconnect
            break
        except Exception as e:
            print(f"\nError tak terduga di thread pembaca: {e}")
            break

def poll_task(ser, interval_ms, command="req.gas"):
    """Thread untuk melakukan polling secara periodik."""
    global stop_polling_event
    interval_seconds = interval_ms / 1000.0
    if command == "req.gas":
        print(f"\nMemulai polling ke 'gas' setiap {interval_seconds:.2f} detik. Ketik 'poll.gas.stop' untuk berhenti.")
    elif command == "req.env":
        print(f"\nMemulai polling ke 'env' setiap {interval_seconds:.2f} detik. Ketik 'poll.env.stop' untuk berhenti.")

    while not stop_polling_event.is_set():
        try:
            # Pastikan tidak ada yang sedang menunggu respons sebelum kita mengirim
            response_received.clear()
            ser.write(b'{REQ_GAS}' if command == "req.gas" else b'{REQ_ENV}')  # Kirim perintah sesuai argumen
            # Tidak perlu wait di sini, biarkan thread utama yang menanganinya jika perlu
        except (serial.SerialException, serial.serialutil.PortNotOpenError):
            print("\nKoneksi terputus saat polling. Polling dihentikan.")
            break
        time.sleep(interval_seconds)
    print("\nPolling dihentikan.")

def main():
    """Fungsi utama program dengan logika auto-reconnect."""
    global polling_thread
    ser = None
    read_thread = None

    while True:
        try:
            if ser is None or not ser.is_open:
                ser = connect_serial(PORT, BAUDRATE)
                response_received.clear()
                
                read_thread = threading.Thread(target=read_from_port, args=(ser, response_received))
                read_thread.daemon = True
                read_thread.start()
                print("Perintah: ping.all, req.all, poll.gas.start.<ms>, poll.gas.stop, poll.env.start.<ms>, poll.env.stop, poll.all.start.<ms>, poll.all.stop, exit")

            # Thread utama menunggu input dari pengguna
            command = input("> ").strip().lower()

            # Kosongkan event sebelum mengirim perintah baru
            response_received.clear()

            if command == "exit":
                break

            elif command == "ping.all":
                ser.write(b'{PING_GAS}')
                time.sleep(0.05)  # Jeda 50ms antar perintah
                ser.write(b'{PING_ENV}')
            elif command == "req.all":
                ser.write(b'{REQ_GAS}')
                time.sleep(0.05) # Jeda 50ms antar perintah
                ser.write(b'{REQ_ENV}')
            elif command == "ping.gas":
                ser.write(b'{PING_GAS}')
            elif command == "ping.env":
                ser.write(b'{PING_ENV}')
            elif command == "req.gas":
                ser.write(b'{REQ_GAS}')
            elif command == "req.env":
                ser.write(b'{REQ_ENV}')
            # --- Polling Per Slave ---

            elif command.startswith("poll.gas.start."):
                try:
                    parts = command.split('.')
                    interval = int(parts[3])
                    # Hentikan polling lama jika ada yang berjalan
                    if polling_thread and polling_thread.is_alive():
                        stop_polling_event.set()
                        polling_thread.join()
                    
                    stop_polling_event.clear()
                    polling_thread = threading.Thread(target=poll_task, args=(ser, interval))
                    polling_thread.daemon = True
                    polling_thread.start()
                except (ValueError, IndexError) as e:
                    print("Format salah. Gunakan: poll.gas.start.<interval_ms>")
                    print(f"Detail error: {e}")

            elif command == "poll.gas.stop":
                stop_polling_event.set()

            elif command.startswith("poll.env.start."):
                try:
                    parts = command.split('.')
                    interval = int(parts[3])
                    if polling_thread and polling_thread.is_alive():
                        stop_polling_event.set()
                        polling_thread.join()
                    stop_polling_event.clear()
                    polling_thread = threading.Thread(target=poll_task, args=(ser, interval), kwargs={"command": "req.env"})
                    polling_thread.daemon = True
                    polling_thread.start()
                except (ValueError, IndexError) as e:
                    print("Format salah. Gunakan: poll.env.start.<interval_ms>")
                    print(f"Detail error: {e}")
            
            elif command == "poll.env.stop":
                stop_polling_event.set()

            # --- Polling All ---

            elif command.startswith("poll.all.start."):  
                parts = command.split('.')
                if len(parts) == 4 and parts[3].isdigit():
                    poll_all(ser,int(parts[3]))
            elif command == "poll.all.stop":
                print("\nMenghentikan polling 'all'...")
                stop_polling_event.set()
            else:
                print("Perintah tidak dikenal. Coba lagi.")
                continue
            
            # Tunggu respons dari slave selama maksimal 2 detik
            # Event akan di-set oleh thread pembaca jika ada data masuk
            # received = response_received.wait(timeout=2.0) #wait dihilangkan
            # if not received:
            #     # Cek apakah thread pembaca masih hidup. Jika tidak, koneksi mungkin terputus.
            #     if not read_thread.is_alive():
            #         raise serial.SerialException("Thread pembaca tidak aktif, koneksi mungkin terputus.")
            #     print("Timeout: Tidak ada respons dari slave.") #Timeout dihilangkan

        

        except (serial.SerialException, serial.serialutil.PortNotOpenError) as e:
            print(f"\nKoneksi serial terputus: {e}. Mencoba menghubungkan kembali...")
            if ser and ser.is_open:
                ser.close()
            ser = None
            if read_thread and read_thread.is_alive():
                read_thread.join() # Tunggu thread pembaca selesai sebelum loop berikutnya
            time.sleep(2) # Beri jeda sebelum mencoba koneksi lagi
        except KeyboardInterrupt:
            print("\nProgram dihentikan oleh pengguna.")
            if polling_thread and polling_thread.is_alive():
                stop_polling_event.set() # Pastikan polling thread berhenti
            break
    
    if ser and ser.is_open:
        ser.close()

    if polling_thread and polling_thread.is_alive():  # Pastikan thread polling berhenti
        polling_thread.join()

    print("Koneksi ditutup. Selamat tinggal!")


def poll_all(ser,interval_ms):
    """Memulai thread background untuk melakukan polling ke semua slave secara bergantian."""
    global polling_thread, stop_polling_event

    # Hentikan polling yang mungkin sedang berjalan
    if polling_thread and polling_thread.is_alive():
        stop_polling_event.set()
        polling_thread.join()
    
    stop_polling_event.clear()

    def poll_all_target():
        """Fungsi yang akan dijalankan oleh thread polling 'all'."""
        interval_seconds = interval_ms / 1000.0
        # Interval total adalah interval_ms, jadi jeda antar request adalah setengahnya
        sleep_duration = interval_seconds / 2.0 
        print(f"\nMemulai polling 'all'. Interval per siklus: {interval_seconds:.2f} detik. Ketik 'poll.all.stop' untuk berhenti.")

        while not stop_polling_event.is_set():
            try:
                # --- Polling GAS ---
                if stop_polling_event.is_set(): break
                response_received.clear()
                ser.write(b'{REQ_GAS}')
                if not response_received.wait(timeout=1.0):
                    print(f"\nTimeout: Tidak ada respons dari slave GAS.", end="")
                
                # Beri jeda setelah request pertama
                time.sleep(sleep_duration)

                # --- Polling ENV (DHT) ---
                if stop_polling_event.is_set(): break
                response_received.clear()
                ser.write(b'{REQ_ENV}')
                if not response_received.wait(timeout=1.0):
                    print(f"\nTimeout: Tidak ada respons dari slave ENV (DHT).", end="")

                # Beri jeda sebelum siklus berikutnya
                time.sleep(sleep_duration)
                
            except (serial.SerialException, serial.serialutil.PortNotOpenError):
                print("\nKoneksi terputus saat polling. Polling dihentikan.")
                break
        print("\nPolling 'all' dihentikan.\n> ", end="")

    polling_thread = threading.Thread(target=poll_all_target)
    polling_thread.daemon = True
    polling_thread.start()

if __name__ == "__main__":
    main()
            response_received.clear()
            ser.write(b'{REQ_GAS}')
            gas_received = response_received.wait(timeout=1.0)  # Tunggu respons GAS
            if not gas_received:
                print("\nTimeout: Tidak ada respons dari slave GAS.")

            if stop_polling_event.is_set(): break  # Cek flag stop setelah setiap operasi

            time.sleep(interval_seconds/2)  # Jeda sebelum polling berikutnya

            # Polling ENV (DHT)
            response_received.clear()
            ser.write(b'{REQ_ENV}')
            env_received = response_received.wait(timeout=1.0)  # Tunggu respons ENV
            if not env_received:
                print("\nTimeout: Tidak ada respons dari slave ENV (DHT).")
            time.sleep(interval_seconds/2)  # Jeda sebelum polling berikutnya
            
            if stop_polling_event.is_set(): break
        except (serial.SerialException, serial.serialutil.PortNotOpenError):
            print("\nKoneksi terputus saat polling. Polling dihentikan.")
            break
    print("\nPolling dihentikan.")
if __name__ == "__main__":
    main()
