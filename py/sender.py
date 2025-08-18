from pynput import keyboard
import serial
import serial.tools.list_ports
import threading

# Mostra le porte COM disponibili
ports = list(serial.tools.list_ports.comports())
print("Porte disponibili:")
for i, p in enumerate(ports):
    print(f"[{i}] {p.device} - {p.description}")

# Chiede quale usare
scelta = input("Seleziona il numero della porta COM o scrivi direttamente il nome (es. COM7): ")

if scelta.isdigit():
    index = int(scelta)
    if index < 0 or index >= len(ports):
        raise ValueError("Indice non valido.")
    port_name = ports[index].device
else:
    port_name = scelta.strip()

# Apre la porta scelta
ser = serial.Serial(port=port_name, baudrate=115200, timeout=1)
print(f"Connesso al Pico sulla porta {ser.port}... (ESC per uscire)")

# Thread che legge dalla seriale
def leggi_seriale():
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    print(f"[Dal Pico] {line}")
        except Exception as e:
            print(f"Errore seriale: {e}")
            break

# Listener tastiera → manda i tasti al Pico
def on_press(key):
    try:
        msg = f"{key.char}\n"
        ser.write(msg.encode())
    except AttributeError:
        msg = f"{key}\n"
        ser.write(msg.encode())

def on_release(key):
    if key == keyboard.Key.esc:  # ESC = esci
        print("Uscita dal programma.")
        ser.close()
        return False

# Avvio thread per leggere dalla seriale
threading.Thread(target=leggi_seriale, daemon=True).start()

# Avvio listener tastiera
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()
