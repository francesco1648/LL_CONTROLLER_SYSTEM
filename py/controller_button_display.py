from pynput import keyboard

def on_press(key):
    try:
        print(f"Tasto premuto: {key.char}")
    except AttributeError:
        print(f"Tasto speciale: {key}")

def on_release(key):
    if key == keyboard.Key.esc:  # esci premendo ESC
        print("Uscita dal programma.")
        return False

with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    print("In ascolto dei tasti... (premi ESC per uscire)")
    listener.join()
