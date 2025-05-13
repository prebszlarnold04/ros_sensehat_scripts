#!/usr/bin/env python3
from sense_hat import SenseHat

def read_voltage_from_file():
    """Olvasd ki a feszültséget a fájlból."""
    try:
        # Az akkumulátor feszültségét tároló fájl elérési útja
        with open("/sys/class/power_supply/BAT0/voltage_now", "r") as f:
            microvolts = int(f.read().strip())  # Mikrováltásban
            volts = microvolts / 1_000_000  # Váltás voltba
            return volts
    except FileNotFoundError:
        return None  # Ha a fájl nem található

def display_voltage_on_sensehat(voltage):
    """A Sense HAT-en kiírjuk a feszültséget."""
    sense = SenseHat()

    if voltage is not None:
        # Ha sikerült olvasni a feszültséget
        message = f"Voltage: {voltage:.2f}V"
        sense.show_message(message, scroll_speed=0.05, text_colour=[0, 255, 0])  # Zöld szín
    else:
        # Ha nem találtuk meg a feszültség adatot
        message = "No voltage data"
        sense.show_message(message, scroll_speed=0.05, text_colour=[255, 0, 0])  # Piros szín

def main():
    """A fő program."""
    voltage = read_voltage_from_file()
    display_voltage_on_sensehat(voltage)

if __name__ == "__main__":
    main()
