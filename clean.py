#!/usr/bin/env python3

from sense_hat import SenseHat
import time

# A Sense HAT példányosítása
sense = SenseHat()

# Színek listája a szivárványhoz
rainbow_colors = [
    (255, 0, 0),    # Piros
    (255, 127, 0),  # Narancs
    (255, 255, 0),  # Sárga
    (0, 255, 0),    # Zöld
    (0, 0, 255),    # Kék
    (75, 0, 130),   # Indigó
    (238, 130, 238) # Ibolya
]

# Szivárvány kirajzolása
def draw_rainbow():
    pixels = []
    num_colors = len(rainbow_colors)
    for i in range(64):
        pixels.append(rainbow_colors[i % num_colors])  # Szivárvány színei ciklikusan
    sense.set_pixels(pixels)

# Törlés a kijelzőn
def clear_display():
    sense.clear()

def main():
    # Szivárvány kirajzolása
    draw_rainbow()
    time.sleep(3)  # 3 másodpercig látni fogjuk a szivárványt

    # Törlés a kijelzőn
    clear_display()

if __name__ == '__main__':
    main()

