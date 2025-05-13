#!/usr/bin/env python3
from sense_hat import SenseHat
import time

def rotate_pattern(pattern, times=1):
    """ 64 elemű listát 90°-kal elforgatja óramutató járásával megegyezően times-szor. """
    mat = [pattern[i*8:(i+1)*8] for i in range(8)]
    for _ in range(times):
        mat = [ [mat[7 - j][i] for j in range(8)] for i in range(8) ]
    return [pixel for row in mat for pixel in row]

def main():
    sense = SenseHat()
    sense.clear()

    # Színek
    G = (0, 255, 0)
    B = (0, 0, 0)

    # Fel nyíl a felső 3 sorban, középre igazítva
    arrow_up = [
        B, B, B, G, G, B, B, B,
        B, B, G, G, G, G, B, B,
        B, G, B, G, G, B, G, B,
    ] + [B]* (64 - 24)

    # Más irányok: elforgatjuk az up mintát
    arrow_right = rotate_pattern(arrow_up, times=1)
    arrow_down  = rotate_pattern(arrow_up, times=2)
    arrow_left  = rotate_pattern(arrow_up, times=3)

    # Középpont pont mintája (●)
    dot = [B]*64
    dot[3 + 3*8] = G
    dot[4 + 3*8] = G
    dot[3 + 4*8] = G
    dot[4 + 4*8] = G

    patterns = {
        'up': arrow_up,
        'right': arrow_right,
        'down': arrow_down,
        'left': arrow_left,
        'middle': dot,
    }

    try:
        while True:
            for event in sense.stick.get_events():
                if event.action == 'pressed':
                    pat = patterns.get(event.direction)
                    if pat:
                        sense.set_pixels(pat)
                        time.sleep(0.3)
                        sense.clear()
            time.sleep(0.01)
    except KeyboardInterrupt:
        sense.clear()

if __name__ == '__main__':
    main()
