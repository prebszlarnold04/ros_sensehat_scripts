#!/usr/bin/env python3
import psutil
import math
from sense_hat import SenseHat
import time

# Inicializálás
sense = SenseHat()
sense.clear()

# Színek definiálása
CPU_COLOR = (255, 50, 50)  # Világosabb piros
RAM_COLOR = (50, 255, 50)  # Zöld (különbözik a CPU-tól)
BACKGROUND = (20, 20, 20)  # Sötét szürke háttér
SEPARATOR = (40, 40, 40)   # Elválasztó vonal színe

def draw_bar(column_start, column_end, usage, color):
    # Háttér és elválasztó vonal
    for x in range(column_start, column_end):
        for y in range(8):
            sense.set_pixel(x, y, BACKGROUND)
    
    # Százalékból sorok számává (minimum 1 LED ha >0%)
    height = math.ceil((usage * 8) / 100)
    height = max(1, height) if usage > 0 else 0
    height = min(height, 8)
    
    # Sáv rajzolása alulról felfelé
    for x in range(column_start, column_end):
        for y in range(8 - height, 8):
            sense.set_pixel(x, y, color)
    
    # Dinamikus él színe (intenzitás növelése)
    if height > 0:
        edge_color = tuple(min(c + 80, 255) for c in color)
        for x in range(column_start, column_end):
            sense.set_pixel(x, 8 - height, edge_color)

try:
    while True:
        # Adatok frissítése
        cpu = psutil.cpu_percent(interval=0.3)
        ram = psutil.virtual_memory().percent
        
        # Kijelző frissítése
        sense.clear()
        draw_bar(0, 4, cpu, CPU_COLOR)  # CPU sáv (bal oldal)
        draw_bar(4, 8, ram, RAM_COLOR)  # RAM sáv (jobb oldal)
        
        # Elválasztó vonal középen
        for y in range(8):
            sense.set_pixel(3, y, SEPARATOR)
            sense.set_pixel(4, y, SEPARATOR)
        
        time.sleep(0.2)

except KeyboardInterrupt:
    sense.clear()
