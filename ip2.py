#!/usr/bin/env python3
import netifaces
from sense_hat import SenseHat

# A funkció, amely lekérdezi az aktív hálózati interfész IP címét
def get_ip_address():
    try:
        # Az 'eth0' vagy 'wlan0' interfészt próbáljuk először, ha van
        ip_info = netifaces.ifaddresses('wlan0')[netifaces.AF_INET]
        ip_address = ip_info[0]['addr']
    except (ValueError, KeyError):
        try:
            ip_info = netifaces.ifaddresses('eth0')[netifaces.AF_INET]
            ip_address = ip_info[0]['addr']
        except (ValueError, KeyError):
            ip_address = "No IP found"
    
    return ip_address

# A program fő része
def main():
    sense = SenseHat()
    
    # IP cím lekérdezése
    ip_address = get_ip_address()

    # Színek a szöveghez
    TEXT_COLOR = (0, 255, 255)  # Zöld
    BACKGROUND_COLOR = (128, 0, 128)  # Fekete háttér

    # Kiírjuk az IP címet a Sense HAT kijelzőjére
    sense.show_message(f"IP: {ip_address}", text_colour=TEXT_COLOR, back_colour=BACKGROUND_COLOR, scroll_speed=0.1)

if __name__ == '__main__':
    main()
