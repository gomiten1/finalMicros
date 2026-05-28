from lcd_api import LcdApi
from time import sleep_ms

# Mapeos comunes de PCF8574 (bits)
# Cada tupla es: (RS, RW, E, BL, D4, D5, D6, D7)
CANDIDATE_MAPS = [
    (0, 1, 2, 3, 4, 5, 6, 7),  # estándar
    (0, 2, 1, 3, 4, 5, 6, 7),  # E/RW intercambiados
    (2, 1, 0, 3, 4, 5, 6, 7),  # RS/E intercambiados
    (0, 1, 2, 7, 4, 5, 6, 3),  # BL en otro bit
]

class I2cLcd(LcdApi):
    def __init__(self, i2c, addr, rows, cols):
        self.i2c = i2c
        self.addr = addr
        self.rows = rows
        self.cols = cols

        self.map = None
        self._detect_map()
        super().__init__(rows, cols)

    def _write(self, data):
        self.i2c.writeto(self.addr, bytes([data]))

    def _pulse(self, data, en_bit):
        self._write(data | (1 << en_bit))
        sleep_ms(1)
        self._write(data & ~(1 << en_bit))

    def _send4(self, nibble, mode):
        RS, RW, E, BL, D4, D5, D6, D7 = self.map

        data = 0
        data |= ((nibble >> 0) & 1) << D4
        data |= ((nibble >> 1) & 1) << D5
        data |= ((nibble >> 2) & 1) << D6
        data |= ((nibble >> 3) & 1) << D7
        data |= (1 << BL)  # backlight

        if mode:
            data |= (1 << RS)

        self._pulse(data, E)

    def _send(self, value, mode=0):
        self._send4(value >> 4, mode)
        self._send4(value & 0x0F, mode)

    def _init_lcd(self):
        sleep_ms(50)
        self._send4(0x03, 0)
        sleep_ms(5)
        self._send4(0x03, 0)
        sleep_ms(1)
        self._send4(0x03, 0)
        self._send4(0x02, 0)

        self._send(0x28)
        self._send(0x08)
        self._send(0x01)
        sleep_ms(2)
        self._send(0x06)
        self._send(0x0C)

    def _detect_map(self):
        for m in CANDIDATE_MAPS:
            try:
                self.map = m
                self._init_lcd()
                self.clear()
                return
            except:
                pass
        raise Exception("No se pudo detectar el mapeo")

    def clear(self):
        self._send(0x01)
        sleep_ms(2)

    def move_to(self, col, row):
        addr = col + (0x40 * row)
        self._send(0x80 | addr)

    def putchar(self, char):
        self._send(ord(char), 1)