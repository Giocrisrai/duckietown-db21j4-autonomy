"""Driver PCA9685 directo (sin ROS): logica de registros + envio por bus I2C.

Adaptado del repo del colega Luissalamanca23/Duckietown_Follow_Line_Pi
(drivers/duckie_motor.py). El bus se INYECTA (cualquier objeto con
write_byte_data/read_byte_data, p.ej. smbus2.SMBus) -> testeable sin hardware.
Parametrizable por hw_profile (bus, addr): sirve para la HUT o un PCA9685 externo.

Las funciones puras (prescale_for, duty_to_off, pwm_registers) se testean solas;
la clase PCA9685 solo agrega el envio.
"""

# Registros y bits del PCA9685.
MODE1, MODE2, PRESCALE = 0x00, 0x01, 0xFE
LED0_ON_L = 0x06
RESTART, AI, SLEEP, ALLCALL = 0x80, 0x20, 0x10, 0x01
OUTDRV = 0x04
OSC = 25_000_000.0


def prescale_for(freq_hz, osc=OSC):
    """Valor del registro PRESCALE para la frecuencia PWM pedida."""
    return int(round(osc / (4096.0 * freq_hz)) - 1)


def duty_to_off(duty):
    """duty [0..1] -> cuenta OFF [0..4095] (clamp en los extremos)."""
    duty = max(0.0, min(1.0, duty))
    return int(round(duty * 4095))


def pwm_registers(channel, on, off):
    """Lista [(reg, val)] para fijar ON/OFF de un canal (12 bits c/u)."""
    base = LED0_ON_L + 4 * channel
    return [
        (base, on & 0xFF),
        (base + 1, (on >> 8) & 0x0F),
        (base + 2, off & 0xFF),
        (base + 3, (off >> 8) & 0x0F),
    ]


class PCA9685:
    def __init__(self, bus, addr=0x60):
        self.bus, self.addr = bus, addr

    def _w(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val & 0xFF)

    def _r(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def init(self, freq_hz=1600):
        """Secuencia de arranque: OUTDRV, ALLCALL, despertar (clear SLEEP), freq."""
        self._w(MODE2, OUTDRV)
        self._w(MODE1, ALLCALL)
        self._w(MODE1, self._r(MODE1) & ~SLEEP)   # wake
        self.set_freq(freq_hz)

    def set_freq(self, freq_hz):
        prescale = prescale_for(freq_hz)
        old = self._r(MODE1)
        self._w(MODE1, (old & 0x7F) | SLEEP)      # sleep para escribir prescale
        self._w(PRESCALE, prescale)
        self._w(MODE1, old)
        self._w(MODE1, old | RESTART | AI)

    def set_pwm(self, channel, on, off):
        for reg, val in pwm_registers(channel, on, off):
            self._w(reg, val)

    def set_duty(self, channel, duty):
        """Fija el duty [0..1] de un canal (ON=0, OFF=duty*4095)."""
        self.set_pwm(channel, 0, duty_to_off(duty))
