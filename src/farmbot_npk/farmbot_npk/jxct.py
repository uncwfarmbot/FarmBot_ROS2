"""
Driver for the JXCT 7-in-1 soil probe on BotanyBot, RS-485 Modbus RTU.

The register map was measured on this probe (2026-08-27, exhaustive register scan, Diary note
rs485-npk-sensor), not taken from a manual: all seven values answer as one 8-register block at
0x0030, which also makes a read a single transaction:

    0x0030 temperature  0.1 C, signed      0x0034 pH          0.01 (not 0.1)
    0x0031 moisture     0.1 %              0x0035 nitrogen    mg/kg
    0x0032 EC           uS/cm              0x0036 phosphorus  mg/kg
    0x0033 mirrors 0x0032, unused          0x0037 potassium   mg/kg

Measured block [220, 312, 256, 256, 778, 18, 25, 51] = 22.0 C, 31.2 %, 256 uS/cm, pH 7.78,
N 18, P 25, K 51. The forum 0x0000 layout and the JXBS-3001 manual's scattered layout belong to
other members of the family and decode this probe into plausible-looking nonsense.
"""
from dataclasses import dataclass, field
import random
import statistics

from farmbot_npk.modbus_rtu import ModbusClient, ModbusError, to_signed

# (first register, count)
BLOCK = (0x0030, 8)

DEFAULT_ADDRESS = 1
DEFAULT_BAUDRATE = 9600

# Datasheet measurement ranges (manual section 1.3)
RANGES = {
    'moisture_pct': (0.0, 100.0),
    'soil_temp_c': (-40.0, 80.0),
    'ec_us_cm': (0.0, 10000.0),
    'ph': (3.0, 9.0),
    'nitrogen_mg_kg': (0.0, 1999.0),
    'phosphorus_mg_kg': (0.0, 1999.0),
    'potassium_mg_kg': (0.0, 1999.0),
}
FIELDS = tuple(RANGES)


@dataclass
class NpkSample:
    """One read of every value, in physical units."""

    moisture_pct: float = 0.0
    soil_temp_c: float = 0.0
    ec_us_cm: float = 0.0
    ph: float = 0.0
    nitrogen_mg_kg: float = 0.0
    phosphorus_mg_kg: float = 0.0
    potassium_mg_kg: float = 0.0
    valid: bool = False
    status: str = 'not read'
    raw_registers: list = field(default_factory=list)

    def out_of_range(self) -> list:
        """
        Fields outside the datasheet range.

        A clean CRC does not make a reading possible: a probe in air, swapped A/B lines or the
        wrong register map all produce well-formed frames.
        """
        return [name for name, (lo, hi) in RANGES.items()
                if not lo <= getattr(self, name) <= hi]


def decode(registers: list) -> NpkSample:
    """Decode the 8-register block at 0x0030 into a sample."""
    temp, moisture, ec, _ec_mirror, ph, n, p, k = registers
    sample = NpkSample(
        moisture_pct=moisture / 10.0,
        soil_temp_c=to_signed(temp) / 10.0,
        ec_us_cm=float(ec),
        ph=ph / 100.0,
        nitrogen_mg_kg=float(n),
        phosphorus_mg_kg=float(p),
        potassium_mg_kg=float(k),
        raw_registers=list(registers),
    )
    bad = sample.out_of_range()
    sample.valid = not bad
    sample.status = 'ok' if not bad else 'out of range: ' + ', '.join(bad)
    return sample


def median_sample(samples: list) -> NpkSample:
    """
    Combine several reads into one, field by field, using only the valid ones.

    A single probe read can glitch; the median of three rejects one bad value without the
    smearing an average would give.
    """
    valid = [s for s in samples if s.valid]
    if not valid:
        status = samples[-1].status if samples else 'no samples'
        return NpkSample(valid=False, status=f'no valid reads ({status})')
    combined = NpkSample(valid=True, status='ok', raw_registers=valid[-1].raw_registers)
    for name in FIELDS:
        setattr(combined, name, float(statistics.median(getattr(s, name) for s in valid)))
    if all(s.status == 'mock' for s in valid):
        combined.status = 'mock'
    elif len(valid) < len(samples):
        combined.status = f'ok ({len(valid)}/{len(samples)} reads valid)'
    return combined


class JxctProbe:
    """Reads the probe over a USB-RS485 adapter. The port is opened only for each read."""

    def __init__(self, port: str, baudrate: int = DEFAULT_BAUDRATE,
                 address: int = DEFAULT_ADDRESS, timeout: float = 1.0, retries: int = 2):
        """Configure the link. Nothing is opened until read()."""
        self.port = port
        self.baudrate = baudrate
        self.address = address
        self.timeout = timeout
        self.retries = retries

    def read(self) -> NpkSample:
        """Read the block once. Never raises for a probe fault."""
        import serial  # only needed for the real probe, not for mock or tests

        try:
            with serial.Serial(self.port, self.baudrate, bytesize=8, parity='N', stopbits=1,
                               timeout=self.timeout) as port:
                client = ModbusClient(port, retries=self.retries)
                registers = client.read_registers(self.address, *BLOCK)
        except (ModbusError, OSError) as e:
            return NpkSample(valid=False, status=f'{type(e).__name__}: {e}')
        return decode(registers)


class MockProbe:
    """Synthetic readings for bench work and simulation. Every sample says status 'mock'."""

    def __init__(self, seed: int = 0):
        """Start from a plausible, slowly drying soil."""
        self.rng = random.Random(seed)
        self.moisture = 32.0

    def read(self) -> NpkSample:
        """Return a noisy, in-range sample."""
        self.moisture = max(12.0, self.moisture - self.rng.uniform(0.0, 0.3))
        g = self.rng.gauss
        return NpkSample(
            moisture_pct=round(self.moisture + g(0, 0.3), 1),
            soil_temp_c=round(21.0 + g(0, 0.4), 1),
            ec_us_cm=float(round(650 + g(0, 15))),
            ph=round(6.6 + g(0, 0.03), 2),
            nitrogen_mg_kg=float(round(42 + g(0, 2))),
            phosphorus_mg_kg=float(round(58 + g(0, 2))),
            potassium_mg_kg=float(round(130 + g(0, 4))),
            valid=True,
            status='mock',
        )
