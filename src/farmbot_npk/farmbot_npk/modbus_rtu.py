"""
Minimal Modbus RTU master: function 03, read holding registers.

Only what the soil probe needs. Frames are built and checked here so they can be unit tested
without a serial port; the transport is anything with pyserial's write/read/flush interface.
"""
import struct
import time


class ModbusError(Exception):
    """The probe did not answer, or answered with a bad or exception frame."""


def crc16(data: bytes) -> int:
    """Modbus CRC16 (polynomial 0xA001, initial 0xFFFF)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def with_crc(frame: bytes) -> bytes:
    """Append the CRC, low byte first as Modbus sends it."""
    return frame + struct.pack('<H', crc16(frame))


def read_request(address: int, register: int, count: int) -> bytes:
    """Build a function 03 request frame."""
    return with_crc(struct.pack('>BBHH', address, 0x03, register, count))


def parse_read_response(address: int, count: int, frame: bytes) -> list:
    """
    Check a function 03 response and return its registers.

    Raises ModbusError for a short frame, wrong address, exception response, bad length or
    bad CRC.
    """
    if len(frame) < 5:
        raise ModbusError(f'short response ({len(frame)} bytes): {frame.hex(" ")}')
    if frame[0] != address:
        raise ModbusError(f'response from unit {frame[0]}, expected {address}')
    if frame[1] == 0x83:
        if crc16(frame[:3]) != struct.unpack('<H', frame[3:5])[0]:
            raise ModbusError('exception response with bad CRC')
        raise ModbusError(f'exception code {frame[2]}')
    if frame[1] != 0x03:
        raise ModbusError(f'unexpected function code 0x{frame[1]:02x}')

    expected = 5 + 2 * count
    if frame[2] != 2 * count or len(frame) < expected:
        raise ModbusError(f'expected {2 * count} data bytes, got header {frame[2]} '
                          f'and {len(frame)} bytes in total')
    frame = frame[:expected]
    if crc16(frame[:-2]) != struct.unpack('<H', frame[-2:])[0]:
        raise ModbusError(f'bad CRC: {frame.hex(" ")}')
    return list(struct.unpack(f'>{count}H', frame[3:-2]))


def to_signed(value: int) -> int:
    """Interpret a 16-bit register as two's complement."""
    return value - 0x10000 if value & 0x8000 else value


class ModbusClient:
    """Function 03 reads over an already-open serial port."""

    def __init__(self, port, retries: int = 2, gap_s: float = 0.05):
        """
        Wrap an open pyserial Serial, or anything with write/read/flush/reset_input_buffer.

        retries is the number of extra attempts after a failed transaction; gap_s is the quiet
        time left between transactions, since the probe ignores frames sent too close together.
        """
        self.port = port
        self.retries = retries
        self.gap_s = gap_s

    def read_registers(self, address: int, register: int, count: int) -> list:
        """Read count holding registers starting at register."""
        last_error = None
        for _ in range(self.retries + 1):
            self.port.reset_input_buffer()
            self.port.write(read_request(address, register, count))
            self.port.flush()
            frame = self.port.read(5 + 2 * count)
            time.sleep(self.gap_s)
            try:
                return parse_read_response(address, count, frame)
            except ModbusError as e:
                last_error = e
        raise last_error
