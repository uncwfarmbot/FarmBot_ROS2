"""Frame, CRC and decoding tests for the NPK probe driver. No serial port needed."""
import struct

from farmbot_npk.jxct import decode, median_sample, NpkSample
from farmbot_npk.modbus_rtu import (crc16, ModbusClient, ModbusError, parse_read_response,
                                    read_request, with_crc)

import pytest


@pytest.mark.parametrize('register, frame', [
    # The single-value request frames printed in JXCT's NPK manual and reproduced widely
    (0x001E, '01 03 00 1e 00 01 e4 0c'),
    (0x001F, '01 03 00 1f 00 01 b5 cc'),
    (0x0020, '01 03 00 20 00 01 85 c0'),
    (0x0000, '01 03 00 00 00 01 84 0a'),
])
def test_read_request_matches_known_frames(register, frame):
    assert read_request(1, register, 1).hex(' ') == frame


def response(address, registers):
    body = struct.pack('>BBB', address, 3, 2 * len(registers))
    body += struct.pack(f'>{len(registers)}H', *registers)
    return with_crc(body)


def test_parse_round_trip():
    assert parse_read_response(1, 3, response(1, [42, 58, 130])) == [42, 58, 130]


def test_parse_rejects_bad_crc():
    frame = bytearray(response(1, [42]))
    frame[-1] ^= 0xFF
    with pytest.raises(ModbusError, match='CRC'):
        parse_read_response(1, 1, bytes(frame))


def test_parse_rejects_wrong_unit_and_silence():
    with pytest.raises(ModbusError, match='unit 2'):
        parse_read_response(1, 1, response(2, [1]))
    with pytest.raises(ModbusError, match='short'):
        parse_read_response(1, 1, b'')


def test_parse_exception_response():
    with pytest.raises(ModbusError, match='exception code 2'):
        parse_read_response(1, 1, with_crc(bytes([1, 0x83, 2])))


def test_crc_of_frame_with_crc_is_zero():
    assert crc16(read_request(1, 0x12, 2)) == 0


def registers(ph=778, moisture=312, temp=220, ec=256, n=18, p=25, k=51):
    return [temp, moisture, ec, ec, ph, n, p, k]


def test_decode_measured_block():
    # The block read off the BotanyBot probe on 2026-08-27 (Diary: rs485-npk-sensor)
    s = decode([220, 312, 256, 256, 778, 18, 25, 51])
    assert (s.soil_temp_c, s.moisture_pct, s.ec_us_cm, s.ph) == (22.0, 31.2, 256.0, 7.78)
    assert (s.nitrogen_mg_kg, s.phosphorus_mg_kg, s.potassium_mg_kg) == (18, 25, 51)
    assert s.valid and s.status == 'ok'


def test_decode_negative_temperature():
    assert decode(registers(temp=0x10000 - 52)).soil_temp_c == -5.2


def test_decode_flags_impossible_values():
    s = decode(registers(ph=1500))  # pH 15
    assert not s.valid and 'ph' in s.status


def test_median_rejects_one_glitch():
    good = [decode(registers(n=n)) for n in (40, 44)]
    glitch = NpkSample(valid=False, status='TimeoutError')
    m = median_sample(good + [glitch])
    assert m.valid and m.nitrogen_mg_kg == 42 and '2/3' in m.status


def test_median_all_invalid():
    m = median_sample([NpkSample(valid=False, status='no reply')])
    assert not m.valid and 'no reply' in m.status


class FakePort:
    def __init__(self, replies):
        self.replies = list(replies)
        self.written = []

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.written.append(data)

    def flush(self):
        pass

    def read(self, n):
        return self.replies.pop(0)


def test_client_retries_after_garbage():
    port = FakePort([b'\x00\x01', response(1, [308, 215])])
    client = ModbusClient(port, retries=1, gap_s=0)
    assert client.read_registers(1, 0x12, 2) == [308, 215]
    assert len(port.written) == 2
