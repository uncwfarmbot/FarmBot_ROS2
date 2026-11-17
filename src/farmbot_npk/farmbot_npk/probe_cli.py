#!/usr/bin/env python3
"""
Bench tool: talk to the NPK probe with no ROS involved.

First bring-up over SSH, before any launch file is involved. The probe needs its 24 V, so
switch Peripheral 4 on first (P4_1 from the keyboard controller) or feed it from a bench supply.

    ros2 run farmbot_npk probe_cli --port /dev/farmbot_npk
    ros2 run farmbot_npk probe_cli --port /dev/ttyUSB0 --scan

Nothing here writes to the probe's configuration registers.
"""
import argparse
import sys

from farmbot_npk.jxct import BLOCK, DEFAULT_ADDRESS, DEFAULT_BAUDRATE, JxctProbe
from farmbot_npk.modbus_rtu import ModbusClient, ModbusError


def scan(port: str, timeout: float, max_address: int) -> int:
    """Sweep the baud rates and unit ids the probe family supports until something answers."""
    import serial

    found = 0
    for baudrate in (9600, 4800, 2400):
        print(f'{baudrate} baud, units 1-{max_address}...')
        with serial.Serial(port, baudrate, timeout=timeout) as link:
            client = ModbusClient(link, retries=0)
            for address in range(1, max_address + 1):
                try:
                    client.read_registers(address, BLOCK[0], 1)
                except ModbusError:
                    continue
                print(f'  FOUND unit {address} at {baudrate} baud')
                found += 1
    if not found:
        print('Nothing answered. Swap A and B first (harmless, and the usual fault), then check '
              'the probe has 24 V and shares a ground with the adapter.')
    return found


def main() -> int:
    """Parse arguments and read or scan."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', default='/dev/farmbot_npk')
    parser.add_argument('--baudrate', type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument('--address', type=int, default=DEFAULT_ADDRESS)
    parser.add_argument('--timeout', type=float, default=0.5)
    parser.add_argument('--scan', action='store_true', help='hunt for baud rate and unit id')
    parser.add_argument('--max-address', type=int, default=10)
    args = parser.parse_args()

    if args.scan:
        return 0 if scan(args.port, args.timeout, args.max_address) else 1

    sample = JxctProbe(args.port, args.baudrate, args.address, args.timeout).read()
    print(f'status      {sample.status}')
    if sample.raw_registers:
        print(f'moisture    {sample.moisture_pct:8.1f} %')
        print(f'temperature {sample.soil_temp_c:8.1f} C')
        print(f'EC          {sample.ec_us_cm:8.0f} uS/cm')
        print(f'pH          {sample.ph:8.2f}')
        print(f'nitrogen    {sample.nitrogen_mg_kg:8.0f} mg/kg')
        print(f'phosphorus  {sample.phosphorus_mg_kg:8.0f} mg/kg')
        print(f'potassium   {sample.potassium_mg_kg:8.0f} mg/kg')
        print(f'raw         {sample.raw_registers}')
    return 0 if sample.valid else 1


if __name__ == '__main__':
    sys.exit(main())
