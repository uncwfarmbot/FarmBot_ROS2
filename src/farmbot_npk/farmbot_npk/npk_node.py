#!/usr/bin/env python3
"""
NPK probe node for the ROS2 Farmbot.

Reads the JXCT 7-in-1 probe carried on the UTM. The probe's RS-485 pair runs on unassigned UTM
conductors to a USB-RS485 adapter on the Pi; its 24 V is switched through Peripheral 4
(Farmduino pin 10), so it is only powered while a reading is taken. Electrodes left live in wet
soil corrode and drift.

Power is switched by publishing P4_1/P4_0 on input_topic, so the stack's own serial node stays
the only writer on the Farmduino port.

Service  npk/read (ReadNpk): power on, settle, read, power off, return the reading
Topic    soil/npk (SoilNpk): every reading the service takes
"""
import threading
import time

from farmbot_interfaces.msg import SoilNpk
from farmbot_interfaces.srv import ReadNpk

from farmbot_npk.jxct import JxctProbe, median_sample, MockProbe, NpkSample

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import String


class NpkProbeNode(Node):
    """Serve on-demand NPK probe readings, switching the probe's power around each one."""

    def __init__(self):
        """Declare parameters, create the probe, the service and the power watchdog."""
        super().__init__('npk_probe')

        self.declare_parameter('port', '/dev/farmbot_npk')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('slave_address', 1)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('retries', 2)
        self.declare_parameter('warmup_s', 20.0)
        self.declare_parameter('samples', 3)
        self.declare_parameter('sample_interval_s', 1.0)
        self.declare_parameter('manage_power', True)
        self.declare_parameter('max_on_s', 90.0)
        self.declare_parameter('mock', False)
        self.declare_parameter('frame_id', 'npk_probe')

        p = self.get_parameter
        self.warmup_s = float(p('warmup_s').value)
        self.samples = max(1, int(p('samples').value))
        self.sample_interval_s = float(p('sample_interval_s').value)
        self.manage_power = bool(p('manage_power').value)
        self.max_on_s = float(p('max_on_s').value)
        self.frame_id = p('frame_id').value

        if bool(p('mock').value):
            self.get_logger().warn('MOCK MODE: synthetic readings, no probe is read. '
                                   "Every reading carries status 'mock'.")
            self.probe = MockProbe()
        else:
            self.probe = JxctProbe(port=p('port').value,
                                   baudrate=int(p('baudrate').value),
                                   address=int(p('slave_address').value),
                                   timeout=float(p('timeout').value),
                                   retries=int(p('retries').value))
            self.get_logger().info(f"Probe on {p('port').value} @ {p('baudrate').value} 8N1, "
                                   f"unit {p('slave_address').value}")

        self.read_lock = threading.Lock()
        self.powered_since = None

        self.cmd_pub = self.create_publisher(String, 'input_topic', 10)
        self.reading_pub = self.create_publisher(SoilNpk, 'soil/npk', 10)
        # The read blocks for the warm-up, so it gets its own group and the watchdog keeps
        # running beside it
        self.read_srv = self.create_service(ReadNpk, 'npk/read', self.read_callback,
                                            callback_group=MutuallyExclusiveCallbackGroup())
        self.watchdog = self.create_timer(1.0, self.power_watchdog,
                                          callback_group=MutuallyExclusiveCallbackGroup())

        self.get_logger().info('NPK probe node initialized')

    def power(self, on: bool):
        """Switch Peripheral 4 through the Farmbot controller."""
        if not self.manage_power:
            return
        self.cmd_pub.publish(String(data='P4_1' if on else 'P4_0'))
        self.powered_since = time.monotonic() if on else None

    def power_watchdog(self):
        """Cut the probe's power if it has been on too long, e.g. a read that never returned."""
        if self.powered_since is not None:
            if time.monotonic() - self.powered_since > self.max_on_s:
                self.get_logger().warn(f'Probe powered for over {self.max_on_s} s, '
                                       'switching Peripheral 4 off')
                self.power(False)

    def read_callback(self, request, response):
        """Power the probe, wait for it to settle, read it, and power it off."""
        warmup = request.warmup_s if request.warmup_s > 0.0 else self.warmup_s
        with self.read_lock:
            self.power(True)
            try:
                # The 20 s default was measured on this probe: shorter reads moisture low and
                # pH high while the electrodes settle
                if self.manage_power:
                    time.sleep(warmup)
                samples = []
                for i in range(self.samples):
                    if i:
                        time.sleep(self.sample_interval_s)
                    samples.append(self.probe.read())
                sample = median_sample(samples)
            except Exception as e:  # a read must never leave the probe powered
                sample = NpkSample(valid=False, status=f'{type(e).__name__}: {e}')
            finally:
                self.power(False)

        response.reading = self.to_msg(sample, request.plant_index)
        response.success = sample.valid
        self.reading_pub.publish(response.reading)
        log = self.get_logger().info if sample.valid else self.get_logger().warn
        log(f'Plant {request.plant_index}: {self.describe(sample)}')
        return response

    def to_msg(self, sample: NpkSample, plant_index: int) -> SoilNpk:
        """Convert a sample to a SoilNpk message."""
        msg = SoilNpk()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.plant_index = plant_index
        msg.moisture_pct = sample.moisture_pct
        msg.soil_temp_c = sample.soil_temp_c
        msg.ec_us_cm = sample.ec_us_cm
        msg.ph = sample.ph
        msg.nitrogen_mg_kg = sample.nitrogen_mg_kg
        msg.phosphorus_mg_kg = sample.phosphorus_mg_kg
        msg.potassium_mg_kg = sample.potassium_mg_kg
        msg.valid = sample.valid
        msg.status = sample.status
        msg.raw_registers = [int(r) for r in sample.raw_registers]
        return msg

    @staticmethod
    def describe(sample: NpkSample) -> str:
        """One-line summary for the log."""
        if not sample.valid:
            return f'INVALID ({sample.status})'
        return (f'moisture {sample.moisture_pct:.1f} %, {sample.soil_temp_c:.1f} C, '
                f'EC {sample.ec_us_cm:.0f} uS/cm, pH {sample.ph:.2f}, '
                f'N {sample.nitrogen_mg_kg:.0f} P {sample.phosphorus_mg_kg:.0f} '
                f'K {sample.potassium_mg_kg:.0f} mg/kg [{sample.status}]')

    def destroy_node(self):
        """Leave the probe unpowered on the way out."""
        if self.powered_since is not None:
            self.power(False)
        return super().destroy_node()


def main(args=None):
    """Initialize and run the NPK probe node."""
    rclpy.init(args=args)
    node = NpkProbeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
