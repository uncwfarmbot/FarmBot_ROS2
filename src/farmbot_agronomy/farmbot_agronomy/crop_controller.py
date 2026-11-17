#!/usr/bin/env python3
"""
Crop controller node for the ROS2 Farmbot.

Closes the loop the stock autonomous_controller leaves open: instead of firing a fixed command
at a fixed time, each cycle it surveys every plant with the NPK probe, lets one of the four
controllers decide each plant's water and nitrogen, and hands the resulting sequence to the
sequencer. It talks to the stack only through its existing interfaces:

    input_topic      <- P_10 (NPK survey)
    soil/npk         -> this cycle's readings
    sequencer_state  -> 'DONE survey' / 'DONE apply'
    map_info         <- P_LIST (plants), P_11 (sequence for the actions)
    sequencer        <- the action sequence (not in dry_run)
    estop            -> abandon the cycle

With dry_run (the default) everything is decided, logged and published on agronomy/plan, but
nothing is sent to the sequencer.
"""
import csv
import datetime
import json
import os

from ament_index_python.packages import get_package_share_directory

from farmbot_agronomy.action_mapper import Actuation, map_command, to_action
from farmbot_agronomy.soil_model import SoilModel
from farmbot_agronomy.strategies import CycleContext, PlantObservation, STRATEGIES

from farmbot_interfaces.msg import SoilNpk
from farmbot_interfaces.srv import StringRepReq

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Temperature

from std_msgs.msg import Bool, String

from std_srvs.srv import Trigger

import yaml

READING_FIELDS = ('moisture_pct', 'soil_temp_c', 'ec_us_cm', 'ph', 'nitrogen_mg_kg',
                  'phosphorus_mg_kg', 'potassium_mg_kg')
# Map 'soil' block keys for the same fields
MAP_SOIL_KEYS = ('moisture_pct', 'soil_temp_c', 'ec_us_cm', 'ph', 'n', 'p', 'k')


def build_strategy(name: str, config: dict, state: dict):
    """Instantiate a strategy by name with its section of the config."""
    if name == 'rl':
        from farmbot_agronomy.rl_policy import RlPolicy
        return RlPolicy(config['rl'], state)
    if name not in STRATEGIES:
        raise ValueError(f"Unknown strategy '{name}'. Choose one of "
                         f"{sorted(list(STRATEGIES) + ['rl'])}")
    return STRATEGIES[name](config[name], state)


def days_since_planting(plant: dict, today: datetime.date) -> float:
    """Days since the plant's recorded planting date, 0 when it was never recorded."""
    date = (plant.get('status') or {}).get('plant_date') or {}
    try:
        planted = datetime.date(int(date['year']), int(date['month']), int(date['day']))
    except (KeyError, TypeError, ValueError):
        return 0.0
    return float(max((today - planted).days, 0))


class CropController(Node):
    """Run survey -> decide -> act cycles with one of the four crop controllers."""

    def __init__(self):
        """Load the configuration and strategy, and connect to the stack."""
        super().__init__('crop_controller')

        share = get_package_share_directory('farmbot_agronomy')
        self.declare_parameter('strategy', 'threshold')
        self.declare_parameter('dry_run', True)
        self.declare_parameter('survey', True)
        self.declare_parameter('run_at', '07:00')    # daily local time, '' for manual only
        self.declare_parameter('data_dir', os.path.expanduser('~/farmbot_data/agronomy'))
        self.declare_parameter('config_file', os.path.join(share, 'config', 'agronomy.yaml'))
        self.declare_parameter('survey_timeout_s', 3600.0)
        self.declare_parameter('apply_timeout_s', 3600.0)
        self.declare_parameter('max_reading_age_h', 36.0)

        p = self.get_parameter
        self.strategy_name = p('strategy').value
        self.dry_run = bool(p('dry_run').value)
        self.survey = bool(p('survey').value)
        self.run_at = p('run_at').value
        self.data_dir = p('data_dir').value
        self.survey_timeout_s = float(p('survey_timeout_s').value)
        self.apply_timeout_s = float(p('apply_timeout_s').value)
        self.max_reading_age_h = float(p('max_reading_age_h').value)
        os.makedirs(self.data_dir, exist_ok=True)

        with open(p('config_file').value) as f:
            self.config = yaml.safe_load(f)
        self.soil = SoilModel(**self.config['soil'])
        self.actuation = Actuation(**self.config['actuation'])
        self.state_file = os.path.join(self.data_dir, f'state_{self.strategy_name}.json')
        self.strategy = build_strategy(self.strategy_name, self.config, self.load_state())

        self.phase = 'IDLE'
        self.deadline = None
        self.readings = {}
        self.air_temps = []     # (time, degrees C) over the last day
        # Starting the node after today's run_at must not move the robot straight away
        now = datetime.datetime.now()
        self.last_run_date = (now.date() if self.run_at and now.strftime('%H:%M') >= self.run_at
                              else None)

        self.input_pub = self.create_publisher(String, 'input_topic', 10)
        self.sequence_pub = self.create_publisher(String, 'sequencer', 10)
        self.plan_pub = self.create_publisher(String, 'agronomy/plan', 10)
        self.create_subscription(SoilNpk, 'soil/npk', self.reading_callback, 10)
        self.create_subscription(String, 'sequencer_state', self.sequencer_state_callback, 10)
        self.create_subscription(Bool, 'estop', self.estop_callback, 10)
        self.create_subscription(Temperature, 'weather/air_temperature',
                                 self.air_temp_callback, 10)
        self.map_client = self.create_client(StringRepReq, 'map_info')
        self.create_service(Trigger, 'agronomy/run_once', self.run_once_callback)
        self.create_timer(30.0, self.tick)

        self.get_logger().info(
            f"Crop controller '{self.strategy_name}' ready, "
            f"{'DRY RUN (nothing is sent to the sequencer)' if self.dry_run else 'LIVE'}, "
            f"runs {'daily at ' + self.run_at if self.run_at else 'on agronomy/run_once only'}")

    # Persistence

    def load_state(self) -> dict:
        """Load the strategy's persisted per-plant state."""
        try:
            with open(self.state_file) as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def save_state(self):
        """Persist the strategy's per-plant state."""
        with open(self.state_file, 'w') as f:
            json.dump(self.strategy.state, f, indent=1)

    # Triggers

    def tick(self):
        """Start the daily cycle on time, and abandon a phase that has timed out."""
        now = datetime.datetime.now()
        if self.phase != 'IDLE' and self.deadline and now > self.deadline:
            self.get_logger().warn(f'{self.phase} timed out, abandoning this cycle')
            if self.phase == 'SURVEY':
                # Decide on whatever arrived rather than throwing the readings away
                self.request_plants()
            else:
                self.phase = 'IDLE'
            return
        if (self.run_at and self.phase == 'IDLE' and self.last_run_date != now.date()
                and now.strftime('%H:%M') >= self.run_at):
            self.last_run_date = now.date()
            self.start_cycle()

    def run_once_callback(self, request, response):
        """Start a cycle now."""
        response.success = self.start_cycle()
        response.message = 'cycle started' if response.success else f'busy ({self.phase})'
        return response

    def start_cycle(self) -> bool:
        """Begin with a survey, or go straight to deciding on the readings stored in the map."""
        if self.phase != 'IDLE':
            return False
        self.readings = {}
        if self.survey:
            self.get_logger().info('Cycle started: surveying every plant (P_10)')
            self.phase = 'SURVEY'
            self.deadline = (datetime.datetime.now()
                             + datetime.timedelta(seconds=self.survey_timeout_s))
            self.input_pub.publish(String(data='P_10'))
        else:
            self.get_logger().info('Cycle started on stored readings')
            self.request_plants()
        return True

    # Stack callbacks

    def reading_callback(self, msg: SoilNpk):
        """Collect this cycle's valid readings by plant."""
        if self.phase == 'SURVEY' and msg.valid and msg.plant_index >= 0:
            self.readings[msg.plant_index] = {f: float(getattr(msg, f)) for f in READING_FIELDS}
            self.readings[msg.plant_index]['status'] = msg.status

    def sequencer_state_callback(self, msg: String):
        """Advance when the survey or the actions have finished on the robot."""
        if msg.data == 'DONE survey' and self.phase == 'SURVEY':
            self.get_logger().info(f'Survey done, {len(self.readings)} valid readings')
            self.request_plants()
        elif msg.data == 'DONE apply' and self.phase == 'APPLY':
            self.get_logger().info('Actions applied, cycle complete')
            self.phase = 'IDLE'

    def estop_callback(self, msg: Bool):
        """Abandon the cycle on E-stop, which clears the sequencer so it cannot finish."""
        if msg.data and self.phase != 'IDLE':
            self.get_logger().warn(f'E-stop during {self.phase}, cycle abandoned')
            self.phase = 'IDLE'

    def air_temp_callback(self, msg: Temperature):
        """Keep the last day of air temperatures for Hargreaves' Tmin/Tmax."""
        now = datetime.datetime.now()
        self.air_temps.append((now, msg.temperature))
        cutoff = now - datetime.timedelta(days=1)
        self.air_temps = [(t, v) for t, v in self.air_temps if t > cutoff]

    def map_request(self, cmd: str, callback):
        """Send a map_info request; give up on the cycle if the map server is missing."""
        if not self.map_client.service_is_ready():
            self.get_logger().error('map_info is not available, cycle abandoned')
            self.phase = 'IDLE'
            return
        request = StringRepReq.Request()
        request.data = cmd
        self.map_client.call_async(request).add_done_callback(callback)

    # Decide

    def request_plants(self):
        """Fetch the plant list to decide on."""
        self.phase = 'PLAN'
        self.map_request('P_LIST', self.plants_callback)

    def observation(self, index: int, plant: dict, now: datetime.datetime) -> PlantObservation:
        """Build a plant's observation from this cycle's reading, or the stored one."""
        reading = self.readings.get(index)
        soil = plant.get('soil') or {}
        if reading is None and not self.survey and soil.get('stamp'):
            age_h = (now - datetime.datetime.fromisoformat(soil['stamp'])).total_seconds() / 3600
            if age_h <= self.max_reading_age_h:
                reading = {f: float(soil[k]) for f, k in zip(READING_FIELDS, MAP_SOIL_KEYS)}
                reading['status'] = f'stored, {age_h:.1f} h old'
        return PlantObservation(index=index,
                                days_since_planting=days_since_planting(plant, now.date()),
                                reading=reading)

    def cycle_context(self, now: datetime.datetime) -> CycleContext:
        """Bed-wide inputs: the soil model and the day's air temperature range."""
        weather = self.config['weather']
        if len(self.air_temps) >= weather['min_samples']:
            values = [v for _, v in self.air_temps]
            return CycleContext(now=now, soil=self.soil, t_min=min(values), t_max=max(values),
                                temperature_source='sensor')
        return CycleContext(now=now, soil=self.soil, t_min=weather['default_t_min'],
                            t_max=weather['default_t_max'], temperature_source='default')

    def plants_callback(self, future):
        """Decide every plant's actions, log them, and request the action sequence."""
        try:
            plants = yaml.safe_load(future.result().data)['plants'] or {}
        except Exception as e:
            self.get_logger().error(f'Could not read the plant list: {e!r}')
            self.phase = 'IDLE'
            return

        now = datetime.datetime.now()
        ctx = self.cycle_context(now)
        rows, actions = [], []
        for index in sorted(plants, key=int):
            obs = self.observation(int(index), plants[index], now)
            decision = self.strategy.decide(obs, ctx)
            action = to_action(obs.index, decision.water_ml, decision.n_mg, self.actuation)
            actions.append(action)
            rows.append(self.log_row(now, obs, decision, action))
            self.get_logger().info(
                f'Plant {obs.index}: water {action.water_ml:.0f} mL ({action.water_ms} ms), '
                f'N {action.n_mg:.0f} mg ({action.granules} granules) - {decision.reason}')

        self.save_state()
        self.write_log(rows)
        self.plan_pub.publish(String(data=json.dumps({
            'time': now.isoformat(timespec='seconds'), 'strategy': self.strategy_name,
            'dry_run': self.dry_run, 'temperature_source': ctx.temperature_source,
            'plants': rows})))

        command = map_command(actions)
        if self.dry_run or command == 'P_11':
            self.get_logger().info('Dry run, nothing sent' if self.dry_run
                                   else 'No plant needs anything this cycle')
            self.phase = 'IDLE'
            return
        self.map_request(command, self.sequence_callback)

    def sequence_callback(self, future):
        """Send the action sequence to the sequencer."""
        sequence = future.result().data
        if sequence in ('', 'FAILED', 'UNRECOGNIZED'):
            self.get_logger().error(f'map_info could not build the action sequence: {sequence}')
            self.phase = 'IDLE'
            return
        self.phase = 'APPLY'
        self.deadline = datetime.datetime.now() + datetime.timedelta(seconds=self.apply_timeout_s)
        self.sequence_pub.publish(String(data=sequence))

    # Logging

    def log_row(self, now, obs, decision, action) -> dict:
        """One CSV/plan row per plant."""
        row = {'time': now.isoformat(timespec='seconds'), 'strategy': self.strategy_name,
               'dry_run': self.dry_run, 'plant': obs.index,
               'days_since_planting': obs.days_since_planting,
               'reading': (obs.reading or {}).get('status', 'none')}
        for f in READING_FIELDS:
            row[f] = (obs.reading or {}).get(f, '')
        row.update({'water_ml_decided': round(decision.water_ml, 1),
                    'n_mg_decided': round(decision.n_mg, 1),
                    'water_ms': action.water_ms, 'granules': action.granules,
                    'water_ml': round(action.water_ml, 1), 'n_mg': round(action.n_mg, 1),
                    'reason': decision.reason})
        return row

    def write_log(self, rows: list):
        """Append the cycle to decisions.csv in data_dir."""
        if not rows:
            return
        path = os.path.join(self.data_dir, 'decisions.csv')
        new = not os.path.exists(path)
        with open(path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            if new:
                writer.writeheader()
            writer.writerows(rows)


def main(args=None):
    """Initialize and run the crop controller node."""
    rclpy.init(args=args)
    node = CropController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
