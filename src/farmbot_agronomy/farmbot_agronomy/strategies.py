"""
The four crop controllers compared in simulation (Capstone Deliverable 5, section 4.3).

Each one turns a plant's latest soil reading into a water volume (mL) and a nitrogen mass (mg)
for this cycle. They are plain Python with no ROS so they can be tested on their own. State
that has to survive between cycles, such as FAO-56's running depletion, lives in a dict the
node saves to disk.
"""
from dataclasses import dataclass, field
import datetime

from farmbot_agronomy import fao56
from farmbot_agronomy.soil_model import SoilModel


@dataclass
class PlantObservation:
    """What a controller knows about one plant this cycle."""

    index: int
    days_since_planting: float
    # SoilNpk fields of this cycle's reading, or None when there is no valid one
    reading: dict = None


@dataclass
class CycleContext:
    """What a controller knows about the bed this cycle."""

    now: datetime.datetime
    soil: SoilModel
    t_min: float = 18.0
    t_max: float = 28.0
    temperature_source: str = 'default'


@dataclass
class Decision:
    """A controller's output for one plant."""

    water_ml: float = 0.0
    n_mg: float = 0.0
    reason: str = ''
    extra: dict = field(default_factory=dict)


def days_between(earlier: str, now: datetime.datetime) -> float:
    """Days from an ISO timestamp to now; infinite when there is no earlier time."""
    if not earlier:
        return float('inf')
    return (now - datetime.datetime.fromisoformat(earlier)).total_seconds() / 86400.0


class Strategy:
    """Base class: parameters in, persistent per-plant state kept in self.state."""

    name = 'base'
    needs_reading = True

    def __init__(self, params: dict, state: dict):
        """Keep the parameters and the persisted state (keyed by plant index as a string)."""
        self.params = params
        self.state = state

    def plant_state(self, index: int) -> dict:
        """Return the persisted state of one plant, created empty on first use."""
        return self.state.setdefault(str(index), {})

    def decide(self, obs: PlantObservation, ctx: CycleContext) -> Decision:
        """Return this cycle's decision for one plant."""
        raise NotImplementedError

    def nitrogen_due(self, obs: PlantObservation, ctx: CycleContext, interval_days: float) -> bool:
        """Check whether interval_days have passed since this plant's last nitrogen dose."""
        return days_between(self.plant_state(obs.index).get('last_n', ''),
                            ctx.now) >= interval_days

    def record_nitrogen(self, obs: PlantObservation, ctx: CycleContext):
        """Remember that this plant was dosed now."""
        self.plant_state(obs.index)['last_n'] = ctx.now.isoformat(timespec='seconds')


class FixedSchedule(Strategy):
    """The control group: the stock FarmBot routine, blind to the sensor."""

    name = 'fixed'
    needs_reading = False

    def decide(self, obs, ctx):
        """Give fixed water every cycle and fixed nitrogen every n_interval_days."""
        n_mg = 0.0
        if self.nitrogen_due(obs, ctx, self.params['n_interval_days']):
            n_mg = self.params['n_mg']
            self.record_nitrogen(obs, ctx)
        return Decision(water_ml=self.params['water_ml'], n_mg=n_mg, reason='schedule')


class SensorThreshold(Strategy):
    """Reactive IoT-style rules on the current reading only."""

    name = 'threshold'

    def decide(self, obs, ctx):
        """Refill to field capacity past the depletion threshold; dose N below its threshold."""
        if obs.reading is None:
            return Decision(reason='no valid reading, skipped')
        soil = ctx.soil
        depletion = soil.depletion_mm(obs.reading['moisture_pct'])
        root_n = soil.root_zone_mg(obs.reading['nitrogen_mg_kg'])

        water_ml = 0.0
        reasons = []
        if depletion > self.params['depletion_fraction'] * soil.taw_mm:
            water_ml = soil.ml_from_mm(depletion)
            reasons.append(f'depletion {depletion:.1f} mm > '
                           f"{self.params['depletion_fraction']:.0%} of TAW")

        n_mg = 0.0
        if (root_n < self.params['n_threshold_mg']
                and self.nitrogen_due(obs, ctx, self.params['n_min_interval_days'])):
            n_mg = self.params['n_dose_mg']
            self.record_nitrogen(obs, ctx)
            reasons.append(f'root-zone N {root_n:.0f} mg < {self.params["n_threshold_mg"]} mg')

        return Decision(water_ml=water_ml, n_mg=n_mg, reason='; '.join(reasons) or 'no action',
                        extra={'depletion_mm': depletion, 'root_zone_n_mg': root_n})


class Fao56WaterBalance(Strategy):
    """Predictive daily root-zone water balance, plus an uptake-based nitrogen budget."""

    name = 'fao56'
    needs_reading = False

    def decide(self, obs, ctx):
        """Advance the plant's depletion by Kc*ET0 since last cycle; irrigate past RAW."""
        p = self.params
        soil = ctx.soil
        st = self.plant_state(obs.index)
        now_iso = ctx.now.isoformat(timespec='seconds')

        if 'dr_mm' not in st:
            # Start from the probe when there is a reading, otherwise from field capacity
            st['dr_mm'] = (soil.depletion_mm(obs.reading['moisture_pct'])
                           if obs.reading else 0.0)
            st['n_uptake_mg'] = 0.0
            dt = 0.0
        else:
            dt = max(0.0, days_between(st['last_update'], ctx.now))
        st['last_update'] = now_iso

        et0 = fao56.hargreaves_et0(ctx.t_min, ctx.t_max, p['latitude_deg'],
                                   ctx.now.timetuple().tm_yday)
        kc = fao56.crop_coefficient(obs.days_since_planting, p['stage_days'], p['kc'])
        etc = kc * et0 * dt
        st['dr_mm'] = min(st['dr_mm'] + etc - p['rain_mm_per_day'] * dt, soil.taw_mm)
        st['dr_mm'] = max(st['dr_mm'], 0.0)
        if p['assimilate_sensor'] and obs.reading is not None:
            st['dr_mm'] = soil.depletion_mm(obs.reading['moisture_pct'])

        raw = p['p'] * soil.taw_mm
        water_ml = 0.0
        reasons = [f'ETc {etc:.2f} mm (Kc {kc:.2f}, ET0 {et0:.2f} mm/d, '
                   f'T {ctx.temperature_source})']
        if st['dr_mm'] > raw:
            water_ml = soil.ml_from_mm(st['dr_mm'])
            reasons.append(f"Dr {st['dr_mm']:.1f} mm > RAW {raw:.1f} mm")
            st['dr_mm'] = 0.0

        stage = fao56.stage_index(obs.days_since_planting, p['stage_days'])
        st['n_uptake_mg'] += p['n_uptake_mg_per_day'][stage] * dt
        n_mg = 0.0
        if self.nitrogen_due(obs, ctx, p['n_interval_days']) and st['n_uptake_mg'] > 0:
            n_mg = p['n_factor'] * st['n_uptake_mg']
            reasons.append(f"N budget {p['n_factor']} x {st['n_uptake_mg']:.0f} mg uptake")
            st['n_uptake_mg'] = 0.0
            self.record_nitrogen(obs, ctx)

        return Decision(water_ml=water_ml, n_mg=n_mg, reason='; '.join(reasons),
                        extra={'dr_mm': st['dr_mm'], 'et0_mm': et0, 'kc': kc})


STRATEGIES = {cls.name: cls for cls in (FixedSchedule, SensorThreshold, Fao56WaterBalance)}
