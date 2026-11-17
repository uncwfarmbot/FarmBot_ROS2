"""Tests for the soil model, FAO-56 maths, the four strategies and the action mapping."""
import datetime

from farmbot_agronomy import fao56
from farmbot_agronomy.action_mapper import Actuation, map_command, to_action
from farmbot_agronomy.soil_model import SoilModel
from farmbot_agronomy.strategies import (CycleContext, Fao56WaterBalance, FixedSchedule,
                                         PlantObservation, SensorThreshold)

import pytest
import yaml

CONFIG = yaml.safe_load(open(__file__.replace('test/test_controllers.py',
                                              'config/agronomy.yaml')))
SOIL = SoilModel(**CONFIG['soil'])
T0 = datetime.datetime(2026, 6, 1, 7, 0)


def reading(moisture=30.0, n=42.0):
    return {'moisture_pct': moisture, 'soil_temp_c': 21.0, 'ec_us_cm': 650.0, 'ph': 6.6,
            'nitrogen_mg_kg': n, 'phosphorus_mg_kg': 58.0, 'potassium_mg_kg': 130.0}


def ctx(days=0.0):
    return CycleContext(now=T0 + datetime.timedelta(days=days), soil=SOIL)


def test_soil_model():
    assert SOIL.taw_mm == pytest.approx(40.0)
    assert SOIL.depletion_mm(35.0) == 0.0          # at field capacity
    assert SOIL.depletion_mm(25.0) == pytest.approx(20.0)
    assert SOIL.depletion_mm(5.0) == pytest.approx(40.0)  # capped at TAW
    assert SOIL.ml_from_mm(10.0) == pytest.approx(400.0)  # 10 mm over 0.04 m2
    assert SOIL.root_zone_soil_kg == pytest.approx(9.6)


def test_ra_matches_fao56_example_8():
    # FAO-56 example 8: 20 deg S on 3 September -> Ra = 32.2 MJ m-2 day-1
    assert fao56.extraterrestrial_radiation(-20.0, 246) == pytest.approx(32.2, abs=0.1)


def test_hargreaves_is_plausible_for_wilmington_summer():
    et0 = fao56.hargreaves_et0(21.0, 31.0, 34.2, 182)
    assert 4.5 < et0 < 6.5


def test_crop_coefficient_curve():
    stages, kc = [20, 30, 30, 10], [0.6, 1.0, 0.95]
    assert fao56.crop_coefficient(10, stages, kc) == 0.6
    assert fao56.crop_coefficient(35, stages, kc) == pytest.approx(0.8)
    assert fao56.crop_coefficient(60, stages, kc) == 1.0
    assert fao56.crop_coefficient(200, stages, kc) == 0.95
    assert fao56.stage_index(25, stages) == 1


def test_fixed_schedule_doses_n_weekly():
    s = FixedSchedule(CONFIG['fixed'], {})
    obs = PlantObservation(index=1, days_since_planting=10)
    first = s.decide(obs, ctx(0))
    assert (first.water_ml, first.n_mg) == (250.0, 120.0)
    assert s.decide(obs, ctx(3)).n_mg == 0.0
    assert s.decide(obs, ctx(7)).n_mg == 120.0


def test_threshold_waters_back_to_field_capacity():
    s = SensorThreshold(CONFIG['threshold'], {})
    wet = s.decide(PlantObservation(1, 10, reading(moisture=32.0)), ctx())
    assert wet.water_ml == 0.0                     # 6 mm < 14 mm
    dry = s.decide(PlantObservation(1, 10, reading(moisture=25.0)), ctx())
    assert dry.water_ml == pytest.approx(800.0)    # 20 mm back to FC over 0.04 m2


def test_threshold_nitrogen_and_redose_interval():
    s = SensorThreshold(CONFIG['threshold'], {})
    low_n = reading(n=20.0)                        # 20 mg/kg x 9.6 kg = 192 mg < 300
    assert s.decide(PlantObservation(1, 10, low_n), ctx(0)).n_mg == 150.0
    assert s.decide(PlantObservation(1, 10, low_n), ctx(1)).n_mg == 0.0
    assert s.decide(PlantObservation(1, 10, low_n), ctx(3)).n_mg == 150.0
    assert s.decide(PlantObservation(2, 10, reading(n=42.0)), ctx(0)).n_mg == 0.0


def test_threshold_skips_without_reading():
    d = SensorThreshold(CONFIG['threshold'], {}).decide(PlantObservation(1, 10), ctx())
    assert d.water_ml == 0.0 and 'no valid reading' in d.reason


def test_fao56_accumulates_then_irrigates():
    state = {}
    s = Fao56WaterBalance(CONFIG['fao56'], state)
    obs = PlantObservation(1, 50, reading(moisture=35.0))
    assert s.decide(obs, ctx(0)).water_ml == 0.0   # initialised at field capacity
    waters = [s.decide(PlantObservation(1, 50 + d), ctx(d)).water_ml for d in range(1, 8)]
    # Kc 1.0 x ~4.5 mm/day crosses RAW (14 mm) on day 4, then refills to FC
    assert waters.index(next(w for w in waters if w > 0)) in (2, 3)
    assert state['1']['dr_mm'] < CONFIG['fao56']['p'] * SOIL.taw_mm


def test_fao56_nitrogen_budget():
    s = Fao56WaterBalance(CONFIG['fao56'], {})
    s.decide(PlantObservation(1, 50, reading()), ctx(0))   # initialises, doses nothing
    doses = [s.decide(PlantObservation(1, 50 + d), ctx(d)).n_mg for d in range(1, 8)]
    # mid stage, 8 mg/day uptake: first dose after day 1, then nothing until 7 days later
    assert doses[0] == pytest.approx(1.2 * 8.0)
    assert doses[1:6] == [0.0] * 5
    assert doses[6] == 0.0
    assert s.decide(PlantObservation(1, 58), ctx(8)).n_mg == pytest.approx(1.2 * 8.0 * 7)


def test_action_mapping_quantises_and_caps():
    cal = Actuation(pump_ml_per_s=20.0, granule_mg=30.0, max_water_ml=1000.0, max_granules=10)
    a = to_action(3, water_ml=250.0, n_mg=150.0, cal=cal)
    assert (a.water_ms, a.granules, a.n_mg) == (12500, 5, 150.0)
    capped = to_action(3, water_ml=5000.0, n_mg=9000.0, cal=cal)
    assert (capped.water_ml, capped.granules) == (1000.0, 10)
    tiny = to_action(3, water_ml=1.0, n_mg=10.0, cal=cal)
    assert (tiny.water_ms, tiny.granules) == (0, 0)


def test_map_command_skips_no_ops():
    cal = Actuation()
    actions = [to_action(1, 250.0, 0.0, cal), to_action(2, 0.0, 0.0, cal)]
    assert map_command(actions) == 'P_11 1:12500:0'
