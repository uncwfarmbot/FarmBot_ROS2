"""
Turn decisions in mL and mg into what the robot can actually do.

Water is time-based (there is no flow meter), so a volume becomes pump milliseconds through a
measured pump rate. Nitrogen is granules placed by the seeder, so a mass becomes a whole number
of granules through a weighed granule mass. Both are capped, because a calibration typo should
cost a dry plant, not a flooded or burnt one.
"""
from dataclasses import dataclass


@dataclass
class Actuation:
    """Calibration and safety limits."""

    pump_ml_per_s: float = 20.0
    granule_mg: float = 30.0
    max_water_ml: float = 1000.0
    max_granules: int = 10
    min_water_ms: int = 200     # shorter pulses only dribble


@dataclass
class PlantAction:
    """One plant's actuation for this cycle."""

    index: int
    water_ml: float
    water_ms: int
    n_mg: float
    granules: int

    def as_map_arg(self) -> str:
        """Format as the 'index:water_ms:granules' argument map_info P_11 takes."""
        return f'{self.index}:{self.water_ms}:{self.granules}'


def to_action(index: int, water_ml: float, n_mg: float, cal: Actuation) -> PlantAction:
    """Quantise and cap one plant's decision."""
    water_ml = min(max(water_ml, 0.0), cal.max_water_ml)
    water_ms = int(round(water_ml / cal.pump_ml_per_s * 1000.0))
    if water_ms < cal.min_water_ms:
        water_ms = 0
    granules = min(int(round(max(n_mg, 0.0) / cal.granule_mg)), cal.max_granules)
    return PlantAction(index=index,
                       water_ml=water_ms / 1000.0 * cal.pump_ml_per_s,
                       water_ms=water_ms,
                       n_mg=granules * cal.granule_mg,
                       granules=granules)


def map_command(actions: list) -> str:
    """Build the map_info request that turns these actions into a sequence."""
    return ' '.join(['P_11'] + [a.as_map_arg() for a in actions
                                if a.water_ms > 0 or a.granules > 0])
