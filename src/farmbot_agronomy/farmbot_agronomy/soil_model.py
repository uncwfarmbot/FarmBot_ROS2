"""
Conversion between what the probe reports and what the controllers reason in.

This is the sim-to-real seam, kept in one place. The probe reports volumetric moisture (%) and
nutrient concentrations (mg/kg); the controllers from the simulation reason in root-zone
depletion (mm) and nutrient mass per plant (mg). Every number here is a per-bed calibration:
measure theta_fc and theta_wp by saturating a cell, letting it drain for a day and reading the
probe, and dry-down to wilting for the other end.
"""
from dataclasses import dataclass


@dataclass
class SoilModel:
    """Per-bed soil and plant geometry."""

    theta_fc: float = 0.35          # field capacity, volumetric fraction
    theta_wp: float = 0.15          # wilting point, volumetric fraction
    root_depth_mm: float = 200.0    # effective root-zone depth
    plant_area_m2: float = 0.04     # bed area each plant draws from (20 x 20 cm)
    bulk_density_g_cm3: float = 1.2

    @property
    def taw_mm(self) -> float:
        """Total available water in the root zone (FAO-56 eq. 82)."""
        return (self.theta_fc - self.theta_wp) * self.root_depth_mm

    def depletion_mm(self, moisture_pct: float) -> float:
        """Root-zone depletion below field capacity, from a probe moisture reading."""
        theta = moisture_pct / 100.0
        return min(max((self.theta_fc - theta) * self.root_depth_mm, 0.0), self.taw_mm)

    def ml_from_mm(self, depth_mm: float) -> float:
        """Water volume that raises the plant's area by depth_mm (1 mm over 1 m2 is 1 L)."""
        return depth_mm * self.plant_area_m2 * 1000.0

    @property
    def root_zone_soil_kg(self) -> float:
        """Mass of soil in one plant's root zone."""
        volume_m3 = self.plant_area_m2 * self.root_depth_mm / 1000.0
        return volume_m3 * self.bulk_density_g_cm3 * 1000.0

    def root_zone_mg(self, concentration_mg_kg: float) -> float:
        """Nutrient mass in one plant's root zone, from a probe concentration."""
        return concentration_mg_kg * self.root_zone_soil_kg
