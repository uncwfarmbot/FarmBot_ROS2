"""
FAO-56 reference evapotranspiration by the Hargreaves equation.

Allen, Pereira, Raes and Smith, Crop Evapotranspiration, FAO Irrigation and Drainage Paper 56
(1998): extraterrestrial radiation from eqs. 21-25, Hargreaves from eq. 52.
"""
import math

SOLAR_CONSTANT = 0.0820  # MJ m-2 min-1


def extraterrestrial_radiation(latitude_deg: float, day_of_year: int) -> float:
    """Ra in MJ m-2 day-1 (FAO-56 eq. 21)."""
    phi = math.radians(latitude_deg)
    dr = 1 + 0.033 * math.cos(2 * math.pi * day_of_year / 365)              # eq. 23
    delta = 0.409 * math.sin(2 * math.pi * day_of_year / 365 - 1.39)        # eq. 24
    ws = math.acos(max(-1.0, min(1.0, -math.tan(phi) * math.tan(delta))))   # eq. 25
    return (24 * 60 / math.pi * SOLAR_CONSTANT * dr
            * (ws * math.sin(phi) * math.sin(delta)
               + math.cos(phi) * math.cos(delta) * math.sin(ws)))


def hargreaves_et0(t_min: float, t_max: float, latitude_deg: float, day_of_year: int) -> float:
    """Return reference evapotranspiration ET0 in mm/day (FAO-56 eq. 52)."""
    ra_mm = 0.408 * extraterrestrial_radiation(latitude_deg, day_of_year)
    t_mean = (t_min + t_max) / 2
    return max(0.0, 0.0023 * (t_mean + 17.8) * math.sqrt(max(t_max - t_min, 0.0)) * ra_mm)


def crop_coefficient(days: float, stages: list, kc: list) -> float:
    """
    Return Kc for a crop age, following the FAO-56 piecewise-linear curve (fig. 34).

    days is days since planting, stages the lengths in days of the initial, development, mid
    and late stages, and kc the (Kc ini, Kc mid, Kc end) triple.
    """
    ini, dev, mid, late = stages
    kc_ini, kc_mid, kc_end = kc
    if days <= ini:
        return kc_ini
    if days <= ini + dev:
        return kc_ini + (kc_mid - kc_ini) * (days - ini) / dev
    if days <= ini + dev + mid:
        return kc_mid
    if days <= ini + dev + mid + late:
        return kc_mid + (kc_end - kc_mid) * (days - ini - dev - mid) / late
    return kc_end


def stage_index(days: float, stages: list) -> int:
    """0-3 for the initial, development, mid and late stage a crop age falls in."""
    total = 0
    for i, length in enumerate(stages):
        total += length
        if days <= total:
            return i
    return len(stages) - 1
