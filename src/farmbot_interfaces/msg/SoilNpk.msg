# One reading of the JXCT 7-in-1 soil probe (RS-485 Modbus RTU) carried on the UTM.
#
# Values are as the probe reports them, scaled to physical units and nothing more. No
# calibration, no normalisation: that belongs to whoever builds a controller's observation.
# N, P and K are estimated by the probe from EC, so treat them as repeatable rather than accurate.

std_msgs/Header header

int64 plant_index           # map index of the plant this was read beside, -1 for an ad hoc read

float32 moisture_pct        # volumetric water content, %
float32 soil_temp_c         # degrees C, may be negative
float32 ec_us_cm            # electrical conductivity, uS/cm
float32 ph                  # pH units
float32 nitrogen_mg_kg      # mg/kg
float32 phosphorus_mg_kg    # mg/kg
float32 potassium_mg_kg     # mg/kg

# False when the read failed or a value was outside the datasheet range. Controllers dose
# water and fertilizer off these numbers, so a bad reading is never marked valid.
bool valid
string status               # "ok", "mock", or the failure reason
uint16[] raw_registers      # the registers as read, in read order, for later re-decoding
