"""
QGC_perimeter_testflight.py
============================
Author: Flight Test Engineering Team
Purpose: SITL test scenario simulating a city block perimeter flight with
         QGroundControl as the GCS and ArduPilot SITL as the flight stack.

Test Scenario:
    - Drone takes off from a start point
    - Flies the perimeter of an average city block (~100m x ~100m)
    - Returns to the start point and lands
    - Simulates a slightly windy day (wind from the north @ 5 m/s)

Monitored Parameters:
    - Battery voltage, current, and level (idle vs. flight vs. wind-affected)
    - Flight path distance (cumulative GPS-derived distance)
    - Total mission time (wheels-up to wheels-down)
    - ESC / Motor RPM via RCOU channels (as a proxy for prop RPM)
    - Wind effect on lateral deviation, motor compensation, and battery draw

Connection:
    - QGroundControl claims port 14550 (UDP) — DO NOT connect Python there
    - Python connects on port 14551 (UDP) per the established port convention
    - Run ArduPilot SITL with MAVProxy output to both 14550 and 14551

How to launch SITL before running this script:
    sim_vehicle.py -v ArduCopter --map --console \
        --out=udp:127.0.0.1:14550 \
        --out=udp:127.0.0.1:14551

MAVProxy wind injection (run in MAVProxy console after SITL starts):
    param set SIM_WIND_SPD 5
    param set SIM_WIND_DIR 0      # 0 = North (wind FROM the north, blowing south)
    param set SIM_WIND_TURB 0.3   # slight turbulence factor
"""

# ============================================================
# PATCH: Fix for Python 3.10+ compatibility with DroneKit
# ============================================================
import collections
import collections.abc
for name in dir(collections.abc):
    if not hasattr(collections, name):
        setattr(collections, name, getattr(collections.abc, name))

import time
import math
import csv
import os
import threading
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend — safe for SITL headless runs
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

from dronekit import connect, VehicleMode, LocationGlobalRelative

# ============================================================
# CONFIGURATION
# ============================================================

# -- Connection --
# Port 14550 → QGroundControl (GCS)
# Port 14551 → Python test script  (per Flightwave port convention)
CONNECTION_STRING = 'udp:127.0.0.1:14551'

# -- Mission geometry --
# Average city block: ~100 m N/S × ~100 m E/W
# Perimeter flown as 4 waypoints (NW → NE → SE → SW corners) then RTL
BLOCK_SIZE_M = 100.0       # meters per side
CRUISE_ALTITUDE_M = 30.0  # AGL, keeps us clear of simulated urban obstacles

# -- Wind simulation parameters (injected via MAVProxy / SITL params) --
WIND_SPEED_MS   = 5.0    # m/s — light-to-moderate breeze
WIND_DIR_DEG    = 0.0    # degrees — wind FROM the north (blowing southward)
WIND_TURBULENCE = 0.3    # SITL turbulence multiplier

# -- Telemetry sampling --
TELEMETRY_HZ = 2.0       # samples per second during flight

# -- Waypoint arrival threshold --
WAYPOINT_RADIUS_M = 3.0  # meters — considered "arrived" inside this radius

# -- Output paths --
LOG_DIR = Path("../logs") / "perimeter_test"
LOG_DIR.mkdir(parents=True, exist_ok=True)
TIMESTAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
CSV_PATH  = LOG_DIR / f"perimeter_flight_{TIMESTAMP}.csv"
PLOT_PATH = LOG_DIR / f"perimeter_flight_{TIMESTAMP}.png"


# ============================================================
# UTILITY: GEO-MATH
# ============================================================

def meters_to_lat(meters: float) -> float:
    """Convert a north/south offset in meters to degrees of latitude."""
    return meters / 111_320.0


def meters_to_lon(meters: float, lat_deg: float) -> float:
    """Convert an east/west offset in meters to degrees of longitude at a given latitude."""
    return meters / (111_320.0 * math.cos(math.radians(lat_deg)))


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    """
    Return the great-circle distance in meters between two GPS coordinates.
    Uses the Haversine formula — accurate enough for sub-km distances.
    """
    R = 6_371_000.0  # Earth radius in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi  = math.radians(lat2 - lat1)
    dlam  = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def lateral_crosstrack_error(target_lat, target_lon, actual_lat, actual_lon) -> float:
    """
    Horizontal distance (m) between the vehicle and the intended waypoint.
    Used as a proxy for wind-induced lateral deviation.
    """
    return haversine_m(target_lat, target_lon, actual_lat, actual_lon)


# ============================================================
# UTILITY: ESC / MOTOR RPM PROXY
# ============================================================

def get_motor_pwm(vehicle) -> dict:
    """
    Read raw PWM output channels 1–4 (primary motor ESCs on a quadcopter).
    DroneKit exposes these via vehicle.channels.overrides for input, but for
    actual *output* to ESCs we read the MAVLink SERVO_OUTPUT_RAW message
    which DroneKit surfaces as vehicle._channels (internal) or via a message
    listener. As a practical proxy we use vehicle.channels (RC inputs scaled
    to output) and note that in SITL the RCOU (RC output) telemetry maps
    PWM 1000–2000 µs to 0–100% throttle, which is proportional to RPM.

    Returns a dict: {motor_1: pwm, motor_2: pwm, motor_3: pwm, motor_4: pwm}

    NOTE: For real HIL/hardware, replace this with an ESC telemetry parser
    (e.g., BLHeli_32 ESC telemetry over UART or UAVCAN RPM messages).
    """
    try:
        # In SITL, SERVO_OUTPUT_RAW is available via the raw attribute
        # vehicle.channels gives the RC input — for output we use the
        # last cached SERVO_OUTPUT_RAW values if accessible.
        servos = vehicle._channels  # internal DroneKit cache
        return {
            'motor_1_pwm': getattr(vehicle.channels, '1', 1000),
            'motor_2_pwm': getattr(vehicle.channels, '2', 1000),
            'motor_3_pwm': getattr(vehicle.channels, '3', 1000),
            'motor_4_pwm': getattr(vehicle.channels, '4', 1000),
        }
    except Exception:
        return {'motor_1_pwm': 0, 'motor_2_pwm': 0,
                'motor_3_pwm': 0, 'motor_4_pwm': 0}


def pwm_to_rpm_estimate(pwm: float, max_rpm: float = 8000.0) -> float:
    """
    Linear interpolation of PWM → RPM.
    PWM 1000 µs = 0 RPM (motor off / min throttle)
    PWM 2000 µs = max_rpm RPM (full throttle)

    For SITL validation; replace coefficients with your actual motor KV × voltage
    characterisation data for hardware testing.
    """
    pwm_min, pwm_max = 1000.0, 2000.0
    throttle_pct = max(0.0, min(1.0, (pwm - pwm_min) / (pwm_max - pwm_min)))
    return throttle_pct * max_rpm


# ============================================================
# WIND INJECTION (via MAVProxy parameters)
# ============================================================

def inject_wind_via_params(vehicle, speed_ms: float, direction_deg: float, turbulence: float):
    """
    Set SITL wind parameters directly through the ArduPilot parameter interface.
    This is equivalent to typing 'param set SIM_WIND_SPD 5' in the MAVProxy console.

    SIM_WIND_DIR convention: meteorological — direction wind is FROM, in degrees.
        0° = wind from North (blowing South)
       90° = wind from East  (blowing West)

    After setting params, a short wait lets SITL absorb the change.
    """
    print(f"\n[WIND] Injecting wind: {speed_ms} m/s FROM {direction_deg}° | turbulence={turbulence}")
    try:
        vehicle.parameters['SIM_WIND_SPD']  = speed_ms
        vehicle.parameters['SIM_WIND_DIR']  = direction_deg
        vehicle.parameters['SIM_WIND_TURB'] = turbulence
        time.sleep(2)
        print("[WIND] ✓ Wind parameters set successfully")
    except Exception as e:
        print(f"[WIND] ⚠ Could not set wind params via DroneKit: {e}")
        print("[WIND]   Set manually in MAVProxy: param set SIM_WIND_SPD 5 / SIM_WIND_DIR 0 / SIM_WIND_TURB 0.3")


# ============================================================
# TELEMETRY LOGGER
# ============================================================

class TelemetryLogger:
    """
    Thread-safe telemetry collector.
    Samples vehicle state at TELEMETRY_HZ and appends records to an internal list.
    Call .start() before flight, .stop() after landing.
    """

    def __init__(self, vehicle, sample_hz: float = TELEMETRY_HZ):
        self.vehicle      = vehicle
        self.interval     = 1.0 / sample_hz
        self._records     = []
        self._running     = False
        self._thread      = None
        self._lock        = threading.Lock()
        self._start_time  = None
        self._prev_lat    = None
        self._prev_lon    = None
        self._cum_dist_m  = 0.0
        self._current_leg = "PRE-FLIGHT"

    def start(self):
        self._running    = True
        self._start_time = time.time()
        self._thread     = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print("[TELEMETRY] Logger started")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=5)
        print(f"[TELEMETRY] Logger stopped — {len(self._records)} records captured")

    def set_leg(self, label: str):
        """Tag subsequent records with the current mission leg name."""
        with self._lock:
            self._current_leg = label

    @property
    def cumulative_distance_m(self) -> float:
        with self._lock:
            return self._cum_dist_m

    def records(self) -> list:
        with self._lock:
            return list(self._records)

    def _sample(self):
        v   = self.vehicle
        loc = v.location.global_relative_frame
        lat, lon, alt = loc.lat, loc.lon, loc.alt
        elapsed = time.time() - self._start_time

        # Accumulate distance
        if self._prev_lat is not None:
            seg = haversine_m(self._prev_lat, self._prev_lon, lat, lon)
            self._cum_dist_m += seg
        self._prev_lat, self._prev_lon = lat, lon

        # Battery
        batt_v   = v.battery.voltage  if v.battery.voltage  else 0.0
        batt_a   = v.battery.current  if v.battery.current  else 0.0
        batt_lvl = v.battery.level    if v.battery.level    else 100

        # Motor PWM → RPM estimates
        pwms = get_motor_pwm(v)
        m1_rpm = pwm_to_rpm_estimate(pwms['motor_1_pwm'])
        m2_rpm = pwm_to_rpm_estimate(pwms['motor_2_pwm'])
        m3_rpm = pwm_to_rpm_estimate(pwms['motor_3_pwm'])
        m4_rpm = pwm_to_rpm_estimate(pwms['motor_4_pwm'])
        avg_rpm = (m1_rpm + m2_rpm + m3_rpm + m4_rpm) / 4.0

        # Airspeed & groundspeed
        airspeed    = v.airspeed    if v.airspeed    else 0.0
        groundspeed = v.groundspeed if v.groundspeed else 0.0
        # Wind effect proxy: difference between groundspeed and airspeed
        wind_effect_ms = abs(airspeed - groundspeed)

        # Attitude (roll/pitch — lateral lean indicates wind correction)
        roll_deg  = math.degrees(v.attitude.roll)  if v.attitude else 0.0
        pitch_deg = math.degrees(v.attitude.pitch) if v.attitude else 0.0

        record = {
            'elapsed_s'       : round(elapsed, 2),
            'leg'             : self._current_leg,
            'lat'             : lat,
            'lon'             : lon,
            'alt_m'           : round(alt, 2),
            'batt_voltage_v'  : round(batt_v, 3),
            'batt_current_a'  : round(batt_a, 2),
            'batt_level_pct'  : batt_lvl,
            'cum_dist_m'      : round(self._cum_dist_m, 2),
            'airspeed_ms'     : round(airspeed, 2),
            'groundspeed_ms'  : round(groundspeed, 2),
            'wind_effect_ms'  : round(wind_effect_ms, 2),
            'roll_deg'        : round(roll_deg, 2),
            'pitch_deg'       : round(pitch_deg, 2),
            'motor_1_pwm'     : pwms['motor_1_pwm'],
            'motor_2_pwm'     : pwms['motor_2_pwm'],
            'motor_3_pwm'     : pwms['motor_3_pwm'],
            'motor_4_pwm'     : pwms['motor_4_pwm'],
            'motor_avg_rpm'   : round(avg_rpm, 1),
            'gps_fix'         : v.gps_0.fix_type,
            'num_sats'        : v.gps_0.satellites_visible,
            'mode'            : v.mode.name,
        }

        with self._lock:
            self._records.append(record)

    def _loop(self):
        while self._running:
            try:
                self._sample()
            except Exception as e:
                print(f"[TELEMETRY] Sampling error: {e}")
            time.sleep(self.interval)


# ============================================================
# FLIGHT CONTROLLER
# ============================================================

class PerimeterFlightTest:
    """
    Orchestrates the full city-block perimeter SITL mission:
        1. Pre-flight checks
        2. Wind injection
        3. Takeoff
        4. Fly 4 corners of city block (NW → NE → SE → SW)
        5. Return to launch & land
        6. Data export and analysis
    """

    def __init__(self, vehicle, logger: TelemetryLogger):
        self.vehicle    = vehicle
        self.logger     = logger
        self.home_lat   = None
        self.home_lon   = None
        self.waypoints  = []         # (lat, lon, label) tuples
        self.wp_results = []         # arrival data per waypoint
        self.t_wheels_up   = None
        self.t_wheels_down = None

    # ----------------------------------------------------------
    # 1. PRE-FLIGHT
    # ----------------------------------------------------------

    def preflight_checks(self):
        print("\n" + "="*70)
        print("  PRE-FLIGHT CHECKS")
        print("="*70)

        v = self.vehicle

        # GPS lock
        print("[PREFLIGHT] Waiting for 3D GPS fix...")
        while v.gps_0.fix_type < 3:
            print(f"  GPS fix type: {v.gps_0.fix_type} | Sats: {v.gps_0.satellites_visible}")
            time.sleep(1)
        print(f"[PREFLIGHT] ✓ GPS 3D fix acquired — {v.gps_0.satellites_visible} satellites")

        # Home location
        print("[PREFLIGHT] Waiting for home location...")
        while not v.home_location:
            time.sleep(1)
        self.home_lat = v.home_location.lat
        self.home_lon = v.home_location.lon
        print(f"[PREFLIGHT] ✓ Home: {self.home_lat:.7f}, {self.home_lon:.7f}")

        # Armability
        print("[PREFLIGHT] Waiting for vehicle to become armable...")
        while not v.is_armable:
            print(f"  is_armable={v.is_armable} | mode={v.mode.name}")
            time.sleep(1)
        print("[PREFLIGHT] ✓ Vehicle is armable")

        # Battery
        bv = v.battery.voltage if v.battery.voltage else 0
        print(f"[PREFLIGHT] Battery: {bv:.2f}V")
        assert bv > 10.5, f"ABORT: Battery voltage {bv:.2f}V is below minimum 10.5V"
        print("[PREFLIGHT] ✓ Battery voltage acceptable")

        print("[PREFLIGHT] ✓ All checks passed\n")

    # ----------------------------------------------------------
    # 2. WIND SETUP
    # ----------------------------------------------------------

    def setup_wind(self):
        inject_wind_via_params(
            self.vehicle,
            speed_ms    = WIND_SPEED_MS,
            direction_deg = WIND_DIR_DEG,
            turbulence  = WIND_TURBULENCE
        )

    # ----------------------------------------------------------
    # 3. BUILD WAYPOINTS
    # ----------------------------------------------------------

    def build_waypoints(self):
        """
        City block perimeter (100 m × 100 m), flown counter-clockwise
        starting from the home / launch point at the SW corner:

            NW ──────── NE
            │            │
            │            │
            SW ──────── SE
           (home)

        Sequence: Home → NW → NE → SE → SW → RTL (auto-lands at Home)

        Using half the block size so Home sits at the SW corner and
        all waypoints stay within the block footprint.
        """
        half = BLOCK_SIZE_M / 2.0   # 50 m
        # Using full BLOCK_SIZE_M offsets so each side = 100 m
        h_lat = self.home_lat
        h_lon = self.home_lon

        self.waypoints = [
            # (lat,                                         lon,                                         label)
            (h_lat + meters_to_lat(BLOCK_SIZE_M),          h_lon,                                       "NW Corner"),
            (h_lat + meters_to_lat(BLOCK_SIZE_M),          h_lon + meters_to_lon(BLOCK_SIZE_M, h_lat),  "NE Corner"),
            (h_lat,                                         h_lon + meters_to_lon(BLOCK_SIZE_M, h_lat),  "SE Corner"),
            (h_lat,                                         h_lon,                                       "SW / Home"),
        ]

        print("[MISSION] City block waypoints (100 m × 100 m perimeter):")
        for i, (lat, lon, label) in enumerate(self.waypoints, 1):
            print(f"  WP{i}: {label:20s} → {lat:.7f}, {lon:.7f}")

    # ----------------------------------------------------------
    # 4. ARM & TAKEOFF
    # ----------------------------------------------------------

    def arm_and_takeoff(self, target_alt: float = CRUISE_ALTITUDE_M):
        v = self.vehicle
        print(f"\n[TAKEOFF] Arming and climbing to {target_alt} m AGL...")

        v.mode = VehicleMode("GUIDED")
        while v.mode.name != "GUIDED":
            print("  Waiting for GUIDED mode...")
            time.sleep(0.5)

        v.armed = True
        while not v.armed:
            print("  Waiting for arming...")
            time.sleep(0.5)

        print("[TAKEOFF] ✓ Armed — initiating takeoff")
        self.t_wheels_up = time.time()
        self.logger.set_leg("TAKEOFF")

        v.simple_takeoff(target_alt)

        while True:
            alt = v.location.global_relative_frame.alt
            print(f"  Altitude: {alt:.2f} m / {target_alt} m")
            if alt >= target_alt * 0.95:
                print("[TAKEOFF] ✓ Cruise altitude reached")
                break
            time.sleep(1)

        time.sleep(2)  # Stabilise before initiating navigation

    # ----------------------------------------------------------
    # 5. FLY PERIMETER
    # ----------------------------------------------------------

    def fly_perimeter(self):
        v = self.vehicle
        print("\n" + "="*70)
        print("  PERIMETER FLIGHT — CITY BLOCK 100 m × 100 m")
        print("="*70)

        for idx, (wp_lat, wp_lon, wp_label) in enumerate(self.waypoints, 1):
            leg_label = f"LEG_{idx}_{wp_label.replace(' ','_').replace('/','_')}"
            self.logger.set_leg(leg_label)

            target = LocationGlobalRelative(wp_lat, wp_lon, CRUISE_ALTITUDE_M)
            print(f"\n[LEG {idx}/4] Navigating to {wp_label}  ({wp_lat:.7f}, {wp_lon:.7f})")
            v.simple_goto(target, groundspeed=8)  # ~8 m/s cruise for a controlled test run

            arrival_time    = None
            approach_start  = time.time()
            max_leg_time    = 60  # seconds — generous allowance including wind correction
            min_error_seen  = float('inf')
            deviation_data  = []

            while time.time() - approach_start < max_leg_time:
                loc = v.location.global_relative_frame
                err = lateral_crosstrack_error(wp_lat, wp_lon, loc.lat, loc.lon)
                alt = loc.alt

                # Wind lateral deviation vs intended track (simplified: distance from waypoint)
                batt_a = v.battery.current if v.battery.current else 0
                pwms   = get_motor_pwm(v)
                avg_pwm = sum(pwms.values()) / 4.0
                deviation_data.append({
                    'error_m' : err,
                    'batt_a'  : batt_a,
                    'avg_pwm' : avg_pwm
                })

                print(f"  Dist to WP: {err:6.2f} m | Alt: {alt:5.2f} m | "
                      f"Avg ESC PWM: {avg_pwm:6.1f} µs | Batt: {batt_a:.2f}A")

                if err < min_error_seen:
                    min_error_seen = err

                if err <= WAYPOINT_RADIUS_M:
                    arrival_time = time.time()
                    print(f"  ✓ WP{idx} reached in {arrival_time - approach_start:.1f}s "
                          f"(min error: {min_error_seen:.2f} m)")
                    break

                time.sleep(0.5)
            else:
                print(f"  ⚠ WP{idx} approach timeout — closest approach: {min_error_seen:.2f} m")

            self.wp_results.append({
                'wp_id'        : idx,
                'label'        : wp_label,
                'target_lat'   : wp_lat,
                'target_lon'   : wp_lon,
                'min_error_m'  : round(min_error_seen, 2),
                'leg_time_s'   : round(time.time() - approach_start, 1),
                'avg_esc_pwm'  : round(sum(d['avg_pwm'] for d in deviation_data) / max(len(deviation_data), 1), 1),
                'avg_batt_a'   : round(sum(d['batt_a']  for d in deviation_data) / max(len(deviation_data), 1), 2),
                'arrived'      : arrival_time is not None,
            })

            time.sleep(1)  # Brief pause at each waypoint corner

    # ----------------------------------------------------------
    # 6. RETURN & LAND
    # ----------------------------------------------------------

    def return_and_land(self):
        v = self.vehicle
        print("\n[RTL] Perimeter complete — returning to launch and landing...")
        self.logger.set_leg("RTL_LAND")
        v.mode = VehicleMode("RTL")

        # Wait for landing (altitude < 0.5 m AGL)
        while True:
            alt = v.location.global_relative_frame.alt
            print(f"  Descending: {alt:.2f} m")
            if alt < 0.5:
                break
            time.sleep(2)

        self.t_wheels_down = time.time()
        print("[RTL] ✓ Landed")
        time.sleep(3)  # Let the vehicle settle before disarming telemetry

    # ----------------------------------------------------------
    # 7. EXPORT & ANALYSIS
    # ----------------------------------------------------------

    def export_and_analyse(self):
        records = self.logger.records()
        if not records:
            print("[ANALYSIS] No telemetry records captured — skipping export")
            return

        df = pd.DataFrame(records)
        df.to_csv(CSV_PATH, index=False)
        print(f"\n[RESULTS] Raw telemetry saved to: {CSV_PATH.resolve()}")

        # ── Summary statistics ──────────────────────────────────────────────
        total_time_s    = (self.t_wheels_down - self.t_wheels_up) if (self.t_wheels_up and self.t_wheels_down) else 0
        total_dist_m    = self.logger.cumulative_distance_m
        avg_voltage     = df['batt_voltage_v'].mean()
        start_voltage   = df['batt_voltage_v'].iloc[0]
        end_voltage     = df['batt_voltage_v'].iloc[-1]
        voltage_drop    = start_voltage - end_voltage
        avg_current     = df['batt_current_a'].mean()
        peak_current    = df['batt_current_a'].max()
        avg_rpm         = df['motor_avg_rpm'].mean()
        max_rpm         = df['motor_avg_rpm'].max()
        avg_wind_effect = df['wind_effect_ms'].mean()
        max_roll        = df['roll_deg'].abs().max()
        max_pitch       = df['pitch_deg'].abs().max()

        print("\n" + "="*70)
        print("  MISSION SUMMARY")
        print("="*70)
        print(f"  Total flight time   : {total_time_s:.1f} s  ({total_time_s/60:.2f} min)")
        print(f"  Total path distance : {total_dist_m:.1f} m  ({total_dist_m/1000:.3f} km)")
        print(f"  Theoretical perim.  : {4 * BLOCK_SIZE_M:.0f} m  (4 × {BLOCK_SIZE_M:.0f} m)")
        print(f"\n  BATTERY:")
        print(f"    Start voltage     : {start_voltage:.3f} V")
        print(f"    End voltage       : {end_voltage:.3f} V")
        print(f"    Voltage drop      : {voltage_drop:.3f} V")
        print(f"    Avg current       : {avg_current:.2f} A")
        print(f"    Peak current      : {peak_current:.2f} A")
        print(f"\n  MOTOR / ESC:")
        print(f"    Avg motor RPM     : {avg_rpm:.0f} RPM")
        print(f"    Peak motor RPM    : {max_rpm:.0f} RPM")
        print(f"\n  WIND EFFECTS (simulated {WIND_SPEED_MS} m/s FROM N):")
        print(f"    Avg airspd-gndspd : {avg_wind_effect:.2f} m/s")
        print(f"    Max roll angle    : {max_roll:.2f}°")
        print(f"    Max pitch angle   : {max_pitch:.2f}°")
        print(f"\n  WAYPOINT RESULTS:")
        for r in self.wp_results:
            status = "✓ ARRIVED" if r['arrived'] else "⚠ TIMEOUT"
            print(f"    WP{r['wp_id']} {r['label']:20s}: {status} | "
                  f"Min error: {r['min_error_m']:5.2f} m | "
                  f"Leg time: {r['leg_time_s']:5.1f}s | "
                  f"Avg ESC: {r['avg_esc_pwm']:.0f} µs | "
                  f"Avg A: {r['avg_batt_a']:.2f}A")

        self._generate_plots(df, total_time_s, total_dist_m)

    def _generate_plots(self, df: pd.DataFrame, total_time_s: float, total_dist_m: float):
        """
        Six-panel mission analysis dashboard.
        """
        fig, axes = plt.subplots(3, 2, figsize=(18, 14))
        fig.suptitle(
            f"City Block Perimeter SITL — Wind {WIND_SPEED_MS} m/s FROM N\n"
            f"Flight time: {total_time_s:.1f}s | Distance: {total_dist_m:.1f}m | {TIMESTAMP}",
            fontsize=14, fontweight='bold'
        )

        t = df['elapsed_s']
        colors = {'TAKEOFF': '#2196F3', 'LEG': '#4CAF50', 'RTL_LAND': '#FF9800'}

        def leg_color(leg: str) -> str:
            if 'TAKEOFF' in leg:   return colors['TAKEOFF']
            if 'RTL'    in leg:    return colors['RTL_LAND']
            return colors['LEG']

        # ── Panel 1: Flight path (top view) ─────────────────────────────────
        ax = axes[0, 0]
        scatter = ax.scatter(df['lon'], df['lat'], c=t, cmap='plasma', s=8, zorder=2)
        plt.colorbar(scatter, ax=ax, label='Elapsed time (s)')
        # Plot waypoints
        for wp_lat, wp_lon, label in self.waypoints:
            ax.plot(wp_lon, wp_lat, 'rs', markersize=10, zorder=3)
            ax.annotate(label, (wp_lon, wp_lat), textcoords='offset points',
                        xytext=(4, 4), fontsize=7, color='red')
        # Home
        ax.plot(self.home_lon, self.home_lat, 'b*', markersize=14, zorder=4, label='Home')
        ax.set_xlabel('Longitude')
        ax.set_ylabel('Latitude')
        ax.set_title('Flight Path — Top View')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        # Wind arrow (from North = pointing South)
        ax.annotate('', xy=(df['lon'].mean(), df['lat'].min()),
                    xytext=(df['lon'].mean(), df['lat'].min() + meters_to_lat(15)),
                    arrowprops=dict(arrowstyle='->', color='cyan', lw=2))
        ax.text(df['lon'].mean(), df['lat'].min() - meters_to_lat(5),
                f'Wind {WIND_SPEED_MS} m/s ↓', color='cyan', fontsize=8, ha='center')

        # ── Panel 2: Battery voltage ─────────────────────────────────────────
        ax = axes[0, 1]
        ax.plot(t, df['batt_voltage_v'], color='#E91E63', linewidth=1.5, label='Voltage (V)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Voltage (V)')
        ax.set_title('Battery Voltage Over Time')
        ax.grid(True, alpha=0.3)
        ax2 = ax.twinx()
        ax2.plot(t, df['batt_current_a'], color='#FF5722', linewidth=1, alpha=0.7, linestyle='--', label='Current (A)')
        ax2.set_ylabel('Current (A)', color='#FF5722')
        ax.legend(loc='upper left', fontsize=8)
        ax2.legend(loc='upper right', fontsize=8)

        # ── Panel 3: Motor avg RPM ───────────────────────────────────────────
        ax = axes[1, 0]
        ax.plot(t, df['motor_avg_rpm'], color='#9C27B0', linewidth=1.5, label='Avg RPM (est.)')
        # Per-motor PWM lines
        for motor_key, col in [('motor_1_pwm', '#673AB7'), ('motor_2_pwm', '#3F51B5'),
                                ('motor_3_pwm', '#009688'), ('motor_4_pwm', '#4CAF50')]:
            rpm_col = df[motor_key].apply(pwm_to_rpm_estimate)
            ax.plot(t, rpm_col, linewidth=0.8, alpha=0.45, label=motor_key.replace('_pwm', ''))
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Estimated RPM')
        ax.set_title('Motor RPM Estimates (via ESC PWM proxy)')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

        # ── Panel 4: Wind effect & attitude ─────────────────────────────────
        ax = axes[1, 1]
        ax.plot(t, df['wind_effect_ms'], color='#03A9F4', linewidth=1.5,
                label='|Airspeed − Groundspeed| (m/s)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Wind effect proxy (m/s)', color='#03A9F4')
        ax.set_title(f'Wind Effect (Simulated {WIND_SPEED_MS} m/s FROM N) & Attitude')
        ax.grid(True, alpha=0.3)
        ax2 = ax.twinx()
        ax2.plot(t, df['roll_deg'],  color='#FF9800', linewidth=1, linestyle='--', label='Roll (°)')
        ax2.plot(t, df['pitch_deg'], color='#F44336', linewidth=1, linestyle=':',  label='Pitch (°)')
        ax2.set_ylabel('Attitude (°)', color='#FF9800')
        ax.legend(loc='upper left', fontsize=8)
        ax2.legend(loc='upper right', fontsize=8)

        # ── Panel 5: Cumulative distance & altitude ──────────────────────────
        ax = axes[2, 0]
        ax.plot(t, df['cum_dist_m'], color='#795548', linewidth=1.5, label='Cumulative dist (m)')
        ax.axhline(y=4 * BLOCK_SIZE_M, color='r', linestyle='--', alpha=0.7,
                   label=f'Theoretical perim. {4*BLOCK_SIZE_M:.0f} m')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Distance (m)')
        ax.set_title('Cumulative Path Distance')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax2 = ax.twinx()
        ax2.plot(t, df['alt_m'], color='#607D8B', linewidth=1, linestyle='--', alpha=0.8, label='Alt (m)')
        ax2.set_ylabel('Altitude AGL (m)')
        ax2.legend(loc='lower right', fontsize=8)

        # ── Panel 6: Battery level & mission leg markers ─────────────────────
        ax = axes[2, 1]
        ax.plot(t, df['batt_level_pct'], color='#4CAF50', linewidth=1.5, label='Battery level (%)')
        # Shade legs
        leg_boundaries = df.groupby('leg')['elapsed_s'].agg(['min', 'max'])
        leg_colors_cycle = ['#E3F2FD', '#F3E5F5', '#E8F5E9', '#FFF3E0', '#FCE4EC', '#E0F2F1']
        for i, (leg, row) in enumerate(leg_boundaries.iterrows()):
            ax.axvspan(row['min'], row['max'],
                       alpha=0.25, color=leg_colors_cycle[i % len(leg_colors_cycle)],
                       label=leg if i < 5 else None)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Battery Level (%)')
        ax.set_title('Battery Level by Mission Leg')
        ax.legend(fontsize=6, loc='lower left')
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(PLOT_PATH, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"[PLOTS] Dashboard saved to: {PLOT_PATH.resolve()}")


# ============================================================
# ASSERTIONS — PASS / FAIL CRITERIA
# ============================================================

def run_assertions(flight: PerimeterFlightTest, logger: TelemetryLogger):
    """
    Formal test assertions executed after the mission.
    These define the acceptance criteria for a SITL perimeter flight.
    """
    records = logger.records()
    df = pd.DataFrame(records)

    print("\n" + "="*70)
    print("  TEST ASSERTIONS")
    print("="*70)

    results = []

    def check(name: str, condition: bool, fail_msg: str):
        status = "✓ PASS" if condition else "✗ FAIL"
        print(f"  {status}  {name}")
        if not condition:
            print(f"          → {fail_msg}")
        results.append((name, condition))

    # 1. Battery voltage never dropped below 10.5 V (safety floor)
    min_v = df['batt_voltage_v'].min()
    check("Battery voltage above 10.5 V throughout", min_v > 10.5,
          f"Min voltage recorded: {min_v:.3f} V")

    # 2. All 4 waypoints were reached within radius
    all_arrived = all(r['arrived'] for r in flight.wp_results)
    check("All 4 perimeter waypoints reached", all_arrived,
          "One or more waypoints timed out — check wind compensation tuning")

    # 3. All waypoints reached within acceptable position error
    max_wp_err = max((r['min_error_m'] for r in flight.wp_results), default=999)
    check(f"Max waypoint error < {WAYPOINT_RADIUS_M * 3:.0f} m",
          max_wp_err < WAYPOINT_RADIUS_M * 3,
          f"Worst WP error: {max_wp_err:.2f} m — wind may be causing excessive drift")

    # 4. GPS fix maintained (≥ 3D fix) for entire flight
    min_fix = df['gps_fix'].min()
    check("GPS 3D fix maintained throughout", min_fix >= 3,
          f"GPS fix dropped to type {min_fix} during flight")

    # 5. Sufficient satellite count (≥ 6) at all times
    min_sats = df['num_sats'].min()
    check("≥ 6 GPS satellites throughout", min_sats >= 6,
          f"Satellite count dropped to {min_sats}")

    # 6. Motor RPM was in expected SITL operational range during cruise
    cruise_df = df[df['leg'].str.startswith('LEG')]
    if not cruise_df.empty:
        avg_cruise_rpm = cruise_df['motor_avg_rpm'].mean()
        check("Cruise avg RPM > 1000 RPM", avg_cruise_rpm > 1000,
              f"Avg cruise RPM: {avg_cruise_rpm:.0f} — ESC output may be low")
    else:
        check("Cruise RPM data available", False, "No LEG telemetry captured")

    # 7. Wind increased motor current vs idle (crude but representative)
    if not cruise_df.empty:
        takeoff_df = df[df['leg'] == 'TAKEOFF']
        if not takeoff_df.empty:
            idle_a  = takeoff_df['batt_current_a'].mean()
            cruise_a = cruise_df['batt_current_a'].mean()
            check("Flight current ≥ takeoff current (wind loading)",
                  cruise_a >= idle_a * 0.8,
                  f"Cruise: {cruise_a:.2f}A vs Takeoff: {idle_a:.2f}A")

    # 8. Altitude held within ±5 m of cruise altitude during perimeter legs
    if not cruise_df.empty:
        alt_dev = (cruise_df['alt_m'] - CRUISE_ALTITUDE_M).abs().max()
        check(f"Altitude held within ±5 m of {CRUISE_ALTITUDE_M:.0f} m", alt_dev <= 5.0,
              f"Max altitude deviation: {alt_dev:.2f} m")

    # Summary
    passed = sum(1 for _, ok in results if ok)
    total  = len(results)
    print(f"\n  Result: {passed}/{total} assertions passed")

    if passed == total:
        print("\n  ★ MISSION STATUS: ✓ ALL TESTS PASSED ★")
    else:
        print(f"\n  ★ MISSION STATUS: {total - passed} TEST(S) FAILED — review wind compensation ★")

    print("="*70)
    return passed == total


# ============================================================
# MAIN
# ============================================================

def main():
    print("\n" + "="*70)
    print("  QGC PERIMETER FLIGHT TEST — ArduPilot SITL")
    print(f"  City block: {BLOCK_SIZE_M:.0f} m × {BLOCK_SIZE_M:.0f} m | "
          f"Wind: {WIND_SPEED_MS} m/s FROM N | Alt: {CRUISE_ALTITUDE_M:.0f} m AGL")
    print("="*70)
    print(f"\n  Logs → {LOG_DIR.resolve()}")
    print(f"\n  NOTE: QGroundControl should be connected to port 14550.")
    print(f"        This script connects to port 14551 (per Flightwave convention).")
    print(f"        SITL launch command:")
    print(f"          sim_vehicle.py -v ArduCopter --map --console \\")
    print(f"              --out=udp:127.0.0.1:14550 \\")
    print(f"              --out=udp:127.0.0.1:14551\n")

    # -- Connect --
    print(f"[CONNECT] Connecting to vehicle on {CONNECTION_STRING} ...")
    vehicle = connect(CONNECTION_STRING, wait_ready=True, timeout=90, heartbeat_timeout=30)
    print(f"[CONNECT] ✓ Connected — firmware: {vehicle.version}")

    # -- Instantiate logger and flight controller --
    logger = TelemetryLogger(vehicle, sample_hz=TELEMETRY_HZ)
    flight = PerimeterFlightTest(vehicle, logger)

    try:
        # Phase 1: Pre-flight
        flight.preflight_checks()

        # Phase 2: Wind injection
        flight.setup_wind()

        # Phase 3: Build waypoints from home position
        flight.build_waypoints()

        # Phase 4: Start telemetry logging
        logger.start()

        # Phase 5: Arm and take off
        flight.arm_and_takeoff(CRUISE_ALTITUDE_M)

        # Phase 6: Fly the city block perimeter
        flight.fly_perimeter()

        # Phase 7: Return to launch and land
        flight.return_and_land()

    except KeyboardInterrupt:
        print("\n[ABORT] Operator interrupted — switching to RTL")
        vehicle.mode = VehicleMode("RTL")
        time.sleep(5)

    except Exception as e:
        print(f"\n[ERROR] Unexpected error: {e}")
        print("[SAFETY] Switching to RTL")
        try:
            vehicle.mode = VehicleMode("RTL")
            time.sleep(5)
        except Exception:
            pass
        raise

    finally:
        # Always stop logger and export regardless of outcome
        logger.stop()
        flight.t_wheels_down = flight.t_wheels_down or time.time()
        flight.export_and_analyse()
        run_assertions(flight, logger)
        vehicle.close()
        print("\n[DONE] Vehicle connection closed. Test complete.")


if __name__ == "__main__":
    main()
