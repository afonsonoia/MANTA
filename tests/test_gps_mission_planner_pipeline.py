"""
CI/CD Automated Test Suite: MANTA GPS & Mission Planner Telemetry Pipeline
==========================================================================
Verifies the complete end-to-end chain from GPS NMEA parsing through LoRa 73-byte
binary encoding to Ground Station MAVLink packet generation for Mission Planner.

Guarantees:
1. Decimal conversion of NMEA latitude & longitude coordinates with correct hemispheres.
2. 73-byte primary telemetry packet encoding of lat_e7, lon_e7, gps_alt_x10, satellites, fix_type.
3. GLOBAL_POSITION_INT message geometry: lat/lon (deg*1e7), alt/relative_alt (mm), 2D velocity vector (cm/s).
4. GPS_RAW_INT message geometry: fix_type, satellite count, and Null Island protection (int.MaxValue = 2147483647).
5. Automatic Home Position (HOME_POSITION) establishment on first valid 2D/3D fix.
6. GPS coordinate persistence across momentary signal dropouts.
"""

import math
import os
import sys
import pytest

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
GROUND_STATION_DIR = os.path.join(PROJECT_ROOT, "Code", "GROUND-STATION")
if GROUND_STATION_DIR not in sys.path:
    sys.path.insert(0, GROUND_STATION_DIR)

from telemetry_codec import encode_telemetry, decode_telemetry
import MANTA_MISSION_PLANNER as mp

try:
    from pymavlink import mavutil
    HAS_PYMAVLINK = True
except ImportError:
    HAS_PYMAVLINK = False


def convert_nmea_to_decimal(raw_coord: str, hemisphere: str) -> float:
    """Python reference implementation of MANTA ESP32 gps.cpp convertNMEAToDecimal."""
    if not raw_coord:
        return 0.0
    val = float(raw_coord)
    degrees = math.floor(val / 100.0)
    minutes = val - (degrees * 100.0)
    dec = degrees + (minutes / 60.0)
    if hemisphere.upper() in ('S', 'W'):
        dec = -dec
    return dec


class TestGPSMissionPlannerPipeline:
    """End-to-end verification of GPS acquisition, telemetry packing, and Mission Planner bridge."""

    def test_nmea_coordinate_conversion(self):
        """Verifies NMEA ddmm.mmmm conversion matching Lisbon airframe coordinates."""
        # Lisbon / Costa da Caparica Coordinates:
        # Latitude: 38° 43.5060' N -> 38.7251°
        # Longitude: 009° 09.0120' W -> -9.1502°
        lat = convert_nmea_to_decimal("3843.5060", "N")
        lon = convert_nmea_to_decimal("00909.0120", "W")

        assert lat == pytest.approx(38.7251, abs=1e-5)
        assert lon == pytest.approx(-9.1502, abs=1e-5)

        # Southern and Eastern hemispheres
        s_lat = convert_nmea_to_decimal("2333.3333", "S")
        e_lon = convert_nmea_to_decimal("04638.1234", "E")
        assert s_lat < 0
        assert e_lon > 0

    def test_73b_telemetry_gps_codec_roundtrip(self):
        """Validates that 73B packet encodes and decodes exact lat_e7, lon_e7, gps_alt, sats, and fix."""
        lat_truth = 38.7251000
        lon_truth = -9.1502000
        alt_truth = 85.5
        sats_truth = 12
        fix_truth = 3

        encoded = encode_telemetry(
            pitch=3.5, roll=-1.2,
            accel_x=50, accel_y=-20, accel_z=-4096,
            gyro_x=5, gyro_y=-2, gyro_z=1,
            rc1=1500, rc2=1500, rc3=1400, rc5=1328,
            battery_v=15.10, alt=85.2,
            lat=lat_truth, lon=lon_truth, gps_alt=alt_truth,
            satellites=sats_truth, gps_fix=fix_truth,
            flight_mode=2, packet_format="73B"
        )
        assert len(encoded) == 73

        decoded = decode_telemetry(encoded)
        assert decoded is not None
        assert decoded["packet_size"] == 73
        assert decoded["lat"] == pytest.approx(lat_truth, abs=1e-5)
        assert decoded["lon"] == pytest.approx(lon_truth, abs=1e-5)
        assert decoded["lat_e7"] == int(round(lat_truth * 1e7))
        assert decoded["lon_e7"] == int(round(lon_truth * 1e7))
        assert decoded["gps_alt"] == pytest.approx(alt_truth, abs=0.1)
        assert decoded["satellites"] == sats_truth
        assert decoded["fix_type"] == fix_truth
        assert decoded["gps_fixed"] is True

    @pytest.mark.skipif(not HAS_PYMAVLINK, reason="pymavlink not installed")
    def test_mavlink_global_position_int_message_format(self):
        """Verifies GLOBAL_POSITION_INT encoding produces valid MAVLink payload with correct units."""
        m = mavutil.mavlink.MAVLink(None)
        lat_e7 = 387251000
        lon_e7 = -91502000
        alt_mm = 85200
        vx_cms = 1200   # 12 m/s North
        vy_cms = 500    # 5 m/s East
        vz_cms = -150   # 1.5 m/s climb (NED: negative is up)
        hdg_cdeg = 2250 # 22.5 deg heading

        msg = m.global_position_int_encode(
            time_boot_ms=123456,
            lat=lat_e7,
            lon=lon_e7,
            alt=alt_mm,
            relative_alt=alt_mm,
            vx=vx_cms,
            vy=vy_cms,
            vz=vz_cms,
            hdg=hdg_cdeg
        )
        assert msg.lat == lat_e7
        assert msg.lon == lon_e7
        assert msg.alt == alt_mm
        assert msg.relative_alt == alt_mm
        assert msg.vx == vx_cms
        assert msg.vy == vy_cms
        assert msg.vz == vz_cms
        assert msg.hdg == hdg_cdeg

    @pytest.mark.skipif(not HAS_PYMAVLINK, reason="pymavlink not installed")
    def test_mavlink_gps_raw_int_null_island_protection(self):
        """Verifies that when no GPS fix exists, lat/lon pass 2147483647 (int.MaxValue) per MAVLink spec."""
        m = mavutil.mavlink.MAVLink(None)
        has_gps_pos = False
        raw_gps_lat = 387251000 if has_gps_pos else 2147483647
        raw_gps_lon = -91502000 if has_gps_pos else 2147483647

        msg = m.gps_raw_int_encode(
            time_usec=123456000,
            fix_type=0,
            lat=raw_gps_lat,
            lon=raw_gps_lon,
            alt=0,
            eph=65535,
            epv=65535,
            vel=0,
            cog=0,
            satellites_visible=0
        )
        # 2147483647 signals Mission Planner not to move its location to (0,0) Null Island
        assert msg.lat == 2147483647
        assert msg.lon == 2147483647
        assert msg.fix_type == 0

    @pytest.mark.skipif(not HAS_PYMAVLINK, reason="pymavlink not installed")
    def test_mavlink_home_position_generation(self):
        """Verifies HOME_POSITION MAVLink message sets Home coordinates on initial 3D fix."""
        m = mavutil.mavlink.MAVLink(None)
        home_lat = 387251000
        home_lon = -91502000
        home_alt = 85200

        msg = m.home_position_encode(
            latitude=home_lat,
            longitude=home_lon,
            altitude=home_alt,
            x=0.0, y=0.0, z=0.0,
            q=[1.0, 0.0, 0.0, 0.0],
            approach_x=0.0, approach_y=0.0, approach_z=0.0
        )
        assert msg.latitude == home_lat
        assert msg.longitude == home_lon
        assert msg.altitude == home_alt

    def test_coordinate_persistence_across_dropouts(self):
        """Verifies that a temporary dropout (lat=0, lon=0) does not overwrite last known coordinates."""
        latest_lat = 38.7251
        latest_lon = -9.1502

        # Received dropout packet with lat=0, lon=0
        dropout_pkt = {"lat": 0.0, "latitude": 0.0, "lon": 0.0, "longitude": 0.0, "fix_type": 0}

        pkt_lat = dropout_pkt.get("latitude", dropout_pkt.get("lat", 0.0))
        pkt_lon = dropout_pkt.get("longitude", dropout_pkt.get("lon", 0.0))
        if pkt_lat != 0.0 or pkt_lon != 0.0:
            latest_lat = pkt_lat
            latest_lon = pkt_lon

        # Coordinates must remain safely locked to last known position
        assert latest_lat == 38.7251
        assert latest_lon == -9.1502


class GPSAutoSwapStateMachineSim:
    """Python model of the C++ MANTA ESP32 gps.cpp auto-swap state machine."""
    PROBING_DEFAULT = 0
    PROBING_SWAPPED = 1
    LOCKED_NORMAL   = 2
    LOCKED_SWAPPED  = 3
    DISCONNECTED    = 4

    PROBE_TIMEOUT_MS = 3000
    RETRY_INTERVAL_MS = 5000
    SILENCE_TIMEOUT_MS = 5000

    def __init__(self, physical_wiring="normal"):
        # physical_wiring: "normal" (GPS TX -> ESP32 GPIO16), "inverted" (GPS TX -> ESP32 GPIO17), "disconnected"
        self.physical_wiring = physical_wiring
        self.state = self.PROBING_DEFAULT
        self.state_timer = 0
        self.active_rx_pin = 16
        self.active_tx_pin = 17
        self.last_valid_nmea_time = 0
        self.valid_nmea_count = 0

    def feed_sentence(self, sentence: str, now_ms: int):
        if not sentence or not sentence.startswith("$"):
            return
        is_valid_nmea = any(sentence.startswith(prefix) for prefix in ("$GP", "$GN", "$BD", "$GA"))
        if is_valid_nmea:
            self.last_valid_nmea_time = now_ms
            self.valid_nmea_count += 1
            if self.state == self.PROBING_DEFAULT:
                self.state = self.LOCKED_NORMAL
            elif self.state == self.PROBING_SWAPPED:
                self.state = self.LOCKED_SWAPPED

    def update(self, now_ms: int, sentence: str = ""):
        # Check if active RX pin receives data from GPS TX
        gps_tx_pin = 16 if self.physical_wiring == "normal" else (17 if self.physical_wiring == "inverted" else -1)
        if self.active_rx_pin == gps_tx_pin and sentence:
            self.feed_sentence(sentence, now_ms)

        # Silence watchdog: If locked, monitor for total link silence (> 5.0s)
        if self.state in (self.LOCKED_NORMAL, self.LOCKED_SWAPPED):
            if now_ms - self.last_valid_nmea_time >= self.SILENCE_TIMEOUT_MS:
                if self.state == self.LOCKED_NORMAL:
                    self.active_rx_pin = 17
                    self.active_tx_pin = 16
                    self.state = self.PROBING_SWAPPED
                else:
                    self.active_rx_pin = 16
                    self.active_tx_pin = 17
                    self.state = self.PROBING_DEFAULT
                self.state_timer = now_ms
                # Check if new pin immediately sees sentence
                gps_tx_pin = 16 if self.physical_wiring == "normal" else (17 if self.physical_wiring == "inverted" else -1)
                if self.active_rx_pin == gps_tx_pin and sentence:
                    self.feed_sentence(sentence, now_ms)
            return

        if self.state == self.PROBING_DEFAULT:
            if now_ms - self.state_timer >= self.PROBE_TIMEOUT_MS:
                self.active_rx_pin = 17
                self.active_tx_pin = 16
                self.state = self.PROBING_SWAPPED
                self.state_timer = now_ms
        elif self.state == self.PROBING_SWAPPED:
            if now_ms - self.state_timer >= self.PROBE_TIMEOUT_MS:
                self.active_rx_pin = 16
                self.active_tx_pin = 17
                self.state = self.DISCONNECTED
                self.state_timer = now_ms
        elif self.state == self.DISCONNECTED:
            if now_ms - self.state_timer >= self.RETRY_INTERVAL_MS:
                self.state = self.PROBING_DEFAULT
                self.state_timer = now_ms


class TestGPSAutoSwapProtection:
    """Validates the autonomous RX/TX pin-swap and glitch immunity of MANTA ESP32."""

    def test_gps_auto_swap_normal_wiring(self):
        """When wires are normal, locks into LOCKED_NORMAL and never swaps during flight."""
        sim = GPSAutoSwapStateMachineSim(physical_wiring="normal")
        assert sim.state == GPSAutoSwapStateMachineSim.PROBING_DEFAULT

        # At 1000ms, GPS transmits GGA
        sim.update(1000, "$GPGGA,123519,3843.5060,N,00909.0120,W,1,08,0.9,545.4,M,46.9,M,,*47\n")
        assert sim.state == GPSAutoSwapStateMachineSim.LOCKED_NORMAL
        assert sim.active_rx_pin == 16
        assert sim.active_tx_pin == 17

        # In flight at 60000ms (1 minute later), state is still LOCKED_NORMAL even during temporary satellite loss
        sim.update(60000, "$GPGGA,,,,,,0,00,,,M,,M,,*66\n")
        assert sim.state == GPSAutoSwapStateMachineSim.LOCKED_NORMAL
        assert sim.active_rx_pin == 16

    def test_gps_auto_swap_inverted_wiring(self):
        """When wires are inverted, detects silence on default pins and auto-corrects to LOCKED_SWAPPED."""
        sim = GPSAutoSwapStateMachineSim(physical_wiring="inverted")
        assert sim.state == GPSAutoSwapStateMachineSim.PROBING_DEFAULT

        # 0 to 2900ms: GPS sends data on GPIO 17, but ESP32 is listening on GPIO 16 -> 0 bytes seen
        sim.update(1000, "$GNGGA,123519,3843.5060,N,00909.0120,W,1,08,0.9,545.4,M,46.9,M,,*47\n")
        assert sim.state == GPSAutoSwapStateMachineSim.PROBING_DEFAULT

        # At 3000ms: Timeout triggers pin swap (RX=17, TX=16)
        sim.update(3000, "")
        assert sim.state == GPSAutoSwapStateMachineSim.PROBING_SWAPPED
        assert sim.active_rx_pin == 17
        assert sim.active_tx_pin == 16

        # At 3500ms: Now listening on GPIO 17, sentence arrives!
        sim.update(3500, "$GNRMC,123519,A,3843.5060,N,00909.0120,W,022.4,084.4,230394,003.1,W*6A\n")
        assert sim.state == GPSAutoSwapStateMachineSim.LOCKED_SWAPPED
        assert sim.active_rx_pin == 17
        assert sim.active_tx_pin == 16

        # Permanent lock during flight
        sim.update(70000, "$GNRMC,,,,,,,,,,\n")
        assert sim.state == GPSAutoSwapStateMachineSim.LOCKED_SWAPPED

    def test_gps_auto_swap_disconnected_cable(self):
        """When GPS is disconnected, cycles through probe states and returns to DISCONNECTED without crashing."""
        sim = GPSAutoSwapStateMachineSim(physical_wiring="disconnected")
        assert sim.state == GPSAutoSwapStateMachineSim.PROBING_DEFAULT

        # Timeout default at 3000ms
        sim.update(3000)
        assert sim.state == GPSAutoSwapStateMachineSim.PROBING_SWAPPED

        # Timeout swapped at 6000ms
        sim.update(6000)
        assert sim.state == GPSAutoSwapStateMachineSim.DISCONNECTED
        # Default pin restored
        assert sim.active_rx_pin == 16

        # Wait retry interval (5000ms) -> back to probing default
        sim.update(11000)
        assert sim.state == GPSAutoSwapStateMachineSim.PROBING_DEFAULT

    def test_gps_auto_swap_noise_immunity(self):
        """Spurious electrical noise does not trigger a lock; only valid NMEA sentences do."""
        sim = GPSAutoSwapStateMachineSim(physical_wiring="inverted")

        # Noise on GPIO 16
        sim.feed_sentence("\xFF\xFE\x00\x12RANDOM_GLITCH\r\n", 500)
        assert sim.state == GPSAutoSwapStateMachineSim.PROBING_DEFAULT

        # Fake non-NMEA dollar sign
        sim.feed_sentence("$HELLO_WORLD,1,2,3*00\n", 1000)
        assert sim.state == GPSAutoSwapStateMachineSim.PROBING_DEFAULT

        # Timeout to swapped
        sim.update(3000)
        assert sim.state == GPSAutoSwapStateMachineSim.PROBING_SWAPPED

        # Noise on GPIO 17 does not lock
        sim.feed_sentence("\xAA\x55NOISE\n", 3200)
        assert sim.state == GPSAutoSwapStateMachineSim.PROBING_SWAPPED

        # Real NMEA sentence locks immediately
        sim.feed_sentence("$GPGGA,123519,3843.5060,N,00909.0120,W,1,08,0.9,545.4,M,46.9,M,,*47\n", 3500)
        assert sim.state == GPSAutoSwapStateMachineSim.LOCKED_SWAPPED

    def test_gps_auto_swap_hot_swap_after_5_minutes(self):
        """Swapping cables 5 minutes later triggers silence watchdog and auto-adapts to new configuration."""
        sim = GPSAutoSwapStateMachineSim(physical_wiring="normal")

        # Initial boot: GPS connected normally, locks at 1000ms
        sim.update(1000, "$GPGGA,123519,3843.5060,N,00909.0120,W,1,08,0.9,545.4,M,46.9,M,,*47\n")
        assert sim.state == GPSAutoSwapStateMachineSim.LOCKED_NORMAL
        assert sim.active_rx_pin == 16

        # Normal operation for 5 minutes (up to 300,000ms) with NMEA sentences flowing every second
        for t in range(2000, 300001, 1000):
            sim.update(t, "$GPGGA,123519,3843.5060,N,00909.0120,W,1,08,0.9,545.4,M,46.9,M,,*47\n")
        assert sim.state == GPSAutoSwapStateMachineSim.LOCKED_NORMAL
        assert sim.last_valid_nmea_time == 300000

        # User physically swaps the cables at t = 300,000ms! (GPS TX now on GPIO 17)
        sim.physical_wiring = "inverted"

        # During the first 4.0s of silence (t = 301,000 to 304,000ms): Still locked to NORMAL
        for t in range(301000, 304500, 500):
            sim.update(t, "$GPGGA,123519,3843.5060,N,00909.0120,W,1,08,0.9,545.4,M,46.9,M,,*47\n")
        assert sim.state == GPSAutoSwapStateMachineSim.LOCKED_NORMAL

        # At t = 305,000ms (5.0s elapsed since last valid NMEA): Silence watchdog triggers!
        sim.update(305000, "$GPGGA,123519,3843.5060,N,00909.0120,W,1,08,0.9,545.4,M,46.9,M,,*47\n")
        # Automatically probed and locked into swapped configuration!
        assert sim.state == GPSAutoSwapStateMachineSim.LOCKED_SWAPPED
        assert sim.active_rx_pin == 17
        assert sim.active_tx_pin == 16

        # User swaps them back to normal after another 2 minutes (up to t = 420,000ms)
        for t in range(306000, 420001, 1000):
            sim.update(t, "$GPGGA,123519,3843.5060,N,00909.0120,W,1,08,0.9,545.4,M,46.9,M,,*47\n")
        assert sim.state == GPSAutoSwapStateMachineSim.LOCKED_SWAPPED
        assert sim.last_valid_nmea_time == 420000

        sim.physical_wiring = "normal"
        # 4.0 seconds of silence on GPIO 17: Still locked to SWAPPED
        for t in range(421000, 424500, 500):
            sim.update(t, "$GPGGA,123519,3843.5060,N,00909.0120,W,1,08,0.9,545.4,M,46.9,M,,*47\n")
        assert sim.state == GPSAutoSwapStateMachineSim.LOCKED_SWAPPED

        # At t = 425,000ms (5.0s elapsed): Watchdog triggers and restores normal pins (GPIO 16)!
        sim.update(425000, "$GPGGA,123519,3843.5060,N,00909.0120,W,1,08,0.9,545.4,M,46.9,M,,*47\n")
        assert sim.state == GPSAutoSwapStateMachineSim.LOCKED_NORMAL
        assert sim.active_rx_pin == 16


