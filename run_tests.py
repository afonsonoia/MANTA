#!/usr/bin/env python3
"""MANTA Avionics CI/CD Test Suite Runner.

Organized test runner for local validation before pushing commits to GitHub.

Usage:
    python run_tests.py                     # Run all test suites
    python run_tests.py --category control  # Run only Flight Control tests
    python run_tests.py --category sensors  # Run only Sensor & Ground Truth tests
    python run_tests.py --list              # List all test categories
"""

import sys
import subprocess
import os
import argparse
import time

CATEGORIES = {
    "compilation": {
        "title": "Firmware & Python Compilation",
        "description": "PlatformIO C++ build verification (MANTA + RX) and Ground Station syntax",
        "files": ["tests/test_compilation.py"]
    },
    "control": {
        "title": "Fly-By-Wire Control, PI-D & Kinematics",
        "description": "FBW PI-D attitude loops, anti-stall, flaperon governor & CH5 thresholds",
        "files": ["tests/test_fbw_pid_control.py", "tests/test_ch5_calibration_midpoints.py"]
    },
    "telemetry": {
        "title": "Simplex Telemetry Codec & Protocols",
        "description": "61-byte packet binary codec, CRC16 LUT, CSV logging & Ground Station buzzer",
        "files": ["tests/test_telemetry_codec.py", "tests/test_ground_station_buzzer.py"]
    },
    "safety": {
        "title": "Flight Safety, Failsafes & SysID Alignment",
        "description": "Soft voltage power floor, cascaded failsafes, SysID & Raspberry Pi boot manager",
        "files": ["tests/test_flight_safety_and_sysid.py", "tests/test_manta_pi_boot.py"]
    },
    "sensors": {
        "title": "Sensor Calibration & Ground Truth Verification",
        "description": "Battery voltage formula vs multimeter ground truth & 100Hz IMU stability",
        "files": ["tests/test_voltage_sensor.py", "tests/test_imu_quality.py"]
    }
}


def print_banner():
    print("=" * 76)
    print("                  MANTA AVIONICS CI/CD TEST SUITE RUNNER            ")
    print("=" * 76)


def list_categories():
    print_banner()
    print("Available Test Categories:\n")
    for key, info in CATEGORIES.items():
        print(f"  * {key:<12} : {info['title']}")
        print(f"                   {info['description']}")
        print(f"                   Files: {', '.join(info['files'])}\n")
    print("=" * 76)


def run_category(category_key: str, info: dict, verbose: bool = True) -> int:
    files = info["files"]
    print(f"\n>> Category: {info['title']}")
    print(f"   Target:   {', '.join(files)}")
    print("-" * 76)

    cmd = [sys.executable, "-m", "pytest"] + files + (["-v", "-s"] if verbose else ["-q"])
    start = time.time()
    result = subprocess.run(cmd)
    elapsed = time.time() - start

    if result.returncode == 0:
        print(f"   Status:   [ PASS ] (took {elapsed:.2f}s)")
    else:
        print(f"   Status:   [ FAIL ] (exit code {result.returncode}, took {elapsed:.2f}s)")

    return result.returncode


def main():
    parser = argparse.ArgumentParser(description="MANTA Avionics CI/CD Test Suite Runner")
    parser.add_argument(
        "-c", "--category",
        choices=list(CATEGORIES.keys()) + ["all"],
        default="all",
        help="Run a specific test category (default: all)"
    )
    parser.add_argument(
        "-l", "--list",
        action="store_true",
        help="List available test categories and descriptions"
    )
    parser.add_argument(
        "-q", "--quiet",
        action="store_true",
        help="Run tests with concise output"
    )
    args = parser.parse_args()

    project_root = os.path.dirname(os.path.abspath(__file__))
    os.chdir(project_root)

    if args.list:
        list_categories()
        return 0

    print_banner()

    if args.category != "all":
        info = CATEGORIES[args.category]
        ret = run_category(args.category, info, verbose=not args.quiet)
        return ret

    # Run all categories
    cmd = [sys.executable, "-m", "pytest", "tests", "-v", "-s"]
    start_total = time.time()
    result = subprocess.run(cmd)
    total_elapsed = time.time() - start_total

    print("\n" + "=" * 76)
    if result.returncode == 0:
        print(f"  SUCCESS: All MANTA CI/CD tests PASSED cleanly! ({total_elapsed:.2f}s)")
        print("  Categories Verified:")
        for key, info in CATEGORIES.items():
            print(f"    [PASS] {info['title']}")
    else:
        print(f"  FAILED: One or more test suites FAILED! (exit code {result.returncode})")
    print("=" * 76 + "\n")

    return result.returncode


if __name__ == "__main__":
    sys.exit(main())
