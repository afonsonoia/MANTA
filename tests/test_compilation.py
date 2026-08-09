import os
import sys
import glob
import py_compile
import subprocess
import pytest

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
GROUND_STATION_DIR = os.path.join(PROJECT_ROOT, "Code", "GROUND-STATION")
MANTA_ESP32_DIR = os.path.join(PROJECT_ROOT, "Code", "MANTA_ESP32")


def get_ground_station_py_files():
    py_files = glob.glob(os.path.join(GROUND_STATION_DIR, "*.py"))
    return sorted(py_files)


@pytest.mark.parametrize("py_file", get_ground_station_py_files())
def test_ground_station_python_syntax(py_file):
    """Verifies that each Ground Station Python file compiles cleanly without syntax errors."""
    rel_path = os.path.relpath(py_file, PROJECT_ROOT)
    try:
        py_compile.compile(py_file, doraise=True)
    except py_compile.PyCompileError as exc:
        pytest.fail(f"Syntax error compiling {rel_path}:\n{exc}")


def test_manta_esp32_firmware_compilation():
    """Verifies that MANTA ESP32 firmware compiles with PlatformIO without errors."""
    assert os.path.exists(MANTA_ESP32_DIR), f"MANTA_ESP32 directory not found at {MANTA_ESP32_DIR}"
    cmd = [sys.executable, "-m", "platformio", "run"]
    result = subprocess.run(cmd, cwd=MANTA_ESP32_DIR, capture_output=True, text=True)
    assert result.returncode == 0, (
        f"MANTA ESP32 firmware compilation failed with exit code {result.returncode}:\n"
        f"STDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
    )


def test_ground_station_esp32_firmware_compilation():
    """Verifies that Ground Station ESP32 firmware compiles with PlatformIO without errors."""
    assert os.path.exists(GROUND_STATION_DIR), f"GROUND-STATION directory not found at {GROUND_STATION_DIR}"
    cmd = [sys.executable, "-m", "platformio", "run"]
    result = subprocess.run(cmd, cwd=GROUND_STATION_DIR, capture_output=True, text=True)
    assert result.returncode == 0, (
        f"Ground Station ESP32 firmware compilation failed with exit code {result.returncode}:\n"
        f"STDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
    )
