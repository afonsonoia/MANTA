import os
import openpyxl
import pytest

# ESP32 Polynomial Voltage Calculation Formula (Matching MANTA_ESP32 firmware)
def calculate_battery_voltage(raw_adc: float) -> float:
    """Calculates battery voltage matching the ESP32 polynomial equation:
       voltage = -0.000000884 * (raw_adc^2) + 0.008835 * raw_adc - 5.6904
    """
    voltage = -0.000000884 * (raw_adc ** 2) + 0.008835 * raw_adc - 5.6904
    return max(0.0, voltage)

MAX_ALLOWED_ERROR_V = 0.10  # Strict CI/CD Error threshold: 0.10V max allowed error

def get_ground_truth_filepath():
    possible_paths = [
        os.path.join(os.path.dirname(__file__), "..", "Code", "Battery_Monitor", "voltage_sensor_analysis.xlsx"),
        os.path.join(os.path.dirname(__file__), "..", "voltage_sensor_analysis.xlsx"),
        os.path.join(os.getcwd(), "Code", "Battery_Monitor", "voltage_sensor_analysis.xlsx"),
    ]
    for path in possible_paths:
        abs_path = os.path.abspath(path)
        if os.path.exists(abs_path):
            return abs_path
    raise FileNotFoundError("Could not find voltage_sensor_analysis.xlsx dataset!")

def load_ground_truth_data():
    excel_path = get_ground_truth_filepath()
    wb = openpyxl.load_workbook(excel_path)
    ws = wb.active

    data = []
    for row_idx, row in enumerate(ws.iter_rows(min_row=2, values_only=True), start=2):
        if row and row[0] is not None and row[1] is not None:
            try:
                raw_adc = float(row[0])
                multimeter_v = float(row[1])
                data.append((row_idx, raw_adc, multimeter_v))
            except (ValueError, TypeError):
                continue
    return data

@pytest.mark.parametrize("row_idx, raw_adc, multimeter_v", load_ground_truth_data())
def test_voltage_accuracy_against_ground_truth(row_idx, raw_adc, multimeter_v):
    calculated_v = calculate_battery_voltage(raw_adc)
    error_v = abs(calculated_v - multimeter_v)

    assert error_v <= MAX_ALLOWED_ERROR_V, (
        f"[Row #{row_idx}] FAILED Ground Truth Accuracy Test!\n"
        f"  ADC Input           : {raw_adc}\n"
        f"  Multimeter (Truth)  : {multimeter_v:.2f} V\n"
        f"  ESP32 Formula Calc  : {calculated_v:.2f} V\n"
        f"  Absolute Error      : {error_v:.3f} V (Exceeds maximum allowed threshold of {MAX_ALLOWED_ERROR_V:.2f} V)"
    )

def test_overall_max_error():
    data = load_ground_truth_data()
    assert len(data) > 0, "Ground truth dataset is empty!"

    print("\n" + "="*72)
    print(f" {'Row':<5} | {'Raw ADC':<8} | {'Multimeter (V)':<14} | {'Formula Calc (V)':<16} | {'Error (V)':<9} ")
    print("="*72)

    errors = []
    for row_idx, raw_adc, multimeter_v in data:
        calc_v = calculate_battery_voltage(raw_adc)
        err = abs(calc_v - multimeter_v)
        errors.append(err)
        print(f" #{row_idx:<4} | {raw_adc:<8.0f} | {multimeter_v:<14.2f} | {calc_v:<16.2f} | {err:<9.3f} ")

    print("="*72)
    max_err = max(errors)
    print(f"[CI/CD Report] Evaluated {len(data)} measurement points.")
    print(f"[CI/CD Report] Maximum Voltage Error: {max_err:.3f} V (Strict Limit: {MAX_ALLOWED_ERROR_V:.2f} V)")
    print("="*72 + "\n")

    assert max_err <= MAX_ALLOWED_ERROR_V
