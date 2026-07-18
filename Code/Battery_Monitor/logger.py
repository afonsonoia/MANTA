import serial
import time
import openpyxl
from openpyxl import Workbook
import os

# Config
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
EXCEL_FILE = 'registo_bateria.xlsx'

def main():
    print(f"Starting battery monitor on {SERIAL_PORT}...")
    
    if os.path.exists(EXCEL_FILE):
        wb = openpyxl.load_workbook(EXCEL_FILE)
        ws = wb.active
        record_number = ws.max_row
    else:
        wb = Workbook()
        ws = wb.active
        ws.title = "Battery Log"
        ws.append(["Record Number", "Voltage (V)"])
        record_number = 1
        
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print("Connected! Waiting for data...")
        
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    try:
                        voltage = float(line)
                        ws.append([record_number, voltage])
                        wb.save(EXCEL_FILE)
                        print(f"Record {record_number}: {voltage}V saved.")
                        record_number += 1
                    except ValueError:
                        print(f"Invalid data received: {line}")
                        
    except serial.SerialException as e:
        print(f"Serial port error: {e}")
    except KeyboardInterrupt:
        print("\nLogging stopped by user.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")
        if 'wb' in locals():
            wb.save(EXCEL_FILE)

if __name__ == "__main__":
    main()
