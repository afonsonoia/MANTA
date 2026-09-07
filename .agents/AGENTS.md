# MANTA Aircraft Configuration & Pin Mapping

## Airframe Type
- **Type**: V-Tail

## RC Receiver Channel Inputs
- **CH1**: Roll / Elevons ("Rollerons") — Pin VN / GPIO 39
- **CH2**: Elevator / Pitch — Pin D34 / GPIO 34
- **CH3**: Throttle / ESC — Pin D35 / GPIO 35
- **CH4**: Pin D32 / GPIO 32
- **CH5**: Pin D33 / GPIO 33

## Servo Outputs & Control Pins
- **BR** (Back Right / Traseira Direita): Pin D13 / GPIO 13
- **BL** (Back Left / Traseira Esquerda): Pin D14 / GPIO 14
- **FR** (Front Right / Frontal Direita): Pin D27 / GPIO 27
- **FL** (Front Left / Frontal Esquerda): Pin D26 / GPIO 26
- **Throttle / ESC**: Pin D25 / GPIO 25

## LoRa & Sensors Bus
- **LoRa (VSPI)**: SCK=GPIO18, MISO=GPIO19, MOSI=GPIO23, CS=GPIO5, DIO0=GPIO4
- **I2C (MPU6050 + BMP280)**: SDA=GPIO21, SCL=GPIO22

## Flight Modes & Transmitter CH5 Calibration (SWC + SWB)
> **NOTA DE SEGURANÇA CRÍTICA (Anti-Stall)**: O controlo e calibração de **Pitch** (PI-D e Extremum Seeking) foram **completamente desativados** nos Modos 2 e 3 para prevenir que o profundor seja puxado agressivamente para cima induzindo stall. O eixo de **Pitch é 100% manual direto pelo stick CH2 do piloto em todos os modos**, com telemetria reportando ganhos de pitch a 0.00. O controlo FBW e adaptativo atua exclusivamente sobre o eixo de **Roll** quando ativado pelo SWB (Roll ON).

| SWC | SWB | CH5 PWM Medido | Modo Ativo | Descrição |
|:---:|:---:|:--------------:|:----------:|:----------|
| 1 | OFF | **1166 µs** (< 1247) | **Modo 1** | 100% Manual Direto (Pitch Manual + Roll Manual) |
| 2 | OFF | **1328 µs** (1247–1370) | **Modo 2** | Pitch Manual Direto + Roll Manual (Pitch PID Desativado) |
| 3 | OFF | **1411 µs** (1370–1476) | **Modo 3** | Pitch Manual Direto + Roll Manual (Pitch PID & Calibração Desativados) |
| 1 | ON  | **1541 µs** (1476–1683) | **Modo 1** | Pitch Manual Direto + Roll Envelope Assist (Roll ON) |
| 2 | ON  | **1825 µs** (1683–1884) | **Modo 2** | Pitch Manual Direto + FBW Roll Nominal Fixo (Roll ON) |
| 3 | ON  | **1942 µs** (>= 1884) | **Modo 3** | Pitch Manual Direto + FBW Roll Adaptativo Extremum Seeking (Roll ON) |

## Telemetry Architecture (Pure Simplex Downlink)
- **Aeronave (MANTA ESP32)**: Transmissor Beacon **Simplex TX** dedicado (433 MHz, SF7, BW 250kHz, CR 4/5, Sync 0x12).
  - Emite pacotes de telemetria binária downlink de 61 bytes (`sendTelemetry()` @ 20 Hz):
    - Cabeçalho 'MT', sequência, timestamp_ms, pitch_x10, roll_x10, 6 eixos IMU, 4 canais RC, 5 atuadores (4 servos + ESC), bateria_x100, altitude_x10, flags de estado, **modo ativo de voo** (`flight_mode`: 1, 2 ou 3) e os **6 ganhos PID adaptados em tempo real** (`pitch_kp_x100`, `pitch_ki_x100`, `pitch_kd_x1000`, `roll_kp_x100`, `roll_ki_x100`, `roll_kd_x1000`) protegidos por CRC16.
  - **Não** possui receptor LoRa ativo, não processa comandos de uplink nem responde com ACKs pelo ar.
  - Todos os parâmetros estáticos de voo (Deadband RC, filtros SMA/EMA/WMA, inversões e limites de servos, ganhos nominais) são configurados estaticamente no firmware (`Code/MANTA_ESP32/include/config.h`).
- **Ground Station (ESP32 RX)**: Receptor **Simplex RX** dedicado.
  - Recebe pacotes de telemetria LoRa e reencaminha pela porta Serial USB (`115200 baud`) para a interface PC (`gui.py`, `lora_logger.py`, `MANTA_MISSION_PLANNER.py`, `telemetry_csv_logger.py`).
  - Grava automaticamente nos flight logs (`flight_logs/manta_flight_XXXX.csv` e `flight_logs/manta_pid_flight_XXXX.csv`) o modo ativo de voo, status de ESC ativo e os 6 ganhos PID dinâmicos em todas as linhas para análise pós-voo e SysID.
  - Possui Buzzer local (GPIO 22) controlado por comandos via Serial USB (`BEEP:CONTINUOUS`, `BEEP:INTERMITTENT`, `BEEP:OFF`, `BEEP:SHORT`, `BEEP:0.7S`, `BEEP:1S`) e emite autonomamente um apito sonoro de feedback de 0.7 segundo (`MODE_CHANGE_BEEP_MS = 700`) cada vez que uma transição de modo de voo (SWC / SWB) é detetada na telemetria sem bloquear a receção LoRa.
  - Parâmetros da Ground Station (margem deadband de alerta, corte de tensão `cutoff`, offsets de horizonte) são guardados localmente em `imu_calibration.json`.

