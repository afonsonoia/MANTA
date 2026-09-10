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

### Calibração Estática de Superfícies (Trim Neutros) & Offsets IMU
- **Deflexão Máxima de Superfícies (Limite Angular)**: `DEFAULT_SERVO_MAX_ANGLE_DEG = 25.0°` (`anglePulseLimit = ±278 µs`). Todos os 4 servos operam estritamente dentro da faixa segura de hardware [1000, 2000] µs em torno dos seus neutros trimados.
- **BR (Back Right)**: `TRIM_DEG_BR = -2.0°` (`TRIM_US_BR = -22 µs`) -> **Neutro: 1478 µs** (Span: 1200 a 1756 µs; +2.0° UP / Cabrar para compensar descida de nariz em manual)
- **BL (Back Left)**: `TRIM_DEG_BL = -12.0°` (`TRIM_US_BL = +133 µs`) -> **Neutro: 1633 µs** (Span: 1355 a 1911 µs, margem de 89 µs do teto; +2.0° UP / Cabrar invertido)
- **FR (Front Right)**: `TRIM_DEG_FR = 0.0°` (`TRIM_US_FR = 0 µs`) -> **Neutro: 1500 µs** (Span: 1222 a 1778 µs)
- **FL (Front Left)**: `TRIM_DEG_FL = +4.0°` (`TRIM_US_FL = +44 µs`) -> **Neutro: 1544 µs** (Span: 1266 a 1822 µs; trim mecânico de encaixe da estria do servo na buzina da superfície de controlo — define o neutro físico plano da asa)
- **IMU Pitch Offset**: `IMU_PITCH_MOUNTING_OFFSET_DEG = 9.2°` (alinhamento de nivelamento longitudinal)
- **IMU Roll Offset**: `IMU_ROLL_MOUNTING_OFFSET_DEG = 4.4°` (1.9° original + 2.5° para compensar tendência de rolamento para a esquerda em voo reto)

## LoRa & Sensors Bus
- **LoRa (VSPI)**: SCK=GPIO18, MISO=GPIO19, MOSI=GPIO23, CS=GPIO5, DIO0=GPIO4
- **I2C (MPU6050 + BMP280)**: SDA=GPIO21, SCL=GPIO22

## Flight Modes & Transmitter CH5 Calibration (SWC + SWB)
> **NOTA DE CONTROLO DE VOO (Roll Permanente & Flaperons 20° DOWN)**: O controlo de **Roll está permanentemente ativo** em todos os modos normais de voo (Roll Assist no Modo 1, FBW PI-D no Modo 2, FBW Adaptativo no Modo 3). O switch binário **SWB** opera **Flaperons (até +20.0° para BAIXO)** nas superfícies FR e FL, calculados a partir dos neutros mecânicos trimados (1500 µs e 1544 µs), aumentando sustentação ($C_L$) e arrasto ($C_D$) para facilitar aproximações e aterragens estáveis.
>
> **GOVERNADOR DE ACELERAÇÃO DOS FLAPERONS & ANTI-SATURAÇÃO**: Os flaperons variam uniformemente entre **1500 µs e 1200 µs** de Throttle. Acima de $1500\,\mu\text{s}$ estão totalmente recolhidos (0° offset). Abaixo de $1200\,\mu\text{s}$ atingem a deflexão máxima (+20.0° DOWN = $\pm 222\,\mu\text{s}$). Entre $1200\,\mu\text{s}$ e $1500\,\mu\text{s}$, opera uma **rampa linear contínua suave** ($\Delta \text{offset} = 20.0^\circ \times \frac{1500 - \text{Throttle}}{300}$).
>
> **PRIORIDADE DE ROLL E PROTEÇÃO ANTI-SATURAÇÃO (HEADROOM SCALING)**: O comando de Roll do piloto e do FBW mantém **100% de autoridade e prioridade**. Quando o comando de Roll aproxima o servo do limite angular (`anglePulseLimit = ±278 µs`), o offset dos flaperons é dinamicamente reduzido pela margem disponível (`flaperonHeadroom = 1.0 - |rollDiff| / anglePulseLimit`), garantindo **resposta de Roll imediata sem saturação, sem clipping e sem esforço mecânico excessivo nos servos** (mantendo-os sempre com folga ampla dentro da faixa [1000, 2000] µs).
>
> **REGRA DE SEGURANÇA ANTI-APRENDIZAGEM**: Se o modo de Flaperons estiver ligado (SWB ON), o **Modo 3 não pode estar ativo** (mesmo com motor alto). Qualquer tentativa ou comando para Modo 3 coloca a aeronave **automaticamente no Modo 2** para impedir que o algoritmo Extremum Seeking adapte ganhos PID com base em características aerodinâmicas temporárias de aterragem.
>
> **Feedback Negativo Estável & Anti-Stall**: A equação de controlo de **Pitch PI-D** garante realimentação estritamente negativa (`pitchDiff = -constrain(pitchPidOut)`). Limite anti-stall de subida em FBW: **25°** (`FBW_MAX_PITCH_DEG = 25.0f`, $K_p = 5.00$). Teto anti-windup da componente integrativa: **±100 µs** (`MAX_INTEGRAL_PULSE_US = 100.0f`, garantindo ±9.0° de autoridade de trim para eliminar erro residual mantendo 64% de folga dinâmica para P+D).

| SWC | SWB | CH5 PWM Medido | Modo Ativo | Flaperons | Descrição |
|:---:|:---:|:--------------:|:----------:|:---------:|:----------|
| 1 | OFF | **1166 µs** (< 1247) | **Modo 1** | **OFF** | Pitch 100% Manual + Roll Assist Permanente (Flaps OFF) |
| 2 | OFF | **1328 µs** (1247–1370) | **Modo 2** | **OFF** | FBW Pitch PI-D + FBW Roll PI-D Fixo (Flaps OFF) |
| 3 | OFF | **1411 µs** (1370–1476) | **Modo 3** | **OFF** | FBW Pitch PI-D + FBW Roll Adaptativo Extremum Seeking (Flaps OFF) |
| 1 | ON  | **1541 µs** (1476–1683) | **Modo 1** | **ON**  | Pitch 100% Manual + Roll Assist + **Flaperons 20° DOWN** |
| 2 | ON  | **1825 µs** (1683–1884) | **Modo 2** | **ON**  | FBW Pitch PI-D + FBW Roll PI-D + **Flaperons 20° DOWN** |
| 3 | ON  | **1942 µs** (>= 1884) | **Modo 2** *(Auto)* | **ON**  | **Demotado para Modo 2** + **Flaperons 20° DOWN** (Regra Anti-Learning) |

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

