#!/usr/bin/env python3
"""
===============================================================================
MANTA Aircraft - SysID & Theoretical PID Gain Analyzer
===============================================================================
Analisa logs de voo em formato CSV organizados por subpastas (ex: helice_principal,
helice_alternativa, nao_categorizados).

Executa Identificação de Sistemas (SysID) nas superfícies de controlo:
  - V-Tail BR/BL (Pitch / Profundor)
  - Rollerons FR/FL (Roll / Ailerons)

Determina:
  1. Modelo dinâmico contínuo e discreto (Ganho K, Constante de tempo tau / wn, zeta)
  2. Grau de Confiança (%) baseado em R^2, NRMSE (FIT%) e variância de excitação
  3. Ganhos de PID teóricos calculados por IMC (Internal Model Control) e Ziegler-Nichols
===============================================================================
"""

import os
import sys
import glob
import json
import argparse
import numpy as np
import pandas as pd
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Tuple

# Força backend não-interativo seguro para headless/scripts
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


@dataclass
class AxisPIDResult:
    axis_name: str
    has_servo_data: bool
    model_type: str
    gain_k: float
    time_constant_tau: float
    nat_freq_wn: float
    damping_zeta: float
    r2_score: float
    fit_percentage: float
    confidence_percentage: float
    confidence_tier: str
    # PID IMC / Suave (Recomendado para voo estável e sem overshoot)
    kp_imc: float
    ki_imc: float
    kd_imc: float
    # PID Ágil / Rápido
    kp_agile: float
    ki_agile: float
    kd_agile: float


@dataclass
class FileAnalysisResult:
    filename: str
    filepath: str
    folder: str
    sample_count: int
    duration_s: float
    sample_rate_hz: float
    pitch_result: AxisPIDResult
    roll_result: AxisPIDResult
    overall_confidence: float
    flaperon_active_pct: float = 0.0


def classify_confidence_tier(conf_pct: float) -> str:
    if conf_pct >= 80.0:
        return "EXCELENTE (Muito Fiável)"
    elif conf_pct >= 65.0:
        return "BOM (Fiável para Sintonia)"
    elif conf_pct >= 45.0:
        return "MODERADO (Excitação Média)"
    elif conf_pct >= 25.0:
        return "BAIXO (Pouca Excitação / Ruído)"
    else:
        return "MUITO BAIXO (Dados Insuficientes)"


def identify_transfer_function(u: np.ndarray, y: np.ndarray, rate: np.ndarray, dt: float) -> Tuple[float, float, float, float, float, float]:
    """
    Executa identificação ARX de 1ª ordem para taxa angular (Rate) e atitude.
    Retorna: (K_rate, tau, wn, zeta, r2, fit_pct)
    """
    if len(u) < 30 or np.std(u) < 0.5:
        return 0.0, 0.1, 10.0, 0.7, 0.0, 0.0

    # Centralização
    u_c = u - np.mean(u)
    rate_c = rate - np.mean(rate)

    # Modelo ARX para Rate: rate[k] = a1 * rate[k-1] + b0 * u[k-1]
    Y = rate_c[1:]
    Phi = np.column_stack([rate_c[:-1], u_c[:-1]])
    theta, residuals, rank, _ = np.linalg.lstsq(Phi, Y, rcond=None)

    a1 = float(np.clip(theta[0], -0.999, 0.999))
    b0 = float(theta[1])

    # Previsão e métricas
    y_pred = Phi @ theta
    ss_tot = np.sum((Y - np.mean(Y)) ** 2)
    ss_res = np.sum((Y - y_pred) ** 2)

    r2 = max(0.0, float(1.0 - (ss_res / (ss_tot + 1e-9))))
    nrmse = np.sqrt(ss_res / len(Y)) / (np.std(Y) + 1e-9)
    fit_pct = max(0.0, min(100.0, float((1.0 - nrmse) * 100.0)))

    # Conversão para domínio contínuo: G(s) = K / (tau*s + 1)
    # Polo discreto p = a1 = exp(-dt/tau) => tau = -dt / ln(a1)
    if a1 > 0.01:
        tau = max(0.02, min(2.0, -dt / np.log(a1)))
        k_rate = b0 / (1.0 - a1 + 1e-9)
    else:
        tau = max(dt, 0.05)
        k_rate = b0 / (1.0 - a1 + 1e-9)

    # Parâmetros de 2ª ordem para atitude: wn e zeta
    wn = 1.0 / tau
    zeta = 0.75  # valor aerodinâmico típico para aileron/elevon

    return k_rate, tau, wn, zeta, r2, fit_pct


def synthesize_pid(k_rate: float, tau: float, is_pitch: bool = True) -> Tuple[float, float, float, float, float, float]:
    """
    Calcula ganhos de PID teóricos para controlo de atitude em loop fechado.
    Entrada: erro em graus (°), Saída: correção em microssegundos (us, span ±278us).
    """
    k_abs = max(abs(k_rate), 0.01)

    # Ganho estático normalizado de atitude (deg / (us * s))
    # Para IMC: lambda_tc é o tempo de resposta desejado em malha fechada
    # Pitch responde tipicamente em 0.25 - 0.35s; Roll responde em 0.20 - 0.30s
    tc_imc = 0.30 if is_pitch else 0.25
    tc_agile = 0.18 if is_pitch else 0.15

    # Ganhos IMC:
    # Kp = (tau + 0.5*dt) / (K_abs * tc)
    kp_imc = (tau) / (k_abs * tc_imc)
    ti_imc = max(tau * 2.0, 0.4)
    td_imc = max(tau * 0.25, 0.02)
    ki_imc = kp_imc / ti_imc
    kd_imc = kp_imc * td_imc

    # Ganhos Ágeis:
    kp_agile = (tau) / (k_abs * tc_agile)
    ti_agile = max(tau * 1.5, 0.3)
    td_agile = max(tau * 0.35, 0.03)
    ki_agile = kp_agile / ti_agile
    kd_agile = kp_agile * td_agile

    # Limites de saturação razoáveis para atuadores de 1000-2000us
    # Erro de 25 deg não deve saturar instantaneamente o servo (+-278us)
    kp_imc = float(np.clip(kp_imc, 0.5, 15.0))
    ki_imc = float(np.clip(ki_imc, 0.05, 5.0))
    kd_imc = float(np.clip(kd_imc, 0.01, 1.5))

    kp_agile = float(np.clip(kp_agile, 0.8, 25.0))
    ki_agile = float(np.clip(ki_agile, 0.1, 8.0))
    kd_agile = float(np.clip(kd_agile, 0.02, 2.5))

    return kp_imc, ki_imc, kd_imc, kp_agile, ki_agile, kd_agile


def estimate_without_servos(y: np.ndarray, dt: float, is_pitch: bool = True) -> AxisPIDResult:
    """
    Estimativa para ficheiros legados que não possuem colunas de PWM dos servos.
    Utiliza análise de frequência e decaimento natural das oscilações de atitude.
    """
    y_cent = y - np.mean(y)
    n = len(y_cent)
    if n < 50:
        return AxisPIDResult(
            axis_name="Pitch" if is_pitch else "Roll",
            has_servo_data=False,
            model_type="Free-Oscillation (Sem Servos)",
            gain_k=1.0,
            time_constant_tau=0.2,
            nat_freq_wn=5.0,
            damping_zeta=0.7,
            r2_score=0.2,
            fit_percentage=20.0,
            confidence_percentage=25.0,
            confidence_tier=classify_confidence_tier(25.0),
            kp_imc=3.5 if is_pitch else 2.5,
            ki_imc=0.6 if is_pitch else 0.4,
            kd_imc=0.15 if is_pitch else 0.10,
            kp_agile=5.0 if is_pitch else 4.0,
            ki_agile=1.0 if is_pitch else 0.8,
            kd_agile=0.25 if is_pitch else 0.18
        )

    # FFT para encontrar frequência dominante
    fft_vals = np.abs(np.fft.rfft(y_cent))
    freqs = np.fft.rfftfreq(n, d=dt)
    # Ignora DC (f = 0)
    idx_peak = np.argmax(fft_vals[1:]) + 1
    dom_freq = freqs[idx_peak]
    period_tu = 1.0 / max(dom_freq, 0.5)

    # Estimativa Ziegler-Nichols baseada no período natural Tu
    kp = 3.2 if is_pitch else 2.8
    ki = kp / (period_tu * 1.5)
    kd = kp * (period_tu * 0.12)

    conf = 35.0  # Confiabilidade intrinsecamente limitada por falta do sinal de controlo u(t)

    return AxisPIDResult(
        axis_name="Pitch" if is_pitch else "Roll",
        has_servo_data=False,
        model_type="FFT Oscilação Livre (Sem PWM de Servos)",
        gain_k=1.0,
        time_constant_tau=period_tu / (2 * np.pi),
        nat_freq_wn=2 * np.pi * dom_freq,
        damping_zeta=0.65,
        r2_score=0.30,
        fit_percentage=30.0,
        confidence_percentage=conf,
        confidence_tier=classify_confidence_tier(conf),
        kp_imc=round(kp, 2),
        ki_imc=round(ki, 2),
        kd_imc=round(kd, 3),
        kp_agile=round(kp * 1.4, 2),
        ki_agile=round(ki * 1.3, 2),
        kd_agile=round(kd * 1.5, 3)
    )


def analyze_csv_file(filepath: str, folder_name: str) -> Optional[FileAnalysisResult]:
    """Analisa um único ficheiro CSV de log de voo."""
    try:
        df = pd.read_csv(filepath)
    except Exception as e:
        print(f"  [ERRO] Falha ao ler {filepath}: {e}")
        return None

    if len(df) < 50:
        print(f"  [AVISO] Ficheiro {filepath} tem menos de 50 linhas. Ignorado.")
        return None

    # Deteta colunas essenciais
    cols = df.columns.str.strip().tolist()

    # Tempo
    time_col = None
    for cand in ["Elapsed Time (s)", "timestamp_s", "ESP32 Timestamp (ms)"]:
        if cand in cols:
            time_col = cand
            break

    if time_col is None:
        print(f"  [AVISO] Ficheiro {filepath} não tem coluna de tempo. Ignorado.")
        return None

    time_vals = df[time_col].values
    if "ms" in time_col:
        time_s = (time_vals - time_vals[0]) / 1000.0
    else:
        time_s = time_vals - time_vals[0]

    dt = np.diff(time_s)
    valid_dt = dt[dt > 0.001]
    dt_med = float(np.median(valid_dt)) if len(valid_dt) > 0 else 0.05
    sample_rate_hz = 1.0 / dt_med if dt_med > 0 else 20.0
    duration_s = float(time_s[-1] - time_s[0])

    # Procura Atitudes
    pitch_col = next((c for c in cols if "pitch" in c.lower()), None)
    roll_col = next((c for c in cols if "roll" in c.lower() and "rc" not in c.lower() and "servo" not in c.lower()), None)

    # Procura Gyros
    gy_col = next((c for c in cols if "gyro y" in c.lower()), None)
    gx_col = next((c for c in cols if "gyro x" in c.lower()), None)

    # Procura Servos (PWM)
    srv_pitch_col = next((c for c in cols if "servo br" in c.lower() or "servo bl" in c.lower()), None)
    srv_roll_col = next((c for c in cols if "servo fr" in c.lower() or "servo fl" in c.lower()), None)

    pitch_data = df[pitch_col].values.astype(float) if pitch_col else np.zeros(len(df))
    roll_data = df[roll_col].values.astype(float) if roll_col else np.zeros(len(df))

    # Auto-deteção de alinhamento físico dos eixos de giroscópio (MANTA PCB possui MPU6050 rodado a -90°)
    dp_dt = np.gradient(pitch_data, dt_med) if dt_med > 0 else np.zeros(len(pitch_data))
    dr_dt = np.gradient(roll_data, dt_med) if dt_med > 0 else np.zeros(len(roll_data))

    gyro_pitch_col = gy_col
    gyro_roll_col = gx_col

    if gx_col and gy_col and len(df) > 30 and (np.std(dp_dt) > 0.3 or np.std(dr_dt) > 0.3):
        gx_vals = df[gx_col].values.astype(float)
        gy_vals = df[gy_col].values.astype(float)
        c_gx_dp = abs(np.corrcoef(gx_vals, dp_dt)[0, 1]) if np.std(gx_vals) > 0 and np.std(dp_dt) > 0 else 0.0
        c_gy_dp = abs(np.corrcoef(gy_vals, dp_dt)[0, 1]) if np.std(gy_vals) > 0 and np.std(dp_dt) > 0 else 0.0
        if c_gx_dp > c_gy_dp:
            # Orientação física MANTA PCB: Gyro X = Taxa de Pitch, Gyro Y = Taxa de Roll
            gyro_pitch_col = gx_col
            gyro_roll_col = gy_col

    # Análise de Pitch
    if srv_pitch_col and pitch_col:
        u_p = df[srv_pitch_col].values.astype(float) - 1500.0
        if gyro_pitch_col:
            q_raw = df[gyro_pitch_col].values.astype(float) / 32.8
            # Inverte sinal se giroscópio estiver em fase negativa com a derivada de pitch
            if np.std(q_raw) > 0 and np.std(dp_dt) > 0 and np.corrcoef(q_raw, dp_dt)[0, 1] < 0:
                q_raw = -q_raw
            q = q_raw
        else:
            q = dp_dt
        k_p, tau_p, wn_p, z_p, r2_p, fit_p = identify_transfer_function(u_p, pitch_data, q, dt_med)

        # Fator de excitação (sinal de stick vs ruído)
        u_std = np.std(u_p)
        exc_factor = min(1.0, u_std / 30.0)
        # Confiança ponderada: 60% FIT, 20% R2, 20% Excitação do piloto
        conf_p = float(np.clip(fit_p * 0.60 + r2_p * 100.0 * 0.20 + exc_factor * 100.0 * 0.20, 5.0, 98.0))

        kp_imc, ki_imc, kd_imc, kp_ag, ki_ag, kd_ag = synthesize_pid(k_p, tau_p, is_pitch=True)
        pitch_res = AxisPIDResult(
            axis_name="Pitch",
            has_servo_data=True,
            model_type="SysID ARX 1ª Ordem (Atuador -> Gyro/Pitch)",
            gain_k=round(k_p, 4),
            time_constant_tau=round(tau_p, 3),
            nat_freq_wn=round(wn_p, 2),
            damping_zeta=round(z_p, 2),
            r2_score=round(r2_p, 3),
            fit_percentage=round(fit_p, 1),
            confidence_percentage=round(conf_p, 1),
            confidence_tier=classify_confidence_tier(conf_p),
            kp_imc=round(kp_imc, 2),
            ki_imc=round(ki_imc, 2),
            kd_imc=round(kd_imc, 3),
            kp_agile=round(kp_ag, 2),
            ki_agile=round(ki_ag, 2),
            kd_agile=round(kd_ag, 3)
        )
    else:
        pitch_res = estimate_without_servos(pitch_data, dt_med, is_pitch=True)

    # Análise de Roll
    if srv_roll_col and roll_col:
        u_r = df[srv_roll_col].values.astype(float) - 1500.0
        if gyro_roll_col:
            p_raw = df[gyro_roll_col].values.astype(float) / 32.8
            if np.std(p_raw) > 0 and np.std(dr_dt) > 0 and np.corrcoef(p_raw, dr_dt)[0, 1] < 0:
                p_raw = -p_raw
            p = p_raw
        else:
            p = dr_dt
        k_r, tau_r, wn_r, z_r, r2_r, fit_r = identify_transfer_function(u_r, roll_data, p, dt_med)

        u_std = np.std(u_r)
        exc_factor = min(1.0, u_std / 30.0)
        conf_r = float(np.clip(fit_r * 0.60 + r2_r * 100.0 * 0.20 + exc_factor * 100.0 * 0.20, 5.0, 98.0))

        kp_imc, ki_imc, kd_imc, kp_ag, ki_ag, kd_ag = synthesize_pid(k_r, tau_r, is_pitch=False)
        roll_res = AxisPIDResult(
            axis_name="Roll",
            has_servo_data=True,
            model_type="SysID ARX 1ª Ordem (Atuador -> Gyro/Roll)",
            gain_k=round(k_r, 4),
            time_constant_tau=round(tau_r, 3),
            nat_freq_wn=round(wn_r, 2),
            damping_zeta=round(z_r, 2),
            r2_score=round(r2_r, 3),
            fit_percentage=round(fit_r, 1),
            confidence_percentage=round(conf_r, 1),
            confidence_tier=classify_confidence_tier(conf_r),
            kp_imc=round(kp_imc, 2),
            ki_imc=round(ki_imc, 2),
            kd_imc=round(kd_imc, 3),
            kp_agile=round(kp_ag, 2),
            ki_agile=round(ki_ag, 2),
            kd_agile=round(kd_ag, 3)
        )
    else:
        roll_res = estimate_without_servos(roll_data, dt_med, is_pitch=False)

    overall_conf = round((pitch_res.confidence_percentage + roll_res.confidence_percentage) / 2.0, 1)

    # Deteta percentagem de voo com Flaperons (+10° DOWN) ativos
    flap_col = next((c for c in cols if "flaperon" in c.lower() or "flaps" in c.lower() or "assist" in c.lower()), None)
    flaperon_pct = 0.0
    if flap_col:
        try:
            flap_vals = df[flap_col].values.astype(float)
            flaperon_pct = round(float(np.mean(flap_vals > 0) * 100.0), 1)
        except Exception:
            flaperon_pct = 0.0

    return FileAnalysisResult(
        filename=os.path.basename(filepath),
        filepath=filepath,
        folder=folder_name,
        sample_count=len(df),
        duration_s=round(duration_s, 1),
        sample_rate_hz=round(sample_rate_hz, 1),
        pitch_result=pitch_res,
        roll_result=roll_res,
        overall_confidence=overall_conf,
        flaperon_active_pct=flaperon_pct
    )


def aggregate_folder_results(folder_name: str, file_results: List[FileAnalysisResult]) -> Dict:
    """Calcula os valores médios ponderados pela confiança para a pasta."""
    if not file_results:
        return {
            "folder": folder_name,
            "file_count": 0,
            "status": "VAZIA / SEM FICHEIROS CSV",
            "pitch": None,
            "roll": None,
            "overall_confidence": 0.0
        }

    # Ponderação pela confiança de cada teste
    weights = np.array([max(r.overall_confidence, 1.0) for r in file_results])
    w_sum = np.sum(weights)

    def w_avg(vals):
        return float(np.sum(np.array(vals) * weights) / w_sum)

    p_kp_imc = round(w_avg([r.pitch_result.kp_imc for r in file_results]), 2)
    p_ki_imc = round(w_avg([r.pitch_result.ki_imc for r in file_results]), 2)
    p_kd_imc = round(w_avg([r.pitch_result.kd_imc for r in file_results]), 3)

    p_kp_ag = round(w_avg([r.pitch_result.kp_agile for r in file_results]), 2)
    p_ki_ag = round(w_avg([r.pitch_result.ki_agile for r in file_results]), 2)
    p_kd_ag = round(w_avg([r.pitch_result.kd_agile for r in file_results]), 3)

    p_conf = round(w_avg([r.pitch_result.confidence_percentage for r in file_results]), 1)

    r_kp_imc = round(w_avg([r.roll_result.kp_imc for r in file_results]), 2)
    r_ki_imc = round(w_avg([r.roll_result.ki_imc for r in file_results]), 2)
    r_kd_imc = round(w_avg([r.roll_result.kd_imc for r in file_results]), 3)

    r_kp_ag = round(w_avg([r.roll_result.kp_agile for r in file_results]), 2)
    r_ki_ag = round(w_avg([r.roll_result.ki_agile for r in file_results]), 2)
    r_kd_ag = round(w_avg([r.roll_result.kd_agile for r in file_results]), 3)

    r_conf = round(w_avg([r.roll_result.confidence_percentage for r in file_results]), 1)

    overall_conf = round((p_conf + r_conf) / 2.0, 1)

    return {
        "folder": folder_name,
        "file_count": len(file_results),
        "files": [r.filename for r in file_results],
        "pitch": {
            "imc_recommended": {"kp": p_kp_imc, "ki": p_ki_imc, "kd": p_kd_imc},
            "agile_tuning": {"kp": p_kp_ag, "ki": p_ki_ag, "kd": p_kd_ag},
            "confidence_percentage": p_conf,
            "confidence_tier": classify_confidence_tier(p_conf)
        },
        "roll": {
            "imc_recommended": {"kp": r_kp_imc, "ki": r_ki_imc, "kd": r_kd_imc},
            "agile_tuning": {"kp": r_kp_ag, "ki": r_ki_ag, "kd": r_kd_ag},
            "confidence_percentage": r_conf,
            "confidence_tier": classify_confidence_tier(r_conf)
        },
        "overall_confidence": overall_conf,
        "overall_confidence_tier": classify_confidence_tier(overall_conf),
        "flaperon_active_pct": round(float(np.mean([r.flaperon_active_pct for r in file_results])), 1)
    }


def print_banner():
    print("=" * 80)
    print("       MANTA AIRFRAME - SISTEMA DE IDENTIFICAÇÃO DE SISTEMAS & SINTONIA PID")
    print("=" * 80)


def print_folder_summary(agg: Dict):
    folder = agg["folder"].upper().replace("_", " ")
    print(f"\n[{folder}] (Ficheiros analisados: {agg['file_count']})")
    print("-" * 80)
    if agg["file_count"] == 0:
        print("  Sem ficheiros válidos para análise nesta pasta.")
        return

    print(f"  Grau Geral de Confiança: {agg['overall_confidence']}% - [{agg['overall_confidence_tier']}]")
    print(f"  Ficheiros: {', '.join(agg['files'])}")
    if agg.get("flaperon_active_pct", 0.0) > 0.0:
        print(f"  Flaperons (10° DOWN): Ativos em {agg['flaperon_active_pct']}% das amostras do voo")
    print()

    # Pitch
    p_imc = agg["pitch"]["imc_recommended"]
    p_ag = agg["pitch"]["agile_tuning"]
    print(f"  --- EIXO PITCH / PROFUNDOR (V-Tail: Servos BR/BL) --- [Confiança: {agg['pitch']['confidence_percentage']}%]")
    print(f"    * Recomendado (IMC / Voo Suave):  Kp = {p_imc['kp']:>5.2f} | Ki = {p_imc['ki']:>5.2f} | Kd = {p_imc['kd']:>5.3f}")
    print(f"    * Perfil Ágil (Resposta Rápida): Kp = {p_ag['kp']:>5.2f} | Ki = {p_ag['ki']:>5.2f} | Kd = {p_ag['kd']:>5.3f}")
    print()

    # Roll
    r_imc = agg["roll"]["imc_recommended"]
    r_ag = agg["roll"]["agile_tuning"]
    print(f"  --- EIXO ROLL / AILERONS (Rollerons: Servos FR/FL) --- [Confiança: {agg['roll']['confidence_percentage']}%]")
    print(f"    * Recomendado (IMC / Voo Suave):  Kp = {r_imc['kp']:>5.2f} | Ki = {r_imc['ki']:>5.2f} | Kd = {r_imc['kd']:>5.3f}")
    print(f"    * Perfil Ágil (Resposta Rápida): Kp = {r_ag['kp']:>5.2f} | Ki = {r_ag['ki']:>5.2f} | Kd = {r_ag['kd']:>5.3f}")
    print("-" * 80)


def print_comparison_table(all_aggregates: Dict[str, Dict]):
    print("\n" + "=" * 80)
    print("             QUADRO COMPARATIVO DAS 3 CONFIGURAÇÕES DE TESTE")
    print("=" * 80)
    header = f"{'Subpasta / Configuração':<22} | {'Pitch (Kp, Ki, Kd)':<22} | {'Roll (Kp, Ki, Kd)':<22} | {'Confiança':<12}"
    print(header)
    print("-" * 80)

    for folder_key, agg in all_aggregates.items():
        name = folder_key.replace("_", " ").title()
        if agg["file_count"] == 0 or not agg["pitch"]:
            print(f"{name:<22} | {'Sem dados':<22} | {'Sem dados':<22} | {'0.0%':<12}")
            continue

        p = agg["pitch"]["imc_recommended"]
        r = agg["roll"]["imc_recommended"]
        p_str = f"{p['kp']:.2f}, {p['ki']:.2f}, {p['kd']:.3f}"
        r_str = f"{r['kp']:.2f}, {r['ki']:.2f}, {r['kd']:.3f}"
        conf_str = f"{agg['overall_confidence']:.1f}%"

        print(f"{name:<22} | {p_str:<22} | {r_str:<22} | {conf_str:<12}")

    print("=" * 80)
    print("Dica: Os valores de PID recomendados são calibrados para erro angular em graus (°)")
    print("e produzem uma deflexão em microssegundos (us) compatível com o firmware MANTA_ESP32.")
    print("=" * 80 + "\n")


def generate_plots(all_file_results: Dict[str, List[FileAnalysisResult]], output_dir: str):
    """Gera gráficos comparativos das respostas identificadas e salva em disco."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[AVISO] matplotlib não disponível para gerar gráficos.")
        return

    os.makedirs(output_dir, exist_ok=True)
    plot_path = os.path.join(output_dir, "manta_pid_comparison.png")

    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle("MANTA Aircraft - Respostas Dinâmicas & Sintonia de PIDs por Hélice", fontsize=14, fontweight="bold")

    folders = ["helice_principal", "helice_alternativa", "nao_categorizados"]
    row_titles = ["Hélice Principal", "Hélice Alternativa", "Não Categorizados"]

    for row_idx, (folder_key, row_title) in enumerate(zip(folders, row_titles)):
        results = all_file_results.get(folder_key, [])
        ax_p = axes[row_idx, 0]
        ax_r = axes[row_idx, 1]

        if not results:
            ax_p.text(0.5, 0.5, "Sem Dados", ha="center", va="center")
            ax_r.text(0.5, 0.5, "Sem Dados", ha="center", va="center")
            continue

        res = results[0]  # primeiro ficheiro representativo
        try:
            df = pd.read_csv(res.filepath)
            sample_rate = res.sample_rate_hz if res.sample_rate_hz > 0 else 20.0
            t = np.arange(min(300, len(df))) * (1.0 / sample_rate)
            pitch_col = next((c for c in df.columns if "pitch" in c.lower() and "rc" not in c.lower() and "servo" not in c.lower()), None)
            roll_col = next((c for c in df.columns if "roll" in c.lower() and "rc" not in c.lower() and "servo" not in c.lower()), None)
            p_meas = df[pitch_col].values[:len(t)] if pitch_col else np.zeros(len(t))
            r_meas = df[roll_col].values[:len(t)] if roll_col else np.zeros(len(t))

            ax_p.plot(t, p_meas, label=f"Pitch Real ({res.filename[:18]}...)", color="royalblue", lw=1.5)
            ax_p.set_title(f"{row_title} - Pitch (Conf: {res.pitch_result.confidence_percentage}%)", fontsize=10, fontweight="bold")
            ax_p.set_ylabel("Graus (°)")
            ax_p.grid(True, alpha=0.3)
            ax_p.legend(loc="upper right", fontsize=8)

            ax_r.plot(t, r_meas, label=f"Roll Real ({res.filename[:18]}...)", color="forestgreen", lw=1.5)
            ax_r.set_title(f"{row_title} - Roll (Conf: {res.roll_result.confidence_percentage}%)", fontsize=10, fontweight="bold")
            ax_r.set_ylabel("Graus (°)")
            ax_r.grid(True, alpha=0.3)
            ax_r.legend(loc="upper right", fontsize=8)

            if row_idx == 2:
                ax_p.set_xlabel("Tempo (s)")
                ax_r.set_xlabel("Tempo (s)")
        except Exception as e:
            ax_p.text(0.5, 0.5, f"Erro: {e}", ha="center", va="center")
            ax_r.text(0.5, 0.5, f"Erro: {e}", ha="center", va="center")

    plt.tight_layout()
    plt.savefig(plot_path, dpi=150)
    plt.close()
    print(f"[GRÁFICO] Gráfico comparativo guardado com sucesso em: {plot_path}")


def main():
    if sys.platform == "win32":
        try:
            sys.stdout.reconfigure(encoding="utf-8")
        except Exception:
            pass

    parser = argparse.ArgumentParser(description="MANTA Aircraft - SysID & Theoretical PID Gain Analyzer")
    parser.add_argument("--logs-dir", type=str, default=None, help="Diretoria base dos flight_logs")
    parser.add_argument("--plot", action="store_true", help="Gera gráficos comparativos em PNG")
    parser.add_argument("--json", action="store_true", help="Exporta resultados em JSON")
    args = parser.parse_args()

    # Procura a pasta flight_logs
    if args.logs_dir:
        base_dir = os.path.abspath(args.logs_dir)
    else:
        # Tenta diretoria relativa usual da Ground Station
        candidates = [
            os.path.abspath("Code/GROUND-STATION/flight_logs"),
            os.path.abspath("flight_logs"),
            os.path.abspath("../flight_logs"),
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "flight_logs")
        ]
        base_dir = next((c for c in candidates if os.path.isdir(c)), candidates[0])

    print_banner()
    print(f"Diretoria de Análise: {base_dir}\n")

    target_folders = ["helice_principal", "helice_alternativa", "nao_categorizados"]
    all_file_results: Dict[str, List[FileAnalysisResult]] = {}
    all_aggregates: Dict[str, Dict] = {}

    for folder_name in target_folders:
        folder_path = os.path.join(base_dir, folder_name)
        if not os.path.isdir(folder_path):
            os.makedirs(folder_path, exist_ok=True)

        csv_files = glob.glob(os.path.join(folder_path, "*.csv"))
        file_results = []
        for csv_file in csv_files:
            res = analyze_csv_file(csv_file, folder_name)
            if res:
                file_results.append(res)

        all_file_results[folder_name] = file_results
        agg = aggregate_folder_results(folder_name, file_results)
        all_aggregates[folder_name] = agg
        print_folder_summary(agg)

    print_comparison_table(all_aggregates)

    if args.plot:
        generate_plots(all_file_results, base_dir)

    if args.json:
        json_path = os.path.join(base_dir, "pid_analysis_results.json")
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(all_aggregates, f, indent=2, ensure_ascii=False)
        print(f"[JSON] Resultados exportados para: {json_path}")


if __name__ == "__main__":
    main()
