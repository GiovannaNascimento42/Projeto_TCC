#!/usr/bin/env python3
import argparse
import math
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def wrap_angle(a):
    return (a + np.pi) % (2*np.pi) - np.pi


def arr(df, col):
    return pd.to_numeric(df[col], errors="coerce").to_numpy(dtype=float)


def time_axis(df):
    t = arr(df, "time")
    return t - t[0]


def pos_error(df):
    if "position_error" in df.columns:
        return np.abs(arr(df, "position_error"))
    return np.hypot(arr(df, "goal_x") - arr(df, "x"),
                    arr(df, "goal_y") - arr(df, "y"))


def ang_error(df):
    if "heading_error" in df.columns:
        return wrap_angle(arr(df, "heading_error"))
    if "goal_theta" in df.columns:
        return wrap_angle(arr(df, "goal_theta") - arr(df, "theta"))
    return np.zeros(len(df))


def integral(y, t):
    if len(y) < 2:
        return 0.0
    return float(np.trapezoid(y, t) if hasattr(np, "trapezoid")
                 else np.trapz(y, t))


def first_below(t, e, limit):
    idx = np.flatnonzero(e <= limit)
    return float(t[idx[0]]) if len(idx) else np.nan


def settling_time(t, e, limit):
    for i in range(len(t)):
        if np.all(e[i:] <= limit):
            return float(t[i])
    return np.nan


def metrics(df):
    t = time_axis(df)
    e = pos_error(df)
    ae = ang_error(df)
    x, y, theta = arr(df, "x"), arr(df, "y"), arr(df, "theta")
    v = arr(df, "linear_cmd") if "linear_cmd" in df else np.zeros(len(df))
    w = arr(df, "angular_cmd") if "angular_cmd" in df else np.zeros(len(df))

    path = np.sum(np.hypot(np.diff(x), np.diff(y)))
    ef = float(e[-1])
    af = float(ae[-1])

    return {
        "amostras": len(df),
        "duracao_s": float(t[-1]),
        "erro_posicao_inicial_m": float(e[0]),
        "erro_posicao_final_m": ef,
        "erro_posicao_max_m": float(np.max(e)),
        "MAE_posicao_m": float(np.mean(np.abs(e))),
        "RMSE_posicao_m": float(np.sqrt(np.mean(e**2))),
        "tempo_ate_10cm_s": first_below(t, e, 0.10),
        "tempo_ate_5cm_s": first_below(t, e, 0.05),
        "tempo_ate_2cm_s": first_below(t, e, 0.02),
        "tempo_acomodacao_5cm_s": settling_time(t, e, 0.05),
        "MAE_orientacao_rad": float(np.mean(np.abs(ae))),
        "RMSE_orientacao_rad": float(np.sqrt(np.mean(ae**2))),
        "erro_orientacao_final_rad": af,
        "erro_orientacao_final_graus": math.degrees(af),
        "max_erro_orientacao_graus": math.degrees(float(np.max(np.abs(ae)))),
        "comprimento_trajetoria_m": float(path),
        "max_linear_cmd_m_s": float(np.max(np.abs(v))),
        "max_angular_cmd_rad_s": float(np.max(np.abs(w))),
        "integral_v2": integral(v**2, t),
        "integral_w2": integral(w**2, t),
        "esforco_total": integral(v**2, t) + integral(w**2, t),
        "x_final_m": float(x[-1]),
        "y_final_m": float(y[-1]),
        "theta_final_rad": float(theta[-1]),
        "theta_final_graus": math.degrees(float(theta[-1])),
    }


def save_plot(fig, path):
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_error(df, out):
    t, e = time_axis(df), pos_error(df)
    fig, ax = plt.subplots(figsize=(10, 5.5))
    ax.plot(t, e, linewidth=1.8, label="Erro de posição")
    ax.axhline(0.05, linestyle="--", linewidth=1.1, label="5 cm")
    ax.axhline(0.10, linestyle=":", linewidth=1.1, label="10 cm")
    ax.set(xlabel="Tempo (s)", ylabel="Erro de posição (m)",
           title="Erro de posição × tempo")
    ax.grid(True, alpha=0.3)
    ax.legend()
    save_plot(fig, out)


def plot_xy(df, out):
    x, y = arr(df, "x"), arr(df, "y")
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(x, y, linewidth=1.8, label="Trajetória")
    ax.scatter(x[0], y[0], s=50, label="Início")
    ax.scatter(x[-1], y[-1], s=50, label="Final")
    if "goal_x" in df and "goal_y" in df:
        ax.scatter(arr(df, "goal_x")[0], arr(df, "goal_y")[0],
                   marker="x", s=80, linewidths=2, label="Objetivo")
    ax.set(xlabel="X (m)", ylabel="Y (m)", title="Trajetória XY")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    save_plot(fig, out)


def plot_orientation(df, out):
    t, th = time_axis(df), np.degrees(arr(df, "theta"))
    fig, ax = plt.subplots(figsize=(10, 5.5))
    ax.plot(t, th, linewidth=1.8, label="θ")
    if "goal_theta" in df:
        ax.plot(t, np.degrees(arr(df, "goal_theta")),
                "--", linewidth=1.1, label="θ objetivo")
    ax.set(xlabel="Tempo (s)", ylabel="Orientação (graus)",
           title="Orientação × tempo")
    ax.grid(True, alpha=0.3)
    ax.legend()
    save_plot(fig, out)


def plot_ang_error(df, out):
    t, e = time_axis(df), np.degrees(ang_error(df))
    fig, ax = plt.subplots(figsize=(10, 5.5))
    ax.plot(t, e, linewidth=1.8, label="Erro de orientação")
    ax.axhline(0, linewidth=1)
    ax.axhline(5, linestyle="--", linewidth=1, label="±5°")
    ax.axhline(-5, linestyle="--", linewidth=1)
    ax.set(xlabel="Tempo (s)", ylabel="Erro angular (graus)",
           title="Erro de orientação × tempo")
    ax.grid(True, alpha=0.3)
    ax.legend()
    save_plot(fig, out)


def plot_commands(df, out):
    t = time_axis(df)
    fig, ax = plt.subplots(figsize=(10, 5.5))
    if "linear_cmd" in df:
        ax.plot(t, arr(df, "linear_cmd"), linewidth=1.8,
                label="Comando linear")
    if "angular_cmd" in df:
        ax.plot(t, arr(df, "angular_cmd"), linewidth=1.8,
                label="Comando angular")
    ax.set(xlabel="Tempo (s)", ylabel="Comando",
           title="Comandos de controle × tempo")
    ax.grid(True, alpha=0.3)
    ax.legend()
    save_plot(fig, out)


def plot_velocities(df, out):
    if "v_pose" not in df.columns and "w_pose" not in df.columns:
        return
    t = time_axis(df)
    fig, ax = plt.subplots(figsize=(10, 5.5))
    if "v_pose" in df:
        ax.plot(t, arr(df, "v_pose"), linewidth=1.8,
                label="Velocidade linear medida")
    if "w_pose" in df:
        ax.plot(t, arr(df, "w_pose"), linewidth=1.8,
                label="Velocidade angular medida")
    ax.set(xlabel="Tempo (s)", ylabel="Velocidade",
           title="Velocidades medidas × tempo")
    ax.grid(True, alpha=0.3)
    ax.legend()
    save_plot(fig, out)


def analyze(csv_file, base_out):
    csv_file = Path(csv_file).expanduser().resolve()
    df = pd.read_csv(csv_file)

    required = {"time", "x", "y", "theta"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"faltam colunas obrigatórias: {', '.join(sorted(missing))}")

    for c in required:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    df = df.dropna(subset=list(required)).reset_index(drop=True)
    if len(df) < 2:
        raise ValueError("menos de 2 amostras válidas")

    out = Path(base_out).expanduser().resolve() / csv_file.stem
    out.mkdir(parents=True, exist_ok=True)

    m = metrics(df)
    pd.DataFrame([m]).to_csv(out / "metricas.csv", index=False)
    with open(out / "metricas.txt", "w", encoding="utf-8") as f:
        f.write(f"Arquivo: {csv_file}\n\n")
        for k, val in m.items():
            f.write(f"{k}: {val:.8f}\n" if isinstance(val, float)
                    else f"{k}: {val}\n")

    plot_error(df, out / "erro_posicao.png")
    plot_xy(df, out / "trajetoria_xy.png")
    plot_orientation(df, out / "orientacao.png")
    plot_ang_error(df, out / "erro_orientacao.png")
    plot_commands(df, out / "comandos.png")
    plot_velocities(df, out / "velocidades.png")

    print(f"\n{csv_file.name}")
    print(f"  saída: {out}")
    print(f"  duração: {m['duracao_s']:.3f} s")
    print(f"  erro final: {m['erro_posicao_final_m']:.4f} m")
    print(f"  MAE: {m['MAE_posicao_m']:.4f} m")
    print(f"  RMSE: {m['RMSE_posicao_m']:.4f} m")
    print(f"  até 10 cm: {m['tempo_ate_10cm_s']:.3f} s")
    print(f"  até 5 cm: {m['tempo_ate_5cm_s']:.3f} s")
    print(f"  erro angular final: {m['erro_orientacao_final_graus']:.3f}°")
    return m


def main():
    parser = argparse.ArgumentParser(
        description="Gera gráficos e métricas de experimentos PID a partir de CSVs."
    )
    parser.add_argument("inputs", nargs="+",
                        help="CSV(s) ou diretório contendo CSVs")
    parser.add_argument("--output", default="./analises_pid",
                        help="diretório base dos resultados")
    args = parser.parse_args()

    files = []
    for item in args.inputs:
        p = Path(item).expanduser()
        if p.is_dir():
            files += sorted(p.glob("*.csv"))
        elif p.is_file() and p.suffix.lower() == ".csv":
            files.append(p)

    # remove duplicatas
    files = list(dict.fromkeys(str(p.resolve()) for p in files))
    if not files:
        raise SystemExit("Nenhum CSV encontrado.")

    results = []
    for f in files:
        try:
            m = analyze(f, args.output)
            m["arquivo"] = f
            results.append(m)
        except Exception as exc:
            print(f"ERRO em {f}: {exc}")

    if results:
        out = Path(args.output).expanduser().resolve()
        out.mkdir(parents=True, exist_ok=True)
        comparison = pd.DataFrame(results)
        cols = ["arquivo"] + [c for c in comparison.columns if c != "arquivo"]
        comparison[cols].to_csv(out / "comparacao_metricas.csv", index=False)
        print(f"\nComparação: {out / 'comparacao_metricas.csv'}")


if __name__ == "__main__":
    main()
