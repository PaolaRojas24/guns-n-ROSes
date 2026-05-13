"""
Calibración de kr y kl usando datos experimentales.

Método analítico (línea recta, θ ≈ 0, wr = wl = ω):
  Con H ≈ I y Σ acumulada en N pasos:
    σ_xx  = N · (r·dt/2)² · (kr+kl) · ω          →  kr + kl
    σ_θθ  = N · (r·dt/2)² · (kr+kl) · ω · 4/L²   →  verificación
    σ_xθ  = N · (r·dt/2)² · (kr−kl) · ω · 2/L    →  kr − kl
"""

import math
import csv
import os


# ── Parámetros del robot ──────────────────────────────────────────────────────
r  = 0.05    # radio rueda [m]
L  = 0.108   # distancia entre ruedas [m]
dt = 0.02    # periodo de muestreo [s]
v  = 0.2     # velocidad del experimento A [m/s]


# ── Utilidades ───────────────────────────────────────────────────────────────
def leer_csv(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"Archivo no encontrado: {path}")
    datos = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            datos.append((float(row['x_final']),
                          float(row['y_final']),
                          float(row['theta_final_rad'])))
    return datos


def estadisticas(datos):
    n   = len(datos)
    xs  = [d[0] for d in datos]
    ys  = [d[1] for d in datos]
    ts  = [d[2] for d in datos]

    def mean(v):      return sum(v) / n
    def var(v):
        m = mean(v)
        return sum((xi - m)**2 for xi in v) / (n - 1)
    def cov(a, b):
        ma, mb = mean(a), mean(b)
        return sum((ai - ma)*(bi - mb) for ai, bi in zip(a, b)) / (n - 1)

    return {
        'n':       n,
        'x_mu':    mean(xs),  'y_mu':  mean(ys),  't_mu':  mean(ts),
        'x_var':   var(xs),   'y_var': var(ys),   't_var': var(ts),
        'xt_cov':  cov(xs, ts),
    }


# ── Calibración analítica ────────────────────────────────────────────────────
def calibrar(st):
    d     = st['x_mu']           # distancia recorrida ≈ x final medio
    omega = v / r                 # vel. angular rueda [rad/s]
    N     = d / (v * dt)          # número de pasos

    # Factor base: N · (r·dt/2)² · ω
    base = N * (r * dt / 2)**2 * omega

    kr_kl_from_x   = st['x_var']  / base               # kr + kl  (ec. 1)
    kr_kl_from_t   = st['t_var']  / (base * 4 / L**2)  # kr + kl  (ec. 2, check)
    kr_mkl         = st['xt_cov'] / (base * 2 / L)     # kr − kl  (ec. 3)

    kr = (kr_kl_from_x + kr_mkl) / 2
    kl = (kr_kl_from_x - kr_mkl) / 2

    return {
        'N':           N,
        'kr_kl_x':     kr_kl_from_x,
        'kr_kl_t':     kr_kl_from_t,
        'kr_mkl':      kr_mkl,
        'kr':          kr,
        'kl':          kl,
    }


# ── Impresión de resultados ───────────────────────────────────────────────────
def imprimir_stats(nombre, st, target_x, target_y):
    print(f"\n{'='*58}")
    print(f"  {nombre}")
    print(f"{'='*58}")
    print(f"  Corridas : {st['n']}")
    print(f"  x  media : {st['x_mu']:.5f} m   "
          f"| error vs target ({target_x:.1f}m): {abs(st['x_mu']-target_x)*100:.1f} cm")
    print(f"  y  media : {st['y_mu']:.5f} m   "
          f"| error vs target ({target_y:.1f}m): {abs(st['y_mu']-target_y)*100:.1f} cm")
    print(f"  θ  media : {math.degrees(st['t_mu']):.3f}°")
    print(f"  σ_x      : {math.sqrt(st['x_var'])*1e3:.3f} mm")
    print(f"  σ_y      : {math.sqrt(st['y_var'])*1e3:.3f} mm")
    print(f"  σ_θ      : {math.degrees(math.sqrt(st['t_var'])):.4f}°")
    print(f"  cov(x,θ) : {st['xt_cov']:.4e}")


def imprimir_calibracion(cal):
    print(f"\n{'='*58}")
    print("  CALIBRACIÓN ANALÍTICA  (usando Experimento A)")
    print(f"{'='*58}")
    print(f"  Pasos estimados  N  = {cal['N']:.1f}")
    print(f"  (kr+kl) desde σ_xx  = {cal['kr_kl_x']:.6f}")
    print(f"  (kr+kl) desde σ_θθ  = {cal['kr_kl_t']:.6f}  ← verificación")
    print(f"  (kr−kl) desde cov   = {cal['kr_mkl']:.6f}")
    print()
    print(f"  ► kr = {cal['kr']:.5f}")
    print(f"  ► kl = {cal['kl']:.5f}")
    print()
    print("  Copia estos valores en config/params.yaml:")
    print(f"    kr: {round(cal['kr'], 4)}")
    print(f"    kl: {round(cal['kl'], 4)}")

    if abs(cal['kr'] - cal['kl']) / ((cal['kr'] + cal['kl']) / 2 + 1e-9) < 0.10:
        print("\n  Las ruedas son simétricas (|kr-kl|/media < 10%).")
        k_sym = (cal['kr'] + cal['kl']) / 2
        print(f"  Puedes usar kr = kl = {k_sym:.5f} si lo prefieres.")


# ── Main ──────────────────────────────────────────────────────────────────────
base_dir = os.path.dirname(os.path.abspath(__file__))

datos_A = leer_csv(os.path.join(base_dir, 'experimentoA.csv'))
datos_B = leer_csv(os.path.join(base_dir, 'experimentoB.csv'))

st_A = estadisticas(datos_A)
st_B = estadisticas(datos_B)

imprimir_stats("Experimento A — línea recta (0,0)→(1,0)", st_A, 1.0, 0.0)
imprimir_stats("Experimento B — trayectoria L   (0,0)→(1,0)→(1,1)", st_B, 1.0, 1.0)

cal = calibrar(st_A)
imprimir_calibracion(cal)
