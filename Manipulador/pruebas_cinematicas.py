import numpy as np
import pandas as pd
import math
import os
import sys
import matplotlib.pyplot as plt

# Intenta importar scipy para los p-valores de correlación
try:
    import scipy.stats as stats
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

# =========================================================================
#  VALIDACIÓN DE CINEMÁTICA INVERSA – ROBOT AR-4DF (VERSIÓN PYTHON)
#  Analisis extendido con Correlación y Gráfica de Barras
# =========================================================================

# Parámetros del robot
L1 = 7.6
L2 = 10.5
L3 = 10.0
L4 = 4.0
UMBRAL_POS = 0.001  # cm (Según el valor estricto de la nueva versión)

print("\n==========================================================")
print("   VALIDACIÓN DE CINEMÁTICA INVERSA – AR-4DF")
print("==========================================================\n")

if not SCIPY_AVAILABLE:
    print("AVISO: La librería 'scipy' no está instalada.")
    print("Para ver los p-valores en la correlación, instala con: pip install scipy\n")

# Intentar leer primero la versión Python, si no existe buscar la original
archivo_excel = 'MTH_Workspace_AR4DF_Completo2_Python.xlsx'
if not os.path.exists(archivo_excel):
    archivo_excel = 'MTH_Workspace_AR4DF_Completo2_Python.csv'
    if not os.path.exists(archivo_excel):
        archivo_excel = 'MTH_Workspace_AR4DF_Completo2.xlsx'
        
if not os.path.exists(archivo_excel):
    print("Error: No se encontró ningún archivo de Volumen de Trabajo para probar.")
    print("Por favor ejecuta primero 'volumen_de_trabajo.py'")
    sys.exit(1)

print(f"Leyendo datos del archivo: {archivo_excel}...")
try:
    if archivo_excel.endswith('.csv'): df = pd.read_csv(archivo_excel)
    else: df = pd.read_excel(archivo_excel)
except Exception as e:
    print(f"Error al leer Excel ({e}). Por favor instala openpyxl: pip install openpyxl pandas")
    sys.exit(1)

# Filtrar NaN
df = df.dropna()
n_filas = len(df)
print(f"  Filas válidas: {n_filas}")

def determinar_cuadrante(Px, Py):
    if Px >= 0 and Py >= 0: return 1
    elif Px < 0 and Py >= 0: return 2
    elif Px < 0 and Py < 0: return 3
    else: return 4

def directaGeometrica(q_deg):
    q1, q2, q3, q4 = map(math.radians, q_deg)
    q23 = -(q2 + q3)
    Noa = np.array([
        [math.cos(q1)*math.cos(q23), -math.cos(q1)*math.sin(q23)*math.sin(q4)-math.sin(q1)*math.cos(q4), -math.cos(q1)*math.sin(q23)*math.cos(q4)+math.sin(q1)*math.sin(q4), 0],
        [math.sin(q1)*math.cos(q23), -math.sin(q1)*math.sin(q23)*math.sin(q4)+math.cos(q1)*math.cos(q4), -math.sin(q1)*math.sin(q23)*math.cos(q4)-math.cos(q1)*math.sin(q4), 0],
        [math.sin(q23),              math.cos(q23)*math.sin(q4),                                           math.cos(q23)*math.cos(q4),                                         0],
        [0, 0, 0, 1]
    ])
    r = L2*math.cos(-q2) + (L3+L4)*math.cos(-q2-q3)
    T = np.array([
        [1, 0, 0, r*math.cos(q1)],
        [0, 1, 0, r*math.sin(q1)],
        [0, 0, 1, L1 + L2*math.sin(-q2) + (L3+L4)*math.sin(-q2-q3)],
        [0, 0, 0, 1]
    ])
    return T @ Noa

def denavit(theta, d, a, alfa):
    return np.array([
        [math.cos(theta), -math.cos(alfa)*math.sin(theta), math.sin(alfa)*math.sin(theta), a*math.cos(theta)],
        [math.sin(theta),  math.cos(alfa)*math.cos(theta), -math.sin(alfa)*math.cos(theta), a*math.sin(theta)],
        [0,                math.sin(alfa),                 math.cos(alfa),                 d],
        [0,                0,                              0,                              1]
    ])

def directaDH(q_deg):
    q1, q2, q3, q4 = map(math.radians, q_deg)
    M1 = denavit(q1, L1, 0, -math.pi/2)
    M2 = denavit(q2, 0, L2, 0)
    M3 = denavit(q3 - math.pi/2, 0, 0, -math.pi/2)
    M4 = denavit(q4, L3+L4, 0, 0)
    return M1 @ M2 @ M3 @ M4

def errorNOAP(T1, T2):
    ep = np.linalg.norm(T1[:3, 3] - T2[:3, 3])
    er = np.linalg.norm(T1[:3, :3] - T2[:3, :3], 'fro')
    return ep, er, ep + er

def inversaAnalitica(T):
    L34 = L3 + L4
    Ax, Ay, Az = T[0,2], T[1,2], T[2,2]
    Nz, Oz = T[2,0], T[2,1]
    Px, Py, Pz = T[0,3], T[1,3], T[2,3]
    
    q23 = math.atan2(-Az, math.sqrt(max(0, 1 - Az**2)))
    q1 = math.atan2(Ay, Ax)
    q4 = math.atan2(-Oz, Nz)
    
    W, S, N = math.cos(q23), math.sin(q23), math.cos(q1)
    denom = N * L2
    if abs(denom) < 1e-9: return [np.nan]*4
    
    senq2 = (Pz - L1 + L34*S) / (-L2)
    cosq2 = (Px - N*W*L34) / denom
    q2 = math.atan2(senq2, cosq2)
    q3 = q23 - q2
    
    q1, q2, q3, q4 = map(math.degrees, [q1, q2, q3, q4])
    
    if (Px < 0 and Py < 0) or (Px >= 0 and Py < 0):
        q1 += 180
        q2 = -180 - q2
        q3 = -q3
        q4 = (q4 + 180) % 360 - 180
        if q4 > 90: q4 -= 180
        if q4 < -90: q4 += 180
        
    if abs(q4) < 0.01: q4 = 0
    if abs(q4) > 90: q4 = 0
        
    return [q1, q2, q3, q4]

def inversaGeometrica(T):
    Px, Py, Pz = float(T[0,3]), float(T[1,3]), float(T[2,3])
    x4 = np.array([T[0,0], T[1,0], T[2,0]], dtype=float)
    l1, l2, l34 = L1, L2, L3 + L4
    
    q1 = math.atan2(Py, Px)
    if q1 < 0: q1 += math.pi
    try:
        val = max(-1.0, min(1.0, float((Py**2 + Px**2 + (Pz - l1)**2 - l34**2 - l2**2) / (2 * l34 * l2))))
        q3 = math.atan2(math.sqrt(1 - val**2), val)
    except Exception: return []
        
    q31 = float(q3); q32 = -q31
    dist = math.sqrt(Px**2 + Py**2)
    if dist == 0: dist = 0.001
    
    q21 = -math.atan((Pz-l1)/dist) - math.atan((l34*math.sin(-q31))/(l2 + l34*math.cos(-q31)))
    q22 =  math.atan((Pz-l1)/dist) + math.atan((l34*math.sin(-q32))/(l2 + l34*math.cos(-q32)))
    q23, q24 = -q21, -q22
    
    if q21 > 0: q21 -= math.pi
    if q22 > 0: q22 -= math.pi
    if q23 > 0: q23 -= math.pi
    if q24 > 0: q24 -= math.pi
    
    pairs = [(q21, q31), (q22, q32), (q21, q32), (q22, q31), (q23, q31), (q23, q32), (q24, q31), (q24, q32)]
    Q = []
    for q2_val, q3_val in pairs:
        M03 = denavit(q1, l1, 0, -math.pi/2) @ denavit(q2_val, 0, l2, 0) @ denavit(q3_val-math.pi/2, 0, 0, -math.pi/2)
        x3 = M03[:3,0]; y3 = M03[:3,1]
        denom = np.dot(x3, x4)
        if abs(denom) < 1e-6: denom = 1e-6
        q4k = math.atan(np.dot(x4, y3)/denom)
        Q.append([math.degrees(q) for q in (q1, q2_val, q3_val, q4k)])
    return Q

# Ejecutar Pruebas
print('Generando NOAP de referencia y ejecutando validaciones...\n')
val_ana = []
val_geo = []

data_mat = df.values
for idx in range(n_filas):
    q_deg = data_mat[idx, 0:4]
    
    T_excel = np.eye(4)
    T_excel[:4, :] = data_mat[idx, 4:20].reshape(4,4)
    c = determinar_cuadrante(T_excel[0,3], T_excel[1,3])
    T_ref = directaGeometrica(q_deg)
    
    # --- PRUEBA ANALÍTICA ---
    Q_ana = inversaAnalitica(T_excel)
    if not np.isnan(Q_ana[0]):
        T_rec = directaDH(Q_ana)
        ep, er, et = errorNOAP(T_ref, T_rec)
        val_ana.append([idx, c, ep, Q_ana[0], Q_ana[1], Q_ana[2], Q_ana[3]])
    
    # --- PRUEBA GEOMÉTRICA ---
    Q_geo = inversaGeometrica(T_excel)
    if len(Q_geo) > 0:
        mejor_ep, mejor_sol = float('inf'), None
        for q_sol in Q_geo:
            T_rec = directaDH(q_sol)
            ep, er, et = errorNOAP(T_ref, T_rec)
            if ep < mejor_ep: mejor_ep, mejor_sol = ep, q_sol
        if mejor_sol is not None:
            val_geo.append([idx, c, mejor_ep, mejor_sol[0], mejor_sol[1], mejor_sol[2], mejor_sol[3]])

# Resultados
print("==========================================================")
print("  ANÁLISIS DE RESULTADOS")
print("==========================================================")
df_ana = pd.DataFrame(val_ana, columns=['idx','c','ep','q1','q2','q3','q4'])
df_geo = pd.DataFrame(val_geo, columns=['idx','c','ep','q1','q2','q3','q4'])

for name, res_df in [("GEOMÉTRICA", df_geo), ("ANALÍTICA", df_ana)]:
    print(f"\n[{name}]")
    if len(res_df) == 0: print("Sin datos válidos."); continue
    prec_global = (res_df['ep'] <= UMBRAL_POS).mean() * 100
    print(f"  GLOBAL: media={res_df['ep'].mean():.6f} cm | std={res_df['ep'].std():.6f} | Precision={prec_global:.1f}%")

print("\n==========================================================")
print("  ANÁLISIS DE CORRELACIÓN ENTRE MÉTODOS")
print("==========================================================\n")

# Alinear errores por índice
merged = pd.merge(df_geo[['idx', 'ep', 'c']], df_ana[['idx', 'ep']], on='idx', suffixes=('_geo', '_ana')).dropna()

validos_corr = len(merged)
print(f"Puntos con solución en ambos métodos: {validos_corr} / {n_filas}")

if validos_corr >= 3:
    ep_geo = merged['ep_geo']
    ep_ana = merged['ep_ana']
    
    if SCIPY_AVAILABLE:
        r_pearson, p_pearson = stats.pearsonr(ep_geo, ep_ana)
        r_spearman, p_spearman = stats.spearmanr(ep_geo, ep_ana)
    else:
        r_pearson = ep_geo.corr(ep_ana, method='pearson')
        r_spearman = ep_geo.corr(ep_ana, method='spearman')
        p_pearson = p_spearman = float('nan')

    print("\n--- CORRELACIÓN DE PEARSON ---")
    print(f"  Coeficiente de correlación (r): {r_pearson:.8f}")
    if SCIPY_AVAILABLE:
        print(f"  p-valor: {p_pearson:.6e}")
    
    # Interpretación Pearson
    abs_r = abs(r_pearson)
    if abs_r >= 0.9: print("  Interpretación: Correlación muy fuerte")
    elif abs_r >= 0.7: print("  Interpretación: Correlación fuerte")
    elif abs_r >= 0.5: print("  Interpretación: Correlación moderada")
    elif abs_r >= 0.3: print("  Interpretación: Correlación débil")
    else: print("  Interpretación: Correlación muy débil o nula")

    if SCIPY_AVAILABLE:
        if p_pearson < 0.05: print("  ✓ Estadísticamente significativa (p < 0.05)")
        else: print("  ✗ NO estadísticamente significativa")

    print("\n--- CORRELACIÓN DE SPEARMAN (por rangos) ---")
    print(f"  Coeficiente de Spearman (ρ): {r_spearman:.8f}")
    if SCIPY_AVAILABLE:
        print(f"  p-valor: {p_spearman:.6e}")
        
    abs_rs = abs(r_spearman)
    if abs_rs >= 0.9: print("  Correlación monotónica muy fuerte")
    elif abs_rs >= 0.7: print("  Correlación monotónica fuerte")
    elif abs_rs >= 0.5: print("  Correlación monotónica moderada")
    elif abs_rs >= 0.3: print("  Correlación monotónica débil")
    else: print("  Correlación monotónica muy débil o nula")

    # Análisis de residuos
    residuos = ep_ana - ep_geo
    print("\n--- ANÁLISIS DE RESIDUOS ---")
    print(f"  Media de residuos: {residuos.mean():.8f} cm")
    print(f"  Desviación estándar: {residuos.std():.8f} cm")
    print(f"  Error cuadrático medio (RMSE): {np.sqrt(np.mean(residuos**2)):.8f} cm")

# =========================================================================
#  GRÁFICAS: DISPERSIÓN (CORRELACIÓN) Y CAJAS (RANGOS) POR CUADRANTE
# =========================================================================
print("\nGenerando gráficas interactiva de Correlación y Rangos de Error...")

plt.style.use('default')
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
fig.canvas.manager.set_window_title('Análisis de Correlación y Rangos de Datos')

colors = ['#4472c4', '#ed7d31', '#70ad47', '#ffc000']
labels = ['Q1 (Px>0, Py>0)', 'Q2 (Px<0, Py>0)', 'Q3 (Px<0, Py<0)', 'Q4 (Px>0, Py<0)']

# --- 1. Gráfico de Dispersión (Correlación Visual) ---
for c in range(1, 5):
    df_c = merged[merged['c'] == c]
    if len(df_c) > 0:
        # Puntos de dispersión
        ax1.scatter(df_c['ep_geo'], df_c['ep_ana'], alpha=0.6, color=colors[c-1], label=labels[c-1], edgecolors='k', linewidth=0.5)
        # Línea de tendencia lineal (Correlación)
        if len(df_c) > 1:
            z = np.polyfit(df_c['ep_geo'], df_c['ep_ana'], 1)
            p = np.poly1d(z)
            ax1.plot(df_c['ep_geo'], p(df_c['ep_geo']), color=colors[c-1], linestyle='-', alpha=0.8)

# Línea de identidad (Referencia perfecta x=y)
max_val = max(merged['ep_geo'].max(), merged['ep_ana'].max()) if len(merged) > 0 else 1
ax1.plot([0, max_val], [0, max_val], 'k--', alpha=0.5, label='y = x (Identidad)')

ax1.set_xlabel('Error Geométrica [cm]', fontsize=11)
ax1.set_ylabel('Error Analítica [cm]', fontsize=11)
ax1.set_title('Correlación por Cuadrante (Geométrica vs Analítica)', fontsize=13, fontweight='bold')
ax1.legend(fontsize=9)
ax1.grid(True, linestyle=':', alpha=0.6)

# --- 2. Gráfico de Coeficientes de Correlación (r y ρ) por Cuadrante (Sin Barras) ---
r_pearson_vals = []
r_spearman_vals = []

for c in range(1, 5):
    df_c = merged[merged['c'] == c]
    if len(df_c) >= 3:
        if SCIPY_AVAILABLE:
            rc_p, _ = stats.pearsonr(df_c['ep_geo'], df_c['ep_ana'])
            rc_s, _ = stats.spearmanr(df_c['ep_geo'], df_c['ep_ana'])
        else:
            rc_p = df_c['ep_geo'].corr(df_c['ep_ana'], method='pearson')
            rc_s = df_c['ep_geo'].corr(df_c['ep_ana'], method='spearman')
        r_pearson_vals.append(rc_p)
        r_spearman_vals.append(rc_s)
    else:
        r_pearson_vals.append(float('nan'))
        r_spearman_vals.append(float('nan'))

# Línea horizontal guía en Cero
ax2.axhline(0, color='gray', linestyle='--', linewidth=1)

# Dibujamos las líneas marcadas (evitando gráfica de barras según solicitado)
labels_qx = ['Q1', 'Q2', 'Q3', 'Q4']
ax2.plot(labels_qx, r_pearson_vals, marker='o', markersize=8, linewidth=2, color='#4472c4', label='Pearson (r)')
ax2.plot(labels_qx, r_spearman_vals, marker='s', markersize=8, linewidth=2, color='#ed7d31', label='Spearman (ρ)')

# Etiquetas exactas numéricas de los valores en cada punto
for i in range(4):
    if not np.isnan(r_pearson_vals[i]):
        ax2.annotate(f'{r_pearson_vals[i]:.3f}', (i, r_pearson_vals[i]), 
                     textcoords="offset points", xytext=(0,8), ha='center', fontsize=9, color='#1d4b9b', fontweight='bold')
    if not np.isnan(r_spearman_vals[i]):
        ax2.annotate(f'{r_spearman_vals[i]:.3f}', (i, r_spearman_vals[i]), 
                     textcoords="offset points", xytext=(0,-13), ha='center', fontsize=9, color='#b05210', fontweight='bold')

ax2.set_ylim(-1.1, 1.1)
ax2.set_ylabel('Coeficiente de Correlación [-1 a 1]', fontsize=11)
ax2.set_title('Coeficientes de Correlación por Cuadrante', fontsize=13, fontweight='bold')
ax2.grid(True, axis='y', linestyle=':', alpha=0.6)
ax2.legend(fontsize=10, loc='best')

plt.tight_layout()
print("Gráfica mostrándose en pantalla. Cierra la ventana interactiva para finalizar.")
plt.show()
