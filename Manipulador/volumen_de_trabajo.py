import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt

# Parámetros del manipulador
L1 = 7.6      # altura base [cm]
L2 = 10.5     # longitud eslabón 2 [cm]
L3 = 10.0     # longitud eslabón 3 [cm]
L4 = 4.0      # longitud eslabón 4 [cm]
L34 = L3 + L4 # distancia morfológica entre articulación 3 y 4

N_por_cuadrante = 3000
N_total = N_por_cuadrante * 4

print("==========================================================")
print("   GENERACIÓN DE VOLUMEN DE TRABAJO - AR-4DF")
print("==========================================================\n")
print(f"Generando {N_por_cuadrante} configuraciones por cuadrante...")
print(f"Total de configuraciones: {N_total}\n")

# Rangos articulares
theta1_min, theta1_max = -math.pi, math.pi
theta2_min, theta2_max = -math.pi, 0
theta3_min, theta3_max = -math.pi/2, math.pi/2
theta4_min, theta4_max = -math.pi/2, math.pi/2

# Matrices de almacenamiento
datos = np.zeros((N_total, 20))
X = np.zeros(N_total)
Y = np.zeros(N_total)
Z = np.zeros(N_total)
cuadrantes = np.zeros(N_total, dtype=int)

idx_global = 0

for cuadrante in range(1, 5):
    print(f"Generando cuadrante {cuadrante}...")
    
    for i in range(N_por_cuadrante):
        t1 = theta1_min + (theta1_max - theta1_min) * np.random.rand()
        t2 = theta2_min + (theta2_max - theta2_min) * np.random.rand()
        t3 = theta3_min + (theta3_max - theta3_min) * np.random.rand()
        t4 = theta4_min + (theta4_max - theta4_min) * np.random.rand()
        
        # Calcular posición para verificar cuadrante
        r = L2 * math.cos(-t2) + L34 * math.cos(-t2 - t3)
        Px = r * math.cos(t1)
        Py = r * math.sin(t1)
        Pz = L1 + L2 * math.sin(-t2) + L34 * math.sin(-t2 - t3)
        
        # Determinar cuadrante actual
        if Px >= 0 and Py >= 0: cuadrante_actual = 1
        elif Px < 0 and Py >= 0: cuadrante_actual = 2
        elif Px < 0 and Py < 0: cuadrante_actual = 3
        else: cuadrante_actual = 4
        
        # Si no es el cuadrante, forzar t1
        if cuadrante_actual != cuadrante:
            if cuadrante == 1:   t1 = math.atan2(abs(Py), abs(Px))
            elif cuadrante == 2: t1 = math.pi - math.atan2(abs(Py), abs(Px))
            elif cuadrante == 3: t1 = -math.pi + math.atan2(abs(Py), abs(Px))
            else:                t1 = -math.atan2(abs(Py), abs(Px))
                
        # Recalcular posiciones
        r = L2 * math.cos(-t2) + L34 * math.cos(-t2 - t3)
        Px = r * math.cos(t1)
        Py = r * math.sin(t1)
        Pz = L1 + L2 * math.sin(-t2) + L34 * math.sin(-t2 - t3)
        
        C1, S1 = math.cos(t1), math.sin(t1)
        C2, S2 = math.cos(t2), math.sin(t2)
        C23, S23 = math.cos(t2 + t3), math.sin(t2 + t3)
        C4, S4 = math.cos(t4), math.sin(t4)
        
        # Matriz de Transformación Homogénea (Directa D-H o NOAP)
        T = np.array([
            [C1*C4*S23 + S1*S4,  -C1*S4*S23 + C4*S1,  C1*C23,  C1*(L2*C2 + L34*C23)],
            [-C1*S4 + C4*S1*S23, -C1*C4 - S1*S4*S23,  S1*C23,  S1*(L2*C2 + L34*C23)],
            [C4*C23,             -S4*C23,            -S23,     L1 - L2*S2 - L34*S23],
            [0,                  0,                  0,        1]
        ])
        
        X[idx_global] = T[0, 3]
        Y[idx_global] = T[1, 3]
        Z[idx_global] = T[2, 3]
        cuadrantes[idx_global] = cuadrante
        
        # Guardar en la matriz (en grados)
        fila = [
            math.degrees(t1), math.degrees(t2), math.degrees(t3), math.degrees(t4),
            T[0,0], T[0,1], T[0,2], T[0,3],
            T[1,0], T[1,1], T[1,2], T[1,3],
            T[2,0], T[2,1], T[2,2], T[2,3],
            T[3,0], T[3,1], T[3,2], T[3,3]
        ]
        
        datos[idx_global, :] = fila
        idx_global += 1
        
    print(f"  Cuadrante {cuadrante} completado.")

# Crear DataFrame para Pandas
nombres_cols = [
    'theta1_deg','theta2_deg','theta3_deg','theta4_deg',
    'T11','T12','T13','T14',
    'T21','T22','T23','T24',
    'T31','T32','T33','T34',
    'T41','T42','T43','T44'
]

df = pd.DataFrame(datos, columns=nombres_cols)
nombre_archivo_py = 'MTH_Workspace_AR4DF_Completo2_Python.xlsx'

print("\nGuardando Excel (esto puede tomar unos segundos)...")
# Usar engine='openpyxl' si está disponible
try:
    df.to_excel(nombre_archivo_py, index=False)
except ImportError:
    print("Falta openpyxl. Guardando como CSV alternativo...")
    nombre_archivo_py = 'MTH_Workspace_AR4DF_Completo2_Python.csv'
    df.to_csv(nombre_archivo_py, index=False)

print("\n==========================================================")
print(f"Archivo Excel generado correctamente: {nombre_archivo_py}")
print(f"Total de configuraciones: {len(datos)}")
print("==========================================================\n")

# ================= VISUALIZACIÓN 3D =================
plt.style.use('dark_background')
fig = plt.figure(figsize=(12, 6), num="Volumen de Trabajo - AR-4DF (Python)")

# Primer subplot: Coloreado por cuadrantes
ax1 = fig.add_subplot(121, projection='3d')
cQ = {1: '#1f77b4', 2: '#2ca02c', 3: '#d62728', 4: '#ff7f0e'}  # Colores similares a MATLAB

for c in range(1, 5):
    mask = (cuadrantes == c)
    if np.any(mask):
        ax1.scatter(X[mask], Y[mask], Z[mask], c=cQ[c], s=10, alpha=0.6, label=f'Q{c}')

ax1.set_xlabel('X [cm]')
ax1.set_ylabel('Y [cm]')
ax1.set_zlabel('Z [cm]')
ax1.set_title('Volumen de Trabajo por Cuadrante')
ax1.legend()
ax1.view_init(elev=30, azim=45)

# Segundo subplot: Coloreado por altura Z
ax2 = fig.add_subplot(122, projection='3d')
sc = ax2.scatter(X, Y, Z, c=Z, cmap='viridis', s=10, alpha=0.8)
ax2.set_xlabel('X [cm]')
ax2.set_ylabel('Y [cm]')
ax2.set_zlabel('Z [cm]')
ax2.set_title('Volumen de Trabajo (color por altura Z)')
fig.colorbar(sc, ax=ax2, shrink=0.5, aspect=10)
ax2.view_init(elev=30, azim=45)

fig.suptitle('AR-4DF - Volumen de Trabajo', fontsize=14, fontweight='bold')

print("Estadísticas por cuadrante:")
print("-" * 50)
for c in range(1, 5):
    mask = (cuadrantes == c)
    if not np.any(mask): continue
    print(f"Cuadrante {c}: {np.sum(mask)} puntos")
    print(f"  X: media={np.mean(X[mask]):.2f}, min={np.min(X[mask]):.2f}, max={np.max(X[mask]):.2f}")
    print(f"  Y: media={np.mean(Y[mask]):.2f}, min={np.min(Y[mask]):.2f}, max={np.max(Y[mask]):.2f}")
    print(f"  Z: media={np.mean(Z[mask]):.2f}, min={np.min(Z[mask]):.2f}, max={np.max(Z[mask]):.2f}\n")

print("¡Archivo de datos y reporte listos! Cerrando la ventana del gráfico terminará el programa.")
plt.tight_layout()
plt.show()
