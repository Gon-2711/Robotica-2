import customtkinter as ctk
import tkinter as tk
from tkinter import messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import serial
import time
import math
import typing

class DarkRobotApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("MATLAB V5 - Python Interface")
        self.geometry("1450x900")
        
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")
        plt.style.use('dark_background')
        
        self.l1 = 7.6; self.l2 = 10.5; self.l3 = 10.0; self.l4 = 4.0
        self.arduino: typing.Any = None
        self.is_running = False
        self.qq_actual = np.zeros(4)
        self.noap_original = np.eye(4, dtype=float)
        
        self.metodo_dir_var = tk.StringVar(value="geo")
        self.metodo_inv_var = tk.StringVar(value="geo")
        self.anim_paso_var = tk.BooleanVar(value=False)
        
        self.mth_total_vars = [[tk.StringVar(value="0.0") for _ in range(4)] for _ in range(4)]
        self.val_inver_vars = [[tk.StringVar(value="0.0") for _ in range(4)] for _ in range(4)]
        self.soluciones_vars = [[tk.StringVar(value="-") for _ in range(8)] for _ in range(4)]
        self.solucion_optima_vars = [tk.StringVar(value="-") for _ in range(4)]
        
        self.fig = Figure(figsize=(7, 4.0), dpi=100)
        self.fig.patch.set_facecolor('#242424')  # type: ignore
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor('#242424')  # type: ignore
        
        self.q1_entry: ctk.CTkEntry; self.q2_entry: ctk.CTkEntry
        self.q3_entry: ctk.CTkEntry; self.q4_entry: ctk.CTkEntry
        self.efc_entry: ctk.CTkEntry; self.puerto_entry: ctk.CTkEntry
        self.btn_serial: ctk.CTkButton; self.lbl_estado: ctk.CTkLabel
        self.canvas: FigureCanvasTkAgg; self.btn_theme: ctk.CTkButton
        
        self.crear_interfaz()
        self.ejecutar_directa()

    def toggle_theme(self):
        if ctk.get_appearance_mode() == "Dark":
            ctk.set_appearance_mode("light")
            self.btn_theme.configure(text="☀️ Modo Claro", fg_color="#f0b62e", text_color="black")
            plt.style.use('default')
            self.fig.patch.set_facecolor('#f0f0f0')  # type: ignore
            self.ax.set_facecolor('#f0f0f0')  # type: ignore
        else:
            ctk.set_appearance_mode("dark")
            self.btn_theme.configure(text="🌙 Modo Oscuro", fg_color="#2b2b2b", text_color="white")
            plt.style.use('dark_background')
            self.fig.patch.set_facecolor('#242424')  # type: ignore
            self.ax.set_facecolor('#242424')  # type: ignore
        self.ejecutar_directa()

    # ================= MATH =================
    def denavit(self, theta: float, d: float, a: float, alfa: float):
        return np.array([
            [np.cos(theta), -np.cos(alfa)*np.sin(theta),  np.sin(alfa)*np.sin(theta), a*np.cos(theta)],
            [np.sin(theta),  np.cos(alfa)*np.cos(theta), -np.sin(alfa)*np.cos(theta), a*np.sin(theta)],
            [0,             np.sin(alfa),               np.cos(alfa),                 d],
            [0,             0,                          0,                            1]
        ], dtype=float)

    def rotacion_compuesta(self, q1: float, q2: float, q3: float, q4: float):
        q23 = -(q2 + q3)
        return np.array([
            [np.cos(q1)*np.cos(q23), -np.cos(q1)*np.sin(q23)*np.sin(q4) - np.sin(q1)*np.cos(q4), -np.cos(q1)*np.sin(q23)*np.cos(q4) + np.sin(q1)*np.sin(q4), 0],
            [np.sin(q1)*np.cos(q23), -np.sin(q1)*np.sin(q23)*np.sin(q4) + np.cos(q1)*np.cos(q4), -np.sin(q1)*np.sin(q23)*np.cos(q4) - np.cos(q1)*np.sin(q4), 0],
            [np.sin(q23),            np.cos(q23)*np.sin(q4),                                     np.cos(q23)*np.cos(q4),                                     0],
            [0,                      0,                                                          0,                                                          1]
        ], dtype=float)

    def get_matrices_dh(self, q1: float, q2: float, q3: float, q4: float):
        t1, t2, t3, t4 = [float(x) for x in np.radians([q1, q2, q3, q4])]
        M0 = np.eye(4, dtype=float)
        M1 = self.denavit(t1, self.l1, 0.0, -math.pi/2)
        M2 = M1 @ self.denavit(t2, 0.0, self.l2, 0.0)
        M3 = M2 @ self.denavit(t3 - math.pi/2, 0.0, 0.0, -math.pi/2)
        M4 = M3 @ self.denavit(t4, self.l3 + self.l4, 0.0, 0.0)
        return M0, M1, M2, M3, M4

    def directa_dh(self, q1: float, q2: float, q3: float, q4: float, update_ui=True):
        M0, M1, M2, M3, M4 = self.get_matrices_dh(q1, q2, q3, q4)
        if update_ui:
            self.actualizar_tabla(self.mth_total_vars, M4)
            self.dibujar_robot([M0, M1, M2, M3, M4], is_dh=True)
            self.noap_original = M4.copy()
        return M4

    def directa_geometrica(self, q1: float, q2: float, q3: float, q4: float, update_ui=True):
        t1, t2, t3, t4 = [float(x) for x in np.radians([q1, q2, q3, q4])]
        Com = np.array([[0,0,1,0],[0,-1,0,0],[1,0,0,0],[0,0,0,1]], dtype=float)
        G = self.rotacion_compuesta(t1, t2, t3, t4) @ Com
        r = self.l2*math.cos(-t2) + (self.l3+self.l4)*math.cos(-t2 - t3)
        Px = r * math.cos(t1); Py = r * math.sin(t1)
        Pz = self.l1 + self.l2*math.sin(-t2) + (self.l3+self.l4)*math.sin(-t2-t3)
        A = np.array([[1,0,0,Px],[0,1,0,Py],[0,0,1,Pz],[0,0,0,1]], dtype=float)
        M4 = A @ G
        M0 = np.eye(4, dtype=float)
        M1 = np.array([[math.cos(t1),-math.sin(t1),0,0],[math.sin(t1),math.cos(t1),0,0],[0,0,1,self.l1],[0,0,0,1]], dtype=float)
        r2 = self.l2*math.cos(t2); z2 = self.l1 - self.l2*math.sin(t2)
        M2 = np.array([[math.cos(t1)*math.cos(t2),-math.cos(t1)*math.sin(t2),-math.sin(t1),r2*math.cos(t1)],
                       [math.sin(t1)*math.cos(t2),-math.sin(t1)*math.sin(t2),math.cos(t1),r2*math.sin(t1)],
                       [math.sin(t2),math.cos(t2),0,z2],[0,0,0,1]], dtype=float)
        r3 = self.l2*math.cos(t2) + self.l3*math.cos(t2+t3)
        z3 = self.l1 - self.l2*math.sin(t2) - self.l3*math.sin(t2+t3)
        M3 = np.array([[math.cos(t1)*math.cos(t2+t3),-math.cos(t1)*math.sin(t2+t3),-math.sin(t1),r3*math.cos(t1)],
                       [math.sin(t1)*math.cos(t2+t3),-math.sin(t1)*math.sin(t2+t3),math.cos(t1),r3*math.sin(t1)],
                       [math.sin(t2+t3),math.cos(t2+t3),0,z3],[0,0,0,1]], dtype=float)
        if update_ui:
            self.actualizar_tabla(self.mth_total_vars, M4)
            self.dibujar_robot([M0, M1, M2, M3, M4], is_dh=False)
            self.noap_original = M4.copy()
        return M4

    def seleccionar_mejor_solucion(self, Qsol, T_deseada):
        # type: ignore
        pos_actual = [float(self.q1_entry.get() or 0), float(self.q2_entry.get() or 0), float(self.q3_entry.get() or 0), float(self.q4_entry.get() or 0)]
        mejor_sol = None
        menor_error = float('inf')
        
        for idx, sol in enumerate(Qsol):
            # Limites [-180 180; -180 0; -90 90; -90 90]
            if not (-180 <= sol[0] <= 180 and -180 <= sol[1] <= 0 and -90 <= sol[2] <= 90 and -90 <= sol[3] <= 90):
                continue
            
            T_calc = self.directa_dh(float(sol[0]), float(sol[1]), float(sol[2]), float(sol[3]), update_ui=False)
            error_NOAP = np.linalg.norm(T_deseada[:3, :4] - T_calc[:3, :4])
            diferencia_angular = sum(abs(s - p) for s, p in zip(sol, pos_actual))
            diferencia_q2q3 = abs(sol[1] - pos_actual[1]) + abs(sol[2] - pos_actual[2])
            
            error_total = error_NOAP * 1000 + diferencia_angular + diferencia_q2q3 * 2
            
            if error_total < menor_error:
                menor_error = float(error_total)
                mejor_sol = sol
                
        if mejor_sol is None:
            mejor_sol = Qsol[0]
        return mejor_sol

    def inv_analitica(self, T):
        Ax, Ay, Az = float(T[0,2]), float(T[1,2]), float(T[2,2])
        Nz, Oz = float(T[2,0]), float(T[2,1])
        Px, Py, Pz = float(T[0,3]), float(T[1,3]), float(T[2,3])
        L1, L2, L34 = self.l1, self.l2, self.l3 + self.l4
        
        q23 = math.atan2(-Az, math.sqrt(max(0.0, float(1.0 - (-Az)**2))))
        q1 = math.atan2(Ay, Ax)
        q4 = math.atan2(-Oz, Nz)
        W, S, N = math.cos(q23), math.sin(q23), math.cos(q1)
        
        try:
            senq2 = (Pz - L1 + L34 * S) / (-L2)
            cosq2 = (Px - N * W * L34) / (N * L2)
            q2 = math.atan2(senq2, cosq2)
        except ZeroDivisionError: q2 = 0.0
            
        q3 = q23 - q2
        q1, q2, q3, q4 = map(math.degrees, [q1, q2, q3, q4])

        if Px < 0 and Py < 0:
            q1 += 180; q2 = math.sqrt(q2**2) - 180; q3 = math.sqrt(q3**2)
            q4 = math.degrees(math.cos(math.pi) * math.radians(q4))
        elif Px < 0 or (Px < 0 and Py >= 0): 
            q1 += 180; q2 = -180 - q2; q3 = -q3
            q4 = (q4 + 180) % 360 - 180
            if q4 > 0: q4 -= 180
            else: q4 += 180
            if q4 > 90: q4 -= 180
            elif q4 < -90: q4 += 180
            
        return [[q1, q2, q3, q4]]

    def get_axes_for_q4(self, q1: float, q2: float, q3: float):
        M03 = self.denavit(q1, self.l1, 0.0, -math.pi/2) @ self.denavit(q2, 0.0, self.l2, 0.0) @ self.denavit(q3 - math.pi/2, 0.0, 0.0, -math.pi/2)
        return M03[:3,0], M03[:3,1]

    def inv_geo(self, T):
        Px, Py, Pz = float(T[0,3]), float(T[1,3]), float(T[2,3])
        x4 = np.array([T[0,0], T[1,0], T[2,0]], dtype=float)
        l1, l2, l34 = self.l1, self.l2, self.l3 + self.l4
        
        q1 = math.atan2(Py, Px)
        if q1 < 0: q1 += math.pi
        try:
            val = max(-1.0, min(1.0, float((Py**2 + Px**2 + (Pz - l1)**2 - l34**2 - l2**2) / (2 * l34 * l2))))
            q3 = math.atan2(math.sqrt(1 - val**2), val)
        except Exception: q3 = 0.0
            
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
        sols = []
        for q2_val, q3_val in pairs:
            x3, y3 = self.get_axes_for_q4(q1, q2_val, q3_val)
            denom = np.dot(x3, x4)
            if abs(denom) < 1e-6: denom = 1e-6
            sols.append(math.atan(np.dot(x4, y3)/denom))
            
        return [
            [math.degrees(q1), math.degrees(q24), math.degrees(q31), math.degrees(sols[7])],
            [math.degrees(q1), math.degrees(q23), math.degrees(q32), math.degrees(sols[4])],
            [math.degrees(q1), math.degrees(q23), math.degrees(q31), math.degrees(sols[0])],
            [math.degrees(q1), math.degrees(q24), math.degrees(q32), math.degrees(sols[6])],
            [math.degrees(q1), math.degrees(q22), math.degrees(q31), math.degrees(sols[2])],
            [math.degrees(q1), math.degrees(q22), math.degrees(q32), math.degrees(sols[5])],
            [math.degrees(q1), math.degrees(q21), math.degrees(q31), math.degrees(sols[3])],
            [math.degrees(q1), math.degrees(q21), math.degrees(q32), math.degrees(sols[1])]
        ]

    # ================= UI & CONTROL =================
    def ejecutar_directa(self):
        try:
            q1, q2 = float(self.q1_entry.get() or "0"), float(self.q2_entry.get() or "0")
            q3, q4 = float(self.q3_entry.get() or "0"), float(self.q4_entry.get() or "0")
            if self.metodo_dir_var.get() == "dh": self.directa_dh(q1, q2, q3, q4)
            else: self.directa_geometrica(q1, q2, q3, q4)
            self.actualizar_tabla(self.val_inver_vars, self.noap_original)
        except ValueError: messagebox.showwarning("Error", "Valores inválidos.")

    def animar_directa(self):
        try:
            q_target = np.array([float(e.get() or "0") for e in (self.q1_entry, self.q2_entry, self.q3_entry, self.q4_entry)])
            self.enviar_serial()
            q_actual = self.qq_actual.copy()
            for i in range(1, 21):
                Q = q_actual + (q_target - q_actual) * (i / 20)
                if self.metodo_dir_var.get() == "dh": self.directa_dh(float(Q[0]), float(Q[1]), float(Q[2]), float(Q[3]))
                else: self.directa_geometrica(float(Q[0]), float(Q[1]), float(Q[2]), float(Q[3]))
                self.update(); time.sleep(0.02)
            self.qq_actual = q_target
        except ValueError: pass

    def ejecutar_inversa(self):
        T_data = np.zeros((4,4), dtype=float)
        try:
            for i in range(4):
                for j in range(4): T_data[i][j] = float(self.val_inver_vars[i][j].get())
        except ValueError: return
        
        Qsol = self.inv_geo(T_data) if self.metodo_inv_var.get() == "geo" else self.inv_analitica(T_data)
        
        # Llenar la tabla de 8 soluciones
        for i in range(4):
            for j in range(8):
                if j < len(Qsol): 
                    # type: ignore
                    self.soluciones_vars[i][j].set(f"{float(Qsol[j][i]):.2f}")
                else: 
                    self.soluciones_vars[i][j].set("-")

        # Seleccionar la mejor solución usando la métrica V5
        mejor = self.seleccionar_mejor_solucion(Qsol, T_data)
        
        # Llenar la tabla de S.Optima (V5 Feature)
        for i in range(4):
            # type: ignore
            self.solucion_optima_vars[i].set(f"{float(mejor[i]):.2f}")

        # Ejecutar la cinemática directa sobre la solución óptima y actualizar las tablas (V5 Feature)
        self.qq_actual = mejor
        T_resultante = self.directa_dh(float(mejor[0]), float(mejor[1]), float(mejor[2]), float(mejor[3]), update_ui=False)
        self.actualizar_tabla(self.val_inver_vars, T_resultante)

        for e, v in zip([self.q1_entry, self.q2_entry, self.q3_entry, self.q4_entry], mejor):
            e.delete(0, 'end'); e.insert(0, f"{float(v):.2f}") # type: ignore
            
        self.animar_directa()

    def dibujar_sistema(self, Mth, scale, type_sys):
        # type_sys: 0=base, 1=intermedio, 2=efector final
        origin = Mth[:3, 3]
        x_axis = origin + Mth[:3, 0] * scale
        y_axis = origin + Mth[:3, 1] * scale
        z_axis = origin + Mth[:3, 2] * scale
        
        axis_width = 3 if type_sys in (0, 2) else 2
        
        self.ax.plot([origin[0], x_axis[0]], [origin[1], x_axis[1]], [origin[2], x_axis[2]], color='r', linewidth=axis_width)
        self.ax.plot([origin[0], y_axis[0]], [origin[1], y_axis[1]], [origin[2], y_axis[2]], color='g', linewidth=axis_width)
        self.ax.plot([origin[0], z_axis[0]], [origin[1], z_axis[1]], [origin[2], z_axis[2]], color='b', linewidth=axis_width)
        
        if type_sys == 0 or type_sys == 2:
            self.ax.text(x_axis[0]*1.1, x_axis[1]*1.1, x_axis[2]*1.1, 'X', color='r', fontweight='bold', fontsize=7)
            self.ax.text(y_axis[0]*1.1, y_axis[1]*1.1, y_axis[2]*1.1, 'Y', color='g', fontweight='bold', fontsize=7)
            self.ax.text(z_axis[0]*1.1, z_axis[1]*1.1, z_axis[2]*1.1, 'Z', color='b', fontweight='bold', fontsize=7)

    def dibujar_robot(self, M_list, is_dh=True):
        self.ax.cla()
        is_dark = (ctk.get_appearance_mode() == "Dark")
        color_text = "white" if is_dark else "black"
        
        self.ax.set_title('Manipulador de 4 GDL', color=color_text, fontsize=14, fontweight='bold')
        self.ax.set_xlabel('X [cm]'); self.ax.set_ylabel('Y [cm]'); self.ax.set_zlabel('Z [cm]')
        self.ax.set_xlim([-50, 50]); self.ax.set_ylim([-50, 50]); self.ax.set_zlim([0, 50])
        
        self.ax.xaxis.label.set_color(color_text); self.ax.tick_params(axis='x', colors=color_text)  # type: ignore
        self.ax.yaxis.label.set_color(color_text); self.ax.tick_params(axis='y', colors=color_text)  # type: ignore
        self.ax.zaxis.label.set_color(color_text); self.ax.tick_params(axis='z', colors=color_text)  # type: ignore

        # Dibujar los sistemas coordenados (V5 Feature)
        self.dibujar_sistema(M_list[0], 6, 0)
        self.dibujar_sistema(M_list[1], 4, 1)
        self.dibujar_sistema(M_list[2], 4, 1)
        self.dibujar_sistema(M_list[3], 4, 1)
        self.dibujar_sistema(M_list[4], 6, 2)

        # Determinar los puntos articulares a unir
        p0 = M_list[0][:3,3]
        p1 = M_list[1][:3,3]
        p2 = M_list[2][:3,3]
        if is_dh: p3_visual = p2 + (M_list[3][:3, 2] * self.l3)
        else: p3_visual = M_list[3][:3,3]
        p4 = M_list[4][:3,3]
        
        pts = [p0, p1, p2, p3_visual, p4]
        colors = ['g', 'b', 'r', 'y']
        
        for i in range(4):
            self.ax.plot([pts[i][0], pts[i+1][0]], [pts[i][1], pts[i+1][1]], [pts[i][2], pts[i+1][2]], colors[i]+'-', linewidth=6)
            self.ax.scatter(pts[i][0], pts[i][1], pts[i][2], color=color_text, s=50)
            
        self.canvas.draw()

    def actualizar_tabla(self, variables, matriz):
        for i in range(4):
            for j in range(4):
                # type: ignore
                variables[i][j].set(f"{float(matriz[i][j]):.3f}")  
                
    def toggle_serial(self):
        if not self.is_running:
            try:
                self.arduino = serial.Serial(self.puerto_entry.get(), 9600, timeout=1)  # type: ignore
                time.sleep(2)
                self.is_running = True
                self.btn_serial.configure(text="DESACTIVAR", fg_color="#c94f4f")
                self.lbl_estado.configure(text="🟢")
            except Exception as e: messagebox.showerror("Error", str(e))
        else:
            if self.arduino: self.arduino.close()  # type: ignore
            self.is_running = False
            self.btn_serial.configure(text="ACTIVAR", fg_color=["#3a7ebf", "#1f538d"])
            self.lbl_estado.configure(text="🔴")

    def enviar_serial(self):
        # type: ignore
        if self.is_running and self.arduino and self.arduino.is_open:
            try:
                trama = f"{self.q1_entry.get()},{self.q2_entry.get()},{self.q3_entry.get()},{self.q4_entry.get()},{self.efc_entry.get() or '0'}\n"
                self.arduino.write(trama.encode('utf-8'))  # type: ignore
            except Exception: pass

    # ================= FRONTEND =================
    def crear_interfaz(self):
        frame_main = ctk.CTkFrame(self, fg_color="transparent")
        frame_main.pack(fill="both", expand=True, padx=10, pady=5)
        
        col_izq = ctk.CTkFrame(frame_main, fg_color="transparent", width=350)
        col_izq.pack(side="left", fill="y", padx=5)
        col_der = ctk.CTkFrame(frame_main, fg_color="transparent")
        col_der.pack(side="left", fill="both", expand=True, padx=5)

        header_der = ctk.CTkFrame(col_der, fg_color="transparent")
        header_der.pack(fill="x", pady=(0,0))
        self.btn_theme = ctk.CTkButton(header_der, text="🌙 Modo Oscuro", width=80, 
                                       command=self.toggle_theme, font=ctk.CTkFont(weight="bold"), 
                                       fg_color="#2b2b2b", hover_color="#454545")
        self.btn_theme.pack(side="right", padx=10, pady=2)

        p_serial = ctk.CTkFrame(col_izq)
        p_serial.pack(fill="x", pady=2)
        self.btn_serial = ctk.CTkButton(p_serial, text="ACTIVAR", command=self.toggle_serial, width=100, font=ctk.CTkFont(weight="bold"))
        self.btn_serial.pack(side="left", padx=10, pady=5)
        self.lbl_estado = ctk.CTkLabel(p_serial, text="🔴", font=ctk.CTkFont(size=18))
        self.lbl_estado.pack(side="left", padx=(0, 5))
        ctk.CTkLabel(p_serial, text="SERIAL", font=ctk.CTkFont(weight="bold")).pack(side="left", padx=5)
        self.puerto_entry = ctk.CTkEntry(p_serial, width=60, justify="center")
        self.puerto_entry.insert(0, "COM3")
        self.puerto_entry.pack(side="left", padx=5)
        
        p_art = ctk.CTkFrame(col_izq)
        p_art.pack(fill="x", pady=5)
        q_frame = ctk.CTkFrame(p_art, fg_color="transparent")
        q_frame.pack(side="left", padx=10, pady=5)
        ctk.CTkLabel(q_frame, text="Valores Articulares", font=ctk.CTkFont(weight="bold", size=15)).grid(row=0, column=0, columnspan=2, pady=2)
        
        def set_q(row, text, limit, default="0"):
            ctk.CTkLabel(q_frame, text=text, font=ctk.CTkFont(weight="bold")).grid(row=row, column=0, padx=2, pady=2)
            e = ctk.CTkEntry(q_frame, width=60, justify="center"); e.insert(0, default)
            e.grid(row=row, column=1); ctk.CTkLabel(q_frame, text=limit, font=ctk.CTkFont(size=10)).grid(row=row, column=2, padx=2)
            return e
            
        self.q1_entry = set_q(1, "Q1", "0 a 180")
        self.q2_entry = set_q(2, "Q2", "-180 a 0")
        self.q3_entry = set_q(3, "Q3", "-90 a 90")
        self.q4_entry = set_q(4, "Q4", "-90 a 90")
        
        dir_ctrl_f = ctk.CTkFrame(p_art, fg_color="transparent")
        dir_ctrl_f.pack(side="right", padx=5, pady=5)
        f_efc = ctk.CTkFrame(dir_ctrl_f, fg_color="transparent")
        f_efc.pack(pady=2)
        ctk.CTkLabel(f_efc, text="EFC", font=ctk.CTkFont(weight="bold")).pack(side="left")
        self.efc_entry = ctk.CTkEntry(f_efc, width=40, justify="center"); self.efc_entry.insert(0, "0"); self.efc_entry.pack(side="left", padx=5)
        
        ctk.CTkLabel(dir_ctrl_f, text="Cinematica Directa", font=ctk.CTkFont(weight="bold", size=14)).pack(pady=(5,2))
        m_sel = ctk.CTkFrame(dir_ctrl_f)
        m_sel.pack(fill="x")
        ctk.CTkLabel(m_sel, text="Seleccione Metodo", font=ctk.CTkFont(weight="bold", size=12)).pack()
        ctk.CTkRadioButton(m_sel, text="Geometrica", variable=self.metodo_dir_var, value="geo").pack(anchor="w", padx=5, pady=2)
        ctk.CTkRadioButton(m_sel, text="Denavit-H", variable=self.metodo_dir_var, value="dh").pack(anchor="w", padx=5, pady=2)
        ctk.CTkCheckBox(dir_ctrl_f, text="Animar Movimiento", variable=self.anim_paso_var).pack(pady=5)
        
        btn_f = ctk.CTkFrame(col_izq, fg_color="transparent")
        btn_f.pack(fill="x", pady=2)
        ctk.CTkButton(btn_f, text="Ejecutar", font=ctk.CTkFont(weight="bold"), command=self.ejecutar_directa).pack(side="left", expand=True, padx=2)
        ctk.CTkButton(btn_f, text="Animar", font=ctk.CTkFont(weight="bold"), command=self.animar_directa).pack(side="right", expand=True, padx=2)

        def build_4x4(parent, title, var_table, editable=False):
            f = ctk.CTkFrame(parent)
            ctk.CTkLabel(f, text=title, font=ctk.CTkFont(weight="bold", size=14)).pack(pady=2)
            g = ctk.CTkFrame(f, fg_color="transparent")
            g.pack(pady=2)
            for j, h in enumerate(["N", "O", "A", "P"]): ctk.CTkLabel(g, text=h, font=ctk.CTkFont(weight="bold")).grid(row=0, column=j, padx=2)
            for i in range(4):
                for j in range(4):
                    e = ctk.CTkEntry(g, width=65, justify="center", textvariable=var_table[i][j], state="normal" if editable else "disabled")
                    if not editable: e.configure(fg_color=("#cccccc", "#2b2b2b"), text_color=("black", "white"))
                    e.grid(row=i+1, column=j, padx=2, pady=2)
            return f

        build_4x4(col_izq, "MTH Total", self.mth_total_vars).pack(fill="x", pady=5)
        build_4x4(col_izq, "Valores De La Inversa (Editar)", self.val_inver_vars, editable=True).pack(fill="x", pady=5)

        # ---------------- DERECHA ----------------
        self.canvas = FigureCanvasTkAgg(self.fig, master=col_der)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, pady=5)

        p_inv = ctk.CTkFrame(col_der)
        p_inv.pack(fill="x", pady=5)
        inv_ctrl = ctk.CTkFrame(p_inv, fg_color="transparent")
        inv_ctrl.pack(fill="x", padx=10, pady=5)
        
        m_sel_inv = ctk.CTkFrame(inv_ctrl)
        m_sel_inv.pack(side="left")
        ctk.CTkLabel(m_sel_inv, text="Seleccione Metodo", font=ctk.CTkFont(weight="bold")).grid(row=0, column=0, columnspan=2, padx=10, pady=2)
        ctk.CTkRadioButton(m_sel_inv, text="Geometrica", variable=self.metodo_inv_var, value="geo").grid(row=1, column=0, padx=10, pady=5)
        ctk.CTkRadioButton(m_sel_inv, text="Analitica", variable=self.metodo_inv_var, value="ana").grid(row=1, column=1, padx=10, pady=5)
        
        ctk.CTkLabel(inv_ctrl, text="Cinematica Inversa", font=ctk.CTkFont(weight="bold", size=18)).pack(side="left", padx=50)
        ctk.CTkButton(inv_ctrl, text="Ejecutar Inversa", font=ctk.CTkFont(weight="bold", size=14), command=self.ejecutar_inversa).pack(side="right", padx=10)
        
        p_sol = ctk.CTkFrame(p_inv, fg_color="transparent")
        p_sol.pack(fill="x", padx=10, pady=(0, 5))
        
        f_title = ctk.CTkFrame(p_sol, fg_color="transparent")
        f_title.pack(fill="x")
        ctk.CTkLabel(f_title, text="Soluciones de la Cinematica Inversa", font=ctk.CTkFont(weight="bold", size=14)).pack(side="left", expand=True)
        ctk.CTkLabel(f_title, text="Solucion Optima", font=ctk.CTkFont(weight="bold", size=14), text_color="#f0b62e").pack(side="right", padx=30)
        
        f_sols_grid = ctk.CTkFrame(p_sol, fg_color="transparent")
        f_sols_grid.pack(fill="x", pady=5)
        
        g_sol = ctk.CTkFrame(f_sols_grid, fg_color=("#cccccc", "#1a1a1a"))  # type: ignore
        g_sol.pack(side="left", expand=True)
        ctk.CTkLabel(g_sol, text="Q", font=ctk.CTkFont(weight="bold")).grid(row=0, column=0, padx=5, pady=2)
        for j in range(8): ctk.CTkLabel(g_sol, text=f"S{j+1}", font=ctk.CTkFont(weight="bold")).grid(row=0, column=j+1, padx=5)
        for i, q_lbl in enumerate(["Q1", "Q2", "Q3", "Q4"]):
            ctk.CTkLabel(g_sol, text=q_lbl, font=ctk.CTkFont(weight="bold")).grid(row=i+1, column=0, padx=5)
            for j in range(8):
                e = ctk.CTkEntry(g_sol, width=58, justify="center", textvariable=self.soluciones_vars[i][j], state="disabled", corner_radius=3)
                e.configure(fg_color=("#e6e6e6", "#2b2b2b"), text_color=("black", "white"))
                e.grid(row=i+1, column=j+1, padx=2, pady=2)

        g_opt = ctk.CTkFrame(f_sols_grid, fg_color=("#cccccc", "#1a1a1a"), border_width=2, border_color="#f0b62e")  # type: ignore
        g_opt.pack(side="right", padx=10)
        ctk.CTkLabel(g_opt, text="Q", font=ctk.CTkFont(weight="bold")).grid(row=0, column=0, padx=5, pady=2)
        ctk.CTkLabel(g_opt, text="S.O", font=ctk.CTkFont(weight="bold", size=14), text_color="#f0b62e").grid(row=0, column=1, padx=5)
        for i, q_lbl in enumerate(["Q1", "Q2", "Q3", "Q4"]):
            ctk.CTkLabel(g_opt, text=q_lbl, font=ctk.CTkFont(weight="bold")).grid(row=i+1, column=0, padx=5)
            e = ctk.CTkEntry(g_opt, width=58, justify="center", textvariable=self.solucion_optima_vars[i], state="disabled", corner_radius=3)
            e.configure(fg_color=("#e6e6e6", "#2b2b2b"), text_color=("black", "#f0b62e"))
            e.grid(row=i+1, column=1, padx=2, pady=2)

if __name__ == "__main__":
    app = DarkRobotApp()
    app.protocol("WM_DELETE_WINDOW", lambda: (app.arduino.close() if app.arduino and getattr(app.arduino, 'is_open', False) else None, app.destroy())) # type: ignore
    app.mainloop()
