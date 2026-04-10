# Proyecto de Robótica 2

Este repositorio contiene los archivos y el código fuente desarrollados para el curso/proyecto de Robótica 2. El proyecto está dividido en dos partes principales: el control y simulación de un brazo manipulador, y una aplicación para cálculos dinámicos utilizando el enfoque de Lagrange-Euler.

## Estructura del Proyecto

### 1. Manipulador (`/Manipulador`)
Esta carpeta contiene todo lo relacionado con la cinemática, el control y la interfaz gráfica del brazo robótico.
- **Scripts de Python:** Incluye el control del brazo (`control_brazo.py`), la interfaz gráfica (`app_robotica_gui.py`, `main.py`), y pruebas cinemáticas y de volumen de trabajo.
- **Interfaz en MATLAB:** Aplicaciones de MATLAB (`.mlapp`) para la simulación y control previo.
- **Controlador Arduino:** Código fuente (`brazo_arduino.ino`) para la placa Arduino que controla físicamente los motores del brazo.
- **Archivos de Cinemática:** Hojas de cálculo de Excel (`.xlsx`) y scripts para los parámetros de Denavit-Hartenberg (MTH) y cálculos de cinemática inversa.

### 2. Algoritmo de Lagrange-Euler (`/Lagrange_Euler`)
- **Aplicación Web:** Un archivo `lagrange_euler_app.html` que sirve como interfaz o aplicación para visualizar y calcular la dinámica del robot utilizando las ecuaciones de Lagrange-Euler.

## Tecnologías Utilizadas
- **Python** (Control, cálculos, GUI)
- **Arduino** (Control de hardware)
- **MATLAB** (Simulaciones previas)
- **HTML/JS** (Aplicación web dinámica)

---
*Desarrollado por Gonzalo.*
