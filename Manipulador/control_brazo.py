import serial
import time

class BrazoRobotico:
    def __init__(self, puerto, baudrate=9600):
        """
        Inicializa la conexión serial con el Arduino.
        :param puerto: El puerto COM donde está conectado el Arduino (ej. 'COM3' en Windows, '/dev/ttyUSB0' en Linux).
        :param baudrate: Velocidad de comunicación en baudios (debe coincidir con Serial.begin en Arduino).
        """
        self.puerto = puerto
        self.baudrate = baudrate
        self.serial_conn = None
        
        try:
            # Conexión al puerto serial
            self.serial_conn = serial.Serial(self.puerto, self.baudrate, timeout=1)
            print(f"Conectado a {self.puerto} a {self.baudrate} baudios.")
            
            # Esperar a que el Arduino se reinicie tras la conexión serial
            time.sleep(2)
            
            if self.serial_conn is not None:
                # Leer cualquier mensaje inicial del Arduino ("Sistema Listo - Esperando MATLAB")
                while self.serial_conn.in_waiting > 0:
                    mensaje = self.serial_conn.readline().decode('utf-8').strip()
                if mensaje:
                    print(f"Arduino dice: {mensaje}")
                    
        except serial.SerialException as e:
            print(f"Error al conectar con el puerto {self.puerto}: {e}")

    def enviar_angulos(self, q1, q2, q3, q4, pinza=0):
        """
        Envía los 5 ángulos (grados) al Arduino separados por coma.
        :param q1: Ángulo base (0 a 180)
        :param q2: Ángulo hombro (0 a 180)
        :param q3: Ángulo codo (-90 a 90)
        :param q4: Ángulo muñeca (-90 a 90)
        :param pinza: Estado pinza (0 abierto, 1 cerrado)
        """
        if self.serial_conn is None or not self.serial_conn.is_open:
            print("Error: No hay conexión serial abierta.")
            return

        # Construir el string con valores separados por coma, finalizado con salto de línea.
        comando = f"{q1},{q2},{q3},{q4},{pinza}\n"
        
        try:
            # Enviar el comando al Arduino
            self.serial_conn.write(comando.encode('utf-8'))
            print(f"Enviando Comando: {comando.strip()}")
            
            if self.serial_conn is not None:    
                # Dar tiempo corto al Arduino para procesar y responder
                time.sleep(0.1)
                
                # Leer la respuesta del Arduino ("Recibido y Ejecutado: ...")
                while getattr(self.serial_conn, 'in_waiting', 0) > 0:
                    respuesta = self.serial_conn.readline().decode('utf-8').strip()
                if respuesta:
                    print(f"Respuesta Arduino: {respuesta}")
                    
        except Exception as e:
            print(f"Error al enviar datos: {e}")

    def cerrar_conexion(self):
        """Cierra la comunicación serial."""
        if self.serial_conn is not None and getattr(self.serial_conn, 'is_open', False):
            self.serial_conn.close()
            print("Conexión serial cerrada.")

# ===================== EJEMPLO DE USO =====================
if __name__ == "__main__":
    # Cambia 'COM3' por el puerto donde esté tu Arduino en Windows (o '/dev/ttyUSB0' en Linux)
    PUERTO_ARDUINO = 'COM3' 
    
    # Crear objeto de control del brazo
    brazo = BrazoRobotico(puerto=PUERTO_ARDUINO)
    
    # Asegúrate de que se pudo conectar antes de enviar comandos
    if brazo.serial_conn and brazo.serial_conn.is_open:
        try:
            # --- Mover a posición inicial (Home) ---
            print("\n--- Moviendo a posición inicial ---")
            # q1=90, q2=90, q3=0, q4=0, pinza=0 (abierta)
            brazo.enviar_angulos(90, 90, 0, 0, 0)
            time.sleep(2) # Espera a que termine el movimiento
            
            # --- Mover a nueva posición ---
            print("\n--- Moviendo articulaciones ---")
            # Ej: base 45, hombro 110, codo 45, muñeca -30, pinza=1 (cerrada)
            brazo.enviar_angulos(45, 110, 45, -30, 1)
            time.sleep(2)

            # --- Abrir pinza ---
            print("\n--- Abriendo pinza ---")
            brazo.enviar_angulos(45, 110, 45, -30, 0)
            time.sleep(1)

        finally:
            # Siempre cerrar conexión al terminar
            brazo.cerrar_conexion()
