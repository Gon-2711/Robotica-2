import os
import sys

# Prevención de crasheos para prints cuando usamos --windowed / --noconsole
if sys.stdout is None:
    sys.stdout = open(os.devnull, 'w')
if sys.stderr is None:
    sys.stderr = open(os.devnull, 'w')

import eel

def get_html_path():
    # Detectar si estamos corriendo dentro de PyInstaller o en desarrollo puro
    if hasattr(sys, '_MEIPASS'):
        return sys._MEIPASS
    return os.path.dirname(os.path.abspath(__file__))

if __name__ == '__main__':
    folder_path = get_html_path()
    
    # Inicializamos eel apuntando a la carpeta extraída
    eel.init(folder_path)
    
    # Arranca el bucle de la aplicación web usando Edge (preinstalado en Windows 11)
    try:
        eel.start('lagrange_euler_app.html', size=(1200, 800), mode='edge', port=0)
    except EnvironmentError:
        # Si también falla Edge, abre el navegador predeterminado como último recurso
        eel.start('lagrange_euler_app.html', size=(1200, 800), mode='default', port=0)