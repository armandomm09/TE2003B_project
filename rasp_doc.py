import asyncio  
import sys  
import math 
import cv2  
import numpy as np  
import pyfiglet  
from simple_pid import PID 
from termcolor import colored  
import websockets 
import serial 
import json 
import time 
import os 

PC_SERVER_IP = "10.50.94.90"  # IP del servidor PC
PC_SERVER_PORT = 8000  # Puerto del servidor PC
WEBSOCKET_URI = f"ws://{PC_SERVER_IP}:{PC_SERVER_PORT}/ws/control"  # URI para conectar vía WebSocket para manejo manual

SERIAL_PORT = os.getenv("SERIAL_PORT")  # Puerto serial (ej. "/dev/ttyUSB0")
BAUD_RATE = 9600  # Velocidad de baudios para serial
RECONNECT_DELAY = 5  # Retardo antes de reintentar conexión serial
SERIAL_READ_TIMEOUT = 0.1  # Tiempo de espera para lectura serial

OUT_MIN_VALUE = 8  # Valor mínimo de salida para PID
OUT_MAX_VALUE = 70  # Valor máximo de salida para PID

KP = float(os.getenv("PIDP", "0"))  # kP del PID
KI, KD = 0, 0  # k integral y derivativa del PID
BASE_SPEED = float(os.getenv("SPEED", "0"))  # Velocidad base para el robot
pid_controller = PID(KP, KI, KD)  # Instancia del controlador PID

ser = None  # Variable global para el objeto serial
stop_serial_reader = asyncio.Event()  # Evento para detener el lector serial

ARDUINO_PRINT_INTERVAL = 1.0  # Intervalo en segundos para imprimir lecturas de Arduino

def get_steering_angle_and_pid_output(img, pid_instance, show_visualization=False):
    # Obtiene ángulo de dirección y salida PID a partir de la imagen
    img_bgr_original = img.copy()  # Copia de la imagen original 
    height, width, _ = img_bgr_original.shape  # Dimensiones de la imagen

    img_gray = cv2.cvtColor(img_bgr_original, cv2.COLOR_BGR2GRAY)  # Convierte a escala de grises
    img_blur = cv2.GaussianBlur(img_gray, (5, 5), 0)  # Gaussian Blur
    img_canny = cv2.Canny(img_blur, 50, 150)  # Detección de bordes con Canny

    
    roi_vertices = np.array([[(0, 479), (150, 160), (640 - 150, 160), (639, 479)]], dtype=np.int32)  # ROI específica para 640x480

    roi_mask = np.zeros_like(img_gray)  # Máscara inicialmente vacía
    cv2.fillPoly(roi_mask, roi_vertices, 255)  # Rellena la ROI en la máscara
    img_masked_canny = cv2.bitwise_and(img_canny, roi_mask)  # Aplica la máscara a la imagen de bordes

    lines = cv2.HoughLinesP(
        img_masked_canny,
        rho=1,
        theta=np.pi / 180,
        threshold=30,
        minLineLength=30,
        maxLineGap=20
    )  # Deteccion de lineas

    if show_visualization: # En caso de que queramos ver en tiempo real lo que sucede
        img_lines_viz = img_bgr_original.copy()  # Imagen para visualizar líneas
        cv2.polylines(img_lines_viz, [roi_vertices], isClosed=True, color=(0, 255, 255), thickness=1)  # Dibuja ROI

    if lines is None:
        if show_visualization:
            cv2.imshow("Line Detection Debug", img_lines_viz if 'img_lines_viz' in locals() else img_masked_canny)  # Muestra debug si no hay líneas
        pid_output = pid_instance(0)  # PID con error 0 si no hay líneas
        return 0, pid_output  # Retorna cero como ángulo y salida del PID

    line_angles_degrees = []  # Lista para ángulos de errores
    valid_lines_for_angle_calc = []  # Líneas válidas para cálculo de ángulo

    for line_segment in lines:
        x1, y1, x2, y2 = line_segment[0]  # Coordenadas del segmento de línea

        delta_y = abs(y2 - y1)  # Diferencia vertical
        delta_x = abs(x2 - x1)  # Diferencia horizontal
        if delta_x == 0:
            angle_from_vertical_rad = 0  # Línea vertical sin cálculo de ángulo horizontal
        elif delta_y < delta_x * 0.2:
            if show_visualization:
                cv2.line(img_lines_viz, (x1, y1), (x2, y2), (100, 100, 100), 1)  # Dibuja líneas muy horizontales en gris
            continue  # Descarta líneas casi horizontales

        angle_rad_horizontal = math.atan2(y2 - y1, x2 - x1)  # Ángulo con respecto al eje horizontal
        img_coord_angle_deg = math.degrees(angle_rad_horizontal)  # Convierte a grados

        if x2 == x1:
            angle_from_vertical_deg = 0  # Línea perfectamente vertical -> ángulo de error 0
        else:
            # En caso de que p2 este mas arriba de p1, invertimos
            if y2 < y1:
                angle_deg = math.degrees(math.atan2(y1 - y2, x2 - x1))  # Cálculo de ángulo si y2 < y1
                error_angle_deg = angle_deg - 90  # Desviación respecto a vertical
            elif y1 < y2:
                angle_deg = math.degrees(math.atan2(y2 - y1, x1 - x2))  # Cálculo de ángulo si y1 < y2
                error_angle_deg = angle_deg - 90  # Desviación respecto a vertical
            else:
                continue  # Omitir si no hay cambio vertical

            line_angles_degrees.append(error_angle_deg)  # Agrega ángulo de error
            valid_lines_for_angle_calc.append(line_segment[0])  # Agrega línea válida
            if show_visualization:
                cv2.line(img_lines_viz, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Dibuja líneas válidas en verde

    if not line_angles_degrees:
        if show_visualization:
            cv2.imshow("Line Detection Debug", img_lines_viz)  # Muestra debug si no se calcularon ángulos
        pid_output = pid_instance(0)  # PID con error 0 si no hay ángulos
        return 0, pid_output  # Retorna cero como ángulo y salida del PID

    average_steering_error_deg = np.mean(line_angles_degrees)  # Promedio de ángulos de error
    pid_control_output = pid_instance(average_steering_error_deg)  # Salida del PID con error promedio

    if show_visualization:
        cv2.putText(img_lines_viz, f"Error Angle: {average_steering_error_deg:.2f} deg", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)  # Muestra texto de error de ángulo
        cv2.putText(img_lines_viz, f"PID Output: {pid_control_output:.2f}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)  # Muestra texto de salida PID
        cv2.imshow("Line Detection Debug", img_lines_viz)  # Muestra imagen con líneas y texto

    return average_steering_error_deg, pid_control_output  # Retorna ángulo y salida PID

def setup_serial():
    # Configura el puerto serial
    global ser
    try:
        if ser and ser.is_open:
            ser.close()  # Cierra puerto serial si ya estaba abierto
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_READ_TIMEOUT)  # Abre puerto serial
        ser.reset_input_buffer()  # Limpia buffer de entrada
        ser.reset_output_buffer()  # Limpia buffer de salida
        return True  # Indica que la apertura fue exitosa
    except Exception as e:
        print(f"Error abriendo serial {SERIAL_PORT}: {e}")  # Muestra error en apertura serial
        ser = None  # Resetea la variable serial
        return False  # Indica que la apertura falló

async def send_xyz_to_mbot_serial(x_val, y_val, z_val):
    # Envía valores X, Y, Z al mBot vía serial
    global ser
    if not ser or not ser.is_open:
        if not setup_serial():
            return  # Si falla configurar serial, retorna

    command_to_mbot = f"{int(x_val)},{int(y_val)},{int(z_val)}\n"  # Formatea comando para mBot
    try:
        ser.write(command_to_mbot.encode("utf-8"))  # Envía comando al serial
    except serial.SerialException as e:
        print(f"Error de comunicación serial al escribir: {e}")  # Maneja error de escritura serial
        if ser:
            ser.close()  # Cierra puerto serial en caso de error
        ser = None  # Resetea variable serial
    except Exception as e:
        print(f"Error inesperado al escribir en serial: {e}")  # Captura errores inesperados

async def read_from_serial():
    # Lee datos continuamente desde el puerto serial
    global ser
    last_print_time = time.time()  # Marca de tiempo de última impresión
    print("Lector serial iniciado.")  # Mensaje de inicio de lector serial

    while not stop_serial_reader.is_set():
        if ser and ser.is_open:
            try:
                if ser.in_waiting > 0:
                    line = ser.readline()  # Lee una línea desde serial
                    if line:
                        try:
                            decoded_line = line.decode("utf-8").rstrip()  # Decodifica y limpia saltos de línea
                            if decoded_line:
                                now = time.time()  # Tiempo actual
                                print(f"Arduino: {decoded_line}")  # Imprime datos de Arduino
                                last_print_time = now  # Actualiza tiempo de impresión
                        except UnicodeDecodeError:
                            now = time.time()  # Tiempo actual
                            # if now - last_print_time >= ARDUINO_PRINT_INTERVAL:
                            print(f"Arduino : {line}")  # Imprime datos crudos en caso de error de decodificación
                            last_print_time = now  # Actualiza tiempo de impresión
                else:
                    await asyncio.sleep(0.01)  # Espera breve si no hay datos
            except serial.SerialException as e:
                print(f"Error leyendo del serial: {e}. El puerto puede haberse cerrado.")  # Maneja cierre serial imprevisto
                if ser:
                    ser.close()  # Cierra puerto serial en caso de error
                ser = None  # Resetea variable serial
                await asyncio.sleep(1)  # Espera antes de intentar de nuevo
            except Exception as e:
                print(f"Error inesperado en lector serial: {e}")  # Captura errores inesperados en lectura
                await asyncio.sleep(1)  # Espera antes de intentar de nuevo
        else:
            await asyncio.sleep(1)  # Espera breve si serial no está abierto

    print("Lector serial detenido.")  # Mensaje de detención del lector serial

async def mbot_control_client():
    # WebSocket para recibir comandos de control del mBot
    global ser
    try:
        async with websockets.connect(WEBSOCKET_URI) as websocket:  # Conecta al servidor WebSocket
            
            async for message in websocket:  # Recorre mensajes entrantes
                if stop_serial_reader.is_set():
                    break  # Sale si se solicita detener el lector serial
                try:
                    data = json.loads(message)  # Parsea mensaje JSON
                    x_val = data.get("x", 0)  # Obtiene valor X del JSON
                    y_val = data.get("y", 0)  # Obtiene valor Y del JSON
                    z_val = data.get("z", 0)  # Obtiene valor Z del JSON

                    await send_xyz_to_mbot_serial(y_val, x_val, z_val)  # Envía valores al mBot (Y->X, X->Y, Z)

                except json.JSONDecodeError:
                    print(f"Error: Mensaje no es JSON válido: {message}")  # Maneja JSON inválido
                except Exception as e:
                    print(f"Error procesando mensaje WebSocket o enviando a mBot: {e}")  # Captura errores inesperados
    except (websockets.exceptions.ConnectionClosedError, websockets.exceptions.ConnectionClosedOK):
        pass  # Ignora cierre normal o erróneo de WebSocket
    except ConnectionRefusedError:
        print(f"Conexión WebSocket rechazada por {PC_SERVER_IP}:{PC_SERVER_PORT}.")  # Maneja rechazo de conexión
    except asyncio.CancelledError:
        pass  # Permite cancelación asincrónica
    except Exception as e:
        print(f"Error en WebSocket o de conexión principal: {e}")  # Captura errores generales en WebSocket

async def send_pid_to_mbot(cap):
    # Lee frames de cámara y envía control PID al mBot
    global ser
    print("Iniciando control PID...")  # Mensaje de inicio de control PID
    try:
        while not stop_serial_reader.is_set():
            ret, frame = cap.read()  # Captura un frame de la cámara
            if not ret:
                print("Error: No se pudo leer el frame de la cámara.")  # Maneja error en captura de frame
                await asyncio.sleep(0.1)  # Espera breve antes de intentar de nuevo
                continue

            drive_angle, pid_out = get_steering_angle_and_pid_output(frame, pid_controller)  # Calcula ángulo y PID

            if pid_out > 0:
                if pid_out < OUT_MIN_VALUE:
                    out = 0  # Si el output esta muy pequeño, salida 0
                elif pid_out > OUT_MAX_VALUE:
                    out = OUT_MAX_VALUE  # Si el output excede el máximo, se limita
                else:
                    out = int(pid_out)  # Conversión a entero de PID positivo
            elif pid_out < 0:
                if pid_out > -OUT_MIN_VALUE:
                    out = 0  # Si PID negativo muy pequeño, salida 0
                elif pid_out < -OUT_MAX_VALUE:
                    out = -OUT_MAX_VALUE  # Si PID negativo excede máximo, se limita
                else:
                    out = int(pid_out)  # Conversión a entero de PID negativo
            else:
                out = 0  # Si PID es cero, salida cero

            await send_xyz_to_mbot_serial(0, BASE_SPEED, out)  # Envía comando con velocidad base y salida PID
            await asyncio.sleep(0.05)  # Pausa breve entre envíos

    except asyncio.CancelledError:
        pass  # Permite cancelación asincrónica
    finally:
        print("Finalizando send_pid_to_mbot.")  # Mensaje de finalización del envío PID

async def main():
    # Función principal que arranca tareas según argumentos
    global ser
    serial_task = None  # Tarea para lector serial

    setup_serial()  # Intenta abrir puerto serial

    if ser and ser.is_open:
        serial_task = asyncio.create_task(read_from_serial())  # Inicia tarea de lectura serial
    else:
        print("Advertencia: Puerto serial no se abrió inicialmente.")  # Advierte si no se abrió el serial

    main_task = None  # Tarea principal (WebSocket o PID)
    try:
        if len(sys.argv) > 1 and sys.argv[1] == "drive":
            main_task = asyncio.create_task(mbot_control_client())  # Inicia tarea de control por WebSocket
            await main_task  # Espera a que termine la tarea

        elif len(sys.argv) > 1 and sys.argv[1] == "pid":
            cap = None  # Inicializa objeto de captura de cámara
            try:
                cap = cv2.VideoCapture(0)  # Abre la cámara por defecto
                if not cap.isOpened():
                    print("Error Fatal: No se pudo abrir la cámara.")  # Maneja error si cámara no se abre
                    return
                main_task = asyncio.create_task(send_pid_to_mbot(cap))  # Inicia tarea de control PID
                await main_task  # Espera a que termine la tarea
            finally:
                if cap:
                    cap.release()  # Libera la cámara al finalizar
        else:
            print("Uso: python tu_script.py [drive | <modo_pid>]")  # Instrucciones de uso si no hay argumentos
            print("Ejemplos:")
            print("  python tu_script.py drive")
            print("  python tu_script.py follow_line")

    except KeyboardInterrupt:
        if main_task and not main_task.done():
            main_task.cancel()  # Cancela tarea principal al presionar Ctrl+C
            if serial_task and not serial_task.done():
                stop_serial_reader.set()  # Señala detener lector serial
                serial_task.cancel()  # Cancela tarea de lectura serial

    except Exception as e:
        print(f"Error inesperado en la ejecución principal: {e}")  # Captura errores generales en main
    finally:
        stop_serial_reader.set()  # Señala detener lector serial
        if main_task and not main_task.done():
            main_task.cancel()  # Cancela tarea principal si sigue activa
            try:
                await main_task
            except asyncio.CancelledError:
                pass  # Ignora excepción de cancelación

        if serial_task:
            if not serial_task.done():
                try:
                    await asyncio.wait_for(serial_task, timeout=1.0)  # Espera breve a que termine lector serial
                except asyncio.TimeoutError:
                    serial_task.cancel()  # Cancela lector serial si no responde
                    try:
                        await serial_task
                    except asyncio.CancelledError:
                        pass  # Ignora excepción de cancelación

        if ser and ser.is_open:
            try:
                ser.write(b"0,0,0\n")  # Envía comando de parada al mBot
                time.sleep(0.1)  # Pausa breve antes de cerrar
                ser.close()  # Cierra puerto serial al finalizar
            except Exception as e_serial_final:
                print(f"Error en el cierre final del puerto serial: {e_serial_final}")  # Maneja error al cerrar serial

if __name__ == "__main__":
    try:
        ASCII_BOTICS = pyfiglet.figlet_format("BOTICS")  # Genera texto ASCII para bienvenida
        colored_ascii = colored(ASCII_BOTICS, color="blue")  # Colorea el texto ASCII en azul
        print(colored_ascii)  # Imprime arte ASCII en pantalla
        asyncio.run(main())  # Ejecuta la función principal asíncrona
    except KeyboardInterrupt:
        print("\nSalida forzada del programa.")  # Mensaje si se interrumpe con Ctrl+C
    except Exception as e:
        print(f"Excepción no controlada a nivel de asyncio.run: {e}")  # Captura errores no controlados en asyncio.run
