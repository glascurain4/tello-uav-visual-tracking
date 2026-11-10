from djitellopy import Tello
import cv2
import numpy as np
import time

# VARIABLES GLOBALES
drone = Tello()
drone_flying = False
frame_reader = None

WINDOW_NAME = "Tello Camera"
TRACKBARS_NAME = "Trackbars"
MAX_HEIGHT = 300

# Umbrales de batería
BATTERY_MIN_TAKEOFF = 15
BATTERY_EMERGENCY_LAND = 10

# Variables para control de advertencias
last_battery_warning = 0
WARNING_COOLDOWN = 5  # 5s entre warnings

# Variables de detección de color
area_min = 2000
width = 800
height = 600
threshold_x = 25
threshold_y = 25

# Valores iniciales HSV (punto de partida, se calibran en vivo con las trackbars)
H_min_init = 0
H_max_init = 179
S_min_init = 94
S_max_init = 255
V_min_init = 97
V_max_init = 255

# Modo tracking
auto_follow = False
follow_speed_factor = 0.5


# UI - TRACKBARS
def cb(x):
    pass

cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WINDOW_NAME, 960, 720)
cv2.createTrackbar("Speed", WINDOW_NAME, 50, 100, cb)

cv2.namedWindow(TRACKBARS_NAME)
cv2.createTrackbar("H_min", TRACKBARS_NAME, H_min_init, 179, cb)
cv2.createTrackbar("H_max", TRACKBARS_NAME, H_max_init, 179, cb)
cv2.createTrackbar("S_min", TRACKBARS_NAME, S_min_init, 255, cb)
cv2.createTrackbar("S_max", TRACKBARS_NAME, S_max_init, 255, cb)
cv2.createTrackbar("V_min", TRACKBARS_NAME, V_min_init, 255, cb)
cv2.createTrackbar("V_max", TRACKBARS_NAME, V_max_init, 255, cb)


# Safety measures (takeoff y landing)
def safe_takeoff():
    global drone_flying

    if drone_flying:
        print("The drone is already flying.")
        return

    battery = drone.get_battery()
    if battery <= BATTERY_MIN_TAKEOFF:
        print(f"   WARNING: Cannot take off - Battery too low: {battery}%")
        print(f"   Minimum required: {BATTERY_MIN_TAKEOFF}%")
        return

    print(f"Battery level: {battery}% - Taking off...")
    drone.takeoff()
    drone_flying = True
    print("Airborne")


def safe_land():
    global drone_flying
    if drone_flying:
        print("Landing...")
        drone.land()
        drone_flying = False
    else:
        print("The drone is already on land.")


def emergency_land():
    global drone_flying
    if drone_flying:
        print(" EMERGENCY: Battery critical! Landing immediately...")
        drone.land()
        drone_flying = False
        print(" Emergency landing completed")


def check_battery_status():
    global last_battery_warning

    if not drone_flying:
        return True

    battery = drone.get_battery()
    current_time = time.time()

    if battery < BATTERY_EMERGENCY_LAND:
        emergency_land()
        return False

    elif battery <= BATTERY_MIN_TAKEOFF:
        if current_time - last_battery_warning > WARNING_COOLDOWN:
            print(f"  WARNING: Low battery ({battery}%) - Land soon!")
            last_battery_warning = current_time

    return True


def check_height_limit(up_down_command):
    if up_down_command <= 0:
        return up_down_command

    current_height = drone.get_height()

    if current_height >= MAX_HEIGHT:
        print(f"  WARNING: Max height reached ({current_height} cm) - Cannot ascend further")
        return 0

    return up_down_command


# MAIN
def main():
    global drone_flying, frame_reader, last_battery_warning, auto_follow

    try:
        print("Connecting to Tello...")
        drone.connect()

        battery = drone.get_battery()
        print(f"Battery: {battery}%")

        if battery <= BATTERY_MIN_TAKEOFF:
            print(f"  CAUTION: Battery is low ({battery}%) - Consider charging before flight")

        print("Starting video stream...")
        drone.streamon()
        frame_reader = drone.get_frame_read()

        print("\n" + "=" * 50)
        print("CONTROLS:")
        print("=" * 50)
        print("t -> takeoff")
        print("l -> land")
        print("q -> rotate left")
        print("e -> rotate right")
        print("r -> up")
        print("f -> down")
        print("w -> forward")
        print("s -> backward")
        print("a -> left")
        print("d -> right")
        print("m -> toggle auto-follow mode")
        print("p -> exit")
        print("=" * 50)

        while True:
            frame = frame_reader.frame

            if frame is None:
                continue

            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            frame = cv2.resize(frame, (width, height))

            # DETECCIÓN DE COLOR
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            h_min = cv2.getTrackbarPos("H_min", TRACKBARS_NAME)
            h_max = cv2.getTrackbarPos("H_max", TRACKBARS_NAME)
            s_min = cv2.getTrackbarPos("S_min", TRACKBARS_NAME)
            s_max = cv2.getTrackbarPos("S_max", TRACKBARS_NAME)
            v_min = cv2.getTrackbarPos("V_min", TRACKBARS_NAME)
            v_max = cv2.getTrackbarPos("V_max", TRACKBARS_NAME)

            hsv_min = np.array([h_min, s_min, v_min])
            hsv_max = np.array([h_max, s_max, v_max])

            mask = cv2.inRange(hsv_frame, hsv_min, hsv_max)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # centro y umbrales
            cx = width // 2
            cy_y = height // 2
            cv2.line(frame, (cx, 0), (cx, height), (255, 0, 0), 1)
            cv2.line(frame, (cx - threshold_x, 0), (cx - threshold_x, height), (255, 0, 0), 1)
            cv2.line(frame, (cx + threshold_x, 0), (cx + threshold_x, height), (255, 0, 0), 1)

            largest_contour = None
            largest_area = 0
            target_center_x = 0
            target_center_y = 0

            for contour in contours:
                area = cv2.contourArea(contour)

                if area > area_min:
                    x, y, w, h = cv2.boundingRect(contour)
                    center_x = x + w // 2
                    center_y = y + h // 2

                    # Dibujar rectángulo y centro
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                    cv2.circle(frame, (center_x, center_y), 7, (0, 0, 255), -1)

                    # Mostrar el area
                    cv2.putText(
                        frame,
                        f"Area: {int(area)}",
                        (x, y - 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2
                    )

                    # Mostrar posición relativa
                    if center_x > cx + threshold_x:
                        pos_text = "Derecha"
                    elif center_x < cx - threshold_x:
                        pos_text = "Izquierda"
                    else:
                        pos_text = "Centro"

                    cv2.putText(
                        frame,
                        pos_text,
                        (x, y - 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 0, 0),
                        2
                    )

                    # Guardar el contorno más grande para seguir
                    if area > largest_area:
                        largest_area = area
                        largest_contour = contour
                        target_center_x = center_x
                        target_center_y = center_y

            # UI TELEMETRIA
            battery = drone.get_battery()
            speed_value = cv2.getTrackbarPos("Speed", WINDOW_NAME)
            current_height = drone.get_height()

            # Color de batería
            if battery > BATTERY_MIN_TAKEOFF:
                battery_color = (0, 255, 0)
            elif battery > BATTERY_EMERGENCY_LAND:
                battery_color = (0, 255, 255)
            else:
                battery_color = (0, 0, 255)

            cv2.putText(frame, f"Battery: {battery}%", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, battery_color, 2)
            cv2.putText(frame, f"Speed: {speed_value}", (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

            status_text = "FLYING" if drone_flying else "LANDED"
            status_color = (0, 255, 0) if drone_flying else (0, 0, 255)
            cv2.putText(frame, f"Status: {status_text}", (10, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)

            height_color = (0, 255, 0) if current_height < MAX_HEIGHT else (0, 0, 255)
            cv2.putText(frame, f"Height: {current_height}cm / {MAX_HEIGHT}cm", (10, 135),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, height_color, 2)

            # Indicador de modo seguimiento
            mode_text = "AUTO-FOLLOW" if auto_follow else "MANUAL"
            mode_color = (0, 255, 255) if auto_follow else (255, 255, 255)
            cv2.putText(frame, f"Mode: {mode_text}", (10, 170),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, mode_color, 2)

            # Advertencia de batería baja
            if drone_flying and battery <= BATTERY_MIN_TAKEOFF:
                warning_text = "LOW BATTERY - LAND NOW!"
                cv2.putText(frame, warning_text,
                            (width // 2 - 200, height // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

            # Controles en pantalla
            cv2.putText(
                frame,
                "t:tkof l:land q/e:yaw r/f:up/dn w/s:fb a/d:lr m:auto p:exit",
                (10, height - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1
            )

            cv2.imshow(WINDOW_NAME, frame)
            cv2.imshow("Mask", mask)

            # CONTROL DE TELEOPERACION TECLADO
            key = cv2.waitKey(1) & 0xFF

            left_right = 0
            forward_backward = 0
            up_down = 0
            yaw = 0

            if key == ord('p'):
                print("Exit requested.")
                break

            elif key == ord('t'):
                safe_takeoff()

            elif key == ord('l'):
                safe_land()

            elif key == ord('m'):
                auto_follow = not auto_follow
                print(f"Auto-follow mode: {'ON' if auto_follow else 'OFF'}")

            if drone_flying:
                if not check_battery_status():
                    drone.send_rc_control(0, 0, 0, 0)
                    continue

                if auto_follow and largest_contour is not None:
                    # SEGUIMIENTO AUTOMÁTICO
                    error_x = target_center_x - cx

                    # Control de yaw para centrar el objeto
                    if error_x < -threshold_x:
                        yaw = -int(speed_value * follow_speed_factor)
                    elif error_x > threshold_x:
                        yaw = int(speed_value * follow_speed_factor)

                    # Ajusta target_area según el objeto y distancia deseada
                    target_area = 15000
                    if largest_area < target_area - 3000:
                        forward_backward = int(speed_value * 0.3)
                    elif largest_area > target_area + 3000:
                        forward_backward = -int(speed_value * 0.3)

                    # Control de altura para seguir el color
                    error_y = target_center_y - cy_y
                    if error_y < -threshold_y:
                        up_down = check_height_limit(int(speed_value * follow_speed_factor))
                    elif error_y > threshold_y:
                        up_down = -int(speed_value * follow_speed_factor)

                else:
                    # CONTROL MANUAL
                    if key == ord('a'):
                        left_right = -speed_value
                    elif key == ord('d'):
                        left_right = speed_value

                    if key == ord('w'):
                        forward_backward = speed_value
                    elif key == ord('s'):
                        forward_backward = -speed_value

                    if key == ord('r'):
                        up_down = check_height_limit(speed_value)
                    elif key == ord('f'):
                        up_down = -speed_value

                    if key == ord('q'):
                        yaw = -speed_value
                    elif key == ord('e'):
                        yaw = speed_value

                drone.send_rc_control(left_right, forward_backward, up_down, yaw)

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("Keyboard interruption detected.")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        print("Shutting down safely...")

        try:
            if drone_flying:
                print("Landing drone...")
                drone.land()
        except Exception as e:
            print(f"Could not land safely: {e}")

        try:
            drone.send_rc_control(0, 0, 0, 0)
        except:
            pass

        try:
            drone.streamoff()
        except:
            pass

        try:
            drone.end()
        except:
            pass

        cv2.destroyAllWindows()
        print("Program finished.")


if __name__ == "__main__":
    main()