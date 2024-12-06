import cv2
import time
from pymavlink import mavutil

# MAVLink
drone = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
drone.wait_heartbeat()
print('Конект з дроном')
cap = cv2.VideoCapture(0)
yaw = 0
if not cap.isOpened():
    print("Не відкрилась камера")
    exit(1)

tracker = cv2.legacy.TrackerCSRT_create()
tracking_initialized = False
channel = 10


def get_switch_state(channel, threshold=1500):
    try:
        message = drone.recv_match(type='RC_CHANNELS', blocking=True, timeout=0.2)

        if message:
            ch_key = f'chan{channel}_raw'
            if ch_key in message.__dict__:
                if message.__dict__[ch_key] > 1000 and message.__dict__[ch_key] < 1250:
                    return 'Reset'
                elif message.__dict__[ch_key] > 1250 and message.__dict__[ch_key] < 1700:
                    return 'target'
                elif message.__dict__[ch_key] > 1750:
                    return 'Control Operator'
            elif ch_key not in message.__dict__:
                print(f"Chanel {channel} not found in msg")
            else:
                return False
    except Exception as e:
        print("Помилка перевірки стану вимикача:", e)
        return None
    return False


def move_towards_target(forward_speed, vertical_speed,yaw):
    try:
        drone.mav.set_position_target_local_ned_send(
            0,
            drone.target_system,
            drone.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0, 0, 0,                            # Позиція (не використовується)
            forward_speed, 0, vertical_speed,  # Швидкість по осях
            0, 0, 0,                # Прискорення
            0, yaw)
    except Exception as e:
        print("Помилка відправки команди руху:", e)


# Основна програма
def main():
    global tracking_initialized
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Eror read camera")
            break

        screen_center_x = frame.shape[1] // 2
        screen_center_y = frame.shape[0] // 2

        # Відстеження цілі
        if tracking_initialized:
            success, bbox = tracker.update(frame)
            if success:
                bbox_center_x = bbox[0] + bbox[2] // 2
                bbox_center_y = bbox[1] + bbox[3] // 2

                #Коригування
                vertical_speed = 0
                if bbox_center_y < screen_center_y - 30:
                    vertical_speed = -5
                elif bbox_center_y > screen_center_y + 30:
                    vertical_speed = 5
                yaw = 0
                if bbox_center_x < screen_center_x - 30:
                    yaw = -5
                elif bbox_center_x > screen_center_x + 30:
                    yaw = 5

                forward_speed = 1  # Швидкість вперед
                move_towards_target(forward_speed, vertical_speed, yaw)

                #Малюємо квадрат
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                cv2.rectangle(frame, p1, p2, (0, 255, 0), 2, 1)
            else:
                cv2.putText(frame, "Vidstejenya Vtracheno", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

        cv2.imshow("Tracking", frame)

        get_switch_state = get_switch_state(channel)

        if get_switch_state == 'target':
            bbox = cv2.selectROI("Tracking", frame, fromCenter=False, showCrosshair=True)
            tracker.init(frame, bbox)
            tracking_initialized = True
        elif get_switch_state == 'Reset':
            tracking_initialized = False
        else:
             tracking_initialized = False
        time.sleep(0.03)

    cap.release()
    cv2.destroyAllWindows()
    drone.close()


if __name__ == "__main__":
    main()