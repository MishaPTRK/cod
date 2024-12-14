import cv2

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Не вдалося отримати доступ до камери.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Не вдалося зчитати кадр.")
        break
    cv2.imshow("Tracking", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or key == 27:
        break
cap.release()
cv2.destroyAllWindows()