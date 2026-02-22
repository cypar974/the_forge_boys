import cv2

for i in range(3): # Check indices 0, 1, and 2
    cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            print(f"Success! Camera found at index {i}")
            cap.release()
            break
        cap.release()