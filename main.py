import cv2
import numpy as np
import urllib.request
import time

# Arduino IP (Update this with the IP shown in Serial Monitor)
ROBOT_IP = "172.20.10.12" 
last_instruction = ""
last_send_time = 0
SEND_INTERVAL = 0.5 # seconds

def nothing(x):
    pass

# Create a window for trackbars to tune the color and middle region
cv2.namedWindow("Tuning")
cv2.createTrackbar("L-H", "Tuning", 0, 179, nothing)
cv2.createTrackbar("L-S", "Tuning", 0, 255, nothing)
cv2.createTrackbar("L-V", "Tuning", 0, 255, nothing)
cv2.createTrackbar("U-H", "Tuning", 179, 179, nothing)
cv2.createTrackbar("U-S", "Tuning", 255, 255, nothing)
cv2.createTrackbar("U-V", "Tuning", 255, 255, nothing)
cv2.createTrackbar("Mid Width", "Tuning", 40, 300, nothing)
# Mode: 0=Manual, 1=Red, 2=White
cv2.createTrackbar("Mode", "Tuning", 0, 2, nothing)

cap = cv2.VideoCapture(1)

# Initialize CLAHE
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame")
        break
    
    # Rotate 90 degrees clockwise
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    height, width = frame.shape[:2]
    
    # 1. Preprocessing: Gaussian Blur to reduce noise
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    
    # 2. Lighting Normalization: Apply CLAHE on the V channel of HSV
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    v = clahe.apply(v)
    hsv_normalized = cv2.merge([h, s, v])
    
    # Get current trackbar positions
    l_h = cv2.getTrackbarPos("L-H", "Tuning")
    l_s = cv2.getTrackbarPos("L-S", "Tuning")
    l_v = cv2.getTrackbarPos("L-V", "Tuning")
    u_h = cv2.getTrackbarPos("U-H", "Tuning")
    u_s = cv2.getTrackbarPos("U-S", "Tuning")
    u_v = cv2.getTrackbarPos("U-V", "Tuning")
    mid_width = cv2.getTrackbarPos("Mid Width", "Tuning")
    mode = cv2.getTrackbarPos("Mode", "Tuning")
    
    # 3. Create mask based on mode
    if mode == 1: # Red Mode (Handles hue wrap-around)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv_normalized, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_normalized, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
    elif mode == 2: # White Mode (Low saturation, high value)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 50, 255])
        mask = cv2.inRange(hsv_normalized, lower_white, upper_white)
    else: # Manual Mode (Uses sliders)
        lower_color = np.array([l_h, l_s, l_v])
        upper_color = np.array([u_h, u_s, u_v])
        mask = cv2.inRange(hsv_normalized, lower_color, upper_color)
    
    # Noise reduction
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    
    # Define "middle" region
    mid_x = width // 2
    left_bound = mid_x - mid_width
    right_bound = mid_x + mid_width
    
    # 4. Line Segment Detection: Find centroids in two horizontal strips
    # We'll look at the bottom 25% and a strip from 25% to 50% of the frame
    bottom_strip = mask[int(height*0.75):height, :]
    top_strip = mask[int(height*0.5):int(height*0.75), :]
    
    M_bottom = cv2.moments(bottom_strip)
    M_top = cv2.moments(top_strip)
    
    cx_bottom, cy_bottom = None, None
    cx_top, cy_top = None, None
    
    if M_bottom["m00"] > 100:
        cx_bottom = int(M_bottom["m10"] / M_bottom["m00"])
        cy_bottom = int(M_bottom["m01"] / M_bottom["m00"]) + int(height*0.75)
        cv2.circle(frame, (cx_bottom, cy_bottom), 8, (0, 0, 255), -1)
        
    if M_top["m00"] > 100:
        cx_top = int(M_top["m10"] / M_top["m00"])
        cy_top = int(M_top["m01"] / M_top["m00"]) + int(height*0.5)
        cv2.circle(frame, (cx_top, cy_top), 8, (0, 255, 255), -1)
        
    # 5. Determine and Print Containment Status
    if cx_bottom is not None and cx_top is not None:
        # Draw the line segment
        cv2.line(frame, (cx_bottom, cy_bottom), (cx_top, cy_top), (0, 255, 0), 3)
        
        # Check if fully contained
        if (left_bound < cx_bottom < right_bound) and (left_bound < cx_top < right_bound):
            status = "Fully Contained"
            color = (0, 255, 0)
        else:
            status = "Partial/Outside"
            color = (0, 0, 255)
            
        print(status)
        cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
    else:
        # Fallback to general direction if one part is missing
        if cx_bottom is not None:
            if cx_bottom < left_bound:
                direction = "left"
            elif cx_bottom > right_bound:
                direction = "right"
            else:
                direction = "good"
            print(direction)
            cv2.putText(frame, f"Searching... {direction}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Send instruction to Arduino
            current_time = time.time()
            instruction = f"go {direction}" if direction in ["left", "right"] else direction
            
            if instruction != last_instruction or (current_time - last_send_time) > SEND_INTERVAL:
                try:
                    url = f"http://{ROBOT_IP}/control?msg={instruction.replace(' ', '+')}"
                    urllib.request.urlopen(url, timeout=0.1)
                    last_instruction = instruction
                    last_send_time = current_time
                except Exception as e:
                    pass
    
    # Handle "Fully Contained" case
    if 'status' in locals() and status == "Fully Contained":
        instruction = "good"
        current_time = time.time()
        if instruction != last_instruction or (current_time - last_send_time) > SEND_INTERVAL:
            try:
                url = f"http://{ROBOT_IP}/control?msg={instruction}"
                urllib.request.urlopen(url, timeout=0.1)
                last_instruction = instruction
                last_send_time = current_time
            except Exception as e:
                pass
    
    # Draw boundaries
    cv2.line(frame, (left_bound, 0), (left_bound, height), (255, 0, 0), 2)
    cv2.line(frame, (right_bound, 0), (right_bound, height), (255, 0, 0), 2)

    cv2.imshow("Robot view", frame)
    cv2.imshow("Mask", mask)
    
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
