## 1. iVCam Setup (Camera)
1.  **Phone**: Open the **iVCam** app.
2.  **Computer**: Open the **iVCam** desktop app.
3.  **Connection**: Ensure your phone connects to your laptop (it should happen automatically if both are on the iPhone hotspot).
4.  **Settings**: Ensure the video is flowing to the iVCam desktop window.

## 2. iPhone Configuration (WiFi)
- **Personal Hotspot**: ON.
- **Maximize Compatibility**: **MUST BE ON** (This is critical for the Arduino UNO R4 to "see" the network).
- **SSID**: `iPhone de Cyprien`
- **Password**: `cypcyp974`

## 2. Arduino Setup
1. Open `robot/robot.ino` in the Arduino IDE.
2. Select the **Arduino UNO R4 WiFi** and the correct **COM Port**.
3. **Upload** the code.
4. Open the **Serial Monitor** and set the baud rate to **115200**.
5. Wait for the connection message, you might need to press the Reset button on the Arduino. It should print:
   `IP Address: 172.20.10.12` (If it doesn't print, press the Reset button on the Arduino).

## 3. Vision System Setup
1. Open `main.py`.
2. Ensure the `ROBOT_IP` variable matches the IP from the Serial Monitor (we found it is `172.20.10.12`):
   ```python
   ROBOT_IP = "172.20.10.12"
   ```
3. Run the script from your terminal or IDE.

## 4. Operation & Monitoring
- **Web UI**: Open [http://172.20.10.12/](http://172.20.10.12/) in your browser (phone or laptop).
- **Joystick**: You can manually drive the robot from this page.
- **Instruction Display**: The blue box at the top will show "go left" or "go right" live when `main.py` detects a color.
- **Vision**: `main.py` will automatically send these instructions to the display whenever it detects the target colors.
