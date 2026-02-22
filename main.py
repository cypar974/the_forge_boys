import cv2
import numpy as np
import time
import socket
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.parse

# Arduino IP (Update this with the IP shown in Serial Monitor)
ROBOT_IP = "172.20.10.12" 
UDP_PORT = 4210
last_instruction = "Waiting..."
last_send_time = 0
SEND_INTERVAL = 0.1 # Faster heartbeat for UDP
DASHBOARD_PORT = 8000

# Unified UDP sending helper
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_udp(msg):
    try:
        udp_sock.sendto(msg.encode(), (ROBOT_IP, UDP_PORT))
    except Exception as e:
        print(f"UDP Error: {e}")

# Recovery state
last_direction = "none" # "left", "right", or "none"


class DashboardHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass # Suppress server logs to keep console clean

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        query = urllib.parse.parse_qs(parsed.query)

        if parsed.path == "/":
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            self.wfile.write(self.get_html().encode())
            
        elif parsed.path == "/status":
            self.send_response(200)
            self.send_header("Content-type", "text/plain")
            self.end_headers()
            self.wfile.write(last_instruction.encode())

        elif parsed.path == "/drive":
            x = int(query.get("x", [0])[0])
            y = int(query.get("y", [0])[0])
            t = int(query.get("t", [100])[0])
            # Laptop handles the mix math to keep Arduino simple
            left = max(-100, min(100, y + x))
            right = max(-100, min(100, y - x))
            left = int((left * t) / 100)
            right = int((right * t) / 100)
            send_udp(f"drive:{left},{right}")
            self.send_response(200)
            self.end_headers()

        elif parsed.path == "/btn":
            # Direct relay of button strings if needed
            btn_id = query.get("id", [""])[0]
            send_udp(f"btn:{btn_id}")
            self.send_response(200)
            self.end_headers()

        else:
            self.send_response(404)
            self.end_headers()

    def get_html(self):
        # Using the same proven UI from the Arduino, pointed to localhost
        return """
        <!doctype html><html><head><meta charset='utf-8'/>
        <meta name='viewport' content='width=device-width,initial-scale=1'/>
        <title>Robot Dashboard (Offloaded)</title>
        <style>
          body{font-family:system-ui,sans-serif;margin:15px;background:#f0f2f5;color:#1c1e21}
          #wrap{max-width:480px;margin:0 auto;background:#fff;padding:20px;border-radius:20px;box-shadow:0 10px 25px rgba(0,0,0,0.1)}
          .row{margin:20px 0}
          h2{text-align:center;color:#1877f2;margin-bottom:25px}
          #joy{width:260px;height:260px;border:3px solid #e4e6eb;border-radius:50%;margin:0 auto;position:relative;background:#f7f8fa;touch-action:none}
          #stick{width:70px;height:70px;background:#1877f2;border-radius:50%;position:absolute;left:95px;top:95px;box-shadow:0 5px 15px rgba(24,119,242,0.4)}
          .thrRow{background:#f7f8fa;padding:20px;border-radius:15px;border:1px solid #e4e6eb}
          input[type=range]{width:100%;height:30px;cursor:pointer;accent-color:#1877f2}
          #status{font-size:12px;color:#65676b;font-family:monospace;margin-top:15px;text-align:center;background:#f7f8fa;padding:10px;border-radius:10px}
          #instructionRow{padding:20px; background:#e7f3ff; border-radius:15px; border:1px solid #bbdefb; text-align:center; min-height:80px; display:flex; flex-direction:column; justify-content:center}
          #instruction{font-size:28px; font-weight:800; color:#0d47a1; text-transform:uppercase; letter-spacing:1px}
        </style></head><body><div id='wrap'>
        <h2>Robot Control</h2>
        
        <div class='row' id='instructionRow'>
          <div style='font-size:12px; color:#1976d2; font-weight:bold; margin-bottom:5px; text-transform:uppercase'>Vision Command</div>
          <div id='instruction'>Waiting...</div>
        </div>

        <div class='row' style='display:flex; justify-content:center;'>
          <button id='autoBtn' style='width:100%; padding:20px; font-size:20px; font-weight:bold; border-radius:15px; border:none; background:#e4e6eb; color:#1c1e21; cursor:pointer; transition:all 0.3s;'>
            START AUTOPILOT
          </button>
        </div>

        <div class='row'><div id='joy'><div id='stick'></div></div></div>

        <div class='row thrRow'>
          <div style='display:flex; justify-content:space-between; margin-bottom:10px'>
            <label style='font-weight:bold'>Speed Limit</label>
            <span id='tval' style='color:#1877f2; font-weight:bold'>60%</span>
          </div>
          <input id='thr' type='range' min='0' max='100' value='60' step='1'/>
        </div>

        <div id='status'>Ready</div>

        <script>
        let x=0,y=0,t=60, auto=false;
        const joy=document.getElementById('joy'), stick=document.getElementById('stick');
        const thr=document.getElementById('thr'), tval=document.getElementById('tval'), status=document.getElementById('status');
        const autoBtn=document.getElementById('autoBtn');

        function clamp(v,a,b){return Math.max(a,Math.min(b,v));}
        function setStick(px,py){stick.style.left=(px-35)+'px'; stick.style.top=(py-35)+'px';}
        function updateStatus(extra=''){status.textContent=`X: ${x} | Y: ${y} | T: ${t}%` + (extra?' | '+extra:'');}

        let lastSend=0;
        function sendDrive(){
          const now=Date.now();
          if(now - lastSend < 50 && (x!==0 || y!==0)) return; // Rate limit but allow 0
          lastSend=now;
          fetch(`/drive?x=${x}&y=${y}&t=${t}`).catch(()=>{});
        }

        autoBtn.onclick=()=>{
          auto = !auto;
          if(auto){
            autoBtn.textContent = 'STOP AUTOPILOT';
            autoBtn.style.background = '#fb3958';
            autoBtn.style.color = '#fff';
          } else {
            autoBtn.textContent = 'START AUTOPILOT';
            autoBtn.style.background = '#e4e6eb';
            autoBtn.style.color = '#1c1e21';
          }
          fetch(`/btn?id=AUTO`).catch(()=>{});
        };

        function posToXY(cX,cY){
          const r=joy.getBoundingClientRect();
          const dx=cX-r.left-r.width/2, dy=cY-r.top-r.height/2;
          const max=r.width/2-35;
          const ndx=clamp(dx,-max,max), ndy=clamp(dy,-max,max);
          x=Math.round((ndx/max)*100); y=Math.round((-ndy/max)*100);
          if(Math.abs(x)<5)x=0; if(Math.abs(y)<5)y=0;
          setStick(130+ndx, 130+ndy); 
          
          if(x!==0 || y!==0) {
             // Disable visual autopilot button if manual control starts
             if(auto){ auto=false; autoBtn.textContent='START AUTOPILOT'; autoBtn.style.background='#e4e6eb'; autoBtn.style.color='#1c1e21'; }
          }

          updateStatus(); sendDrive();
        }

        let dragging=false;
        joy.onpointerdown=(e)=>{ dragging=true; joy.setPointerCapture(e.pointerId); posToXY(e.clientX,e.clientY); };
        joy.onpointermove=(e)=>{ if(dragging) posToXY(e.clientX,e.clientY); };
        joy.onpointerup=()=>{ dragging=false; x=0; y=0; setStick(130,130); updateStatus('Stopped'); sendDrive(); };

        thr.oninput=()=>{ t=thr.value; tval.textContent=t+'%'; sendDrive(); };

        function poll(){
          fetch('/status').then(r=>r.text()).then(txt=>{
            document.getElementById('instruction').textContent=txt;
          }).finally(()=>{ setTimeout(poll, 500); });
        }
        poll();
        </script>
        </div></body></html>
        """

def start_server():
    server = HTTPServer(('0.0.0.0', DASHBOARD_PORT), DashboardHandler)
    print(f"--- Dashboard hosted at http://localhost:{DASHBOARD_PORT} ---")
    server.serve_forever()

# Start the dashboard thread
threading.Thread(target=start_server, daemon=True).start()

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
cv2.createTrackbar("Mode", "Tuning", 1, 2, nothing)

cap = cv2.VideoCapture(1)

# Initialize CLAHE
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame")
        break
    
    # Rotate 90 degrees counter-clockwise
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    height, width = frame.shape[:2]
    
    # Preprocessing
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    v = clahe.apply(v)
    hsv_normalized = cv2.merge([h, s, v])
    
    if cv2.getWindowProperty("Tuning", cv2.WND_PROP_VISIBLE) < 1:
        break

    # Get tuning values
    l_h, l_s, l_v = cv2.getTrackbarPos("L-H", "Tuning"), cv2.getTrackbarPos("L-S", "Tuning"), cv2.getTrackbarPos("L-V", "Tuning")
    u_h, u_s, u_v = cv2.getTrackbarPos("U-H", "Tuning"), cv2.getTrackbarPos("U-S", "Tuning"), cv2.getTrackbarPos("U-V", "Tuning")
    mid_width, mode = cv2.getTrackbarPos("Mid Width", "Tuning"), cv2.getTrackbarPos("Mode", "Tuning")
    
    if mode == 1: # Red
        mask = cv2.bitwise_or(cv2.inRange(hsv_normalized, np.array([0, 100, 100]), np.array([10, 255, 255])),
                              cv2.inRange(hsv_normalized, np.array([160, 100, 100]), np.array([180, 255, 255])))
    elif mode == 2: # White
        mask = cv2.inRange(hsv_normalized, np.array([0, 0, 200]), np.array([180, 50, 255]))
    else: # Manual
        mask = cv2.inRange(hsv_normalized, np.array([l_h, l_s, l_v]), np.array([u_h, u_s, u_v]))
    
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    
    mid_x = width // 2
    left_bound, right_bound = mid_x - mid_width, mid_x + mid_width
    
    M_bottom = cv2.moments(mask[int(height*0.75):height, :])
    M_top = cv2.moments(mask[int(height*0.5):int(height*0.75), :])
    
    cx_bottom, cx_top = None, None
    if M_bottom["m00"] > 100:
        cx_bottom = int(M_bottom["m10"] / M_bottom["m00"])
        cv2.circle(frame, (cx_bottom, int(M_bottom["m01"] / M_bottom["m00"]) + int(height*0.75)), 8, (0, 0, 255), -1)
    if M_top["m00"] > 100:
        cx_top = int(M_top["m10"] / M_top["m00"])
        cv2.circle(frame, (cx_top, int(M_top["m01"] / M_top["m00"]) + int(height*0.5)), 8, (0, 255, 255), -1)
        
    target_msg = "searching"
    
    # New Anticipatory Logic:
    # If BOTH points are seen, we turn if EITHER is outside.
    # If only one is seen, we turn if it is outside.
    
    if cx_bottom is not None and cx_top is not None:
        # Both visible - if top is already turning, we should start turning early
        if cx_bottom < left_bound or cx_top < left_bound:
            target_msg = "go left"
            last_direction = "left"
        elif cx_bottom > right_bound or cx_top > right_bound:
            target_msg = "go right"
            last_direction = "right"
        else:
            target_msg = "good"
            
    elif cx_bottom is not None:
        # Only bottom visible
        if cx_bottom < left_bound:
            target_msg = "go left"
            last_direction = "left"
        elif cx_bottom > right_bound:
            target_msg = "go right"
            last_direction = "right"
        else:
            target_msg = "good"
            
    elif cx_top is not None:
        # Only top visible (anticipating a turn or recovery)
        if cx_top < left_bound:
            target_msg = "go left"
            last_direction = "left"
        elif cx_top > right_bound:
            target_msg = "go right"
            last_direction = "right"
        else:
            target_msg = "good"

    # Priority 3: Last known direction (recovery)
    elif last_direction != "none":
        target_msg = f"go {last_direction}"

            
    # UDP Dispatch (High Speed)

    current_time = time.time()
    if (target_msg != last_instruction) or (current_time - last_send_time > SEND_INTERVAL):
        send_udp(target_msg)
        last_instruction = target_msg
        last_send_time = current_time
    
    # UI
    cv2.line(frame, (left_bound, 0), (left_bound, height), (255, 0, 0), 2)
    cv2.line(frame, (right_bound, 0), (right_bound, height), (255, 0, 0), 2)
    cv2.putText(frame, target_msg, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Robot view", frame)
    cv2.imshow("Mask", mask)
    
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
