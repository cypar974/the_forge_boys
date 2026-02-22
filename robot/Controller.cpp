//
// Created by Oscar Tesniere on 11/02/2026.
//

#include "Controller.h"

Controller::Controller(const char* ssid, const char* password)
    : _ssid(ssid), _password(password) {}

bool Controller::registerButton(const char* label, void (*cb)()) {
    if (_buttonCount >= MAX_BUTTONS) return false;
    _buttons[_buttonCount].label = label;
    _buttons[_buttonCount].cb = cb;
    _buttonCount++;
    return true;
}

void Controller::clearButtons() {
    _buttonCount = 0;
}

void Controller::registerCallback(void (*callback)(const String&)) {
    _onMessage = callback;
}

void Controller::registerDriveCallback(void (*callback)(int8_t left, int8_t right)) {
    _onDrive = callback;
}

void Controller::setFailsafeTimeoutMs(uint16_t ms) {
    _failsafeTimeoutMs = ms;
}

void Controller::configureL298N(
    uint8_t ena, uint8_t in1, uint8_t in2,
    uint8_t enb, uint8_t in3, uint8_t in4
) {
    _l298nEnabled = true;
    _ena = ena; _in1 = in1; _in2 = in2;
    _enb = enb; _in3 = in3; _in4 = in4;
}

void Controller::setMotorDebugPrintIntervalMs(uint16_t ms) {
    _motorDebugPrintMs = ms;
}

bool Controller::beginAP(bool debug) {

    if (_ledEnabled) setLedStateHold(LED_BOOTING, 1500);
    _debug = debug;

    if (_l298nEnabled) {
        pinMode(_in1, OUTPUT); pinMode(_in2, OUTPUT);
        pinMode(_in3, OUTPUT); pinMode(_in4, OUTPUT);
        pinMode(_ena, OUTPUT); pinMode(_enb, OUTPUT);
        motorInitSafeStop();
    }

    if (wifiSSIDExistsNearby()) { // _debug &&
        Serial.print("[WiFi] NOTE: an AP with SSID already exists nearby: ");
        Serial.println(_ssid);
        if (_ledEnabled) setLedStateHold(LED_ERROR, 2000);
        // If you want to abort instead of just warn, uncomment:
        // return false;
    }

    String fv = WiFi.firmwareVersion();
    if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
        Serial.println("Warning: WiFi firmware may be outdated. Consider upgrading.");
        setLedStateForce(LED_ERROR);
        setLedStateHold(LED_ERROR, 1000);
    }

    Serial.print("Starting AP: ");
    Serial.println(_ssid);

    WiFi.config(IPAddress(10, 0, 0, 2));

    _status = WiFi.beginAP(_ssid, _password);
    if (_status != WL_AP_LISTENING) {
        if (debug) Serial.println("Failed to start AP");
        setLedState(LED_ERROR);
        return false;
    }

    _server.begin();
    _udp.begin(UDP_PORT);

    if (debug) {
        Serial.println("AP Mode Started");
        Serial.print("UDP listening on port: ");
        Serial.println(UDP_PORT);
        printWiFiStatus();
    }
    _lastDriveMs = millis();
    _failsafeStopped = false;

    Serial.println("AP mode started");
    printWiFiStatus();
    return true;
}

bool Controller::beginSTA(bool debug) {
    if (_ledEnabled) setLedStateHold(LED_BOOTING, 1500);
    _debug = debug;

    if (_l298nEnabled) {
        pinMode(_in1, OUTPUT); pinMode(_in2, OUTPUT);
        pinMode(_in3, OUTPUT); pinMode(_in4, OUTPUT);
        pinMode(_ena, OUTPUT); pinMode(_enb, OUTPUT);
        motorInitSafeStop();
    }

    Serial.println("  [Controller] Init WiFi...");
    Serial.print("  [Controller] Connecting to SSID: ");
    Serial.println(_ssid);

    _status = WiFi.begin(_ssid, _password);
    Serial.println("  [Controller] WiFi.begin() called. Waiting for link...");
    
    // Wait for connection
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
        delay(500);
        if (debug) Serial.print(".");
        if (_ledEnabled) {
            _ledLevel = !_ledLevel;
            digitalWrite(_ledPin, _ledLevel);
        }
    }

    if (WiFi.status() != WL_CONNECTED) {
        if (debug) Serial.println("\nFailed to connect to WiFi");
        setLedStateForce(LED_ERROR);
        return false;
    }

    if (debug) Serial.println("\nWiFi Connected!");
    setLedState(LED_AP_READY);

    _server.begin();
    _udp.begin(UDP_PORT);
    if (debug) {
        Serial.print("UDP listening on port: ");
        Serial.println(UDP_PORT);
    }

    _lastDriveMs = millis();
    _failsafeStopped = false;

    printWiFiStatus();
    return true;
}

void Controller::update() {
    processUDP();
    
    // Handle pending clients (up to 4 per loop to keep it responsive)
    for (int i = 0; i < 4; i++) {
        WiFiClient client = _server.available();
        if (!client) break;
        
        if (_debug) Serial.println("> [Server] New client");
        client.setTimeout(150); // Fast timeout
        handleClient(client);
        client.flush();
        client.stop(); // Explicitly stop after flush
    }
    delay(2); // Small yield for WiFi stack

    // Failsafe check
    const unsigned long now = millis();
    if (_failsafeTimeoutMs > 0 && (now - _lastDriveMs) > _failsafeTimeoutMs) {
        _failsafeStopped = true;
        setLedStateHold(LED_FAILSAFE, 1200);
    }

    // Apply smoothing and notify motors (also handles failsafe)
    applySmoothingAndNotify();
    updateStatusLED();   // update the LED status (if enabled)
}

void Controller::applySmoothingAndNotify() {
    auto applyDeadband = [&](int8_t v) -> int8_t {
        if (abs((int)v) < (int)_deadband) return 0;
        return v;
    };

    int8_t targetL = _failsafeStopped ? 0 : applyDeadband(_cmdLeft);
    int8_t targetR = _failsafeStopped ? 0 : applyDeadband(_cmdRight);

    auto stepToward = [&](int8_t cur, int8_t tgt) -> int8_t {
        int d = (int)tgt - (int)cur;

        // Use a bigger step when we are braking toward zero
        int step = (tgt == 0) ? (int)_slewPerUpdateStop : (int)_slewPerUpdate;

        if (d > step) d = step;
        if (d < -step) d = -step;
        return (int8_t)((int)cur + d);
    };

    int8_t newL = stepToward(_outLeft, targetL);
    int8_t newR = stepToward(_outRight, targetR);

    if (newL == _outLeft && newR == _outRight) return;

    _outLeft = newL;
    _outRight = newR;

    // Internal motor driver (if enabled)
    if (_l298nEnabled) {
        motorApply(_outLeft, _outRight);
    }

    // Optional external callback
    if (_onDrive) {
        _onDrive(_outLeft, _outRight);
    }
}

int8_t Controller::speedLeft() const { return _outLeft; }
int8_t Controller::speedRight() const { return _outRight; }

bool Controller::wifiSSIDExistsNearby() {
    int n = WiFi.scanNetworks();
    if (n < 0) return false;

    for (int i = 0; i < n; i++) {
        if (WiFi.SSID(i) == String(_ssid)) {
            return true;
        }
    }
    return false;
}

void Controller::debugWiFiScanForSSID()  {
    Serial.println("[WiFi] Scanning for nearby networks...");
    int n = WiFi.scanNetworks();
    if (n < 0) {
        Serial.println("[WiFi] scanNetworks() failed");
        return;
    }

    Serial.print("[WiFi] Found ");
    Serial.print(n);
    Serial.println(" networks:");

    bool foundSame = false;

    for (int i = 0; i < n; i++) {
        String s = WiFi.SSID(i);
        int32_t rssi = WiFi.RSSI(i);

        Serial.print("  - ");
        Serial.print(s);
        Serial.print("  RSSI=");
        Serial.println(rssi);

        if (s == String(_ssid)) foundSame = true;
    }

    if (foundSame) {
        Serial.print("[WiFi] WARNING: SSID already present nearby: ");
        setLedStateForce(LED_ERROR);
        setLedStateHold(LED_ERROR, 2000);
        Serial.println(_ssid);

    } else {
        Serial.print("[WiFi] OK: SSID not seen nearby: ");
        Serial.println(_ssid);
    }
}

void Controller::printWiFiStatus() const {
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    Serial.print("To control: http://");
    Serial.print(ip);
    Serial.println("/");
}
void Controller::setMotorMinPWM(uint8_t pwm) {
    _motorMinPWM = pwm;
}

String Controller::readRequestLine(WiFiClient& client) {
    unsigned long start = millis();
    while (client.connected() && !client.available()) {
        if (millis() - start > 150) return ""; // 150ms timeout for first line
        delay(1);
    }
    String line = client.readStringUntil('\n');
    line.trim();
    if (_debug && line.length() > 0) {
        Serial.print("  [Server] Req: ");
        Serial.println(line);
    }
    return line;
}

void Controller::sendHttpOk(WiFiClient& client, const char* contentType, const String& body) {
    String resp = "HTTP/1.1 200 OK\r\n";
    resp += "Content-Type: " + String(contentType) + "\r\n";
    resp += "Connection: close\r\n";
    resp += "Content-Length: " + String(body.length()) + "\r\n\r\n";
    resp += body;
    client.print(resp);
}

void Controller::sendHttpNotFound(WiFiClient& client) {
    const String body = "Not Found";
    client.println("HTTP/1.1 404 Not Found");
    client.println("Content-Type: text/plain; charset=utf-8");
    client.println("Connection: close");
    client.print("Content-Length: ");
    client.println(body.length());
    client.println();
    client.print(body);
}

int Controller::clampInt(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

bool Controller::extractQueryInt(const String& requestLine, const char* key, int& outValue) {
    int q = requestLine.indexOf('?');
    if (q < 0) return false;

    int end = requestLine.indexOf(' ', q);
    if (end < 0) return false;

    String query = requestLine.substring(q + 1, end);

    String k = String(key) + "=";
    int pos = query.indexOf(k);
    if (pos < 0) return false;

    int valStart = pos + k.length();
    int amp = query.indexOf('&', valStart);
    String valStr = (amp >= 0) ? query.substring(valStart, amp) : query.substring(valStart);

    valStr.replace("+", " ");
    outValue = valStr.toInt();
    return true;
}

void Controller::handleClient(WiFiClient& client) {
    unsigned long hStart = millis();
    String requestLine = readRequestLine(client);
    if (requestLine.length() == 0) return;

    // Fast drain headers: we only care about the GET line
    unsigned long start = millis();
    while (client.connected() && (millis() - start < 150)) {
        if (client.available()) {
            client.read(); // Just dump the byte, much faster than readStringUntil
        } else {
            delay(1);
        }
    }

    bool handled = true;
    if (requestLine.startsWith("GET / ") || requestLine.startsWith("GET /?")) {
        handleRoot(client);
        setLedStateHold(LED_CLIENT_CONNECTED, 2000);
    }
    else if (requestLine.startsWith("GET /drive")) {
        handleDrive(client, requestLine);
    }
    else if (requestLine.startsWith("GET /btn?")) {
        handleBtn(client, requestLine);
    }
    else if (requestLine.startsWith("GET /sld?")) {
        handleSlider(client, requestLine);
    }
    else if (requestLine.startsWith("GET /control?msg=")) {
        handleControlMsg(client, requestLine);
    }
    else if (requestLine.startsWith("GET /health")) {
        handleHealth(client);
    }
    else if (requestLine.startsWith("GET /status")) {
        handleStatus(client);
    }
    else if (requestLine.startsWith("GET /favicon.ico")) {
        sendHttpNotFound(client);
    }
    else {
        sendHttpNotFound(client);
        handled = false;
    }

    if (_debug) {
        if (!handled) Serial.println("  [Server] 404 Not Found");
        Serial.print("  [Server] Done in ");
        Serial.print(millis() - hStart);
        Serial.println("ms");
    }
}

void Controller::processUDP() {
    int packetSize;
    // Drain ALL waiting packets to get the freshest one
    while ((packetSize = _udp.parsePacket()) > 0) {
        char packetBuffer[64];
        int len = _udp.read(packetBuffer, 63);
        if (len > 0) {
            packetBuffer[len] = 0;
            String msg = String(packetBuffer);
            msg.trim();
            
            if (_onMessage) _onMessage(msg);
            _uiInstruction = msg;
            
            // Only debug the last one if we want to save serial bandwidth,
            // but for now let's keep it simple.
            if (_debug) {
                Serial.print("  [UDP] Recv: ");
                Serial.println(msg);
            }
        }
    }
}

void Controller::handleHealth(WiFiClient& client) {
    sendHttpOk(client, "text/plain; charset=utf-8", "OK");
}

void Controller::handleControlMsg(WiFiClient& client, const String& requestLine) {
    int start = String("GET /control?msg=").length();
    int end = requestLine.indexOf(' ', start);
    String msg = (end > start) ? requestLine.substring(start, end) : "";
    msg.replace("+", " ");

    if (_onMessage) _onMessage(msg);

    _uiInstruction = msg;

    sendHttpOk(client, "text/plain; charset=utf-8", "OK");
}

void Controller::handleStatus(WiFiClient& client) {
    sendHttpOk(client, "text/plain; charset=utf-8", _uiInstruction);
}

bool Controller::registerSlider(const char* label, void (*cb)(int value),
                                int minVal, int maxVal, int initial, int step) {
    if (_sliderCount >= MAX_SLIDERS) return false;
    if (minVal > maxVal) { int tmp = minVal; minVal = maxVal; maxVal = tmp; }
    if (step <= 0) step = 1;

    initial = clampInt(initial, minVal, maxVal);

    _sliders[_sliderCount].label  = label;
    _sliders[_sliderCount].minVal = minVal;
    _sliders[_sliderCount].maxVal = maxVal;
    _sliders[_sliderCount].step   = step;
    _sliders[_sliderCount].value  = initial;
    _sliders[_sliderCount].cb     = cb;

    _sliderCount++;
    return true;
}

void Controller::clearSliders() {
    _sliderCount = 0;
}

void Controller::handleBtn(WiFiClient& client, const String& requestLine) {
    int id = -1;
    if (!extractQueryInt(requestLine, "id", id)) {
        sendHttpOk(client, "text/plain; charset=utf-8", "Missing id");
        return;
    }

    if (id < 0 || id >= (int)_buttonCount) {
        sendHttpOk(client, "text/plain; charset=utf-8", "Bad id");
        return;
    }

    if (_buttons[id].cb) _buttons[id].cb();

    if (_onMessage) _onMessage(String("btn:") + _buttons[id].label);

    sendHttpOk(client, "text/plain; charset=utf-8", "OK");
}
void Controller::handleSlider(WiFiClient& client, const String& requestLine) {
    int id = -1;
    int v  = 0;

    if (!extractQueryInt(requestLine, "id", id)) {
        sendHttpOk(client, "text/plain; charset=utf-8", "Missing id");
        return;
    }
    if (!extractQueryInt(requestLine, "v", v)) {
        sendHttpOk(client, "text/plain; charset=utf-8", "Missing v");
        return;
    }

    if (id < 0 || id >= (int)_sliderCount) {
        sendHttpOk(client, "text/plain; charset=utf-8", "Bad id");
        return;
    }

    // Clamp + store
    v = clampInt(v, _sliders[id].minVal, _sliders[id].maxVal);
    _sliders[id].value = v;

    // Call callback
    if (_sliders[id].cb) _sliders[id].cb(v);

    // Optional message callback (consistent with buttons)
    if (_onMessage) _onMessage(String("sld:") + _sliders[id].label + "=" + String(v));

    sendHttpOk(client, "text/plain; charset=utf-8", "OK");
}

void Controller::handleDrive(WiFiClient& client, const String& requestLine) {
    int x = 0;   // -100..100
    int y = 0;   // -100..100
    int t = 100; // 0..100

    extractQueryInt(requestLine, "x", x);
    extractQueryInt(requestLine, "y", y);
    extractQueryInt(requestLine, "t", t);

    x = clampInt(x, -100, 100);
    y = clampInt(y, -100, 100);
    t = clampInt(t, 0, 100);

    int left  = clampInt(y + x, -100, 100);
    int right = clampInt(y - x, -100, 100);

    left  = (left * t) / 100;
    right = (right * t) / 100;

    _cmdLeft = (int8_t)left;
    _cmdRight = (int8_t)right;

    _lastDriveMs = millis();
    _failsafeStopped = false;

    setLedStateHold(LED_CLIENT_CONNECTED, 1000);

    // Optional debug prints (beware: will spam if heartbeat is enabled)
    // Serial.print("Drive: L="); Serial.print(_cmdLeft);
    // Serial.print(" R="); Serial.println(_cmdRight);

    sendHttpOk(client, "text/plain; charset=utf-8", "OK");
}

void Controller::enableStatusLED(uint8_t pin) {
    _ledPin = pin;
    _ledEnabled = true;
    pinMode(_ledPin, OUTPUT);
    digitalWrite(_ledPin, LOW);
}

void Controller::setLedState(Controller::LedState s) {
    if (!_ledEnabled) return;

    unsigned long now = millis();
    if (now < _ledHoldUntilMs) return;  // respect hold

    _ledState = s;
    _ledTimer = now;
}

void Controller::setLedStateHold(Controller::LedState s, uint16_t holdMs) {
    if (!_ledEnabled) return;

    unsigned long now = millis();
    _ledState = s;
    _ledTimer = now;
    _ledHoldUntilMs = now + (unsigned long)holdMs;
}

void Controller::setLedStateForce(Controller::LedState s) {
    if (!_ledEnabled) return;

    unsigned long now = millis();
    _ledHoldUntilMs = 0;      // clear hold
    _ledState = s;
    _ledTimer = now;
}

void Controller::updateStatusLED() {
    if (!_ledEnabled) return;

    unsigned long now = millis();

    switch (_ledState) {

        case LED_BOOTING:
            if (now - _ledTimer > 100) {
                _ledTimer = now;
                _ledLevel = !_ledLevel;
                digitalWrite(_ledPin, _ledLevel);
            }
            break;

        case LED_AP_READY:
            if (now - _ledTimer > 500) {
                _ledTimer = now;
                _ledLevel = !_ledLevel;
                digitalWrite(_ledPin, _ledLevel);
            }
            break;

        case LED_CLIENT_CONNECTED:
            digitalWrite(_ledPin, HIGH);
            break;

        case LED_FAILSAFE:
            // double blink pattern
            if (now - _ledTimer > 150) {
                _ledTimer = now;
                _ledLevel = !_ledLevel;
                digitalWrite(_ledPin, _ledLevel);
            }
            break;

        case LED_ERROR:
            if (now - _ledTimer > 70) {
                _ledTimer = now;
                _ledLevel = !_ledLevel;
                digitalWrite(_ledPin, _ledLevel);
            }
            break;
    }
}

void Controller::handleRoot(WiFiClient& client) {
    String buttonsHtml;
    for (uint8_t i = 0; i < _buttonCount; i++) {
        buttonsHtml += "<button class='uBtn' data-id='";
        buttonsHtml += i;
        buttonsHtml += "'>";
        buttonsHtml += _buttons[i].label;
        buttonsHtml += "</button> ";
    }
    if (_buttonCount == 0) {
        buttonsHtml = "<div style='opacity:.7'>No buttons registered</div>";
    }
    String slidersHtml;
    for (uint8_t i = 0; i < _sliderCount; i++) {
        slidersHtml += "<div class='row sldRow'>";
        slidersHtml += "  <div class='thrHeader'>";
        slidersHtml += "    <div class='thrLabel'>";
        slidersHtml += _sliders[i].label;
        slidersHtml += "    </div>";
        slidersHtml += "    <div class='thrValue'><span class='sldVal' data-id='";
        slidersHtml += i;
        slidersHtml += "'>";
        slidersHtml += _sliders[i].value;
        slidersHtml += "</span></div>";
        slidersHtml += "  </div>";
        slidersHtml += "  <input class='thr uSld' data-id='";
        slidersHtml += i;
        slidersHtml += "' type='range' min='";
        slidersHtml += _sliders[i].minVal;
        slidersHtml += "' max='";
        slidersHtml += _sliders[i].maxVal;
        slidersHtml += "' value='";
        slidersHtml += _sliders[i].value;
        slidersHtml += "' step='";
        slidersHtml += _sliders[i].step;
        slidersHtml += "'/>";
        slidersHtml += "</div>";
    }
    if (_sliderCount == 0) {
        slidersHtml = "<div class='row' style='opacity:.7'>No sliders registered</div>";
    }

    String page;
    page.reserve(7500);

    page += "<!doctype html><html><head><meta charset='utf-8'/>";
    page += "<meta name='viewport' content='width=device-width,initial-scale=1'/>";
    page += "<title>Robot Controller</title>";
    page += "<style>";
    page += "body{font-family:system-ui,sans-serif;margin:15px;background:#f8f9fa;color:#333}";
    page += "#wrap{max-width:480px;margin:0 auto}";
    page += ".row{margin:15px 0}";
    page += ".uBtn{padding:12px 18px;font-size:16px;border-radius:10px;border:1px solid #ddd;background:#fff;margin:0 5px 5px 0;cursor:pointer}";
    page += "#joy{width:240px;height:240px;border:2px solid #ccc;border-radius:50%;margin:0 auto;position:relative;background:#fff;touch-action:none}";
    page += "#stick{width:60px;height:60px;background:#333;border-radius:50%;position:absolute;left:90px;top:90px}";
    page += ".thrRow{background:#fff;padding:15px;border-radius:12px;border:1px solid #eee}";
    page += "input[type=range]{width:100%;height:30px;cursor:pointer}";
    page += "#status{font-size:11px;color:#999;font-family:monospace;margin-top:10px}";
    page += "</style></head><body><div id='wrap'>";
    page += "<h2>Robot Controller</h2>";

    page += "<div class='row' id='buttons'>";
    page += buttonsHtml;
    page += "</div>";

    page += "<div class='row' id='sliders'>";
    page += slidersHtml;
    page += "</div>";

    page += "<div class='row' id='instructionRow' style='padding:15px; background:#eef; border-radius:12px; border:1px solid #ccd; text-align:center;'>";
    page += "  <div style='font-size:14px; color:#666; margin-bottom:5px;'>Instruction</div>";
    page += "  <div id='instruction' style='font-size:24px; font-weight:bold; color:#333;'>Waiting...</div>";
    page += "</div>";

    page += "<div class='row'><div id='joy'><div id='stick'></div></div></div>";

    page += "<div class='row thrRow'>";
    page += "  <label>Throttle: <span id='tval'>100</span>%</label>";
    page += "  <input id='thr' type='range' min='0' max='100' value='100' step='1'/>";
    page += "</div>";

    page += "<div id='status' style='font-size:12px; color:#888; font-family:monospace; margin-top:10px;'></div>";

    // --- JS (STOP priority even if a request is in-flight) + HEARTBEAT resend ---
    page += "<script>";
    page += "let x=0,y=0,t=100;";
    page += "const joy=document.getElementById('joy');";
    page += "const stick=document.getElementById('stick');";
    page += "const thr=document.getElementById('thr');";
    page += "const tval=document.getElementById('tval');";
    page += "const status=document.getElementById('status');";

    page += "function clamp(v,a,b){return Math.max(a,Math.min(b,v));}";
    page += "function setStick(px,py){stick.style.left=(px-35)+'px'; stick.style.top=(py-35)+'px';}";
    page += "function updateStatus(extra=''){status.textContent=`x=${x} y=${y} t=${t}` + (extra?('\\n'+extra):'');}";

    // Dynamic buttons
    page += "document.querySelectorAll('.uBtn').forEach(b=>{";
    page += "  b.addEventListener('click',()=>{";
    page += "    const id=b.getAttribute('data-id');";
    page += "    fetch(`/btn?id=${id}&_=${Date.now()}`, {cache:'no-store'}).catch(()=>{});";
    page += "    updateStatus('btn id=' + id);";
    page += "  });";
    page += "});";

    // Dynamic sliders
    // --- Dynamic sliders (add-on sliders like buttons) ---
    page += "document.querySelectorAll('.uSld').forEach(s=>{";
    page += "  const id = s.getAttribute('data-id');";
    page += "  const vEl = document.querySelector(`.sldVal[data-id='${id}']`);";
    page += "  function sendSlider(){";
    page += "    const v = parseInt(s.value,10) || 0;";
    page += "    if (vEl) vEl.textContent = v;";
    page += "    fetch(`/sld?id=${id}&v=${v}&_=${Date.now()}`, {cache:'no-store'}).catch(()=>{});";
    page += "  }";
    page += "  s.addEventListener('input', ()=>{";
    page += "    sendSlider();";
    page += "  });";
    page += "});";

    // --- Drive send logic: 1 in-flight, STOP priority, + heartbeat keepalive ---
    page += "let inFlight=false;";
    page += "let pending=false;";
    page += "let lastSentX=999,lastSentY=999,lastSentT=999;";
    page += "let lastSendMs=0;";
    page += "const HEARTBEAT_MS=200;";

    page += "function sendDriveNow(force=false){";
    page += "  const now=Date.now();";
    page += "  const same = (x===lastSentX && y===lastSentY && t===lastSentT);";
    page += "  if (!force && same && (now - lastSendMs) < HEARTBEAT_MS) return;";
    page += "  const isStop = (x===0 && y===0);";

    // Keep your existing throttle rule, but allow heartbeat sends too
    page += "  if (inFlight && !isStop){ pending=true; return; }";
    page += "  if (!isStop){ inFlight=true; pending=false; }";

    page += "  const url=`/drive?x=${x}&y=${y}&t=${t}&_=${now}`;";
    page += "  lastSendMs=now;";

    page += "  fetch(url,{cache:'no-store', keepalive:true})";
    page += "    .catch(()=>{})";
    page += "    .finally(()=>{";
    page += "      lastSentX=x; lastSentY=y; lastSentT=t;";
    page += "      if (!isStop){";
    page += "        inFlight=false;";
    page += "        if (pending) sendDriveNow(true);";
    page += "      }";
    page += "    });";
    page += "}";

    // Heartbeat: keep sending while held away from center (prevents failsafe)
    page += "setInterval(()=>{";
    page += "  if (x!==0 || y!==0) sendDriveNow(false);";
    page += "}, HEARTBEAT_MS);";

    // Joystick mapping
    page += "function posToXY(clientX,clientY){";
    page += "  const r=joy.getBoundingClientRect();";
    page += "  const cx=clientX - r.left;";
    page += "  const cy=clientY - r.top;";
    page += "  const dx=cx - r.width/2;";
    page += "  const dy=cy - r.height/2;";
    page += "  const max=r.width/2 - 35;";
    page += "  const ndx=clamp(dx,-max,max);";
    page += "  const ndy=clamp(dy,-max,max);";
    page += "  x=Math.round((ndx/max)*100);";
    page += "  y=Math.round((-ndy/max)*100);";
    page += "  if (Math.abs(x) < 4) x=0;";
    page += "  if (Math.abs(y) < 4) y=0;";
    page += "  setStick(r.width/2 + ndx, r.height/2 + ndy);";
    page += "  updateStatus();";
    page += "  sendDriveNow(true);"; // force immediate send on changes
    page += "}";

    page += "let dragging=false;";
    page += "joy.addEventListener('pointerdown',(e)=>{";
    page += "  dragging=true;";
    page += "  joy.setPointerCapture(e.pointerId);";
    page += "  posToXY(e.clientX,e.clientY);";
    page += "});";
    page += "joy.addEventListener('pointermove',(e)=>{";
    page += "  if(!dragging) return;";
    page += "  posToXY(e.clientX,e.clientY);";
    page += "});";
    page += "joy.addEventListener('pointerup',()=>{";
    page += "  dragging=false;";
    page += "  x=0; y=0;";
    page += "  setStick(130,130);";
    page += "  updateStatus('released');";
    page += "  sendDriveNow(true);"; // force STOP send
    page += "});";
    page += "joy.addEventListener('pointercancel',()=>{";
    page += "  dragging=false;";
    page += "  x=0; y=0;";
    page += "  setStick(130,130);";
    page += "  updateStatus('cancel');";
    page += "  sendDriveNow(true);"; // force STOP send
    page += "});";

    // Slider
    page += "thr.addEventListener('input',()=>{";
    page += "  t=parseInt(thr.value,10)||0;";
    page += "  tval.textContent=t;";
    page += "  updateStatus('slider');";
    page += "  sendDriveNow(true);";
    page += "});";

    page += "updateStatus('ready');";
    page += "sendDriveNow(true);";

    // Polling for instructions (sequential to avoid stacking)
    page += "function pollStatus(){";
    page += "  fetch('/status?_='+Date.now()).then(r=>r.text()).then(t=>{";
    page += "    const el=document.getElementById('instruction');";
    page += "    if(el) el.textContent=t;";
    page += "  }).catch(()=>{})";
    page += "  .finally(()=>{ setTimeout(pollStatus, 2500); });";
    page += "}";
    page += "pollStatus();";

    page += "</script>";

    page += "</div></body></html>";

    sendHttpOk(client, "text/html; charset=utf-8", page);
}

// -------------------- L298N implementation --------------------

void Controller::motorInitSafeStop() {
    // Ensure stopped at boot (BRAKE)
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, HIGH);
    analogWrite(_ena, 0);

    digitalWrite(_in3, HIGH);
    digitalWrite(_in4, HIGH);
    analogWrite(_enb, 0);
}

void Controller::speedToCmd(int8_t spd, bool &forward, uint8_t &pwm) {
    int s = spd; // -100..100
    if (s >= 0) {
        forward = true;
    } else {
        forward = false;
        s = -s;
    }
    s = constrain(s, 0, 100);
    pwm = (uint8_t)map(s, 0, 100, 0, 255);
}

void Controller::setMotorOne(uint8_t en, uint8_t inA, uint8_t inB, int8_t spd) {
    int s = spd;
    if (s > 0) {
        digitalWrite(inA, HIGH);
        digitalWrite(inB, LOW);
    } else if (s < 0) {
        digitalWrite(inA, LOW);
        digitalWrite(inB, HIGH);
        s = -s;
    } else {
        // BRAKE (stops faster than coast)
        digitalWrite(inA, HIGH);
        digitalWrite(inB, HIGH);
        analogWrite(en, 0);
        return;
    }

    int pwm = map(constrain(s, 0, 100), 0, 100, 0, 255);
  // to prevent motor whining when starting from rest, we can enforce a minimum PWM threshold (tune as needed)
    if (pwm > 0 && pwm < _motorMinPWM) pwm = _motorMinPWM;
    analogWrite(en, pwm);
}

void Controller::debugMotors(int8_t left, int8_t right) {
    if (!_debug) return;

    const unsigned long now = millis();
    const bool changed = (left != _lastDbgL) || (right != _lastDbgR);
    const bool timeOk  = (now - _lastDbgPrintMs) >= _motorDebugPrintMs;

    if (!changed && !timeOk) return;

    bool lfwd, rfwd;
    uint8_t lpwm, rpwm;
    speedToCmd(left, lfwd, lpwm);
    speedToCmd(right, rfwd, rpwm);

    Serial.print("[MOTOR] L=");
    Serial.print(left);
    Serial.print(lfwd ? " FWD " : " REV ");
    Serial.print("PWM=");
    Serial.print(lpwm);

    Serial.print(" | R=");
    Serial.print(right);
    Serial.print(rfwd ? " FWD " : " REV ");
    Serial.print("PWM=");
    Serial.print(rpwm);

    _lastDbgL = left;
    _lastDbgR = right;
    _lastDbgPrintMs = now;
}

void Controller::motorApply(int8_t left, int8_t right) {
    debugMotors(left, right);
    setMotorOne(_ena, _in1, _in2, left);
    setMotorOne(_enb, _in3, _in4, right);
}
