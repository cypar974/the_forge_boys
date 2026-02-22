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

void Controller::setMotorMinPWM(uint8_t pwm) {
    _motorMinPWM = pwm;
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

    _udp.begin(UDP_PORT);

    if (debug) {
        Serial.println("AP Mode Started");
        Serial.print("UDP listening on port: ");
        Serial.println(UDP_PORT);
    }
    Serial.println("AP mode started");
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

    _udp.begin(UDP_PORT);
    if (debug) {
        Serial.print("UDP listening on port: ");
        Serial.println(UDP_PORT);
    }

    _lastDriveMs = millis();
    _failsafeStopped = false;

    return true;
}

void Controller::update() {
    processUDP();
    
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

void Controller::processUDP() {
    int packetSize;
    while ((packetSize = _udp.parsePacket()) > 0) {
        char packetBuffer[64];
        int len = _udp.read(packetBuffer, 63);
        if (len > 0) {
            packetBuffer[len] = 0;
            String msg = String(packetBuffer);
            msg.trim();
            
            if (msg.startsWith("drive:")) {
                // drive:L,R
                int comma = msg.indexOf(',');
                if (comma > 6) {
                    _cmdLeft = (int8_t)msg.substring(6, comma).toInt();
                    _cmdRight = (int8_t)msg.substring(comma + 1).toInt();
                    _lastDriveMs = millis();
                    _failsafeStopped = false;
                }
            } else {
                if (_onMessage) _onMessage(msg);
                _uiInstruction = msg;
            }
            
            if (_debug) {
                Serial.print("  [UDP] Recv: ");
                Serial.println(msg);
            }
        }
    }
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

// -------------------- L298N implementation --------------------

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

int Controller::clampInt(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
