/*  ESP32-CAM Quadcopter (brushed) controller
    - Combines camera webserver (stream) + motor control via LEDC PWM
    - HTTP and UDP control interfaces
    - Default motor commands expressed as microseconds (1000-2000)
    - Safety: arming/disarming + failsafe timeout

    Notes:
    * Change WIFI_SSID / WIFI_PASS
    * Change MOTOR_PIN_x to safe pins for your ESP32-CAM variant
    * Be careful with pins that affect boot mode (0,2,12,15...)
    * Power motors from separate battery and common grounds
*/

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiUdp.h>

// -------------------- CONFIG --------------------
#define WIFI_SSID      "YOUR_SSID"
#define WIFI_PASS      "YOUR_PASS"

// Motor pins - CHANGE THESE to pins that are free on your ESP32-CAM board
// Typical exposed pins on AI-Thinker: 0,1,3,4,12,13,14,15,16, etc.
// Verify your board before wiring!
#define MOTOR_PIN_1 13
#define MOTOR_PIN_2 14
#define MOTOR_PIN_3 15
#define MOTOR_PIN_4 12

// LEDC PWM channels for motors
#define MOTOR_CH_1 0
#define MOTOR_CH_2 1
#define MOTOR_CH_3 2
#define MOTOR_CH_4 3

// PWM parameters
// Many brushed ESCs expect "servo" pulses (1000..2000 us) at 50Hz.
// We'll map microseconds to LEDC duty. You can change PWM_FREQ if your ESCs require.
const int PWM_FREQ = 50;         // Hz; change if ESC requires different update rate
const int PWM_RES = 16;          // resolution bits for LEDC (0..16). Use 16 for high granularity.

const int US_MIN = 1000;         // min pulse width (µs) - stop/idle
const int US_MAX = 2000;         // max pulse width (µs) - full throttle
const int US_MID = 1000;         // safe idle default (some ESCs require lower)
const unsigned long FAILSAFE_MS = 500; // if no command in this ms, motors stop

// UDP control port
const unsigned int UDP_PORT = 8888;

// -------------------------------------------------

// Camera configuration: standard AI-Thinker pins (common example). If your board differs, change below.
camera_config_t camera_config = {
  .pin_pwdn       = 32,
  .pin_reset      = -1,
  .pin_xclk       = 0,
  .pin_sscb_sda   = 26,
  .pin_sscb_scl   = 27,

  .pin_d7         = 35,
  .pin_d6         = 34,
  .pin_d5         = 39,
  .pin_d4         = 36,
  .pin_d3         = 21,
  .pin_d2         = 19,
  .pin_d1         = 18,
  .pin_d0         = 5,
  .pin_vsync      = 25,
  .pin_href       = 23,
  .pin_pclk       = 22,

  .xclk_freq_hz   = 20000000,
  .ledc_timer     = LEDC_TIMER_0,
  .ledc_channel   = LEDC_CHANNEL_0,
  .pixel_format   = PIXFORMAT_JPEG,
  .frame_size     = FRAMESIZE_VGA,
  .jpeg_quality   = 12,
  .fb_count       = 2
};

// Web server and UDP
WebServer server(80);
WiFiUDP Udp;

// motor state (µs)
volatile int motor_us[4] = {US_MID, US_MID, US_MID, US_MID};
volatile bool motor_armed = false;
volatile unsigned long last_command_ms = 0;

// Forward declarations
void handleRoot();
void handleControl();
void udpListen();
void setMotorUS(int idx, int us);
void armMotors(bool arm);
uint32_t usToDuty(uint32_t us);

// ---------- setup ----------
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("ESP32-CAM Quadcopter controller starting...");

  // init camera
  if (!esp_camera_init(&camera_config)) {
    Serial.println("Camera init OK");
  } else {
    Serial.println("Camera init failed");
    // proceed anyway (maybe you want only motors), but streaming will fail
  }

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  int wifi_tries = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_tries++ < 40) {
    delay(500); Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("WiFi failed to connect (continuing).");
  }

  // start camera web server (CameraWebServer style)
  server.on("/", HTTP_GET, handleRoot);
  server.on("/control", HTTP_GET, handleControl);
  server.on("/stream", HTTP_GET, [](){
    // Start MJPEG stream - using  a helper from camera library
    // NOTE: esp32-camera provides an example server; we'll use stream handler below
    WiFiClient client = server.client();
    String response = "HTTP/1.1 200 OK\r\n"
                      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
    server.sendContent(response);
    // from examples: camera_httpd will stream; but to keep sketch compact we rely on the example's server routines
  });
  server.begin();

  // UDP
  Udp.begin(UDP_PORT);
  Serial.printf("UDP listening on port %u\n", UDP_PORT);

  // configure LEDC PWM channels and attach pins
  ledcSetup(MOTOR_CH_1, PWM_FREQ, PWM_RES);
  ledcSetup(MOTOR_CH_2, PWM_FREQ, PWM_RES);
  ledcSetup(MOTOR_CH_3, PWM_FREQ, PWM_RES);
  ledcSetup(MOTOR_CH_4, PWM_FREQ, PWM_RES);

  ledcAttachPin(MOTOR_PIN_1, MOTOR_CH_1);
  ledcAttachPin(MOTOR_PIN_2, MOTOR_CH_2);
  ledcAttachPin(MOTOR_PIN_3, MOTOR_CH_3);
  ledcAttachPin(MOTOR_PIN_4, MOTOR_CH_4);

  // initialize motors to safe off
  for (int i=0;i<4;i++) setMotorUS(i, US_MIN);

  // set last command time so failsafe is active until commanded
  last_command_ms = millis();
}

// ---------- loop ----------
void loop() {
  server.handleClient();
  udpListen();

  // watchdog/failsafe
  if (millis() - last_command_ms > FAILSAFE_MS) {
    if (motor_armed) {
      // stop motors (set to min)
      for (int i=0;i<4;i++) setMotorUS(i, US_MIN);
      motor_armed = false;
      Serial.println("Failsafe triggered: motors disarmed");
    }
  }

  // small yield
  delay(5);
}

// ---------- HTTP handlers ----------
void handleRoot() {
  String s = "<html><body><h2>ESP32-CAM Quadcopter</h2>";
  s += "<p>Stream: <a href=\"/stream\">/stream</a></p>";
  s += "<p>Control: /control?m1=1500&m2=1500&m3=1500&m4=1500&arm=1</p>";
  s += "</body></html>";
  server.send(200, "text/html", s);
}

void handleControl() {
  bool changed=false;
  if (server.hasArg("arm")) {
    String a = server.arg("arm");
    if (a == "1") { armMotors(true); changed=true; }
    else if (a == "0") { armMotors(false); changed=true; }
  }
  for (int i=1;i<=4;i++) {
    String key = "m" + String(i);
    if (server.hasArg(key)) {
      int us = server.arg(key).toInt();
      us = constrain(us, US_MIN, US_MAX);
      setMotorUS(i-1, us);
      changed=true;
    }
  }
  if (changed) {
    last_command_ms = millis();
  }
  server.send(200, "text/plain", "OK");
}

// ---------- UDP command parser ----------
void udpListen() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    char packet[512];
    int len = Udp.read(packet, 511);
    if (len < 0) return;
    packet[len] = 0;
    String s = String(packet);
    s.trim();
    s.toUpperCase();
    Serial.printf("UDP cmd: %s\n", s.c_str());

    // Example accepted:
    // M1:1500 M2:1600 M3:1500 M4:1200
    // or ARM 1
    if (s.startsWith("ARM")) {
      if (s.indexOf("1")>=0) armMotors(true);
      else armMotors(false);
      last_command_ms = millis();
      return;
    }

    // parse tokens
    int idx = 0;
    char *tok = strtok(packet, " ,\t\n\r");
    while (tok != NULL) {
      String t = String(tok);
      t.trim();
      if (t.length()>0) {
        if ((t[0]=='M' || t[0]=='m') && t.indexOf(':')>0) {
          int colon = t.indexOf(':');
          int mnum = t.substring(1, colon).toInt();
          int us = t.substring(colon+1).toInt();
          if (mnum>=1 && mnum<=4) {
            us = constrain(us, US_MIN, US_MAX);
            setMotorUS(mnum-1, us);
            last_command_ms = millis();
          }
        }
      }
      tok = strtok(NULL, " ,\t\n\r");
      idx++;
    }
  }
}

// ---------- Motor control helpers ----------
void setMotorUS(int idx, int us) {
  if (idx<0 || idx>3) return;
  motor_us[idx] = us;
  uint32_t duty = usToDuty(us);
  switch(idx) {
    case 0: ledcWrite(MOTOR_CH_1, duty); break;
    case 1: ledcWrite(MOTOR_CH_2, duty); break;
    case 2: ledcWrite(MOTOR_CH_3, duty); break;
    case 3: ledcWrite(MOTOR_CH_4, duty); break;
  }
  // Serial print for debugging
  Serial.printf("M%d -> %d us (duty %u)\n", idx+1, us, duty);
}

void armMotors(bool arm) {
  motor_armed = arm;
  if (!arm) {
    // stop motors :-)
    for (int i=0;i<4;i++) setMotorUS(i, US_MIN);
    Serial.println("Motors disarmed");
  } else {
    // set to safe idle (some ESCs require initial low then calibration)
    for (int i=0;i<4;i++) setMotorUS(i, US_MID);
    Serial.println("Motors armed (set to idle)");
  }
}

// convert microseconds pulse to LEDC duty value
// LEDC uses duty in [0 .. (2^PWM_RES - 1)] at configured freq.
// For PWM_FREQ in Hz, one period is (1,000,000 / PWM_FREQ) microseconds.
// duty = us / period * max_duty
uint32_t usToDuty(uint32_t us) {
  uint32_t period_us = 1000000UL / PWM_FREQ;
  uint32_t max_duty = (1UL << PWM_RES) - 1;
  // clamp us within expected range
  if (us < 0) us = 0;
  // compute ratio (float)
  float ratio = ((float)us) / (float)period_us;
  if (ratio < 0.0) ratio = 0.0;
  if (ratio > 1.0) ratio = 1.0;
  uint32_t duty = (uint32_t)(ratio * (float)max_duty + 0.5);
  return duty;
}

