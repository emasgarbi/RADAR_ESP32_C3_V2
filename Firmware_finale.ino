#include <Arduino.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <HardwareSerial.h>
#include <stdint.h>
#include <Preferences.h>  // For non-volatile storage (survives power cycles)
#include "hungarian.h"

// =================================================================
// PERSISTENT CALIBRATION STORAGE
// =================================================================
// Preferences object for non-volatile storage (survives power down)
Preferences preferences;

// Calibration data structure - all parameters that need to persist
struct CalibrationData {
  // X-axis offset correction
  float x_offset;

  // Y-axis global offset (added to ALL measurements before zone corrections)
  float y_global_offset;

  // Y-axis calibration zones (piecewise linear correction)
  // Zone 1: 0-100cm (ground level)
  float zone1_max;          // 100.0 cm
  float zone1_correction;   // 0.0 (no correction)

  // Zone 2: 100-250cm (low-mid range)
  float zone2_max;          // 250.0 cm
  float zone2_base;         // 100.0 cm (starting point)
  float zone2_multiplier;   // 0.1 (slope)

  // Zone 3: 250-320cm (mid range)
  float zone3_max;          // 320.0 cm
  float zone3_base_corr;    // 15.0 cm (base correction)
  float zone3_base_y;       // 250.0 cm (starting Y)
  float zone3_multiplier;   // 0.64 (slope)

  // Zone 4: >320cm (high range)
  float zone4_transition;   // 380.0 cm (transition point)
  float zone4_base_corr;    // -30.0 cm (base correction)
  float zone4_base_y;       // 320.0 cm (starting Y)
  float zone4_multiplier;   // 0.5 (slope for 320-380 range)

  // Validation flag and version
  bool valid;               // true if calibration data is valid
  uint8_t version;          // calibration structure version
};

// Global calibration data - loaded from flash on startup
CalibrationData calibration;

// Default calibration values (factory defaults)
const CalibrationData DEFAULT_CALIBRATION = {
  .x_offset = 0.0,
  .y_global_offset = 100.0,  // Aumentato a 100cm - se vedi 120cm quando persona è a 180cm, prova 100-120cm

  .zone1_max = 180.0,        // Esteso zona 1 fino a 180cm
  .zone1_correction = 0.0,   // Nessuna correzione aggiuntiva in zona 1

  .zone2_max = 300.0,        // Zona 2: 180-300cm
  .zone2_base = 180.0,
  .zone2_multiplier = 0.15,  // Aumentato per correzione maggiore

  .zone3_max = 350.0,        // Zona 3: 300-350cm
  .zone3_base_corr = 18.0,
  .zone3_base_y = 300.0,
  .zone3_multiplier = 0.5,

  .zone4_transition = 400.0, // Zona 4: >350cm
  .zone4_base_corr = -7.0,
  .zone4_base_y = 350.0,
  .zone4_multiplier = 0.3,

  .valid = true,
  .version = 1
};

// =================================================================
// CALIBRATION PERSISTENCE FUNCTIONS
// =================================================================

// Load calibration from non-volatile storage (flash memory)
void loadCalibration() {
  preferences.begin("radar_calib", false);  // Open in read-write mode

  // Check if valid calibration exists
  bool isValid = preferences.getBool("valid", false);

  if (isValid) {
    // Load all calibration parameters from flash
    calibration.x_offset = preferences.getFloat("x_offset", 0.0);
    calibration.y_global_offset = preferences.getFloat("y_global_off", 60.0);

    calibration.zone1_max = preferences.getFloat("z1_max", 100.0);
    calibration.zone1_correction = preferences.getFloat("z1_corr", 0.0);

    calibration.zone2_max = preferences.getFloat("z2_max", 250.0);
    calibration.zone2_base = preferences.getFloat("z2_base", 100.0);
    calibration.zone2_multiplier = preferences.getFloat("z2_mult", 0.1);

    calibration.zone3_max = preferences.getFloat("z3_max", 320.0);
    calibration.zone3_base_corr = preferences.getFloat("z3_bcorr", 15.0);
    calibration.zone3_base_y = preferences.getFloat("z3_basey", 250.0);
    calibration.zone3_multiplier = preferences.getFloat("z3_mult", 0.64);

    calibration.zone4_transition = preferences.getFloat("z4_trans", 380.0);
    calibration.zone4_base_corr = preferences.getFloat("z4_bcorr", -30.0);
    calibration.zone4_base_y = preferences.getFloat("z4_basey", 320.0);
    calibration.zone4_multiplier = preferences.getFloat("z4_mult", 0.5);

    calibration.valid = true;
    calibration.version = preferences.getUChar("version", 1);

    Serial.println("\n[CALIBRATION] Loaded from flash memory:");
    Serial.printf("  X_OFFSET: %.2f cm\n", calibration.x_offset);
    Serial.printf("  Y_GLOBAL_OFFSET: %.2f cm (added to all Y measurements)\n", calibration.y_global_offset);
    Serial.printf("  Zone 1 (0-%.0f): correction=%.2f\n",
                  calibration.zone1_max, calibration.zone1_correction);
    Serial.printf("  Zone 2 (%.0f-%.0f): base=%.0f, mult=%.2f\n",
                  calibration.zone1_max, calibration.zone2_max,
                  calibration.zone2_base, calibration.zone2_multiplier);
    Serial.printf("  Zone 3 (%.0f-%.0f): base_corr=%.1f, base_y=%.0f, mult=%.2f\n",
                  calibration.zone2_max, calibration.zone3_max,
                  calibration.zone3_base_corr, calibration.zone3_base_y,
                  calibration.zone3_multiplier);
    Serial.printf("  Zone 4 (>%.0f): transition=%.0f, base_corr=%.1f, mult=%.2f\n",
                  calibration.zone3_max, calibration.zone4_transition,
                  calibration.zone4_base_corr, calibration.zone4_multiplier);
    Serial.printf("  Version: %d\n", calibration.version);
  } else {
    // No valid calibration in flash, use defaults
    calibration = DEFAULT_CALIBRATION;
    Serial.println("\n[CALIBRATION] No saved calibration found, using defaults");

    // Save defaults to flash for next boot
    saveCalibration();
  }

  preferences.end();
}

// Save current calibration to non-volatile storage
void saveCalibration() {
  preferences.begin("radar_calib", false);  // Open in read-write mode

  // Save all calibration parameters to flash
  preferences.putFloat("x_offset", calibration.x_offset);
  preferences.putFloat("y_global_off", calibration.y_global_offset);

  preferences.putFloat("z1_max", calibration.zone1_max);
  preferences.putFloat("z1_corr", calibration.zone1_correction);

  preferences.putFloat("z2_max", calibration.zone2_max);
  preferences.putFloat("z2_base", calibration.zone2_base);
  preferences.putFloat("z2_mult", calibration.zone2_multiplier);

  preferences.putFloat("z3_max", calibration.zone3_max);
  preferences.putFloat("z3_bcorr", calibration.zone3_base_corr);
  preferences.putFloat("z3_basey", calibration.zone3_base_y);
  preferences.putFloat("z3_mult", calibration.zone3_multiplier);

  preferences.putFloat("z4_trans", calibration.zone4_transition);
  preferences.putFloat("z4_bcorr", calibration.zone4_base_corr);
  preferences.putFloat("z4_basey", calibration.zone4_base_y);
  preferences.putFloat("z4_mult", calibration.zone4_multiplier);

  preferences.putBool("valid", true);
  preferences.putUChar("version", calibration.version);

  preferences.end();

  Serial.println("\n[CALIBRATION] Saved to flash memory - will persist across power cycles");
}

// Reset calibration to factory defaults
void resetCalibration() {
  calibration = DEFAULT_CALIBRATION;
  saveCalibration();
  Serial.println("\n[CALIBRATION] Reset to factory defaults");
}

// =================================================================
// CONFIGURAZIONE WIFI e AP
// =================================================================
const char *apSSID = "RADAR_CONFIG";
const char *apPassword = "12345678";

// Default WiFi credentials (can be overridden via web interface)
const char *DEFAULT_WIFI_SSID = "Taua-Wifi";
const char *DEFAULT_WIFI_PASSWORD = "Bastia2023";

char clientSSID[50] = "";
char clientPassword[50] = "";
String config_message = "";
bool credentials_submitted = false;
bool wifi_from_storage = false;  // Flag to track if WiFi loaded from storage

// =================================================================
// WIFI CREDENTIALS PERSISTENCE FUNCTIONS
// =================================================================

// Load WiFi credentials from non-volatile storage
bool loadWiFiCredentials() {
  preferences.begin("wifi_config", false);

  bool hasWiFi = preferences.getBool("wifi_saved", false);

  if (hasWiFi) {
    String ssid = preferences.getString("wifi_ssid", "");
    String password = preferences.getString("wifi_pass", "");

    if (ssid.length() > 0) {
      strncpy(clientSSID, ssid.c_str(), sizeof(clientSSID) - 1);
      strncpy(clientPassword, password.c_str(), sizeof(clientPassword) - 1);
      clientSSID[sizeof(clientSSID) - 1] = '\0';
      clientPassword[sizeof(clientPassword) - 1] = '\0';

      preferences.end();

      Serial.println("\n[WIFI] Loaded credentials from flash:");
      Serial.printf("  SSID: %s\n", clientSSID);
      Serial.println("  Password: ********");
      return true;
    }
  }

  preferences.end();

  // No saved WiFi, use defaults
  strncpy(clientSSID, DEFAULT_WIFI_SSID, sizeof(clientSSID) - 1);
  strncpy(clientPassword, DEFAULT_WIFI_PASSWORD, sizeof(clientPassword) - 1);
  clientSSID[sizeof(clientSSID) - 1] = '\0';
  clientPassword[sizeof(clientPassword) - 1] = '\0';

  Serial.println("\n[WIFI] No saved credentials, using defaults:");
  Serial.printf("  SSID: %s\n", clientSSID);
  Serial.println("  Password: ********");

  // Save defaults to flash for next boot
  saveWiFiCredentials();

  return true;
}

// Save WiFi credentials to non-volatile storage
void saveWiFiCredentials() {
  preferences.begin("wifi_config", false);

  preferences.putString("wifi_ssid", clientSSID);
  preferences.putString("wifi_pass", clientPassword);
  preferences.putBool("wifi_saved", true);

  preferences.end();

  Serial.println("\n[WIFI] Credentials saved to flash memory");
}

// Clear WiFi credentials from storage
void clearWiFiCredentials() {
  preferences.begin("wifi_config", false);
  preferences.clear();
  preferences.end();

  Serial.println("\n[WIFI] Credentials cleared from flash");
}

// =================================================================
// SYSTEM MONITORING FUNCTIONS
// =================================================================

// Print system information
void printSystemInfo() {
  Serial.println("\n========================================");
  Serial.println("       SYSTEM INFORMATION");
  Serial.println("========================================");

  // ESP32 chip info
  Serial.printf("Chip Model: %s\n", ESP.getChipModel());
  Serial.printf("Chip Revision: %d\n", ESP.getChipRevision());
  Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());

  // Memory info
  Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Heap Size: %d bytes\n", ESP.getHeapSize());
  Serial.printf("Min Free Heap: %d bytes\n", ESP.getMinFreeHeap());

  // Flash info
  Serial.printf("Flash Size: %d bytes\n", ESP.getFlashChipSize());
  Serial.printf("Flash Speed: %d Hz\n", ESP.getFlashChipSpeed());

  // WiFi info
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi SSID: %s\n", WiFi.SSID().c_str());
    Serial.printf("WiFi IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("WiFi RSSI: %d dBm\n", WiFi.RSSI());
  } else {
    Serial.println("WiFi: Not connected");
  }

  Serial.println("========================================\n");
}

// =================================================================
// MODALITA' OPERATIVA
// =================================================================
enum OperatingMode {
  CONFIG_MODE,
  TRACKING_MODE
};
OperatingMode currentMode = CONFIG_MODE;

// =================================================================
// CONFIGURAZIONE TRACKING
// =================================================================
#define MAX_TRACKS 10
#define MAX_MEASUREMENTS 10
#define MATCHING_THRESHOLD 75  // Distanza massima in cm
#define TIMEOUT_MS 2000        // Rimuovi tracce inattive dopo 2s

// =================================================================
// CALIBRAZIONE DINAMICA MIGLIORATA - V3 (With Persistent Storage)
// =================================================================
// Calibrazione Y migliorata con punti di controllo multipli
// NOTA: I valori di calibrazione sono ora memorizzati in memoria flash
//       e possono essere modificati via web interface o comandi seriali
// Punti di calibrazione verificati:
// Y_raw=320 → Y_target=180 (target reale a 180cm)
// Y_raw=230 → Y_target=180
// Y_raw=140 → Y_target=180
// Y_raw=80  → Y_target=80 (a terra)

float calibrateHeight(float x, float y_measured) {
  // CALIBRAZIONE OTTIMIZZATA usando parametri persistenti
  // - Applica offset globale Y a tutte le misurazioni
  // - Corregge errori zona bassa-media con correzioni specifiche per zona
  // - Passa valori alti quasi diretti per raggiungere riga 7
  // - Parametri caricati da flash memory (sopravvivono a power-down)

  // STEP 1: Apply global Y offset first (baseline correction)
  float y_adjusted = y_measured + calibration.y_global_offset;

  float correction;

  // STEP 2: Apply zone-specific corrections to adjusted value
  // ZONA 1: Ground level (0-zone1_max cm) - no correction
  if (y_adjusted <= calibration.zone1_max) {
    correction = calibration.zone1_correction;
  }
  // ZONA 2: Low-mid range (zone1_max - zone2_max cm) - light correction
  else if (y_adjusted <= calibration.zone2_max) {
    // Linear correction: Y=zone2_base → corr=0, Y=zone2_max → corr=+15
    correction = (y_adjusted - calibration.zone2_base) * calibration.zone2_multiplier;
  }
  // ZONA 3: Mid range (zone2_max - zone3_max cm) - moderate correction
  else if (y_adjusted <= calibration.zone3_max) {
    // Decreasing correction: Y=zone2_max → corr=+15, Y=zone3_max → corr=-30
    correction = calibration.zone3_base_corr -
                 (y_adjusted - calibration.zone3_base_y) * calibration.zone3_multiplier;
  }
  // ZONA 4: High range (>zone3_max cm) - minimal or no correction
  else {
    // Two-stage correction:
    // - zone3_max to zone4_transition: gradual reduction to 0
    // - >zone4_transition: no correction (direct pass-through)
    if (y_adjusted <= calibration.zone4_transition) {
      correction = calibration.zone4_base_corr +
                   (y_adjusted - calibration.zone4_base_y) * calibration.zone4_multiplier;
    } else {
      correction = 0;  // Pass direct value for very high measurements
    }
  }

  // STEP 3: Apply zone correction and clamp to valid range
  float result = y_adjusted + correction;
  if (result < 0) result = 0;
  // Limit to 399cm (not 400) because canvas checks yPos < 400
  if (result >= 400) result = 399;

  return result;
}

// =================================================================
// CONFIGURAZIONE RILEVAMENTO CADUTA (aggiornata per nuovo sistema coordinate)
// =================================================================
// NOTA: Dopo lo scambio coordinate, kf_x contiene laterale, kf_y contiene range/altezza
// Rilevamento caduta basato su variazione velocità verticale e posizione griglia

#define FALL_VELOCITY_THRESHOLD -30  // cm/s - velocità negativa che indica caduta rapida
#define FALL_HEIGHT_LOW 100           // cm - sotto questa altezza si considera "a terra"
#define FALL_HEIGHT_HIGH 200          // cm - sopra questa altezza si considera "in piedi/rialzato"
#define FALL_COOLDOWN 5000            // ms - tempo prima di rilevare nuova caduta per stesso track
#define MOVEMENT_THRESHOLD 30.0       // cm - spostamento X minimo per considerare movimento
#define MOVEMENT_COOLDOWN 2500        // ms - cooldown dopo movimento prima di rilevare caduta
// Configurazione UART per il sensore radar //
HardwareSerial radarSerial(1);
#define RX_PIN 6
#define TX_PIN 7

// Web Server e WebSocket Server //
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// Timing variables
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 10000;  // 10 secondi

// System health monitoring
unsigned long lastSystemCheck = 0;
const unsigned long SYSTEM_CHECK_INTERVAL = 60000;  // Check every minute
// =================================================================
// STRUTTURE DATI PER TRACKING
// =================================================================
struct Kalman1D {
  float x, v, p;
  float q, r;
  bool is_height; // Flag per distinguere altezza (più reattiva) da laterale

// INIZIALIZZAZIONE delle coordinate del punto
  void init(float x0, bool height = false) {
    x = x0;
    v = 0;
    p = 1;
    is_height = height;
    // Parametri più reattivi per altezza (rileva cadute rapidamente)
    if (is_height) {
      q = 0.2;  // Maggiore rumore processo = più reattivo
      r = 0.3;  // Maggiore rumore misura = meno smoothing
    } else {
      q = 0.05; // Parametri standard per coordinata laterale
      r = 0.1;
    }
  }

// PREDIZIONE con filtro di KALMAN
  void predict(float dt) {
    x += v * dt;
    p += q;
  }

// UPDATE delle coordinate del soggetto sull'asse x-y
  void update(float z, float dt) {
    float k = p / (p + r);
    float residual = z - x;
    x += k * residual;
    p *= (1 - k);

    // Calcolo velocità più reattivo per altezza
    if (is_height) {
      v = residual / dt; // Velocità diretta dal residuo (più reattiva)
    } else {
      v += k * residual / dt; // Velocità smoothed per laterale
    }
  }
};
// STRUTTURA: Che tiene i dati di KALMAN
struct Track {
  Kalman1D kf_x, kf_y;
  unsigned long last_update;
  bool active = false;
  int id;
  // Variabili per rilevamento caduta
  float prev_x = 0;              // Posizione X precedente (per rilevare movimento)
  float prev_y = 0;              // Posizione Y precedente
  bool fall_detected = false;    // Flag caduta già rilevata
  unsigned long fall_time = 0;   // Timestamp ultima caduta
  unsigned long movement_time = 0; // Timestamp ultimo movimento X (per cooldown)
};
// TRACKS: last-time
Track tracks[MAX_TRACKS];
unsigned long last_time = 0;

// Contatore globale cadute
int fall_counter = 0;

// Array per memorizzare le posizioni delle cadute (max 50)
#define MAX_FALL_POSITIONS 50
struct FallPosition {
  float x;
  float y;
  bool active;
};
FallPosition fall_positions[MAX_FALL_POSITIONS];
int fall_position_count = 0;

// ================================================================= //
// FUNZIONI PAGINE WEB                                               //
// ================================================================= //
String generateConfigHTML() {
  String html = "<!DOCTYPE html><html><head><title>ESP32 Config</title>";
  html += "<style>";
  html += "body{font-family:Arial;text-align:center;background:#4f85c2;margin:0; padding:0px;}";
  html += "h1{font-size:40px; color:#10243D; text-align:center; margin-top:40px; margin-bottom:40px;}";
  html += "header{background:#10243D;padding:10px;color:white;}";
  html += "form{width: 100%; max-width:900px;margin:auto;padding:20px;background:white;border-radius:10px;box-shadow:0 0 10px rgba(0,0,0,0.1);margin-bottom: 50px;}";
  html += "input{width:60%; padding:30px; margin:20px 0; height:60px; font-size:35px; box-sizing:border-box; border-radius:5px; border:1px solid #ccc;}";
  html += "label{display:block; font-size:26px; font-weight:bold; margin-bottom:5px;}";
  html += "button{height:70px;background:#10243d;color:white;font-size:36px;padding:10px ;border:none;cursor:pointer;width:100%;}";
  html += "</style></head><body>";
  html += "<h1>Configurazione WiFi Radar</h1>";
  html += "<form action='/submit' method='get'>";
  html += "<label for='ssid'>SSID:</label>";
  html += "<input type='text' name='ssid'><br>";
  html += "<label for='password'>Password:</label>";
  html += "<input type='password' name='password'><br>";
  html += "<button type='submit'>Conferma</button></form>";
  html += "<p>" + config_message + "</p>";
  html += "</body></html>";
  return html;
}
// =================================================================
// TRACKING PAGINE WEB
// =================================================================
String generateTrackingHTML() {
    return R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Care TB = Radar Tracking & Heatmap</title>
    <style>
        body { font-family: Arial, sans-serif; text-align: center; background-color: #072a39; margin: 0; padding: 20px; color: #ffffff; }
        h1 { font-size:28px; margin-top:0; }
        #people-count { font-size: 20px; font-weight: bold; margin-bottom: 20px; }
        #calibration-link { position: absolute; top: 20px; right: 20px; background: #667eea; color: white; padding: 10px 20px; border-radius: 8px; text-decoration: none; font-weight: bold; transition: all 0.3s; }
        #calibration-link:hover { background: #5568d3; transform: translateY(-2px); box-shadow: 0 5px 15px rgba(102,126,234,0.3); }
        #calibration-link::before { content: "["; }
        #calibration-link::after { content: "]"; }
        #main-container { display: flex; justify-content: center; align-items: flex-start; gap: 20px; flex-wrap: wrap; }
        .canvas-container { display: flex; flex-direction: column; align-items: center; }
        .canvas-title { font-size: 16px; margin-bottom: 10px; }
        #canvas, #heatmap-canvas { width: 400px; height: 400px; border: 2px solid black; position: relative; }
        #canvas { background-color: #f0f0f0; background-image: linear-gradient(to right, rgba(0, 0, 0, 0.1) 1px, transparent 1px), linear-gradient(to bottom, rgba(0, 0, 0, 0.1) 1px, transparent 1px); background-size: 50px 50px; }
        #heatmap-canvas { background-color: white; }
        .target { width: 20px; height: 20px; border-radius: 50%; position: absolute; background-color: red; transform: translate(-50%, -50%); }
        .grid-label { position: absolute; font-size: 5px; color: #555; pointer-events: none; }
        #movement-log { width: 824px; height: 150px; background: #333; color: #lime; font-family: monospace; text-align: left; overflow-y: scroll; border: 1px solid white; margin: 20px auto; padding: 5px; }
        #reset-btn { background-color: #ff4444; color: white; border: none; padding: 12px 24px; font-size: 16px; font-weight: bold; border-radius: 5px; cursor: pointer; margin-top: 10px; }
        #reset-btn:hover { background-color: #cc0000; }
    </style>
</head>
<body>
    <a href="/calibration" id="calibration-link">CALIBRATION</a>
    <h1>Radar Tracking & Heatmap</h1>
    <h2 id="people-count">People detected: 0</h2>
    <h2 id="fall-count" style="color: #ff6b6b; font-size: 24px; font-weight: bold;">Falls detected: 0</h2>
    <button id="reset-btn" onclick="resetFalls()">Reset Falls & Indicators</button>
    <div id="main-container">
        <div class="canvas-container">
            <div class="canvas-title">Real-time Tracking</div>
            <div id="canvas"></div>
        </div>
        <div class="canvas-container">
            <div class="canvas-title">Permanence Heatmap</div>
            <canvas id="heatmap-canvas" width="400" height="400"></canvas>
        </div>
    </div>
    <div class="canvas-container">
        <div class="canvas-title">Movement Log</div>
        <pre id="movement-log"></pre>
    </div>

    <script>
        // ========= WebSocket Setup ========= //
        let socket;
        let reconnectInterval = 3000;

        function connectWebSocket() {
            socket = new WebSocket("ws://" + location.hostname + ":81/");
            socket.onopen = () => console.log("WebSocket connesso");
            socket.onmessage = handleSocketMessage;
            socket.onclose = () => setTimeout(connectWebSocket, reconnectInterval);
            socket.onerror = (err) => { console.error("WebSocket errore:", err); socket.close(); };
        }

        // ========= Grid and Canvas Setup ========= //
        const GRID_COLS = 8;
        const GRID_ROWS = 8;
        const CANVAS_WIDTH = 400;
        const CANVAS_HEIGHT = 400;
        const CELL_WIDTH = CANVAS_WIDTH / GRID_COLS;
        const CELL_HEIGHT = CANVAS_HEIGHT / GRID_ROWS;

        // ========= Real-time Tracking Setup ========= //
        const trackingCanvas = document.getElementById('canvas');
        for (let i = 0; i < 10; i++) {
            let target = document.createElement('div');
            target.id = `target${i+1}`;
            target.className = 'target';
            target.style.display = 'none';
            trackingCanvas.appendChild(target);
        }

        // Container per gli indicatori di caduta (pallini blu)
        let fallIndicators = [];
        // ========= THE CALCOLOUS OF GRID =========== //
        for (let r = 0; r < GRID_ROWS; r++) {
            for (let c = 0; c < GRID_COLS; c++) {
                const label = document.createElement('div');
                label.className = 'grid-label';
                label.textContent = `${c}.${r}`;
                label.style.left = `${c * CELL_WIDTH + 2}px`;
                label.style.top = `${r * CELL_HEIGHT + 2}px`;
                trackingCanvas.appendChild(label);
            }
        }

        // ========= Heatmap Setup ========= //
        const heatmapCanvas = document.getElementById('heatmap-canvas');
        const heatmapCtx = heatmapCanvas.getContext('2d');
        let heatGrid = Array(GRID_ROWS).fill(0).map(() => Array(GRID_COLS).fill(0));
        const UPDATE_INTERVAL = 100;

        function getHeatColor(milliseconds) {
            const seconds = milliseconds / 1000;
            if (seconds < 1) return 'rgba(255,255,255,0)';
            if (seconds < 30) return 'pink';
            if (seconds < 60) return 'red';
            if (seconds < 90) return 'purple';
            return 'blue';
        }

        function drawHeatmap() {
            heatmapCtx.clearRect(0, 0, heatmapCanvas.width, heatmapCanvas.height);
            for (let r = 0; r < GRID_ROWS; r++) {
                for (let c = 0; c < GRID_COLS; c++) {
                    if (heatGrid[r][c] > 0) {
                        heatmapCtx.fillStyle = getHeatColor(heatGrid[r][c]);
                        heatmapCtx.fillRect(c * CELL_WIDTH, r * CELL_HEIGHT, CELL_WIDTH, CELL_HEIGHT);
                    }
                    heatmapCtx.fillStyle = 'black';
                    heatmapCtx.font = '5px Arial';
                    heatmapCtx.fillText(`${c}.${r}`, c * CELL_WIDTH + 2, r * CELL_HEIGHT + 9);
                }
            }
        }

        // ========= Movement Log Setup ========= //
        const logElement = document.getElementById('movement-log');
        const MAX_LOG_ENTRIES = 50;
        let logMessages = [];
        function logMovement(message) {
            const timestamp = new Date().toLocaleTimeString();
            logMessages.unshift(`${timestamp}: ${message}`);
            if (logMessages.length > MAX_LOG_ENTRIES) {
                logMessages.pop();
            }
            logElement.textContent = logMessages.join('\n');
        }

        // ========= Main Logic ========= //
        let activeTargets = [];
        let lastTargetPositions = {}; // Memorizza l'ultima posizione {col, row} per ogni target

        function handleSocketMessage(event) {
            if (event.data === "ping") return;

            // Parse del nuovo formato: "FALLS:N|x1,y1,x2,y2,...|fx1,fy1,fx2,fy2,..."
            let sections = event.data.split("|");
            let fallCount = 0;

            // Sezione 1: Contatore cadute
            if (sections[0].startsWith("FALLS:")) {
                fallCount = parseInt(sections[0].split(":")[1]);
                document.getElementById("fall-count").textContent = `Falls detected: ${fallCount}`;
            }

            // Sezione 2: Coordinate target attivi
            let targetData = sections[1] ? sections[1].split(",").map(Number) : [];
            let visibleCount = 0;
            activeTargets = [];
            let currentVisibleTargets = {};
            for (let i = 0; i < 10; i++) {
                let element = document.getElementById(`target${i + 1}`);
                if (i * 2 < targetData.length) {
                    // COORDINATE DAL FIRMWARE (scambiate per mapping corretto):
                    // targetData[i*2]   = kf_y.x = Height/Range calibrata (0-400cm)
                    // targetData[i*2+1] = kf_x.x = Posizione laterale (-200 a +200cm)
                    let x = targetData[i * 2];      // Height/Range calibrata (valore Y reale)
                    let y = targetData[i * 2 + 1];  // Lateral position (valore X reale)

                    // MAPPING CANVAS:
                    // x (height) → yPos sul canvas (verticale, 0=vicino/basso, 400=lontano/alto)
                    // y (lateral) → xPos sul canvas (orizzontale, centrato a 200px)
                    let xPos = y + 200;  // Converti laterale da [-200,+200] a [0,400]
                    let yPos = x;        // Range diretto [0,400] → canvas verticale

                    if (xPos >= 0 && xPos < CANVAS_WIDTH && yPos >= 0 && yPos < CANVAS_HEIGHT) {
                        element.style.left = xPos + "px";
                        element.style.top = yPos + "px";
                        element.style.display = "block";
                        visibleCount++;
                        activeTargets.push({x: xPos, y: yPos});
                        currentVisibleTargets[i] = true;

                        const gridCol = Math.floor(xPos / CELL_WIDTH);
                        const gridRow = Math.floor(yPos / CELL_HEIGHT);
                        const lastPos = lastTargetPositions[i];

                        // COORDINATE CORRETTE (firmware invia kf_y.x come primo valore, kf_x.x come secondo):
                        // x = kf_y.x = Altezza/Range calibrata (cm) - valore Y reale del target
                        // y = kf_x.x = Posizione laterale (cm) - valore X laterale del target
                        // yPos = distanza sul canvas (0-400px, verticale)
                        // xPos = laterale sul canvas (0-400px, orizzontale)

                        if (!lastPos || lastPos.x !== x || lastPos.y !== y) {
                            // Usa i valori REALI calibrati, non stime dalla griglia!
                            logMovement(`Target ${i}: Height=${x.toFixed(1)}cm, Lateral=${y.toFixed(1)}cm, Grid[${gridCol}.${gridRow}]`);
                        }

                        lastTargetPositions[i] = { col: gridCol, row: gridRow, x: x, y: y };

                    } else {
                        element.style.display = "none";
                    }
                } else {
                    element.style.display = "none";
                }
            }

            // Rimuovi i target che non sono più visibili
            for (let i in lastTargetPositions) {
                if (!currentVisibleTargets[i]) {
                    delete lastTargetPositions[i];
                }
            }

            document.getElementById("people-count").textContent = `People detected: ${visibleCount}`;

            // Sezione 3: Posizioni cadute (pallini blu permanenti)
            // Rimuovi sempre i vecchi indicatori prima di ricrearli
            fallIndicators.forEach(indicator => {
                if (indicator.parentNode) {
                    indicator.parentNode.removeChild(indicator);
                }
            });
            fallIndicators = [];

            // Crea nuovi indicatori solo se ci sono dati
            if (sections[2] && sections[2].trim() !== "") {
                let fallData = sections[2].split(",").map(Number);

                // Crea nuovi indicatori per ogni caduta
                for (let i = 0; i < fallData.length; i += 2) {
                    if (fallData[i] !== undefined && fallData[i + 1] !== undefined && !isNaN(fallData[i]) && !isNaN(fallData[i + 1])) {
                        // COORDINATE CADUTE (stesso formato dei target):
                        // fallData[i]   = Height/Range calibrata dove è avvenuta la caduta
                        // fallData[i+1] = Posizione laterale dove è avvenuta la caduta
                        let x = fallData[i];      // Height/Range (valore Y reale)
                        let y = fallData[i + 1];  // Lateral (valore X reale)

                        // MAPPING CANVAS (identico ai target):
                        let xPos = y + 200;  // Laterale [-200,+200] → canvas [0,400]
                        let yPos = x;        // Range [0,400] → canvas verticale

                        if (xPos >= 0 && xPos < CANVAS_WIDTH && yPos >= 0 && yPos < CANVAS_HEIGHT) {
                            let indicator = document.createElement('div');
                            indicator.style.width = '15px';
                            indicator.style.height = '15px';
                            indicator.style.borderRadius = '50%';
                            indicator.style.position = 'absolute';
                            indicator.style.backgroundColor = 'blue';
                            indicator.style.border = '2px solid white';
                            indicator.style.left = xPos + 'px';
                            indicator.style.top = yPos + 'px';
                            indicator.style.transform = 'translate(-50%, -50%)';
                            indicator.style.zIndex = '5';
                            trackingCanvas.appendChild(indicator);
                            fallIndicators.push(indicator);
                        }
                    }
                }
            }
        }
        
        function updateHeatmap() {
            activeTargets.forEach(target => {
                const gridCol = Math.floor(target.x / CELL_WIDTH);
                const gridRow = Math.floor(target.y / CELL_HEIGHT);
                if (gridRow >= 0 && gridRow < GRID_ROWS && gridCol >= 0 && gridCol < GRID_COLS) {
                    heatGrid[gridRow][gridCol] += UPDATE_INTERVAL;
                }
            });
            drawHeatmap();
        }

        // ========= Reset Function ========= //
        function resetFalls() {
            if (socket && socket.readyState === WebSocket.OPEN) {
                socket.send("RESET");
                logMovement("Reset comando inviato - Azzeramento cadute e indicatori");
            } else {
                alert("WebSocket non connesso. Impossibile inviare il comando di reset.");
            }
        }

        // ========= Life-cycles ========= //
        window.onload = function() {
            connectWebSocket();
            setInterval(updateHeatmap, UPDATE_INTERVAL);
        };
    </script>
</body>
</html>
)rawliteral";
}

// =================================================================
// CALIBRATION WEB PAGE
// =================================================================
String generateCalibrationHTML() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Radar Calibration Settings</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta charset="UTF-8">
    <style>
        body {
            font-family: Arial, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            margin: 0;
            padding: 20px;
            color: #333;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
            background: white;
            border-radius: 15px;
            box-shadow: 0 10px 40px rgba(0,0,0,0.3);
            padding: 30px;
        }
        h1 {
            text-align: center;
            color: #667eea;
            margin-bottom: 10px;
        }
        .subtitle {
            text-align: center;
            color: #666;
            font-size: 14px;
            margin-bottom: 30px;
        }
        .main-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 30px;
        }
        .section {
            margin-bottom: 30px;
            padding: 20px;
            background: #f8f9fa;
            border-radius: 10px;
            border-left: 4px solid #667eea;
        }
        .section h2 {
            color: #667eea;
            margin-top: 0;
            font-size: 18px;
        }
        .param-row {
            display: grid;
            grid-template-columns: 150px 120px;
            gap: 10px;
            margin-bottom: 15px;
            align-items: center;
        }
        .param-label {
            font-weight: bold;
            color: #555;
            font-size: 14px;
        }
        .param-value {
            padding: 8px;
            border: 2px solid #ddd;
            border-radius: 5px;
            font-size: 16px;
            width: 100%;
            box-sizing: border-box;
        }
        .button-group {
            display: flex;
            gap: 15px;
            justify-content: center;
            margin-top: 30px;
            grid-column: span 2;
        }
        button {
            padding: 15px 30px;
            font-size: 16px;
            font-weight: bold;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            transition: all 0.3s;
        }
        .btn-save {
            background: #28a745;
            color: white;
        }
        .btn-save:hover {
            background: #218838;
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(40,167,69,0.3);
        }
        .btn-reset {
            background: #dc3545;
            color: white;
        }
        .btn-reset:hover {
            background: #c82333;
        }
        .btn-back {
            background: #6c757d;
            color: white;
        }
        .btn-test {
            background: #17a2b8;
            color: white;
        }
        .btn-test:hover {
            background: #138496;
        }
        .status-message {
            padding: 15px;
            margin: 20px 0;
            border-radius: 8px;
            text-align: center;
            font-weight: bold;
            display: none;
            grid-column: span 2;
        }
        .status-success {
            background: #d4edda;
            color: #155724;
            border: 1px solid #c3e6cb;
        }
        .info-box {
            background: #e7f3ff;
            border-left: 4px solid #2196F3;
            padding: 15px;
            margin-bottom: 20px;
            border-radius: 5px;
            grid-column: span 2;
        }
        .test-section {
            grid-column: span 2;
            background: #fff3cd;
            border-left: 4px solid #ffc107;
        }
        .test-results {
            margin-top: 15px;
            background: white;
            padding: 15px;
            border-radius: 5px;
            font-family: monospace;
            font-size: 13px;
        }
        .test-row {
            display: grid;
            grid-template-columns: 100px 120px 120px 120px 150px;
            padding: 8px;
            border-bottom: 1px solid #eee;
            gap: 10px;
        }
        .test-header {
            font-weight: bold;
            background: #667eea;
            color: white;
            padding: 10px 8px;
        }
        .zone-indicator {
            display: inline-block;
            padding: 2px 8px;
            border-radius: 3px;
            font-size: 11px;
            font-weight: bold;
        }
        .zone-1 { background: #90EE90; color: #000; }
        .zone-2 { background: #FFD700; color: #000; }
        .zone-3 { background: #FFA500; color: #000; }
        .zone-4 { background: #FF6347; color: #fff; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Radar Calibration Settings</h1>
        <div class="subtitle">Persistent calibration parameters (survives power down)</div>

        <div class="info-box">
            <strong>INFO:</strong> These values are stored in non-volatile flash memory.<br><br>
            <strong>Quick Setup Guide:</strong><br>
            1. Open Serial Monitor (115200 baud) to see [CALIB] debug messages<br>
            2. Stand at known distances (50cm, 100cm, 180cm) and note Y_raw values<br>
            3. Adjust Y Global Offset so Y_final matches your real distance<br>
            4. Example: If Y_raw=60cm when you're at 180cm, set offset to 120cm (180-60=120)<br>
            5. Use Test Calibration button to preview before saving
        </div>

        <div id="statusMessage" class="status-message"></div>

        <form id="calibrationForm" class="main-grid">
            <!-- Left Column -->
            <div>
                <!-- Global Offsets Section -->
                <div class="section">
                    <h2>Global Offsets</h2>
                    <div class="param-row">
                        <div class="param-label">X Offset (cm):</div>
                        <input type="number" step="0.1" class="param-value" id="x_offset" value=")rawliteral" + String(calibration.x_offset, 1) + R"rawliteral(">
                    </div>
                    <div class="param-row">
                        <div class="param-label">Y Global Offset (cm):</div>
                        <input type="number" step="0.1" class="param-value" id="y_global_offset" value=")rawliteral" + String(calibration.y_global_offset, 1) + R"rawliteral(">
                    </div>
                    <div style="font-size:12px; color:#666; margin-top:8px; padding:8px; background:#f8f9fa; border-radius:4px;">
                        <strong>Note:</strong> Y Global Offset is added to ALL height measurements before zone corrections.
                        If sensor reads 40cm when person is at 100cm, set this to +60cm.
                    </div>
                </div>

                <!-- Zone 1 Section -->
                <div class="section">
                    <h2>Zone 1: Ground (0-100cm)</h2>
                    <div class="param-row">
                        <div class="param-label">Max Height:</div>
                        <input type="number" step="1" class="param-value" id="z1_max" value=")rawliteral" + String(calibration.zone1_max, 0) + R"rawliteral(">
                    </div>
                    <div class="param-row">
                        <div class="param-label">Correction:</div>
                        <input type="number" step="0.1" class="param-value" id="z1_corr" value=")rawliteral" + String(calibration.zone1_correction, 1) + R"rawliteral(">
                    </div>
                </div>

                <!-- Zone 2 Section -->
                <div class="section">
                    <h2>Zone 2: Low-Mid (100-250cm)</h2>
                    <div class="param-row">
                        <div class="param-label">Max Height:</div>
                        <input type="number" step="1" class="param-value" id="z2_max" value=")rawliteral" + String(calibration.zone2_max, 0) + R"rawliteral(">
                    </div>
                    <div class="param-row">
                        <div class="param-label">Base Y:</div>
                        <input type="number" step="1" class="param-value" id="z2_base" value=")rawliteral" + String(calibration.zone2_base, 0) + R"rawliteral(">
                    </div>
                    <div class="param-row">
                        <div class="param-label">Multiplier:</div>
                        <input type="number" step="0.01" class="param-value" id="z2_mult" value=")rawliteral" + String(calibration.zone2_multiplier, 2) + R"rawliteral(">
                    </div>
                </div>
            </div>

            <!-- Right Column -->
            <div>
                <!-- Zone 3 Section -->
                <div class="section">
                    <h2>Zone 3: Mid (250-320cm)</h2>
                    <div class="param-row">
                        <div class="param-label">Max Height:</div>
                        <input type="number" step="1" class="param-value" id="z3_max" value=")rawliteral" + String(calibration.zone3_max, 0) + R"rawliteral(">
                    </div>
                    <div class="param-row">
                        <div class="param-label">Base Corr:</div>
                        <input type="number" step="0.1" class="param-value" id="z3_bcorr" value=")rawliteral" + String(calibration.zone3_base_corr, 1) + R"rawliteral(">
                    </div>
                    <div class="param-row">
                        <div class="param-label">Base Y:</div>
                        <input type="number" step="1" class="param-value" id="z3_basey" value=")rawliteral" + String(calibration.zone3_base_y, 0) + R"rawliteral(">
                    </div>
                    <div class="param-row">
                        <div class="param-label">Multiplier:</div>
                        <input type="number" step="0.01" class="param-value" id="z3_mult" value=")rawliteral" + String(calibration.zone3_multiplier, 2) + R"rawliteral(">
                    </div>
                </div>

                <!-- Zone 4 Section -->
                <div class="section">
                    <h2>Zone 4: High (>320cm)</h2>
                    <div class="param-row">
                        <div class="param-label">Transition:</div>
                        <input type="number" step="1" class="param-value" id="z4_trans" value=")rawliteral" + String(calibration.zone4_transition, 0) + R"rawliteral(">
                    </div>
                    <div class="param-row">
                        <div class="param-label">Base Corr:</div>
                        <input type="number" step="0.1" class="param-value" id="z4_bcorr" value=")rawliteral" + String(calibration.zone4_base_corr, 1) + R"rawliteral(">
                    </div>
                    <div class="param-row">
                        <div class="param-label">Base Y:</div>
                        <input type="number" step="1" class="param-value" id="z4_basey" value=")rawliteral" + String(calibration.zone4_base_y, 0) + R"rawliteral(">
                    </div>
                    <div class="param-row">
                        <div class="param-label">Multiplier:</div>
                        <input type="number" step="0.01" class="param-value" id="z4_mult" value=")rawliteral" + String(calibration.zone4_multiplier, 2) + R"rawliteral(">
                    </div>
                </div>
            </div>

            <!-- Test Section -->
            <div class="section test-section">
                <h2>Calibration Test Preview</h2>
                <p>Click "Test Calibration" to see how current settings affect height measurements for 180cm and 100cm targets.</p>
                <button type="button" class="btn-test" onclick="testCalibration()">Test Calibration</button>
                <div id="testResults" class="test-results" style="display:none;"></div>
            </div>

            <div class="button-group">
                <button type="submit" class="btn-save">Save Calibration</button>
                <button type="button" class="btn-reset" onclick="resetToDefaults()">Reset Defaults</button>
                <button type="button" class="btn-back" onclick="location.href='/'">Back</button>
            </div>
        </form>
    </div>

    <script>
        function testCalibration() {
            // Get current form values
            const y_global_offset = parseFloat(document.getElementById('y_global_offset').value);
            const z1_max = parseFloat(document.getElementById('z1_max').value);
            const z1_corr = parseFloat(document.getElementById('z1_corr').value);
            const z2_max = parseFloat(document.getElementById('z2_max').value);
            const z2_base = parseFloat(document.getElementById('z2_base').value);
            const z2_mult = parseFloat(document.getElementById('z2_mult').value);
            const z3_max = parseFloat(document.getElementById('z3_max').value);
            const z3_bcorr = parseFloat(document.getElementById('z3_bcorr').value);
            const z3_basey = parseFloat(document.getElementById('z3_basey').value);
            const z3_mult = parseFloat(document.getElementById('z3_mult').value);
            const z4_trans = parseFloat(document.getElementById('z4_trans').value);
            const z4_bcorr = parseFloat(document.getElementById('z4_bcorr').value);
            const z4_basey = parseFloat(document.getElementById('z4_basey').value);
            const z4_mult = parseFloat(document.getElementById('z4_mult').value);

            // Function to calculate calibration
            function calibrate(y_measured) {
                // STEP 1: Apply global Y offset
                let y_adjusted = y_measured + y_global_offset;

                let correction = 0;
                let zone = '';

                // STEP 2: Apply zone-specific corrections
                if (y_adjusted <= z1_max) {
                    correction = z1_corr;
                    zone = 'Zone 1';
                } else if (y_adjusted <= z2_max) {
                    correction = (y_adjusted - z2_base) * z2_mult;
                    zone = 'Zone 2';
                } else if (y_adjusted <= z3_max) {
                    correction = z3_bcorr - (y_adjusted - z3_basey) * z3_mult;
                    zone = 'Zone 3';
                } else {
                    if (y_adjusted <= z4_trans) {
                        correction = z4_bcorr + (y_adjusted - z4_basey) * z4_mult;
                        zone = 'Zone 4a';
                    } else {
                        correction = 0;
                        zone = 'Zone 4b';
                    }
                }

                // STEP 3: Apply zone correction
                let result = y_adjusted + correction;
                if (result < 0) result = 0;
                if (result >= 400) result = 399;

                return { result: result, correction: correction, zone: zone, y_adjusted: y_adjusted };
            }

            // Test heights from 50cm to 400cm
            let html = '<div class="test-row test-header">';
            html += '<div>Y Raw (cm)</div>';
            html += '<div>Correction (cm)</div>';
            html += '<div>Y Calibrated (cm)</div>';
            html += '<div>Zone</div>';
            html += '<div>Target Match</div>';
            html += '</div>';

            const testHeights = [50, 80, 100, 120, 150, 180, 200, 220, 250, 280, 300, 320, 350, 380, 400];

            testHeights.forEach(y_raw => {
                const calib = calibrate(y_raw);
                let zoneClass = 'zone-1';
                if (calib.zone.includes('2')) zoneClass = 'zone-2';
                else if (calib.zone.includes('3')) zoneClass = 'zone-3';
                else if (calib.zone.includes('4')) zoneClass = 'zone-4';

                // Check if close to target heights
                let targetMatch = '';
                if (Math.abs(calib.result - 100) < 10) {
                    targetMatch = '<strong style="color:green">~ 100cm TARGET</strong>';
                } else if (Math.abs(calib.result - 180) < 10) {
                    targetMatch = '<strong style="color:blue">~ 180cm TARGET</strong>';
                }

                html += '<div class="test-row">';
                html += '<div>' + y_raw.toFixed(0) + '</div>';
                html += '<div>' + (calib.correction >= 0 ? '+' : '') + calib.correction.toFixed(1) + '</div>';
                html += '<div><strong>' + calib.result.toFixed(1) + '</strong></div>';
                html += '<div><span class="zone-indicator ' + zoneClass + '">' + calib.zone + '</span></div>';
                html += '<div>' + targetMatch + '</div>';
                html += '</div>';
            });

            document.getElementById('testResults').innerHTML = html;
            document.getElementById('testResults').style.display = 'block';
        }

        document.getElementById('calibrationForm').addEventListener('submit', function(e) {
            e.preventDefault();

            const formData = new URLSearchParams();
            formData.append('x_offset', document.getElementById('x_offset').value);
            formData.append('y_global_offset', document.getElementById('y_global_offset').value);
            formData.append('z1_max', document.getElementById('z1_max').value);
            formData.append('z1_corr', document.getElementById('z1_corr').value);
            formData.append('z2_max', document.getElementById('z2_max').value);
            formData.append('z2_base', document.getElementById('z2_base').value);
            formData.append('z2_mult', document.getElementById('z2_mult').value);
            formData.append('z3_max', document.getElementById('z3_max').value);
            formData.append('z3_bcorr', document.getElementById('z3_bcorr').value);
            formData.append('z3_basey', document.getElementById('z3_basey').value);
            formData.append('z3_mult', document.getElementById('z3_mult').value);
            formData.append('z4_trans', document.getElementById('z4_trans').value);
            formData.append('z4_bcorr', document.getElementById('z4_bcorr').value);
            formData.append('z4_basey', document.getElementById('z4_basey').value);
            formData.append('z4_mult', document.getElementById('z4_mult').value);

            fetch('/save-calibration?' + formData.toString())
                .then(response => response.text())
                .then(data => {
                    showMessage(data, 'success');
                })
                .catch(error => {
                    showMessage('Error: ' + error, 'error');
                });
        });

        function resetToDefaults() {
            if (confirm('Reset all calibration values to factory defaults?')) {
                fetch('/reset-calibration')
                    .then(response => response.text())
                    .then(data => {
                        showMessage(data, 'success');
                        setTimeout(() => location.reload(), 1500);
                    });
            }
        }

        function showMessage(message, type) {
            const statusDiv = document.getElementById('statusMessage');
            statusDiv.textContent = message;
            statusDiv.className = 'status-message status-' + type;
            statusDiv.style.display = 'block';
            setTimeout(() => {
                statusDiv.style.display = 'none';
            }, 5000);
        }
    </script>
</body>
</html>
)rawliteral";
    return html;
}

// =================================================================
// HANDLER SERVER WEB
// =================================================================
void handleRoot_Config() {
  server.send(200, "text/html", generateConfigHTML());
}

void handleSubmit() {
  if (server.hasArg("ssid") && server.hasArg("password")) {
    strncpy(clientSSID, server.arg("ssid").c_str(), sizeof(clientSSID) - 1);
    strncpy(clientPassword, server.arg("password").c_str(), sizeof(clientPassword) - 1);
    clientSSID[sizeof(clientSSID) - 1] = '\0';  // Ensure null termination
    clientPassword[sizeof(clientPassword) - 1] = '\0';

    config_message = "Credenziali ricevute. Connessione in corso...";
    server.send(200, "text/html", generateConfigHTML());
    delay(1000);  // Reduced delay for faster response
    credentials_submitted = true;

    Serial.println("\n[CONFIG] New WiFi credentials submitted via web");
    Serial.printf("[CONFIG] SSID: %s\n", clientSSID);
  } else {
    config_message = "Errore: SSID o password mancanti.";
    server.send(400, "text/html", generateConfigHTML());
  }
}
// TRACKING del root dell'handle //
void handleRoot_Tracking() {
  server.send(200, "text/html", generateTrackingHTML());
}

// CALIBRATION page handler //
void handleCalibration() {
  server.send(200, "text/html", generateCalibrationHTML());
}

// SAVE CALIBRATION handler //
void handleSaveCalibration() {
  // Parse all calibration parameters from URL arguments
  if (server.hasArg("x_offset")) {
    calibration.x_offset = server.arg("x_offset").toFloat();
  }
  if (server.hasArg("y_global_offset")) {
    calibration.y_global_offset = server.arg("y_global_offset").toFloat();
  }

  if (server.hasArg("z1_max")) {
    calibration.zone1_max = server.arg("z1_max").toFloat();
  }
  if (server.hasArg("z1_corr")) {
    calibration.zone1_correction = server.arg("z1_corr").toFloat();
  }

  if (server.hasArg("z2_max")) {
    calibration.zone2_max = server.arg("z2_max").toFloat();
  }
  if (server.hasArg("z2_base")) {
    calibration.zone2_base = server.arg("z2_base").toFloat();
  }
  if (server.hasArg("z2_mult")) {
    calibration.zone2_multiplier = server.arg("z2_mult").toFloat();
  }

  if (server.hasArg("z3_max")) {
    calibration.zone3_max = server.arg("z3_max").toFloat();
  }
  if (server.hasArg("z3_bcorr")) {
    calibration.zone3_base_corr = server.arg("z3_bcorr").toFloat();
  }
  if (server.hasArg("z3_basey")) {
    calibration.zone3_base_y = server.arg("z3_basey").toFloat();
  }
  if (server.hasArg("z3_mult")) {
    calibration.zone3_multiplier = server.arg("z3_mult").toFloat();
  }

  if (server.hasArg("z4_trans")) {
    calibration.zone4_transition = server.arg("z4_trans").toFloat();
  }
  if (server.hasArg("z4_bcorr")) {
    calibration.zone4_base_corr = server.arg("z4_bcorr").toFloat();
  }
  if (server.hasArg("z4_basey")) {
    calibration.zone4_base_y = server.arg("z4_basey").toFloat();
  }
  if (server.hasArg("z4_mult")) {
    calibration.zone4_multiplier = server.arg("z4_mult").toFloat();
  }

  // Save to non-volatile storage
  saveCalibration();

  // Send success response
  server.send(200, "text/plain", "[OK] Calibration saved successfully! Values will persist across power cycles.");

  // Log the update
  Serial.println("\n[CALIBRATION UPDATE] New values saved:");
  Serial.printf("  X_OFFSET: %.2f cm\n", calibration.x_offset);
  Serial.printf("  Y_GLOBAL_OFFSET: %.2f cm (applied to all Y measurements)\n", calibration.y_global_offset);
  Serial.printf("  Zone 1: max=%.0f, corr=%.2f\n",
                calibration.zone1_max, calibration.zone1_correction);
  Serial.printf("  Zone 2: max=%.0f, base=%.0f, mult=%.2f\n",
                calibration.zone2_max, calibration.zone2_base, calibration.zone2_multiplier);
  Serial.printf("  Zone 3: max=%.0f, base_corr=%.1f, base_y=%.0f, mult=%.2f\n",
                calibration.zone3_max, calibration.zone3_base_corr,
                calibration.zone3_base_y, calibration.zone3_multiplier);
  Serial.printf("  Zone 4: trans=%.0f, base_corr=%.1f, base_y=%.0f, mult=%.2f\n",
                calibration.zone4_transition, calibration.zone4_base_corr,
                calibration.zone4_base_y, calibration.zone4_multiplier);
}

// RESET CALIBRATION handler //
void handleResetCalibration() {
  resetCalibration();
  server.send(200, "text/plain", "[OK] Calibration reset to factory defaults!");
  Serial.println("\n[CALIBRATION] Reset to factory defaults via web interface");
}

// GET CURRENT SENSOR DATA handler (for real-time calibration testing)
void handleGetSensorData() {
  // This endpoint returns the last raw sensor reading
  // Format: "raw_y:value1,value2,value3|calibrated_y:value1,value2,value3"
  String response = "Use this for real-time monitoring in future version";
  server.send(200, "text/plain", response);
}
// WEBSOCKET of the event //
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[WebSocket] Client #%d connected from %s\n", num, ip.toString().c_str());
      }
      break;

    case WStype_DISCONNECTED:
      Serial.printf("[WebSocket] Client #%d disconnected\n", num);
      break;

    case WStype_TEXT:
      {
        // Handle incoming messages from client
        if (length > 0 && payload != NULL) {
          // Use char array instead of String for better memory management
          char message[20];
          size_t msgLen = min(length, sizeof(message) - 1);
          memcpy(message, payload, msgLen);
          message[msgLen] = '\0';

          if (strcmp(message, "RESET") == 0) {
            // Reset fall counter and indicators
            fall_counter = 0;
            fall_position_count = 0;

            // Clear all fall positions
            for (int i = 0; i < MAX_FALL_POSITIONS; i++) {
              fall_positions[i].active = false;
            }

            Serial.println("[RESET] Fall counter and indicators cleared");
            // Send confirmation to client
            webSocket.broadcastTXT("FALLS:0||");
          }
        }
      }
      break;

    case WStype_ERROR:
      Serial.printf("[WebSocket] Error on client #%d\n", num);
      break;

    default:
      break;
  }
}
// =================================================================
// FUNZIONI DI TRACKING
// =================================================================
float distance(float x1, float y1, float x2, float y2) {
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

void run_tracking_logic() {
  if (radarSerial.available() > 20) {
    byte frame[120];
    int index = 0;
    while (radarSerial.available() && index < sizeof(frame)) {
      frame[index++] = radarSerial.read();
    }

    //Radar frames that indicates the coordinates of the target.
    if (frame[0] == 0x53 && frame[1] == 0x59 && frame[2] == 0x82 && frame[3] == 0x02) {
      unsigned long current_time = millis();
      float dt = (current_time - last_time) / 1000.0;
      if (dt <= 0) dt = 0.01;
      last_time = current_time;

      uint8_t data_length = frame[5];
      int num_measurements = data_length / 11;

      float meas_x[num_measurements];
      float meas_y[num_measurements];

      for (int k = 0; k < num_measurements; k++) {
        // Estrai coordinate grezze e applica offset X (caricato da memoria persistente)
        meas_x[k] = (frame[9 + 11 * k] & 0x80 ? -1 : 1) * ((frame[9 + 11 * k] & 0x7F) << 8 | frame[10 + 11 * k]) + calibration.x_offset;
        float y_raw = (frame[11 + 11 * k] & 0x80 ? -1 : 1) * ((frame[11 + 11 * k] & 0x7F) << 8 | frame[12 + 11 * k]);

        // Applica calibrazione dinamica Y (usa parametri caricati da flash memory)
        meas_y[k] = calibrateHeight(meas_x[k], y_raw);

        // DEBUG: Log calibrazione (ABILITATO per diagnostica - ogni 10 frames)
        static int debug_counter = 0;
        if (debug_counter++ % 10 == 0 && k == 0) {
          // Calcola valori intermedi per debug
          float y_adjusted = y_raw + calibration.y_global_offset;
          float total_correction = meas_y[k] - y_raw;
          float zone_correction = meas_y[k] - y_adjusted;

          // Determina in quale zona siamo (basato su y_adjusted)
          const char* zone_name;
          if (y_adjusted <= calibration.zone1_max) {
            zone_name = "Zone1";
          } else if (y_adjusted <= calibration.zone2_max) {
            zone_name = "Zone2";
          } else if (y_adjusted <= calibration.zone3_max) {
            zone_name = "Zone3";
          } else {
            zone_name = "Zone4";
          }

          // Calcola gridRow per debug
          int grid_row = (int)(meas_y[k] / 50.0);
          if (grid_row > 7) grid_row = 7;

          Serial.printf("[CALIB] Y_raw=%.1f + Offset=%.1f = %.1f -> Y_final=%.1f (GridRow=%d) | %s\n",
                        y_raw, calibration.y_global_offset, y_adjusted, meas_y[k], grid_row, zone_name);
        }
      }

      // Step 1: PREDIZIONE //
      for (int i = 0; i < MAX_TRACKS; i++) {
        if (tracks[i].active) {
          tracks[i].kf_x.predict(dt);
          tracks[i].kf_y.predict(dt);
        }
      }

      // Step 2: ASSOCIAZIONE (GREEDY) //
      bool measurement_used[num_measurements] = { false };
      for (int i = 0; i < MAX_TRACKS; i++) {
        if (!tracks[i].active) continue;

        int best_meas = -1;
        float best_dist = MATCHING_THRESHOLD;

        for (int j = 0; j < num_measurements; j++) {
          if (measurement_used[j]) continue;
          float d = distance(tracks[i].kf_x.x, tracks[i].kf_y.x, meas_x[j], meas_y[j]);
          if (d < best_dist) {
            best_dist = d;
            best_meas = j;
          }
        }

        if (best_meas != -1) {
          tracks[i].kf_x.update(meas_x[best_meas], dt);
          tracks[i].kf_y.update(meas_y[best_meas], dt);
          tracks[i].last_update = current_time;
          measurement_used[best_meas] = true;
        }
      }

      // Step 2.5: RILEVAMENTO CADUTA (solo tra righe adiacenti) //
      for (int i = 0; i < MAX_TRACKS; i++) {
        if (!tracks[i].active) continue;

        float current_x = tracks[i].kf_x.x;      // Laterale
        float current_height = tracks[i].kf_y.x;  // Range X calibrato (yPos)

        // Calcola riga corrente e precedente (celle 50cm)
        int current_row = (int)(current_height / 50.0);
        int prev_row = (int)(tracks[i].prev_y / 50.0);
        int row_drop = prev_row - current_row;  // Positivo = caduta verso riga inferiore

        // Rileva movimento laterale X (se si sposta di oltre 30cm)
        float x_displacement = fabs(current_x - tracks[i].prev_x);
        if (x_displacement > MOVEMENT_THRESHOLD) {
          tracks[i].movement_time = current_time;
          Serial.printf("[MOVEMENT] Track %d spostato di %.1fcm (X=%.1f->%.1f) - Cooldown 2.5s\n",
                        tracks[i].id, x_displacement, tracks[i].prev_x, current_x);
        }

        // Cooldown: verifica tempo dall'ultima caduta e dall'ultimo movimento
        bool fall_cooldown_ok = (current_time - tracks[i].fall_time) > FALL_COOLDOWN;
        bool movement_cooldown_ok = (current_time - tracks[i].movement_time) > MOVEMENT_COOLDOWN;

        // Rileva caduta SOLO se:
        // 1. Cooldown caduta scaduto (5s dall'ultima caduta)
        // 2. Cooldown movimento scaduto (2.5s dall'ultimo movimento X)
        // 3. Velocità negativa rapida (sta cadendo)
        // 4. Caduta di ESATTAMENTE 1 riga (da riga N a riga N-1)
        if (fall_cooldown_ok &&
            movement_cooldown_ok &&
            tracks[i].kf_y.v < FALL_VELOCITY_THRESHOLD &&
            row_drop == 1) {  // SOLO caduta tra righe adiacenti!

          // Nuova caduta rilevata!
          if (!tracks[i].fall_detected) {
            fall_counter++;
            tracks[i].fall_detected = true;
            tracks[i].fall_time = current_time;

            // Memorizza posizione caduta
            if (fall_position_count < MAX_FALL_POSITIONS) {
              fall_positions[fall_position_count].x = tracks[i].kf_x.x;
              fall_positions[fall_position_count].y = tracks[i].kf_y.x;
              fall_positions[fall_position_count].active = true;
              fall_position_count++;
            }

            // Log dettagliato con righe
            Serial.printf("[FALL DETECTED] Track %d: Riga %d->%d, Height=%.1f->%.1fcm, Vel=%.1fcm/s, X=%.1fcm\n",
                          tracks[i].id, prev_row, current_row, tracks[i].prev_y, current_height,
                          tracks[i].kf_y.v, current_x);
          }
        }

        // Reset flag caduta se target si rialza (sale di almeno 1 riga)
        if (tracks[i].fall_detected && row_drop < 0) {  // row_drop negativo = salita
          tracks[i].fall_detected = false;
          Serial.printf("[RECOVERY] Track %d rialzato (Riga %d->%d, Height=%.1fcm)\n",
                        tracks[i].id, prev_row, current_row, current_height);
        }

        // Aggiorna posizioni precedenti
        tracks[i].prev_x = current_x;
        tracks[i].prev_y = current_height;
      }

      // Step 2.6: LOG SEMPLIFICATO ALTEZZE TARGET (DISABILITATO - solo WebSocket) //
      // static unsigned long last_log_time = 0;
      // if (current_time - last_log_time > 1000) {
      //   last_log_time = current_time;
      //   Serial.print("Target attivi: ");
      //   bool first = true;
      //   for (int i = 0; i < MAX_TRACKS; i++) {
      //     if (tracks[i].active) {
      //       if (!first) Serial.print(" | ");
      //       Serial.printf("T%d: X=%.0f Y=%.0f cm", tracks[i].id, tracks[i].kf_x.x, tracks[i].kf_y.x);
      //       first = false;
      //     }
      //   }
      //   if (first) Serial.print("Nessuno");
      //   Serial.println();
      // }

      /* LOG DETTAGLIATO (COMMENTATO) - Decommentare per debug avanzato
      static unsigned long last_log_time = 0;
      if (current_time - last_log_time > 1000) {
        last_log_time = current_time;
        Serial.println("========== STATO TARGET ==========");
        for (int i = 0; i < MAX_TRACKS; i++) {
          if (tracks[i].active) {
            Serial.printf("Track %d: X=%.1f cm, Y=%.1f cm, VelY=%.1f cm/s, Caduta=%s, Soglie(Vel=%.0f, Y=%.0f)\n",
                          tracks[i].id, tracks[i].kf_x.x, tracks[i].kf_y.x, tracks[i].kf_y.v,
                          tracks[i].fall_detected ? "SI" : "NO",
                          (float)FALL_VELOCITY_THRESHOLD, (float)FALL_Y_THRESHOLD);
          }
        }
        Serial.println("==================================");
      }
      */

      // Step 3: Crea nuove TRACCE //
      for (int i = 0; i < num_measurements; i++) {
        if (!measurement_used[i]) {
          for (int j = 0; j < MAX_TRACKS; j++) {
            if (!tracks[j].active) {
              tracks[j].kf_x.init(meas_x[i], false);  // Laterale: smoothing normale
              tracks[j].kf_y.init(meas_y[i], true);   // Altezza: più reattivo per cadute
              tracks[j].last_update = current_time;
              tracks[j].active = true;
              tracks[j].id = j;
              tracks[j].prev_x = meas_x[i];
              tracks[j].prev_y = meas_y[i];
              tracks[j].movement_time = current_time; // Inizializza cooldown movimento
              Serial.printf("[NEW TARGET] Track %d: X=%.1fcm, Y=%.1fcm\n",
                            tracks[j].id, meas_x[i], meas_y[i]);
              break;
            }
          }
        }
      }

      // Step 4: Rimuovi tracce SCADUTE //
      for (int i = 0; i < MAX_TRACKS; i++) {
        if (tracks[i].active && (current_time - tracks[i].last_update > TIMEOUT_MS)) {
          // Serial.printf("✗ Track %d uscito dalla griglia (ultima pos: X=%.1f cm, Y=%.1f cm)\n",
          //               tracks[i].id, tracks[i].kf_x.x, tracks[i].kf_y.x);
          tracks[i].active = false;
        }
      }

      // Step 5: Invia dati via WebSocket //
      // Formato: "FALLS:N|height1,lateral1,height2,lateral2,...|fh1,fl1,fh2,fl2,..."
      //
      // COORDINATE INVIATE:
      // - Primo valore: kf_y.x = Altezza/Range calibrata (0-400cm)
      // - Secondo valore: kf_x.x = Posizione laterale (-200 a +200cm)
      //
      // Il JavaScript riceve questi valori come:
      // - x = height/range (usato come yPos verticale sul canvas)
      // - y = lateral (usato come xPos orizzontale sul canvas)

      // Rate limiting: send WebSocket updates every 100ms max
      static unsigned long lastWebSocketSend = 0;
      const unsigned long WEBSOCKET_SEND_INTERVAL = 100;

      if (current_time - lastWebSocketSend >= WEBSOCKET_SEND_INTERVAL) {
        lastWebSocketSend = current_time;

        String message = "FALLS:" + String(fall_counter) + "|";

        // Aggiungi posizioni target attivi
        bool first_target = true;
        for (int i = 0; i < MAX_TRACKS; i++) {
          if (tracks[i].active) {
            if (!first_target) message += ",";
            // Invia: Height/Range calibrata, poi Lateral position
            message += String(tracks[i].kf_y.x) + "," + String(tracks[i].kf_x.x);
            first_target = false;
          }
        }

        message += "|";

        // Aggiungi posizioni cadute (stesso formato)
        for (int i = 0; i < fall_position_count; i++) {
          if (fall_positions[i].active) {
            if (i > 0) message += ",";
            // Invia: Height/Range dove è caduto, poi Lateral position
            message += String(fall_positions[i].y) + "," + String(fall_positions[i].x);
          }
        }

        // Only send if there are connected clients
        if (webSocket.connectedClients() > 0) {
          webSocket.broadcastTXT(message);
        }
      }
    }
  }
}
// =================================================================
// SETUP e LOOP
// =================================================================
// Handler Unificato per la pagina radice ("/")
void handleRoot() {
  if (currentMode == CONFIG_MODE) {
    server.send(200, "text/html", generateConfigHTML());
  } else {  // TRACKING_MODE
    server.send(200, "text/html", generateTrackingHTML());
  }
}
// SETUP del serial begin.
void setup() {
  Serial.begin(115200);
  delay(100);  // Give serial time to initialize
  radarSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial.println("\n\n========================================");
  Serial.println("    RADAR SYSTEM STARTING UP");
  Serial.println("========================================");

  // Print system information
  printSystemInfo();

  // Load calibration from non-volatile memory (flash)
  Serial.println("[STARTUP] Loading calibration data...");
  loadCalibration();
  Serial.println("[STARTUP] Calibration loaded successfully");

  // Load WiFi credentials from non-volatile memory
  Serial.println("\n[STARTUP] Loading WiFi credentials...");
  wifi_from_storage = loadWiFiCredentials();
  Serial.println("[STARTUP] WiFi credentials loaded");

  // Configure WiFi for better stability
  WiFi.mode(WIFI_STA);  // Station mode only
  WiFi.setAutoReconnect(true);  // Auto-reconnect on disconnect
  WiFi.persistent(false);  // Don't write to flash every connection (faster)

  Serial.println("\n========================================");
  Serial.println("[WIFI] Attempting connection...");
  Serial.printf("[WIFI] SSID: %s\n", clientSSID);
  Serial.println("========================================\n");

  // Try to connect to WiFi
  WiFi.begin(clientSSID, clientPassword);

  int attempts = 0;
  int maxAttempts = 40;  // 20 seconds timeout

  while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
    delay(500);
    Serial.print(".");
    attempts++;

    // Every 10 attempts, print status
    if (attempts % 10 == 0) {
      Serial.printf(" [%d/%d]\n", attempts, maxAttempts);
    }
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    // SUCCESS - Connected to WiFi
    currentMode = TRACKING_MODE;

    Serial.println("\n========================================");
    Serial.println("[WIFI] CONNECTED SUCCESSFULLY!");
    Serial.print("[WIFI] IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("[WIFI] Signal Strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    Serial.println("========================================\n");

    // Save credentials if this was first connection
    if (credentials_submitted) {
      saveWiFiCredentials();
      credentials_submitted = false;
    }

    // Register web handlers
    server.on("/", handleRoot);
    server.on("/submit", handleSubmit);
    server.on("/calibration", handleCalibration);
    server.on("/save-calibration", handleSaveCalibration);
    server.on("/reset-calibration", handleResetCalibration);
    server.begin();

    // Start WebSocket server
    webSocket.begin();
    webSocket.onEvent(onWebSocketEvent);
    last_time = millis();

    Serial.println("[TRACKING_MODE] System ready!");
    Serial.println("[TRACKING_MODE] Access web interface at: http://" + WiFi.localIP().toString());
    Serial.println("========================================\n");

  } else {
    // FAILED - Could not connect to WiFi, start AP mode for configuration
    currentMode = CONFIG_MODE;

    Serial.println("\n========================================");
    Serial.println("[WIFI] Connection FAILED");
    Serial.println("[CONFIG_MODE] Starting Access Point...");
    Serial.println("========================================\n");

    // Stop any existing WiFi connection
    WiFi.disconnect(true);
    delay(100);

    // Start Access Point
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apSSID, apPassword);
    delay(100);

    IPAddress myIP = WiFi.softAPIP();
    Serial.println("\n========================================");
    Serial.println("[CONFIG_MODE] Access Point Active");
    Serial.printf("[CONFIG_MODE] SSID: %s\n", apSSID);
    Serial.printf("[CONFIG_MODE] Password: %s\n", apPassword);
    Serial.print("[CONFIG_MODE] IP Address: ");
    Serial.println(myIP);
    Serial.println("[CONFIG_MODE] Connect to configure WiFi");
    Serial.println("========================================\n");

    // Register web handlers
    server.on("/", handleRoot);
    server.on("/submit", handleSubmit);
    server.on("/calibration", handleCalibration);
    server.on("/save-calibration", handleSaveCalibration);
    server.on("/reset-calibration", handleResetCalibration);
    server.begin();
  }
}

// WiFi reconnection state variables
unsigned long lastWiFiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = 5000;  // Check WiFi every 5 seconds
int wifiReconnectAttempts = 0;
const int MAX_WIFI_RECONNECT_ATTEMPTS = 3;

void loop() {
  // Always handle web requests
  server.handleClient();

  if (currentMode == CONFIG_MODE) {
    // CONFIG MODE: Waiting for WiFi credentials via web interface
    if (credentials_submitted) {
      Serial.println("\n========================================");
      Serial.println("[CONFIG_MODE] New credentials received");
      Serial.println("[CONFIG_MODE] Switching to TRACKING_MODE...");
      Serial.println("========================================\n");

      // Save new credentials
      saveWiFiCredentials();

      // Stop AP and server
      WiFi.softAPdisconnect(true);
      server.stop();
      delay(100);

      // Switch to station mode and connect
      WiFi.mode(WIFI_STA);
      currentMode = TRACKING_MODE;

      Serial.printf("[WIFI] Connecting to: %s\n", clientSSID);
      WiFi.begin(clientSSID, clientPassword);

      int attempts = 0;
      while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print(".");
        attempts++;
        if (attempts % 10 == 0) {
          Serial.printf(" [%d/40]\n", attempts);
        }
      }

      Serial.println();

      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n========================================");
        Serial.println("[WIFI] CONNECTED!");
        Serial.print("[WIFI] IP: ");
        Serial.println(WiFi.localIP());
        Serial.println("========================================\n");

        credentials_submitted = false;

        // Start server and WebSocket
        server.begin();
        webSocket.begin();
        webSocket.onEvent(onWebSocketEvent);
        last_time = millis();
        lastWiFiCheck = millis();
        wifiReconnectAttempts = 0;

        Serial.println("[TRACKING_MODE] System ready!");
      } else {
        Serial.println("\n[ERROR] Connection failed after credential submission");
        Serial.println("[ERROR] Restarting to CONFIG_MODE...\n");
        delay(1000);
        ESP.restart();
      }
    }
    // Small delay to prevent busy-waiting
    delay(50);

  } else if (currentMode == TRACKING_MODE) {
    // TRACKING MODE: Normal operation with WiFi monitoring

    // Periodic WiFi health check (non-blocking)
    unsigned long now = millis();
    if (now - lastWiFiCheck > WIFI_CHECK_INTERVAL) {
      lastWiFiCheck = now;

      if (WiFi.status() != WL_CONNECTED) {
        wifiReconnectAttempts++;
        Serial.printf("\n[WARNING] WiFi disconnected (attempt %d/%d)\n",
                      wifiReconnectAttempts, MAX_WIFI_RECONNECT_ATTEMPTS);

        if (wifiReconnectAttempts <= MAX_WIFI_RECONNECT_ATTEMPTS) {
          Serial.println("[WIFI] Attempting reconnection...");

          // Try to reconnect (non-blocking approach)
          WiFi.disconnect();
          delay(100);
          WiFi.begin(clientSSID, clientPassword);

          int attempts = 0;
          while (WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            Serial.print(".");
            attempts++;
          }

          if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\n[WIFI] Reconnected successfully!");
            Serial.print("[WIFI] IP: ");
            Serial.println(WiFi.localIP());
            wifiReconnectAttempts = 0;  // Reset counter on success
          } else {
            Serial.println("\n[WIFI] Reconnection failed");
          }
        } else {
          // Too many failed attempts, restart system
          Serial.println("\n[ERROR] Max reconnection attempts reached");
          Serial.println("[ERROR] Restarting system...\n");
          delay(1000);
          ESP.restart();
        }
      } else {
        // WiFi is connected, reset attempt counter
        if (wifiReconnectAttempts > 0) {
          wifiReconnectAttempts = 0;
        }
      }
    }

    // WebSocket handling
    webSocket.loop();

    // Heartbeat
    if (now - lastHeartbeat > HEARTBEAT_INTERVAL) {
      webSocket.broadcastTXT("ping");
      lastHeartbeat = now;
    }

    // Radar tracking logic
    run_tracking_logic();

    // Periodic system health check
    if (now - lastSystemCheck > SYSTEM_CHECK_INTERVAL) {
      lastSystemCheck = now;

      // Check free heap
      uint32_t freeHeap = ESP.getFreeHeap();
      if (freeHeap < 20000) {  // Less than 20KB free
        Serial.printf("\n[WARNING] Low memory: %d bytes free\n", freeHeap);
      }

      // Print WiFi signal strength
      if (WiFi.status() == WL_CONNECTED) {
        int rssi = WiFi.RSSI();
        if (rssi < -80) {
          Serial.printf("[WARNING] Weak WiFi signal: %d dBm\n", rssi);
        }
      }
    }

    // Small delay to prevent busy-waiting
    delay(10);
  }
}
