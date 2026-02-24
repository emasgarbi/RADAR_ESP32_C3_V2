# 🚀 Guida Rapida - Sistema Radar con Rilevamento Cadute

## Indice Veloce
1. [Setup Iniziale (5 minuti)](#setup-iniziale)
2. [Configurazione WiFi](#configurazione-wifi)
3. [Calibrazione Cadute](#calibrazione-cadute)
4. [Test Rapido](#test-rapido)
5. [Risoluzione Problemi Comuni](#problemi-comuni)

---

## Setup Iniziale

### 1. Hardware necessario
- ESP32 (ESP32-WROOM o compatibile)
- Sensore Radar (UART, 115200 baud)
- Alimentazione 5V/2A
- Cavi collegamento

### 2. Connessioni
```
ESP32 GPIO 6  →  Radar TX
ESP32 GPIO 7  →  Radar RX
ESP32 GND     →  Radar GND
ESP32 5V      →  Radar VCC
```

### 3. Upload Firmware
1. Apri `Firmware_finale.ino` in Arduino IDE
2. Seleziona board: "ESP32 Dev Module"
3. Seleziona porta COM corretta
4. Clicca "Upload"
5. Attendi "Hard resetting via RTS pin..."

---

## Configurazione WiFi

### Prima Accensione

1. **Accendi ESP32**
   - LED WiFi dovrebbe lampeggiare
   - ESP32 crea AP "RADAR_CONFIG"

2. **Connetti al WiFi temporaneo**
   ```
   SSID: RADAR_CONFIG
   Password: 12345678
   ```

3. **Apri browser**
   - Vai su: `http://192.168.4.1`
   - Vedrai form di configurazione

4. **Inserisci credenziali WiFi**
   - SSID della tua rete
   - Password della tua rete
   - Clicca "Conferma"

5. **ESP32 si riavvia**
   - Si connette alla tua rete
   - Trova IP su Serial Monitor (115200 baud)
   - Esempio: `Connesso al Wi-Fi: 192.168.1.150`

### Accesso Successivi

- Apri browser su: `http://[IP_ESP32]`
- Esempio: `http://192.168.1.150`

---

## Calibrazione Cadute

### Parametri Critici

Apri `Firmware_finale.ino` e trova queste righe (~37-41):

```cpp
#define FALL_VELOCITY_THRESHOLD 150.0      // cm/s
#define FALL_ACCELERATION_THRESHOLD 300.0  // cm/s²
#define FALL_Y_CHANGE_THRESHOLD 80.0       // cm
#define IMMOBILITY_TIME_MS 3000            // ms
```

### Tabella Calibrazione Rapida

| Problema | Modifica | Nuovo Valore |
|----------|----------|--------------|
| **Troppi falsi positivi** (alert quando si siede) | Aumenta tutte le soglie | +20-30% |
| **Cadute non rilevate** | Diminuisci tutte le soglie | -20-30% |
| **Alert troppo lento** | Riduci `IMMOBILITY_TIME_MS` | 2000 ms |
| **Alert troppo rapido** | Aumenta `IMMOBILITY_TIME_MS` | 4000 ms |

### Profili Pre-configurati

#### Per Anziani (>70 anni)
```cpp
#define FALL_VELOCITY_THRESHOLD 100.0
#define FALL_ACCELERATION_THRESHOLD 200.0
#define FALL_Y_CHANGE_THRESHOLD 60.0
#define IMMOBILITY_TIME_MS 2000
```

#### Per Adulti (30-70 anni) - DEFAULT
```cpp
// Valori già presenti nel codice
```

#### Per Ambienti Affollati
```cpp
#define FALL_VELOCITY_THRESHOLD 180.0
#define FALL_ACCELERATION_THRESHOLD 350.0
#define FALL_Y_CHANGE_THRESHOLD 100.0
#define IMMOBILITY_TIME_MS 4000
```

---

## Test Rapido

### Test 1: Tracking Funziona? (2 min)

1. Apri interfaccia web
2. Cammina davanti al radar
3. ✅ Dovresti vedere un pallino rosso che ti segue
4. ✅ Il contatore "People detected" dovrebbe dire "1"

**Problemi?**
- Nessun pallino? → Controlla connessioni UART
- Pallino salta? → Riduci `MATCHING_THRESHOLD` a 60
- Pallini multipli per 1 persona? → Aumenta `MATCHING_THRESHOLD` a 90

### Test 2: Caduta Funziona? (5 min)

⚠️ **SICUREZZA PRIMA DI TUTTO**
- Usa materassi o cuscini
- Non fare se sei solo
- Simulazione, non caduta reale!

1. Stai in piedi davanti al radar
2. **Simulazione:** Scendi rapidamente su ginocchia
3. Resta immobile per 5 secondi
4. ✅ Dopo ~3 secondi dovresti vedere:
   - Alert rosso lampeggiante a schermo
   - Suono di allarme (beep)
   - Log: "🚨 FALL ALERT - Target X"
5. Rialzati e muoviti
6. ✅ Alert dovrebbe sparire

**Problemi?**
- Nessun alert? → Vedi [Calibrazione Cadute](#calibrazione-cadute)
- Alert immediato? → Aumenta `IMMOBILITY_TIME_MS`
- Alert quando ti siedi normale? → Aumenta tutte le soglie del 30%

### Test 3: Multi-Target (3 min)

1. 2-3 persone camminano nell'area
2. ✅ Ognuno dovrebbe avere il proprio pallino
3. ✅ ID non dovrebbero scambiarsi quando si incrociano

**Problemi?**
- ID si scambiano? → Algoritmo Hungarian dovrebbe risolvere (controlla che sia attivo)
- Persone scompaiono? → Aumenta `TIMEOUT_MS` a 3000

---

## Problemi Comuni

### Problema: "WiFi.status() != WL_CONNECTED"

**Causa:** ESP32 non riesce a connettersi

**Soluzioni:**
```
1. Verifica SSID e password corretti
2. Avvicina ESP32 al router
3. Controlla che il WiFi sia 2.4GHz (non 5GHz!)
4. Riavvia ESP32
```

### Problema: "Guru Meditation Error"

**Causa:** Out of memory (RAM insufficiente)

**Soluzione rapida:**
```cpp
// In Firmware_finale.ino (~29)
#define MAX_TRACKS 5  // era 10
```

### Problema: WebSocket disconnette spesso

**Causa:** WiFi instabile o troppi dati

**Soluzione:**
```cpp
// In Firmware_finale.ino (~42)
const unsigned long HEARTBEAT_INTERVAL = 5000;  // era 10000
```

### Problema: Sensore non risponde

**Checklist:**
- ✅ LED sensore acceso?
- ✅ Cavi RX/TX invertiti? (prova a scambiarli)
- ✅ Serial Monitor mostra dati? (imposta 115200 baud)
- ✅ Frame corretti? Cerca "53 59 82 02" nell'hex dump

### Problema: Troppi falsi positivi

**Fix rapido:**
```cpp
// Aumenta tutte le soglie del 30%
#define FALL_VELOCITY_THRESHOLD 195.0      // 150 * 1.3
#define FALL_ACCELERATION_THRESHOLD 390.0  // 300 * 1.3
#define FALL_Y_CHANGE_THRESHOLD 104.0      // 80 * 1.3
```

### Problema: Cadute non rilevate

**Fix rapido:**
```cpp
// Diminuisci tutte le soglie del 30%
#define FALL_VELOCITY_THRESHOLD 105.0      // 150 * 0.7
#define FALL_ACCELERATION_THRESHOLD 210.0  // 300 * 0.7
#define FALL_Y_CHANGE_THRESHOLD 56.0       // 80 * 0.7
```

---

## Serial Monitor - Cosa Cercare

### Output Normale
```
AP IP address: 192.168.4.1
Server in modalità configurazione avviato.
Credenziali inviate. Passaggio a TRACKING_MODE.
Connessione a YourWiFi
.....
Connesso al Wi-Fi
192.168.1.150
```

### Caduta Rilevata
```
⚠️ CADUTA RILEVATA - Target 0: v=180.5 cm/s, a=420.3 cm/s²
🚨 ALERT CADUTA - Target 0: caduto di 95.2 cm e immobile da 3200 ms
✓ Target 0 si è rialzato dopo la caduta
```

### Errori da Non Ignorare
```
❌ Riconnessione fallita. Riavvio...         → Problema WiFi grave
❌ Guru Meditation Error                      → Out of memory
❌ ESP32: Brownout detector was triggered     → Alimentazione insufficiente
```

---

## Comandi Debug Utili

### 1. Verifica Memoria
```cpp
// Aggiungi nel loop():
Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
```

### 2. Log Velocità e Accelerazione
```cpp
// In detectFall(), prima di if(!track.fall_detected):
Serial.printf("T%d: v=%.1f a=%.1f\n", track.id, vertical_velocity, vertical_acceleration);
```

### 3. Test Singola Funzione
```cpp
void loop() {
  // Commenta tutto tranne:
  run_tracking_logic();
  delay(10);
}
```

---

## Checklist Pre-Deployment

Prima di installare definitivamente:

- [ ] Tracking stabile per 30 minuti
- [ ] Rilevamento caduta testato 5 volte (con protezioni!)
- [ ] Zero falsi positivi in 1 ora di movimenti normali
- [ ] WiFi riconnette automaticamente dopo interruzione
- [ ] WebSocket stabile per 2 ore
- [ ] Alert visivo e sonoro funzionano
- [ ] Calibrazione ottimale per l'ambiente specifico
- [ ] Documentato IP e credenziali WiFi
- [ ] Piano di manutenzione definito

---

## Link Utili

- **Documentazione Completa:** `DOCUMENTAZIONE_MODIFICHE.md`
- **Esempi Codice:** `ESEMPI_UTILIZZO.md`
- **Codice Sorgente:** `Firmware_finale.ino`

---

## Supporto Rapido

### Ho bisogno di aiuto NOW!

1. **Controlla Serial Monitor** (115200 baud) - Ti dice esattamente cosa non va
2. **Riavvia ESP32** - Risolve 70% dei problemi
3. **Verifica connessioni** - UART è la causa #1 di problemi
4. **Consulta Troubleshooting** nella documentazione completa

### Contatti Emergenza

```
Email: [TUO_SUPPORTO]
Forum: [TUO_FORUM]
GitHub Issues: [TUO_REPO]
```

---

**Ultima revisione:** 20 Novembre 2025

*Per dettagli tecnici completi, consulta `DOCUMENTAZIONE_MODIFICHE.md`*
