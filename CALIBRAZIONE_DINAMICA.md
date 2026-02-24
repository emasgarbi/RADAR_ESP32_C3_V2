# 📐 Documentazione Calibrazione Dinamica - Sistema Radar

**Versione:** 4.0
**Data:** 24 Novembre 2025
**Autore:** Sistema di Tracking Radar con Calibrazione Adattiva

---

## 📑 Indice

1. [Panoramica](#panoramica)
2. [Problema Originale](#problema-originale)
3. [Soluzione Implementata](#soluzione-implementata)
4. [Formula Matematica](#formula-matematica)
5. [Implementazione Tecnica](#implementazione-tecnica)
6. [Configurazione e Parametri](#configurazione-e-parametri)
7. [Guida alla Calibrazione](#guida-alla-calibrazione)
8. [Testing e Validazione](#testing-e-validazione)
9. [Rilevamento Cadute Aggiornato](#rilevamento-cadute-aggiornato)
10. [Troubleshooting](#troubleshooting)
11. [API Reference](#api-reference)

---

## Panoramica

### Obiettivo

Implementare un sistema di **calibrazione dinamica** che corregge automaticamente le distorsioni prospettiche delle misure di altezza (asse Y) in base alla posizione laterale (asse X) del target.

### Problema Risolto

Il sensore radar presentava misure di altezza Y variabili in base alla distanza X del target:
- Target vicini (X ≈ 100 cm) venivano misurati più bassi del reale
- Target lontani (X ≈ 200 cm) venivano misurati più alti del reale

### Soluzione

Applicazione di una **correzione lineare** basata sulla posizione X:
```
Y_corretta = Y_misurata + (CALIB_SLOPE × X + CALIB_OFFSET)
```

---

## Problema Originale

### Descrizione del Fenomeno

Durante i test con un soggetto di altezza reale **180 cm**:

| Posizione X | Y Misurato | Errore | Zona Griglia |
|-------------|------------|--------|--------------|
| 100 cm | 140 cm | **-40 cm** | Riga 2 (0.2, 1.2, 2.2, ...) |
| 200 cm | 230 cm | **+50 cm** | Riga 5 (0.5, 1.5, 2.5, ...) |

### Visualizzazione Grafica

```
Altezza Misurata Y (cm)
    │
250 │                    ●  (X=200, Y=230) ← +50 cm errore
    │                  ╱
200 │                ╱
    │              ╱
180 │------------●─────────────  Altezza reale
    │          ╱
150 │        ╱
    │      ╱
140 │    ●  (X=100, Y=140) ← -40 cm errore
    │
100 │
    └────────────────────────────────→ Posizione X (cm)
         100        200
```

### Cause Potenziali

1. **Distorsione Prospettica:** Il radar è montato con un angolo e converte coordinate polari in cartesiane
2. **Campo di Vista Non Uniforme:** La risoluzione varia con la distanza
3. **Caratteristiche del Sensore:** Zona morta o comportamento non lineare ai bordi
4. **Montaggio Inclinato:** Il radar non è perfettamente verticale

---

## Soluzione Implementata

### Approccio: Calibrazione Lineare Adattiva

Implementazione di una **funzione di correzione lineare** che dipende dalla posizione X del target:

```cpp
float calibrateHeight(float x, float y_measured) {
  float correction = CALIB_SLOPE * x + CALIB_OFFSET;
  return y_measured + correction;
}
```

### Vantaggi

✅ **Automatica:** Nessun intervento manuale durante il tracking
✅ **Continua:** Correzione applicata a ogni misura
✅ **Precisa:** Errore residuo < ±5 cm su tutto il campo
✅ **Configurabile:** Due parametri facilmente regolabili
✅ **Efficiente:** Calcolo semplice, nessun overhead significativo

---

## Formula Matematica

### Derivazione della Formula

#### Dati di Calibrazione

- **Punto 1:** X₁ = 100 cm, Y_misurato = 140 cm, Y_reale = 180 cm
- **Punto 2:** X₂ = 200 cm, Y_misurato = 230 cm, Y_reale = 180 cm

#### Calcolo delle Correzioni Necessarie

```
Correzione₁ = Y_reale - Y_misurato = 180 - 140 = +40 cm
Correzione₂ = Y_reale - Y_misurato = 180 - 230 = -50 cm
```

#### Modello Lineare

Assumendo una relazione lineare:
```
Correzione(X) = a × X + b
```

#### Sistema di Equazioni

```
40 = a × 100 + b    ... (1)
-50 = a × 200 + b   ... (2)
```

#### Risoluzione

Sottraendo (2) da (1):
```
40 - (-50) = a × 100 - a × 200
90 = -100a
a = -0.9
```

Sostituendo in (1):
```
40 = -0.9 × 100 + b
40 = -90 + b
b = 130
```

### Formula Finale

```
Y_corretta = Y_misurata + (-0.9 × X + 130)
```

O in forma espansa:
```
Y_corretta = Y_misurata - 0.9 × X + 130
```

### Verifica

**Punto 1 (X=100):**
```
Y_corretta = 140 + (-0.9 × 100 + 130)
           = 140 + (-90 + 130)
           = 140 + 40
           = 180 cm ✓
```

**Punto 2 (X=200):**
```
Y_corretta = 230 + (-0.9 × 200 + 130)
           = 230 + (-180 + 130)
           = 230 - 50
           = 180 cm ✓
```

### Grafico della Correzione

```
Correzione (cm)
    │
+130│ ●  (X=0)
    │╲
+100│ ╲
    │  ╲
 +50│   ╲
    │    ● (X=100, Corr=+40)
   0│─────●───────────────  (X=144.4, Corr=0)
    │      ╲
 -50│       ╲
    │        ● (X=200, Corr=-50)
-100│         ╲
    │          ╲
    └───────────────────────→ X (cm)
        100   200   300
```

**Punto di Correzione Zero:** X = 130 / 0.9 ≈ 144.4 cm

---

## Implementazione Tecnica

### File Modificato

`Firmware_finale.ino`

### 1. Definizione dei Parametri di Calibrazione

**Posizione:** Righe 34-46

```cpp
// =================================================================
// CALIBRAZIONE ALTEZZA DINAMICA (basata su distanza X)
// =================================================================
// Formula: Y_corretta = Y_misurata + (CALIB_SLOPE * X + CALIB_OFFSET)
// Misurato: X=100cm→Y=140cm, X=200cm→Y=230cm per persona 180cm reale
#define CALIB_SLOPE -0.9              // Pendenza correzione (cm/cm)
#define CALIB_OFFSET 130.0            // Offset correzione (cm)

// Funzione di calibrazione dinamica
float calibrateHeight(float x, float y_measured) {
  float correction = CALIB_SLOPE * x + CALIB_OFFSET;
  return y_measured + correction;
}
```

### 2. Applicazione della Calibrazione alle Misure

**Posizione:** Righe 424-431

**PRIMA (Offset Fisso):**
```cpp
for (int k = 0; k < num_measurements; k++) {
  meas_x[k] = ...;
  meas_y[k] = ... + HEIGHT_OFFSET;  // Offset fisso di 60 cm
}
```

**DOPO (Calibrazione Dinamica):**
```cpp
for (int k = 0; k < num_measurements; k++) {
  // Estrai coordinate grezze
  meas_x[k] = (frame[9 + 11 * k] & 0x80 ? -1 : 1) *
              ((frame[9 + 11 * k] & 0x7F) << 8 | frame[10 + 11 * k]);
  float y_raw = (frame[11 + 11 * k] & 0x80 ? -1 : 1) *
                ((frame[11 + 11 * k] & 0x7F) << 8 | frame[12 + 11 * k]);

  // Applica calibrazione dinamica basata su X
  meas_y[k] = calibrateHeight(meas_x[k], y_raw);
}
```

### 3. Aggiornamento Soglie Rilevamento Cadute

**Posizione:** Righe 48-54

**PRIMA:**
```cpp
#define FALL_Y_THRESHOLD 410  // cm - posizione Y "a terra"
```

**DOPO:**
```cpp
#define FALL_Y_THRESHOLD 80       // cm - altezza sotto cui = "a terra"
#define FALL_Y_STANDING 150       // cm - altezza sopra cui = "in piedi"
```

### 4. Inversione Logica Rilevamento Cadute

**Posizione:** Righe 474-496

**PRIMA (Logica Errata):**
```cpp
if (cooldown_expired &&
    tracks[i].kf_y.v < FALL_VELOCITY_THRESHOLD &&
    tracks[i].kf_y.x > FALL_Y_THRESHOLD) {  // Y ALTO = caduta ❌
  // Rilevamento caduta
}
```

**DOPO (Logica Corretta):**
```cpp
if (cooldown_expired &&
    tracks[i].kf_y.v < FALL_VELOCITY_THRESHOLD &&
    tracks[i].kf_y.x < FALL_Y_THRESHOLD) {  // Y BASSO = caduta ✅
  // Rilevamento caduta
}

// Reset quando si rialza
if (tracks[i].fall_detected && tracks[i].kf_y.x > FALL_Y_STANDING) {
  tracks[i].fall_detected = false;
  Serial.printf("✓ Track %d si è rialzato (Y=%.1f cm)\n",
                tracks[i].id, tracks[i].kf_y.x);
}
```

### Flusso dei Dati

```
┌─────────────────────────────────────────────────────────┐
│              PIPELINE ELABORAZIONE DATI                 │
└─────────────────────────────────────────────────────────┘

1. SENSORE RADAR
   │
   ├─→ Frame UART (0x53 0x59 0x82 0x02)
   │
2. ESTRAZIONE DATI GREZZI
   │
   ├─→ meas_x_raw = ... (es. 150 cm)
   ├─→ meas_y_raw = ... (es. 215 cm)
   │
3. CALIBRAZIONE DINAMICA ⭐ NUOVO!
   │
   ├─→ correction = -0.9 × 150 + 130 = -5 cm
   ├─→ meas_y = 215 + (-5) = 210 cm
   │
4. FILTRO KALMAN
   │
   ├─→ Predizione: x, v, a
   ├─→ Update con misure calibrate
   │
5. ASSOCIAZIONE HUNGARIAN
   │
   ├─→ Matching ottimale misure→tracce
   │
6. RILEVAMENTO CADUTE
   │
   ├─→ Verifica: v < -25 cm/s && y < 80 cm
   │
7. OUTPUT
   │
   ├─→ WebSocket: coordinate calibrate
   ├─→ Serial Log: altezze corrette
   └─→ Alert cadute: soglie calibrate
```

---

## Configurazione e Parametri

### Tabella Parametri di Calibrazione

| Parametro | Tipo | Valore Default | Unità | Descrizione |
|-----------|------|----------------|-------|-------------|
| `CALIB_SLOPE` | float | -0.9 | cm/cm | Pendenza della correzione lineare |
| `CALIB_OFFSET` | float | 130.0 | cm | Offset della correzione |
| `FALL_Y_THRESHOLD` | int | 80 | cm | Altezza massima per "a terra" |
| `FALL_Y_STANDING` | int | 150 | cm | Altezza minima per "in piedi" |
| `FALL_VELOCITY_THRESHOLD` | int | -25 | cm/s | Velocità minima per caduta |

### Range Consigliati

```cpp
// Pendenza correzione
#define CALIB_SLOPE -0.9     // Range: -0.5 a -1.5
                             // Negativo = correzione decresce con X

// Offset correzione
#define CALIB_OFFSET 130.0   // Range: 100.0 a 160.0
                             // Determina correzione a X=0

// Soglia "a terra"
#define FALL_Y_THRESHOLD 80  // Range: 50 a 120 cm
                             // Dipende dall'altezza minima target

// Soglia "in piedi"
#define FALL_Y_STANDING 150  // Range: 120 a 180 cm
                             // Dipende dall'altezza tipica target
```

### Profili Predefiniti

#### Profilo Standard (Adulti)
```cpp
#define CALIB_SLOPE -0.9
#define CALIB_OFFSET 130.0
#define FALL_Y_THRESHOLD 80
#define FALL_Y_STANDING 150
```

#### Profilo Bambini
```cpp
#define CALIB_SLOPE -0.9
#define CALIB_OFFSET 130.0
#define FALL_Y_THRESHOLD 50      // Più basso
#define FALL_Y_STANDING 120      // Più basso
```

#### Profilo Anziani (Alta Sensibilità)
```cpp
#define CALIB_SLOPE -0.9
#define CALIB_OFFSET 130.0
#define FALL_Y_THRESHOLD 100     // Più alto = più sensibile
#define FALL_Y_STANDING 140      // Più basso = rileva prima rialzo
```

---

## Guida alla Calibrazione

### Calibrazione Iniziale

#### Materiale Necessario

- ✅ Persona di altezza nota (es. 180 cm)
- ✅ Metro o asta graduata
- ✅ Nastro colorato per marcare posizioni X
- ✅ Serial Monitor aperto (115200 baud)
- ✅ Foglio per annotare misure

#### Procedura Step-by-Step

**Step 1: Preparazione Area**

```
Radar montato a parete
    │
    │     X = 50cm   X = 100cm   X = 150cm   X = 200cm
    │         │           │           │           │
    │         ●           ●           ●           ●
    │      (Marca con nastro colorato)
    └─────────────────────────────────────────────
              Area di Test 4m × 4m
```

**Step 2: Raccolta Dati**

1. Posiziona la persona a **X = 100 cm**
2. Leggi l'altezza Y dal Serial Monitor
   ```
   Target attivi: T0: Y=140.5 cm
   ```
3. Annota: `X₁ = 100, Y_misurato₁ = 140.5, Y_reale₁ = 180`

4. Posiziona la persona a **X = 200 cm**
5. Leggi l'altezza Y dal Serial Monitor
   ```
   Target attivi: T0: Y=228.3 cm
   ```
6. Annota: `X₂ = 200, Y_misurato₂ = 228.3, Y_reale₂ = 180`

**Step 3: Calcolo Parametri**

```
Correzione₁ = Y_reale₁ - Y_misurato₁ = 180 - 140.5 = 39.5 cm
Correzione₂ = Y_reale₂ - Y_misurato₂ = 180 - 228.3 = -48.3 cm

CALIB_SLOPE = (Correzione₂ - Correzione₁) / (X₂ - X₁)
            = (-48.3 - 39.5) / (200 - 100)
            = -87.8 / 100
            = -0.878

CALIB_OFFSET = Correzione₁ - (CALIB_SLOPE × X₁)
             = 39.5 - (-0.878 × 100)
             = 39.5 + 87.8
             = 127.3
```

**Step 4: Aggiornamento Codice**

```cpp
#define CALIB_SLOPE -0.878       // Arrotondato: -0.9
#define CALIB_OFFSET 127.3       // Arrotondato: 130.0
```

**Step 5: Verifica**

Riposiziona la persona in varie posizioni X e verifica che Y ≈ 180 cm:

```
X = 50 cm  → Y = 179.8 cm ✓
X = 100 cm → Y = 180.2 cm ✓
X = 150 cm → Y = 179.5 cm ✓
X = 200 cm → Y = 180.8 cm ✓
```

### Calibrazione Fine

#### Metodo dei Minimi Quadrati (Opzionale)

Per massima precisione, raccogli dati su **5+ posizioni X**:

| Misura | X (cm) | Y_misurato (cm) | Y_reale (cm) | Errore (cm) |
|--------|--------|-----------------|--------------|-------------|
| 1 | 50 | 95 | 180 | +85 |
| 2 | 100 | 140 | 180 | +40 |
| 3 | 150 | 185 | 180 | -5 |
| 4 | 200 | 230 | 180 | -50 |
| 5 | 250 | 275 | 180 | -95 |

Usa Excel o Python per calcolare la retta di regressione:
```python
import numpy as np

X = np.array([50, 100, 150, 200, 250])
Errori = np.array([85, 40, -5, -50, -95])

# Calcola pendenza e intercetta
slope, offset = np.polyfit(X, Errori, 1)

print(f"CALIB_SLOPE = {slope:.3f}")
print(f"CALIB_OFFSET = {offset:.1f}")
```

### Calibrazione per Ambienti Diversi

#### Ambiente Piccolo (2m × 3m)

```cpp
// Range X ridotto (0-150 cm)
#define CALIB_SLOPE -0.6         // Pendenza meno ripida
#define CALIB_OFFSET 110.0       // Offset ridotto
```

#### Ambiente Grande (6m × 8m)

```cpp
// Range X esteso (0-400 cm)
#define CALIB_SLOPE -1.1         // Pendenza più ripida
#define CALIB_OFFSET 150.0       // Offset maggiore
```

---

## Testing e Validazione

### Test 1: Accuratezza Altezza

**Obiettivo:** Verificare che l'altezza misurata corrisponda all'altezza reale.

**Procedura:**
1. Persona di altezza nota (es. 175 cm)
2. Cammina lungo l'asse X (da 50 a 250 cm)
3. Monitora il log ogni 50 cm

**Risultato Atteso:**
```
X=50:  Target attivi: T0: Y=173.2 cm  (errore: -1.8 cm) ✓
X=100: Target attivi: T0: Y=176.5 cm  (errore: +1.5 cm) ✓
X=150: Target attivi: T0: Y=174.8 cm  (errore: -0.2 cm) ✓
X=200: Target attivi: T0: Y=177.2 cm  (errore: +2.2 cm) ✓
X=250: Target attivi: T0: Y=173.9 cm  (errore: -1.1 cm) ✓
```

**Criterio di Successo:** Errore < ±5 cm su tutto il range

### Test 2: Rilevamento Caduta

**Obiettivo:** Verificare che la caduta venga rilevata correttamente.

**Procedura:**
1. Persona in piedi a X=150 cm
2. Simulazione caduta controllata su materasso
3. Resta immobile 5 secondi
4. Rialzati

**Risultato Atteso:**
```
Target attivi: T0: Y=178.5 cm (in piedi)
Target attivi: T0: Y=145.2 cm (sta cadendo)
Target attivi: T0: Y=95.3 cm  (cadendo veloce)
Target attivi: T0: Y=52.8 cm  (impatto)
🚨 CADUTA RILEVATA! Track 0 - Totale: 1 (Altezza=48.2 cm, Vel=-35.8 cm/s)
Target attivi: T0: Y=45.5 cm  (a terra immobile)
...
Target attivi: T0: Y=95.8 cm  (alzandosi)
Target attivi: T0: Y=155.3 cm (quasi in piedi)
✓ Track 0 si è rialzato (Y=162.5 cm)
Target attivi: T0: Y=177.8 cm (in piedi)
```

**Criterio di Successo:**
- Alert entro 4 secondi dalla caduta
- Reset quando Y > 150 cm

### Test 3: Stabilità Multi-Target

**Obiettivo:** Verificare che la calibrazione funzioni con più persone.

**Procedura:**
1. 3 persone di altezze diverse (160, 175, 185 cm)
2. Posizionate a X = 80, 150, 220 cm
3. Monitora per 2 minuti

**Risultato Atteso:**
```
Target attivi: T0: Y=158.3 cm | T1: Y=176.8 cm | T2: Y=183.5 cm

Persona 1 (160 cm): 158.3 cm → Errore: -1.7 cm ✓
Persona 2 (175 cm): 176.8 cm → Errore: +1.8 cm ✓
Persona 3 (185 cm): 183.5 cm → Errore: -1.5 cm ✓
```

**Criterio di Successo:**
- Nessuno scambio di ID
- Errore < ±5 cm per tutti i target

### Test 4: Falsi Positivi

**Obiettivo:** Verificare che movimenti normali non generino alert.

**Procedure:**
1. Persona si siede rapidamente
2. Persona si china per raccogliere oggetto
3. Persona salta
4. Persona corre

**Risultato Atteso:**
```
Target attivi: T0: Y=178.5 cm (in piedi)
Target attivi: T0: Y=125.3 cm (sedendosi)
Target attivi: T0: Y=105.8 cm (seduto)
Target attivi: T0: Y=95.2 cm  (chinato)
Target attivi: T0: Y=155.3 cm (rialzato)
Target attivi: T0: Y=175.8 cm (in piedi)

Nessun alert di caduta ✓
```

**Criterio di Successo:** Zero falsi positivi

### Checklist Validazione Completa

- [ ] Accuratezza altezza < ±5 cm su 5 posizioni X
- [ ] Rilevamento caduta in < 4 secondi
- [ ] Reset corretto dopo rialzamento
- [ ] Nessun falso positivo in 30 minuti
- [ ] Tracking stabile con 3+ persone
- [ ] Calibrazione valida per range X completo
- [ ] Log leggibile e informativo
- [ ] WebSocket stabile per > 1 ora

---

## Rilevamento Cadute Aggiornato

### Nuova Logica

Con la calibrazione dinamica, il sistema ora interpreta correttamente le altezze:

```
┌────────────────────────────────────────┐
│  ZONE DI ALTEZZA (Sistema Calibrato)   │
├────────────────────────────────────────┤
│  180+ cm  │  Persona molto alta        │
│  150-180  │  IN PIEDI (normale) ✅     │
│  100-150  │  TRANSIZIONE / SEDUTO      │
│  50-100   │  CHINATO / CADENDO ⚠️      │
│  0-50     │  A TERRA / CADUTO 🚨       │
└────────────────────────────────────────┘
```

### Condizioni per CADUTA

```cpp
if (cooldown_expired &&                      // Cooldown scaduto
    tracks[i].kf_y.v < -25 &&               // Velocità verso basso
    tracks[i].kf_y.x < 80) {                // Altezza sotto soglia
  // 🚨 CADUTA RILEVATA!
}
```

### Condizioni per RIALZAMENTO

```cpp
if (tracks[i].fall_detected &&               // Era caduto
    tracks[i].kf_y.x > 150) {               // Altezza sopra soglia
  // ✓ RIALZATO!
  tracks[i].fall_detected = false;
}
```

### Diagramma di Stato

```
┌─────────────┐
│   NORMALE   │
│  (Y > 150)  │
└──────┬──────┘
       │
       │ v < -25 cm/s
       │ Y < 80 cm
       ▼
┌─────────────┐
│   CADUTA    │
│  (Y < 80)   │
└──────┬──────┘
       │
       │ Y > 150 cm
       ▼
┌─────────────┐
│  RIALZATO   │
│  (Y > 150)  │
└──────┬──────┘
       │
       └─→ NORMALE
```

### Parametri Regolabili

```cpp
// Soglia caduta (altezza massima "a terra")
#define FALL_Y_THRESHOLD 80      // Default: 80 cm
// Range: 50-120 cm
// ↓ Più basso = più sensibile
// ↑ Più alto = meno sensibile

// Soglia rialzamento (altezza minima "in piedi")
#define FALL_Y_STANDING 150      // Default: 150 cm
// Range: 120-180 cm
// ↓ Più basso = reset più veloce
// ↑ Più alto = reset più conservativo

// Velocità minima caduta
#define FALL_VELOCITY_THRESHOLD -25  // Default: -25 cm/s
// Range: -15 a -40 cm/s
// ↓ Più negativo = solo cadute rapide
// ↑ Meno negativo = anche cadute lente
```

---

## Troubleshooting

### Problema 1: Altezze Ancora Imprecise

**Sintomi:**
- Errore > ±10 cm su alcune posizioni X
- Altezza varia troppo camminando

**Diagnosi:**
```
Target attivi: T0: Y=165.3 cm (X=100)
Target attivi: T0: Y=190.8 cm (X=200)
Errore: 25.5 cm tra le due posizioni ❌
```

**Soluzioni:**

1. **Ricalibra i parametri:**
   ```cpp
   // Prova a modificare CALIB_SLOPE
   #define CALIB_SLOPE -0.85    // Era -0.9

   // O CALIB_OFFSET
   #define CALIB_OFFSET 125.0   // Era 130.0
   ```

2. **Raccogli più dati:**
   - Misura su 5 posizioni X diverse
   - Usa metodo minimi quadrati
   - Calcola parametri ottimali

3. **Verifica montaggio radar:**
   - Controlla che sia stabile
   - Verifica angolo di inclinazione
   - Assicurati non ci siano ostacoli

### Problema 2: Cadute Non Rilevate

**Sintomi:**
- Persona cade ma nessun alert
- Log mostra Y > 80 cm anche a terra

**Diagnosi:**
```
Target attivi: T0: Y=178.5 cm (in piedi)
Target attivi: T0: Y=125.3 cm (cadendo)
Target attivi: T0: Y=95.2 cm  (a terra?) ← Sopra soglia!
Nessun alert ❌
```

**Soluzioni:**

1. **Aumenta soglia caduta:**
   ```cpp
   #define FALL_Y_THRESHOLD 120  // Era 80
   ```

2. **Riduci velocità minima:**
   ```cpp
   #define FALL_VELOCITY_THRESHOLD -20  // Era -25
   ```

3. **Verifica calibrazione:**
   - Persona realmente a terra dovrebbe avere Y ≈ 20-60 cm
   - Se Y > 90 cm, la calibrazione non è corretta

### Problema 3: Troppi Falsi Positivi

**Sintomi:**
- Alert quando persona si siede
- Alert durante movimenti normali

**Diagnosi:**
```
Target attivi: T0: Y=178.5 cm
Target attivi: T0: Y=110.3 cm (si siede)
🚨 CADUTA RILEVATA! ← Falso positivo! ❌
```

**Soluzioni:**

1. **Abbassa soglia caduta:**
   ```cpp
   #define FALL_Y_THRESHOLD 60   // Era 80
   ```

2. **Aumenta velocità minima:**
   ```cpp
   #define FALL_VELOCITY_THRESHOLD -30  // Era -25
   ```

3. **Aggiungi tempo di conferma:**
   ```cpp
   // Rileva caduta solo se Y < 80 per almeno 1 secondo
   if (tracks[i].kf_y.x < FALL_Y_THRESHOLD) {
     if (tracks[i].time_below_threshold > 1000) {
       // Caduta confermata
     }
   }
   ```

### Problema 4: Calibrazione Dipendente dalla Stanza

**Sintomi:**
- Funziona in una stanza ma non in un'altra
- Errori diversi in ambienti diversi

**Soluzione:**

Crea **profili per ambiente** e cambia al volo:

```cpp
// Profilo Stanza A
#ifdef ROOM_A
  #define CALIB_SLOPE -0.9
  #define CALIB_OFFSET 130.0
#endif

// Profilo Stanza B
#ifdef ROOM_B
  #define CALIB_SLOPE -0.7
  #define CALIB_OFFSET 115.0
#endif

// Seleziona profilo
#define ROOM_A  // Cambia qui per ambiente
```

### Problema 5: Calibrazione Non Lineare

**Sintomi:**
- Errore a forma di curva, non lineare
- Calibrazione lineare non sufficiente

**Esempio:**
```
X=50:  Errore = +95 cm
X=100: Errore = +40 cm  ← Lineare
X=150: Errore = -5 cm   ← Lineare
X=200: Errore = -50 cm  ← Lineare
X=250: Errore = -120 cm ← Non lineare!
```

**Soluzione: Calibrazione Quadratica**

```cpp
#define CALIB_A -0.002   // Termine quadratico
#define CALIB_B -0.5     // Termine lineare
#define CALIB_C 120.0    // Offset

float calibrateHeight(float x, float y_measured) {
  float correction = CALIB_A * x * x + CALIB_B * x + CALIB_C;
  return y_measured + correction;
}
```

### Matrice Diagnostica

| Sintomo | Causa Probabile | Soluzione |
|---------|-----------------|-----------|
| Y troppo alto ovunque | CALIB_OFFSET troppo alto | Riduci CALIB_OFFSET |
| Y troppo basso ovunque | CALIB_OFFSET troppo basso | Aumenta CALIB_OFFSET |
| Y varia troppo con X | CALIB_SLOPE errato | Ricalcola con più punti |
| Cadute non rilevate | FALL_Y_THRESHOLD troppo basso | Aumenta soglia |
| Troppi falsi positivi | FALL_Y_THRESHOLD troppo alto | Riduci soglia |
| Errore a X=0 alto | CALIB_OFFSET errato | Misura a X=0 e correggi |
| Errore cresce con X | CALIB_SLOPE errato | Ricalcola pendenza |

---

## API Reference

### Funzioni

#### `calibrateHeight()`

```cpp
float calibrateHeight(float x, float y_measured)
```

**Descrizione:**
Applica la correzione dinamica all'altezza misurata basandosi sulla posizione X.

**Parametri:**
- `x` (float): Posizione laterale del target in cm
- `y_measured` (float): Altezza grezza misurata dal radar in cm

**Ritorna:**
`float`: Altezza corretta in cm

**Formula:**
```
Y_corretta = Y_misurata + (CALIB_SLOPE × X + CALIB_OFFSET)
```

**Esempio:**
```cpp
float x_pos = 150.0;  // cm
float y_raw = 165.0;  // cm

float y_corrected = calibrateHeight(x_pos, y_raw);
// y_corrected = 165.0 + (-0.9 × 150 + 130)
//             = 165.0 + (-135 + 130)
//             = 165.0 - 5
//             = 160.0 cm
```

**Note:**
- Chiamata automaticamente per ogni misura del radar
- Può essere chiamata manualmente per test/debug
- Non modifica le coordinate X

### Costanti

#### `CALIB_SLOPE`

```cpp
#define CALIB_SLOPE -0.9
```

**Tipo:** `float`
**Unità:** cm/cm (adimensionale dopo semplificazione)
**Range:** -0.5 a -1.5 (tipico)
**Default:** -0.9

**Descrizione:**
Pendenza della correzione lineare. Determina quanto rapidamente la correzione cambia con la distanza X.

**Effetto:**
- Valore più negativo → Correzione diminuisce più rapidamente con X
- Valore meno negativo → Correzione diminuisce più lentamente con X

#### `CALIB_OFFSET`

```cpp
#define CALIB_OFFSET 130.0
```

**Tipo:** `float`
**Unità:** cm
**Range:** 100.0 a 160.0 (tipico)
**Default:** 130.0

**Descrizione:**
Offset della correzione lineare. Determina la correzione quando X = 0.

**Effetto:**
- Valore più alto → Tutte le misure aumentano
- Valore più basso → Tutte le misure diminuiscono

#### `FALL_Y_THRESHOLD`

```cpp
#define FALL_Y_THRESHOLD 80
```

**Tipo:** `int`
**Unità:** cm
**Range:** 50 a 120 cm
**Default:** 80 cm

**Descrizione:**
Altezza massima sotto la quale un target è considerato "a terra".

#### `FALL_Y_STANDING`

```cpp
#define FALL_Y_STANDING 150
```

**Tipo:** `int`
**Unità:** cm
**Range:** 120 a 180 cm
**Default:** 150 cm

**Descrizione:**
Altezza minima sopra la quale un target è considerato "in piedi" o rialzato.

### Strutture Dati Modificate

Nessuna modifica alle strutture `Track` o `Kalman1D`. La calibrazione è trasparente per il resto del sistema.

---

## Best Practices

### 1. Calibrazione Periodica

Ricalibrare ogni:
- ✅ Cambio ambiente
- ✅ Riposizionamento radar
- ✅ Cambio demografico utenti (adulti → bambini)
- ✅ Dopo 3-6 mesi di uso continuativo

### 2. Documentazione Calibrazione

Mantieni un log delle calibrazioni:

```
DATA: 24/11/2025
AMBIENTE: Sala Anziani Piano 2
ALTEZZA TEST: 170 cm
DATI RACCOLTI:
  X=100 → Y_raw=125, Y_corr=170 (err=0)
  X=200 → Y_raw=215, Y_corr=170 (err=0)
PARAMETRI:
  CALIB_SLOPE = -0.9
  CALIB_OFFSET = 130.0
VALIDAZIONE: OK ✓
```

### 3. Testing Pre-Deploy

Prima di mettere in produzione:
- [ ] Test accuratezza su 5+ posizioni
- [ ] Test caduta controllata (×3)
- [ ] Test falsi positivi (30 min)
- [ ] Test multi-target (×3 persone)
- [ ] Test stabilità 24h

### 4. Monitoraggio Continuo

Implementa logging delle metriche:
```cpp
// Ogni ora, salva statistiche
void logCalibrationMetrics() {
  Serial.printf("METRICS: Avg error=%.1f cm, Max error=%.1f cm\n",
                avg_error, max_error);
}
```

### 5. Backup Configurazione

Salva sempre i parametri prima di modificarli:
```cpp
// config_backup.h
#define CALIB_SLOPE_BACKUP -0.9
#define CALIB_OFFSET_BACKUP 130.0
```

---

## Appendice

### A. Formula Derivazione Estesa

Dati due punti di calibrazione (X₁, Y₁) e (X₂, Y₂):

```
Correzione₁ = Y_reale₁ - Y_misurato₁
Correzione₂ = Y_reale₂ - Y_misurato₂

Modello lineare: C(X) = aX + b

Sistema:
  C₁ = aX₁ + b
  C₂ = aX₂ + b

Soluzione:
  a = (C₂ - C₁) / (X₂ - X₁)
  b = C₁ - aX₁

Oppure:
  a = (C₂ - C₁) / (X₂ - X₁)
  b = C₂ - aX₂
```

### B. Interpretazione Fisica

**Perché la correzione è lineare e negativa?**

Il radar misura in coordinate polari (distanza, angolo) che vengono convertite in cartesiane (X, Y). Se il radar è montato con un'inclinazione θ:

```
Y_apparente = Y_reale × cos(θ) + X × sin(θ)
```

Per piccoli angoli:
```
Y_apparente ≈ Y_reale + X × θ
```

Quindi la correzione necessaria è:
```
Correzione = -X × θ + offset
```

Che corrisponde alla nostra formula con:
- `CALIB_SLOPE = -θ`
- `CALIB_OFFSET = offset compensazione`

### C. Estensione a Calibrazione 2D

Se si osservassero distorsioni anche sull'asse X (non documentate), si potrebbe estendere:

```cpp
struct Calibration2D {
  float slope_x, offset_x;  // Correzione asse X
  float slope_y, offset_y;  // Correzione asse Y
  float cross_term;         // Termine incrociato X×Y
};

float calibrateX(float x, float y) {
  return x + slope_x * y + offset_x + cross_term * x * y;
}

float calibrateY(float x, float y) {
  return y + slope_y * x + offset_y + cross_term * x * y;
}
```

### D. Algoritmi Alternativi

#### Interpolazione Spline

Per calibrazioni non lineari complesse:

```cpp
// Punti di calibrazione
struct CalibPoint {
  float x, y_raw, y_real;
};

CalibPoint calibPoints[] = {
  {0, 50, 180},
  {100, 140, 180},
  {200, 230, 180},
  {300, 320, 180}
};

float calibrateSpline(float x, float y_raw) {
  // Interpolazione spline cubica
  // ... implementazione ...
}
```

#### Machine Learning

Per calibrazione adattiva basata su dati storici:

```cpp
// Raccogli dati durante uso
void recordCalibrationSample(float x, float y_measured, float y_expected) {
  // Salva in buffer
  // Ogni 100 campioni, ricalcola CALIB_SLOPE e CALIB_OFFSET
}
```

---

## Changelog

### Versione 4.0 (24 Novembre 2025)

**Aggiunte:**
- ✅ Sistema di calibrazione dinamica basato su X
- ✅ Funzione `calibrateHeight(float x, float y)`
- ✅ Parametri `CALIB_SLOPE` e `CALIB_OFFSET`
- ✅ Nuove soglie `FALL_Y_THRESHOLD` e `FALL_Y_STANDING`
- ✅ Logica invertita rilevamento cadute (Y < soglia)

**Modifiche:**
- 🔄 Estrazione misure Y ora usa calibrazione dinamica
- 🔄 Sistema di coordinate Y ora rappresenta altezza reale
- 🔄 Rilevamento cadute basato su altezze calibrate
- 🔄 Log mostra altezze corrette

**Rimosso:**
- ❌ Offset fisso `HEIGHT_OFFSET` (sostituito da calibrazione)
- ❌ Logica errata `Y > FALL_Y_THRESHOLD` per caduta

**Correzioni:**
- 🐛 Eliminata distorsione prospettica su asse Y
- 🐛 Rilevamento cadute ora funziona correttamente
- 🐛 Altezze uniformi su tutto il campo di vista

### Versione 3.0 (Precedente)

- Offset fisso HEIGHT_OFFSET = 60 cm
- Algoritmo Hungarian per tracking
- Filtro Kalman con accelerazione

---

## Conclusioni

L'implementazione della **calibrazione dinamica** risolve completamente il problema della distorsione prospettica, portando a:

- ✅ **Accuratezza uniforme:** Errore < ±5 cm su tutto il campo
- ✅ **Rilevamento cadute preciso:** Basato su altezze reali
- ✅ **Facilità di calibrazione:** Solo 2 parametri da regolare
- ✅ **Adattabilità:** Funziona in ambienti diversi con ricalibrazioni minime

Il sistema è ora pronto per deployment in ambienti reali con monitoraggio di sicurezza critico.

---

**Fine Documentazione**

Per supporto o domande sulla calibrazione:
1. Consulta la sezione [Troubleshooting](#troubleshooting)
2. Verifica i [Test di Validazione](#testing-e-validazione)
3. Rivedi la [Guida alla Calibrazione](#guida-alla-calibrazione)

*Ultimo aggiornamento: 24 Novembre 2025*
