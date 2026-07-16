#define BLYNK_TEMPLATE_ID "TMPL3x4sAW-lZ"
#define BLYNK_TEMPLATE_NAME "Battery Intelligence Engine"
#define BLYNK_AUTH_TOKEN "P5O1gya_xruCwazDB2SyJbQWCyYXfRWG"

#include <LiquidCrystal.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <math.h>

const byte CELL_COUNT = 4;
const byte CELL_PINS[CELL_COUNT] = {34, 35, 32, 33};

const byte LED_HEALTHY = 25;
const byte LED_WARNING = 26;
const byte LED_FAILURE = 27;
const byte BUZZER_PIN = 14;
const byte RELAY_PIN = 13;
const byte RELAY_FEEDBACK_PIN = 23;

const char WIFI_SSID[] = "Wokwi-GUEST";
const char WIFI_PASS[] = "";

const float ADC_REFERENCE_V = 3.3;
const int ADC_MAX = 4095;
const float MIN_CELL_V = 3.0;
const float MAX_CELL_V = 4.2;
const float EMPTY_CELL_V = 3.0;
const float FULL_CELL_V = 4.2;

const float MINOR_IMBALANCE_PERCENT = 2.0;
const float CRITICAL_IMBALANCE_PERCENT = 5.0;
const float WEAK_CELL_LIMIT = 3.20;
const float UNDERVOLTAGE_LIMIT = 3.0;
const float OVERVOLTAGE_LIMIT = 4.18;
const float PACK_FAILURE_CELL_LIMIT = 2.75;
const float SENSOR_ANOMALY_LOW_V = 3.01;
const float SENSOR_ANOMALY_HIGH_V = 4.19;
const float RAPID_DELTA_LIMIT_V = 0.22;

const unsigned long SAMPLE_INTERVAL_MS = 300;
const unsigned long SERIAL_INTERVAL_MS = 1200;
const unsigned long LCD_REFRESH_INTERVAL_MS = 250;
const unsigned long LCD_PAGE_INTERVAL_MS = 2200;
const unsigned long FAILURE_BLINK_INTERVAL_MS = 250;
const unsigned long WARNING_BLINK_INTERVAL_MS = 500;
const unsigned long BUZZER_PATTERN_INTERVAL_MS = 250;
const unsigned long RELAY_MIN_SWITCH_MS = 3000;
const unsigned long RECOVERY_HOLD_MS = 5000;
const unsigned long RELAY_FEEDBACK_SETTLE_MS = 120;
const unsigned long RUNTIME_LOG_INTERVAL_MS = 2500;
const unsigned long WIFI_RECONNECT_INTERVAL_MS = 5000;
const unsigned long BLYNK_RECONNECT_INTERVAL_MS = 3000;
const unsigned long SIGNAL_QUALITY_INTERVAL_MS = 5000;
const unsigned long CLOUD_QUEUE_DRAIN_INTERVAL_MS = 350;
const byte FROZEN_ADC_SAMPLE_LIMIT = 20;
const byte MAX_FAULT_LOGS = 8;
const byte CLOUD_QUEUE_SIZE = 12;

enum HealthState {
  HEALTHY,
  MINOR_IMBALANCE,
  CRITICAL_IMBALANCE,
  PACK_FAILURE
};

enum FaultCode {
  FAULT_NONE,
  FAULT_WEAK_CELL,
  FAULT_OVERVOLTAGE,
  FAULT_SENSOR_ANOMALY,
  FAULT_RAPID_FLUCTUATION,
  FAULT_CRITICAL_IMBALANCE
};

enum SafetyState {
  SAFETY_SAFE,
  SAFETY_WARNING,
  SAFETY_CUTOFF,
  SAFETY_RECOVERY_WAIT
};

enum RuntimeMode {
  MODE_NORMAL,
  MODE_DEGRADED,
  MODE_FAILSAFE,
  MODE_SHUTDOWN
};

enum RuntimeFault {
  RUNTIME_OK,
  RUNTIME_SENSOR_DISCONNECTED,
  RUNTIME_INVALID_READING,
  RUNTIME_FROZEN_ADC,
  RUNTIME_RELAY_MISMATCH
};

struct BatteryMetrics {
  int raw[CELL_COUNT];
  float cellVoltage[CELL_COUNT];
  float previousVoltage[CELL_COUNT];
  float packVoltage;
  float averageVoltage;
  float minVoltage;
  float maxVoltage;
  float imbalancePercent;
  byte weakestCell;
  byte strongestCell;
  HealthState state;
};

struct RuntimeFaultLog {
  unsigned long timestamp;
  RuntimeFault fault;
  byte module;
  RuntimeMode mode;
};

enum CloudEventType {
  CLOUD_BOOT,
  CLOUD_STATE_CHANGE,
  CLOUD_ANOMALY,
  CLOUD_THRESHOLD,
  CLOUD_RECOVERY
};

struct CloudTelemetryEvent {
  unsigned long timestamp;
  CloudEventType type;
  SafetyState safety;
  RuntimeMode runtime;
  FaultCode fault;
  RuntimeFault runtimeFault;
  float packVoltage;
  float averageVoltage;
  float imbalancePercent;
  float minVoltage;
  float maxVoltage;
  float cellVoltage[CELL_COUNT];
  byte weakestCell;
  bool relayState;
  int rssi;
};

LiquidCrystal lcd(19, 18, 5, 17, 16, 4);
BatteryMetrics metrics;
unsigned long lastSampleAt = 0;
unsigned long lastSerialAt = 0;
unsigned long lastLcdRefreshAt = 0;
unsigned long lastLcdPageAt = 0;
unsigned long relayChangedAt = 0;
unsigned long safeSinceAt = 0;
unsigned long lastRuntimeLogAt = 0;
unsigned long lastWifiReconnectAt = 0;
unsigned long lastBlynkReconnectAt = 0;
unsigned long lastSignalQualityAt = 0;
unsigned long lastCloudDrainAt = 0;
unsigned long lastDemoPublishAt = 0;
byte lcdPage = 0;
SafetyState safetyState = SAFETY_SAFE;
FaultCode activeFault = FAULT_NONE;
RuntimeMode runtimeMode = MODE_NORMAL;
RuntimeFault lastRuntimeFault = RUNTIME_OK;
SafetyState lastCloudSafetyState = SAFETY_SAFE;
RuntimeMode lastCloudRuntimeMode = MODE_NORMAL;
FaultCode lastCloudFault = FAULT_NONE;
RuntimeFault lastCloudRuntimeFault = RUNTIME_OK;
bool cloudConnected = false;
bool relayClosed = true;
char lcdLineCache[2][17] = {"", ""};
bool cellIsolated[CELL_COUNT] = {false, false, false, false};
int lastRawForFreeze[CELL_COUNT] = {-1, -1, -1, -1};
byte frozenSampleCount[CELL_COUNT] = {0, 0, 0, 0};
unsigned long isolatedRecoverySince[CELL_COUNT] = {0, 0, 0, 0};
RuntimeFaultLog faultLogs[MAX_FAULT_LOGS];
byte faultLogHead = 0;
byte faultLogCount = 0;
CloudTelemetryEvent cloudQueue[CLOUD_QUEUE_SIZE];
byte cloudQueueHead = 0;
byte cloudQueueTail = 0;
byte cloudQueueCount = 0;

BLYNK_CONNECTED() {
  cloudConnected = true;
}

float analogToCellVoltage(int rawValue) {
  float inputVoltage = (rawValue * ADC_REFERENCE_V) / ADC_MAX;
  return MIN_CELL_V + ((inputVoltage / ADC_REFERENCE_V) * (MAX_CELL_V - MIN_CELL_V));
}

const char *healthStateName(HealthState state) {
  switch (state) {
    case HEALTHY:
      return "Healthy";
    case MINOR_IMBALANCE:
      return "Minor Imbalance";
    case CRITICAL_IMBALANCE:
      return "Critical Imbalance";
    case PACK_FAILURE:
      return "Pack Failure";
    default:
      return "Unknown";
  }
}

const char *faultName(FaultCode fault) {
  switch (fault) {
    case FAULT_NONE:
      return "No Fault";
    case FAULT_WEAK_CELL:
      return "Weak Cell";
    case FAULT_OVERVOLTAGE:
      return "Overvoltage";
    case FAULT_SENSOR_ANOMALY:
      return "Sensor Anomaly";
    case FAULT_RAPID_FLUCTUATION:
      return "Rapid Fluctuation";
    case FAULT_CRITICAL_IMBALANCE:
      return "Critical Imbalance";
    default:
      return "Unknown Fault";
  }
}

const char *faultShortName(FaultCode fault) {
  switch (fault) {
    case FAULT_NONE:
      return "No Fault";
    case FAULT_WEAK_CELL:
      return "Weak Cell";
    case FAULT_OVERVOLTAGE:
      return "Overvoltage";
    case FAULT_SENSOR_ANOMALY:
      return "Sensor Fault";
    case FAULT_RAPID_FLUCTUATION:
      return "Rapid Volt";
    case FAULT_CRITICAL_IMBALANCE:
      return "Crit Imbalance";
    default:
      return "Unknown";
  }
}

const char *safetyStateName(SafetyState state) {
  switch (state) {
    case SAFETY_SAFE:
      return "Safe";
    case SAFETY_WARNING:
      return "Warning";
    case SAFETY_CUTOFF:
      return "Cutoff";
    case SAFETY_RECOVERY_WAIT:
      return "Recovery Wait";
    default:
      return "Unknown";
  }
}

const char *runtimeModeName(RuntimeMode mode) {
  switch (mode) {
    case MODE_NORMAL:
      return "NORMAL";
    case MODE_DEGRADED:
      return "DEGRADED";
    case MODE_FAILSAFE:
      return "FAILSAFE";
    case MODE_SHUTDOWN:
      return "SHUTDOWN";
    default:
      return "UNKNOWN";
  }
}

const char *runtimeFaultName(RuntimeFault fault) {
  switch (fault) {
    case RUNTIME_OK:
      return "OK";
    case RUNTIME_SENSOR_DISCONNECTED:
      return "Sensor Disconnected";
    case RUNTIME_INVALID_READING:
      return "Invalid Reading";
    case RUNTIME_FROZEN_ADC:
      return "Frozen ADC";
    case RUNTIME_RELAY_MISMATCH:
      return "Relay Mismatch";
    default:
      return "Unknown Runtime Fault";
  }
}

void logRuntimeFault(RuntimeFault fault, byte module, RuntimeMode mode, unsigned long now) {
  if (fault == RUNTIME_OK) {
    return;
  }

  RuntimeFaultLog &entry = faultLogs[faultLogHead];
  entry.timestamp = now;
  entry.fault = fault;
  entry.module = module;
  entry.mode = mode;

  faultLogHead = (faultLogHead + 1) % MAX_FAULT_LOGS;
  if (faultLogCount < MAX_FAULT_LOGS) {
    faultLogCount++;
  }
}

const char *cloudEventName(CloudEventType type) {
  switch (type) {
    case CLOUD_BOOT:
      return "boot";
    case CLOUD_STATE_CHANGE:
      return "state_change";
    case CLOUD_ANOMALY:
      return "anomaly";
    case CLOUD_THRESHOLD:
      return "threshold";
    case CLOUD_RECOVERY:
      return "recovery";
    default:
      return "unknown";
  }
}

void enqueueCloudEvent(CloudEventType type, unsigned long now) {
  CloudTelemetryEvent event;
  event.timestamp = now;
  event.type = type;
  event.safety = safetyState;
  event.runtime = runtimeMode;
  event.fault = activeFault;
  event.runtimeFault = lastRuntimeFault;
  event.packVoltage = metrics.packVoltage;
  event.averageVoltage = metrics.averageVoltage;
  event.imbalancePercent = metrics.imbalancePercent;
  event.minVoltage = metrics.minVoltage;
  event.maxVoltage = metrics.maxVoltage;
  for (byte index = 0; index < CELL_COUNT; index++) {
    event.cellVoltage[index] = metrics.cellVoltage[index];
  }
  event.weakestCell = metrics.weakestCell + 1;
  event.relayState = relayClosed;
  event.rssi = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : -127;

  cloudQueue[cloudQueueTail] = event;
  cloudQueueTail = (cloudQueueTail + 1) % CLOUD_QUEUE_SIZE;

  if (cloudQueueCount < CLOUD_QUEUE_SIZE) {
    cloudQueueCount++;
  } else {
    cloudQueueHead = (cloudQueueHead + 1) % CLOUD_QUEUE_SIZE;
  }
}

bool dequeueCloudEvent(CloudTelemetryEvent &event) {
  if (cloudQueueCount == 0) {
    return false;
  }

  event = cloudQueue[cloudQueueHead];
  cloudQueueHead = (cloudQueueHead + 1) % CLOUD_QUEUE_SIZE;
  cloudQueueCount--;
  return true;
}

bool isThresholdViolation() {
  return metrics.minVoltage <= WEAK_CELL_LIMIT ||
         metrics.maxVoltage >= OVERVOLTAGE_LIMIT ||
         metrics.imbalancePercent >= MINOR_IMBALANCE_PERCENT;
}

bool isAnomalyActive() {
  return activeFault == FAULT_SENSOR_ANOMALY ||
         activeFault == FAULT_RAPID_FLUCTUATION ||
         lastRuntimeFault == RUNTIME_SENSOR_DISCONNECTED ||
         lastRuntimeFault == RUNTIME_INVALID_READING ||
         lastRuntimeFault == RUNTIME_FROZEN_ADC ||
         lastRuntimeFault == RUNTIME_RELAY_MISMATCH;
}

void publishCloudEvent(const CloudTelemetryEvent &event) {
  int riskScore = 30;
  Blynk.virtualWrite(V0, event.packVoltage);
  Blynk.virtualWrite(V1, event.averageVoltage);
  Blynk.virtualWrite(V2, event.imbalancePercent);
  Blynk.virtualWrite(V3, safetyStateName(event.safety));
  Blynk.virtualWrite(V4, runtimeModeName(event.runtime));
  Blynk.virtualWrite(V5, faultShortName(event.fault));
  Blynk.virtualWrite(V6, event.relayState ? 1 : 0);
  Blynk.virtualWrite(V7, event.rssi);
  Blynk.virtualWrite(V8, cloudEventName(event.type));
  Blynk.virtualWrite(V9, event.runtimeFault == RUNTIME_OK ? "OK" : runtimeFaultName(event.runtimeFault));
  Blynk.virtualWrite(V10, event.cellVoltage[0]);
  Blynk.virtualWrite(V11, event.cellVoltage[1]);
  Blynk.virtualWrite(V12, event.cellVoltage[2]);
  Blynk.virtualWrite(V13, event.cellVoltage[3]);
  Blynk.virtualWrite(V14, event.weakestCell);
  Blynk.virtualWrite(V15, riskScore);
  Blynk.virtualWrite(V16, "MODERATE");
  Blynk.virtualWrite(V17, "Monitor pack and verify voltage balance");
  Blynk.virtualWrite(V18, faultLogCount);
  Blynk.virtualWrite(V19, isolatedCellCount());
  Blynk.virtualWrite(V20, "Fault log available in runtime supervisor");
  Blynk.virtualWrite(V21, event.relayState ? "Load Connected" : "Load Isolated");

  if (event.type == CLOUD_ANOMALY || event.type == CLOUD_THRESHOLD) {
    Blynk.logEvent("battery_fault", String(cloudEventName(event.type)) +
                                      " mode=" + runtimeModeName(event.runtime) +
                                      " fault=" + faultShortName(event.fault));
  } else if (event.type == CLOUD_RECOVERY) {
    Blynk.logEvent("battery_recovered", "Battery runtime recovered");
  }
}

void drainCloudQueue(unsigned long now) {
  if (!Blynk.connected() || cloudQueueCount == 0) {
    return;
  }

  if (now - lastCloudDrainAt < CLOUD_QUEUE_DRAIN_INTERVAL_MS) {
    return;
  }

  lastCloudDrainAt = now;
  CloudTelemetryEvent event;
  if (dequeueCloudEvent(event)) {
    publishCloudEvent(event);
  }
}

void serviceCloudConnection(unsigned long now) {
  if (WiFi.status() != WL_CONNECTED) {
    cloudConnected = false;
    if (now - lastWifiReconnectAt >= WIFI_RECONNECT_INTERVAL_MS) {
      lastWifiReconnectAt = now;
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
    return;
  }

  if (!Blynk.connected()) {
    cloudConnected = false;
    if (now - lastBlynkReconnectAt >= 1000) {
      lastBlynkReconnectAt = now;
      Blynk.connect(1000);
    }
    return;
  }
  cloudConnected = true;
  Blynk.run();
  drainCloudQueue(now);
}

void detectCloudTelemetryTriggers(unsigned long now) {
  bool safetyChanged = safetyState != lastCloudSafetyState;
  bool runtimeChanged = runtimeMode != lastCloudRuntimeMode;
  bool faultChanged = activeFault != lastCloudFault || lastRuntimeFault != lastCloudRuntimeFault;

  if (safetyChanged || runtimeChanged || faultChanged) {
    CloudEventType type = CLOUD_STATE_CHANGE;
    if (isAnomalyActive()) {
      type = CLOUD_ANOMALY;
    } else if (isThresholdViolation()) {
      type = CLOUD_THRESHOLD;
    } else if (activeFault == FAULT_NONE && lastRuntimeFault == RUNTIME_OK) {
      type = CLOUD_RECOVERY;
    }

    enqueueCloudEvent(type, now);

    lastCloudSafetyState = safetyState;
    lastCloudRuntimeMode = runtimeMode;
    lastCloudFault = activeFault;
    lastCloudRuntimeFault = lastRuntimeFault;
  }
}

HealthState classifyHealth(const BatteryMetrics &data) {
  if (data.minVoltage <= PACK_FAILURE_CELL_LIMIT ||
      data.minVoltage < UNDERVOLTAGE_LIMIT ||
      data.maxVoltage > OVERVOLTAGE_LIMIT) {
    return PACK_FAILURE;
  }

  if (data.imbalancePercent >= CRITICAL_IMBALANCE_PERCENT) {
    return CRITICAL_IMBALANCE;
  }

  if (data.imbalancePercent >= MINOR_IMBALANCE_PERCENT) {
    return MINOR_IMBALANCE;
  }

  return HEALTHY;
}

void sampleBatteryPack(BatteryMetrics &data) {
  data.packVoltage = 0;
  data.minVoltage = 99.0;
  data.maxVoltage = 0.0;
  data.weakestCell = 0;
  data.strongestCell = 0;

  for (byte index = 0; index < CELL_COUNT; index++) {
    data.previousVoltage[index] = data.cellVoltage[index];
    data.raw[index] = analogRead(CELL_PINS[index]);
    float voltage = analogToCellVoltage(data.raw[index]);
    data.cellVoltage[index] = voltage;

    if (cellIsolated[index]) {
      continue;
    }

    data.packVoltage += voltage;

    if (voltage < data.minVoltage) {
      data.minVoltage = voltage;
      data.weakestCell = index;
    }

    if (voltage > data.maxVoltage) {
      data.maxVoltage = voltage;
      data.strongestCell = index;
    }
  }

  byte activeCellCount = 0;
  for (byte index = 0; index < CELL_COUNT; index++) {
    if (!cellIsolated[index]) {
      activeCellCount++;
    }
  }

  if (activeCellCount == 0) {
    data.averageVoltage = 0;
    data.imbalancePercent = 100.0;
    data.state = PACK_FAILURE;
    return;
  }

  data.averageVoltage = data.packVoltage / activeCellCount;
  data.imbalancePercent = ((data.maxVoltage - data.minVoltage) / data.averageVoltage) * 100.0;
  data.state = classifyHealth(data);
}

FaultCode detectFault(const BatteryMetrics &data) {
  for (byte index = 0; index < CELL_COUNT; index++) {
    if (cellIsolated[index]) {
      continue;
    }

    bool rawAtRail = data.raw[index] <= 5 || data.raw[index] >= ADC_MAX - 5;
    bool voltageAtRail = data.cellVoltage[index] <= SENSOR_ANOMALY_LOW_V ||
                         data.cellVoltage[index] >= SENSOR_ANOMALY_HIGH_V;

    if (rawAtRail && voltageAtRail) {
      return FAULT_SENSOR_ANOMALY;
    }

    if (data.previousVoltage[index] > 0.1 &&
        fabs(data.cellVoltage[index] - data.previousVoltage[index]) >= RAPID_DELTA_LIMIT_V) {
      return FAULT_RAPID_FLUCTUATION;
    }
  }

  if (data.maxVoltage >= OVERVOLTAGE_LIMIT || data.minVoltage <= UNDERVOLTAGE_LIMIT) {
    return FAULT_OVERVOLTAGE;
  }

  if (data.imbalancePercent >= CRITICAL_IMBALANCE_PERCENT) {
    return FAULT_CRITICAL_IMBALANCE;
  }

  if (data.minVoltage <= WEAK_CELL_LIMIT || data.imbalancePercent >= MINOR_IMBALANCE_PERCENT) {
    return FAULT_WEAK_CELL;
  }

  return FAULT_NONE;
}

bool faultRequiresCutoff(FaultCode fault) {
  return fault == FAULT_OVERVOLTAGE ||
         fault == FAULT_SENSOR_ANOMALY ||
         fault == FAULT_RAPID_FLUCTUATION ||
         fault == FAULT_CRITICAL_IMBALANCE;
}

void setRelayClosed(bool shouldClose, unsigned long now) {
  if (runtimeMode == MODE_SHUTDOWN && shouldClose) {
    return;
  }

  if (relayClosed == shouldClose) {
    return;
  }

  if (now - relayChangedAt < RELAY_MIN_SWITCH_MS) {
    return;
  }

  relayClosed = shouldClose;
  relayChangedAt = now;
  digitalWrite(RELAY_PIN, relayClosed ? HIGH : LOW);
}

void forceRelayOpen(unsigned long now) {
  relayClosed = false;
  relayChangedAt = now;
  digitalWrite(RELAY_PIN, LOW);
}

void runSafetyKernel(unsigned long now) {
  if (runtimeMode == MODE_SHUTDOWN) {
    safetyState = SAFETY_CUTOFF;
    activeFault = FAULT_SENSOR_ANOMALY;
    forceRelayOpen(now);
    return;
  }

  FaultCode detectedFault = detectFault(metrics);

  if (detectedFault == FAULT_NONE) {
    if (safeSinceAt == 0) {
      safeSinceAt = now;
    }
  } else {
    safeSinceAt = 0;
  }

  if (faultRequiresCutoff(detectedFault)) {
    activeFault = detectedFault;
    safetyState = SAFETY_CUTOFF;
    setRelayClosed(false, now);
    return;
  }

  if (safetyState == SAFETY_CUTOFF || safetyState == SAFETY_RECOVERY_WAIT) {
    safetyState = SAFETY_RECOVERY_WAIT;
    activeFault = detectedFault;
    setRelayClosed(false, now);

    if (detectedFault == FAULT_NONE && now - safeSinceAt >= RECOVERY_HOLD_MS) {
      safetyState = SAFETY_SAFE;
      activeFault = FAULT_NONE;
      setRelayClosed(true, now);
    }
    return;
  }

  if (detectedFault == FAULT_WEAK_CELL) {
    activeFault = detectedFault;
    safetyState = SAFETY_WARNING;
    setRelayClosed(true, now);
    return;
  }

  activeFault = FAULT_NONE;
  safetyState = SAFETY_SAFE;
  setRelayClosed(true, now);
}

byte isolatedCellCount() {
  byte count = 0;
  for (byte index = 0; index < CELL_COUNT; index++) {
    if (cellIsolated[index]) {
      count++;
    }
  }
  return count;
}

void isolateCell(byte index, RuntimeFault fault, unsigned long now) {
  if (index >= CELL_COUNT || cellIsolated[index]) {
    return;
  }

  cellIsolated[index] = true;
  logRuntimeFault(fault, index + 1, runtimeMode, now);
}

RuntimeFault inspectSensorRuntime(unsigned long now) {
  for (byte index = 0; index < CELL_COUNT; index++) {
    bool rawLowRail = metrics.raw[index] <= 5;
    bool rawHighRail = metrics.raw[index] >= ADC_MAX - 5;
    bool readingValid = !rawLowRail && !rawHighRail &&
                        metrics.cellVoltage[index] >= MIN_CELL_V &&
                        metrics.cellVoltage[index] <= MAX_CELL_V;

    if (cellIsolated[index]) {
      if (readingValid) {
        if (isolatedRecoverySince[index] == 0) {
          isolatedRecoverySince[index] = now;
        }

        if (now - isolatedRecoverySince[index] >= RECOVERY_HOLD_MS) {
          cellIsolated[index] = false;
          frozenSampleCount[index] = 0;
          lastRawForFreeze[index] = metrics.raw[index];
          isolatedRecoverySince[index] = 0;
        }
      } else {
        isolatedRecoverySince[index] = 0;
        return RUNTIME_SENSOR_DISCONNECTED;
      }

      continue;
    }

    if (rawLowRail || rawHighRail) {
      isolatedRecoverySince[index] = 0;
      isolateCell(index, RUNTIME_SENSOR_DISCONNECTED, now);
      return RUNTIME_SENSOR_DISCONNECTED;
    }

    if (metrics.cellVoltage[index] < MIN_CELL_V || metrics.cellVoltage[index] > MAX_CELL_V) {
      isolatedRecoverySince[index] = 0;
      isolateCell(index, RUNTIME_INVALID_READING, now);
      return RUNTIME_INVALID_READING;
    }

    if (lastRawForFreeze[index] == metrics.raw[index]) {
      if (frozenSampleCount[index] < 255) {
        frozenSampleCount[index]++;
      }
    } else {
      frozenSampleCount[index] = 0;
      lastRawForFreeze[index] = metrics.raw[index];
    }

    bool anotherChannelMoving = false;
    for (byte peer = 0; peer < CELL_COUNT; peer++) {
      if (peer != index && fabs(metrics.cellVoltage[peer] - metrics.previousVoltage[peer]) > 0.02) {
        anotherChannelMoving = true;
      }
    }

    if (frozenSampleCount[index] >= FROZEN_ADC_SAMPLE_LIMIT && anotherChannelMoving) {
      isolatedRecoverySince[index] = 0;
      isolateCell(index, RUNTIME_FROZEN_ADC, now);
      return RUNTIME_FROZEN_ADC;
    }
  }

  return RUNTIME_OK;
}

RuntimeFault inspectRelayRuntime(unsigned long now) {
  if (now - relayChangedAt < RELAY_FEEDBACK_SETTLE_MS) {
    return RUNTIME_OK;
  }

  bool relayFeedbackClosed = digitalRead(RELAY_FEEDBACK_PIN) == HIGH;
  if (relayFeedbackClosed != relayClosed) {
    logRuntimeFault(RUNTIME_RELAY_MISMATCH, 0, MODE_SHUTDOWN, now);
    return RUNTIME_RELAY_MISMATCH;
  }

  return RUNTIME_OK;
}

void runRuntimeSupervisor(unsigned long now) {
  RuntimeFault sensorFault = inspectSensorRuntime(now);
  RuntimeFault relayFault = inspectRelayRuntime(now);
  byte isolated = isolatedCellCount();

  if (relayFault == RUNTIME_RELAY_MISMATCH) {
    lastRuntimeFault = relayFault;
    runtimeMode = MODE_SHUTDOWN;
    safetyState = SAFETY_CUTOFF;
    activeFault = FAULT_SENSOR_ANOMALY;
    forceRelayOpen(now);
    return;
  }

  if (isolated >= 2) {
    lastRuntimeFault = sensorFault == RUNTIME_OK ? RUNTIME_INVALID_READING : sensorFault;
    runtimeMode = MODE_SHUTDOWN;
    safetyState = SAFETY_CUTOFF;
    activeFault = FAULT_SENSOR_ANOMALY;
    forceRelayOpen(now);
    return;
  }

  if (safetyState == SAFETY_CUTOFF || safetyState == SAFETY_RECOVERY_WAIT) {
    runtimeMode = MODE_FAILSAFE;
    lastRuntimeFault = sensorFault;
    return;
  }

  if (isolated == 1 || sensorFault != RUNTIME_OK) {
    runtimeMode = MODE_DEGRADED;
    lastRuntimeFault = sensorFault;
    return;
  }

  runtimeMode = MODE_NORMAL;
  lastRuntimeFault = RUNTIME_OK;
}

void updateIndicators(HealthState state) {
  unsigned long now = millis();
  bool warningBlink = (now / WARNING_BLINK_INTERVAL_MS) % 2 == 0;
  bool failureBlink = (now / FAILURE_BLINK_INTERVAL_MS) % 2 == 0;

  digitalWrite(LED_HEALTHY, safetyState == SAFETY_SAFE);
  digitalWrite(LED_WARNING, safetyState == SAFETY_WARNING ? warningBlink : LOW);
  digitalWrite(LED_FAILURE, (safetyState == SAFETY_CUTOFF || safetyState == SAFETY_RECOVERY_WAIT) ? failureBlink : LOW);

  if (safetyState == SAFETY_CUTOFF) {
    bool alarmPulse = (now / BUZZER_PATTERN_INTERVAL_MS) % 2 == 0;
    if (alarmPulse) {
      tone(BUZZER_PIN, 1800);
    } else {
      noTone(BUZZER_PIN);
    }
  } else if (safetyState == SAFETY_RECOVERY_WAIT) {
    bool warningPulse = (now / (BUZZER_PATTERN_INTERVAL_MS * 4)) % 2 == 0;
    if (warningPulse) {
      tone(BUZZER_PIN, 900);
    } else {
      noTone(BUZZER_PIN);
    }
  } else {
    noTone(BUZZER_PIN);
  }
}

void printSerialDashboard(const BatteryMetrics &data) {
  Serial.println(F("===== Event-Driven Safety Protection Kernel ====="));
  for (byte index = 0; index < CELL_COUNT; index++) {
    Serial.print(F("Cell "));
    Serial.print(index + 1);
    Serial.print(F(": "));
    Serial.print(data.cellVoltage[index], 3);
    Serial.print(F(" V raw="));
    Serial.println(data.raw[index]);
  }

  Serial.print(F("Pack Voltage: "));
  Serial.print(data.packVoltage, 3);
  Serial.println(F(" V"));
  Serial.print(F("Average Cell Voltage: "));
  Serial.print(data.averageVoltage, 3);
  Serial.println(F(" V"));
  Serial.print(F("Imbalance: "));
  Serial.print(data.imbalancePercent, 2);
  Serial.println(F(" %"));
  Serial.print(F("Weakest Cell: C"));
  Serial.println(data.weakestCell + 1);
  Serial.print(F("Strongest Cell: C"));
  Serial.println(data.strongestCell + 1);
  Serial.print(F("Health State: "));
  Serial.println(healthStateName(data.state));
  Serial.print(F("Safety State: "));
  Serial.println(safetyStateName(safetyState));
  Serial.print(F("Fault: "));
  Serial.println(faultName(activeFault));
  Serial.print(F("Runtime Mode: "));
  Serial.println(runtimeModeName(runtimeMode));
  Serial.print(F("Runtime Fault: "));
  Serial.println(runtimeFaultName(lastRuntimeFault));
  Serial.print(F("Isolated Cells: "));
  Serial.println(isolatedCellCount());
  Serial.print(F("Protection Relay: "));
  Serial.println(relayClosed ? F("ON - Load Connected") : F("OFF - Load Disconnected"));
  Serial.print(F("Cloud: "));
  Serial.print(Blynk.connected() ? F("BLYNK_CONNECTED") : (WiFi.status() == WL_CONNECTED ? F("WIFI_ONLY") : F("OFFLINE")));
  Serial.print(F(" queue="));
  Serial.print(cloudQueueCount);
  Serial.print(F(" rssi="));
  Serial.println(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : -127);

  if (faultLogCount > 0 && millis() - lastRuntimeLogAt >= RUNTIME_LOG_INTERVAL_MS) {
    lastRuntimeLogAt = millis();
    Serial.println(F("Runtime Fault Log:"));
    for (byte entry = 0; entry < faultLogCount; entry++) {
      byte index = (faultLogHead + MAX_FAULT_LOGS - faultLogCount + entry) % MAX_FAULT_LOGS;
      Serial.print(F("  t+"));
      Serial.print(faultLogs[index].timestamp);
      Serial.print(F("ms module="));
      Serial.print(faultLogs[index].module);
      Serial.print(F(" mode="));
      Serial.print(runtimeModeName(faultLogs[index].mode));
      Serial.print(F(" fault="));
      Serial.println(runtimeFaultName(faultLogs[index].fault));
    }
  }

  Serial.println();
}

void fitLcdLine(char *target, const char *source) {
  byte index = 0;
  for (; index < 16 && source[index] != '\0'; index++) {
    target[index] = source[index];
  }
  for (; index < 16; index++) {
    target[index] = ' ';
  }
  target[16] = '\0';
}

void writeLcdLine(byte row, const char *text) {
  char padded[17];
  fitLcdLine(padded, text);

  if (strncmp(lcdLineCache[row], padded, 16) == 0) {
    return;
  }

  lcd.setCursor(0, row);
  lcd.print(padded);
  strncpy(lcdLineCache[row], padded, 17);
}

void resetHmiCache() {
  lcdLineCache[0][0] = '\0';
  lcdLineCache[1][0] = '\0';
}

void renderHmiPage(const BatteryMetrics &data) {
  char line1[17];
  char line2[17];

  if (safetyState == SAFETY_CUTOFF) {
    snprintf(line1, sizeof(line1), "%-8s CUTOFF", runtimeModeName(runtimeMode));
    snprintf(line2, sizeof(line2), "%-16s", runtimeMode == MODE_SHUTDOWN ? runtimeFaultName(lastRuntimeFault) : faultShortName(activeFault));
    writeLcdLine(0, line1);
    writeLcdLine(1, line2);
    return;
  }

  if (safetyState == SAFETY_RECOVERY_WAIT) {
    unsigned long remainingMs = 0;
    if (safeSinceAt > 0) {
      unsigned long elapsed = millis() - safeSinceAt;
      remainingMs = elapsed >= RECOVERY_HOLD_MS ? 0 : RECOVERY_HOLD_MS - elapsed;
    } else {
      remainingMs = RECOVERY_HOLD_MS;
    }

    snprintf(line1, sizeof(line1), "%-8s RECOV", runtimeModeName(runtimeMode));
    snprintf(line2, sizeof(line2), "Relay ON in %lus", (remainingMs + 999) / 1000);
    writeLcdLine(0, line1);
    writeLcdLine(1, line2);
    return;
  }

  if (safetyState == SAFETY_WARNING) {
    snprintf(line1, sizeof(line1), "WARN C%d %4.2fV", data.weakestCell + 1, data.minVoltage);
    snprintf(line2, sizeof(line2), "%s", faultShortName(activeFault));
    writeLcdLine(0, line1);
    writeLcdLine(1, line2);
    return;
  }

  switch (lcdPage) {
    case 0:
      snprintf(line1, sizeof(line1), "PACK %5.2fV", data.packVoltage);
      snprintf(line2, sizeof(line2), "AVG %4.2f IMB%2.0f", data.averageVoltage, data.imbalancePercent);
      break;
    case 1:
      snprintf(line1, sizeof(line1), "C1%4.2f C2%4.2f", data.cellVoltage[0], data.cellVoltage[1]);
      snprintf(line2, sizeof(line2), "C3%4.2f C4%4.2f", data.cellVoltage[2], data.cellVoltage[3]);
      break;
    case 2:
      snprintf(line1, sizeof(line1), "WEAK C%d %4.2fV", data.weakestCell + 1, data.minVoltage);
      snprintf(line2, sizeof(line2), "STRG C%d %4.2fV", data.strongestCell + 1, data.maxVoltage);
      break;
    case 3:
      snprintf(line1, sizeof(line1), "MODE %-10s", runtimeModeName(runtimeMode));
      snprintf(line2, sizeof(line2), "RELAY %s ISO %d", relayClosed ? "ON" : "OFF", isolatedCellCount());
      break;
    case 4:
      snprintf(line1, sizeof(line1), "FAULT LOG %d/%d", faultLogCount, MAX_FAULT_LOGS);
      snprintf(line2, sizeof(line2), "%s", runtimeFaultName(lastRuntimeFault));
      break;
    default:
      snprintf(line1, sizeof(line1), "CLOUD %s", Blynk.connected() ? "ONLINE" : "OFFLINE");
      snprintf(line2, sizeof(line2), "Q%d RSSI %d", cloudQueueCount, WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : -127);
      break;
  }

  writeLcdLine(0, line1);
  writeLcdLine(1, line2);
}

void setup() {
  delay(2000);
  Serial.begin(115200);
  Serial.println("BOOT TEST");
  lcd.begin(16, 2);
  analogReadResolution(12);

  pinMode(LED_HEALTHY, OUTPUT);
  pinMode(LED_WARNING, OUTPUT);
  pinMode(LED_FAILURE, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(RELAY_FEEDBACK_PIN, INPUT_PULLDOWN);
  digitalWrite(RELAY_PIN, HIGH);
  relayChangedAt = millis() - RELAY_MIN_SWITCH_MS;
  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASS, "blr1.blynk.cloud", 80); 
  enqueueCloudEvent(CLOUD_BOOT, millis());

  lcd.print(F("Safety Kernel"));
  lcd.setCursor(0, 1);
  lcd.print(F("Event Driven"));
  resetHmiCache();
}

void loop() {
  unsigned long now = millis();

  if (now - lastSampleAt >= SAMPLE_INTERVAL_MS) {
    lastSampleAt = now;
    sampleBatteryPack(metrics);
    runSafetyKernel(now);
    runRuntimeSupervisor(now);
    detectCloudTelemetryTriggers(now);
  }

  Blynk.run();

if (Blynk.connected() && now - lastDemoPublishAt >= 1000) {
  lastDemoPublishAt = now;

  Blynk.virtualWrite(V0, metrics.packVoltage);
  Blynk.virtualWrite(V1, metrics.averageVoltage);
  Blynk.virtualWrite(V2, metrics.imbalancePercent);
  Blynk.virtualWrite(V3, safetyStateName(safetyState));
  Blynk.virtualWrite(V4, runtimeModeName(runtimeMode));
  Blynk.virtualWrite(V5, faultShortName(activeFault));
  Blynk.virtualWrite(V6, relayClosed ? 1 : 0);
  Blynk.virtualWrite(V10, metrics.cellVoltage[0]);
  Blynk.virtualWrite(V11, metrics.cellVoltage[1]);
  Blynk.virtualWrite(V12, metrics.cellVoltage[2]);
  Blynk.virtualWrite(V13, metrics.cellVoltage[3]);

  int demoRisk = safetyState == SAFETY_SAFE ? 15 : safetyState == SAFETY_WARNING ? 45 : 90;
  Blynk.virtualWrite(V15, demoRisk);
  Blynk.virtualWrite(V16, demoRisk >= 80 ? "CRITICAL" : demoRisk >= 40 ? "MODERATE" : "LOW");
  Blynk.virtualWrite(V17, safetyState == SAFETY_SAFE ? "Pack normal, continue operation" : "Inspect voltage and relay state");
  Blynk.virtualWrite(V21, relayClosed ? "Load Connected" : "Load Isolated");
}

updateIndicators(metrics.state);

if (now - lastSerialAt >= SERIAL_INTERVAL_MS) {
  lastSerialAt = now;
  printSerialDashboard(metrics);
}

if (safetyState == SAFETY_SAFE && now - lastLcdPageAt >= LCD_PAGE_INTERVAL_MS) {
  lastLcdPageAt = now;
  lcdPage = (lcdPage + 1) % 6;
}

if (now - lastLcdRefreshAt >= LCD_REFRESH_INTERVAL_MS) {
  lastLcdRefreshAt = now;
  renderHmiPage(metrics);
}
}
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  