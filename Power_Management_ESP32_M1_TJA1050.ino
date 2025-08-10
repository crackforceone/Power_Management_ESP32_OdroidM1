#include "driver/adc.h"

#include <esp_sleep.h>
#include "driver/twai.h"

#define PWR 27
#define ALIVE 26
#define RadioON 4
#define CANOFF 12
#define USBOFF 14
#define DisplayOFF 13

// CAN Bus pins (adjust based on your hardware)
#define CAN_TX_PIN GPIO_NUM_18
#define CAN_RX_PIN GPIO_NUM_19

// FIXED: Use 0x130 (hex) instead of 130 (decimal)
#define TARGET_CAN_ID 0x130

// CAN Bus variables
static uint8_t previousCANState = 0xFF;
static uint8_t currentCANState = 0xFF;
static bool canInitialized = false;

// Grace period variables
static unsigned long ignitionOffTime = 0;
static bool ignitionOffTimerActive = false;
static const unsigned long GRACE_PERIOD_MS = 120000;  // 15 seconds
static const unsigned long RETRY_PERIOD_MS = 5000;   // 5 seconds for retry attempts
static const unsigned long PWR_RETRY_INTERVAL_MS = 10000;  // 10 seconds between PWR presses
static bool isRetryAttempt = false;
static unsigned long lastDebugTime = 0;  // For grace period debug output
static unsigned long lastPwrTriggerTime = 0;  // Track when PWR was last triggered
static int pwrTriggerCount = 0;  // Count how many times PWR has been triggered this cycle

// NEW: Initial wake-up timer variables
static bool initialWakeTimerActive = false;
static unsigned long initialWakeTime = 0;
static const unsigned long INITIAL_WAKE_PERIOD_MS = 120000;  // 120 seconds = 2 minutes
static unsigned long lastInitialWakeDebugTime = 0;

// Power-on sequence variables
static bool powerOnPwrTriggered = false;  // Track if PWR was triggered on power-on
static bool isInitialPowerOn = true;      // Track if this is initial power-on
static unsigned long powerOnTime = 0;     // Track when system powered on
static bool waitingForIgnition = false;   // Track if we're waiting for ignition after power-on

// NEW: RadioON state tracking to prevent shutdown during engine start
static bool lastRadioONState = false;    // Track previous RadioON state
static unsigned long radioONLowStartTime = 0;  // When RadioON went LOW
static bool radioONLowTimerActive = false;
static const unsigned long RADIO_LOW_GRACE_MS = 5000;  // 5 seconds grace for engine start

// Simplified CAN state management - no queue needed
TaskHandle_t canTaskHandle;
volatile bool canStateChanged = false;
SemaphoreHandle_t canStateMutex;

// System state enumeration for cleaner state management
enum SystemState {
  STATE_SLEEP = 0,   // Radio OFF, Alive OFF
  STATE_ALIVE_ONLY,  // Radio OFF, Alive ON
  STATE_RADIO_ONLY,  // Radio ON, Alive OFF
  STATE_FULL_ACTIVE  // Radio ON, Alive ON
};

// Configure wake-up sources for both pins
void configureWakeupSources() {
  // Use ext1 wake-up to support multiple pins
  // Wake up when either RadioON (GPIO_4) or ALIVE (GPIO_26) goes HIGH
  uint64_t ext_wakeup_pin_mask = (1ULL << GPIO_NUM_4) | (1ULL << GPIO_NUM_26);
  esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
}

// Simplified CAN monitoring task - processes state changes directly
void canMonitorTask(void* parameter) {
  twai_message_t rx_msg;
  uint8_t lastProcessedState = 0xFF;

  while (true) {
    // Continuously wait for CAN messages (blocking call)
    if (twai_receive(&rx_msg, portMAX_DELAY) == ESP_OK) {

      // Check if it's our target ID
      if (rx_msg.identifier == TARGET_CAN_ID && rx_msg.data_length_code >= 1) {
        uint8_t newState = rx_msg.data[0];

        // Only process if state actually changed
        if (newState != lastProcessedState) {
          // Update global state safely
          if (xSemaphoreTake(canStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            previousCANState = currentCANState;
            currentCANState = newState;
            canStateChanged = true;
            xSemaphoreGive(canStateMutex);

            lastProcessedState = newState;
            // Only print on state changes
            Serial.printf("CAN State changed: 0x%02X\n", currentCANState);
          }
        }
      }

      // Handle CAN bus errors and recovery
      twai_status_info_t status_info;
      if (twai_get_status_info(&status_info) == ESP_OK) {
        if (status_info.state == TWAI_STATE_BUS_OFF) {
          Serial.println("CAN Bus OFF - attempting recovery...");
          twai_initiate_recovery();
          vTaskDelay(pdMS_TO_TICKS(100));
        } else if (status_info.state == TWAI_STATE_STOPPED) {
          Serial.println("CAN Bus STOPPED - restarting...");
          twai_start();
          vTaskDelay(pdMS_TO_TICKS(100));
        }
      }
    }

    // Small yield to prevent watchdog issues
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// Simplified CAN state processing
void processCANStateChange() {
  if (!canStateChanged) return;

  // Get current state safely
  if (xSemaphoreTake(canStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    canStateChanged = false;
    xSemaphoreGive(canStateMutex);

    // Process the state change
    processCANState();
  }
}

void initCAN() {
  // FIXED: Use same initialization approach as working code
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_100KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install and start CAN driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("CAN Driver installed");
  } else {
    Serial.println("Failed to install CAN Driver");
    return;
  }

  if (twai_start() == ESP_OK) {
    Serial.println("CAN Driver started");
    canInitialized = true;
  } else {
    Serial.println("Failed to start CAN Driver");
  }
}

void processCANState() {
  switch (currentCANState) {
    case 0x00:
    case 0x40:
      // Ignition OFF states
      handleIgnitionOff();
      break;

    case 0x41:
    case 0x42:
    case 0x45:
    case 0x55:
      // Ignition ON states
      handleIgnitionOn();
      break;

    default:
      // Unknown state - minimal logging
      // Serial.printf("Unknown CAN state: 0x%02X\n", currentCANState);
      break;
  }
}

// NEW: Function to start initial wake timer
void startInitialWakeTimer() {
  initialWakeTimerActive = true;
  initialWakeTime = millis();
  lastInitialWakeDebugTime = 0;
  Serial.println("=== INITIAL WAKE TIMER STARTED (120s) ===");
  Serial.println("System will stay awake for 120 seconds regardless of RadioON state");
}

// NEW: Function to check initial wake timer
void checkInitialWakeTimer() {
  if (!initialWakeTimerActive) return;
  
  unsigned long elapsed = millis() - initialWakeTime;
  
  // Debug output every 10 seconds during initial wake period
  if (millis() - lastInitialWakeDebugTime > 10000) {
    Serial.printf("Initial wake timer: %lu/%lu ms (RadioON: %d, ALIVE: %d)\n", 
                  elapsed, INITIAL_WAKE_PERIOD_MS,
                  digitalRead(RadioON), digitalRead(ALIVE));
    lastInitialWakeDebugTime = millis();
  }
  
  // Check if initial wake period has expired
  if (elapsed >= INITIAL_WAKE_PERIOD_MS) {
    Serial.println("=== INITIAL WAKE TIMER EXPIRED ===");
    Serial.println("Now entering regular operation mode");
    initialWakeTimerActive = false;
    isInitialPowerOn = false;  // No longer in initial power-on phase
    
    // After initial wake timer expires, check current state and act accordingly
    if (digitalRead(RadioON) == LOW && !isIgnitionActive()) {
      Serial.println("RadioON is LOW and ignition not active - starting shutdown sequence");
      handleIgnitionOff();
    }
  }
}

// NEW: Function to check if we're in initial wake period
bool isInInitialWakePeriod() {
  return initialWakeTimerActive;
}

// NEW: Function to check if RadioON LOW should be ignored due to ignition state
bool shouldIgnoreRadioONLow() {
  // Always ignore RadioON LOW if ignition is active
  if (isIgnitionActive()) {
    Serial.printf("RadioON LOW ignored - ignition active (CAN state: 0x%02X)\n", currentCANState);
    return true;
  }
  
  // If we're in initial wake period, also ignore
  if (isInInitialWakePeriod()) {
    Serial.println("RadioON LOW ignored - in initial wake period");
    return true;
  }
  
  return false;
}

// NEW: Function to monitor RadioON transitions
void monitorRadioONTransitions() {
  bool currentRadioON = digitalRead(RadioON) == HIGH;
  
  // Detect RadioON state changes
  if (currentRadioON != lastRadioONState) {
    if (!currentRadioON) {
      // RadioON went LOW
      Serial.printf("RadioON transition: HIGH -> LOW (ignition active: %s)\n", 
                    isIgnitionActive() ? "YES" : "NO");
      
      if (shouldIgnoreRadioONLow()) {
        // Start a grace timer for temporary RadioON LOW during ignition
        radioONLowStartTime = millis();
        radioONLowTimerActive = true;
        Serial.println("Starting RadioON LOW grace period (ignition active)");
      }
    } else {
      // RadioON went HIGH
      Serial.println("RadioON transition: LOW -> HIGH");
      if (radioONLowTimerActive) {
        Serial.println("RadioON back HIGH - cancelling LOW grace period");
        radioONLowTimerActive = false;
      }
    }
    lastRadioONState = currentRadioON;
  }
  
  // Check RadioON LOW grace timer
  if (radioONLowTimerActive) {
    unsigned long elapsed = millis() - radioONLowStartTime;
    
    // If RadioON comes back HIGH, cancel the timer
    if (currentRadioON) {
      Serial.println("RadioON back HIGH during grace period - timer cancelled");
      radioONLowTimerActive = false;
    }
    // If ignition goes OFF during RadioON LOW, start shutdown immediately
    else if (!isIgnitionActive()) {
      Serial.println("Ignition went OFF during RadioON LOW - starting shutdown");
      radioONLowTimerActive = false;
      handleIgnitionOff();
    }
    // If grace period expires and ignition is still ON, keep running normally
    else if (elapsed >= RADIO_LOW_GRACE_MS) {
      if (isIgnitionActive()) {
        Serial.println("RadioON LOW grace period expired - ignition still active, continuing normal operation");
        radioONLowTimerActive = false;
      } else {
        Serial.println("RadioON LOW grace period expired - ignition OFF, starting shutdown");
        radioONLowTimerActive = false;
        handleIgnitionOff();
      }
    }
  }
}

// Power-on sequence handling
void handlePowerOnSequence() {
  // Only trigger power-on sequence once
  if (!powerOnPwrTriggered && digitalRead(RadioON) == HIGH) {
    Serial.println("=== POWER-ON SEQUENCE ===");
    Serial.printf("RadioON detected HIGH - triggering PWR to start system (RadioON: %d, ALIVE: %d)\n",
                  digitalRead(RadioON), digitalRead(ALIVE));
    
    digitalWrite(PWR, LOW);
    delay(250);
    digitalWrite(PWR, HIGH);
    
    powerOnPwrTriggered = true;
    waitingForIgnition = true;
    powerOnTime = millis();
    
    // NEW: Start the initial wake timer
    startInitialWakeTimer();
    
    Serial.println("PWR triggered - now waiting for ignition to turn ON");
    Serial.println("=== POWER-ON SEQUENCE COMPLETE ===");
  }
}

// MODIFIED: Handle RadioON going LOW while waiting for ignition - but respect initial wake timer AND ignition state
void handleRadioOffWhileWaitingForIgnition() {
  if (waitingForIgnition && digitalRead(RadioON) == LOW && digitalRead(ALIVE) == HIGH) {
    
    // NEW: Check if we should ignore this RadioON LOW
    if (shouldIgnoreRadioONLow()) {
      Serial.println("=== RADIO OFF IGNORED DURING IGNITION ===");
      Serial.printf("RadioON went LOW but ignition active or in wake period (RadioON: %d, ALIVE: %d, CAN: 0x%02X)\n",
                    digitalRead(RadioON), digitalRead(ALIVE), currentCANState);
      Serial.println("Ignoring RadioON LOW - staying awake");
      return;  // Don't start shutdown
    }
    
    Serial.println("=== RADIO OFF WHILE WAITING FOR IGNITION ===");
    Serial.printf("RadioON went LOW while waiting for ignition (RadioON: %d, ALIVE: %d)\n",
                  digitalRead(RadioON), digitalRead(ALIVE));
    
    // Reset power-on flags so PWR can be triggered for shutdown
    waitingForIgnition = false;
    
    // Start shutdown sequence - this is a new cycle so reset PWR trigger counters
    pwrTriggerCount = 0;
    lastPwrTriggerTime = 0;
    
    Serial.println("Starting shutdown due to RadioON going LOW");
    ignitionOffTime = millis();
    ignitionOffTimerActive = true;
    isRetryAttempt = false;
    lastDebugTime = 0;
    
    setDisplayLow("RadioON LOW - shutting down");
    Serial.println("=== RADIO OFF SHUTDOWN INITIATED ===");
  }
}

// MODIFIED: Handle ignition off - but respect initial wake timer
void handleIgnitionOff() {
  // NEW: Don't start shutdown if we're in initial wake period
  if (isInInitialWakePeriod()) {
    Serial.println("Ignition OFF detected but in initial wake period - staying awake");
    setDisplayLow("Ignition OFF (in wake period)");
    return;
  }

  // Only turn off Display immediately (USB stays based on RadioON)
  setDisplayLow("Ignition OFF");

  // Start grace period timer if not already active
  if (!ignitionOffTimerActive) {
    ignitionOffTime = millis();
    ignitionOffTimerActive = true;
    isRetryAttempt = false;  // This is the initial attempt
    lastDebugTime = 0;       // Reset debug timer for clean output
    pwrTriggerCount = 0;     // Reset PWR trigger count for new ignition OFF cycle
    lastPwrTriggerTime = 0;  // Reset PWR trigger time
    Serial.printf("Ignition OFF - Starting 15s grace period (RadioON: %d, ALIVE: %d)\n",
                  digitalRead(RadioON), digitalRead(ALIVE));
  }
}

void handleIgnitionOn() {
  // Cancel grace period and restore normal operation
  if (ignitionOffTimerActive) {
    ignitionOffTimerActive = false;
    isRetryAttempt = false;  // Reset retry flag
    lastDebugTime = 0;       // Reset debug timer
    Serial.println("Ignition back ON - Grace period cancelled");
  }
  
  // Cancel RadioON LOW grace period if active
  if (radioONLowTimerActive) {
    radioONLowTimerActive = false;
    Serial.println("Ignition ON - RadioON LOW grace period cancelled");
  }
  
  // If we were waiting for ignition after power-on, we're now in normal operation
  if (waitingForIgnition) {
    waitingForIgnition = false;
    Serial.println("Ignition ON detected - normal operation starting");
    
    // NEW: If ignition comes on during initial wake period, we can end the wake timer early
    if (initialWakeTimerActive) {
      Serial.println("Ignition ON during initial wake period - ending wake timer early");
      initialWakeTimerActive = false;
      isInitialPowerOn = false;
    }
  }
  
  // Reset PWR trigger counters when ignition comes back on
  pwrTriggerCount = 0;
  lastPwrTriggerTime = 0;
  isInitialPowerOn = false;  // No longer initial power-on
}

// MODIFIED: Enhanced USB state management - respect ignition state
void updateUSBState() {
  bool radioOnHigh = digitalRead(RadioON) == HIGH;
  bool usbOffCurrent = digitalRead(USBOFF) == HIGH;
  
  // NEW: If ignition is active, keep USB on even if RadioON is temporarily low
  if (isIgnitionActive() && !radioOnHigh) {
    Serial.printf("USB staying ON - ignition active despite RadioON LOW (CAN: 0x%02X)\n", currentCANState);
    if (!usbOffCurrent) {
      digitalWrite(USBOFF, HIGH);
      Serial.println("Setting USBOFF HIGH (ignition active)");
    }
    return;
  }

  // Normal USB state management based on RadioON
  if (radioOnHigh && !usbOffCurrent) {
    digitalWrite(USBOFF, HIGH);
    Serial.println("Setting USBOFF HIGH (RadioON active)");
  } else if (!radioOnHigh && usbOffCurrent && !isIgnitionActive()) {
    digitalWrite(USBOFF, LOW);
    Serial.println("Setting USBOFF LOW (RadioON inactive)");
  }
}

// MODIFIED: New function to handle DisplayOFF based on ignition state - keep on during engine start
void setDisplayLow(const char* reason) {
  // NEW: Don't turn off display if ignition is active (engine starting)
  if (isIgnitionActive()) {
    Serial.printf("Display staying ON - ignition active: %s (CAN: 0x%02X)\n", reason, currentCANState);
    return;
  }
  
  if (digitalRead(DisplayOFF) == HIGH) {
    digitalWrite(DisplayOFF, LOW);
    Serial.printf("Setting DisplayOFF LOW: %s\n", reason);
  }
}

void setDisplayHighIfIgnitionActive(const char* reason) {
  if (isIgnitionActive()) {
    if (digitalRead(DisplayOFF) == LOW) {
      digitalWrite(DisplayOFF, HIGH);
      Serial.printf("Setting DisplayOFF HIGH: %s\n", reason);
    }
  } else {
    // Only turn off if ignition is truly inactive
    if (digitalRead(DisplayOFF) == HIGH) {
      digitalWrite(DisplayOFF, LOW);
      Serial.printf("Setting DisplayOFF LOW: %s (ignition inactive)\n", reason);
    }
  }
}

// MODIFIED: Function to set outputs low before sleep - but check ignition first
void setOutputsLowForSleep() {
  // NEW: Don't turn off outputs if ignition is still active
  if (isIgnitionActive()) {
    Serial.printf("NOT setting outputs low - ignition still active (CAN: 0x%02X)\n", currentCANState);
    return;
  }
  
  digitalWrite(USBOFF, LOW);
  digitalWrite(DisplayOFF, LOW);
  digitalWrite(CANOFF, LOW);  // Pull CANOFF low before sleep
  Serial.println("Setting USBOFF, DisplayOFF, and CANOFF LOW before sleep");
}

// Function to set CANOFF high when awake
void setCANOFFHigh() {
  if (digitalRead(CANOFF) == LOW) {
    digitalWrite(CANOFF, HIGH);
    Serial.println("Setting CANOFF HIGH (system awake)");
  }
}

// MODIFIED: Prevent power cycle if ignition is active
void handlePowerCycle() {
  // NEW: Don't power cycle if ignition is active
  if (isIgnitionActive()) {
    Serial.printf("Power cycle PREVENTED - ignition active (CAN: 0x%02X)\n", currentCANState);
    return;
  }
  
  Serial.println("Executing power cycle...");
  digitalWrite(PWR, LOW);
  delay(250);
  digitalWrite(PWR, HIGH);
  delay(10000);
  Serial.println("Power cycle complete");
}

// NEW: Function to trigger PWR signal
void triggerPWR() {
  Serial.printf("Triggering PWR signal (attempt #%d)\n", pwrTriggerCount + 1);
  digitalWrite(PWR, LOW);
  delay(250);
  digitalWrite(PWR, HIGH);
  
  pwrTriggerCount++;
  lastPwrTriggerTime = millis();
  
  Serial.printf("PWR trigger complete - total triggers this cycle: %d\n", pwrTriggerCount);
}

// ENHANCED: Grace period handling with PWR retry logic
void checkGracePeriod() {
  if (!ignitionOffTimerActive) return;
  
  // NEW: Cancel grace period if ignition becomes active again
  if (isIgnitionActive()) {
    Serial.printf("Grace period CANCELLED - ignition became active (CAN: 0x%02X)\n", currentCANState);
    ignitionOffTimerActive = false;
    isRetryAttempt = false;
    lastDebugTime = 0;
    pwrTriggerCount = 0;
    lastPwrTriggerTime = 0;
    return;
  }
  
  // Use different timeout periods for initial attempt vs retries
  unsigned long timeoutPeriod = isRetryAttempt ? RETRY_PERIOD_MS : GRACE_PERIOD_MS;
  unsigned long elapsed = millis() - ignitionOffTime;
  
  // Check if we need to retry PWR (only if ALIVE is still HIGH and 10s have passed since last PWR)
  if (pwrTriggerCount > 0 && digitalRead(ALIVE) == HIGH) {
    unsigned long timeSinceLastPwr = millis() - lastPwrTriggerTime;
    if (timeSinceLastPwr >= PWR_RETRY_INTERVAL_MS) {
      Serial.printf("ALIVE still HIGH after %lu ms - triggering PWR retry\n", timeSinceLastPwr);
      triggerPWR();
      
      // Reset the retry timer - give it another chance
      ignitionOffTime = millis();
      isRetryAttempt = true;
      lastDebugTime = 0;
      return;  // Exit early to start fresh timing
    }
  }
  
  // Debug output every 2 seconds during grace period
  if (millis() - lastDebugTime > 2000) {
    const char* periodType;
    if (isRetryAttempt) {
      periodType = pwrTriggerCount > 0 ? "Waiting for ALIVE LOW" : "Retry";
    } else {
      periodType = waitingForIgnition ? "Radio OFF shutdown" : "Grace";
    }
    
    unsigned long timeSinceLastPwr = pwrTriggerCount > 0 ? (millis() - lastPwrTriggerTime) : 0;
    
    Serial.printf("%s period: %lu/%lu ms (RadioON: %d, ALIVE: %d, PWR count: %d, Time since PWR: %lu ms, CAN: 0x%02X)\n", 
                  periodType, elapsed, timeoutPeriod,
                  digitalRead(RadioON), digitalRead(ALIVE),
                  pwrTriggerCount, timeSinceLastPwr, currentCANState);
    lastDebugTime = millis();
  }
  
  // Check if grace period has expired
  if (elapsed >= timeoutPeriod) {
    const char* periodType = isRetryAttempt ? 
      (pwrTriggerCount > 0 ? "Wait for ALIVE" : "Retry") : "Grace";
    Serial.printf("%s period expired - Triggering power management\n", periodType);
    ignitionOffTimerActive = false;
    triggerShutdownSequence();
  }
}

bool isIgnitionActive() {
  // Only allow Display HIGH when ignition is in active states
  switch (currentCANState) {
    case 0x41:
    case 0x42:
    case 0x45:
    case 0x55:
      return true;
    default:
      return false;
  }
}

// MODIFIED: Shutdown sequence with ignition state checking
void triggerShutdownSequence() {
  Serial.println("=== EXECUTING SHUTDOWN SEQUENCE ===");

  // NEW: Final check - don't shutdown if ignition became active
  if (isIgnitionActive()) {
    Serial.printf("SHUTDOWN ABORTED - ignition became active (CAN: 0x%02X)\n", currentCANState);
    ignitionOffTimerActive = false;
    isRetryAttempt = false;
    lastDebugTime = 0;
    pwrTriggerCount = 0;
    lastPwrTriggerTime = 0;
    updateUSBState();
    setDisplayHighIfIgnitionActive("Ignition active during shutdown attempt");
    setCANOFFHigh();
    Serial.println("=== SHUTDOWN SEQUENCE ABORTED ===");
    return;
  }

  setOutputsLowForSleep();

  // Trigger PWR if this is the first attempt in this cycle
  if (pwrTriggerCount == 0) {
    Serial.println("Initial PWR trigger for shutdown");
    triggerPWR();
    
    Serial.println("PWR cycle complete - waiting for ALIVE to go LOW");
    delay(2000);  // Give system time to respond
  } else {
    Serial.printf("PWR already triggered %d times this cycle - checking ALIVE state\n", pwrTriggerCount);
  }

  // Check if ALIVE has gone low (indicating system is responding to shutdown)
  if (digitalRead(ALIVE) == LOW) {
    Serial.println("ALIVE is LOW - checking if we can sleep");
    
    SystemState currentState = getSystemState();
    Serial.printf("State check: RadioON=%d, ALIVE=%d, State=%d\n",
                  digitalRead(RadioON), digitalRead(ALIVE), currentState);
                  
    if (currentState == STATE_SLEEP) {
      Serial.printf("Entering deep sleep after %d PWR attempts...\n", pwrTriggerCount);
      
      // Suspend CAN task before sleep
      if (canTaskHandle != NULL) {
        vTaskSuspend(canTaskHandle);
      }
      
      configureWakeupSources();
      esp_deep_sleep_start();
    } else {
      // RadioON is still HIGH, wait for it to go LOW
      Serial.printf("Cannot sleep - RadioON still HIGH (%d)\n", digitalRead(RadioON));
      
      if (!isIgnitionActive()) {
        Serial.println("Ignition still OFF - will retry sleep check in 5 seconds");
        ignitionOffTime = millis();
        ignitionOffTimerActive = true;
        isRetryAttempt = true;
        lastDebugTime = 0;
        setOutputsLowForSleep();
      } else {
        Serial.println("Ignition is ON according to CAN - returning to normal operation");
        ignitionOffTimerActive = false;
        isRetryAttempt = false;
        lastDebugTime = 0;
        pwrTriggerCount = 0;  // Reset PWR trigger count
        lastPwrTriggerTime = 0;
        updateUSBState();
        setDisplayHighIfIgnitionActive("Ignition active after shutdown attempt");
        setCANOFFHigh();  // Ensure CANOFF is high when returning to normal operation
      }
    }
  } else {
    // ALIVE is still HIGH - system hasn't responded to shutdown yet
    Serial.printf("ALIVE still HIGH (%d) after %d PWR attempts\n", digitalRead(ALIVE), pwrTriggerCount);
    
    if (!isIgnitionActive()) {
      Serial.println("Ignition still OFF - will check ALIVE again and potentially retry PWR in 5 seconds");
      ignitionOffTime = millis();
      ignitionOffTimerActive = true;
      isRetryAttempt = true;
      lastDebugTime = 0;
      setOutputsLowForSleep();
    } else {
      Serial.println("Ignition is ON according to CAN - returning to normal operation");
      ignitionOffTimerActive = false;
      isRetryAttempt = false;
      lastDebugTime = 0;
      pwrTriggerCount = 0;  // Reset PWR trigger count
      lastPwrTriggerTime = 0;
      updateUSBState();
      setDisplayHighIfIgnitionActive("Ignition active after shutdown attempt");
      setCANOFFHigh();  // Ensure CANOFF is high when returning to normal operation
    }
  }
  
  Serial.println("=== SHUTDOWN SEQUENCE COMPLETE ===");
}

SystemState getSystemState() {
  bool radioOn = digitalRead(RadioON) == HIGH;
  bool aliveOn = digitalRead(ALIVE) == HIGH;

  if (!radioOn && !aliveOn) return STATE_SLEEP;
  if (!radioOn && aliveOn) return STATE_ALIVE_ONLY;
  if (radioOn && !aliveOn) return STATE_RADIO_ONLY;
  return STATE_FULL_ACTIVE;
}

// MODIFIED: Enhanced setup function with RadioON state tracking
void setup() {
  setCpuFrequencyMhz(80);

  pinMode(ALIVE, INPUT);
  pinMode(RadioON, INPUT);
  pinMode(PWR, OUTPUT);
  pinMode(USBOFF, OUTPUT);
  pinMode(DisplayOFF, OUTPUT);
  pinMode(CANOFF, OUTPUT);  // Configure CANOFF as output

  digitalWrite(PWR, HIGH);
  digitalWrite(CANOFF, HIGH);  // Set CANOFF HIGH on startup (system is awake)

  // Initialize state variables
  pwrTriggerCount = 0;
  lastPwrTriggerTime = 0;
  powerOnPwrTriggered = false;
  isInitialPowerOn = true;
  waitingForIgnition = false;
  powerOnTime = millis();
  
  // NEW: Initialize RadioON state tracking
  lastRadioONState = digitalRead(RadioON) == HIGH;
  radioONLowStartTime = 0;
  radioONLowTimerActive = false;
  
  // NEW: Initialize initial wake timer variables
  initialWakeTimerActive = false;
  initialWakeTime = 0;
  lastInitialWakeDebugTime = 0;

  Serial.begin(921600);
  while (!Serial)
    ;
  Serial.println("ESP32 CAN Bus Monitor Starting...");
  Serial.printf("Monitoring CAN ID: 0x%03X\n", TARGET_CAN_ID);
  Serial.printf("Initial pin states: RadioON=%d, ALIVE=%d, CANOFF=%d\n", 
                digitalRead(RadioON), digitalRead(ALIVE), digitalRead(CANOFF));

  // Initialize CAN Bus
  initCAN();

  // Create mutex for thread-safe CAN state access
  canStateMutex = xSemaphoreCreateMutex();
  if (canStateMutex == NULL) {
    Serial.println("Failed to create CAN state mutex");
    return;
  }

  // Create CAN monitoring task on Core 0 (Core 1 runs Arduino loop)
  xTaskCreatePinnedToCore(
    canMonitorTask,  // Task function
    "CAN_Monitor",   // Task name
    4096,            // Stack size (bytes)
    NULL,            // Task parameter
    2,               // Priority (higher than loop)
    &canTaskHandle,  // Task handle
    0                // Core 0
  );

  // Configure wake-up sources for both pins
  configureWakeupSources();

  // Initial state synchronization
  updateUSBState();
  setDisplayHighIfIgnitionActive("System startup");

  Serial.println("Setup complete - CAN monitoring active");
  Serial.println("Wake-up configured for RadioON (GPIO 4) or ALIVE (GPIO 26)");
  Serial.println("RadioON LOW protection active - will not shutdown during engine start if ignition is ON");
}

// MODIFIED: Enhanced main loop with RadioON transition monitoring
void loop() {
  // Ensure CANOFF is HIGH when system is awake
  setCANOFFHigh();
  
  // Process CAN state changes (only when they occur)
  processCANStateChange();

  // NEW: HIGHEST PRIORITY - Monitor RadioON transitions to detect engine start
  monitorRadioONTransitions();

  // Update USB state based on RadioON and ignition state - ALWAYS call this
  updateUSBState();

  // NEW: Check initial wake timer
  checkInitialWakeTimer();
  
  // If we're in initial wake period, skip most other logic
  if (isInInitialWakePeriod()) {
    // During initial wake period, keep system alive but still handle power-on sequence
    if (isInitialPowerOn) {
      handlePowerOnSequence();
    }
    
    // Keep display state appropriate during wake period
    if (isIgnitionActive()) {
      setDisplayHighIfIgnitionActive("Initial wake - ignition active");
    } else {
      setDisplayLow("Initial wake - ignition inactive");
    }
    
    delay(100);
    return;  // Skip rest of loop during initial wake period
  }

  // Handle initial power-on sequence (only if not in wake period)
  if (isInitialPowerOn) {
    handlePowerOnSequence();
  }
  
  // Handle RadioON going LOW while waiting for ignition (only if not in wake period and not protected by ignition)
  handleRadioOffWhileWaitingForIgnition();

  // PRIORITY: Check grace period timer - this can override normal state management
  checkGracePeriod();
  
  // If we're in shutdown mode (grace period active), skip normal state management
  if (ignitionOffTimerActive) {
    // During shutdown attempts, only do essential updates
    setDisplayLow("Shutdown pending");
    delay(100);
    return;  // Skip normal state management
  }

  // Handle system states (only when not in shutdown mode or initial wake period)
  SystemState currentState = getSystemState();

  switch (currentState) {
    case STATE_SLEEP:
      setOutputsLowForSleep();
      Serial.println("State: 00 - Entering deep sleep, wake on RadioON or ALIVE HIGH");

      // Suspend CAN task before sleep
      if (canTaskHandle != NULL) {
        vTaskSuspend(canTaskHandle);
      }

      configureWakeupSources();
      esp_deep_sleep_start();
      break;

    case STATE_ALIVE_ONLY:
      setDisplayLow("State: 01 - ALIVE only");
      if (digitalRead(ALIVE) == HIGH) {
        handlePowerCycle();  // This now checks ignition state before power cycling
      }
      break;

    case STATE_RADIO_ONLY:
      if (waitingForIgnition) {
        setDisplayLow("State: 10 - Waiting for ignition");
      } else {
        setDisplayHighIfIgnitionActive("State: 10 - RadioON active");
      }
      if (digitalRead(ALIVE) == LOW) {
        handlePowerCycle();  // This now checks ignition state before power cycling
      }
      break;

    case STATE_FULL_ACTIVE:
      if (waitingForIgnition) {
        setDisplayLow("State: 11 - Waiting for ignition");
      } else {
        setDisplayHighIfIgnitionActive("State: 11 - Both active");
      }
      break;
  }

  delay(100);  // Main loop runs at comfortable pace
}