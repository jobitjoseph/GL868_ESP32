/*
 * GL868_ESP32.cpp
 * Main library implementation for GL868_ESP32 GPS Tracker Library
 *
 * Copyright (c) 2026 Jobit Joseph and Semicon Media
 * https://github.com/jobitjoseph/GL868_ESP32
 * https://github.com/jobitjoseph/GL868_ESP32
 * Licensed under MIT License
 */

#include "GL868_ESP32.h"

// Runtime log level (declared in Logger.h)
LogLevel _glesp868_runtime_log_level = (LogLevel)GL868_ESP32_LOG_LEVEL;

// Global instance
GL868_ESP32 GeoLinker;

GL868_ESP32::GL868_ESP32()
    : _initialized(false), _forceSendRequested(false),
      _motionTriggerEnabled(false), _fullPowerOffEnabled(true),
      _offlineSentFirst(false), _gpsTimeout(GL868_ESP32_GPS_FIX_TIMEOUT),
      _stateTimeout(0), _payloadCount(0), _operatingMode(MODE_FULL),
      _errorRebootTimeout(300000) { // 5 minutes default
  _deviceId[0] = '\0';
  _apiKey[0] = '\0';
  _firstGPSTimestamp[0] = '\0';
}

void GL868_ESP32::begin(const char *deviceId, const char *apiKey) {
  GL868_ESP32_LOG_I("=================================");
  GL868_ESP32_LOG_I("GL868_ESP32 Library Initializing");
  GL868_ESP32_LOG_I("=================================");

  // Store credentials
  strncpy(_deviceId, deviceId, GL868_ESP32_MAX_DEVICE_ID_LEN - 1);
  _deviceId[GL868_ESP32_MAX_DEVICE_ID_LEN - 1] = '\0';

  strncpy(_apiKey, apiKey, GL868_ESP32_MAX_API_KEY_LEN - 1);
  _apiKey[GL868_ESP32_MAX_API_KEY_LEN - 1] = '\0';

  // Initialize components
  led.begin();
  led.setState(LED_BOOT);
  battery.begin();
  modem.begin();

  // Link components that need references
  gsm.setModem(&modem);
  gps.setModem(&modem);
  sms.setModem(&modem);
  sms.setWhitelist(&whitelist);
  call.setModem(&modem);
  call.setWhitelist(&whitelist);
  battery.setModem(&modem); // For AT+CBC readings

  // Set up action callbacks
  sms.setActionCallback([](const char *action, const char *args) {
    GeoLinker.processActionRequest(action, args);
  });
  call.setActionCallback([](const char *action, const char *caller) {
    GeoLinker.processActionRequest(action, caller);
  });

  // Set up LED callbacks for SMS/Call visual feedback
  sms.setLEDCallback([](LEDState state) { GeoLinker.led.setState(state); });
  call.setLEDCallback([](LEDState state) { GeoLinker.led.setState(state); });

  // Initialize motion sensor
  if (!motion.begin()) {
    GL868_ESP32_LOG_E("Motion sensor init failed");
  }

  // Initialize whitelist (loads from NVS)
  whitelist.begin();

  // Initialize queue (default: RAM)
  queue.begin(QUEUE_RAM, GL868_ESP32_RAM_QUEUE_SIZE);

  // Initialize JSON builder
  json.begin(deviceId);

  // Initialize state machine
  stateMachine.begin();

  // Initialize sleep manager
  sleep.begin();

  _initialized = true;

  GL868_ESP32_LOG_I("Device ID: %s", _deviceId);
  GL868_ESP32_LOG_I("Wake source: %s",
                    GL868_ESP32_WakeSourceToString(sleep.getWakeSource()));
  GL868_ESP32_LOG_I("Initialization complete");
}

void GL868_ESP32::update() {
  if (!_initialized)
    return;

  // Check if state changed from PREVIOUS update cycle (flag is cleared at start
  // of update()) Must check BEFORE stateMachine.update() because it clears the
  // flag
  if (stateMachine.stateChanged()) {
    updateLEDForState();
  }

  // Update state machine (this may trigger new state changes)
  stateMachine.update();

  // Check again in case state changed during this update
  if (stateMachine.stateChanged()) {
    updateLEDForState();
  }

  // LED update is now handled by background task in GL868_ESP32_LED

  // Update SMS/Call processing if enabled
  sms.update();
  call.update();

  // Handle current state
  SystemState state = stateMachine.getState();

  // State handlers
  switch (state) {
  case STATE_BOOT:
    handleBoot();
    break;
  case STATE_MODEM_POWER_ON:
    handleModemPowerOn();
    break;
  case STATE_GSM_INIT:
    handleGSMInit();
    break;
  case STATE_GSM_REGISTER:
    handleGSMRegister();
    break;
  case STATE_GPS_POWER_ON:
    handleGPSPowerOn();
    break;
  case STATE_GPS_WAIT_FIX:
    handleGPSWaitFix();
    break;
  case STATE_BUILD_JSON:
    handleBuildJSON();
    break;
  case STATE_GPRS_ATTACH:
    handleGPRSAttach();
    break;
  case STATE_SEND_HTTP:
    handleSendHTTP();
    break;
  case STATE_SEND_OFFLINE_QUEUE:
    handleSendOfflineQueue();
    break;
  case STATE_SLEEP_PREPARE:
    handleSleepPrepare();
    break;
  case STATE_SLEEP:
    handleSleep();
    break;
  case STATE_IDLE:
    handleIdle();
    break;
  }
}

void GL868_ESP32::updateLEDForState() {
  SystemState state = stateMachine.getState();

  switch (state) {
  case STATE_BOOT:
    led.setState(LED_BOOT);
    break;
  case STATE_MODEM_POWER_ON:
    led.setState(LED_MODEM_POWER_ON);
    break;
  case STATE_GSM_INIT:
    led.setState(LED_GSM_INIT);
    break;
  case STATE_GSM_REGISTER:
    led.setState(LED_GSM_REGISTER);
    break;
  case STATE_GPS_POWER_ON:
    led.setState(LED_GPS_POWER_ON);
    break;
  case STATE_GPS_WAIT_FIX:
    led.setState(LED_GPS_WAIT_FIX);
    break;
  case STATE_BUILD_JSON:
    led.setState(LED_BUILD_JSON);
    break;
  case STATE_GPRS_ATTACH:
    led.setState(LED_GPRS_ATTACH);
    break;
  case STATE_SEND_HTTP:
    led.setState(LED_SEND_HTTP);
    break;
  case STATE_SEND_OFFLINE_QUEUE:
    led.setState(LED_SEND_OFFLINE);
    break;
  case STATE_SLEEP_PREPARE:
    led.setState(LED_SLEEP_PREPARE);
    break;
  case STATE_SLEEP:
    led.setState(LED_SLEEP);
    break;
  case STATE_IDLE:
    led.setState(LED_IDLE);
    break;
  }
}

// ============================================================================
// State Handlers
// ============================================================================

void GL868_ESP32::handleBoot() {
  // Reset offline flag at start of each wake cycle
  _offlineSentFirst = false;

  // Reset error blink loop counter (fresh cycle)
  led.resetErrorBlinks();

  // Boot state - initial setup done, move to modem power on
  if (stateMachine.getTimeInState() > 100) {
    stateMachine.forceState(STATE_MODEM_POWER_ON);
  }
}

void GL868_ESP32::handleModemPowerOn() {
  if (stateMachine.stateChanged()) {
    GL868_ESP32_LOG_I("Powering on modem...");
  }

  if (modem.powerOn()) {
    stateMachine.forceState(STATE_GSM_INIT);
  } else {
    if (!stateMachine.incrementRetries(GL868_ESP32_AT_RETRY_COUNT)) {
      stateMachine.reportError(ERROR_MODEM);
      led.setError(ERROR_MODEM);
      stateMachine.forceState(STATE_SLEEP_PREPARE);
    }
  }
}

void GL868_ESP32::handleGSMInit() {
  if (stateMachine.stateChanged()) {
    GL868_ESP32_LOG_I("Initializing GSM...");
  }

  // Check SIM
  if (!gsm.checkSIMReady()) {
    if (!stateMachine.incrementRetries(GL868_ESP32_AT_RETRY_COUNT)) {
      stateMachine.reportError(ERROR_NO_SIM);
      led.setError(ERROR_NO_SIM);
      stateMachine.forceState(STATE_SLEEP_PREPARE);
      return;
    }
    return;
  }

  // Initialize SMS/Call if enabled
  if (sms.isEnabled()) {
    sms.begin();
  }
  if (call.isEnabled()) {
    call.begin();
  }

  stateMachine.forceState(STATE_GSM_REGISTER);
}

void GL868_ESP32::handleGSMRegister() {
  if (stateMachine.stateChanged()) {
    GL868_ESP32_LOG_I("Waiting for network registration...");
    _stateTimeout = millis();
  }

  // Check timeout
  if (millis() - _stateTimeout > GL868_ESP32_NETWORK_REG_TIMEOUT) {
    stateMachine.reportError(ERROR_NO_NETWORK);
    led.setError(ERROR_NO_NETWORK);
    queueCurrentData();
    stateMachine.forceState(STATE_SLEEP_PREPARE);
    return;
  }

  if (gsm.waitNetworkRegistration(5000)) {
    // If woken by a Call or SMS: skip GeoLinker upload entirely.
    // The SMS/Call handlers inside update() will get the GPS fix and
    // send a location reply via SMS to the caller/sender.
    WakeSource ws = sleep.getWakeSource();
    if (ws == WAKE_CALL || ws == WAKE_SMS) {
      GL868_ESP32_LOG_I(
          "Call/SMS wake - skipping GeoLinker upload, entering idle");
      stateMachine.forceState(STATE_IDLE);
      return;
    }

    // Mode-aware routing after network registration
    if (!(_operatingMode & MODE_DATA)) {
      // SMS-only, Call-only, or SMS+Call mode: skip GPS/HTTP, go to IDLE
      GL868_ESP32_LOG_I("Non-data mode - entering idle for SMS/Call");
      stateMachine.forceState(STATE_IDLE);
    } else if (!queue.isEmpty() && !_offlineSentFirst) {
      // OFFLINE PRIORITY: send offline data FIRST before acquiring new GPS fix
      GL868_ESP32_LOG_I("Offline queue has %d entries - sending first",
                        queue.count());
      _offlineSentFirst = true;
      stateMachine.forceState(STATE_GPRS_ATTACH);
    } else {
      stateMachine.forceState(STATE_GPS_POWER_ON);
    }
  } else {
    // Check if wait failed due to a fatal status (0, 3, 4) or just timeout
    // searching
    int stat = gsm.getRegistrationStatus();
    if (stat == 0 || stat == 3 || stat == 4) {
      GL868_ESP32_LOG_E("Immediate registration failure (status: %d)", stat);
      stateMachine.reportError(ERROR_NO_NETWORK);
      led.setError(ERROR_NO_NETWORK);
      queueCurrentData();
      stateMachine.forceState(STATE_SLEEP_PREPARE);
    }
    // If stat is 2 (searching), stay in this state until timeout
  }
}

void GL868_ESP32::handleGPSPowerOn() {
  if (stateMachine.stateChanged()) {
    GL868_ESP32_LOG_I("Powering on GPS...");
  }

  if (gps.powerOn()) {
    stateMachine.forceState(STATE_GPS_WAIT_FIX);
  } else {
    if (!stateMachine.incrementRetries(3)) {
      stateMachine.reportError(ERROR_NO_GPS);
      led.setError(ERROR_NO_GPS);
      stateMachine.forceState(STATE_SLEEP_PREPARE);
    }
  }
}

void GL868_ESP32::handleGPSWaitFix() {
  if (stateMachine.stateChanged()) {
    GL868_ESP32_LOG_I("Waiting for GPS fix...");
    _stateTimeout = millis();
    _firstGPSTimestamp[0] = '\0'; // Reset so we capture the first reading
  }

  // Check timeout
  if (millis() - _stateTimeout > _gpsTimeout) {
    GL868_ESP32_LOG_E("GPS fix timeout");

    if (!stateMachine.incrementRetries(GL868_ESP32_GPS_RETRY_COUNT)) {
      stateMachine.reportError(ERROR_NO_GPS);
      led.setError(ERROR_NO_GPS);
      gps.powerOff();
      stateMachine.forceState(STATE_SLEEP_PREPARE);
      return;
    }

    // Reset timeout for retry
    _stateTimeout = millis();
    return;
  }

  // Accept the first valid fix — GPS was power-cycled in powerOn() so any
  // valid reading here is guaranteed to be fresh satellite data.
  if (gps.getReading(&_currentGPS)) {
    if (_currentGPS.valid) {
      // SIM868 hot-starts from NVRAM even after AT+CGNSPWR=0, returning the
      // last cached position with its old timestamp immediately on power-on.
      // With satellites locked the engine updates every ~1 second.
      // Strategy: record the first timestamp seen, then wait until the GPS
      // produces a reading with a NEWER timestamp — that guarantees the fix
      // is a current satellite-based solution, not the NVRAM cache.
      if (_firstGPSTimestamp[0] == '\0') {
        // First valid reading — stash timestamp and keep waiting
        strncpy(_firstGPSTimestamp, _currentGPS.timestamp,
                sizeof(_firstGPSTimestamp) - 1);
        _firstGPSTimestamp[sizeof(_firstGPSTimestamp) - 1] = '\0';
        GL868_ESP32_LOG_I("Hot-start fix (stale): %s — waiting for current",
                          _firstGPSTimestamp);
        return;
      }

      // Accept only once the GPS has advanced its timestamp (new satellite fix)
      if (strcmp(_currentGPS.timestamp, _firstGPSTimestamp) != 0) {
        GL868_ESP32_LOG_I("GPS fix: %.6f, %.6f (ts: %s)", _currentGPS.latitude,
                          _currentGPS.longitude, _currentGPS.timestamp);
        gps.powerOff();
        stateMachine.forceState(STATE_BUILD_JSON);
      }
      // Same timestamp as first reading = still hot-start cache, keep polling
    }
  }
}

void GL868_ESP32::handleBuildJSON() {
  if (stateMachine.stateChanged()) {
    GL868_ESP32_LOG_I("Building JSON payload...");
  }

  // Get battery
  _currentBattery = battery.readPercentage();

  // Get signal strength
  int signalStrength = gsm.getSignalStrength();

  // Clear and add current data point
  json.clear();
  json.addDataPoint(_currentGPS, _currentBattery, signalStrength);

  // Set payload fields
  if (_payloadCount > 0) {
    json.setPayloadFields(_payloadFields, _payloadCount);
  }

  stateMachine.forceState(STATE_GPRS_ATTACH);
}

void GL868_ESP32::handleGPRSAttach() {
  if (stateMachine.stateChanged()) {
    GL868_ESP32_LOG_I("Attaching GPRS...");
  }

  // Check timeout using state machine's built-in timer (more reliable)
  if (stateMachine.getTimeInState() > GL868_ESP32_GPRS_ATTACH_TIMEOUT) {
    stateMachine.reportError(ERROR_NO_GPRS);
    led.setError(ERROR_NO_GPRS);
    queueCurrentData();
    stateMachine.forceState(STATE_SLEEP_PREPARE);
    return;
  }

  if (gsm.attachGPRS()) {
    // If sending offline data first, go to offline queue
    // Otherwise, send fresh HTTP data
    if (_offlineSentFirst && !queue.isEmpty()) {
      stateMachine.forceState(STATE_SEND_OFFLINE_QUEUE);
    } else {
      stateMachine.forceState(STATE_SEND_HTTP);
    }
  }
}

void GL868_ESP32::handleSendHTTP() {
  if (stateMachine.stateChanged()) {
    GL868_ESP32_LOG_I("Sending data to server...");
  }

  // Check rate limit
  if (!sleep.canSendNow()) {
    GL868_ESP32_LOG_D("Rate limit, waiting...");
    return;
  }

  bool success = sendDataToServer();

  if (success) {
    sleep.resetLastSendTime();
    stateMachine.reportDataSent(true, 200);

    // Check for offline queue
    if (!queue.isEmpty()) {
      stateMachine.forceState(STATE_SEND_OFFLINE_QUEUE);
    } else {
      // After a successful GeoLinker upload (timer/motion/power-on wake),
      // go to sleep. Do NOT loop back to IDLE — that caused continuous
      // 10-second re-send loops. Call/SMS wakes never reach SEND_HTTP
      // (they route to IDLE in GSM_REGISTER), so this is safe for all cases.
      GL868_ESP32_LOG_I("Data sent - entering sleep");
      stateMachine.forceState(STATE_SLEEP_PREPARE);
    }
  } else {
    if (!stateMachine.incrementRetries(GL868_ESP32_HTTP_RETRY_COUNT)) {
      stateMachine.reportError(ERROR_HTTP_FAIL);
      led.setError(ERROR_HTTP_FAIL);
      queueCurrentData();
      stateMachine.forceState(STATE_SLEEP_PREPARE);
    }
  }
}

void GL868_ESP32::handleSendOfflineQueue() {
  if (stateMachine.stateChanged()) {
    GL868_ESP32_LOG_I("Sending offline queue (%d entries)...", queue.count());
  }

  // Check rate limit
  if (!sleep.canSendNow()) {
    return;
  }

  // Check if queue is empty
  if (queue.isEmpty()) {
    // Queue is empty, proceed to next step
    if (_offlineSentFirst) {
      GL868_ESP32_LOG_I("Offline queue done - now getting fresh GPS");
      stateMachine.forceState(STATE_GPS_POWER_ON);
    } else {
      stateMachine.forceState(STATE_SLEEP_PREPARE);
    }
    return;
  }

  // Send only ONE entry at a time to reduce stack usage and simplify logic
  QueueEntry entry;
  if (!queue.pop(&entry)) {
    GL868_ESP32_LOG_E("Failed to pop queue entry");
    stateMachine.forceState(STATE_SLEEP_PREPARE);
    return;
  }

  // Build JSON with single entry
  json.clear();
  json.addDataPoint(entry.gps, entry.battery);

  if (sendDataToServer()) {
    // Success! Entry is already removed from queue
    sleep.resetLastSendTime();
    GL868_ESP32_LOG_I("Sent 1 queued entry successfully (remaining: %d)",
                      queue.count());

    // Continue with next entry if available, else move on
    if (!queue.isEmpty()) {
      return; // Stay in this state for next entry
    }
  } else {
    // Failed! Push entry back to queue for next attempt
    GL868_ESP32_LOG_E("Failed to send queue - preserving entry for retry");
    queue.push(entry);

    // Go to sleep and try again next wake
    stateMachine.forceState(STATE_SLEEP_PREPARE);
    return;
  }

  // If we sent offline data first this wake cycle, now get fresh GPS
  if (_offlineSentFirst) {
    GL868_ESP32_LOG_I("Offline queue done - now getting fresh GPS");
    stateMachine.forceState(STATE_GPS_POWER_ON);
  } else {
    // This was a normal queue send after fresh GPS data
    stateMachine.forceState(STATE_SLEEP_PREPARE);
  }
}

void GL868_ESP32::handleSleepPrepare() {
  if (stateMachine.stateChanged()) {
    GL868_ESP32_LOG_I("Preparing for sleep...");
  }

  // If an error was reported this cycle, wait for the LED blink pattern
  // (GL868_ESP32_ERROR_BLINK_LOOPS repeats) to complete before sleeping
  // so the user can read the error code from the LED.
  if (led.getState() == LED_ERROR && !led.errorBlinksComplete()) {
    // Check for auto-reboot timeout for critical errors
    if (_errorRebootTimeout > 0 && 
        (stateMachine.getLastError() == ERROR_NO_SIM || stateMachine.getLastError() == ERROR_NO_NETWORK)) {
      if (stateMachine.getTimeInState() > _errorRebootTimeout) {
         GL868_ESP32_LOG_E("Critical error timeout reached. Rebooting ESP32...");
         delay(100);
         ESP.restart();
      }
    }
    // Still showing error pattern — stay in this state and wait
    return;
  }

  // Error pattern done (or no error) — turn off RGB and proceed
  led.off();

  // Disable motion interrupt before sleep
  motion.disableInterrupt();

  // Continuous Motion Check (5 seconds)
  if (_motionTriggerEnabled || sleep.isMotionWake() || sleep.isMotionWakeEnabled()) {
     GL868_ESP32_LOG_I("Checking for continuous motion for 5 seconds...");
     motion.clearInterrupt();
     motion.enableInterrupt(); // Re-enable to catch new events
     
     bool motionContinued = false;
     uint32_t startCheck = millis();
     while (millis() - startCheck < 5000) {
         if (motion.motionDetected()) {
             motionContinued = true;
             break;
         }
         delay(100);
     }
     
     if (motionContinued) {
         GL868_ESP32_LOG_I("Continuous motion detected! Aborting sleep and starting new cycle.");
         
         // Need to clear sleep/wake flags and force back to BOOT
         stateMachine.clearSleepRequest();
         stateMachine.forceState(STATE_BOOT); // Restart cycle
         return;
     }
     
     // Disable again if no motion, code will re-enable shortly after for deep sleep
     motion.disableInterrupt(); 
  }

  // Handle modem power based on configuration
  if (sleep.shouldFullPowerOff()) {
    GL868_ESP32_LOG_I("Full modem power off");
    modem.powerOff();
  } else if (sleep.isCallSMSMonitoringEnabled()) {
    // Keep modem at full power (CFUN=1) so it can receive calls/SMS
    // and pulse the RI pin to wake ESP32 from deep sleep.
    // Do NOT set CFUN=0/4 — that disables RF and prevents call/SMS reception.
    GL868_ESP32_LOG_I("Modem stays at CFUN=1 for Call/SMS monitoring");
    // Clean SMS inbox before sleep so only fresh messages wake us next time
    if (sms.isEnabled()) {
      sms.deleteAllSMS();
    }
    // Re-initialize SMS/Call so modem is ready to receive while ESP32 sleeps
    if (sms.isEnabled()) {
      sms.begin();
    }
    if (call.isEnabled()) {
      call.begin();
    }
  } else {
    modem.powerOff();
  }

  // Re-enable motion interrupt for wake
  // Check ALL motion-related flags: motionTrigger OR motionWake
  if (_motionTriggerEnabled || sleep.isMotionWake() ||
      sleep.isMotionWakeEnabled()) {
    // Clear interrupt with full HPF flush (disables INT1_CFG internally)
    motion.clearInterrupt();

    // Re-enable the interrupt for wake detection
    motion.enableInterrupt();

    // CRITICAL: Final clear right before sleep transition
    // enableInterrupt() writes INT1_CFG which can immediately re-latch
    // if there's any residual acceleration. This final clear catches that.
    delay(30); // Let any re-latch settle
    motion.clearInterrupt();

    GL868_ESP32_LOG_D("Motion INT pin state before sleep: %d",
                      digitalRead(GL868_ESP32_MOTION_INT));
  }

  stateMachine.forceState(STATE_SLEEP);
}

void GL868_ESP32::handleSleep() {
  if (stateMachine.stateChanged()) {
    GL868_ESP32_LOG_I("Entering deep sleep...");
  }

  // Wait for modem power-off to complete and log output
  delay(1000);

  // Final interrupt clear right before deep sleep (absolute last chance)
  if (_motionTriggerEnabled || sleep.isMotionWakeEnabled()) {
    motion.clearInterrupt();
  }

  // Enter deep sleep
  sleep.enterSleep();

  // Should never reach here (device reboots after sleep)
}

void GL868_ESP32::handleIdle() {
  if (stateMachine.stateChanged()) {
    GL868_ESP32_LOG_I("Entering idle mode (listening for SMS/Call)...");
    _stateTimeout = millis();
  }

  // SMS/Call update is handled in main update() already

  // For Call/SMS wakes: stay in IDLE long enough for SMS/Call handlers to
  // process the incoming event (get GPS, send location SMS), then sleep.
  // Give it up to 120 seconds to complete the GPS fix + SMS reply.
  WakeSource ws = sleep.getWakeSource();
  if (ws == WAKE_CALL || ws == WAKE_SMS) {
    // Check if we've been in idle long enough for handlers to complete
    if (stateMachine.getTimeInState() > 10000) {
      GL868_ESP32_LOG_I("Call/SMS processing complete - going to sleep");
      stateMachine.forceState(STATE_SLEEP_PREPARE);
      return;
    }
    // Stay in idle — SMS/Call handlers are working via update()
    return;
  }

  // Check if sleep interval has elapsed (for non-data modes staying in IDLE)
  if (sleep.canSendNow() && (_operatingMode & MODE_DATA)) {
    // Time to send data again
    GL868_ESP32_LOG_I("Idle timeout - starting data cycle");
    stateMachine.forceState(STATE_GPS_POWER_ON);
    return;
  }

  // Check for force send request
  if (_forceSendRequested) {
    _forceSendRequested = false;
    GL868_ESP32_LOG_I("Force send requested from idle");
    stateMachine.forceState(STATE_GPS_POWER_ON);
    return;
  }
}

// ============================================================================
// Helper Methods
// ============================================================================

bool GL868_ESP32::sendDataToServer() {
  char jsonBuffer[GL868_ESP32_MAX_JSON_SIZE];

  if (!json.build(jsonBuffer, sizeof(jsonBuffer))) {
    GL868_ESP32_LOG_E("JSON build failed");
    return false;
  }

  // Select endpoint based on config
#if GL868_ESP32_USE_HTTPS
  const char *endpoint = GL868_ESP32_API_ENDPOINT;
  GL868_ESP32_LOG_D("Using HTTPS endpoint");
#else
  const char *endpoint = GL868_ESP32_API_ENDPOINT_HTTP;
  GL868_ESP32_LOG_D("Using HTTP endpoint");
#endif

  int httpCode =
      gsm.httpPOST(endpoint, _apiKey, "application/json", jsonBuffer);

  if (httpCode >= 200 && httpCode < 300) {
    return true;
  }

  GL868_ESP32_LOG_E("HTTP error: %d", httpCode);
  return false;
}

void GL868_ESP32::queueCurrentData() {
  if (!_currentGPS.valid)
    return;

  QueueEntry entry;
  entry.gps = _currentGPS;
  entry.battery = _currentBattery;
  entry.storedTime = millis();

  // Copy payload fields
  entry.payloadCount = _payloadCount;
  for (uint8_t i = 0; i < _payloadCount; i++) {
    entry.payload[i] = _payloadFields[i];
  }

  if (queue.push(entry)) {
    GL868_ESP32_LOG_I("Data queued for later send (queue: %d)", queue.count());
  }
}

void GL868_ESP32::processActionRequest(const char *action, const char *args) {
  GL868_ESP32_LOG_D("Action request: %s, args: %s", action, args);

  if (strcmp(action, "SEND") == 0) {
    _forceSendRequested = true;
  } else if (strcmp(action, "SEND_GPS") == 0 || strcmp(action, "LOC") == 0) {
    // Get fresh GPS fix and send location via SMS
    GL868_ESP32_LOG_I("Getting fresh GPS fix for location request...");

    // Power on GPS and get a fresh fix (blocking, up to 90s)
    GPSData freshGPS;
    bool gotFix = false;

    if (gps.powerOn()) {
      gotFix = gps.getFix(&freshGPS, 90000);
      gps.powerOff();
    }

    if (gotFix && freshGPS.valid) {
      _currentGPS = freshGPS; // Update stored GPS data
      char loc[160];
      snprintf(loc, sizeof(loc),
               "Location: https://www.google.com/maps/@%.7f,%.7f,16z"
               "\nTime: %s\nSats: %d\nBatt: %d%%",
               freshGPS.latitude, freshGPS.longitude, freshGPS.timestamp,
               freshGPS.satellites, battery.readPercentage());
      sms.send(args, loc);
      GL868_ESP32_LOG_I("Location SMS sent to %s", args);
    } else {
      sms.send(args, "GPS fix failed. Try again later.");
      GL868_ESP32_LOG_E("GPS fix failed for location request");
    }
  } else if (strcmp(action, "SLEEP") == 0) {
    stateMachine.requestSleep();
  } else if (strcmp(action, "INTERVAL") == 0) {
    int minutes = atoi(args);
    if (minutes > 0) {
      setSendInterval(minutes * 60);
    }
  } else if (strcmp(action, "STATUS") == 0) {
    // Send status SMS
    char status[160];
    snprintf(status, sizeof(status),
             "State: %s\nBatt: %d%%\nSignal: %d\nQueue: %d",
             GL868_ESP32_StateToString(stateMachine.getState()),
             battery.readPercentage(), gsm.getSignalStrength(), queue.count());
    sms.send(args, status);
  }
}

// ============================================================================
// Configuration Methods
// ============================================================================

void GL868_ESP32::setSendInterval(uint32_t seconds) {
  sleep.setSendInterval(seconds);
  GL868_ESP32_LOG_I("Send interval set to %lu seconds", seconds);
}

void GL868_ESP32::setGPSTimeout(uint32_t seconds) {
  _gpsTimeout = seconds * 1000;
  GL868_ESP32_LOG_D("GPS timeout set to %lu seconds", seconds);
}

void GL868_ESP32::setErrorRebootTimeout(uint32_t seconds) {
  _errorRebootTimeout = seconds * 1000;
  GL868_ESP32_LOG_D("Error reboot timeout set to %lu seconds", seconds);
}

void GL868_ESP32::setTimeOffset(int8_t hours, int8_t minutes) {
  gps.setTimeOffset(hours, minutes);
}

void GL868_ESP32::setBatteryRange(uint16_t minMV, uint16_t maxMV) {
  battery.setVoltageRange(minMV, maxMV);
}

void GL868_ESP32::setBatterySource(BatterySource source) {
  battery.setSource(source);
  GL868_ESP32_LOG_I("Battery source set to %s",
                    source == BATTERY_SOURCE_ADC ? "ADC" : "MODEM");
}

void GL868_ESP32::setLEDPower(bool enabled) {
  led.setPowerEnabled(enabled);
  GL868_ESP32_LOG_D("WS2812B power %s", enabled ? "enabled" : "disabled");
}

void GL868_ESP32::setLEDBrightness(uint8_t percentage) {
  led.setBrightness(percentage);
  GL868_ESP32_LOG_I("WS2812B brightness set to %u%%", percentage);
}

// ============================================================================
// Operating Mode Configuration
// ============================================================================

void GL868_ESP32::setOperatingMode(OperatingMode mode) {
  _operatingMode = static_cast<uint8_t>(mode);
  GL868_ESP32_LOG_I("Operating mode set: DATA=%d SMS=%d CALL=%d",
                    (_operatingMode & MODE_DATA) ? 1 : 0,
                    (_operatingMode & MODE_SMS) ? 1 : 0,
                    (_operatingMode & MODE_CALL) ? 1 : 0);
}

void GL868_ESP32::setOperatingMode(uint8_t modeFlags) {
  _operatingMode = modeFlags & 0x07; // Only keep valid bits
  GL868_ESP32_LOG_I("Operating mode set: DATA=%d SMS=%d CALL=%d",
                    (_operatingMode & MODE_DATA) ? 1 : 0,
                    (_operatingMode & MODE_SMS) ? 1 : 0,
                    (_operatingMode & MODE_CALL) ? 1 : 0);
}

uint8_t GL868_ESP32::getOperatingMode() const { return _operatingMode; }

bool GL868_ESP32::isModeEnabled(OperatingModeFlags flag) const {
  return (_operatingMode & static_cast<uint8_t>(flag)) != 0;
}

// ============================================================================
// Direct AT Command Access
// ============================================================================

bool GL868_ESP32::sendATCommand(const char *cmd, uint32_t timeout) {
  if (!_initialized) {
    GL868_ESP32_LOG_E("Cannot send AT command - not initialized");
    return false;
  }

  // Check if command starts with "AT" or needs prefix added
  if (strncmp(cmd, "AT", 2) == 0) {
    return modem.sendCommand(cmd, timeout);
  } else {
    return modem.sendAT(cmd, timeout);
  }
}

bool GL868_ESP32::sendATCommand(const char *cmd, char *response, size_t maxLen,
                                uint32_t timeout) {
  if (!_initialized) {
    GL868_ESP32_LOG_E("Cannot send AT command - not initialized");
    return false;
  }

  // Check if command starts with "AT" or needs prefix added
  bool result;
  if (strncmp(cmd, "AT", 2) == 0) {
    result = modem.sendCommand(cmd, timeout);
  } else {
    result = modem.sendAT(cmd, timeout);
  }

  if (response != nullptr && maxLen > 0) {
    String resp = modem.getResponse();
    strncpy(response, resp.c_str(), maxLen - 1);
    response[maxLen - 1] = '\0';
  }

  return result;
}

String GL868_ESP32::getATResponse() { return modem.getResponse(); }

HardwareSerial &GL868_ESP32::getModemSerial() { return modem.getSerial(); }

// ============================================================================
// APN Configuration
// ============================================================================

void GL868_ESP32::setAPN(const char *apn) { gsm.setAPN(apn); }

void GL868_ESP32::setAPN(const char *apn, const char *user, const char *pass) {
  gsm.setAPN(apn, user, pass);
}

const char *GL868_ESP32::getAPN() const { return gsm.getAPN(); }

// ============================================================================
// On-Demand GPS Access
// ============================================================================

bool GL868_ESP32::gpsOn() {
  if (!_initialized) {
    GL868_ESP32_LOG_E("Cannot power on GPS - not initialized");
    return false;
  }
  GL868_ESP32_LOG_I("Manually powering on GPS");
  return gps.powerOn();
}

bool GL868_ESP32::gpsOff() {
  if (!_initialized) {
    GL868_ESP32_LOG_E("Cannot power off GPS - not initialized");
    return false;
  }
  GL868_ESP32_LOG_I("Manually powering off GPS");
  return gps.powerOff();
}

bool GL868_ESP32::isGpsPowered() { return gps.isPowered(); }

bool GL868_ESP32::getLocation(GPSData *data, uint32_t timeout) {
  if (!_initialized) {
    GL868_ESP32_LOG_E("Cannot get location - not initialized");
    return false;
  }

  // Power on GPS if not already on
  bool wasOff = !gps.isPowered();
  if (wasOff) {
    GL868_ESP32_LOG_I("GPS not powered - turning on for location request");
    if (!gps.powerOn()) {
      GL868_ESP32_LOG_E("Failed to power on GPS");
      return false;
    }
  }

  // Get fix
  bool result = gps.getFix(data, timeout);

  // Power off GPS if we turned it on
  if (wasOff) {
    GL868_ESP32_LOG_D("Powering off GPS after location request");
    gps.powerOff();
  }

  return result;
}

bool GL868_ESP32::getLocationNow(GPSData *data) {
  if (!_initialized) {
    GL868_ESP32_LOG_E("Cannot get location - not initialized");
    return false;
  }

  if (!gps.isPowered()) {
    GL868_ESP32_LOG_W(
        "GPS not powered - call gpsOn() first for non-blocking reads");
    return false;
  }

  return gps.getReading(data);
}

// ============================================================================
// User-Callable HTTP Functions
// ============================================================================

int GL868_ESP32::httpPost(const char *url, const char *body,
                          const char *headers) {
  if (!_initialized) {
    GL868_ESP32_LOG_E("Cannot send HTTP - not initialized");
    return -1;
  }

  GL868_ESP32_LOG_I("Sending HTTP POST to %s", url);

  // Ensure network is registered
  if (!gsm.isRegistered()) {
    if (!gsm.waitNetworkRegistration(30000)) {
      GL868_ESP32_LOG_E("Network registration failed");
      return -1;
    }
  }

  // Ensure GPRS is attached
  if (!gsm.isGPRSAttached()) {
    if (!gsm.attachGPRS()) {
      GL868_ESP32_LOG_E("GPRS attach failed");
      return -1;
    }
  }

  // Send the HTTP POST
  return gsm.httpPOSTRaw(url, body, headers);
}

int GL868_ESP32::httpPost(const char *url, const char *body,
                          const char *headers, char *response,
                          size_t responseMaxLen) {
  if (!_initialized) {
    GL868_ESP32_LOG_E("Cannot send HTTP - not initialized");
    return -1;
  }

  GL868_ESP32_LOG_I("Sending HTTP POST to %s (with response capture)", url);

  // Ensure network is registered
  if (!gsm.isRegistered()) {
    if (!gsm.waitNetworkRegistration(30000)) {
      GL868_ESP32_LOG_E("Network registration failed");
      return -1;
    }
  }

  // Ensure GPRS is attached
  if (!gsm.isGPRSAttached()) {
    if (!gsm.attachGPRS()) {
      GL868_ESP32_LOG_E("GPRS attach failed");
      return -1;
    }
  }

  // Send the HTTP POST with response capture
  return gsm.httpPOSTRaw(url, body, headers, response, responseMaxLen);
}

// ============================================================================
// User-Callable SMS Functions
// ============================================================================

bool GL868_ESP32::sendSMS(const char *number, const char *message) {
  if (!_initialized) {
    GL868_ESP32_LOG_E("Cannot send SMS - not initialized");
    return false;
  }

  GL868_ESP32_LOG_I("Sending SMS to %s", number);
  return sms.send(number, message);
}

// ============================================================================
// User-Callable Call Functions
// ============================================================================

bool GL868_ESP32::makeCall(const char *number, uint8_t timeout) {
  if (!_initialized) {
    GL868_ESP32_LOG_E("Cannot make call - not initialized");
    return false;
  }

  GL868_ESP32_LOG_I("Making call to %s", number);
  return call.makeCall(number, timeout);
}

bool GL868_ESP32::answerCall() {
  if (!_initialized) {
    GL868_ESP32_LOG_E("Cannot answer call - not initialized");
    return false;
  }

  return call.answer();
}

void GL868_ESP32::hangupCall() {
  if (!_initialized) {
    GL868_ESP32_LOG_E("Cannot hangup call - not initialized");
    return;
  }

  call.hangup();
}

bool GL868_ESP32::isCallActive() {
  if (!_initialized) {
    return false;
  }

  return call.isCallActive();
}

void GL868_ESP32::setPayloads(std::initializer_list<PayloadField> payloads) {
  // Clear existing payloads
  _payloadCount = 0;

  // If empty list, just clear
  if (payloads.size() == 0) {
    json.setPayloadFields(nullptr, 0);
    GL868_ESP32_LOG_D("Payloads cleared");
    return;
  }

  // Copy new payloads
  for (const auto &field : payloads) {
    if (_payloadCount >= MAX_PAYLOAD_KEYS)
      break;
    _payloadFields[_payloadCount] = field;
    _payloadCount++;
  }

  json.setPayloadFields(_payloadFields, _payloadCount);
  GL868_ESP32_LOG_D("Set %d payload fields", _payloadCount);
}

void GL868_ESP32::setPayloadFields(PayloadField *fields, uint8_t count) {
  _payloadCount = min(count, (uint8_t)MAX_PAYLOAD_KEYS);
  for (uint8_t i = 0; i < _payloadCount; i++) {
    _payloadFields[i] = fields[i];
  }
  json.setPayloadFields(_payloadFields, _payloadCount);
}

void GL868_ESP32::clearPayloads() {
  _payloadCount = 0;
  json.setPayloadFields(nullptr, 0);
  GL868_ESP32_LOG_D("Payloads cleared");
}

void GL868_ESP32::enableSMS(bool enabled) { sms.enable(enabled); }

void GL868_ESP32::enableCalls(bool enabled) { call.enable(enabled); }

void GL868_ESP32::enableSMSMonitoring(bool enabled) {
  sleep.enableSMSMonitoring(enabled);
}

void GL868_ESP32::enableCallMonitoring(bool enabled) {
  sleep.enableCallMonitoring(enabled);
}

void GL868_ESP32::registerSMSHandler(const char *command, SMSHandler handler) {
  sms.registerHandler(command, handler);
}

void GL868_ESP32::registerCallHandler(CallHandler handler) {
  call.registerHandler(handler);
}

void GL868_ESP32::setCallAction(CallAction action) {
  call.setDefaultAction(action);
}

void GL868_ESP32::enableMotionTrigger(bool enabled) {
  _motionTriggerEnabled = enabled;
  if (enabled) {
    motion.enableInterrupt();
  } else {
    motion.disableInterrupt();
  }
}

void GL868_ESP32::setMotionSensitivity(uint8_t threshold) {
  motion.setSensitivity(threshold);
}

void GL868_ESP32::setMotionThreshold(float mg) { motion.setThresholdMg(mg); }

void GL868_ESP32::enableFullPowerOff(bool enabled) {
  _fullPowerOffEnabled = enabled;
}

void GL868_ESP32::setSIM868SleepMode(SIM868SleepMode mode) {
  sleep.setSIM868SleepMode(mode);
}

void GL868_ESP32::enableMotionWake(bool enabled) {
  sleep.enableMotionWake(enabled);
}

void GL868_ESP32::setRIPin(uint8_t pin) { sleep.setRIPin(pin); }

void GL868_ESP32::enableHeartbeat(bool enabled) {
  sleep.enableHeartbeat(enabled);
}

void GL868_ESP32::setHeartbeatInterval(uint32_t seconds) {
  sleep.setHeartbeatInterval(seconds);
}

void GL868_ESP32::setQueueStorage(QueueStorage type, uint8_t maxEntries) {
  queue.end();
  queue.begin(type, maxEntries);
}

void GL868_ESP32::setLogLevel(LogLevel level) {
  GL868_ESP32_SetLogLevel(level);
}

// ============================================================================
// Status Methods
// ============================================================================

SystemState GL868_ESP32::getState() const { return stateMachine.getState(); }

WakeSource GL868_ESP32::getWakeSource() const { return sleep.getWakeSource(); }

bool GL868_ESP32::isScheduledWake() const { return sleep.isScheduledWake(); }

bool GL868_ESP32::isMotionWake() const { return sleep.isMotionWake(); }

uint8_t GL868_ESP32::getBatteryPercent() { return battery.readPercentage(); }

uint16_t GL868_ESP32::getBatteryVoltageMV() { return battery.readVoltageMV(); }

int GL868_ESP32::getSignalStrength() { return gsm.getSignalStrength(); }

uint8_t GL868_ESP32::getSatelliteCount() const {
  return _currentGPS.satellites;
}

String GL868_ESP32::getOperator() { return gsm.getOperator(); }

String GL868_ESP32::getIMEI() { return gsm.getIMEI(); }

String GL868_ESP32::getIMSI() { return gsm.getIMSI(); }

String GL868_ESP32::getICCID() { return gsm.getICCID(); }

String GL868_ESP32::getPhoneNumber() { return gsm.getPhoneNumber(); }

// ============================================================================
// Manual Control
// ============================================================================

void GL868_ESP32::forceSend() {
  _forceSendRequested = true;
  GL868_ESP32_LOG_I("Force send requested");
}

void GL868_ESP32::forceSleep() {
  stateMachine.requestSleep();
  GL868_ESP32_LOG_I("Force sleep requested");
}

// ============================================================================
// Callbacks
// ============================================================================

void GL868_ESP32::onStateChange(StateChangeCallback callback) {
  stateMachine.onStateChange(callback);
}

void GL868_ESP32::onError(ErrorCallback callback) {
  stateMachine.onError(callback);
}

void GL868_ESP32::onDataSent(DataSentCallback callback) {
  stateMachine.onDataSent(callback);
}
