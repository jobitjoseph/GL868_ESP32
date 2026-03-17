/*
 * FactoryFirmware.ino
 * GL868_ESP32 Library - Factory Firmware Example
 *
 * This example demonstrates the full factory setup with SMS/Call location:
 * 1. Power on modem and verify AT responsiveness
 * 2. Display SIM info (IMEI, ICCID, IMSI, MSISDN) and activation URL
 * 3. Periodically check for network registration (SIM activation)
 * 4. Set activation flag ONLY when network actually registers
 * 5. Get API key from serial monitor (or hardcode below) and save to NVS
 * 6. Send data to GeoLinker cloud
 *
 * SMS/Call Location Features:
 * - Incoming call: hang up, get GPS fix, reply with location via SMS
 * - SMS "LOC": get GPS fix, reply with location via SMS
 * - ESP32 enters deep sleep between cycles
 * - SIM868 stays at full power to receive calls/SMS
 * - SIM868 RI pin wakes ESP32 from deep sleep
 * - SMS inbox/sent items cleaned up to keep SIM868 memory free
 *
 * Note: The NVS flag alone is not trusted - network registration is
 * verified on every boot to ensure SIM is actually active.
 */

#include <GL868_ESP32.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ============================================================================
// Configuration
// ============================================================================
#define DEVICE_ID "FACTORY_DEVICE"
#define ACTIVATION_URL "https://circuitdigest.cloud/activation"

// If you want to hardcode your API key, enter it here.
// Otherwise leave it empty ("") to prompt via Serial Monitor.
#define API_KEY ""

// NVS namespace and keys
#define NVS_NAMESPACE "gl868_factory"
#define NVS_KEY_ACTIVATED "sim_activated"
#define NVS_KEY_API_KEY "api_key"

// Timing
#define REGISTRATION_CHECK_INTERVAL 10000  // Check every 10 seconds
#define INITIAL_REGISTRATION_TIMEOUT 60000 // 60 seconds for first check
#define MODEM_POWERON_TIMEOUT 60000        // 60 seconds for modem power on
#define REBOOT_TIMEOUT 180000              // Reboot if not registered in 3 min

Preferences prefs;
char apiKey[64] = API_KEY;
bool isActivated = false;

void setup() {
  Serial.begin(115200);
  delay(2000);

  // -------------------------------------------------------------------------
  // Detect wake source FIRST — skip verbose factory steps on sleep wake
  // -------------------------------------------------------------------------
  esp_sleep_wakeup_cause_t wakeReason = esp_sleep_get_wakeup_cause();
  bool isSleepWake = (wakeReason == ESP_SLEEP_WAKEUP_TIMER ||
                      wakeReason == ESP_SLEEP_WAKEUP_EXT0 ||
                      wakeReason == ESP_SLEEP_WAKEUP_EXT1);

  Serial.println("\n========================================");
  Serial.println("   GL868_ESP32 Factory Firmware");
  Serial.println("   SMS/Call Location + Deep Sleep");
  Serial.println("========================================\n");

  // Initialize NVS
  prefs.begin(NVS_NAMESPACE, false);

  // Load saved API key if not hardcoded
  if (strlen(apiKey) == 0) {
    String savedKey = prefs.getString(NVS_KEY_API_KEY, "");
    if (savedKey.length() > 0) {
      savedKey.toCharArray(apiKey, sizeof(apiKey));
      Serial.println("API key loaded from NVS");
    }
  } else {
    Serial.println("Using hardcoded API key");
  }

  // Load activation flag
  isActivated = prefs.getBool(NVS_KEY_ACTIVATED, false);

  // Initialize the library (inits UART, LED, battery, etc.)
  GeoLinker.begin(DEVICE_ID, apiKey);
  GeoLinker.setLEDBrightness(20);

  // --------------------------------------------------------------------------
  // Operating Mode: DATA + SMS + CALL (full mode)
  // --------------------------------------------------------------------------
  GeoLinker.setOperatingMode(MODE_FULL);

  // Enable SMS and Call processing
  GeoLinker.enableSMS(true);
  GeoLinker.enableCalls(true);

  // Set call action: incoming call → hang up + get GPS → reply SMS
  GeoLinker.setCallAction(CALL_SEND_GPS);

  // --------------------------------------------------------------------------
  // Sleep Configuration: ESP32 sleeps, SIM868 stays awake
  // --------------------------------------------------------------------------
  // Enable SMS/Call monitoring during sleep — keeps SIM868 at full power
  GeoLinker.enableSMSMonitoring(true);
  GeoLinker.enableCallMonitoring(true);

  // Set timezone offset for timestamps (IST = +5:30)
  GeoLinker.setTimeOffset(5, 30);

  GeoLinker.setLogLevel(LOG_INFO);

  // Enable motion-triggered wake from deep sleep
  GeoLinker.sleep.enableMotionWake(true);

  // Configure motion sensitivity
  // Adjust based on your use case:
  // - Asset in transit: 300-500mg (detect handling)
  // - Parked vehicle: 200-400mg (detect towing/theft)
  // - Equipment: 400-800mg (detect usage)
  GeoLinker.setMotionThreshold(50.0); // Very high sensitivity: 50mg

  // -------------------------------------------------------------------------
  // If waking from deep sleep (timer or motion or RI), skip factory setup
  // -------------------------------------------------------------------------
  if (isSleepWake) {
    Serial.print("Wake from sleep (reason: ");
    Serial.print(wakeReason == ESP_SLEEP_WAKEUP_TIMER  ? "TIMER"
                 : wakeReason == ESP_SLEEP_WAKEUP_EXT0 ? "RI_PIN (Call/SMS)"
                 : wakeReason == ESP_SLEEP_WAKEUP_EXT1 ? "MOTION"
                                                       : "UNKNOWN");
    Serial.println(") — skipping factory setup.");

    // --- STEP 1: Power on modem (may already be on if SIM868 stayed active)
    GeoLinker.led.setState(LED_BOOT);
    if (!powerOnModem()) {
      Serial.println("\n*** FATAL: Could not power on modem ***");
      GeoLinker.led.setError(ERROR_MODEM);
      while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
    }
    GeoLinker.led.setState(LED_IDLE);

    // If woken by RI pin (call/SMS), the library will handle it via update()
    if (wakeReason == ESP_SLEEP_WAKEUP_EXT0) {
      Serial.println("=== Woken by Call/SMS ===");
      Serial.println("Location SMS will be sent to caller/sender.");
      Serial.println("(GeoLinker upload skipped for Call/SMS wake)");
    } else {
      Serial.println("=== Device Ready (sleep wake) ===");
      Serial.println("Starting GPS tracking...\n");
    }
    return;
  }

  // -------------------------------------------------------------------------
  // Full factory setup (first boot / manual reset)
  // -------------------------------------------------------------------------

  // --- STEP 1: Power on modem ---
  GeoLinker.led.setState(LED_BOOT);
  Serial.println("=== Step 1: Modem Power On ===");
  if (!powerOnModem()) {
    Serial.println("\n*** FATAL: Could not power on modem ***");
    Serial.println("Check hardware connections and power supply.");
    GeoLinker.led.setError(ERROR_MODEM);
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
  Serial.println("Modem ready!\n");

  // Short delay to let modem stabilize (only matters if modem was just powered
  // on)
  delay(1000);
  GeoLinker.modem.flushSerial();

  // --- STEP 2: Display SIM info ---
  GeoLinker.led.setState(LED_GSM_INIT);
  displaySIMInfo();

  // --- STEP 3: Check SIM and network registration ---
  GeoLinker.led.setState(LED_GSM_REGISTER);

  // First ensure SIM is ready
  Serial.println("=== Step 3: SIM & Network Registration ===");
  Serial.print("Checking SIM status... ");
  if (GeoLinker.gsm.checkSIMReady()) {
    Serial.println("SIM Ready!");
  } else {
    Serial.println("SIM not ready! Check SIM card.");
    GeoLinker.led.setError(ERROR_NO_SIM);
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  // Enable network registration status reporting
  GeoLinker.modem.sendAT("+CREG=1", 2000);
  delay(500);
  GeoLinker.modem.flushSerial();

  if (!verifyNetworkRegistration()) {
    // Not registered - reset flag and wait for activation
    if (isActivated) {
      Serial.println("Previously activated but not registering now.");
      Serial.println("SIM may have been deactivated or changed.");
    }
    isActivated = false;
    prefs.putBool(NVS_KEY_ACTIVATED, false);

    // Wait for activation with periodic checks
    waitForActivation();
  } else {
    Serial.println("\n*** Network Registered! SIM is active ***");
    if (!isActivated) {
      // First time activation
      prefs.putBool(NVS_KEY_ACTIVATED, true);
      isActivated = true;
    }
    printOperatorInfo();
  }

  // --- STEP 4: API Key setup ---
  if (strlen(apiKey) == 0) {
    promptForApiKey();
  }

  // All done - set LED to idle
  GeoLinker.led.setState(LED_IDLE);

  Serial.println("\n=== Device Ready ===");
  Serial.println("SMS/Call location enabled.");
  Serial.println("  - Call this device -> get location via SMS");
  Serial.println("  - Send 'LOC' SMS  -> get location via SMS");
  Serial.println("  - Send 'STATUS'   -> get device status via SMS");
  Serial.println("Starting GPS tracking...\n");
}

void loop() { GeoLinker.update(); }

// ============================================================================
// Power On Modem (check first, then power on if needed)
// ============================================================================
bool powerOnModem() {
  // First check if modem is already ON and responding
  Serial.print("Checking if modem is already on... ");
  if (GeoLinker.modem.isResponsive()) {
    Serial.println("YES - modem already running!");
    // Disable echo just in case
    GeoLinker.modem.sendAT("E0");
    GeoLinker.modem.flushSerial();
    return true;
  }
  Serial.println("NO");

  // Modem is off - power it on
  Serial.print("Powering on modem");
  if (GeoLinker.modem.powerOn()) {
    Serial.println(" OK");
    return true;
  }

  // If powerOn() failed, wait up to 60 seconds checking periodically
  Serial.println();
  Serial.println("Modem not responding, waiting up to 60s...");

  uint32_t startTime = millis();
  while (millis() - startTime < MODEM_POWERON_TIMEOUT) {
    delay(2000);
    Serial.print(".");

    if (GeoLinker.modem.isResponsive()) {
      Serial.println(" Modem responded!");
      GeoLinker.modem.sendAT("E0");
      GeoLinker.modem.flushSerial();
      return true;
    }
  }

  // Attempt 2: Try powerOn() one more time (pulses PWRKEY again)
  Serial.println("\nRetrying modem power on...");
  if (GeoLinker.modem.powerOn()) {
    Serial.println("Modem powered on on retry!");
    return true;
  }

  return false;
}

// ============================================================================
// Display SIM Information
// ============================================================================
void displaySIMInfo() {
  Serial.println("=== SIM Card Information ===\n");

  String imei = GeoLinker.getIMEI();
  String iccid = GeoLinker.getICCID();
  String imsi = GeoLinker.getIMSI();
  String msisdn = GeoLinker.getPhoneNumber();

  Serial.print("IMEI:   ");
  Serial.println(imei.length() > 0 ? imei : "Not available");
  Serial.print("ICCID:  ");
  Serial.println(iccid.length() > 0 ? iccid : "Not available");
  Serial.print("IMSI:   ");
  Serial.println(imsi.length() > 0 ? imsi : "Not available");
  Serial.print("MSISDN: ");
  Serial.println(msisdn.length() > 0 ? msisdn : "Not available");

  Serial.println("\n=== Activation & KYC Registration ===");
  Serial.print("Please visit: ");
  Serial.println(ACTIVATION_URL);
  Serial.println();
}

// ============================================================================
// Raw AT+CREG? check — sends command and parses response directly
// Returns: registration status (0-5), or -1 on error
// ============================================================================
int rawCheckCREG() {
  GeoLinker.modem.flushSerial();
  Serial2.println("AT+CREG?");

  uint32_t start = millis();
  while (millis() - start < 3000) {
    if (Serial2.available()) {
      String line = Serial2.readStringUntil('\n');
      line.trim();

      // Skip empty lines and echo
      if (line.length() == 0 || line == "AT+CREG?")
        continue;

      // Print raw response for debugging
      Serial.print("  [CREG raw] '");
      Serial.print(line);
      Serial.println("'");

      // Parse +CREG: n,stat  or  +CREG: stat
      if (line.startsWith("+CREG:")) {
        int commaPos = line.indexOf(',');
        if (commaPos > 0) {
          // Format: +CREG: n,stat[,lac,ci]
          int stat = line.substring(commaPos + 1, commaPos + 2).toInt();
          return stat;
        } else {
          // Format: +CREG: stat (no comma)
          int stat = line.substring(7).toInt();
          return stat;
        }
      }
    }
    yield();
  }
  return -1; // timeout
}

// ============================================================================
// Verify Network Registration (up to 60s, with raw debug output)
// ============================================================================
bool verifyNetworkRegistration() {
  Serial.println("Checking network registration (60s timeout)...");

  uint32_t startTime = millis();
  int checkNum = 0;

  while (millis() - startTime < INITIAL_REGISTRATION_TIMEOUT) {
    checkNum++;
    Serial.print("[");
    Serial.print(checkNum);
    Serial.print("] AT+CREG? -> ");

    int stat = rawCheckCREG();

    if (stat == 1) {
      Serial.println("Registered (Home)!");
      return true;
    } else if (stat == 5) {
      Serial.println("Registered (Roaming)!");
      return true;
    } else if (stat == 0) {
      Serial.println("Not registered, not searching");
    } else if (stat == 2) {
      Serial.println("Not registered, searching...");
    } else if (stat == 3) {
      Serial.println("Registration denied");
    } else if (stat == 4) {
      Serial.println("Unknown");
    } else {
      Serial.println("No response / parse error");
    }

    // If status is 0 (not searching), 3 (denied), or 4 (unknown), exit early
    if (stat == 0 || stat == 3 || stat == 4) {
      Serial.print("Fatal registration status: ");
      Serial.println(stat);
      return false;
    }

    // Wait 5 seconds between checks
    delay(5000);
  }

  Serial.println("Registration TIMEOUT after 60s");
  return false;
}

// ============================================================================
// Wait for SIM Activation (periodic check with raw debug)
// ============================================================================
void waitForActivation() {
  Serial.println("\n=== Waiting for SIM Activation ===");
  Serial.println("Please complete activation at the URL above.");
  Serial.print("Checking every ");
  Serial.print(REGISTRATION_CHECK_INTERVAL / 1000);
  Serial.println(" seconds...\n");

  uint32_t startTime = millis();
  uint32_t lastCheck = 0;
  int checkCount = 0;

  while (true) {
    // Periodic registration check
    if (millis() - lastCheck >= REGISTRATION_CHECK_INTERVAL) {
      lastCheck = millis();
      checkCount++;

      Serial.print("[");
      Serial.print(checkCount);
      Serial.print("] AT+CREG? -> ");

      int stat = rawCheckCREG();

      if (stat == 1 || stat == 5) {
        Serial.println(stat == 1 ? "Registered (Home)!"
                                 : "Registered (Roaming)!");
        Serial.println("\n*** Network Registered! SIM is active ***");

        // Update library state
        GeoLinker.gsm.waitNetworkRegistration(5000);

        // Set activation flag
        prefs.putBool(NVS_KEY_ACTIVATED, true);
        isActivated = true;

        GeoLinker.led.setState(LED_IDLE);
        printOperatorInfo();
        return;
      }

      Serial.print("Status: ");
      Serial.println(stat);

      // If status is 0 (not searching), 3 (denied), or 4 (unknown),
      // block here and show the error.
      if (stat == 0 || stat == 3 || stat == 4) {
        Serial.println("Fatal registration status - stopping.");
        GeoLinker.led.setError(ERROR_NO_NETWORK);
        while (1) {
          vTaskDelay(pdMS_TO_TICKS(1000));
        }
      }
    }

    // Reboot if not registered after timeout
    if (millis() - startTime > REBOOT_TIMEOUT) {
      Serial.println("\n*** Registration timeout ***");
      Serial.println("Rebooting to retry...");
      delay(2000);
      ESP.restart();
    }

    delay(100);
  }
}

// ============================================================================
// Print Operator Information
// ============================================================================
void printOperatorInfo() {
  String op = GeoLinker.getOperator();
  int signal = GeoLinker.getSignalStrength();

  Serial.print("Operator: ");
  Serial.println(op.length() > 0 ? op : "Unknown");
  Serial.print("Signal: ");
  Serial.print(signal);
  Serial.println("/31");
}

// ============================================================================
// API Key Setup
// ============================================================================
void promptForApiKey() {
  Serial.println("\n=== API Key Required ===");
  Serial.println("Enter your GeoLinker API key:");
  Serial.println("(Get your key from https://circuitdigest.cloud)");
  Serial.print("> ");

  // Set LED to green blinking state while waiting
  GeoLinker.led.setState(LED_WAIT_API_KEY);

  // Wait for serial input
  while (!Serial.available()) {
    GeoLinker.update(); // Keep background systems like LED running
    delay(50);
  }

  // Read API key
  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input.length() > 0 && input.length() < sizeof(apiKey)) {
    input.toCharArray(apiKey, sizeof(apiKey));

    // Save to NVS
    prefs.putString(NVS_KEY_API_KEY, input);

    Serial.println("\nAPI key saved!");
    Serial.print("Key: ");
    Serial.println(apiKey);

    // Reinitialize with new key
    GeoLinker.begin(DEVICE_ID, apiKey);
  } else {
    Serial.println("Invalid key. Please reboot and try again.");
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}
