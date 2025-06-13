#include <Arduino.h>
#include <AccelStepper.h>
#include <NeoPixelBus.h>
#include <WiFi.h>
A
// WS2812B LED Rings
#define LED_PIN_1 10
#define LED_PIN_2 18
#define LED_PIN_3 19
#define NUMPIXELS 48

// NeoPixelBus Objects
NeoPixelBus<NeoGrbFeature, NeoEsp32BitBangWs2812xMethod> strip1(NUMPIXELS, LED_PIN_1);
NeoPixelBus<NeoGrbFeature, NeoEsp32BitBangWs2812xMethod> strip2(NUMPIXELS, LED_PIN_2);
NeoPixelBus<NeoGrbFeature, NeoEsp32BitBangWs2812xMethod> strip3(NUMPIXELS, LED_PIN_3);

// Stepper motor control pins
#define ENA_PIN_1 9
#define STEP_PIN_1 8
#define DIR_PIN_1 7
#define ENA_PIN_2 6
#define STEP_PIN_2 5
#define DIR_PIN_2 4
#define ENA_PIN_3 1
#define STEP_PIN_3 2
#define DIR_PIN_3 3
AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);
AccelStepper stepper3(AccelStepper::DRIVER, STEP_PIN_3, DIR_PIN_3);

// WiFi credentials
const char* ssid = "TelstraD81AE7";
const char* password = "password";

WiFiServer server(80);

unsigned long previousMillis = 0;
const long interval = 50;  // Interval for LED update (in milliseconds)
int currentMode1 = 1;
int currentMode2 = 1;
int currentMode3 = 1;
int currentLedMode1 = 0;
int currentLedMode2 = 0;
int currentLedMode3 = 0;
unsigned long startTime1;
unsigned long startTime2;
unsigned long startTime3;
const float pi = 3.14159;   // Ya know, the circle thing
const float frequency = 0.05;     // Frequency of the sinusoidal motion (Hz)
const int microsteppingFactor = 16; // Microstepping factor

TaskHandle_t ledTaskHandle = NULL;
TaskHandle_t wifiTaskHandle = NULL;


void setup() {
  // Set up serial monitor
  Serial.begin(115200);

  // Initialize NeoPixel LED strips
  strip1.Begin();
  strip1.Show();
  strip2.Begin();
  strip2.Show();
  strip3.Begin();
  strip3.Show();
  Serial.println("NeoPixelBus initialized.");

  // Configure stepper motors
  stepper1.setMaxSpeed(1000 * microsteppingFactor);
  stepper1.setSpeed(500 * microsteppingFactor); // Set the continuous speed with microstepping factor
  stepper2.setMaxSpeed(1000 * microsteppingFactor);
  stepper2.setSpeed(500 * microsteppingFactor); // Set the continuous speed with microstepping factor
  stepper3.setMaxSpeed(1000 * microsteppingFactor);
  stepper3.setSpeed(500 * microsteppingFactor); // Set the continuous speed with microstepping factor

  // Configure WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();
  startTime1 = millis();
  startTime2 = millis();
  startTime3 = millis();

  // Create LED update task
  xTaskCreatePinnedToCore(
    ledTask,          // Function to implement the task
    "LED Task",      // Name of the task
    10000,            // Stack size in words
    NULL,             // Task input parameter
    1,                // Priority of the task
    &ledTaskHandle,   // Task handle
    0                 // Core where the task should run
  );

  // Create WiFi task
  xTaskCreatePinnedToCore(
    wifiTask,         // Function to implement the task
    "WiFi Task",     // Name of the task
    10000,            // Stack size in words
    NULL,             // Task input parameter
    1,                // Priority of the task
    &wifiTaskHandle,  // Task handle
    1                 // Core where the task should run
  );
}

void loop() {
  // Run the motors based on the current mode
  runMotor(stepper1, currentMode1, startTime1, 1.0);
  runMotor(stepper2, currentMode2, startTime2, 2.0);
  runMotor(stepper3, currentMode3, startTime3, 3.0);
}

void runMotor(AccelStepper &stepper, int mode, unsigned long &startTime, float motor) {
  if (mode == 1) { // Sinusoidal motion
    unsigned long elapsedTime = millis() - startTime;
    float speed = 50 * microsteppingFactor * sin(2 * pi * frequency * (elapsedTime / 1000.0) + (2 * pi / 3) * motor);
    stepper.setSpeed(speed);
    stepper.runSpeed();
  } else if (mode == 2) { // Continuous motion
    stepper.runSpeed();
  } else if (mode == 0) { // Stop the motor
    stepper.stop();
  }
}

void ledTask(void * parameter) {
  unsigned long previousMillis = 0;
  const long interval = 50;  // Interval for LED update (in milliseconds)
  int colorWheelPos1 = 0;
  int colorWheelPos2 = 0;
  int colorWheelPos3 = 0;
  bool flashState1 = false;
  bool flashState2 = false;
  bool flashState3 = false;

  for (;;) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;

      // Update LED Ring 1
      updateLedRing(strip1, currentLedMode1, colorWheelPos1, flashState1);

      // Update LED Ring 2
      updateLedRing(strip2, currentLedMode2, colorWheelPos2, flashState2);

      // Update LED Ring 3
      updateLedRing(strip3, currentLedMode3, colorWheelPos3, flashState3);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Yield to other tasks
  }
}

// void updateLedRing(Adafruit_NeoPixel &strip, int mode, int &colorWheelPos, bool &flashState) {
void updateLedRing(NeoPixelBus<NeoGrbFeature, NeoEsp32BitBangWs2812xMethod> &strip, int mode, int &colorWheelPos) {
  switch (mode) {
    case 0: // Rainbow Effect
      for (int i = 0; i < NUMPIXELS; i++) {
        strip.SetPixelColor(i, Wheel((colorWheelPos + (i * 256 / NUMPIXELS)) & 255));
      }
      strip.show();
      colorWheelPos = (colorWheelPos + 1) % 256;
      break;

    case 1: // Solid Color (Blue)
      strip.fill(strip.Color(0, 0, 255));
      strip.show();
      break;

    case 2: // Flashing (Red)
      flashState = !flashState;
      if (flashState) {
        strip.fill(strip.Color(255, 0, 0));
      } else {
        strip.fill(strip.Color(0, 0, 0));
      }
      strip.show();
      break;

    case 3: // Theater Chase Effect (White)
      for (int i = 0; i < NUMPIXELS; i++) {
        if ((i + colorWheelPos) % 3 == 0) {
          strip.setPixelColor(i, strip.Color(255, 255, 255));
        } else {
          strip.setPixelColor(i, strip.Color(0, 0, 0));
        }
      }
      strip.show();
      colorWheelPos = (colorWheelPos + 1) % 3;
      break;
    
    case 4: // Double Sinusoidal Brightness with Color Lerp Rotation
      {
        uint32_t color = Wheel(colorWheelPos & 255); // Color based on the wheel position
        float phaseOffset = (2 * pi * colorWheelPos) / 256.0; // Smooth phase offset to avoid jumps
        for (int i = 0; i < NUMPIXELS; i++) {
          float brightness = (sin((4 * pi * i / NUMPIXELS) + phaseOffset) + 1) / 2; // Double sinusoidal brightness from 0 to 1
          uint8_t r = (uint8_t)(((color >> 16) & 0xFF) * brightness);
          uint8_t g = (uint8_t)(((color >> 8) & 0xFF) * brightness);
          uint8_t b = (uint8_t)((color & 0xFF) * brightness);
          strip.setPixelColor(i, strip.Color(r, g, b));
        }
        strip.show();
        colorWheelPos = (colorWheelPos + 3) % 256; // Increase speed of color rotation
      }

    case 5: // Perceptually Adjusted Sinusoidal Distribution
      {
          static float time = 0; // Time variable for controlling the effect
          const float speed = 0.02; // Speed of the effect
          const float maxPhase = 6 * pi; // Total phase for the pattern (multiple revolutions)

          // Custom easing function for wind-up and wind-down
          float normalizedTime = fmod(time, 1.0); // Normalize time to range [0, 1]
          float easingFactor = pow(sin(normalizedTime * pi), 2); // Smooth transition with sine

          // Calculate current phase with easing
          float phase = easingFactor * maxPhase;

          time += speed;

          uint32_t color = Wheel(colorWheelPos & 255); // Color based on wheel position
          for (int i = 0; i < NUMPIXELS; i++) {
              // Adjust sinusoidal brightness with gamma correction
              float rawBrightness = (sin((4 * pi * i / NUMPIXELS) + phase) + 1) / 2; // Original brightness
              float gammaCorrectedBrightness = pow(rawBrightness, 2.2); // Apply gamma correction (2.2 typical)

              // Map the corrected brightness to RGB values
              uint8_t r = (uint8_t)(((color >> 16) & 0xFF) * gammaCorrectedBrightness);
              uint8_t g = (uint8_t)(((color >> 8) & 0xFF) * gammaCorrectedBrightness);
              uint8_t b = (uint8_t)((color & 0xFF) * gammaCorrectedBrightness);

              // Set the pixel color
              strip.setPixelColor(i, strip.Color(r, g, b));
          }
          strip.show();

          // Update colorWheelPos to control rotation speed
          colorWheelPos = (colorWheelPos + 3) % 256;
      }

    break;
  }
}

void wifiTask(void *parameter) {
    for (;;) {
        WiFiClient client = server.available();
        if (client) {
            String request = client.readStringUntil('\r');
            //Serial.println("Client connected");
            //Serial.println("Request: " + request);
            // Serial.println(request);

            client.flush();

            // Add CORS headers to allow cross-origin requests
            client.println("HTTP/1.1 200 OK");
            client.println("Access-Control-Allow-Origin: *"); // Allow all origins
            client.println("Access-Control-Allow-Methods: GET, POST, OPTIONS"); // Allowed methods
            client.println("Access-Control-Allow-Headers: Content-Type"); // Allowed headers
            client.println("Content-type:text/plain");
            client.println();

            // Process the request
            handleRequest(request, client);

            // Close the connection
            client.stop();
            //Serial.println("Client disconnected");
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Yield to other tasks
    }
}

void handleRequest(String request, WiFiClient &client) {
    // Trim whitespace and extract only the actual command portion
    request.trim();
    int startIndex = request.indexOf(' ') + 1; // Find the first space and move one character forward
    int endIndex = request.indexOf(' ', startIndex); // Find the next space after the command
    if (startIndex < 0 || endIndex < 0) {
        client.println("Invalid request format");
        return;
    }

    String command = request.substring(startIndex, endIndex); // Extract the command part
    Serial.println("Extracted Command:");
    Serial.println(command); // For debugging

    // Parse command format: "led_1_flashing" or "motor_2_stop"
    int firstUnderscore = command.indexOf('_');
    int secondUnderscore = command.indexOf('_', firstUnderscore + 1);

    if (firstUnderscore != -1 && secondUnderscore != -1) {
        String type = command.substring(0, firstUnderscore); // Extract "led" or "motor"
        type.remove(0, 1); // Remove leading slash
        String platform = command.substring(firstUnderscore + 1, secondUnderscore); // Extract platform code
        String mode = command.substring(secondUnderscore + 1); // Extract mode (e.g., "flashing")

        Serial.println(type);     // Debugging: Print parsed type
        Serial.println(platform); // Debugging: Print parsed platform
        Serial.println(mode);     // Debugging: Print parsed mode

        if (type == "led") {
            handleLedCommand(platform.toInt(), mode, client);
        } else if (type == "motor") {
            handleMotorCommand(platform.toInt(), mode, client);
        } else {
            client.println("Invalid type");
        }
    } else {
        client.println("Invalid command format");
    }
}


void handleLedCommand(int platform, String mode, WiFiClient &client) {
    int *currentMode = nullptr;

    // Map platform to the correct LED mode variable
    switch (platform) {
        case 1:
            currentMode = &currentLedMode1;
            break;
        case 2:
            currentMode = &currentLedMode2;
            break;
        case 3:
            currentMode = &currentLedMode3;
            break;
        default:
            client.println("Invalid LED platform");
            return;
    }

    // Map mode string to LED mode values
    if (mode == "rainbow") {
        *currentMode = 0;
        client.println("LED set to rainbow mode");
    } else if (mode == "solid") {
        *currentMode = 1;
        client.println("LED set to solid mode");
    } else if (mode == "flashing") {
        *currentMode = 2;
        client.println("LED set to flashing mode");
    } else if (mode == "theater_chase") {
        *currentMode = 3;
        client.println("LED set to theater chase mode");
    } else if (mode == "sindist") {
        *currentMode = 4;
        client.println("LED set to sinusoidal distribution mode");
    } else if (mode == "windupdown") {
        *currentMode = 5;
        client.println("LED set to wind up/down mode");
    } else {
        client.println("Invalid LED mode");
    }
}

void handleMotorCommand(int platform, String mode, WiFiClient &client) {
    int *currentMode = nullptr;

    // Map platform to the correct motor mode variable
    switch (platform) {
        case 1:
            currentMode = &currentMode1;
            break;
        case 2:
            currentMode = &currentMode2;
            break;
        case 3:
            currentMode = &currentMode3;
            break;
        default:
            client.println("Invalid motor platform");
            return;
    }

    // Map mode string to motor mode values
    if (mode == "stop") {
        *currentMode = 0;
        client.println("Motor stopped");
    } else if (mode == "sinusoidal") {
        *currentMode = 1;
        client.println("Motor set to sinusoidal mode");
    } else if (mode == "continuous") {
        *currentMode = 2;
        client.println("Motor set to continuous mode");
    } else {
        client.println("Invalid motor mode");
    }
}


uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip1.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip1.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip1.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}


