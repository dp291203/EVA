#include <Encoder.h>

// Define encoder objects
Encoder myEnc[4] = {Encoder(6, 7), Encoder(10, 11), Encoder(17, 16), Encoder(14, 15)};

void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
}

// Store previous positions
long oldPosition[4] = {0, 0, 0, 0};

void loop() {
  for (int i = 0; i < 4; i++) {
    long newPos = myEnc[i].read();
    if (newPos != oldPosition[i]) {
      oldPosition[i] = newPos;
      Serial.print("Encoder ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(newPos);
    }
  }
}

#include <Encoder.h>

// Define encoder objects
Encoder myEnc[4] = {Encoder(6, 7), Encoder(10, 11), Encoder(17, 16), Encoder(14, 15)};

// Encoder parameters
const float encoderResolution = 2380.0;
const float sampling_time = 0.075; // 75ms in seconds

// Store previous positions and RPM values
long oldPosition[4] = {0, 0, 0, 0};
long lastCount[4] = {0, 0, 0, 0};
float rpm[4] = {0.0, 0.0, 0.0, 0.0};

void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test with RPM:");
}

void loop() {
  for (int i = 0; i < 4; i++) {
    long currentCounts = myEnc[i].read();
    long positionChange = currentCounts - lastCount[i];

    // Calculate RPM
    rpm[i] = (positionChange / encoderResolution) * (60.0 / sampling_time);
    lastCount[i] = currentCounts;

    if (currentCounts != oldPosition[i]) {
      oldPosition[i] = currentCounts;
      Serial.print("Encoder ");
      Serial.print(i + 1);
      Serial.print(": Position = ");
      Serial.print(currentCounts);
      Serial.print(", RPM = ");
      Serial.println(rpm[i]);
    }
  }

  delay(75); // Match the sampling time
}
