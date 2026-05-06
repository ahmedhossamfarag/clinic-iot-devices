#include <LSM6DS3.h>
#include <Wire.h>

#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>
#include <tensorflow/lite/schema/schema_generated.h>

#include "model.h"

#include <ArduinoBLE.h>    // 🛠 Core BLE library for radio control

// =====================
// BLE SETUP
// =====================

#define DEVICE_NAME "CIoT"
#define DEVICE_ID "a38deac2-1ab2-4d23-8ba4-e68399782297"
// 160 units * 0.625 ms = 100 milliseconds (advertises 10 times a second).
#define ADV_INTERVAL 1600 * 1

// =====================
// IMU SETUP
// =====================

LSM6DS3 myIMU(I2C_MODE, 0x6A);

// =====================
// TFLITE SETUP
// =====================

#define SAMPLING_DELAY 20 // ms (50Hz sampling rate)
#define NUM_SAMPLES 119
#define NUM_GESTURES 2
#define DEFULT_GESTURE 0

tflite::MicroErrorReporter tflErrorReporter;

// ONLY REQUIRED OPS (optimized)
tflite::MicroMutableOpResolver<5> tflOpsResolver;

const tflite::Model* tflModel = nullptr;
tflite::MicroInterpreter* tflInterpreter = nullptr;
TfLiteTensor* tflInputTensor = nullptr;
TfLiteTensor* tflOutputTensor = nullptr;

constexpr int tensorArenaSize = 10 * 1024;
byte tensorArena[tensorArenaSize] __attribute__((aligned(16)));

int samplesRead = 0;

int lastPrediction = DEFULT_GESTURE;


// =====================
// HELPER FUNCTION TO UPDATE ADVERTISED GESTURE
// =====================

void setGesture(uint8_t gesture) {
  uint8_t mfgData[3];
  mfgData[0] = 0x34;   // company ID LSB
  mfgData[1] = 0x12;   // company ID MSB
  mfgData[2] = gesture;    // gesture value (0-255)
  BLE.setManufacturerData(mfgData, sizeof(mfgData));
}

// =====================
// SETUP
// =====================

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // 1) Initialize BLE radio
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);                // 🚫 Halt in an infinite loop if the Bluetooth radio is broken
  }

  if (myIMU.begin() != 0) {
    Serial.println("IMU Error");
    while (1);
  }

  // Load model
  tflModel = tflite::GetModel(model);

  if (tflModel->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model mismatch!");
    while (1);
  }

  // =====================
  // REGISTER OPS (IMPORTANT)
  // =====================
  tflOpsResolver.AddFullyConnected();
  tflOpsResolver.AddSoftmax();
  tflOpsResolver.AddReshape();
  tflOpsResolver.AddQuantize();
  tflOpsResolver.AddDequantize();

  // Interpreter
  tflInterpreter = new tflite::MicroInterpreter(
    tflModel,
    tflOpsResolver,
    tensorArena,
    tensorArenaSize,
    &tflErrorReporter
  );

  tflInterpreter->AllocateTensors();

  tflInputTensor = tflInterpreter->input(0);
  tflOutputTensor = tflInterpreter->output(0);

  // 2) Set the device name
  BLE.setLocalName(DEVICE_NAME);

  // 3) Set the advertise interval
  BLE.setAdvertisingInterval(ADV_INTERVAL);

  // 4) Set the advertise data to the device ID.
  BLE.setAdvertisedServiceUuid(DEVICE_ID);

  // 5) Set the manufacturer data
  setGesture(DEFULT_GESTURE);

  // 6) Start advertising
  BLE.advertise();

  Serial.println("Device OK");
}

void loop() {

  float aX, aY, aZ, gX, gY, gZ;

  // =====================
  // 1. COLLECT SAMPLES
  // =====================
  while (samplesRead < NUM_SAMPLES) {

    aX = myIMU.readFloatAccelX();
    aY = myIMU.readFloatAccelY();
    aZ = myIMU.readFloatAccelZ();

    gX = myIMU.readFloatGyroX();
    gY = myIMU.readFloatGyroY();
    gZ = myIMU.readFloatGyroZ();

    // =====================
    // NORMALIZATION
    // =====================
    tflInputTensor->data.f[samplesRead * 6 + 0] = (aX + 4.0) / 8.0;
    tflInputTensor->data.f[samplesRead * 6 + 1] = (aY + 4.0) / 8.0;
    tflInputTensor->data.f[samplesRead * 6 + 2] = (aZ + 4.0) / 8.0;

    tflInputTensor->data.f[samplesRead * 6 + 3] = (gX + 2000.0) / 4000.0;
    tflInputTensor->data.f[samplesRead * 6 + 4] = (gY + 2000.0) / 4000.0;
    tflInputTensor->data.f[samplesRead * 6 + 5] = (gZ + 2000.0) / 4000.0;

    samplesRead++;

    delay(SAMPLING_DELAY);
  }

  // =====================
  // 2. RUN INFERENCE
  // =====================
  TfLiteStatus status = tflInterpreter->Invoke();

  if (status != kTfLiteOk) {
    Serial.println("Inference failed!");
    return;
  }

  // =====================
  // 3. GET RESULTS
  // =====================
  int bestIndex = 0;
  float bestScore = 0;

  for (int i = 0; i < NUM_GESTURES; i++) {

    float score = tflOutputTensor->data.f[i];

    if (score > bestScore) {
      bestScore = score;
      bestIndex = i;
    }
  }

  Serial.print("Best gesture: ");
  Serial.println(bestIndex);

  // =====================
  // 4. ADVERTISE RESULT
  // =====================
  
  if (bestIndex != lastPrediction) {
    BLE.stopAdvertise(); // Stop advertising to update the data
    setGesture(bestIndex + 1); // Update the manufacturer data with the new gesture
    BLE.advertise(); // Restart advertising with the updated data

    Serial.print("Updated advertised data: ");
    Serial.println(bestIndex);

    lastPrediction = bestIndex;
  }

  // =====================
  // 5. RESET
  // =====================
  samplesRead = 0;
}