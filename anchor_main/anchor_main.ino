#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgRTLS.hpp>
#include "t_utils.h"

#define NUM_ANCHOR 3
#define CD 2  // COMMON_DIMEMSIONS =  NUM_ANCHOR - 1
#define FD 2  //FIXED_DIMEMSIONS

// connection pins
#if defined(ESP8266)
const uint8_t PIN_RST = 5; // reset pin
const uint8_t PIN_IRQ = 4; // irq pin
const uint8_t PIN_SS = 15; // spi select pin
#else
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin
#endif

const uint8_t MOVING_AVG_SIZE = 5;
const float RANGE_DIFF = 0.4;

float **b;
float **final_rs;
float **temp;

float data[NUM_ANCHOR][3] = {
  { 0, 0, 0 },
  { 3, 0, 0 },
  { 3, 2.5, 0 }
};

// Extended Unique Identifier register. 64-bit device identifier. Register file: 0x01
char EUI[] = "AA:BB:CC:DD:EE:FF:00:01";

// double range_self;
// double range_B;
// double range_C;

float a_arr[MOVING_AVG_SIZE] = {0.0, 0.0, 0.0, 0.0, 0.0};
float b_arr[MOVING_AVG_SIZE] = {0.0, 0.0, 0.0, 0.0, 0.0};
float c_arr[MOVING_AVG_SIZE] = {0.0, 0.0, 0.0, 0.0, 0.0};

float sum_a = 0.0;
float sum_b = 0.0;
float sum_c = 0.0;

char a_c = 0;
char b_c = 0;
char c_c = 0;

boolean received_B = false;

byte target_eui[8];
byte tag_shortAddress[] = { 0x05, 0x00 };

byte anchor_b[] = { 0x02, 0x00 };
uint16_t next_anchor = 2;

byte anchor_c[] = { 0x03, 0x00 };

device_configuration_t DEFAULT_CONFIG = {
  false,
  true,
  true,
  true,
  false,
  SFDMode::STANDARD_SFD,
  Channel::CHANNEL_5,
  DataRate::RATE_850KBPS,
  PulseFrequency::FREQ_16MHZ,
  PreambleLength::LEN_256,
  PreambleCode::CODE_3
};

frame_filtering_configuration_t ANCHOR_FRAME_FILTER_CONFIG = {
  false,
  false,
  true,
  false,
  false,
  false,
  false,
  true /* This allows blink frames */
};

void setup() {
  // DEBUG monitoring
  Serial.begin(115200);
  Serial.println(F("### DW1000Ng-arduino-ranging-anchorMain ###"));
  // initialize the driver
  #if defined(ESP8266)
    DW1000Ng::initializeNoInterrupt(PIN_SS);
  #else
    DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
  #endif
  Serial.println(F("DW1000Ng initialized ..."));
  // general configuration
  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
  DW1000Ng::enableFrameFiltering(ANCHOR_FRAME_FILTER_CONFIG);

  DW1000Ng::setEUI(EUI);

  DW1000Ng::setPreambleDetectionTimeout(64);
  DW1000Ng::setSfdDetectionTimeout(273);
  DW1000Ng::setReceiveFrameWaitTimeoutPeriod(5000);

  DW1000Ng::setNetworkId(RTLS_APP_ID);
  DW1000Ng::setDeviceAddress(1);
  DW1000Ng::setAntennaDelay(16436);

   Serial.println(F("Committed configuration ..."));
  // // DEBUG chip info and registers pretty printed
  // char msg[128];
  // DW1000Ng::getPrintableDeviceIdentifier(msg);
  // Serial.print("Device ID: ");
  // Serial.println(msg);
  // DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
  // Serial.print("Unique ID: ");
  // Serial.println(msg);
  // DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
  // Serial.print("Network ID & Device Address: ");
  // Serial.println(msg);
  // DW1000Ng::getPrintableDeviceMode(msg);
  // Serial.print("Device mode: ");
  // Serial.println(msg);

  initLS();
}

void loop() {
  if (DW1000NgRTLS::receiveFrame()) {
    size_t recv_len = DW1000Ng::getReceivedDataLength();
    byte recv_data[recv_len];
    DW1000Ng::getReceivedData(recv_data, recv_len);

    if (recv_data[0] == BLINK) {
      DW1000NgRTLS::transmitRangingInitiation(&recv_data[2], tag_shortAddress);
      DW1000NgRTLS::waitForTransmission();

      RangeAcceptResult result = DW1000NgRTLS::anchorRangeAccept(NextActivity::RANGING_CONFIRM, next_anchor);
      if (!result.success) return;
      //range_self = result.range;
      float temp = updateArr(a_arr, result.range, sum_a);
      data[0][2] = temp;
      String rangeString = "Range: ";
      rangeString += temp;
      rangeString += " m";
      rangeString += "\t RX power: ";
      rangeString += DW1000Ng::getReceivePower();
      rangeString += " dBm";
      Serial.println(rangeString);

    } else if (recv_data[9] == 0x60) {
      double range = static_cast<double>(DW1000NgUtils::bytesAsValue(&recv_data[10], 2) / 1000.0);
      String rangeReportString = "Range from: ";
      rangeReportString += recv_data[7];

      float temp;
      if (received_B == false && recv_data[7] == anchor_b[0] && recv_data[8] == anchor_b[1]) {
        //range_B = range;
        temp = updateArr(b_arr, range, sum_b);
        data[1][2] = temp;
        received_B = true;
      } else if (received_B == true && recv_data[7] == anchor_c[0] && recv_data[8] == anchor_c[1]) {
        //range_C = range;
        temp = updateArr(c_arr, range, sum_c);
        data[2][2] = temp;
        calculateLS();
        received_B = false;
      } else {
        received_B = false;
        return;
      }
      rangeReportString += " = ";
      rangeReportString += temp;
      Serial.println(rangeReportString);
    }
  }
}

void initLS() {
  float **a, **at, **temp1, **temp2;
  allocate(&b, FD, 1);
  allocate(&final_rs, FD, 1);
  allocate(&temp, FD, CD);

  allocate(&a, CD, FD);
  allocate(&at, FD, CD);
  allocate(&temp1, FD, FD);
  allocate(&temp2, FD, FD);

  calculatePreLS(a, at, temp1, temp2);

  free2DArray(a, CD);
  free2DArray(at, FD);
  free2DArray(temp1, FD);
  free2DArray(temp2, FD);
}

void calculateB() {
  float b_temp = data[0][0] * data[0][0] + data[0][1] * data[0][1] - data[0][2] * data[0][2];

  for (uint8_t i = 0; i < CD; ++i) {
    b[i][0] = 0.5 * (data[i + 1][0] * data[i + 1][0] + data[i + 1][1] * data[i + 1][1] - data[i + 1][2] * data[i + 1][2] - b_temp);
  }
}

void calculatePreLS(float **a, float **at, float **temp1, float **temp2) {
  uint8_t i, j;
  //Cal a
  for (i = 1; i <= CD; ++i) {
    for (j = 0; j < FD; ++j) {
      if (j == 0) {
        a[i - 1][j] = data[i][0] - data[0][0];
      } else if (j == 1) {
        a[i - 1][j] = data[i][1] - data[0][1];
      }
    }
  }

  //Cal at
  calculateTransform(at, a, CD, FD);

  //Cal a * at
  multiplyMatrices(at, a, temp1, FD, CD, CD, FD);

  //Cal (a*at)-
  calculateInverse(temp2, temp1, FD, FD);

  // Cal (a*at)- * at
  multiplyMatrices(temp2, at, temp, FD, FD, FD, CD);
}

void printMatrixArduino(float **a, uint8_t row, uint8_t col) {
  for (uint8_t i = 0; i < row; ++i) {
    for (uint8_t j = 0; j < col; ++j) {
      Serial.print("x[i][j]: ");
      Serial.println(a[i][j]);
    }
    Serial.print("\n");
  }
}

void calculateLS() {
  calculateB();
  multiplyMatrices(temp, b, final_rs, FD, CD, CD, 1);

  String positioning = "Found position - x: ";
        positioning += final_rs[0][0];
        positioning += " y: ";
        positioning += final_rs[0][1];
        Serial.println(positioning);
}

float updateArr(float *a, float newVal, float &sum) {
    a[MOVING_AVG_SIZE - 1] = newVal;
    sum += newVal;
    memcpy(a_arr, a_arr + 1, (MOVING_AVG_SIZE - 1) * sizeof(float));
    if (0 == a[0]) {
      return;
    }
    sum -= a[0];
    return sum / MOVING_AVG_SIZE;
}