#include <Arduino.h>
#include "ESP8266WiFi.h"
#include <Wire.h>
#include <DHT.h>
#include <SoftwareSerial.h>

#define ADDR 0x23 // 7-bit I2C address of the sensor
#define DHTPIN 2
#define DHTTYPE DHT11   // DHT 11
#define SEND_PIN D0 // MAX485 DE和RE连接的引脚

DHT dht(DHTPIN, DHTTYPE);
SoftwareSerial mySerial(3, 1); // TX, RX for RS485 communication

// Blinker settings
#define BLINKER_PRINT Serial
#define BLINKER_WIFI
#include "Blinker.h"

char auth[] = "559eb880b1cc";
char ssid[] = "iQOO 9";
char pswd[] = "5k6zaew9ebgxahr";

bool autoControl = true;

BlinkerButton Button1("btn-abc");
BlinkerButton ledButton1("led1");
BlinkerButton ledButton2("led2");
BlinkerButton ledButton3("led3");
BlinkerButton ledButton4("led4");
BlinkerButton waterPumpButton("waterPump");
BlinkerButton autoControlButton("autoControl");

BlinkerNumber HUMI("humi");
BlinkerNumber TEMP("temp");
BlinkerNumber LIGHT("light");
BlinkerNumber TEMP_SOIL("temp_soil");
BlinkerNumber HUMI_SOIL("humi_soil");
BlinkerNumber EC("ec");
BlinkerNumber N("n");
BlinkerNumber P("p");
BlinkerNumber K("k");


const int ledPins[] = {14, 12, 13, 15}; // LED连接到D5, D6, D7, D8
const int numLEDs = sizeof(ledPins) / sizeof(ledPins[0]);
const int waterPumpPin = 9; // 水泵连接的引脚

// Define a structure to hold sensor data
struct SensorData {
    float temp;
    float humi;
    float ec;
    float n;
    float p;
    float k;
};
uint32_t read_time = 0;
float humi_read = 50;
float temp_read = 20;
float light = 30;
float humi_soil, temp_soil, ec_read, n_read, p_read, k_read;

// Callback functions are omitted for brevity

// void slider1_callback(int32_t value) {
//     digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
//     BLINKER_LOG("get slider value: ", value);
// }

void dataStorage() {
    Blinker.dataStorage("data0", temp_read);
    Blinker.dataStorage("data1", humi_read);
    Blinker.dataStorage("data2", light);
}

void dataRead(const String &data) {
    BLINKER_LOG("Blinker readString: ", data);

    Blinker.vibrate();

    uint32_t BlinkerTime = millis();

    Blinker.print("millis", BlinkerTime);
}

void heartbeat() {
    HUMI.print(humi_read);
    TEMP.print(temp_read);
    LIGHT.print(light);
    TEMP_SOIL.print(temp_soil);
    HUMI_SOIL.print(humi_soil);
    EC.print(ec_read);
    N.print(n_read);
    P.print(p_read);
    K.print(k_read);
}

// void sendRequest() {
//     digitalWrite(SEND_PIN, HIGH); // Switch to send mode
//     byte data_to_send[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x06, 0xC5, 0xC8};
//     mySerial.write(data_to_send, sizeof(data_to_send));
//     digitalWrite(SEND_PIN, LOW); // Switch back to receive mode
// }

SensorData readDeviceData() {
    SensorData sensorData = {0}; // Initialize with zero values
    byte data_to_send[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x06, 0xC5, 0xC8};

    digitalWrite(SEND_PIN, HIGH); // Switch to send mode
    mySerial.write(data_to_send, sizeof(data_to_send));
    digitalWrite(SEND_PIN, LOW); // Switch back to receive mode

    Blinker.delay(100); // wait for response

    if (mySerial.available() >= 12) {
        byte address = mySerial.read();
        byte function_code = mySerial.read();
        byte data_length = mySerial.read();

        // 读取数据
        byte temperature[2];
        byte humidity[2];
        byte ec[2];
        byte n[2];
        byte p[2];
        byte k[2];

        mySerial.readBytes(temperature, 2);
        mySerial.readBytes(humidity, 2);
        mySerial.readBytes(ec, 2);
        mySerial.readBytes(n, 2);
        mySerial.readBytes(p, 2);
        mySerial.readBytes(k, 2);
        byte crc[2];
        mySerial.readBytes(crc, 2);

        // 数据处理（与原代码相同）
        int temperature_data = (temperature[0] << 8) | temperature[1];
        int humidity_data = (humidity[0] << 8) | humidity[1];
        int ec_value = (ec[0] << 8) | ec[1];
        int n_value = (n[0] << 8) | n[1];
        int p_value = (p[0] << 8) | p[1];
        int k_value = (k[0] << 8) | k[1];

        // 温度计算
        float temperature_value;
        if (temperature_data & 0x8000) {
            temperature_value = -(0x10000 - temperature_data) / 10.0;
        } else {
            temperature_value = temperature_data / 10.0;
        }

        float humidity_value = humidity_data / 10.0;

        sensorData.temp = temperature_value;
        sensorData.humi = humidity_value;
        sensorData.ec = ec_value;
        sensorData.n = n_value;
        sensorData.p = p_value;
        sensorData.k = k_value;

        //
        // // Read and process RS485 data
        // byte data[12];
        // mySerial.readBytes(data, 12);
        //
        // sensorData.ec = (data[3] << 8) | data[4]; // Assuming correct indexing based on the device's protocol
        // sensorData.n = (data[5] << 8) | data[6];
        // sensorData.p = (data[7] << 8) | data[8];
        // sensorData.k = (data[9] << 8) | data[10];
    }
    return sensorData; // Return the struct containing the data
}


float readLightSensor() {
    int val = 0;
    // reset
    Wire.beginTransmission(ADDR);
    Wire.write(0b00000111);
    Wire.endTransmission();

    Wire.beginTransmission(ADDR);
    Wire.write(0b00100000);
    Wire.endTransmission();

    // typical read delay 120ms
    Blinker.delay(120);
    Wire.requestFrom(ADDR, 2); // 2bytes every time
    for (val = 0; Wire.available() >= 1;) {
        char c = Wire.read();
        val = (val << 8) + (c & 0xFF);
    }
    light = val / 1.2; // Convert to lx
    return light; // Return the light value
}

// // 按下按键即会执行该函数
// void button1_callback(const String &state) {
//     BLINKER_LOG("get button state: ", state);
//     if (state == BLINKER_CMD_ON) {
//         digitalWrite(LED_BUILTIN, LOW);
//         Button1.print("on");
//     } else if (state == BLINKER_CMD_OFF) {
//         digitalWrite(LED_BUILTIN, HIGH);
//         Button1.print("off");
//     }
// }

void ledButton1_callback(const String &state) {
    BLINKER_LOG("get ledButton1 state: ", state);
    if (state == "on") {
        digitalWrite(ledPins[0], HIGH);
        ledButton1.print("on");
    } else if (state == "off") {
        digitalWrite(ledPins[0], LOW);
        ledButton1.print("off");
    }
}

void ledButton2_callback(const String &state) {
    BLINKER_LOG("get ledButton2 state: ", state);
    if (state == "on") {
        digitalWrite(ledPins[1], HIGH);
        ledButton2.print("on");
    } else if (state == "off") {
        digitalWrite(ledPins[1], LOW);
        ledButton2.print("off");
    }
}

void ledButton3_callback(const String &state) {
    BLINKER_LOG("get ledButton3 state: ", state);
    if (state == "on") {
        digitalWrite(ledPins[2], HIGH);
        ledButton3.print("on");
    } else if (state == "off") {
        digitalWrite(ledPins[2], LOW);
        ledButton3.print("off");
    }
}

void ledButton4_callback(const String &state) {
    BLINKER_LOG("get ledButton4 state: ", state);
    if (state == "on") {
        digitalWrite(ledPins[3], HIGH);
        ledButton4.print("on");
    } else if (state == "off") {
        digitalWrite(ledPins[3], LOW);
        ledButton4.print("off");
    }
}

void waterPumpButton_callback(const String &state) {
    BLINKER_LOG("get waterPumpButton state: ", state);
    if (state == "on") {
        digitalWrite(waterPumpPin, HIGH);
        waterPumpButton.print("on");
    } else if (state == "off") {
        digitalWrite(waterPumpPin, LOW);
        waterPumpButton.print("off");
    }
}

void autoControlButton_callback(const String &state) {
    BLINKER_LOG("get autoControlButton state: ", state);
    if (state == "on") {
        autoControl = true;
        autoControlButton.print("on");
    } else if (state == "off") {
        autoControl = false;
        autoControlButton.print("off");
    }
}

void setup() {
    Serial.begin(115200);
    BLINKER_DEBUG.stream(Serial);

    for (int i = 0; i < numLEDs; i++) {
        pinMode(ledPins[i], OUTPUT);
        digitalWrite(ledPins[i], LOW);
    }
    pinMode(waterPumpPin, OUTPUT);
    digitalWrite(waterPumpPin, HIGH);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    pinMode(SEND_PIN, OUTPUT);
    digitalWrite(SEND_PIN, LOW); // Set to receive mode

    Blinker.begin(auth, ssid, pswd);
    Blinker.attachData(dataRead);
    Blinker.attachHeartbeat(heartbeat);
    Blinker.attachDataStorage(dataStorage);

    // Button1.attach(button1_callback);
    ledButton1.attach(ledButton1_callback);
    ledButton2.attach(ledButton2_callback);
    ledButton3.attach(ledButton3_callback);
    ledButton4.attach(ledButton4_callback);
    waterPumpButton.attach(waterPumpButton_callback);
    autoControlButton.attach(autoControlButton_callback);

    dht.begin();

    mySerial.begin(9600); // Start RS485 communication
    // Initialize the sensor on I2C here if needed

    while (!Serial) {
        ; // wait for serial port to connect. Needed for Leonardo only
    }
    Wire.begin();
    Wire.beginTransmission(ADDR);
    Wire.write(0x01); // Power on
    Wire.endTransmission();

}

void loop() {
    Blinker.run();

    if (read_time == 0 || (millis() - read_time) >= 2000) {
        read_time = millis();

        float h = dht.readHumidity();
        float t = dht.readTemperature();

        if (isnan(h) || isnan(t)) {
            BLINKER_LOG("Failed to read from DHT sensor!");
            return;
        }

        float hic = dht.computeHeatIndex(t, h, false);

        humi_read = h;
        temp_read = t;

        BLINKER_LOG("Humidity: ", h, " %");
        BLINKER_LOG("Temperature: ", t, " *C");
        BLINKER_LOG("Heat index: ", hic, " *C");


        light = readLightSensor(); // Get light sensor value

        SensorData deviceData = readDeviceData(); // Get RS485 data
        temp_soil = deviceData.temp;
        humi_soil = deviceData.humi;
        ec_read = deviceData.ec;
        n_read = deviceData.n;
        p_read = deviceData.p;
        k_read = deviceData.k;

        BLINKER_LOG("Light: ", light);
        BLINKER_LOG("Humidity of soil: ", humi_soil, " %");
        BLINKER_LOG("Temperature of soil: ", temp_soil, " *C");
        BLINKER_LOG("EC: ", ec_read, " uS/cm");
        BLINKER_LOG("N: ", n_read, " mg/kg");
        BLINKER_LOG("P: ", p_read, " mg/kg");
        BLINKER_LOG("K: ", k_read, " mg/kg");


        if (autoControl) {
            if (light < 30) {
                digitalWrite(ledPins[0], HIGH);
                digitalWrite(ledPins[1], HIGH);
                digitalWrite(ledPins[2], HIGH);
                digitalWrite(ledPins[3], HIGH);
            } else if (light < 60) {
                digitalWrite(ledPins[0], HIGH);
                digitalWrite(ledPins[1], HIGH);
                digitalWrite(ledPins[2], HIGH);
                digitalWrite(ledPins[3], LOW);
            } else if (light < 90) {
                digitalWrite(ledPins[2], HIGH);
                digitalWrite(ledPins[3], HIGH);
                digitalWrite(ledPins[0], LOW);
                digitalWrite(ledPins[1], LOW);
            } else if (light < 120) {
                digitalWrite(ledPins[3], HIGH);
                digitalWrite(ledPins[2], LOW);
                digitalWrite(ledPins[1], LOW);
                digitalWrite(ledPins[0], LOW);
            } else {
                digitalWrite(ledPins[0], LOW);
                digitalWrite(ledPins[1], LOW);
                digitalWrite(ledPins[2], LOW);
                digitalWrite(ledPins[3], LOW);
            }

            if (humi_read < 50) {
                digitalWrite(waterPumpPin, HIGH);
            } else {
                digitalWrite(waterPumpPin, LOW);
            }

        }
    }
}

