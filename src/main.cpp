#include <Arduino.h>
#include "ESP8266WiFi.h"

#include <Wire.h>

#define ADDR 0x23 // 7-bit I2C address of the sensor

#define BLINKER_PRINT Serial
#define BLINKER_WIFI

#include "Blinker.h"

char auth[] = "559eb880b1cc";
char ssid[] = "iQOO 9";
char pswd[] = "5k6zaew9ebgxahr";

bool autoControl = true;

void slider1_callback(int32_t);

// void button1_callback(const String &);
//
// void ledButton1_callback(const String &);
//
// void ledButton2_callback(const String &);
//
// void ledButton3_callback(const String &);
//
// void ledButton4_callback(const String &);
//
// void waterPumpButton_callback(const String &);
//
// void autoControlButton_callback(const String &);


// 新建组件对象
BlinkerButton Button1("btn-abc");
BlinkerButton ledButton1("led1");
BlinkerButton ledButton2("led2");
BlinkerButton ledButton3("led3");
BlinkerButton ledButton4("led4");
BlinkerButton waterPumpButton("waterPump");
BlinkerButton autoControlButton("autoControl");

BlinkerNumber Number1("num-abc");

BlinkerNumber HUMI("humi");
BlinkerNumber TEMP("temp");
BlinkerNumber LIGHT("light");
BlinkerSlider Slider1("Slider_1", slider1_callback);


#include <DHT.h>
#define DHTPIN 2
#define DHTTYPE DHT11   // DHT 11
DHT dht(DHTPIN, DHTTYPE);

// 定义LED和水泵连接的数字引脚
const int ledPins[] = {14, 12, 13, 15}; // LED连接到D5, D6, D7, D8
const int numLEDs = sizeof(ledPins) / sizeof(ledPins[0]);
const int waterPumpPin = 9; // 水泵连接的引脚

uint32_t read_time = 0;

float humi_read = 50;
float temp_read = 20;
float light = 30;


void slider1_callback(int32_t value) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    BLINKER_LOG("get slider value: ", value);
}

// 按下按键即会执行该函数
void button1_callback(const String &state) {
    BLINKER_LOG("get button state: ", state);
    if (state == BLINKER_CMD_ON) {
        digitalWrite(LED_BUILTIN, LOW);
        Button1.print("on");
    } else if (state == BLINKER_CMD_OFF) {
        digitalWrite(LED_BUILTIN, HIGH);
        Button1.print("off");
    }
}

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

void dataStorage() {
    Blinker.dataStorage("data0", temp_read);
    Blinker.dataStorage("data1", humi_read);
    Blinker.dataStorage("data2", light);

    // Blinker.dataStorage("data0", 57);
    // Blinker.dataStorage("data1", 86);
    // Blinker.dataStorage("data2", 45);
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
}

void setup() {
    Serial.begin(115200);
    BLINKER_DEBUG.stream(Serial);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Blinker.begin(auth, ssid, pswd);
    Blinker.attachData(dataRead);
    Blinker.attachHeartbeat(heartbeat);
    Blinker.attachDataStorage(dataStorage);

    Button1.attach(button1_callback);
    ledButton1.attach(ledButton1_callback);
    ledButton2.attach(ledButton2_callback);
    ledButton3.attach(ledButton3_callback);
    ledButton4.attach(ledButton4_callback);
    waterPumpButton.attach(waterPumpButton_callback);
    autoControlButton.attach(autoControlButton_callback);


    dht.begin();

    // Serial.begin(9600);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for Leonardo only
    }
    Wire.begin();
    Wire.beginTransmission(ADDR);
    Wire.write(0x01); // Power on
    Wire.endTransmission();

    // 设置所有LED和水泵引脚为输出模式
    for (int i = 0; i < numLEDs; i++) {
        pinMode(ledPins[i], OUTPUT);
    }
    pinMode(waterPumpPin, OUTPUT);
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
        Blinker.run();
        Wire.requestFrom(ADDR, 2); // 2byte every time
        for (val = 0; Wire.available() >= 1;) {
            char c = Wire.read();
            //Serial.println(c, HEX);
            val = (val << 8) + (c & 0xFF);
        }
        val = val / 1.2;
        // Serial.print("lx: ");
        // Serial.println(val);
        // Serial.println("OK");
        light = val;
        BLINKER_LOG("Light: ", light, " lx");


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