#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <SoftwareSerial.h>
#include <ModbusMaster.h>
#include "main.h"

//#define DEBUG

#ifdef DEBUG
#define TRACE(x) Serial.print(x);
#else
#define TRACE(x)
#endif

// Serial Port defines
#define RX 16
#define TX 17
#define BAUD_RATE 9600

// LED Defines
#define POWER_LED 26
#define ACTION_LED 25

// ModBus register defines

// Energy Meter Registers

// All registers use function code 3

#define METER_REG_VOLTS 0x00         // 2 word INT32
#define METER_REG_AMPS 0x02          // 2 word INT32
#define METER_REG_WATTS 0x04         // 2 word INT32
#define METER_REG_VA 0x06            // 2 word INT32
#define METER_REG_VAR 0x08           // 2 word INT32
#define METER_REG_W_DMD 0x0A         // 2 word INT32
#define METER_REG_W_DMD_PEAK 0x0C    // 2 word INT32
#define METER_REG_PF 0x0E            // 1 word INT32
#define METER_REG_HZ 0x0F            // 1 word INT32
#define METER_REG_KWP_TOTAL 0x10     // 2 word INT32
#define METER_REG_KVARH_TOTAL 0x12   // 2 word INT32
#define METER_REG_KWP_PARTIAL 0x14   // 2 word INT32
#define METER_REG_KVARH_PARTIAL 0x16 // 2 word INT32
#define METER_REG_KWN_TOTAL 0x20     // 2 word INT32

// Solis 1500 Registers

// All registers use function code 4

#define SOLIS1500_REG_ACWATTS 3004   // Read AC Watts as Unsigned 32-Bit
#define SOLIS1500_REG_DCVOLTS 3021   // Read DC Volts as Unsigned 16-Bit
#define SOLIS1500_REG_DCCURRENT 3022 // Read DC Current as Unsigned 16-Bit
#define SOLIS1500_REG_ACVOLTS 3035   // Read AC Volts as Unsigned 16-Bit
#define SOLIS1500_REG_ACCURRENT 3038 // Read AC Current as Unsigned 16-Bit
#define SOLIS1500_REG_ACFREQ 3042    // Read AC Frequency as Unsigned 16-Bit
#define SOLIS1500_REG_TEMP 3041      // Read Inverter Temperature as Signed 16-Bit
#define SOLIS1500_REG_ALLTIMEKW 3008 // Read All Time Energy (KWH Total) as Unsigned 32-Bit
#define SOLIS1500_REG_TODAYKW 3014   // Read Today Energy (KWH Total) as 16-Bit

#define RETRY_ATTEMPTS 5 // Number of retries to connect to WiFi and MQTT

/* RTC Parameters */

#define SLEEP_TIME 10000000 // ESP32 will sleep for 10 seconds
#define TIMER_WAIT 60000000 // Wait 60 second between timer loops

#define LIGHT_THRESHOLD 200

uint8_t previous_light_state = 0;

hw_timer_t *timer1 = NULL;
bool do_timer_loop = true;

RTC_DATA_ATTR float RTCTotalpower = 0;
RTC_DATA_ATTR char RTCTimeStr[14];

/* Put your SSID & Password */
const char *ssid = "ssidhere";         // Enter SSID here
const char *password = "passwordhere"; // Enter Password here

/* Put IP Address details */
IPAddress local_ip(192, 168, 1, 20); // ip address
IPAddress gateway(192, 168, 1, 1); // gateway
IPAddress subnet(255, 255, 255, 0); // subnet
IPAddress primaryDNS(8, 8, 8, 8);   // optional
IPAddress secondaryDNS(8, 8, 4, 4); // optional

// mqtt client

const char *mqtt_server = "192.168.1.10";
const char *mqtt_user = "user";
const char *mqtt_password = "password";

WiFiClient espclient;
PubSubClient client(espclient);

SoftwareSerial softserial; // software serial device

ModbusMaster energymeter;   // ModbusMaster energy meter object
ModbusMaster solarinverter; // ModbusMaster solar inverter object

void go_to_sleep()
{
    // Set sleep timer and go to sleep
    // esp_sleep_enable_timer_wakeup(SLEEP_TIME);
    // esp_deep_sleep_start();
}

bool connect_wifi()
{
    // Connect to WiFi
    TRACE("\nConnecting to Wifi\n");
    WiFi.mode(WIFI_STA);

    if (!WiFi.config(local_ip, gateway, subnet, primaryDNS, secondaryDNS))
    {
        TRACE("Wifi Failed to configure\n");
        return false;
    }

    WiFi.begin(ssid, password);

    // Wait for connection
    uint8_t count = 0;

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        TRACE(".");
        if (count > RETRY_ATTEMPTS)
        {
            return false;
        }
        count++;
    }
    TRACE("\n");
    TRACE("Connected to ");
    TRACE(ssid);
    TRACE("\nIP address: ");
    TRACE(WiFi.localIP());
    TRACE("\nMAC address: ");
    TRACE(WiFi.macAddress());
    TRACE("\n");
    return true;
}

bool connect_mqtt()
{
    // set up mqtt client
    client.setServer(mqtt_server, 1883);

    // Loop until we're connected, or retries exceeds limit
    uint8_t count = 0;

    while (!client.connected())
    {
        TRACE("Attempting MQTT connection...\n");
        // Attempt to connect
        if (client.connect("SolarInverterClient", mqtt_user, mqtt_password))
        {
            TRACE("connected\n");
            return true;
        }
        TRACE("failed, rc=");
        TRACE(client.state());
        TRACE(" try again in 1 seconds\n");
        // Wait 1 seconds before retrying
        delay(1000);
        if (count > RETRY_ATTEMPTS)
            return false;
        count++;
    }
    return false;
}

#define BYTEORDER_BIG 1
#define BYTEORDER_LITTLE 2

void get_modbus_value(ModbusMaster target, u_int8_t functioncode, uint16_t reg, uint16_t length, const char *topic, u_int8_t decimals, uint8_t byteorder=BYTEORDER_BIG)
{
    u_int8_t status = 1;
    if (functioncode == 3)
    {
        status = target.readHoldingRegisters(reg, length);
    }
    else
    {
        status = target.readInputRegisters(reg, length);
    }

    if (status == 0)
    {
        float result = 0;

        if (length == 1)
        {
            result = float(target.getResponseBuffer(0));
        }
        if (length == 2)
        {
            if (byteorder == BYTEORDER_BIG){
                result = float(target.getResponseBuffer(0) + (u_int32_t(target.getResponseBuffer(1) << 16)));
            }
            else{
                result = float(target.getResponseBuffer(1) + (u_int32_t(target.getResponseBuffer(0) << 16)));
            }
        }
        switch (decimals)
        {
            case 1: result = result / 10; break;
            case 2: result = result / 100; break;
            case 3: result = result / 1000; break;
            default: break;
        }
        if (!client.connected())
        {
            connect_mqtt();
        }
        client.publish(topic, String(result).c_str(), false);
    }
    else
    {
        TRACE("Error on ");
        TRACE(topic);
        TRACE("\n");
    }
}

void get_data(void)
{
    TRACE("Getting meter data\n")

    // Energy Meter Registers

    get_modbus_value(energymeter, 3, METER_REG_VOLTS, 2, "/home/solarpv/meter/volts", 1);
    get_modbus_value(energymeter, 3, METER_REG_AMPS, 2, "/home/solarpv/meter/amps", 3);
    get_modbus_value(energymeter, 3, METER_REG_WATTS, 2, "/home/solarpv/meter/watts", 1);
    get_modbus_value(energymeter, 3, METER_REG_VA, 2, "/home/solarpv/meter/va", 1);
    get_modbus_value(energymeter, 3, METER_REG_VAR, 2, "/home/solarpv/meter/var", 1);
    get_modbus_value(energymeter, 3, METER_REG_W_DMD, 2, "/home/solarpv/meter/dmd", 1);
    get_modbus_value(energymeter, 3, METER_REG_W_DMD_PEAK, 2, "/home/solarpv/meter/dmdpeak", 1);
    get_modbus_value(energymeter, 3, METER_REG_PF, 1, "/home/solarpv/meter/pf", 1);
    get_modbus_value(energymeter, 3, METER_REG_HZ, 1, "/home/solarpv/meter/hz", 1);
    get_modbus_value(energymeter, 3, METER_REG_KWP_TOTAL, 2, "/home/solarpv/meter/kwptotal", 1);
    get_modbus_value(energymeter, 3, METER_REG_KVARH_TOTAL, 2, "/home/solarpv/meter/kvarhtotal", 1);
    get_modbus_value(energymeter, 3, METER_REG_KWP_PARTIAL, 2, "/home/solarpv/meter/kwppartial", 1);
    get_modbus_value(energymeter, 3, METER_REG_KVARH_PARTIAL, 2, "/home/solarpv/meter/kvarpartial", 1);
    get_modbus_value(energymeter, 3, METER_REG_KWN_TOTAL, 2, "/home/solarpv/meter/kwntotal", 1);

    // Check if inverter is online.  Inverter switches off at night

    // Solar Inverter Registers
    u_int8_t status = solarinverter.readInputRegisters(SOLIS1500_REG_ACWATTS, 1);
    if (status == solarinverter.ku8MBSuccess)
    {
        TRACE("Solar Inverter Online\n");
        client.publish("/home/solarpv/online", "1", false);
        get_modbus_value(solarinverter, 4, SOLIS1500_REG_ACWATTS, 2, "/home/solarpv/gridpower", 0, BYTEORDER_LITTLE);
        get_modbus_value(solarinverter, 4, SOLIS1500_REG_DCVOLTS, 1, "/home/solarpv/pvvolt", 1);
        get_modbus_value(solarinverter, 4, SOLIS1500_REG_DCCURRENT, 1, "/home/solarpv/pvamp", 1);
        get_modbus_value(solarinverter, 4, SOLIS1500_REG_ACVOLTS, 1, "/home/solarpv/gridvolt", 1);
        get_modbus_value(solarinverter, 4, SOLIS1500_REG_ACCURRENT, 1, "/home/solarpv/gridcurrent", 1);
        get_modbus_value(solarinverter, 4, SOLIS1500_REG_ACFREQ, 1, "/home/solarpv/gridfrequency", 2);
        get_modbus_value(solarinverter, 4, SOLIS1500_REG_TEMP, 1, "/home/solarpv/devicetemp", 1);
        get_modbus_value(solarinverter, 4, SOLIS1500_REG_ALLTIMEKW, 2, "/home/solarpv/totalpower", 0, BYTEORDER_LITTLE);
        get_modbus_value(solarinverter, 4, SOLIS1500_REG_TODAYKW, 1, "/home/solarpv/todaypower", 1);
    }
    else
    {
        // Inverter offline so publish 0 for most values
        TRACE("Solar Inverter Offline\n");

        if (!client.connected())
        {
            connect_mqtt();
        }
        client.publish("/home/solarpv/pvvolt", "0", false);
        client.publish("/home/solarpv/pvamp", "0", false);
        client.publish("/home/solarpv/gridvolt", "0", false);
        client.publish("/home/solarpv/gridcurrent", "0", false);
        client.publish("/home/solarpv/gridpower", "0", false);
        client.publish("/home/solarpv/gridfrequency", "0", false);
        client.publish("/home/solarpv/devicetemp", "0", false);
        client.publish("/home/solarpv/online", "0", false);
    }
    return;
}

void get_temperature()
{
    // Get the temperature from the I2C sensor

    Wire.begin();
    Wire.beginTransmission(0x48);

    Wire.write(0x01);       // Select configuration register
    Wire.write(0x60);       // Continuous conversion mode, Power-up default
    Wire.endTransmission(); // Stop I2C Transmission

    unsigned int data[2];

    Wire.beginTransmission(0x48); // Starts I2C communication
    Wire.write(0x00);             // Select data register
    Wire.endTransmission();       // Stop I2C transmission
    Wire.requestFrom(0x48, 2);    // Request 2 bytes of data

    if (Wire.available() == 2)
    { // Read 2 bytes of data, temp msb, temp lsb
        data[0] = Wire.read();
        data[1] = Wire.read();
    }

    // Convert the data to 12-bits
    int temp = ((data[0] * 256) + data[1]) / 16.0;

    if (temp > 2047) temp -= 4096;

    float ftemp = temp * 0.0625;

    Wire.write(0x01);       // Select configuration register
    Wire.write(0xe1);       // Once-shot mode, Power-down
    Wire.endTransmission(); // Stop I2C Transmission

    Wire.end();

    if (!client.connected())
    {
        connect_mqtt();
    }
    client.publish("/home/shed/temperature", String(ftemp).c_str(), false);

    return;
}

void get_light_level()
{
    float val = analogRead(32);

    uint8_t light_state = 0;

    if (!client.connected()) connect_mqtt();

    if (val < LIGHT_THRESHOLD) light_state = 1;        
    else light_state = 0;

    if (light_state != previous_light_state){
        previous_light_state = light_state;
        client.publish("/home/shed/light", String(light_state).c_str(), false);
    }
}

void IRAM_ATTR onTimer()
{
    do_timer_loop = true;
}

void setup()
{
#ifdef DEBUG
    Serial.begin(115200); // Enable hardware serial port of debug is enabled
#endif

    // enable LEDs
    pinMode(POWER_LED, OUTPUT);
    digitalWrite(POWER_LED, HIGH); // turn power LED on

    pinMode(ACTION_LED, OUTPUT);
    digitalWrite(ACTION_LED, LOW); // turn action LED off

    // Check for wifi and mqtt connections
    if (!connect_wifi())
    {
        go_to_sleep(); // wifi disconnected so sleep and reset
    }

    if (!connect_mqtt())
    {
        go_to_sleep(); // mqtt failed so sleep and reset
    }

    // Create a software serial device
    softserial.begin(BAUD_RATE, SWSERIAL_8N1, RX, TX, false);

    // Create modbus objects using the softserial port
    energymeter.begin(5, softserial);
    solarinverter.begin(2, softserial);

    // Create a timer to trigger at a set interval
    timer1 = timerBegin(0, 80, true);
    timerAttachInterrupt(timer1, &onTimer, true);
    timerAlarmWrite(timer1, TIMER_WAIT, true);
    timerAlarmEnable(timer1);
}

void loop()
{
    if (do_timer_loop)
    {
        digitalWrite(ACTION_LED, HIGH); // turn action LED on

        // Check for wifi and mqtt connections
        if (WiFi.status() != WL_CONNECTED)
        {
            go_to_sleep(); // wifi failed so sleep and reset
        }

        if (!client.connected())
        {
            go_to_sleep(); // mqtt failed so sleep and reset
        }

        get_temperature(); // Get the current temperature
        
        get_data();        // Get Modbus data from devices

        do_timer_loop = false;

        digitalWrite(ACTION_LED, LOW); // turn action LED on
    }

    get_light_level(); // Get the status of the shed light
}