#include <FS.h>
#include <EEPROM.h>
#include <DNSServer.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <WiFiManager.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <DoubleResetDetector.h>

// * Include settings
#include "settings.h"

// * Initiate Double resetDetector
DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);

// * Initiate led blinker library
Ticker ticker;

// * Initiate WIFI client
WiFiClient espClient;

// * Initiate MQTT client
PubSubClient mqtt_client(espClient);

// **********************************
// * WIFI                           *
// **********************************

// * Gets called when WiFiManager enters configuration mode
void configModeCallback(WiFiManager *myWiFiManager)
{
    Serial.println(F("Entered config mode"));
    Serial.println(WiFi.softAPIP());

    // * If you used auto generated SSID, print it
    Serial.println(myWiFiManager->getConfigPortalSSID());

    // * Entered config mode, make led toggle faster
    ticker.attach(0.2, tick);
}

// **********************************
// * Ticker (System LED Blinker)    *
// **********************************

// * Blink on-board Led
void tick()
{
    // * Toggle state
    int state = digitalRead(LED_BUILTIN);  // * Get the current state of GPIO1 pin
    digitalWrite(LED_BUILTIN, !state);     // * Set pin to the opposite state
}

// **********************************
// * MQTT                           *
// **********************************

// * Send a message to a broker topic
void send_mqtt_message(const char *topic, char *payload)
{
    bool result = mqtt_client.publish(topic, payload, false);

    if (!result)
    {
        Serial.printf("MQTT publish to topic %s failed\n", topic);
    }
}

// * Reconnect to MQTT server and subscribe to in and out topics
bool mqtt_reconnect()
{
    // * Loop until we're reconnected
    int MQTT_RECONNECT_RETRIES = 0;

    while (!mqtt_client.connected() && MQTT_RECONNECT_RETRIES < MQTT_MAX_RECONNECT_TRIES)
    {
        MQTT_RECONNECT_RETRIES++;
        Serial.printf("MQTT connection attempt %d / %d ...\n", MQTT_RECONNECT_RETRIES, MQTT_MAX_RECONNECT_TRIES);

        // * Attempt to connect
        if (mqtt_client.connect(HOSTNAME, MQTT_USER, MQTT_PASS))
        {
            Serial.println(F("MQTT connected!"));

            // * Once connected, publish an announcement...
            //char *message = new char[16 + strlen(HOSTNAME) + 1];
            //strcpy(message, "p1 meter alive: ");
            //strcat(message, HOSTNAME);
            //mqtt_client.publish("hass/status", message);
            mqtt_client.publish("hass/status", "p1 meter alive: " HOSTNAME);

            Serial.printf("MQTT root topic: %s\n", MQTT_ROOT_TOPIC);
        }
        else
        {
            Serial.print(F("MQTT Connection failed: rc="));
            Serial.println(mqtt_client.state());
            Serial.println(F(" Retrying in 5 seconds"));
            Serial.println("");

            // * Wait 5 seconds before retrying
            delay(5000);
        }
    }

    if (MQTT_RECONNECT_RETRIES >= MQTT_MAX_RECONNECT_TRIES)
    {
        Serial.printf("*** MQTT connection failed, giving up after %d tries ...\n", MQTT_RECONNECT_RETRIES);
        return false;
    }

    return true;
}

void send_metric(String name, long metric)
{
    char output[10];
    ltoa(metric, output, sizeof(output));

    String topic = String(MQTT_ROOT_TOPIC) + "/" + name;
    send_mqtt_message(topic.c_str(), output);
}

void send_data_to_broker()
{
    send_metric("consumption_active_tariff_in", CONSUMPTION_ACTIVE_TARIFF_OUT);
    send_metric("consumption_active_tariff_out", CONSUMPTION_ACTIVE_TARIFF_IN);
    send_metric("returndelivery_reactive_tariff_out", RETURNDELIVERY_REACTIVE_TARIFF_OUT);
    send_metric("returndelivery_reactive_tariff_in", RETURNDELIVERY_REACTIVE_TARIFF_IN);
    send_metric("active_consumption", ACTIVE_CONSUMPTION);
    send_metric("active_returndelivery", ACTIVE_RETURNDELIVERY);
    send_metric("reactive_consumption", REACTIVE_CONSUMPTION);
    send_metric("reactive_returndelivery", REACTIVE_RETURNDELIVERY);

    send_metric("l1_active_power_usage", L1_ACTIVE_POWER_USAGE);
    send_metric("l1_active_power_return", L1_ACTIVE_POWER_RETURN);
    send_metric("l2_active_power_usage", L2_ACTIVE_POWER_USAGE);
    send_metric("l2_active_power_return", L2_ACTIVE_POWER_RETURN);
    send_metric("l3_active_power_usage", L3_ACTIVE_POWER_USAGE);
    send_metric("l3_active_power_return", L3_ACTIVE_POWER_RETURN);

    send_metric("l1_active_power_current", L1_ACTIVE_POWER_CURRENT);
    send_metric("l2_active_power_current", L2_ACTIVE_POWER_CURRENT);
    send_metric("l3_active_power_current", L3_ACTIVE_POWER_CURRENT);

    send_metric("l1_reactive_power_usage", L1_REACTIVE_POWER_USAGE);
    send_metric("l1_reactive_power_return", L1_REACTIVE_POWER_RETURN);
    send_metric("l2_reactive_power_usage", L2_REACTIVE_POWER_USAGE);
    send_metric("l2_reactive_power_return", L2_REACTIVE_POWER_RETURN);
    send_metric("l3_reactive_power_usage", L3_REACTIVE_POWER_USAGE);
    send_metric("l3_reactive_power_return", L3_REACTIVE_POWER_RETURN);

    send_metric("l1_voltage", L1_VOLTAGE);
    send_metric("l2_voltage", L2_VOLTAGE);
    send_metric("l3_voltage", L3_VOLTAGE);
}

// **********************************
// * P1                             *
// **********************************

unsigned int CRC16(unsigned int crc, unsigned char *buf, int len)
{
	for (int pos = 0; pos < len; pos++)
    {
        crc ^= (unsigned int)buf[pos]; // * XOR byte into least sig. byte of crc
        for (int i = 8; i != 0; i--) // * Loop over each bit
        {
            // * If the LSB is set
            if ((crc & 0x0001) != 0)
            {
                // * Shift right and XOR 0xA001
                crc >>= 1;
                crc ^= 0xA001;
                // crc ^= 0x8005;
            }
            // * Else LSB is not set
            else
                // * Just shift right
                crc >>= 1;
        }
    }
    return crc;
}

bool isNumber(char *res, int len)
{
    for (int i = 0; i < len; i++)
    {
        if (((res[i] < '0') || (res[i] > '9')) && (res[i] != '.' && res[i] != 0))
            return false;
    }
    return true;
}

int FindCharInArrayRev(char array[], char c, int len)
{
    for (int i = len - 1; i >= 0; i--)
    {
        if (array[i] == c)
            return i;
    }
    return -1;
}

long getValue(char *buffer, int maxlen, char startchar, char endchar) // should have more than 4 chars.
{
    int s = FindCharInArrayRev(buffer, startchar, maxlen - 2);
    int l = FindCharInArrayRev(buffer, endchar, maxlen - 2) - s - 1;

    char res[16];
    memset(res, 0, sizeof(res));

    if (strncpy(res, buffer + s + 1, l))
    {
        if (endchar == '*')
        {
            if (isNumber(res, l))
                // * Lazy convert float to long
                return (1000 * atof(res));
        }
        else if (endchar == ')')
        {
            if (isNumber(res, l))
                return atof(res);
        }
    }
    return 0;
}

bool decode_telegram(int len)
{
    int startChar = FindCharInArrayRev(telegram, '/', len);
    int endChar = FindCharInArrayRev(telegram, '!', len);
    bool validCRCFound = false;

    Serial.print("> ");
    // debug
    // Serial.printf("\nstartchar = %d; endchar = %d; \n", startChar, endChar);

    Serial.print("telegram = __");
    for (int cnt = 0; cnt < len; cnt++)
    {
        Serial.print(telegram[cnt]);
    }
    // Serial.print("__\n");
    if (telegram[len - 1] != '\n') {
        Serial.print("\n");
    }

    if (startChar >= 0)
    {
        // debug
        // Serial.println("Branch 1");
        // * Start found. Reset CRC calculation
        currentCRC = CRC16(0x0000, (unsigned char *)telegram + startChar, len - startChar);
    }
    else if (endChar >= 0)
    {
        // * Add to crc calc
        currentCRC = CRC16(currentCRC,(unsigned char*)telegram+endChar, 1);

        char messageCRC[5];
        strncpy(messageCRC, telegram + endChar + 1, 4);

        messageCRC[4] = 0; // * Thanks to HarmOtten (issue 5)

        // debug
        //Serial.printf("\nmessageCRC = %s", messageCRC);

        validCRCFound = ((unsigned int)strtol(messageCRC, NULL, 16) == currentCRC);

        // debug
        //Serial.printf("\ncurrentCRC = %d", currentCRC);

        if (validCRCFound)
            Serial.println(F("CRC Valid!"));
        else
            Serial.println(F("CRC Invalid!"));

        currentCRC = 0;
    }
    else
    {
        // debug
        // Serial.println("Branch 3");

        currentCRC = CRC16(currentCRC, (unsigned char *)telegram, len);
    }

    // 1-0:1.8.0(00006678.394*kWh)
    // 1-0:1.8.0 = Mätarställning Aktiv Energi Uttag
    if (strncmp(telegram, "1-0:1.8.0", strlen("1-0:1.8.1")) == 0)
    {
        CONSUMPTION_ACTIVE_TARIFF_OUT = getValue(telegram, len, '(', '*');
    }

    // 1-0:2.8.0(00000000.000*kWh)
    // 1-0:2.8.0 = Mätarställning Aktiv Energi Inmatning
    if (strncmp(telegram, "1-0:2.8.0", strlen("1-0:2.8.0")) == 0)
    {
        CONSUMPTION_ACTIVE_TARIFF_IN = getValue(telegram, len, '(', '*');
    }
	
    // 1-0:3.8.0(00000021.988*kvarh)
    // 1-0:3.8.0 = Mätarställning Reaktiv Energi Uttag
    if (strncmp(telegram, "1-0:3.8.0", strlen("1-0:3.8.0")) == 0)
    {
        RETURNDELIVERY_REACTIVE_TARIFF_OUT = getValue(telegram, len, '(', '*');
    }

    // 1-0:4.8.0(00001020.971*kvarh)
    // 1-0:4.8.0 = Mätarställning Reaktiv Energi Inmatning
    if (strncmp(telegram, "1-0:4.8.0", strlen("1-0:4.8.0")) == 0)
    {
        RETURNDELIVERY_REACTIVE_TARIFF_IN = getValue(telegram, len, '(', '*');
    }

    // 1-0:1.7.0(0001.727*kW)
    // 1-0:1.7.0 = Aktiv Effekt Uttag - Momentan trefaseffekt
    if (strncmp(telegram, "1-0:1.7.0", strlen("1-0:1.7.0")) == 0)
    {
        ACTIVE_CONSUMPTION = getValue(telegram, len, '(', '*');
    }
    // 1-0:2.7.0(0000.000*kW)
    // 1-0:2.7.0 = Aktiv Effekt Inmatning - Momentan trefaseffekt
    if (strncmp(telegram, "1-0:2.7.0", strlen("1-0:2.7.0")) == 0)
    {
        ACTIVE_RETURNDELIVERY = getValue(telegram, len, '(', '*');
    }
    // 1-0:3.7.0(0000.000*kvar)
    // 1-0:3.7.0 = Reaktiv Effekt Uttag - Momentan trefaseffekt
    if (strncmp(telegram, "1-0:3.7.0", strlen("1-0:3.7.0")) == 0)
    {
        REACTIVE_CONSUMPTION = getValue(telegram, len, '(', '*');
    }
    // 1-0:4.7.0(0000.309*kvar)
    // 1-0:4.7.0 = Reaktiv Effekt Inmatning - Momentan trefaseffekt
    if (strncmp(telegram, "1-0:4.7.0", strlen("1-0:4.7.0")) == 0)
    {
        REACTIVE_RETURNDELIVERY = getValue(telegram, len, '(', '*');
    }
    // 1-0:21.7.0(0001.023*kW)
    // 1-0:21.7.0 = L1 Aktiv Effekt Uttag	- Momentan effekt
    if (strncmp(telegram, "1-0:21.7.0", strlen("1-0:21.7.0")) == 0)
    {
        L1_ACTIVE_POWER_USAGE = getValue(telegram, len, '(', '*');
    }

    // 1-0:22.7.0(0000.350*kW)
    // 1-0:22.7.0 = L1 Aktiv Effekt Inmatning	- Momentan effekt
    if (strncmp(telegram, "1-0:22.7.0", strlen("1-0:22.7.0")) == 0)
    {
        L1_ACTIVE_POWER_RETURN = getValue(telegram, len, '(', '*');
    }

    // 1-0:41.7.0(0000.353*kW)
    // 1-0:41.7.0 = L2 Aktiv Effekt Uttag - Momentan effekt
    if (strncmp(telegram, "1-0:41.7.0", strlen("1-0:41.7.0")) == 0)
    {
        L2_ACTIVE_POWER_USAGE = getValue(telegram, len, '(', '*');
    }

    // 1-0:42.7.0(0000.000*kW)
    // 1-0:42.7.0 = L2 Aktiv Effekt Inmatning	- Momentan effekt
    if (strncmp(telegram, "1-0:42.7.0", strlen("1-0:42.7.0")) == 0)
    {
        L2_ACTIVE_POWER_RETURN = getValue(telegram, len, '(', '*');
    }

    // 1-0:61.7.0(0000.000*kW)
    // 1-0:61.7.0 = L3 Aktiv Effekt Uttag - Momentan effekt
    if (strncmp(telegram, "1-0:61.7.0", strlen("1-0:61.7.0")) == 0)
    {
        L3_ACTIVE_POWER_USAGE = getValue(telegram, len, '(', '*');
    }

    // 1-0:62.7.0(0000.000*kW)
    // 1-0:62.7.0 = L3 Aktiv Effekt Inmatning -	Momentan effekt
    if (strncmp(telegram, "1-0:62.7.0", strlen("1-0:62.7.0")) == 0)
    {
        L3_ACTIVE_POWER_RETURN = getValue(telegram, len, '(', '*');
    }

    // 1-0:31.7.0(002*A)
    // 1-0:31.7.0 = L1 Fasström	- Momentant RMS-värde
    if (strncmp(telegram, "1-0:31.7.0", strlen("1-0:31.7.0")) == 0)
    {
        L1_ACTIVE_POWER_CURRENT = getValue(telegram, len, '(', '*');
    }
    // 1-0:51.7.0(002*A)
    // 1-0:51.7.0 = L2 Fasström	- Momentant RMS-värde
    if (strncmp(telegram, "1-0:51.7.0", strlen("1-0:51.7.0")) == 0)
    {
        L2_ACTIVE_POWER_CURRENT = getValue(telegram, len, '(', '*');
    }
    // 1-0:71.7.0(002*A)
    // 1-0:71.7.0 = L3 Fasström	- Momentant RMS-värde
    if (strncmp(telegram, "1-0:71.7.0", strlen("1-0:71.7.0")) == 0)
    {
        L3_ACTIVE_POWER_CURRENT = getValue(telegram, len, '(', '*');
    }

    // 1-0:23.7.0(0000.000*kvar)
    // 1-0:23.7.0 = L1 Reaktiv Effekt Uttag - Momentan effekt
    if (strncmp(telegram, "1-0:23.7.0", strlen("1-0:23.7.0")) == 0)
    {
        L1_REACTIVE_POWER_USAGE = getValue(telegram, len, '(', '*');
    }

    // 1-0:24.7.0(0000.000*kvar)
    // 1-0:24.7.0 = L1 Reaktiv Effekt Inmatning	- Momentan effekt
    if (strncmp(telegram, "1-0:24.7.0", strlen("1-0:24.7.0")) == 0)
    {
        L1_REACTIVE_POWER_RETURN = getValue(telegram, len, '(', '*');
    }

    // 1-0:43.7.0(0000.000*kvar)
    // 1-0:43.7.0 = L2 Reaktiv Effekt Uttag - Momentan effekt
    if (strncmp(telegram, "1-0:43.7.0", strlen("1-0:43.7.0")) == 0)
    {
        L2_REACTIVE_POWER_USAGE = getValue(telegram, len, '(', '*');
    }

    // 1-0:44.7.0(0000.009*kvar)
    // 1-0:44.7.0 = L2 Reaktiv Effekt Inmatning	- Momentan effekt
    if (strncmp(telegram, "1-0:44.7.0", strlen("1-0:44.7.0")) == 0)
    {
        L2_REACTIVE_POWER_RETURN = getValue(telegram, len, '(', '*');
    }

    // 1-0:63.7.0(0000.161*kvar)
    // 1-0:63.7.0 = L3 Reaktiv Effekt Uttag - Momentan effekt
    if (strncmp(telegram, "1-0:63.7.0", strlen("1-0:63.7.0")) == 0)
    {
        L3_REACTIVE_POWER_USAGE = getValue(telegram, len, '(', '*');
    }

    // 1-0:64.7.0(0000.138*kvar)
    // 1-0:64.7.0 = L3 Reaktiv Effekt Inmatning - Momentan effekt
    if (strncmp(telegram, "1-0:64.7.0", strlen("1-0:64.7.0")) == 0)
    {
        L3_REACTIVE_POWER_RETURN = getValue(telegram, len, '(', '*');
    }

    // 1-0:32.7.0(240.3*V)
    // 1-0:32.7.0 = L1 Fasspänning - Momentant RMS-värde
    if (strncmp(telegram, "1-0:32.7.0", strlen("1-0:32.7.0")) == 0)
    {
        L1_VOLTAGE = getValue(telegram, len, '(', '*');
    }
    // 1-0:52.7.0(240.1*V)
    // 1-0:52.7.0 = L2 Fasspänning - Momentant RMS-värde
    if (strncmp(telegram, "1-0:52.7.0", strlen("1-0:52.7.0")) == 0)
    {
        L2_VOLTAGE = getValue(telegram, len, '(', '*');
    }   
    // 1-0:72.7.0(241.3*V)
    // 1-0:72.7.0 = L3 Fasspänning - Momentant RMS-värde
    if (strncmp(telegram, "1-0:72.7.0", strlen("1-0:72.7.0")) == 0)
    {
        L3_VOLTAGE = getValue(telegram, len, '(', '*');
    }

    return validCRCFound;
}

void read_p1_hardwareserial()
{
    if (Serial.available())
    {
        memset(telegram, 0, sizeof(telegram));

        while (Serial.available())
        {
            // ESP.wdtDisable();
            int len = Serial.readBytesUntil('\n', telegram, P1_MAXLINELENGTH);

            // ESP.wdtEnable(1);

            // debug entire telegram
            // Serial.printf(">>> %s\n", telegram);

            processLine(len);
            memset(telegram, 0, sizeof(telegram));
        }
    }
}

void processLine(int len)
{
    telegram[len] = '\n';
    telegram[len + 1] = 0;
    yield();

    bool result = decode_telegram(len + 1);
    if (result) 
    {
        send_data_to_broker();
        LAST_UPDATE_SENT = millis();
    }
}

// **********************************
// * EEPROM helpers                 *
// **********************************

String read_eeprom(int offset, int len)
{
    Serial.print(F("read_eeprom()"));

    String res = "";
    for (int i = 0; i < len; ++i)
    {
        res += char(EEPROM.read(i + offset));
    }
    return res;
}

void write_eeprom(int offset, int len, String value)
{
    Serial.println(F("write_eeprom()"));
    for (int i = 0; i < len; ++i)
    {
        if ((unsigned)i < value.length())
        {
            EEPROM.write(i + offset, value[i]);
        }
        else
        {
            EEPROM.write(i + offset, 0);
        }
    }
}

// ******************************************
// * Callback for saving WIFI config        *
// ******************************************

bool shouldSaveConfig = false;

// * Callback notifying us of the need to save config
void save_wifi_config_callback()
{
    Serial.println(F("Should save config"));
    shouldSaveConfig = true;
}

// **********************************
// * Setup OTA                      *
// **********************************

// ******************************************
// * Callback for resetting Wifi settings   *
// ******************************************
void resetWifi()
{
    Serial.println("RST was pushed twice...");
    Serial.println("Erasing stored WiFi credentials.");

    // clear WiFi creds.
    WiFiManager wifiManager;
    wifiManager.resetSettings();

    Serial.println("Restarting...");
    ESP.restart(); // builtin, safely restarts the ESP.
}

void setup_ota()
{
    Serial.println(F("Arduino OTA activated."));

    // * Port defaults to 8266
    ArduinoOTA.setPort(8266);

    // * Set hostname for OTA
    ArduinoOTA.setHostname(HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);

    ArduinoOTA.onStart([]()
    {
        Serial.println(F("Arduino OTA: Start"));
    });

    ArduinoOTA.onEnd([]()
    {
        Serial.println(F("Arduino OTA: End (Running reboot)"));
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
    {
        Serial.printf("Arduino OTA Progress: %u%%\r", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](ota_error_t error)
    {
        Serial.printf("Arduino OTA Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
            Serial.println(F("Arduino OTA: Auth Failed"));
        else if (error == OTA_BEGIN_ERROR)
            Serial.println(F("Arduino OTA: Begin Failed"));
        else if (error == OTA_CONNECT_ERROR)
            Serial.println(F("Arduino OTA: Connect Failed"));
        else if (error == OTA_RECEIVE_ERROR)
            Serial.println(F("Arduino OTA: Receive Failed"));
        else if (error == OTA_END_ERROR)
            Serial.println(F("Arduino OTA: End Failed"));
    });

    ArduinoOTA.begin();
    Serial.println(F("Arduino OTA finished"));
}

// **********************************
// * Setup MDNS discovery service   *
// **********************************

void setup_mdns()
{
    Serial.println(F("Starting MDNS responder service"));

    bool mdns_result = MDNS.begin(HOSTNAME);
    if (mdns_result)
    {
        MDNS.addService("http", "tcp", 80);
    }
}

// **********************************
// * Setup Main                     *
// **********************************

void setup()
{
    // * Configure EEPROM
    EEPROM.begin(512);

    // Setup a hw serial connection for communication with the P1 meter and logging (not using inversion)
    Serial.begin(BAUD_RATE, SERIAL_8N1, SERIAL_FULL);
    Serial.setRxBufferSize(1024);
    Serial.println("");
    //Serial.println("Swapping UART0 RX to inverted");
    Serial.flush();

#if SERIAL_RX_INVERTED
    // Invert the RX serialport by setting a register value, this way the TX might continue normally allowing the serial monitor to read println's
    Serial.println("Swapping UART0 RX to inverted");
    USC0(UART0) = USC0(UART0) | BIT(UCRXI);
#endif

    Serial.println("Serial port is ready to recieve.");

    // * Set led pin as output
    pinMode(LED_BUILTIN, OUTPUT);

    // * Setup Double reset detection
    if (drd.detectDoubleReset())
    {
        Serial.println("DRD: Double Reset Detected");
        Serial.println("DRD: RESET WIFI Initiated");
        resetWifi();
    }
    else
    {
        Serial.println("DRD: No Double Reset Detected");
    }

    // * Start ticker with 0.5 because we start in AP mode and try to connect
    ticker.attach(0.6, tick);

    // * Get MQTT Server settings
    String settings_available = read_eeprom(134, 1);

    if (settings_available == "1")
    {
        read_eeprom(0, 64).toCharArray(MQTT_HOST, 64);   // * 0-63
        read_eeprom(64, 6).toCharArray(MQTT_PORT, 6);    // * 64-69
        read_eeprom(70, 32).toCharArray(MQTT_USER, 32);  // * 70-101
        read_eeprom(102, 32).toCharArray(MQTT_PASS, 32); // * 102-133
    }
    WiFi.hostname(HOSTNAME);

    WiFiManagerParameter CUSTOM_MQTT_HOST("host", "MQTT hostname", MQTT_HOST, 64);
    WiFiManagerParameter CUSTOM_MQTT_PORT("port", "MQTT port", MQTT_PORT, 6);
    WiFiManagerParameter CUSTOM_MQTT_USER("user", "MQTT user", MQTT_USER, 32);
    WiFiManagerParameter CUSTOM_MQTT_PASS("pass", "MQTT pass", MQTT_PASS, 32);

    // * WiFiManager local initialization. Once its business is done, there is no need to keep it around
    WiFiManager wifiManager;

    // * Reset settings - uncomment for testing
    // wifiManager.resetSettings();

    // * Set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
    wifiManager.setAPCallback(configModeCallback);

    // * Set timeout
    wifiManager.setConfigPortalTimeout(WIFI_TIMEOUT);

    // * Set save config callback
    wifiManager.setSaveConfigCallback(save_wifi_config_callback);

    // * Add all your parameters here
    wifiManager.addParameter(&CUSTOM_MQTT_HOST);
    wifiManager.addParameter(&CUSTOM_MQTT_PORT);
    wifiManager.addParameter(&CUSTOM_MQTT_USER);
    wifiManager.addParameter(&CUSTOM_MQTT_PASS);

    // * Fetches SSID and pass and tries to connect
    // * Reset when no connection after 10 seconds
    if (!wifiManager.autoConnect())
    {
        Serial.println(F("Failed to connect to WIFI and hit timeout"));

        // * Reset and try again, or maybe put it to deep sleep
        ESP.reset();
        delay(WIFI_TIMEOUT);
    }

    // * Read updated parameters
    strcpy(MQTT_HOST, CUSTOM_MQTT_HOST.getValue());
    strcpy(MQTT_PORT, CUSTOM_MQTT_PORT.getValue());
    strcpy(MQTT_USER, CUSTOM_MQTT_USER.getValue());
    strcpy(MQTT_PASS, CUSTOM_MQTT_PASS.getValue());

    // * Save the custom parameters to FS
    if (shouldSaveConfig)
    {
        Serial.println(F("Saving WiFiManager config"));

        write_eeprom(0, 64, MQTT_HOST);   // * 0-63
        write_eeprom(64, 6, MQTT_PORT);   // * 64-69
        write_eeprom(70, 32, MQTT_USER);  // * 70-101
        write_eeprom(102, 32, MQTT_PASS); // * 102-133
        write_eeprom(134, 1, "1");        // * 134 --> always "1"
        EEPROM.commit();
    }

    // * If you get here you have connected to the WiFi
    Serial.println(F("Connected to WIFI..."));

    // * Keep LED on
    ticker.detach();
    digitalWrite(LED_BUILTIN, LOW);

    // * Configure OTA
    setup_ota();

    // * Startup MDNS Service
    setup_mdns();

    // * Setup MQTT
    Serial.printf("MQTT connecting to: %s:%s\n", MQTT_HOST, MQTT_PORT);

    mqtt_client.setServer(MQTT_HOST, atoi(MQTT_PORT));

}

// **********************************
// * Loop                           *
// **********************************

void loop()
{
    // every 2days --> restart board.
    const unsigned long RESTART_INTERVAL = 2ul * 24ul * 60ul * 60ul * 1000ul;
    if (millis() > RESTART_INTERVAL)
    {
        Serial.println("++++++++++++++++++++++++++++++++++++++++");
        Serial.print("trying to restart after 2 days or ms: ");
        Serial.print(RESTART_INTERVAL);
        Serial.println("");
        Serial.println("++++++++++++++++++++++++++++++++++++++++");
        ESP.restart();
    }

    ArduinoOTA.handle();
    long now = millis();

    if (!mqtt_client.connected())
    {
        if (now - LAST_RECONNECT_ATTEMPT > 5000)
        {
            LAST_RECONNECT_ATTEMPT = now;

            if (mqtt_reconnect())
            {
                LAST_RECONNECT_ATTEMPT = 0;
            }
        }
    }
    else
    {
        // Serial.println("start mqtt Loop");
        mqtt_client.loop();
        // Serial.println("End mqtt Loop");
    }
    
    if (now - LAST_UPDATE_SENT > UPDATE_INTERVAL || LAST_UPDATE_SENT == 0) {
        read_p1_hardwareserial();

    // //debug
    // Serial.println("End Loop");
    // Serial.println("--------");

    // Call the double reset detector loop method every so often,
    // so that it can recognise when the timeout expires.
    // You can also call drd.stop() when you wish to no longer
    // consider the next reset as a double reset.
    drd.loop();
    }
}
