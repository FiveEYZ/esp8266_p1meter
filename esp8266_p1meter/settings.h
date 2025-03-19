// **********************************
// * Settings                       *
// **********************************

//Reset wifi settings:
// *Number of seconds after reset during which a
// *subseqent reset will be considered a double reset.
#define DRD_TIMEOUT 10

// *RTC Memory Address for the DoubleResetDetector to use
#define DRD_ADDRESS 1

// Update treshold in milliseconds, messages will only be sent on this interval
#define UPDATE_INTERVAL 30000 // 30s
//#define UPDATE_INTERVAL 300000 // 5 minutes

// * Baud rate for both hardware and software 
#define BAUD_RATE 115200

// * Change to 0 if you have a hardware inverter
#define SERIAL_RX_INVERTED 1

// The used serial pins, note that this can only be UART0, as other serial port doesn't support inversion
// By default the UART0 serial will be used. These settings displayed here just as a reference. 
// #define SERIAL_RX RX
// #define SERIAL_TX TX

// * Max telegram length
#define P1_MAXLINELENGTH 2048 // 4096 longest normal line is 47 char (+3 for \r\n\0)

// * The hostname of our little creature
#define HOSTNAME "p1meter"

// * The password used for OTA
#define OTA_PASSWORD "password"

// * Wifi timeout in milliseconds
#define WIFI_TIMEOUT 30000

// * MQTT network settings
#define MQTT_MAX_RECONNECT_TRIES 10

// * MQTT root topic
#define MQTT_ROOT_TOPIC "sensors/power/p1meter"

// * MQTT Last reconnection counter
long LAST_RECONNECT_ATTEMPT = 0;

long LAST_UPDATE_SENT = 0;

// * To be filled with EEPROM data
char MQTT_HOST[64] = "";
char MQTT_PORT[6] = "";
char MQTT_USER[32] = "";
char MQTT_PASS[32] = "";

// * Set to store received telegram
char telegram[P1_MAXLINELENGTH];

// * Set to store the data values read
long CONSUMPTION_ACTIVE_TARIFF_OUT;
long CONSUMPTION_ACTIVE_TARIFF_IN;

long RETURNDELIVERY_REACTIVE_TARIFF_OUT;
long RETURNDELIVERY_REACTIVE_TARIFF_IN;

long ACTIVE_CONSUMPTION;
long ACTIVE_RETURNDELIVERY;
long REACTIVE_CONSUMPTION;
long REACTIVE_RETURNDELIVERY;

long L1_ACTIVE_POWER_USAGE;
long L1_ACTIVE_POWER_RETURN;
long L2_ACTIVE_POWER_USAGE;
long L2_ACTIVE_POWER_RETURN;
long L3_ACTIVE_POWER_USAGE;
long L3_ACTIVE_POWER_RETURN;

long L1_ACTIVE_POWER_CURRENT;
long L2_ACTIVE_POWER_CURRENT;
long L3_ACTIVE_POWER_CURRENT;

long L1_REACTIVE_POWER_USAGE;
long L1_REACTIVE_POWER_RETURN;
long L2_REACTIVE_POWER_USAGE;
long L2_REACTIVE_POWER_RETURN;
long L3_REACTIVE_POWER_USAGE;
long L3_REACTIVE_POWER_RETURN;

long L1_VOLTAGE;
long L2_VOLTAGE;
long L3_VOLTAGE;

// * Set during CRC checking
unsigned int currentCRC = 0;
