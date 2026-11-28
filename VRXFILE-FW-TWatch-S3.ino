/**
 * Alternative firmware for LilyGO T-Watch S3 Plus (VRXFILE-FW-TWatch-S3.ino)
 * Author: Rostislav Varzar
 * Email: vrxfile@mail.ru, vrxfile1987@gmail.com
 * The entire program is contained in a single .ino file.
 *
 * Required libraries (install via Arduino IDE Library Manager):
 * 1. LovyanGFX (by lovyan03)
 * 2. TinyGPSPlus (by Mikal Hart)
 * 3. SensorLib (by Lewis He) - optional, enables Bosch BMA423 hardware pedometer microcode
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <time.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <esp_idf_version.h>
#include <lwip/etharp.h>
#include <lwip/netif.h>
#include <lwip/tcpip.h>
#include <lwip/ip4_addr.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLEHIDDevice.h>
#if ESP_IDF_VERSION_MAJOR >= 5
#include <driver/i2s_std.h>
#include <driver/i2s_pdm.h>
#else
#include <driver/i2s.h>
#endif
#include <LovyanGFX.hpp>
#include <TinyGPS++.h>
#if __has_include(<SensorBMA423.hpp>)
#define HAS_SENSORLIB_BMA423 1
#include <SensorBMA423.hpp>
#else
#define HAS_SENSORLIB_BMA423 0
#endif
#include <math.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

// Firmware, device, and author information
#define FW_NAME             "VRXFILE-FW-TWatch-S3"
#define FW_VERSION_MAJOR    2
#define FW_VERSION_MINOR    0
#define FW_VERSION_PATCH    0
#define DEVICE_NAME         "LILYGO T-Watch S3 Plus"

#define FW_AUTHOR         "Rostislav Varzar"
#define FW_AUTHOR_EMAIL   "vrxfile@mail.ru"
#define FW_SUPPORT_EMAIL  "vrxfile1987@gmail.com"

// WeatherAPI key and NOAA SWPC base URL
#define WEATHER_API_KEY     "fe7c9af56ea44415a60113506262211"
#define NOAA_SWPC_BASE_URL  "https://services.swpc.noaa.gov/products/"

// ============================================================================
// 1. PIN DEFINITIONS (per T-Watch S3 Plus schematics)
// ============================================================================

// System I2C0 bus (AXP2101 PMU, PCF8563 RTC, BMA423, DRV2605L)
#define I2C_SDA_PIN         10
#define I2C_SCL_PIN         11

// Touchscreen I2C1 bus (FT6236 / FT6336U)
#define TOUCH_SDA_PIN       39
#define TOUCH_SCL_PIN       40
#define TOUCH_INT_PIN       16

// ST7789V SPI display pins (240x240)
#define LCD_MOSI_PIN        13
#define LCD_SCLK_PIN        18
#define LCD_DC_PIN          38
#define LCD_CS_PIN          12
#define LCD_BL_PIN          45

// Active mode CPU frequency (80 MHz for reduced power consumption)
#define ACTIVE_CPU_FREQ_MHZ 80

// Wake-up interrupt pins for ESP32-S3 Light Sleep (active LOW with INPUT_PULLUP)
#define BMA423_INT_PIN      14  // BMA423 hardware INT1 line (IO14)
#define WAKE_BUTTON_PIN     0   // User button / BOOT pin (IO0)

// MIA-M10Q GPS module pins (fixed 38400 baud)
#define GPS_RX_PIN          41  // Configured as UART RX
#define GPS_TX_PIN          42  // Configured as UART TX
#define GPS_PPS_PIN         0   // TIMEPULSE / PPS line (shared with IO0 wake button)

// IR LED IR12-21C pin (via Q15 MMBT3904 transistor) and LEDC hardware channel
#define IR_SEND_PIN         2
#define IR_LEDC_MODE        LEDC_LOW_SPEED_MODE
#define IR_LEDC_TIMER       LEDC_TIMER_1
#define IR_LEDC_CHANNEL     LEDC_CHANNEL_2
#define IR_LEDC_DUTY_RES    LEDC_TIMER_8_BIT

// PDM microphone (SPM1423HM4H-B) and I2S speaker amplifier (MAX98357A) pins
#define MIC_PDM_CLK_PIN     44
#define MIC_PDM_DATA_PIN    47
#define SPK_I2S_BCK_PIN     48
#define SPK_I2S_WS_PIN      15
#define SPK_I2S_DOUT_PIN    46

// I2C device addresses
#define AXP2101_ADDR        0x34
#define PCF8563_ADDR        0x51
#define FT6336_ADDR         0x38
#define DRV2605_ADDR        0x5A
#define BMA423_ADDR_LOW     0x18
#define BMA423_ADDR_HIGH    0x19

// FT6336U power-saving register definitions (Active / Monitor / Hibernate)
#define FT6336_REG_CTRL             0x86
#define FT6336_REG_TIMEENTERMONITOR 0x87
#define FT6336_REG_PERIODACTIVE     0x88
#define FT6336_REG_PERIODMONITOR    0x89
#define FT6336_REG_G_MODE           0xA4
#define FT6336_REG_PMODE            0xA5
#define FT6336_PMODE_ACTIVE         0x00
#define FT6336_PMODE_MONITOR        0x01
#define FT6336_PMODE_HIBERNATE      0x03

// BMA423 hardware pedometer & INT1 interrupt register definitions
#define BMA423_REG_INT_STATUS_0     0x1C
#define BMA423_REG_STEP_COUNTER_0   0x1E
#define BMA423_REG_INTERNAL_STATUS  0x2A
#define BMA423_REG_INT1_IO_CTRL     0x53
#define BMA423_REG_INT_LATCH        0x55
#define BMA423_REG_INT_MAP_1        0x56
#define BMA423_REG_PWR_CONF         0x7C
#define BMA423_REG_PWR_CTRL         0x7D
#define BMA423_REG_CMD              0x7E

// Backlight brightness levels (0..255)
#define BRIGHTNESS_FULL     240
#define BRIGHTNESS_HALF     120 // Dimmed by half after 10 seconds
#define BRIGHTNESS_OFF      0   // Turn off after 30 seconds
#define BRIGHTNESS_MIN_PCT  5   // Minimum allowed display brightness (5%)
#define BRIGHTNESS_MIN_PWM  13  // Minimum backlight PWM (~5% of 255), preventing zero active brightness

// Backlight timeouts (in milliseconds)
#define TIMEOUT_DIM_MS      10000UL // 10 seconds to half brightness
#define TIMEOUT_OFF_MS      30000UL // 30 seconds to screen off

// Double/triple tap wake timing parameters
#define DOUBLE_TAP_MIN_MS   60UL
#define DOUBLE_TAP_MAX_MS   450UL

// ============================================================================
// 2. LOVYANGFX DISPLAY CONFIGURATION INSIDE .INO FILE
// ============================================================================

class LGFX_TWatchS3 : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789  _panel_instance;
  lgfx::Bus_SPI       _bus_instance;
  lgfx::Light_PWM     _light_instance;

public:
  LGFX_TWatchS3(void) {
    { // SPI bus configuration
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 40000000;
      cfg.freq_read  = 16000000;
      cfg.spi_3wire  = true;
      cfg.use_lock   = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = LCD_SCLK_PIN;
      cfg.pin_mosi = LCD_MOSI_PIN;
      cfg.pin_miso = -1;
      cfg.pin_dc   = LCD_DC_PIN;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    { // ST7789 240x240 panel configuration
      auto cfg = _panel_instance.config();
      cfg.pin_cs           = LCD_CS_PIN;
      cfg.pin_rst          = -1;
      cfg.pin_busy         = -1;
      cfg.panel_width      = 240;
      cfg.panel_height     = 240;
      cfg.offset_x         = 0;
      cfg.offset_y         = 0;
      cfg.offset_rotation  = 2;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits  = 1;
      cfg.readable         = false;
      cfg.invert           = true;
      cfg.rgb_order        = false;
      cfg.dlen_16bit       = false;
      cfg.bus_shared       = false;
      _panel_instance.config(cfg);
    }

    { // Backlight PWM configuration (GPIO45 -> Q14 MMBT3904)
      auto cfg = _light_instance.config();
      cfg.pin_bl      = LCD_BL_PIN;
      cfg.invert      = false;
      cfg.freq        = 12000;
      cfg.pwm_channel = 7;
      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);
    }

    setPanel(&_panel_instance);
  }
};

LGFX_TWatchS3 tft;
LGFX_Sprite canvas(&tft); // Full-screen offscreen buffer (sprite) for flicker-free rendering

// ============================================================================
// 3. DATA STRUCTURES AND APPLICATION STATE
// ============================================================================

// UI screens
enum ScreenState {
  SCREEN_MAIN_MENU = 0,
  SCREEN_CLOCK     = 1,
  SCREEN_GPS       = 2,
  SCREEN_CALC      = 3,
  SCREEN_EARTH     = 4,
  SCREEN_STEPS     = 5,
  SCREEN_IR        = 6,
  SCREEN_CALENDAR  = 7,
  SCREEN_INCLINOMETER = 8,
  SCREEN_WIFI_CONNECT = 9,
  SCREEN_WIFI_LIST    = 10,
  SCREEN_SETTINGS     = 11,
  SCREEN_ASCII        = 12,
  SCREEN_TEMP         = 13,
  SCREEN_PERIODIC     = 14,
  SCREEN_E_ADDITIVES  = 15,
  SCREEN_WIFI_ANALYZER = 16,
  SCREEN_ABOUT_FW     = 17,
  SCREEN_WIFI_INFO    = 18,
  SCREEN_WIFI_JAM     = 19,
  SCREEN_ACCEL        = 20,
  SCREEN_SOUND_SPEC   = 21,
  SCREEN_SOUND_GEN    = 22,
  SCREEN_HOSTS_SCAN   = 23,
  SCREEN_BT_SCAN      = 24,
  SCREEN_SUN_ACTIVITY = 25,
  SCREEN_WEATHER      = 26,
  SCREEN_SYSTEM_INFO  = 27,
  SCREEN_ALARM        = 28,
  SCREEN_STOPWATCH    = 29,
  SCREEN_TIMER        = 30,
  SCREEN_FLASHLIGHT   = 31,
  SCREEN_SOS          = 32,
  SCREEN_MORSE        = 33,
  SCREEN_TETRIS       = 34,
  SCREEN_PAINT        = 35,
  SCREEN_ACCEL_SPEC   = 36,
  SCREEN_BT_KEYBOARD  = 37,
  SCREEN_BT_MOUSE     = 38,
  SCREEN_BT_PRESENTER = 39,
  SCREEN_COLOR_TABLE    = 40,
  SCREEN_RESISTOR_TABLE = 41,
  SCREEN_VOLTAGE_DIVIDER = 42
};

// Backlight states
enum BacklightState {
  BL_STATE_FULL = 0,
  BL_STATE_HALF = 1,
  BL_STATE_OFF  = 2
};

// Selectable fields for editing in Clock screen
enum ClockEditField {
  FIELD_HOUR  = 0,
  FIELD_MIN   = 1,
  FIELD_SEC   = 2,
  FIELD_DAY   = 3,
  FIELD_MONTH = 4,
  FIELD_YEAR  = 5,
  FIELD_DOW   = 6,
  FIELD_UTC   = 7
};

// IR transmission protocols
enum IrProtocol {
  IR_PROTO_NEC = 0,
  IR_PROTO_RC5 = 1,
  IR_PROTO_RC6 = 2
};

// Date and time structure
struct DateTimeRTC {
  uint8_t hour;       // 0..23
  uint8_t minute;     // 0..59
  uint8_t second;     // 0..59
  uint8_t day;        // 1..31
  uint8_t month;      // 1..12
  uint16_t year;      // 2000..2099
  uint8_t dayOfWeek;  // 0..6 (0 = Sun, 1 = Mon, ..., 6 = Sat)
};

// Continent label coordinate structure on 3D Earth
struct ContinentLabel {
  const char* name;
  float latDeg;
  float lonDeg;
};

// City coordinate structure on 3D Earth with label offsets
struct CityMarker {
  const char* name;
  float latDeg;
  float lonDeg;
  int8_t labelOffX;
  int8_t labelOffY;
};

// Global state variables
ScreenState currentScreen  = SCREEN_MAIN_MENU;
BacklightState blState     = BL_STATE_FULL;
#define MAIN_MENU_PAGE_COUNT 5
uint8_t mainMenuPage       = 0;                           // Current main menu page (0..4)

DateTimeRTC rtcTime        = {12, 0, 0, 21, 11, 2026, 6}; // Current time read from RTC
DateTimeRTC editTime       = {12, 0, 0, 21, 11, 2026, 6}; // Editing buffer
int8_t utcOffset           = 3;                           // Time zone (default UTC+3)
bool isEditingClock        = false;                       // Edit mode flag
ClockEditField activeField = FIELD_HOUR;                  // Selected field with underline

uint8_t batteryPercent     = 100;                         // Battery charge level (%)
bool batteryCharging       = false;                       // Active battery charging flag (AXP2101)
uint32_t lastRtcReadMs     = 0;                           // RTC polling timer (1 Hz)
uint32_t lastTouchMs       = 0;                           // Timestamp of last screen touch
uint32_t firstWakeTapMs    = 0;                           // Timestamp of first tap in wake sequence
uint32_t lastWakeTapMs     = 0;                           // Timestamp of last tap in wake sequence
uint8_t wakeTapCounter     = 0;                           // Tap counter for wake gesture (2x or 3x)
bool waitingSecondTap      = false;                       // Waiting for repeat tap
bool touchWasPressed       = false;                       // Finger hold flag (debounce)

// State variables for Calendar screen
uint8_t calMonth           = 11;                          // Displayed month (1..12)
uint16_t calYear           = 2026;                        // Displayed year (2000..2099)

// Month and weekday names for Calendar screen
const char* CAL_MONTH_NAMES[12] = {
  "January", "February", "March", "April", "May", "June",
  "July", "August", "September", "October", "November", "December"
};
const char* CAL_WEEKDAYS[7] = {"Mo", "Tu", "We", "Th", "Fr", "Sa", "Su"};

// Objects and variables for GPS operation
HardwareSerial GPSSerial(1);
TinyGPSPlus gps;
TinyGPSCustom gnggaHdop(gps, "GNGGA", 8);                 // HDOP field from $GNGGA sentence
uint32_t currentGpsBaud    = 38400;                       // Fixed GPS UART baud rate (38400)
uint32_t lastGpsByteMs     = 0;                           // Timestamp of last byte received over UART

// Multi-page GPS screen state (0=data, 1=satellite skyplot, 2=SNR bar chart)
#define GPS_PAGE_COUNT     3
#define GPS_MAX_SATS       24
#define GPS_SAT_STALE_MS   8000UL

struct GpsSatelliteInfo {
  uint8_t prn;          // Satellite PRN / SV ID
  char system;          // 'G'=GPS, 'R'=GLONASS, 'E'=Galileo, 'B'=BeiDou
  int8_t elevation;     // Elevation angle (0..90 deg)
  int16_t azimuth;      // Azimuth angle (0..359 deg clockwise from north)
  uint8_t snr;          // Signal-to-noise ratio C/N0 (0..99 dB-Hz)
  bool usedInFix;       // Used in current navigation fix (from $xxGSA)
  uint32_t lastSeenMs;  // Timestamp of last update
};

uint8_t gpsPage                  = 0;
GpsSatelliteInfo gpsSats[GPS_MAX_SATS];
uint8_t gpsSatCount              = 0;
uint8_t gpsFixPrns[12]           = {0};
uint8_t gpsFixPrnCount           = 0;
char gpsNmeaLine[104]            = {0};
uint8_t gpsNmeaPos               = 0;

// State variables for Calc screen (5-page scientific calculator + base converter)
bool calcSecondPage        = false;                       // Extended function pages flag
uint8_t calcPage           = 0;                           // Page index: 0 = basic, 1 = functions 1, 2 = functions 2, 3 = inv hyperbolics, 4 = base converter
char calcDisplayStr[48]    = "0";                         // Input/result string (up to 12 digits and complex numbers)
double calcStoredVal       = 0.0;                         // Stored first operand
char calcPendingOp         = 0;                           // Pending binary operation ('+', '-', '*', '/', '^', 'r')
bool calcNewInput          = true;                        // Start new number entry flag
bool calcError             = false;                       // Math error flag
int8_t calcInfSign         = 0;                           // 0 = normal number, +1 = +inf, -1 = -inf
uint8_t calcBaseMode       = 10;                          // Active number base on page 5: 2=BIN, 8=OCT, 10=DEC, 16=HEX
char calcBaseStr[48]       = "0";                         // Number string on page 5 (supports fractional radix point)
double calcBaseExactVal    = 0.0;                         // Exact value preserved across lossless base switches
bool calcBaseError         = false;                       // Overflow or parse error flag on page 5

// State variables for Earth / Satellites screen (3D Earth, Kinosputnik & ISS)
float earthLon             = 0.35f;                       // Earth longitude rotation angle (left-right)
float earthLat             = 0.30f;                       // Earth latitude tilt angle (up-down)
int16_t prevTouchX         = -1;                          // Previous touch X coordinate during drag
int16_t prevTouchY         = -1;                          // Previous touch Y coordinate during drag
bool earthDragging         = false;                       // Active drag flag for smooth FPS
bool earthShowSatellite    = false;                       // Show satellite orbits and markers (S button)

// State variables for Step counter screen (BMA423 accelerometer & hardware pedometer)
uint8_t bma423Addr         = 0;                           // Detected BMA423 I2C address (0x18 or 0x19)
bool bma423Ready           = false;                       // BMA423 initialization success flag
bool bma423HwPedometerReady = false;                      // BMA423 internal ASIC hardware step counter active
#define STEP_SAVE_INTERVAL_MS 600000UL                    // Auto-save step count to Flash every 10 minutes
uint32_t stepCount         = 0;                           // Total step count
uint32_t stepCountBase     = 0;                           // Flash-restored base offset added to BMA423 HW counter
uint32_t bma423HwStepZeroOffset = 0;                      // Zero-offset subtracted from raw 0x1E..0x21 after reset
uint32_t bma423LastHwSteps = 0;                           // Last delta step count read from BMA423 0x1E..0x21
uint32_t lastHwStepSyncMs  = 0;                           // 1 Hz hardware step counter read timer
uint32_t lastSavedStepCount = 0;                          // Last step count persisted to Flash
uint32_t lastStepSaveMs    = 0;                           // Step count auto-save timer (10 minutes)
uint32_t lastStepPollMs    = 0;                           // Accelerometer polling timer (50 Hz)
uint32_t lastStepDetectMs  = 0;                           // Timestamp of last detected step
float stepBaselineMag      = 1.0f;                        // Moving average of gravity magnitude (g)
float stepFilteredMag      = 1.0f;                        // Smoothed acceleration vector magnitude (g)
bool stepPeakActive        = false;                       // Step detector hysteresis flag
#if HAS_SENSORLIB_BMA423
SensorBMA423 bma423Sensor;
#endif
void saveStepCountToFlash(bool force);
void configureBma423WakeInterrupt();
uint32_t readBma423HardwareSteps();
void syncBma423HardwareSteps();
void resetBma423HardwareSteps();

// State variables for Inclinometer screen (BMA423 bubble level)
float inclFiltGx           = 0.0f;                        // Smoothed X-axis acceleration (g)
float inclFiltGy           = 0.0f;                        // Smoothed Y-axis acceleration (g)
float inclFiltGz           = 1.0f;                        // Smoothed Z-axis acceleration (g)
float inclAngleX           = 0.0f;                        // Tilt angle along X axis (deg)
float inclAngleY           = 0.0f;                        // Tilt angle along Y axis (deg)
uint32_t lastInclDrawMs    = 0;                           // Smooth refresh timer for Inclinometer screen

// Structures and state variables for Wi-Fi connect (scan, password keyboard, LIFO 64 Flash, auto-connect)
#define WIFI_SCAN_MAX        32
#define WIFI_SAVED_MAX       64
#define WIFI_VISIBLE_ROWS    5
#define WIFI_AUTO_CHECK_MS   60000UL
#define WIFI_CONNECT_TIMEOUT 10000UL

enum WifiConnectView {
  WIFI_VIEW_SCANNING   = 0,
  WIFI_VIEW_LIST       = 1,
  WIFI_VIEW_PASSWORD   = 2,
  WIFI_VIEW_CONNECTING = 3,
  WIFI_VIEW_RESULT     = 4
};

struct WifiScanItem {
  char ssid[33];
  int32_t rssi;
  bool isLocked;
};

struct SavedWifiCred {
  char ssid[33];
  char pass[65];
};

Preferences wifiPrefs;
WifiConnectView wifiView      = WIFI_VIEW_LIST;
WifiScanItem wifiScanList[WIFI_SCAN_MAX];
uint8_t wifiScanCount         = 0;
uint8_t wifiScrollOffset      = 0;
char wifiSelectedSsid[33]     = "";
bool wifiSelectedLocked       = false;
char wifiPasswordBuf[65]      = "";
uint8_t wifiKbPage            = 0;                        // Keyboard page 0..3
bool wifiKbShift              = false;                    // Letter case (false = lowercase, true = uppercase)
bool wifiConnectSuccess       = false;                    // Connection result for OK screen
uint32_t wifiConnectStartMs   = 0;                        // Wi-Fi connection timeout timer
uint32_t lastWifiAutoCheckMs  = 0;                        // Background auto-connect check timer (1 min)
bool wifiBgScanning           = false;                    // Asynchronous background scan flag
SavedWifiCred savedWifiList[WIFI_SAVED_MAX];
uint8_t savedWifiCount        = 0;

// 4 keyboard page layouts for Wi-Fi password entry (lower and upper case)
const char WIFI_KB_P0_LO[12] = {'q','w','e','r','a','s','d','f','z','x','c','v'};
const char WIFI_KB_P0_HI[12] = {'Q','W','E','R','A','S','D','F','Z','X','C','V'};
const char WIFI_KB_P1_LO[11] = {'t','y','u','i','g','h','j','k','b','n','m'};
const char WIFI_KB_P1_HI[11] = {'T','Y','U','I','G','H','J','K','B','N','M'};
const char WIFI_KB_P2_LO[10] = {'o','p','[',']','l',';','\'',',','.','/'};
const char WIFI_KB_P2_HI[10] = {'O','P','{','}','L',':','"','<','>','?'};
const char WIFI_KB_P3_LO[12] = {'1','2','3','4','5','6','7','8','9','0','-','='};
const char WIFI_KB_P3_HI[12] = {'!','@','#','$','%','^','&','*','(',')','_','+'};

// State variables for Wi-Fi list screen (view, connect, and delete saved networks)
enum WifiListView {
  WIFILIST_VIEW_LIST       = 0,
  WIFILIST_VIEW_DIALOG     = 1,
  WIFILIST_VIEW_CONNECTING = 2,
  WIFILIST_VIEW_RESULT     = 3
};

WifiListView wifiListView        = WIFILIST_VIEW_LIST;
uint8_t wifiListScrollOffset     = 0;
int8_t wifiListSelectedIdx       = -1;
bool wifiListConnectSuccess      = false;                 // Connection result from Wi-Fi list
uint32_t wifiListConnectStartMs  = 0;                     // Connection timeout timer from Wi-Fi list
char wifiListConnectSsid[33]     = "";                    // SSID of current connection attempt from Wi-Fi list

// State variables for Wi-Fi analyzer screen (channel occupancy 1..11)
#define WIFI_ANALYZER_CHANNELS     11
#define WIFI_ANALYZER_INTERVAL_MS  2500UL
uint8_t wifiAnalyzerLoad[WIFI_ANALYZER_CHANNELS]    = {0};
uint8_t wifiAnalyzerApCount[WIFI_ANALYZER_CHANNELS] = {0};
bool wifiAnalyzerScanning          = false;
bool wifiAnalyzerHasData           = false;
uint32_t lastWifiAnalyzerScanMs    = 0;

// State variables for Wi-Fi jam detect screen (Deauth/Disassoc frames 0..250/s on channels 1..11)
#define WIFI_JAM_CHANNELS          11
#define WIFI_JAM_MAX_PPS           250
#define WIFI_JAM_DWELL_MS          200UL
portMUX_TYPE wifiJamMux            = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t wifiJamCurrPkts  = 0;
uint16_t wifiJamPps[WIFI_JAM_CHANNELS] = {0};
uint8_t wifiJamCurrChan            = 1;
bool wifiJamActive                 = false;
bool wifiJamWasConnected           = false;
uint32_t wifiJamChanStartMs        = 0;

// Structures and state variables for Wi-Fi info screen (14 parameters of current AP)
#define WIFI_INFO_ROW_COUNT        14
#define WIFI_INFO_VISIBLE_ROWS     5

struct WifiInfoRow {
  char label[20];
  char value[40];
};

struct OuiVendorEntry {
  uint8_t b0;
  uint8_t b1;
  uint8_t b2;
  const char* vendor;
};

const OuiVendorEntry WIFI_OUI_TABLE[] = {
  {0x00, 0x0C, 0x42, "MikroTik"},
  {0x48, 0x8F, 0x5A, "MikroTik"},
  {0xD4, 0xCA, 0x6D, "MikroTik"},
  {0xE4, 0x8D, 0x8C, "MikroTik"},
  {0x04, 0x18, 0xD6, "Ubiquiti"},
  {0x24, 0xA4, 0x3C, "Ubiquiti"},
  {0x74, 0x83, 0xC2, "Ubiquiti"},
  {0xF0, 0x9F, 0xC2, "Ubiquiti"},
  {0x14, 0xCC, 0x20, "TP-Link"},
  {0x50, 0xC7, 0xBF, "TP-Link"},
  {0x98, 0xDA, 0xC4, "TP-Link"},
  {0xC0, 0x25, 0xE9, "TP-Link"},
  {0xE8, 0x94, 0xF6, "TP-Link"},
  {0x04, 0xD9, 0xF5, "ASUS"},
  {0x1C, 0x87, 0x2C, "ASUS"},
  {0x2C, 0xFD, 0xA1, "ASUS"},
  {0x38, 0xD5, 0x47, "ASUS"},
  {0xAC, 0x9E, 0x17, "ASUS"},
  {0x18, 0xD6, 0xC7, "TP-Link"},
  {0x00, 0x1B, 0x11, "D-Link"},
  {0x1C, 0x7E, 0xE5, "D-Link"},
  {0xC8, 0xBE, 0x19, "D-Link"},
  {0x00, 0x14, 0xBF, "Cisco/Linksys"},
  {0x00, 0x23, 0x69, "Cisco/Linksys"},
  {0x20, 0xAA, 0x4B, "Cisco/Linksys"},
  {0x58, 0x6D, 0x8F, "Cisco/Linksys"},
  {0x00, 0x24, 0xB2, "Netgear"},
  {0x20, 0x4E, 0x7F, "Netgear"},
  {0xA4, 0x2B, 0x8C, "Netgear"},
  {0xC0, 0x3F, 0x0E, "Netgear"},
  {0x00, 0x1E, 0x10, "Huawei"},
  {0x28, 0x6E, 0xD4, "Huawei"},
  {0x48, 0x46, 0xFB, "Huawei"},
  {0xAC, 0xE2, 0x15, "Huawei"},
  {0x28, 0x6C, 0x07, "Xiaomi"},
  {0x64, 0x09, 0x80, "Xiaomi"},
  {0x78, 0x11, 0xDC, "Xiaomi"},
  {0xF0, 0xB4, 0x29, "Xiaomi"},
  {0x00, 0x1F, 0xF3, "Apple"},
  {0x28, 0xCF, 0xDA, "Apple"},
  {0x3C, 0x07, 0x54, "Apple"},
  {0xA4, 0x5E, 0x60, "Apple"},
  {0x24, 0x6F, 0x28, "Espressif"},
  {0x30, 0xAE, 0xA4, "Espressif"},
  {0x7C, 0xDF, 0xA1, "Espressif"},
  {0xB4, 0xE6, 0x2D, "Espressif"},
  {0x50, 0xFF, 0x20, "Keenetic"},
  {0xE8, 0x37, 0x7A, "Keenetic"},
  {0x00, 0x19, 0xCB, "ZyXEL"},
  {0xB0, 0xB2, 0xDC, "ZyXEL"},
  {0x00, 0x0B, 0x86, "Aruba"},
  {0x24, 0xDE, 0xC6, "Aruba"},
  {0x00, 0x26, 0x86, "Samsung"},
  {0x94, 0x35, 0x0A, "Samsung"},
  {0xB8, 0x27, 0xEB, "Raspberry Pi"},
  {0xDC, 0xA6, 0x32, "Raspberry Pi"}
};
const uint16_t WIFI_OUI_COUNT = (uint16_t)(sizeof(WIFI_OUI_TABLE) / sizeof(WIFI_OUI_TABLE[0]));

WifiInfoRow wifiInfoRows[WIFI_INFO_ROW_COUNT];
uint8_t wifiInfoScrollOffset       = 0;
bool wifiInfoConnected             = false;
bool wifiInfoInternetOk            = false;

// Structures and state variables for Sound spectrometer screen (PDM mic, FFT + Spectroid waterfall)
#define SND_SPEC_SAMPLE_RATE       16000
#define SND_SPEC_FFT_N             256
#define SND_SPEC_BANDS             112
#define SND_SPEC_WATERFALL_H       55

// Spectroid color palette (Inferno/Magma RGB565, 32 levels from black to white-yellow)
const uint16_t SPEC_COLOR_LUT[32] = {
  0x0000, 0x0003, 0x0807, 0x100B, 0x200F, 0x3012, 0x4014, 0x5015,
  0x6015, 0x7014, 0x8013, 0x9012, 0xA010, 0xB04E, 0xB8CC, 0xC94A,
  0xD1C8, 0xDA66, 0xE305, 0xEBC4, 0xF483, 0xFD43, 0xFDE4, 0xFE86,
  0xFF08, 0xFF6B, 0xFFAE, 0xFFD2, 0xFFE6, 0xFFF2, 0xFFFA, 0xFFFF
};

int16_t sndSpecRawSamples[SND_SPEC_FFT_N];
float sndSpecRe[SND_SPEC_FFT_N];
float sndSpecIm[SND_SPEC_FFT_N];
uint8_t sndSpecLevel[SND_SPEC_BANDS]     = {0};
uint8_t sndSpecPeak[SND_SPEC_BANDS]      = {0};
uint8_t sndSpecWaterfall[SND_SPEC_WATERFALL_H][SND_SPEC_BANDS];
uint8_t sndSpecWfHead                    = 0;
float sndSpecPeakFreqHz                  = 0.0f;
float sndSpecPeakDb                      = -90.0f;
bool sndSpecActive                       = false;
uint32_t lastSndSpecDrawMs               = 0;
#if ESP_IDF_VERSION_MAJOR >= 5
i2s_chan_handle_t sndSpecRxChan          = NULL;
#endif

// Structures and state variables for Sound generator screen (MAX98357A I2S, 6 waveforms)
#define SND_GEN_SAMPLE_RATE        22050
#define SND_GEN_FREQ_MIN           20
#define SND_GEN_FREQ_MAX           12000
#define SND_GEN_CHUNK_SAMPLES      256

enum SoundWaveType {
  SND_WAVE_SINE     = 0,
  SND_WAVE_SQUARE   = 1,
  SND_WAVE_TRIANGLE = 2,
  SND_WAVE_SAWTOOTH = 3,
  SND_WAVE_WHITE    = 4,
  SND_WAVE_PINK     = 5
};

const char* SND_WAVE_LABELS[6]     = { "Sine", "Square", "Tri", "Saw", "White", "Pink" };
const uint16_t SND_FREQ_STEPS[4]   = { 1, 10, 100, 1000 };

volatile uint16_t sndGenFreqHz     = 440;
volatile uint8_t sndGenVolumePct   = 50;
volatile SoundWaveType sndGenWave  = SND_WAVE_SINE;
uint8_t sndGenFreqStepIdx          = 1;     // Default step: 10 Hz (tapping center cycles 1/10/100/1000)
volatile bool sndGenPlaying        = false;
volatile bool sndGenTaskStopReq    = false;
TaskHandle_t sndGenTaskHandle      = NULL;
#if ESP_IDF_VERSION_MAJOR >= 5
i2s_chan_handle_t sndGenTxChan     = NULL;
#endif

// Structures and state variables for Hosts scan screen (Wi-Fi subnet ARP scanner)
#define HOSTS_SCAN_MAX             32
#define HOSTS_VISIBLE_ROWS         5
#define HOSTS_ARP_BATCH_SIZE       6

enum HostsScanView {
  HOSTS_VIEW_NO_WIFI  = 0,
  HOSTS_VIEW_SCANNING = 1,
  HOSTS_VIEW_LIST     = 2,
  HOSTS_VIEW_DETAIL   = 3
};

struct LanHostEntry {
  uint8_t ip[4];
  uint8_t mac[6];
  char ipStr[16];
  char macStr[18];
  char name[24];
  char vendor[18];
  char role[12];
  char openPorts[24];
  int16_t rttMs;
  bool isGateway;
  bool isSelf;
  bool detailsProbed;
};

LanHostEntry lanHosts[HOSTS_SCAN_MAX];
int16_t lanHostCount               = 0;
int16_t hostsScrollOffset          = 0;
int16_t hostsSelectedIdx           = -1;
HostsScanView hostsView            = HOSTS_VIEW_NO_WIFI;
bool hostsScanRunning              = false;
uint8_t hostsScanNetPrefix[3]      = {0, 0, 0};
uint16_t hostsScanNextHost         = 1;
uint16_t hostsScanBatchStart       = 1;
uint8_t hostsScanBatchCount        = 0;
bool hostsScanWaitingArp           = false;
uint32_t hostsScanBatchSentMs      = 0;
char hostsSubnetLabel[24]          = "";

// Structures and state variables for Bluetooth scan screen (BLE 5.0 active scanner)
#define BT_SCAN_MAX                32
#define BT_VISIBLE_ROWS            5
#define BT_SCAN_DURATION_SEC       3
#define BT_SCAN_TIMEOUT_MS         ((BT_SCAN_DURATION_SEC * 1000UL) + 2500UL)

enum BtScanView {
  BT_VIEW_SCANNING = 0,
  BT_VIEW_LIST     = 1,
  BT_VIEW_DETAIL   = 2
};

struct BtDeviceEntry {
  char name[24];
  char macStr[18];
  uint8_t mac[6];
  int8_t rssi;
  int8_t txPower;
  bool hasTxPower;
  uint8_t addrType;
  uint16_t companyId;
  bool hasCompanyId;
  uint16_t appearance;
  char serviceUuid[28];
  char vendor[18];
  char devType[16];
};

BtDeviceEntry btDevices[BT_SCAN_MAX];
int16_t btDeviceCount              = 0;
int16_t btScrollOffset             = 0;
int16_t btSelectedIdx              = -1;
BtScanView btView                  = BT_VIEW_LIST;
bool bleStackInitialized           = false;
bool btScanPending                 = false;
bool btScanRunning                 = false;
volatile bool btScanDoneFlag       = false;
uint32_t btScanStartMs             = 0;
bool btWifiPausedForBle            = false;
bool btWifiSleepForBle             = false;
volatile bool bleHidKbActive       = false;
volatile bool bleHidMouseActive    = false;
bool bleHidKbStartPending          = false;
bool bleHidMouseStartPending       = false;
bool bleHidPresenterMode           = false;
void stopBleHidKeyboard();
void stopBleHidMouse();
void requestBleHidPresenterStart();

// Key definition for Bluetooth HID Keyboard (declared before functions for Arduino IDE auto-prototypes)
struct BtKbKeyDef {
  const char* label;
  const char* shiftLabel;
  uint8_t hidCode;
  uint8_t flags;
};

// Button definition for single-screen Bluetooth HID Presenter (declared before functions for Arduino IDE)
struct BtPresenterBtn {
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
  const char* label;
  const char* sentLabel;
  uint8_t mods;
  uint8_t hidCode;
  uint8_t style;
};

// Named color entry for Color table (SCREEN_COLOR_TABLE: RGB / CMYK + Hex)
struct ColorTableEntry {
  const char* name;
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

// IEC 60062 color band entry for Resistor table (SCREEN_RESISTOR_TABLE: 4-band / 5-band)
struct ResistorColorEntry {
  const char* name;
  uint8_t r;
  uint8_t g;
  uint8_t b;
  int8_t digit;
  const char* multLabel;
  double multValue;
  const char* tolLabel;
};

// Structures and state variables for Weather screen (WeatherAPI: current, 7-day forecast, astronomy, world capitals)
#define WEATHER_FORECAST_MAX       7
#define WEATHER_CITIES_VISIBLE     5
#define WORLD_CAPITALS_COUNT       53

struct WorldCapitalEntry {
  const char* name;
  const char* country;
  const char* query;
};

// List of sovereign world capitals (strictly national capitals)
const WorldCapitalEntry WORLD_CAPITALS[WORLD_CAPITALS_COUNT] = {
  {"Moscow",       "RU", "Moscow"},
  {"Abu Dhabi",    "AE", "Abu%20Dhabi"},
  {"Amsterdam",    "NL", "Amsterdam"},
  {"Ankara",       "TR", "Ankara"},
  {"Astana",       "KZ", "Astana"},
  {"Athens",       "GR", "Athens"},
  {"Baku",         "AZ", "Baku"},
  {"Bangkok",      "TH", "Bangkok"},
  {"Beijing",      "CN", "Beijing"},
  {"Belgrade",     "RS", "Belgrade"},
  {"Berlin",       "DE", "Berlin"},
  {"Bern",         "CH", "Bern"},
  {"Bogota",       "CO", "Bogota"},
  {"Brasilia",     "BR", "Brasilia"},
  {"Brussels",     "BE", "Brussels"},
  {"Bucharest",    "RO", "Bucharest"},
  {"Budapest",     "HU", "Budapest"},
  {"Buenos Aires", "AR", "Buenos%20Aires"},
  {"Cairo",        "EG", "Cairo"},
  {"Canberra",     "AU", "Canberra"},
  {"Copenhagen",   "DK", "Copenhagen"},
  {"Dublin",       "IE", "Dublin"},
  {"Hanoi",        "VN", "Hanoi"},
  {"Helsinki",     "FI", "Helsinki"},
  {"Jakarta",      "ID", "Jakarta"},
  {"Kuala Lumpur", "MY", "Kuala%20Lumpur"},
  {"Kyiv",         "UA", "Kyiv"},
  {"Lima",         "PE", "Lima"},
  {"Lisbon",       "PT", "Lisbon"},
  {"London",       "GB", "London"},
  {"Madrid",       "ES", "Madrid"},
  {"Mexico City",  "MX", "Mexico%20City"},
  {"Minsk",        "BY", "Minsk"},
  {"Nairobi",      "KE", "Nairobi"},
  {"New Delhi",    "IN", "New%20Delhi"},
  {"Oslo",         "NO", "Oslo"},
  {"Ottawa",       "CA", "Ottawa"},
  {"Paris",        "FR", "Paris"},
  {"Prague",       "CZ", "Prague"},
  {"Reykjavik",    "IS", "Reykjavik"},
  {"Riyadh",       "SA", "Riyadh"},
  {"Rome",         "IT", "Rome"},
  {"Seoul",        "KR", "Seoul"},
  {"Singapore",    "SG", "Singapore"},
  {"Stockholm",    "SE", "Stockholm"},
  {"Tashkent",     "UZ", "Tashkent"},
  {"Tbilisi",      "GE", "Tbilisi"},
  {"Tokyo",        "JP", "Tokyo"},
  {"Vienna",       "AT", "Vienna"},
  {"Warsaw",       "PL", "Warsaw"},
  {"Washington",   "US", "Washington"},
  {"Wellington",   "NZ", "Wellington"},
  {"Yerevan",      "AM", "Yerevan"}
};

enum WeatherView {
  WEATHER_VIEW_CURRENT  = 0,
  WEATHER_VIEW_FORECAST = 1,
  WEATHER_VIEW_CITIES   = 2,
  WEATHER_VIEW_LOADING  = 3,
  WEATHER_VIEW_ERROR    = 4
};

struct WeatherForecastDay {
  char date[12];
  float minTempC;
  float maxTempC;
  uint8_t chanceOfRain;
  uint16_t conditionCode;
  char conditionText[24];
};

struct WeatherCurrentData {
  bool valid;
  char city[24];
  char country[18];
  char sourceLabel[10];
  float tempC;
  float feelsLikeC;
  uint8_t humidity;
  float windKph;
  float pressureMb;
  uint16_t conditionCode;
  bool isDay;
  char conditionText[28];
  char sunrise[12];
  char sunset[12];
  char moonPhase[20];
  uint8_t moonIllum;
  char errorMsg[32];
};

WeatherCurrentData weatherData = {
  false, "Moscow", "Russia", "Manual", 0.0f, 0.0f, 0, 0.0f, 1013.0f,
  1000, true, "No data", "--:--", "--:--", "Unknown", 0, ""
};
WeatherForecastDay weatherForecast[WEATHER_FORECAST_MAX];
uint8_t weatherForecastCount       = 0;
WeatherView weatherView            = WEATHER_VIEW_CURRENT;
bool weatherAutoLocate             = true;
uint8_t weatherCityIdx             = 0;
int16_t weatherCityScroll          = 0;
bool weatherFetchPending           = false;

// Structures and state variables for Sun activity screen (NOAA SWPC: storms, Kp, R/S/G, F10.7 30d, Kyoto Dst)
#define SUN_KP_BARS_MAX            16
#define SUN_FORECAST_DAYS          3
#define SUN_FLUX30_MAX             30
#define SUN_DST_BARS_MAX           24

enum SunActivityView {
  SUN_VIEW_CURRENT  = 0,
  SUN_VIEW_FORECAST = 1,
  SUN_VIEW_LOADING  = 2,
  SUN_VIEW_ERROR    = 3,
  SUN_VIEW_FLUX_DST = 4
};

struct SunForecastDay {
  char date[12];
  uint8_t gScale;
  uint8_t rMinorProb;
  uint8_t sProb;
  float maxKp;
};

struct SunActivityData {
  bool valid;
  float kpCurrent;
  uint8_t rScale;
  uint8_t sScale;
  uint8_t gScale;
  char gText[16];
  float windSpeedKms;
  float imfBt;
  float imfBz;
  float f107Flux;
  char stamp[20];
  char errorMsg[32];
  float kpBars[SUN_KP_BARS_MAX];
  uint8_t kpObsCount;
  uint8_t kpBarCount;
  SunForecastDay days[SUN_FORECAST_DAYS];
  float flux30d[SUN_FLUX30_MAX];
  uint8_t flux30dCount;
  float flux30dLatest;
  float flux30dMin;
  float flux30dMax;
  float flux30dAvg;
  int16_t dstBars[SUN_DST_BARS_MAX];
  uint8_t dstCount;
  int16_t dstCurrent;
  int16_t dstMin24h;
  char dstStamp[20];
};

SunActivityData sunData = {
  false, 0.0f, 0, 0, 0, "Quiet", 0.0f, 0.0f, 0.0f, 0.0f, "--", "",
  {0}, 0, 0,
  {{"--", 0, 0, 0, 0.0f}, {"--", 0, 0, 0, 0.0f}, {"--", 0, 0, 0, 0.0f}},
  {0}, 0, 0.0f, 0.0f, 0.0f, 0.0f,
  {0}, 0, 0, 0, "--"
};
SunActivityView sunView            = SUN_VIEW_CURRENT;
bool sunFetchPending               = false;

// Structures and state variables for System info screen
#define SYS_INFO_ROW_COUNT         20
#define SYS_INFO_VISIBLE_ROWS      5

struct SystemInfoRow {
  char label[22];
  char value[36];
  uint16_t color;
};

struct Axp2101PowerInfo {
  uint8_t batPercent;
  uint16_t batVoltageMv;
  uint16_t vbusVoltageMv;
  uint16_t vsysVoltageMv;
  float dieTempC;
  bool batPresent;
  bool vbusGood;
  uint8_t chgDirection;   // 0=Standby, 1=Charging, 2=Discharging
  uint8_t chgState;       // 0=Tri, 1=Pre, 2=CC, 3=CV, 4=Done, 5=Off
  uint8_t dcdcMask;
  uint8_t ldoMask0;
  uint8_t ldoMask1;
};

SystemInfoRow sysInfoRows[SYS_INFO_ROW_COUNT];
uint8_t sysInfoScrollOffset        = 0;

// Structures and state variables for Alarm (2 alarms), Stopwatch, and Timer screens
#define ALARM_COUNT                2
#define ALARM_SNOOZE_OPT_COUNT     5
const uint8_t ALARM_SNOOZE_OPTIONS[ALARM_SNOOZE_OPT_COUNT] = {5, 10, 15, 20, 30};

enum AlertRingtone : uint8_t {
  RINGTONE_BEEP = 0,
  RINGTONE_BELL = 1
};

enum ActiveAlertType : uint8_t {
  ALERT_NONE  = 0,
  ALERT_ALARM = 1,
  ALERT_TIMER = 2
};

struct AlarmEntry {
  uint8_t hour;
  uint8_t minute;
  bool enabled;
  AlertRingtone ringtone;
  uint8_t snoozeMin;
  bool snoozePending;
  uint32_t snoozeTargetMs;
  int32_t lastTriggeredMin;
};

AlarmEntry alarms[ALARM_COUNT] = {
  {7,  0, false, RINGTONE_BEEP, 5,  false, 0, -1},
  {8, 30, false, RINGTONE_BELL, 10, false, 0, -1}
};
uint8_t selectedAlarmIdx           = 0;    // 0 = Alarm 1, 1 = Alarm 2
uint8_t alarmEditField             = 0;    // 0 = HH, 1 = MM

// Stopwatch state
bool swRunning                     = false;
uint32_t swStartMs                 = 0;
uint32_t swAccumulatedMs           = 0;
uint32_t lastSwDrawMs              = 0;

// Countdown Timer state
uint8_t timerSetH                  = 0;
uint8_t timerSetM                  = 1;
uint8_t timerSetS                  = 0;
uint8_t timerEditField             = 1;    // 0 = HH, 1 = MM, 2 = SS
AlertRingtone timerRingtone        = RINGTONE_BEEP;
bool timerRunning                  = false;
bool timerPaused                   = false;
uint32_t timerRemainingMs          = 60000UL;
uint32_t timerEndMs                = 0;
uint32_t lastTimerDrawMs           = 0;

// Active alarm/timer alert dialog and I2S ringtone task
ActiveAlertType activeAlertType    = ALERT_NONE;
uint8_t activeAlertAlarmIdx        = 0;
AlertRingtone activeAlertRingtone  = RINGTONE_BEEP;
uint32_t alertStartedMs            = 0;
uint32_t lastAlertAnimMs           = 0;
uint32_t lastAlertVibMs            = 0;
TaskHandle_t alertAudioTaskHandle  = NULL;
volatile bool alertAudioStopReq    = false;
volatile bool alertAudioPlaying    = false;
#if ESP_IDF_VERSION_MAJOR >= 5
i2s_chan_handle_t alertTxChan      = NULL;
#endif

// Prototypes for alarm/timer ringtone audio control
void startAlertRingtone(AlertRingtone tone);
void stopAlertRingtone();
void wakeDisplayForAlert(AlertRingtone tone);

// State structures and variables for Flashlight, SOS signal, and Morse code screens
#define FLASHLIGHT_COLOR_COUNT 8
struct FlashlightColorItem {
  const char* name;
  uint16_t rgb565;
  bool darkText;
};
const FlashlightColorItem FLASHLIGHT_COLORS[FLASHLIGHT_COLOR_COUNT] = {
  {"White",  0xFFFF, true},
  {"Red",    0xF800, false},
  {"Green",  0x07E0, true},
  {"Blue",   0x001F, false},
  {"Yellow", 0xFFE0, true},
  {"Orange", 0xFC00, true},
  {"Pink",   0xF9B6, false},
  {"Cyan",   0x07FF, true}
};
bool flashlightOn                 = false;
uint8_t flashlightColorIdx        = 0;     // 0=White .. 7=Cyan
uint8_t flashlightBrightness      = 100;   // 5..100%
bool flashlightSliderDragging     = false;

// Shared CW tone I2S task for SOS signal (max volume 2400 Hz) and Morse code (800 Hz)
volatile bool cwAudioPlaying      = false;
volatile bool cwAudioStopReq      = false;
volatile bool cwToneGateOn        = false;
volatile float cwToneFreqHz       = 800.0f;
volatile float cwToneGain         = 1.0f;
TaskHandle_t cwAudioTaskHandle    = NULL;
#if ESP_IDF_VERSION_MAJOR >= 5
i2s_chan_handle_t cwTxChan        = NULL;
#endif

// SOS signal state (ITU ... --- ... continuous timeline: max volume, max brightness flash, motor buzz)
#define SOS_UNIT_MS               160UL
#define SOS_STEP_COUNT            18
const uint8_t SOS_STEP_ACTIVE[SOS_STEP_COUNT] = {
  1,0,1,0,1,0,  1,0,1,0,1,0,  1,0,1,0,1,0
};
const uint8_t SOS_STEP_UNITS[SOS_STEP_COUNT] = {
  1,1,1,1,1,3,  3,1,3,1,3,3,  1,1,1,1,1,7
};
const int8_t SOS_STEP_SYM[SOS_STEP_COUNT] = {
  0,-1,1,-1,2,-1,  3,-1,4,-1,5,-1,  6,-1,7,-1,8,-1
};
bool sosActive                    = false;
uint8_t sosStepIdx                = 0;
uint32_t sosStepStartMs           = 0;
bool sosFlashOn                   = false;

// Morse code table state (26 EN + 33 RU + 10 Digits + 18 Punctuation = 87 rows)
#define MORSE_VISIBLE_ROWS        7
#define MORSE_EN_COUNT            26
#define MORSE_RU_COUNT            33
#define MORSE_NUM_COUNT           10
#define MORSE_SYM_COUNT           18
#define MORSE_NUM_OFFSET          (MORSE_EN_COUNT + MORSE_RU_COUNT)
#define MORSE_SYM_OFFSET          (MORSE_NUM_OFFSET + MORSE_NUM_COUNT)
#define MORSE_TOTAL_COUNT         87
#define MORSE_UNIT_MS             110UL
struct MorseCodeEntry {
  uint8_t cp1251;
  uint8_t isRu;  // 0 = EN, 1 = RU (CP1251), 2 = Digits (0-9), 3 = Punctuation (SYM)
  const char* code;
};
const MorseCodeEntry MORSE_TABLE[MORSE_TOTAL_COUNT] = {
  // English alphabet (A-Z, 26 entries)
  {0x41, 0, ".-"},    {0x42, 0, "-..."},  {0x43, 0, "-.-."},  {0x44, 0, "-.."},
  {0x45, 0, "."},     {0x46, 0, "..-."},  {0x47, 0, "--."},   {0x48, 0, "...."},
  {0x49, 0, ".."},    {0x4A, 0, ".---"},  {0x4B, 0, "-.-"},   {0x4C, 0, ".-.."},
  {0x4D, 0, "--"},    {0x4E, 0, "-."},    {0x4F, 0, "---"},   {0x50, 0, ".--."},
  {0x51, 0, "--.-"},  {0x52, 0, ".-."},   {0x53, 0, "..."},   {0x54, 0, "-"},
  {0x55, 0, "..-"},   {0x56, 0, "...-"},  {0x57, 0, ".--"},   {0x58, 0, "-..-"},
  {0x59, 0, "-.--"},  {0x5A, 0, "--.."},
  // Russian alphabet (CP1251 0xC0..0xDF + 0xA8 Yo, 33 entries, ITU/GOST telegraph codes)
  {0xC0, 1, ".-"},    {0xC1, 1, "-..."},  {0xC2, 1, ".--"},   {0xC3, 1, "--."},
  {0xC4, 1, "-.."},   {0xC5, 1, "."},     {0xA8, 1, "."},     {0xC6, 1, "...-"},
  {0xC7, 1, "--.."},  {0xC8, 1, ".."},    {0xC9, 1, ".---"},  {0xCA, 1, "-.-"},
  {0xCB, 1, ".-.."},  {0xCC, 1, "--"},    {0xCD, 1, "-."},    {0xCE, 1, "---"},
  {0xCF, 1, ".--."},  {0xD0, 1, ".-."},   {0xD1, 1, "..."},   {0xD2, 1, "-"},
  {0xD3, 1, "..-"},   {0xD4, 1, "..-."},  {0xD5, 1, "...."},  {0xD6, 1, "-.-."},
  {0xD7, 1, "---."},  {0xD8, 1, "----"},  {0xD9, 1, "--.-"},  {0xDA, 1, "--.--"},
  {0xDB, 1, "-.--"},  {0xDC, 1, "-..-"},  {0xDD, 1, "..-.."}, {0xDE, 1, "..--"},
  {0xDF, 1, ".-.-"},
  // Digits (0-9, 10 entries, standard ITU-R M.1677)
  {0x30, 2, "-----"}, {0x31, 2, ".----"}, {0x32, 2, "..---"}, {0x33, 2, "...--"},
  {0x34, 2, "....-"}, {0x35, 2, "....."}, {0x36, 2, "-...."}, {0x37, 2, "--..."},
  {0x38, 2, "---.."}, {0x39, 2, "----."},
  // Punctuation marks (18 entries, standard ITU/international telegraph symbols)
  {0x2E, 3, ".-.-.-"}, {0x2C, 3, "--..--"}, {0x3F, 3, "..--.."}, {0x21, 3, "-.-.--"},
  {0x3A, 3, "---..."}, {0x3B, 3, "-.-.-."}, {0x2D, 3, "-....-"}, {0x2F, 3, "-..-."},
  {0x28, 3, "-.--."},  {0x29, 3, "-.--.-"}, {0x22, 3, ".-..-."}, {0x27, 3, ".----."},
  {0x3D, 3, "-...-"},  {0x2B, 3, ".-.-."},  {0x40, 3, ".--.-."}, {0x26, 3, ".-..."},
  {0x5F, 3, "..--.-"}, {0x24, 3, "...-..-"}
};
uint8_t morseScrollOffset         = 0;
int8_t morsePlayingIdx            = -1;
uint8_t morsePlayElemIdx          = 0;
bool morsePlayToneOn              = false;
uint32_t morsePlayNextMs          = 0;

void startCwToneAudio(float freqHz, float gain);
void stopCwToneAudio();
void setMotorVibrate(bool on);
void startSosSignal();
void stopSosSignal();
void updateSosSignal();
void startMorsePlayback(uint8_t idx);
void stopMorsePlayback();
void updateMorsePlayback();

// State structures and variables for Settings screen (persisted to Flash via Preferences)
#define DISPLAY_TIME_OPT_COUNT   9
#define CHARGE_CURRENT_OPT_COUNT 4
#define SETTINGS_ITEM_COUNT      8
#define SETTINGS_VISIBLE_ROWS    4
#define TOUCH_SENS_MIN_VAL       30
#define TOUCH_SENS_MAX_VAL       80
#define TOUCH_SENS_DEFAULT_VAL   50
#define TOUCH_SENS_MIN_PCT       TOUCH_SENS_MIN_VAL
#define TOUCH_SENS_MAX_PCT       TOUCH_SENS_MAX_VAL
const uint16_t DISPLAY_TIME_OPTIONS[DISPLAY_TIME_OPT_COUNT] = {0, 5, 10, 20, 30, 45, 60, 90, 180};
const uint16_t CHARGE_CURRENT_OPTIONS[CHARGE_CURRENT_OPT_COUNT] = {100, 200, 300, 400};

enum SettingsView {
  SETTINGS_VIEW_LIST       = 0,
  SETTINGS_VIEW_WIFI_AUTO  = 1,
  SETTINGS_VIEW_BRIGHTNESS = 2,
  SETTINGS_VIEW_LIGHT_TIME = 3,
  SETTINGS_VIEW_TOUCH_VIB  = 4,
  SETTINGS_VIEW_TOUCH_SENS = 5,
  SETTINGS_VIEW_CHARGE_CURR = 6,
  SETTINGS_VIEW_POWER       = 7,
  SETTINGS_VIEW_WAKE_CONDITION = 8
};

Preferences settingsPrefs;
SettingsView settingsView         = SETTINGS_VIEW_LIST;
uint8_t settingsScrollOffset      = 0;                    // Vertical scroll offset for Settings list
bool settingWifiEnabled           = true;                 // Global Wi-Fi radio power enable
bool settingWifiAutoConnect       = true;                 // Allow automatic Wi-Fi reconnection
uint8_t settingDisplayBrightness  = 94;                   // Backlight brightness (5..100%)
uint16_t settingDisplayLightSec   = 30;                   // Backlight auto-off timeout (0 = never off)
bool settingTouchVibration        = true;                 // Enable haptic feedback on screen touch
uint8_t settingTouchSensitivity   = TOUCH_SENS_DEFAULT_VAL; // FT6236/FT6336U reg 0x80 sensitivity (30..80, default 50)
uint8_t settingWakeTapCount       = 2;                    // Screen wake gesture: 2 = double tap, 3 = triple tap
uint16_t settingChargeCurrentMa   = 200;                  // AXP2101 battery charge current (100/200/300/400 mA)
bool settingPowerGps              = true;                 // AXP2101 BLDO1 GPS power rail enable
bool settingPowerLora             = true;                 // AXP2101 ALDO4 LoRa power rail enable
bool settingWakeOnTouch           = true;                 // Light Sleep wake on screen touch (TOUCH_INT_PIN / IO16)
bool settingWakeOnBma423          = true;                 // Light Sleep wake on BMA423 interrupt (IO14)
bool settingWakeOnButton          = true;                 // Light Sleep wake on button press (IO0)
bool editWifiEnabled              = true;                 // Temporary Wi-Fi power toggle before OK/Cancel
bool editWifiAutoConnect          = true;                 // Temporary toggle state before OK/Cancel
uint8_t editDisplayBrightness     = 94;                   // Temporary brightness before OK/Cancel (5..100%)
uint8_t editDisplayTimeIdx        = 4;                    // Timeout index in DISPLAY_TIME_OPTIONS (4 -> 30 s)
bool editTouchVibration           = true;                 // Temporary haptic toggle before OK/Cancel
uint8_t editTouchSensitivity      = TOUCH_SENS_DEFAULT_VAL; // Temporary reg 0x80 sensitivity before OK/Cancel
uint8_t editWakeTapCount          = 2;                    // Temporary wake gesture (2 or 3 taps) before OK/Cancel
uint16_t editChargeCurrentMa      = 200;                  // Temporary charge current before OK/Cancel
bool editPowerGps                 = true;                 // Temporary GPS BLDO1 toggle before OK/Cancel
bool editPowerLora                = true;                 // Temporary LoRa ALDO4 toggle before OK/Cancel
bool editWakeOnTouch              = true;                 // Temporary wake-on-touch toggle before OK/Cancel
bool editWakeOnBma423             = true;                 // Temporary wake-on-BMA423 toggle before OK/Cancel
bool editWakeOnButton             = true;                 // Temporary wake-on-button toggle before OK/Cancel
bool settingsSliderDragging       = false;                // Brightness slider drag flag
bool touchSensSliderDragging      = false;                // Touch sensitivity slider drag flag
bool touchVibActive               = false;                // Active haptic pulse flag
uint32_t touchVibEndMs            = 0;                    // End timestamp of short haptic pulse
bool displaySleeping              = false;                // True while ST7789V display controller is in sleep mode
bool touchInMonitorMode           = false;                // True while FT6336U is in low-power Monitor/Hibernate mode
bool gpsUartActive                = false;                // True while GPSSerial UART is initialized
bool speakerAmpPowered            = false;                // True while AXP2101 DLDO1 (MAX98357A) rail is enabled
#define TOUCH_VIB_DURATION_MS     35UL                    // Touch vibration pulse duration (ms)
void initTouchScreen();
void setTouchActiveMode();
void setTouchMonitorMode();
void setTouchHibernateMode();
void putDisplayToSleep();
void wakeDisplayFromSleep();
void enterDisplayOffLightSleep();
void applyAxp2101PowerSettings();
void applyAxp2101LightSleepRails();
void setSpeakerAmpPower(bool enable);
void updateSpeakerAmpPower();
void applyGpsUartState();
void applyWifiPowerState();
void reinitDRV2605LAfterSleep();
void stopWifiJamDetect();
void stopHostsScan();

// Tetris game state (10x20 standard well, 7 tetrominoes, score/lines/level, pause, game over)
#define TETRIS_COLS 10
#define TETRIS_ROWS 20
const uint16_t TETRIS_SHAPES[7][4] = {
  {0x0F00, 0x2222, 0x00F0, 0x4444}, // I
  {0x6600, 0x6600, 0x6600, 0x6600}, // O
  {0x4E00, 0x4640, 0x0E40, 0x4C40}, // T
  {0x6C00, 0x4620, 0x06C0, 0x8C40}, // S
  {0xC600, 0x2640, 0x0C60, 0x4C80}, // Z
  {0x8E00, 0x6440, 0x0E20, 0x44C0}, // J
  {0x2E00, 0x4460, 0x0E80, 0xC440}  // L
};
const uint16_t TETRIS_COLORS[7] = {
  0x07FF, // I: Cyan
  0xFFE0, // O: Yellow
  0xF81F, // T: Magenta
  0x07E0, // S: Green
  0xF800, // Z: Red
  0x001F, // J: Blue
  0xFD20  // L: Orange
};
uint8_t tetrisBoard[TETRIS_ROWS][TETRIS_COLS];
uint8_t tetrisCurType  = 0;
uint8_t tetrisCurRot   = 0;
int8_t  tetrisCurX     = 3;
int8_t  tetrisCurY     = 0;
uint8_t tetrisNextType = 1;
uint32_t tetrisScore   = 0;
uint16_t tetrisLines   = 0;
uint8_t  tetrisLevel   = 1;
bool tetrisPaused      = false;
bool tetrisGameOver    = false;
uint32_t lastTetrisDropMs = 0;
void tetrisResetGame();
void updateTetrisGame();

// Paint program state (120x86 4-bit indexed canvas rendered 2x2 to 240x172 at y=28..199, saved in Flash)
#define PAINT_CANVAS_Y     28
#define PAINT_CANVAS_H     172
#define PAINT_BUF_W        120
#define PAINT_BUF_H        86
#define PAINT_BUF_PIXELS   (PAINT_BUF_W * PAINT_BUF_H)
#define PAINT_BUF_BYTES    (PAINT_BUF_PIXELS / 2)
#define PAINT_CHUNK_BYTES  (PAINT_BUF_BYTES / 2)
#define PAINT_COLOR_COUNT  10
#define PAINT_BG_COUNT     6
#define PAINT_SIZE_COUNT   4
#define PAINT_BRUSH_COUNT  4

const uint16_t PAINT_COLORS[PAINT_COLOR_COUNT] = {
  0xFFFF, // 1: White
  0xF800, // 2: Red
  0xFD20, // 3: Orange
  0xFFE0, // 4: Yellow
  0x07E0, // 5: Green
  0x07FF, // 6: Cyan
  0x001F, // 7: Blue
  0xF81F, // 8: Magenta
  0xFC18, // 9: Pink
  0x0000  // 10: Black
};
const uint16_t PAINT_BG_COLORS[PAINT_BG_COUNT] = {
  0x0000, // 0: Black
  0x0842, // 1: Navy
  0xFFFF, // 2: White
  0x2124, // 3: Slate
  0x0261, // 4: Forest
  0xFF5A  // 5: Cream
};
const uint8_t PAINT_SIZES[PAINT_SIZE_COUNT] = {1, 2, 4, 6};
const char* const PAINT_BRUSH_NAMES[PAINT_BRUSH_COUNT] = {"RND", "SQR", "SPR", "DSH"};

Preferences paintPrefs;
uint8_t paintCanvas[PAINT_BUF_BYTES];
uint8_t paintColorIdx     = 5; // Default Cyan
uint8_t paintBgIdx        = 0; // Default Black
uint8_t paintSizeIdx      = 1; // Default 2 logical px (4 screen px)
uint8_t paintBrushIdx     = 0; // Default Round solid brush
bool paintDrawing         = false;
int16_t paintLastX        = -1;
int16_t paintLastY        = -1;
uint32_t paintStrokeStep  = 0;
uint32_t paintSaveBannerMs = 0;

void paintClearCanvas();
void savePaintToFlash();
void loadPaintFromFlash();
void paintDrawSegment(int16_t sx0, int16_t sy0, int16_t sx1, int16_t sy1);

// Constants and state variables for Accel. spectrometer screen (BMA423 vibration FFT + Spectroid waterfall)
#define ACC_SPEC_SAMPLE_RATE       200
#define ACC_SPEC_FFT_N             128
#define ACC_SPEC_BANDS             112
#define ACC_SPEC_WATERFALL_H       55
#define ACC_SPEC_SAMPLE_PERIOD_US  5000UL
#define ACC_SPEC_DRAW_INTERVAL_MS  80UL

float accelSpecRing[ACC_SPEC_FFT_N];
uint16_t accelSpecRingPos          = 0;
uint16_t accelSpecSampleCount      = 0;
float accelSpecRe[ACC_SPEC_FFT_N];
float accelSpecIm[ACC_SPEC_FFT_N];
uint8_t accelSpecLevel[ACC_SPEC_BANDS]    = {0};
uint8_t accelSpecPeak[ACC_SPEC_BANDS]     = {0};
uint8_t accelSpecWaterfall[ACC_SPEC_WATERFALL_H][ACC_SPEC_BANDS];
uint8_t accelSpecWfHead            = 0;
uint8_t accelSpecAxis              = 0; // 0=MAG, 1=X, 2=Y, 3=Z
const char* const ACC_SPEC_AXIS_LABELS[4] = {"MAG", "X", "Y", "Z"};
bool accelSpecActive               = false;
float accelSpecPeakFreqHz          = 0.0f;
float accelSpecPeakG               = 0.0f;
float accelSpecPeakDb              = -80.0f;
uint32_t lastAccelSpecSampleUs     = 0;
uint32_t lastAccelSpecDrawMs       = 0;

void startAccelSpectrometer();
void stopAccelSpectrometer();
void sampleAccelSpectrometerOnce();
void updateAccelSpectrometer();
void drawAccelSpectrometerScreen();

// GPS and NTP clock synchronization parameters (in Clock edit mode)
const char* NTP_SERVER_PRIMARY    = "pool.ntp.org";
const char* NTP_SERVER_SECONDARY  = "time.nist.gov";
char clockSyncStatusMsg[24]       = "";
uint32_t clockSyncStatusMs        = 0;

// Constants and state variables for ASCII table screen (ASCII 0..127 + CP1251 128..255)
#define ASCII_VISIBLE_ROWS 8
#define ASCII_TOTAL_CODES  256
uint8_t asciiScrollOffset         = 0;
const char* ASCII_CTRL_NAMES[32] = {
  "NUL", "SOH", "STX", "ETX", "EOT", "ENQ", "ACK", "BEL",
  "BS",  "TAB", "LF",  "VT",  "FF",  "CR",  "SO",  "SI",
  "DLE", "DC1", "DC2", "DC3", "DC4", "NAK", "SYN", "ETB",
  "CAN", "EM",  "SUB", "ESC", "FS",  "GS",  "RS",  "US"
};

// 5x7 bitmap font (column-major, bit0 = top row) for CP1251 characters (0x80..0xFF)
const uint8_t CP1251_GLYPHS_5X7[128][5] = {
  {0x01,0x41,0x3F,0x45,0x7D}, {0x7E,0x02,0x03,0x02,0x02}, {0x00,0x40,0x30,0x10,0x00}, {0x7C,0x04,0x05,0x04,0x00}, // 0x80..0x83
  {0x30,0x00,0x30,0x00,0x00}, {0x20,0x00,0x20,0x00,0x20}, {0x02,0x02,0x7F,0x02,0x02}, {0x12,0x12,0x7F,0x12,0x12}, // 0x84..0x87
  {0x3E,0x55,0x55,0x55,0x22}, {0x29,0x04,0x3A,0x01,0x38}, {0x7E,0x01,0x7F,0x4A,0x3C}, {0x08,0x14,0x22,0x41,0x00}, // 0x88..0x8B
  {0x7F,0x04,0x7F,0x44,0x38}, {0x7E,0x08,0x15,0x22,0x40}, {0x72,0x0F,0x12,0x7A,0x02}, {0x3F,0x20,0x60,0x20,0x3F}, // 0x8C..0x8F
  {0x02,0x3F,0x4A,0x78,0x00}, {0x00,0x06,0x01,0x00,0x00}, {0x00,0x04,0x03,0x00,0x00}, {0x06,0x01,0x06,0x01,0x00}, // 0x90..0x93
  {0x04,0x03,0x04,0x03,0x00}, {0x00,0x08,0x1C,0x08,0x00}, {0x00,0x08,0x08,0x08,0x00}, {0x08,0x08,0x08,0x08,0x08}, // 0x94..0x97
  {0x00,0x00,0x00,0x00,0x00}, {0x19,0x07,0x09,0x04,0x18}, {0x78,0x04,0x7C,0x54,0x38}, {0x00,0x41,0x22,0x14,0x08}, // 0x98..0x9B
  {0x7C,0x08,0x7C,0x48,0x30}, {0x7C,0x10,0x29,0x44,0x00}, {0x72,0x0F,0x0A,0x70,0x00}, {0x3C,0x20,0x60,0x20,0x3C}, // 0x9C..0x9F
  {0x00,0x00,0x00,0x00,0x00}, {0x0C,0x51,0x52,0x31,0x0C}, {0x0C,0x51,0x52,0x31,0x0C}, {0x30,0x40,0x41,0x3F,0x01}, // 0xA0..0xA3
  {0x22,0x1C,0x14,0x1C,0x22}, {0x7E,0x02,0x02,0x02,0x03}, {0x00,0x00,0x77,0x00,0x00}, {0x0A,0x55,0x55,0x55,0x28}, // 0xA4..0xA7
  {0x7E,0x4B,0x4A,0x4B,0x42}, {0x3E,0x41,0x5D,0x55,0x22}, {0x3E,0x49,0x49,0x49,0x22}, {0x04,0x0A,0x04,0x0A,0x00}, // 0xA8..0xAB
  {0x04,0x04,0x04,0x04,0x1C}, {0x00,0x00,0x00,0x00,0x00}, {0x3E,0x41,0x5D,0x55,0x2A}, {0x00,0x01,0x7C,0x01,0x00}, // 0xAC..0xAF
  {0x02,0x05,0x05,0x02,0x00}, {0x24,0x24,0x3F,0x24,0x24}, {0x00,0x41,0x7F,0x41,0x00}, {0x00,0x44,0x7D,0x40,0x00}, // 0xB0..0xB3
  {0x7C,0x04,0x04,0x06,0x00}, {0x7C,0x20,0x20,0x3C,0x00}, {0x06,0x09,0x7F,0x09,0x7F}, {0x00,0x00,0x08,0x00,0x00}, // 0xB4..0xB7
  {0x38,0x55,0x54,0x55,0x18}, {0x7F,0x06,0x7F,0x0B,0x0B}, {0x38,0x54,0x54,0x54,0x00}, {0x0A,0x04,0x0A,0x04,0x00}, // 0xB8..0xBB
  {0x20,0x40,0x44,0x3D,0x00}, {0x26,0x49,0x49,0x49,0x32}, {0x48,0x54,0x54,0x54,0x20}, {0x00,0x01,0x7C,0x01,0x00}, // 0xBC..0xBF
  {0x7E,0x09,0x09,0x09,0x7E}, {0x7F,0x49,0x49,0x49,0x31}, {0x7F,0x49,0x49,0x49,0x36}, {0x7F,0x01,0x01,0x01,0x01}, // 0xC0..0xC3
  {0x60,0x3F,0x21,0x3F,0x60}, {0x7F,0x49,0x49,0x49,0x41}, {0x63,0x14,0x7F,0x14,0x63}, {0x22,0x41,0x49,0x49,0x36}, // 0xC4..0xC7
  {0x7F,0x10,0x08,0x04,0x7F}, {0x7C,0x21,0x11,0x09,0x7C}, {0x7F,0x08,0x14,0x22,0x41}, {0x40,0x3E,0x01,0x01,0x7F}, // 0xC8..0xCB
  {0x7F,0x02,0x04,0x02,0x7F}, {0x7F,0x08,0x08,0x08,0x7F}, {0x3E,0x41,0x41,0x41,0x3E}, {0x7F,0x01,0x01,0x01,0x7F}, // 0xCC..0xCF
  {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x41,0x41,0x22}, {0x01,0x01,0x7F,0x01,0x01}, {0x27,0x48,0x48,0x48,0x3F}, // 0xD0..0xD3
  {0x1C,0x22,0x7F,0x22,0x1C}, {0x63,0x14,0x08,0x14,0x63}, {0x3F,0x20,0x20,0x3F,0x60}, {0x07,0x08,0x08,0x08,0x7F}, // 0xD4..0xD7
  {0x7F,0x40,0x7F,0x40,0x7F}, {0x3F,0x20,0x3F,0x20,0x7F}, {0x01,0x7F,0x48,0x48,0x30}, {0x7F,0x48,0x78,0x00,0x7F}, // 0xD8..0xDB
  {0x7F,0x48,0x48,0x48,0x30}, {0x22,0x41,0x49,0x49,0x3E}, {0x7F,0x08,0x3E,0x41,0x3E}, {0x46,0x29,0x19,0x09,0x7F}, // 0xDC..0xDF
  {0x20,0x54,0x54,0x54,0x78}, {0x3C,0x4A,0x49,0x49,0x31}, {0x7C,0x54,0x54,0x54,0x28}, {0x7C,0x04,0x04,0x04,0x04}, // 0xE0..0xE3
  {0x60,0x3C,0x24,0x3C,0x60}, {0x38,0x54,0x54,0x54,0x18}, {0x44,0x28,0x7C,0x28,0x44}, {0x44,0x44,0x54,0x54,0x28}, // 0xE4..0xE7
  {0x7C,0x20,0x10,0x08,0x7C}, {0x7C,0x21,0x10,0x09,0x7C}, {0x7C,0x10,0x28,0x44,0x00}, {0x40,0x38,0x04,0x04,0x7C}, // 0xE8..0xEB
  {0x7C,0x08,0x10,0x08,0x7C}, {0x7C,0x10,0x10,0x10,0x7C}, {0x38,0x44,0x44,0x44,0x38}, {0x7C,0x04,0x04,0x04,0x7C}, // 0xEC..0xEF
  {0x7C,0x14,0x14,0x14,0x08}, {0x38,0x44,0x44,0x44,0x00}, {0x04,0x04,0x7C,0x04,0x04}, {0x0C,0x50,0x50,0x30,0x1C}, // 0xF0..0xF3
  {0x18,0x24,0x7E,0x24,0x18}, {0x44,0x28,0x10,0x28,0x44}, {0x3C,0x20,0x20,0x3C,0x60}, {0x0C,0x10,0x10,0x10,0x7C}, // 0xF4..0xF7
  {0x7C,0x40,0x7C,0x40,0x7C}, {0x3C,0x20,0x3C,0x20,0x7C}, {0x04,0x7C,0x50,0x50,0x20}, {0x7C,0x50,0x70,0x00,0x7C}, // 0xF8..0xFB
  {0x7C,0x50,0x50,0x50,0x20}, {0x44,0x44,0x54,0x54,0x38}, {0x7C,0x10,0x38,0x44,0x38}, {0x48,0x34,0x14,0x14,0x7C}  // 0xFC..0xFF
};

// Constants and state variables for Temp. sensor screen (ESP32 + BMA423 + AXP2101, 1 Hz)
#define TEMP_HISTORY_MAX 60
enum TempDisplayUnit {
  TEMP_UNIT_C = 0,
  TEMP_UNIT_K = 1,
  TEMP_UNIT_F = 2
};

float tempHistoryC[TEMP_HISTORY_MAX];
float bmaTempHistoryC[TEMP_HISTORY_MAX];
bool bmaTempHistoryValid[TEMP_HISTORY_MAX];
float axpTempHistoryC[TEMP_HISTORY_MAX];
bool axpTempHistoryValid[TEMP_HISTORY_MAX];
uint8_t tempHistoryCount          = 0;
float currentTempC                = 0.0f;
float currentBmaTempC             = 0.0f;
bool bmaTempValid                 = false;
float currentAxpTempC             = 0.0f;
bool axpTempValid                 = false;
TempDisplayUnit tempGraphUnit     = TEMP_UNIT_C;

// Constants and state variables for Accel. sensor screen (BMA423, X/Y/Z in m/s^2)
#define ACCEL_HISTORY_MAX 60
#define GRAVITY_MPS2      9.80665f
float accelHistoryX[ACCEL_HISTORY_MAX];
float accelHistoryY[ACCEL_HISTORY_MAX];
float accelHistoryZ[ACCEL_HISTORY_MAX];
uint8_t accelHistoryCount         = 0;
float currentAccelX               = 0.0f;
float currentAccelY               = 0.0f;
float currentAccelZ               = GRAVITY_MPS2;
uint32_t lastAccelSampleMs        = 0;
uint32_t lastAccelDrawMs          = 0;

// Constants, structures, and data for Periodic table screen (118 elements, 4 pages)
#define PERIODIC_PAGE_COUNT    4
#define PERIODIC_ELEMENT_COUNT 118

enum PeriodicView {
  PERIODIC_VIEW_TABLE  = 0,
  PERIODIC_VIEW_DIALOG = 1
};

struct PeriodicElement {
  uint8_t atomicNumber;
  const char* symbol;
  const char* name;
  const char* mass;
  const char* density;
  const char* meltC;
  const char* boilC;
  uint8_t period;   // 1..7 (8 = lanthanides 57-71, 9 = actinides 89-103)
  uint8_t group;    // 1..18 for periods 1..7, 1..15 for f-block
  uint8_t category; // 0..9 element category
};

const char* const PERIODIC_CATEGORIES[10] = {
  "Nonmetal", "Noble gas", "Alkali metal", "Alkaline earth", "Metalloid",
  "Halogen", "Transition metal", "Post-transition", "Lanthanide", "Actinide"
};

const char* const PERIODIC_PAGE_TITLES[PERIODIC_PAGE_COUNT] = {
  "Groups 1-6",
  "Groups 7-12",
  "Groups 13-18",
  "Lanthanides & Actinides"
};

const PeriodicElement PERIODIC_ELEMENTS[PERIODIC_ELEMENT_COUNT] = {
  {1,   "H",  "Hydrogen",      "1.008",  "0.00009", "-259.2", "-252.9", 1,  1, 0},
  {2,   "He", "Helium",        "4.0026", "0.00018", "-272.2", "-268.9", 1, 18, 1},
  {3,   "Li", "Lithium",       "6.94",   "0.534",   "180.5",  "1342",   2,  1, 2},
  {4,   "Be", "Beryllium",     "9.0122", "1.85",    "1287",   "2470",   2,  2, 3},
  {5,   "B",  "Boron",         "10.81",  "2.34",    "2076",   "3927",   2, 13, 4},
  {6,   "C",  "Carbon",        "12.011", "2.267",   "3550",   "4027",   2, 14, 0},
  {7,   "N",  "Nitrogen",      "14.007", "0.00125", "-210.0", "-195.8", 2, 15, 0},
  {8,   "O",  "Oxygen",        "15.999", "0.00143", "-218.8", "-183.0", 2, 16, 0},
  {9,   "F",  "Fluorine",      "18.998", "0.00170", "-219.7", "-188.1", 2, 17, 5},
  {10,  "Ne", "Neon",          "20.180", "0.00090", "-248.6", "-246.1", 2, 18, 1},
  {11,  "Na", "Sodium",        "22.990", "0.968",   "97.8",   "883",    3,  1, 2},
  {12,  "Mg", "Magnesium",     "24.305", "1.738",   "650",    "1091",   3,  2, 3},
  {13,  "Al", "Aluminium",     "26.982", "2.70",    "660.3",  "2470",   3, 13, 7},
  {14,  "Si", "Silicon",       "28.085", "2.329",   "1414",   "3265",   3, 14, 4},
  {15,  "P",  "Phosphorus",    "30.974", "1.823",   "44.2",   "280.5",  3, 15, 0},
  {16,  "S",  "Sulfur",        "32.06",  "2.07",    "115.2",  "444.6",  3, 16, 0},
  {17,  "Cl", "Chlorine",      "35.45",  "0.00321", "-101.5", "-34.0",  3, 17, 5},
  {18,  "Ar", "Argon",         "39.948", "0.00178", "-189.4", "-185.9", 3, 18, 1},
  {19,  "K",  "Potassium",     "39.098", "0.862",   "63.5",   "759",    4,  1, 2},
  {20,  "Ca", "Calcium",       "40.078", "1.55",    "842",    "1484",   4,  2, 3},
  {21,  "Sc", "Scandium",      "44.956", "2.985",   "1541",   "2836",   4,  3, 6},
  {22,  "Ti", "Titanium",      "47.867", "4.506",   "1668",   "3287",   4,  4, 6},
  {23,  "V",  "Vanadium",      "50.942", "6.11",    "1910",   "3407",   4,  5, 6},
  {24,  "Cr", "Chromium",      "51.996", "7.19",    "1907",   "2671",   4,  6, 6},
  {25,  "Mn", "Manganese",     "54.938", "7.21",    "1246",   "2061",   4,  7, 6},
  {26,  "Fe", "Iron",          "55.845", "7.874",   "1538",   "2862",   4,  8, 6},
  {27,  "Co", "Cobalt",        "58.933", "8.90",    "1495",   "2927",   4,  9, 6},
  {28,  "Ni", "Nickel",        "58.693", "8.908",   "1455",   "2913",   4, 10, 6},
  {29,  "Cu", "Copper",        "63.546", "8.96",    "1084.6", "2562",   4, 11, 6},
  {30,  "Zn", "Zinc",          "65.38",  "7.14",    "419.5",  "907",    4, 12, 6},
  {31,  "Ga", "Gallium",       "69.723", "5.91",    "29.8",   "2400",   4, 13, 7},
  {32,  "Ge", "Germanium",     "72.630", "5.323",   "938.3",  "2833",   4, 14, 4},
  {33,  "As", "Arsenic",       "74.922", "5.727",   "817",    "614",    4, 15, 4},
  {34,  "Se", "Selenium",      "78.971", "4.81",    "221",    "685",    4, 16, 0},
  {35,  "Br", "Bromine",       "79.904", "3.103",   "-7.2",   "58.8",   4, 17, 5},
  {36,  "Kr", "Krypton",       "83.798", "0.00375", "-157.4", "-153.2", 4, 18, 1},
  {37,  "Rb", "Rubidium",      "85.468", "1.532",   "39.3",   "688",    5,  1, 2},
  {38,  "Sr", "Strontium",     "87.62",  "2.64",    "777",    "1382",   5,  2, 3},
  {39,  "Y",  "Yttrium",       "88.906", "4.472",   "1526",   "3336",   5,  3, 6},
  {40,  "Zr", "Zirconium",     "91.224", "6.52",    "1855",   "4409",   5,  4, 6},
  {41,  "Nb", "Niobium",       "92.906", "8.57",    "2477",   "4744",   5,  5, 6},
  {42,  "Mo", "Molybdenum",    "95.95",  "10.28",   "2623",   "4639",   5,  6, 6},
  {43,  "Tc", "Technetium",    "[98]",   "11.5",    "2157",   "4265",   5,  7, 6},
  {44,  "Ru", "Ruthenium",     "101.07", "12.45",   "2334",   "4150",   5,  8, 6},
  {45,  "Rh", "Rhodium",       "102.91", "12.41",   "1964",   "3695",   5,  9, 6},
  {46,  "Pd", "Palladium",     "106.42", "12.02",   "1554.9", "2963",   5, 10, 6},
  {47,  "Ag", "Silver",        "107.87", "10.49",   "961.8",  "2162",   5, 11, 6},
  {48,  "Cd", "Cadmium",       "112.41", "8.65",    "321.1",  "767",    5, 12, 6},
  {49,  "In", "Indium",        "114.82", "7.31",    "156.6",  "2072",   5, 13, 7},
  {50,  "Sn", "Tin",           "118.71", "7.287",   "231.9",  "2602",   5, 14, 7},
  {51,  "Sb", "Antimony",      "121.76", "6.697",   "630.6",  "1587",   5, 15, 4},
  {52,  "Te", "Tellurium",     "127.60", "6.24",    "449.5",  "988",    5, 16, 4},
  {53,  "I",  "Iodine",        "126.90", "4.933",   "113.7",  "184.3",  5, 17, 5},
  {54,  "Xe", "Xenon",         "131.29", "0.00589", "-111.8", "-108.1", 5, 18, 1},
  {55,  "Cs", "Cesium",        "132.91", "1.93",    "28.5",   "671",    6,  1, 2},
  {56,  "Ba", "Barium",        "137.33", "3.51",    "727",    "1897",   6,  2, 3},
  {57,  "La", "Lanthanum",     "138.91", "6.162",   "920",    "3464",   8,  1, 8},
  {58,  "Ce", "Cerium",        "140.12", "6.770",   "795",    "3443",   8,  2, 8},
  {59,  "Pr", "Praseodymium",  "140.91", "6.77",    "935",    "3520",   8,  3, 8},
  {60,  "Nd", "Neodymium",     "144.24", "7.01",    "1024",   "3074",   8,  4, 8},
  {61,  "Pm", "Promethium",    "[145]",  "7.26",    "1042",   "3000",   8,  5, 8},
  {62,  "Sm", "Samarium",      "150.36", "7.52",    "1072",   "1794",   8,  6, 8},
  {63,  "Eu", "Europium",      "151.96", "5.244",   "826",    "1529",   8,  7, 8},
  {64,  "Gd", "Gadolinium",    "157.25", "7.90",    "1312",   "3273",   8,  8, 8},
  {65,  "Tb", "Terbium",       "158.93", "8.23",    "1356",   "3230",   8,  9, 8},
  {66,  "Dy", "Dysprosium",    "162.50", "8.540",   "1407",   "2567",   8, 10, 8},
  {67,  "Ho", "Holmium",       "164.93", "8.79",    "1461",   "2720",   8, 11, 8},
  {68,  "Er", "Erbium",        "167.26", "9.066",   "1529",   "2868",   8, 12, 8},
  {69,  "Tm", "Thulium",       "168.93", "9.32",    "1545",   "1950",   8, 13, 8},
  {70,  "Yb", "Ytterbium",     "173.05", "6.90",    "824",    "1196",   8, 14, 8},
  {71,  "Lu", "Lutetium",      "174.97", "9.841",   "1652",   "3402",   8, 15, 8},
  {72,  "Hf", "Hafnium",       "178.49", "13.31",   "2233",   "4603",   6,  4, 6},
  {73,  "Ta", "Tantalum",      "180.95", "16.69",   "3017",   "5458",   6,  5, 6},
  {74,  "W",  "Tungsten",      "183.84", "19.25",   "3422",   "5555",   6,  6, 6},
  {75,  "Re", "Rhenium",       "186.21", "21.02",   "3186",   "5596",   6,  7, 6},
  {76,  "Os", "Osmium",        "190.23", "22.59",   "3033",   "5012",   6,  8, 6},
  {77,  "Ir", "Iridium",       "192.22", "22.56",   "2466",   "4428",   6,  9, 6},
  {78,  "Pt", "Platinum",      "195.08", "21.45",   "1768.3", "3825",   6, 10, 6},
  {79,  "Au", "Gold",          "196.97", "19.30",   "1064.2", "2970",   6, 11, 6},
  {80,  "Hg", "Mercury",       "200.59", "13.534",  "-38.8",  "356.7",  6, 12, 6},
  {81,  "Tl", "Thallium",      "204.38", "11.85",   "304",    "1473",   6, 13, 7},
  {82,  "Pb", "Lead",          "207.2",  "11.34",   "327.5",  "1749",   6, 14, 7},
  {83,  "Bi", "Bismuth",       "208.98", "9.78",    "271.4",  "1564",   6, 15, 7},
  {84,  "Po", "Polonium",      "[209]",  "9.196",   "254",    "962",    6, 16, 7},
  {85,  "At", "Astatine",      "[210]",  "6.35",    "302",    "337",    6, 17, 5},
  {86,  "Rn", "Radon",         "[222]",  "0.00973", "-71",    "-61.7",  6, 18, 1},
  {87,  "Fr", "Francium",      "[223]",  "1.87",    "27",     "677",    7,  1, 2},
  {88,  "Ra", "Radium",        "[226]",  "5.5",     "700",    "1737",   7,  2, 3},
  {89,  "Ac", "Actinium",      "[227]",  "10.07",   "1050",   "3200",   9,  1, 9},
  {90,  "Th", "Thorium",       "232.04", "11.72",   "1750",   "4788",   9,  2, 9},
  {91,  "Pa", "Protactinium",  "231.04", "15.37",   "1568",   "4027",   9,  3, 9},
  {92,  "U",  "Uranium",       "238.03", "19.1",    "1132.2", "4131",   9,  4, 9},
  {93,  "Np", "Neptunium",     "[237]",  "20.45",   "644",    "3902",   9,  5, 9},
  {94,  "Pu", "Plutonium",     "[244]",  "19.816",  "639.4",  "3228",   9,  6, 9},
  {95,  "Am", "Americium",     "[243]",  "12.0",    "1176",   "2607",   9,  7, 9},
  {96,  "Cm", "Curium",        "[247]",  "13.51",   "1340",   "3110",   9,  8, 9},
  {97,  "Bk", "Berkelium",     "[247]",  "14.78",   "986",    "2627",   9,  9, 9},
  {98,  "Cf", "Californium",   "[251]",  "15.1",    "900",    "1470",   9, 10, 9},
  {99,  "Es", "Einsteinium",   "[252]",  "8.84",    "860",    "996",    9, 11, 9},
  {100, "Fm", "Fermium",       "[257]",  "9.7",     "1527",   "N/A",    9, 12, 9},
  {101, "Md", "Mendelevium",   "[258]",  "10.3",    "827",    "N/A",    9, 13, 9},
  {102, "No", "Nobelium",      "[259]",  "9.9",     "827",    "N/A",    9, 14, 9},
  {103, "Lr", "Lawrencium",    "[266]",  "14.4",    "1627",   "N/A",    9, 15, 9},
  {104, "Rf", "Rutherfordium", "[267]",  "17.0",    "2100",   "5500",   7,  4, 6},
  {105, "Db", "Dubnium",       "[268]",  "21.6",    "N/A",    "N/A",    7,  5, 6},
  {106, "Sg", "Seaborgium",    "[269]",  "23.0",    "N/A",    "N/A",    7,  6, 6},
  {107, "Bh", "Bohrium",       "[270]",  "26.0",    "N/A",    "N/A",    7,  7, 6},
  {108, "Hs", "Hassium",       "[277]",  "27.0",    "N/A",    "N/A",    7,  8, 6},
  {109, "Mt", "Meitnerium",    "[278]",  "27.0",    "N/A",    "N/A",    7,  9, 6},
  {110, "Ds", "Darmstadtium",  "[281]",  "26.0",    "N/A",    "N/A",    7, 10, 6},
  {111, "Rg", "Roentgenium",   "[282]",  "22.0",    "N/A",    "N/A",    7, 11, 6},
  {112, "Cn", "Copernicium",   "[285]",  "14.0",    "10",     "67",     7, 12, 6},
  {113, "Nh", "Nihonium",      "[286]",  "16.0",    "430",    "1130",   7, 13, 7},
  {114, "Fl", "Flerovium",     "[289]",  "11.4",    "-73",    "107",    7, 14, 7},
  {115, "Mc", "Moscovium",     "[290]",  "13.5",    "400",    "1100",   7, 15, 7},
  {116, "Lv", "Livermorium",   "[293]",  "12.9",    "436",    "812",    7, 16, 7},
  {117, "Ts", "Tennessine",    "[294]",  "7.2",     "450",    "610",    7, 17, 5},
  {118, "Og", "Oganesson",     "[294]",  "7.2",     "52",     "177",    7, 18, 1}
};

uint8_t periodicPage              = 0;
PeriodicView periodicView         = PERIODIC_VIEW_TABLE;
int16_t periodicSelectedIdx       = -1;

// Constants, structures, and data for Food e-additives screen (100 food additives)
#define EADD_VISIBLE_ROWS 5
#define EADD_COUNT        100

enum EAddView {
  EADD_VIEW_LIST   = 0,
  EADD_VIEW_DIALOG = 1
};

struct FoodEAdditive {
  const char* code;
  const char* shortName;
  const char* fullName;
  const char* category;
  uint8_t hazardLevel; // 0=Safe, 1=Low, 2=Moderate, 3=High
  const char* hazardNote;
};

const char* const EADD_HAZARD_LABELS[4] = {
  "Safe",
  "Low hazard",
  "Moderate hazard",
  "High hazard"
};

const FoodEAdditive FOOD_E_ADDITIVES[EADD_COUNT] = {
  {"E100",  "Curcumin",       "Curcumin (Turmeric extract)",     "Color (Natural)",         0, "Generally recognized as safe"},
  {"E101",  "Riboflavin",     "Riboflavin (Vitamin B2)",         "Color (Natural)",         0, "Safe essential vitamin"},
  {"E102",  "Tartrazine",     "Tartrazine (FD&C Yellow 5)",      "Color (Synthetic azo)",   3, "Allergies, child hyperactivity"},
  {"E104",  "Quinoline Yel.", "Quinoline Yellow WS",             "Color (Synthetic)",       2, "May trigger hyperactivity"},
  {"E110",  "Sunset Yellow",  "Sunset Yellow FCF",               "Color (Synthetic azo)",   3, "Allergies, child hyperactivity"},
  {"E120",  "Cochineal",      "Carminic acid / Carmine",         "Color (Natural)",         1, "Rare allergic reactions"},
  {"E122",  "Carmoisine",     "Azorubine / Carmoisine",          "Color (Synthetic azo)",   3, "Allergies, restricted in US"},
  {"E124",  "Ponceau 4R",     "Ponceau 4R (Cochineal Red A)",    "Color (Synthetic azo)",   3, "Allergies, hyperactivity risk"},
  {"E127",  "Erythrosine",    "Erythrosine (FD&C Red 3)",        "Color (Synthetic)",       3, "Thyroid sensitivity concern"},
  {"E129",  "Allura Red AC",  "Allura Red AC (FD&C Red 40)",     "Color (Synthetic azo)",   2, "May affect activity in children"},
  {"E131",  "Patent Blue V",  "Patent Blue V",                   "Color (Synthetic)",       2, "Allergic reaction risk"},
  {"E132",  "Indigo Carmine", "Indigotine (Indigo Carmine)",     "Color (Synthetic)",       1, "Low risk, rare allergies"},
  {"E133",  "Brilliant Blue", "Brilliant Blue FCF",              "Color (Synthetic)",       1, "Low toxicity, rare sensitivity"},
  {"E140",  "Chlorophylls",   "Chlorophylls & Chlorophyllins",   "Color (Natural)",         0, "Safe plant pigment"},
  {"E141",  "Cu-Chlorophyll", "Copper complexes of chlorophyll", "Color (Natural)",         0, "Generally safe"},
  {"E150a", "Plain Caramel",  "Plain Caramel (Class I)",         "Color",                   0, "Generally safe"},
  {"E150c", "Ammonia Caram.", "Ammonia Caramel (Class III)",     "Color",                   2, "Limit high daily intake"},
  {"E150d", "Sulfite Caram.", "Sulfite Ammonia Caramel (IV)",    "Color",                   2, "Contains trace 4-MEI"},
  {"E151",  "Brilliant Black","Brilliant Black BN",              "Color (Synthetic azo)",   2, "Avoid in aspirin allergy"},
  {"E153",  "Veg. Carbon",    "Vegetable Carbon (Charcoal)",     "Color (Natural)",         0, "Generally safe"},
  {"E160a", "Beta-Carotene",  "Carotenes (Provitamin A)",        "Color (Natural)",         0, "Safe natural pigment"},
  {"E160b", "Annatto",        "Annatto, Bixin, Norbixin",        "Color (Natural)",         1, "Rare allergy sensitivity"},
  {"E160c", "Paprika Extr.",  "Paprika extract / Capsanthin",    "Color (Natural)",         0, "Safe natural pigment"},
  {"E160d", "Lycopene",       "Lycopene (Tomato carotenoid)",    "Color (Natural)",         0, "Safe natural antioxidant"},
  {"E162",  "Beetroot Red",   "Beetroot Red (Betanin)",          "Color (Natural)",         0, "Safe natural pigment"},
  {"E163",  "Anthocyanins",   "Anthocyanins (Berry pigments)",   "Color (Natural)",         0, "Safe natural pigment"},
  {"E170",  "Calcium Carb.",  "Calcium Carbonate (Chalk)",       "Color / Anti-caking",     0, "Safe mineral salt"},
  {"E171",  "Titanium Diox.", "Titanium Dioxide (TiO2)",         "Color (Mineral)",         3, "Banned as food additive in EU"},
  {"E172",  "Iron Oxides",    "Iron Oxides and Hydroxides",      "Color (Mineral)",         0, "Generally safe mineral"},
  {"E200",  "Sorbic Acid",    "Sorbic Acid",                     "Preservative",            0, "Low toxicity, widely safe"},
  {"E202",  "Potass. Sorb.",  "Potassium Sorbate",               "Preservative",            0, "Generally safe preservative"},
  {"E210",  "Benzoic Acid",   "Benzoic Acid",                    "Preservative",            2, "Asthma / allergy sensitivity"},
  {"E211",  "Sodium Benz.",   "Sodium Benzoate",                 "Preservative",            2, "Avoid mixing with Vit C"},
  {"E220",  "Sulphur Diox.",  "Sulphur Dioxide (SO2)",           "Preservative / Antioxid.",2, "Triggers asthma, destroys B1"},
  {"E223",  "Sod. Metabis.",  "Sodium Metabisulphite",           "Preservative / Antioxid.",2, "Sulphite allergen risk"},
  {"E234",  "Nisin",          "Nisin (Peptide preservative)",    "Preservative",            0, "Safe natural peptide"},
  {"E235",  "Natamycin",      "Natamycin (Pimaricin)",           "Preservative",            1, "Surface antifungal on cheese"},
  {"E249",  "Potass. Nitrite","Potassium Nitrite",               "Preservative",            3, "Nitrosamine formation risk"},
  {"E250",  "Sodium Nitrite", "Sodium Nitrite",                  "Preservative",            3, "Nitrosamine risk in cured meat"},
  {"E251",  "Sodium Nitrate", "Sodium Nitrate",                  "Preservative",            2, "Converts to nitrite in body"},
  {"E252",  "Potass. Nitrate","Potassium Nitrate (Saltpetre)",   "Preservative",            2, "Limit processed meat intake"},
  {"E260",  "Acetic Acid",    "Acetic Acid (Vinegar)",           "Preservative / Acid",     0, "Safe natural organic acid"},
  {"E262",  "Sod. Acetates",  "Sodium Acetate / Diacetate",      "Preservative / Acid",     0, "Generally safe"},
  {"E270",  "Lactic Acid",    "Lactic Acid",                     "Preservative / Acid",     0, "Safe natural organic acid"},
  {"E280",  "Propionic Acid", "Propionic Acid",                  "Preservative",            1, "Low risk, bread preservative"},
  {"E282",  "Calc. Propion.", "Calcium Propionate",              "Preservative",            1, "Low risk, rare migraine link"},
  {"E290",  "Carbon Dioxide", "Carbon Dioxide (CO2)",            "Propellant / Acid",       0, "Safe carbonation gas"},
  {"E296",  "Malic Acid",     "Malic Acid (Apple acid)",         "Acidity regulator",       0, "Safe natural fruit acid"},
  {"E300",  "Ascorbic Acid",  "Ascorbic Acid (Vitamin C)",       "Antioxidant",             0, "Safe essential vitamin"},
  {"E301",  "Sod. Ascorbate", "Sodium Ascorbate",                "Antioxidant",             0, "Safe Vitamin C salt"},
  {"E306",  "Tocopherols",    "Tocopherol-rich extract (Vit E)", "Antioxidant",             0, "Safe natural Vitamin E"},
  {"E310",  "Propyl Gallate", "Propyl Gallate",                  "Antioxidant",             2, "Allergy / stomach irritation"},
  {"E319",  "TBHQ",           "Tert-Butylhydroquinone",          "Antioxidant",             2, "Limit high doses in oils"},
  {"E320",  "BHA",            "Butylated Hydroxyanisole",        "Antioxidant",             3, "Possible carcinogen concern"},
  {"E321",  "BHT",            "Butylated Hydroxytoluene",        "Antioxidant",             2, "Limit intake, allergy risk"},
  {"E322",  "Lecithins",      "Lecithins (Soy / Sunflower)",     "Emulsifier / Antioxid.",  0, "Safe (check soy allergen)"},
  {"E325",  "Sodium Lactate", "Sodium Lactate",                  "Antioxidant / Humect.",   0, "Generally safe"},
  {"E330",  "Citric Acid",    "Citric Acid",                     "Acidity regulator",       0, "Safe natural citrus acid"},
  {"E331",  "Sod. Citrates",  "Sodium Citrates",                 "Acidity regulator",       0, "Generally safe"},
  {"E334",  "Tartaric Acid",  "L(+)-Tartaric Acid",              "Acidity regulator",       0, "Safe natural grape acid"},
  {"E338",  "Phosphor. Acid", "Phosphoric Acid",                 "Acidity regulator",       2, "Excess affects calcium balance"},
  {"E339",  "Sod. Phosphat.", "Sodium Phosphates",               "Stabilizer / Acid reg.",  1, "Limit excess phosphate intake"},
  {"E341",  "Calc. Phosph.",  "Calcium Phosphates",              "Stabilizer / Raising",    0, "Generally safe mineral"},
  {"E385",  "CaNa2 EDTA",     "Calcium Disodium EDTA",           "Antioxidant / Sequestr.", 1, "Low risk within ADI limit"},
  {"E400",  "Alginic Acid",   "Alginic Acid (Seaweed)",          "Thickener / Stabilizer",  0, "Safe natural seaweed fiber"},
  {"E401",  "Sod. Alginate",  "Sodium Alginate",                 "Thickener / Stabilizer",  0, "Safe natural thickener"},
  {"E406",  "Agar",           "Agar-Agar (Algae extract)",       "Thickener / Gelling",     0, "Safe natural gelling agent"},
  {"E407",  "Carrageenan",    "Carrageenan (Red seaweed)",       "Thickener / Stabilizer",  1, "May irritate sensitive gut"},
  {"E410",  "Locust Bean G.", "Locust Bean Gum (Carob)",         "Thickener / Stabilizer",  0, "Safe natural plant gum"},
  {"E412",  "Guar Gum",       "Guar Gum (Cluster bean)",         "Thickener / Stabilizer",  0, "Safe dietary fiber"},
  {"E414",  "Gum Arabic",     "Acacia Gum (Gum Arabic)",         "Thickener / Stabilizer",  0, "Safe natural plant resin"},
  {"E415",  "Xanthan Gum",    "Xanthan Gum",                     "Thickener / Stabilizer",  0, "Safe fermented thickener"},
  {"E420",  "Sorbitol",       "Sorbitol (Sugar alcohol)",        "Sweetener / Humectant",   1, "Laxative in large amounts"},
  {"E421",  "Mannitol",       "Mannitol (Sugar alcohol)",        "Sweetener / Anti-caking", 1, "Laxative in large amounts"},
  {"E422",  "Glycerol",       "Glycerol (Glycerin)",             "Humectant / Solvent",     0, "Generally safe"},
  {"E440",  "Pectins",        "Pectins (Fruit fiber)",           "Thickener / Gelling",     0, "Safe natural fruit fiber"},
  {"E450",  "Diphosphates",   "Diphosphates (Pyrophosphates)",   "Emulsifier / Raising",    1, "Monitor total phosphate intake"},
  {"E460",  "Cellulose",      "Microcrystalline Cellulose",      "Thickener / Anti-caking", 0, "Safe insoluble plant fiber"},
  {"E466",  "CMC Gum",        "Carboxymethyl Cellulose",         "Thickener / Stabilizer",  1, "Mild gut flora sensitivity"},
  {"E471",  "Mono/Diglyc.",   "Mono- and Diglycerides of FA",    "Emulsifier",              0, "Generally safe fat derivatives"},
  {"E476",  "PGPR",           "Polyglycerol Polyricinoleate",    "Emulsifier",              1, "Low risk within ADI limit"},
  {"E500",  "Sod. Carbon.",   "Sodium Carbonates (Baking soda)", "Acidity reg. / Raising",  0, "Safe baking soda / soda ash"},
  {"E503",  "Amm. Carbon.",   "Ammonium Carbonates",             "Raising agent",           0, "Safe baking leavening agent"},
  {"E509",  "Calc. Chloride", "Calcium Chloride",                "Firming agent",           0, "Safe mineral salt"},
  {"E551",  "Silicon Diox.",  "Silicon Dioxide (Silica)",        "Anti-caking agent",       0, "Inert mineral, generally safe"},
  {"E620",  "Glutamic Acid",  "L-Glutamic Acid",                 "Flavour enhancer",        1, "Natural amino acid (umami)"},
  {"E621",  "MSG",            "Monosodium Glutamate",            "Flavour enhancer",        1, "Safe; mild sensitivity in some"},
  {"E627",  "Disod. Guanyl.", "Disodium Guanylate",              "Flavour enhancer",        1, "Avoid in gout / uric acid"},
  {"E631",  "Disod. Inosin.", "Disodium Inosinate",              "Flavour enhancer",        1, "Avoid in gout / uric acid"},
  {"E635",  "Ribonucleotid.", "Disodium 5-Ribonucleotides",      "Flavour enhancer",        1, "Avoid in gout / uric acid"},
  {"E901",  "Beeswax",        "Beeswax (White and Yellow)",      "Glazing agent",           0, "Safe natural wax"},
  {"E903",  "Carnauba Wax",   "Carnauba Wax (Palm wax)",         "Glazing agent",           0, "Safe plant wax"},
  {"E904",  "Shellac",        "Shellac (Natural resin)",         "Glazing agent",           0, "Generally safe coating"},
  {"E950",  "Acesulfame K",   "Acesulfame Potassium",            "Sweetener (Artificial)",  2, "Use within daily ADI limit"},
  {"E951",  "Aspartame",      "Aspartame",                       "Sweetener (Artificial)",  2, "Unsafe for PKU (phenylalanine)"},
  {"E952",  "Cyclamates",     "Cyclamic acid & Cyclamates",      "Sweetener (Artificial)",  3, "Banned in US, restricted in EU"},
  {"E954",  "Saccharin",      "Saccharin and its salts",         "Sweetener (Artificial)",  2, "Limit intake, bitter aftertaste"},
  {"E955",  "Sucralose",      "Sucralose (Splenda)",             "Sweetener (Artificial)",  1, "Low risk; avoid high heat"},
  {"E960",  "Stevia (Stev.)", "Steviol Glycosides (Stevia)",     "Sweetener (Natural)",     0, "Safe plant sweetener"},
  {"E967",  "Xylitol",        "Xylitol (Birch sugar alcohol)",   "Sweetener (Polyol)",      0, "Safe; laxative in high doses"}
};

uint8_t eAddScrollOffset          = 0;
EAddView eAddView                 = EADD_VIEW_LIST;
int16_t eAddSelectedIdx           = -1;

// State variables for IR screen (infrared transmitter and settings)
char irHexStr[12]          = "0";                         // Entered hex code (up to 8 characters)
uint32_t irSentNoticeMs    = 0;                           // Timestamp of last transmit for "SENT" status display
uint32_t irTestNoticeMs    = 0;                           // Timestamp of last IR LED test ("TEST")
bool irSettingsPage        = false;                       // false = page 1 (keypad), true = page 2 (settings)
IrProtocol irProtocol      = IR_PROTO_NEC;                // Selected protocol (NEC, RC-5, RC-6)
uint8_t irCarrierKhz       = 38;                          // Selected carrier frequency (33, 36, 38 kHz)
bool irToggleBit           = false;                       // Alternating toggle bit for RC-5 and RC-6 protocols
bool irLedcInitialized     = false;                       // One-time LEDC IR transmitter initialization flag
uint8_t irLedcFreqKhz      = 0;                           // Currently configured LEDC IR timer frequency (kHz)

// Kinosputnik satellite TLE parameters:
// 1 77777U          26318.89050926  .00000000  00000-0  10123-4 0    19
// 2 77777  97.7685 211.6388 0011583 240.5778 345.8400 14.93589937    01
const double TLE_EPOCH_JD   = 2461040.5 + 318.89050926;   // Epoch Julian Date: 2026 day 318.89050926
const double TLE_INC_DEG    = 97.7685;                    // Orbital inclination (deg)
const double TLE_RAAN_DEG   = 211.6388;                   // Right ascension of ascending node (deg)
const double TLE_ECC        = 0.0011583;                  // Eccentricity
const double TLE_ARG_P_DEG  = 240.5778;                   // Argument of perigee (deg)
const double TLE_M0_DEG     = 345.8400;                   // Mean anomaly at epoch (deg)
const double TLE_N0_REV_DAY = 14.93589937;                // Mean motion (rev/day)

// International Space Station (ISS, NORAD 25544) TLE parameters:
// 1 25544U 98067A   26325.84919917  .00010277  00000-0  19075-3 0  9998
// 2 25544  51.6320 239.8703 0007384  51.4158 308.7491 15.49540305596195
const double ISS_TLE_EPOCH_JD   = 2461040.5 + 325.84919917; // Epoch Julian Date: 2026 day 325.84919917
const double ISS_TLE_NDOT2      = 0.00010277;               // First derivative of mean motion divided by 2 (rev/day^2)
const double ISS_TLE_INC_DEG    = 51.6320;                  // ISS orbital inclination (deg)
const double ISS_TLE_RAAN_DEG   = 239.8703;                 // ISS right ascension of ascending node (deg)
const double ISS_TLE_ECC        = 0.0007384;                // ISS orbital eccentricity
const double ISS_TLE_ARG_P_DEG  = 51.4158;                  // ISS argument of perigee (deg)
const double ISS_TLE_M0_DEG     = 308.7491;                 // ISS mean anomaly at epoch (deg)
const double ISS_TLE_N0_REV_DAY = 15.49540305;              // ISS mean motion (rev/day)

// Coordinates of major continent labels (latitude and longitude in degrees)
const ContinentLabel EARTH_LABELS[7] = {
  {"N.America",  45.0f, -100.0f},
  {"S.America", -15.0f,  -60.0f},
  {"Europe",     51.0f,   16.0f},
  {"Africa",      6.0f,   20.0f},
  {"Asia",       48.0f,   90.0f},
  {"Australia", -25.0f,  134.0f},
  {"Antarctica",-80.0f,    0.0f}
};

// Coordinates of 7 cities displayed on 3D Earth
const CityMarker EARTH_CITIES[7] = {
  {"Moscow",      55.76f,   37.62f,  6, -7},
  {"Minsk",       53.90f,   27.56f, -6,  8},
  {"Vladivostok", 43.12f,  131.89f,  0,  8},
  {"Vorkuta",     67.50f,   64.05f,  0, -7},
  {"Washington",  38.91f,  -77.04f,  0,  8},
  {"Paris",       48.86f,    2.35f,  0,  8},
  {"London",      51.51f,   -0.13f, -4, -7}
};

// Detailed Earth continent bitmap (72 latitude bands from +90 to -90 deg, 64 longitude columns from -180 to +180 deg)
// Bit 63 = -180 deg, Bit 32 = 0 deg (Greenwich), Bit 0 = +180 deg
const uint64_t EARTH_CONTINENTS[72] = {
  0x0000000000000000ULL, // +90.0 .. +87.5
  0x0000000000000000ULL, // +87.5 .. +85.0
  0x0000007800000000ULL, // +85.0 .. +82.5 (Northern Greenland)
  0x000001FC00000000ULL, // +82.5 .. +80.0 (Greenland, Ellesmere)
  0x000007FE03000000ULL, // +80.0 .. +77.5 (Greenland, Svalbard)
  0x00009FFE03003C00ULL, // +77.5 .. +75.0 (Canadian Arctic islands, Greenland, Taymyr)
  0x001FFFFC0103FFC0ULL, // +75.0 .. +72.5 (Canada, Greenland, Northern Siberia)
  0x01FFFFF801CFFFFEULL, // +72.5 .. +70.0 (Alaska, Canada, Greenland, Scandinavia, Siberia)
  0x0FFFFDF003FFFFFFULL, // +70.0 .. +67.5 (Alaska, Canada, Scandinavia, Russia)
  0x1FFFF9E01FFFFFFFULL, // +67.5 .. +65.0 (Alaska, Canada, Iceland, Russia)
  0x1FFFF8C01FFFFFFEULL, // +65.0 .. +62.5 (Alaska, Canada, Scandinavia, Siberia)
  0x0FFFF0400FFFFFFCULL, // +62.5 .. +60.0 (Canada, Scandinavia, Russia)
  0x07FFF8000FFFFFF8ULL, // +60.0 .. +57.5 (Canada, Britain, Europe, Russia)
  0x03FFF8001FFFFFF4ULL, // +57.5 .. +55.0 (Canada, Britain, Europe, Russia, Kamchatka)
  0x03FFF0003FFFFFF4ULL, // +55.0 .. +52.5 (Canada, Europe, Russia, Kamchatka)
  0x03FFE0007FFFFFF0ULL, // +52.5 .. +50.0 (Canada/USA, Europe, Kazakhstan, Russia)
  0x03FFE000FFFFFFC0ULL, // +50.0 .. +47.5 (USA, Europe, Kazakhstan, Mongolia, Primorye)
  0x03FFC000FFFFFFC0ULL, // +47.5 .. +45.0 (USA, Europe, Ukraine, Kazakhstan, China)
  0x01FFC001FFFFFFC0ULL, // +45.0 .. +42.5 (USA, France, Italy, Balkans, Asia, Japan)
  0x01FF8001EFFFFFC0ULL, // +42.5 .. +40.0 (USA, Spain, Italy, Turkey, China, Japan)
  0x01FF8001C7FFFF80ULL, // +40.0 .. +37.5 (USA, Spain, Greece, Turkey, China, Korea, Japan)
  0x00FF000183FFFF80ULL, // +37.5 .. +35.0 (USA, N. Africa, Syria, Iran, China, Japan)
  0x00FF0003EFFFFF00ULL, // +35.0 .. +32.5 (USA, Morocco, Algeria, Middle East, Tibet, China)
  0x007E0003FFFFFF00ULL, // +32.5 .. +30.0 (Mexico/Texas, N. Africa, Egypt, India, China)
  0x007C0007FFFFFF00ULL, // +30.0 .. +27.5 (Mexico, Florida, Sahara, Arabia, India, China)
  0x003C0007FFFFFF00ULL, // +27.5 .. +25.0 (Mexico, Sahara, Arabia, Pakistan, India, China)
  0x003C0007FF7FFE00ULL, // +25.0 .. +22.5 (Mexico, Cuba, Sahara, Arabia, India, S. China)
  0x001E0007FF3FFC00ULL, // +22.5 .. +20.0 (Mexico, Sahara, Arabia, India, Myanmar, Vietnam)
  0x001E0007FE1FF400ULL, // +20.0 .. +17.5 (Central America, Sahel, Yemen, India, Indochina)
  0x000F0007FC0FF400ULL, // +17.5 .. +15.0 (Central America, W. Africa, Sudan, India, Thailand, Philippines)
  0x00070007FC07E400ULL, // +15.0 .. +12.5 (Central America, W. and E. Africa, India, Philippines)
  0x0007C003FE076400ULL, // +12.5 .. +10.0 (Venezuela, Guinea, Ethiopia, S. India, Vietnam, Philippines)
  0x000FE001FF032200ULL, // +10.0 .. +7.5  (Colombia, Venezuela, Nigeria, Somalia, Sri Lanka)
  0x000FF800FE013000ULL, // +7.5  .. +5.0  (Colombia, Guiana, Cameroon, Kenya, Malaysia, Borneo)
  0x000FFC007E003800ULL, // +5.0  .. +2.5  (Ecuador, Brazil, Congo, Kenya, Sumatra, Borneo)
  0x000FFE007E003800ULL, // +2.5  .. 0.0   (Ecuador, Amazonia, Congo, Tanzania, Sumatra, Borneo)
  0x000FFF007E001C80ULL, // 0.0   .. -2.5  (Peru, Brazil, Congo, Tanzania, Sumatra, Sulawesi)
  0x0007FF807E001EC0ULL, // -2.5  .. -5.0  (Peru, Brazil, Congo, Tanzania, Java, New Guinea)
  0x0007FF807E000EC0ULL, // -5.0  .. -7.5  (Peru, Brazil, Angola, Tanzania, Java, New Guinea)
  0x0003FF807E0007C0ULL, // -7.5  .. -10.0 (Peru, Brazil, Angola, Mozambique, New Guinea)
  0x0003FF007E0000E0ULL, // -10.0 .. -12.5 (Bolivia, Brazil, Angola, Mozambique, N. Australia)
  0x0003FF007F0003E0ULL, // -12.5 .. -15.0 (Bolivia, Brazil, Zambia, Madagascar, N. Australia)
  0x0003FE007F0007F0ULL, // -15.0 .. -17.5 (Bolivia, Brazil, Namibia, Madagascar, Australia)
  0x0003FC007F000FF0ULL, // -17.5 .. -20.0 (Chile, Paraguay, Brazil, Namibia, Madagascar, Australia)
  0x0001F8007F001FF8ULL, // -20.0 .. -22.5 (Chile, Paraguay, Brazil, Botswana, Madagascar, Australia)
  0x0001F0007E001FF8ULL, // -22.5 .. -25.0 (Chile, Argentina, S. Brazil, South Africa, Madagascar, Australia)
  0x0001F0003C001FF8ULL, // -25.0 .. -27.5 (Chile, Argentina, Uruguay, South Africa, Australia)
  0x0001E0003C001FF8ULL, // -27.5 .. -30.0 (Chile, Argentina, Uruguay, South Africa, Australia)
  0x0001E00038001FF0ULL, // -30.0 .. -32.5 (Chile, Argentina, South Africa, S. Australia)
  0x0001C00018000EE0ULL, // -32.5 .. -35.0 (Argentina, Cape of Good Hope, S. Australia)
  0x00018000000006C3ULL, // -35.0 .. -37.5 (Argentina, Victoria, New Zealand)
  0x0001800000000043ULL, // -37.5 .. -40.0 (Argentina, New Zealand)
  0x0001800000000046ULL, // -40.0 .. -42.5 (Patagonia, Tasmania, New Zealand)
  0x0001000000000006ULL, // -42.5 .. -45.0 (Patagonia, S. New Zealand)
  0x0001000000000004ULL, // -45.0 .. -47.5 (Patagonia, S. New Zealand)
  0x0001000000000000ULL, // -47.5 .. -50.0 (S. Patagonia)
  0x0001000000000000ULL, // -50.0 .. -52.5 (Tierra del Fuego)
  0x0000800000000000ULL, // -52.5 .. -55.0 (Cape Horn)
  0x0000000000000000ULL, // -55.0 .. -57.5 (Drake Passage)
  0x0000000000000000ULL, // -57.5 .. -60.0 (Southern Ocean)
  0x0000000000000000ULL, // -60.0 .. -62.5 (Southern Ocean)
  0x0000040000000000ULL, // -62.5 .. -65.0 (Antarctic Peninsula)
  0x00000C0000FFFFE0ULL, // -65.0 .. -67.5 (Antarctic coast)
  0x00003C00FFFFFFF0ULL, // -67.5 .. -70.0 (Antarctica)
  0x0001FF1FFFFFFFF8ULL, // -70.0 .. -72.5 (Antarctica)
  0x003FFFFFFFFFFFFCULL, // -72.5 .. -75.0 (Antarctica)
  0x1FFFFFFFFFFFFFFEULL, // -75.0 .. -77.5 (Antarctica)
  0x7FFFFFFFFFFFFFFFULL, // -77.5 .. -80.0 (Antarctica)
  0xFFFFFFFFFFFFFFFFULL, // -80.0 .. -82.5 (Antarctica)
  0xFFFFFFFFFFFFFFFFULL, // -82.5 .. -85.0 (Antarctica)
  0xFFFFFFFFFFFFFFFFULL, // -85.0 .. -87.5 (Antarctica)
  0xFFFFFFFFFFFFFFFFULL  // -87.5 .. -90.0 (South Pole)
};

// Weekday names
const char* DOW_NAMES[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

// 9 item labels for main menu page 1 (Wi-Fi -> Calendar, LoRaWAN -> Inclinometer)
const char* MENU_LABELS[9] = {
  "Calendar", "Calc", "Clock",
  "GPS", "IR remote control", "Step counter",
  "Inclinometer", "Earth / Satellites", "Settings"
};

// 9 item labels for main menu page 2
const char* MENU_LABELS_PAGE2[9] = {
  "Wi-Fi connect", "Wi-Fi list", "Wi-Fi analyzer",
  "Wi-Fi info", "Wi-Fi jam detect", "Hosts scan",
  "Bluetooth scan", "Sound spectrometer", "Sound generator"
};

// 9 item labels for main menu page 3
const char* MENU_LABELS_PAGE3[9] = {
  "ASCII table", "Periodic table", "Food e-additives",
  "Temp. sensor", "Accel. sensor", "Sun activity",
  "Weather", "System info", "About firmware"
};

// 9 item labels for main menu page 4
const char* MENU_LABELS_PAGE4[9] = {
  "Alarm", "Stopwatch", "Timer",
  "Flashlight", "SOS signal", "Morse code",
  "Tetris", "Paint", "Accel. spectrometer"
};

// 9 item labels for main menu page 5 (Bluetooth HID & reference tables)
const char* MENU_LABELS_PAGE5[9] = {
  "Bluetooth keyboard", "Bluetooth mouse", "Bluetooth presenter",
  "Color table", "Resistor table", "Voltage divider",
  "", "", ""
};

// Boot splash and boot vibration durations (milliseconds)
#define BOOT_VIBRATION_MS 250UL
#define BOOT_SPLASH_MS    1000UL

// Color palette (RGB565)
#define COLOR_BG          0x0842  // Dark navy/slate background
#define COLOR_TOPBAR      0x10A4  // Top status bar background
#define COLOR_HEX_FILL    0x1926  // Inactive hexagon fill
#define COLOR_HEX_ACTIVE  0x1A6B  // Active hexagon fill
#define COLOR_HEX_BORDER  0x4C99  // Hexagon border outline
#define COLOR_ACCENT      0x07FF  // Cyan accent
#define COLOR_WARN        0xFD20  // Orange accent (edit mode)
#define COLOR_TEXT        0xFFFF  // White text
#define COLOR_SUBTEXT     0xAD55  // Light gray subtext
#define COLOR_GREEN       0x2689  // Green (battery indicator / GPS fix / IR send)
#define COLOR_RED         0xF986  // Red (close X button / reset / weekends)
#define COLOR_ORANGE      0xFD20  // Orange (BMA423 temperature trace)
#define COLOR_YELLOW      0xFFE0  // Yellow (AXP2101 temperature trace)

// Forward declaration for use in drawCalendarScreen()
uint8_t getDaysInMonth(uint8_t month, uint16_t year);

// ============================================================================
// 4. AXP2101 PMU, PCF8563 RTC, DRV2605L HAPTIC DRIVER, AND BMA423 I2C HELPERS
// ============================================================================

// Write a single byte to an I2C device register
void writeRegI2C(TwoWire &wire, uint8_t devAddr, uint8_t regAddr, uint8_t data) {
  wire.beginTransmission(devAddr);
  wire.write(regAddr);
  wire.write(data);
  wire.endTransmission();
}

// Read a single byte from an I2C device register
uint8_t readRegI2C(TwoWire &wire, uint8_t devAddr, uint8_t regAddr) {
  wire.beginTransmission(devAddr);
  wire.write(regAddr);
  if (wire.endTransmission(false) != 0) return 0;
  if (wire.requestFrom(devAddr, (uint8_t)1) != 1) return 0;
  return wire.read();
}

// Map charge current (100/200/300/400 mA) to AXP2101 ICC_CHG_SET register 0x62 bits [4:0]
uint8_t chargeCurrentToAxpReg(uint16_t ma) {
  switch (ma) {
    case 100: return 0x04;
    case 300: return 0x09;
    case 400: return 0x0A;
    case 200:
    default:  return 0x08;
  }
}

// AXP2101 LDO_ONOFF_CTRL0 (reg 0x90) rail bit definitions
#define AXP2101_LDO0_CORE_MASK        0x07  // ALDO1(0x01) | ALDO2(0x02) | ALDO3(0x04)
#define AXP2101_LDO0_ALDO4_LORA_MASK  0x08  // ALDO4 (bit 3): LoRa SX1262 VDD
#define AXP2101_LDO0_BLDO1_GPS_MASK   0x10  // BLDO1 (bit 4): GPS MIA-M10Q VDD
#define AXP2101_LDO0_BLDO2_MASK       0x20  // BLDO2 (bit 5): DRV2605L / AVDD
#define AXP2101_LDO0_DLDO1_SPK_MASK   0x80  // DLDO1 (bit 7): MAX98357A speaker amp SPK_VDD
#define AXP2101_LDO0_ACTIVE_BASE_MASK (AXP2101_LDO0_CORE_MASK | AXP2101_LDO0_BLDO2_MASK) // 0x27 (DLDO1 OFF by default)

// Gate MAX98357A speaker amplifier power (AXP2101 DLDO1, reg 0x90 bit 7) on demand
void setSpeakerAmpPower(bool enable) {
  uint8_t ldoEn = readRegI2C(Wire, AXP2101_ADDR, 0x90);
  if (enable) {
    ldoEn |= AXP2101_LDO0_DLDO1_SPK_MASK;
  } else {
    ldoEn &= (uint8_t)(~AXP2101_LDO0_DLDO1_SPK_MASK);
  }
  writeRegI2C(Wire, AXP2101_ADDR, 0x90, ldoEn);
  speakerAmpPowered = enable;
}

// Keep MAX98357A DLDO1 powered only while an audio playback engine is active
void updateSpeakerAmpPower() {
  bool needSpeaker = (sndGenPlaying || alertAudioPlaying || cwAudioPlaying);
  if (needSpeaker != speakerAmpPowered) {
    setSpeakerAmpPower(needSpeaker);
  }
}

// Configure GPS UART pins: active UART when GPS is enabled, high-impedance INPUT when GPS is disabled
void applyGpsUartState() {
  if (settingPowerGps) {
    if (!gpsUartActive) {
      GPSSerial.begin(currentGpsBaud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
      gpsUartActive = true;
    }
  } else {
    if (gpsUartActive) {
      GPSSerial.end();
      gpsUartActive = false;
    }
    // High-impedance INPUT (no pull-up / no drive) prevents current leakage into powered-off GPS module
    pinMode(GPS_RX_PIN, INPUT);
    pinMode(GPS_TX_PIN, INPUT);
  }
}

// Apply saved charge current (reg 0x62) and active LDO enables (reg 0x90: BLDO1/ALDO4/BLDO2, gated DLDO1)
void applyAxp2101PowerSettings() {
  uint8_t reg62 = readRegI2C(Wire, AXP2101_ADDR, 0x62);
  uint8_t iccCode = chargeCurrentToAxpReg(settingChargeCurrentMa);
  writeRegI2C(Wire, AXP2101_ADDR, 0x62, (uint8_t)((reg62 & 0xE0) | (iccCode & 0x1F)));

  uint8_t ldoEn = readRegI2C(Wire, AXP2101_ADDR, 0x90);
  // Active base rails: ALDO1(0x01), ALDO2(0x02), ALDO3(0x04), BLDO2(0x20) = 0x27
  // DLDO1(0x80) MAX98357A speaker amp is kept OFF unless an audio program is actively playing
  ldoEn = (uint8_t)(ldoEn | AXP2101_LDO0_ACTIVE_BASE_MASK);
  if (settingPowerGps)  ldoEn |= AXP2101_LDO0_BLDO1_GPS_MASK;  else ldoEn &= (uint8_t)(~AXP2101_LDO0_BLDO1_GPS_MASK);  // BLDO1 (bit 4): GPS
  if (settingPowerLora) ldoEn |= AXP2101_LDO0_ALDO4_LORA_MASK; else ldoEn &= (uint8_t)(~AXP2101_LDO0_ALDO4_LORA_MASK); // ALDO4 (bit 3): LoRa
  bool needSpeaker = (sndGenPlaying || alertAudioPlaying || cwAudioPlaying);
  if (needSpeaker)      ldoEn |= AXP2101_LDO0_DLDO1_SPK_MASK;  else ldoEn &= (uint8_t)(~AXP2101_LDO0_DLDO1_SPK_MASK);  // DLDO1 (bit 7): Speaker
  speakerAmpPowered = needSpeaker;
  writeRegI2C(Wire, AXP2101_ADDR, 0x90, ldoEn);
}

// Apply AXP2101 LDO rails for BL_STATE_OFF Light Sleep:
// Turn OFF DLDO1 (speaker amp), ALDO4 (LoRa), and BLDO2 (haptic rail),
// while never disabling GPS (BLDO1) if GPS is enabled in Settings.
void applyAxp2101LightSleepRails() {
  uint8_t ldoEn = readRegI2C(Wire, AXP2101_ADDR, 0x90);
  ldoEn = (uint8_t)(ldoEn | AXP2101_LDO0_CORE_MASK);
  ldoEn &= (uint8_t)(~(AXP2101_LDO0_DLDO1_SPK_MASK | AXP2101_LDO0_ALDO4_LORA_MASK | AXP2101_LDO0_BLDO2_MASK));
  if (settingPowerGps) {
    ldoEn |= AXP2101_LDO0_BLDO1_GPS_MASK; // Preserve GPS BLDO1 during BL_STATE_OFF when enabled
  } else {
    ldoEn &= (uint8_t)(~AXP2101_LDO0_BLDO1_GPS_MASK);
  }
  speakerAmpPowered = false;
  writeRegI2C(Wire, AXP2101_ADDR, 0x90, ldoEn);
}

// Initialize AXP2101 PMU (configure voltages, charge current, active LDOs; DLDO1 speaker stays OFF until needed)
void initAXP2101() {
  writeRegI2C(Wire, AXP2101_ADDR, 0x92, 0x1C); // ALDO1: 3.3V (+3V3)
  writeRegI2C(Wire, AXP2101_ADDR, 0x93, 0x1C); // ALDO2: 3.3V (LCD_VDD backlight)
  writeRegI2C(Wire, AXP2101_ADDR, 0x94, 0x1C); // ALDO3: 3.3V (LDO3)
  writeRegI2C(Wire, AXP2101_ADDR, 0x95, 0x1C); // ALDO4: 3.3V (LoRa VDD)
  writeRegI2C(Wire, AXP2101_ADDR, 0x96, 0x1C); // BLDO1: 3.3V (GPS VDD)
  writeRegI2C(Wire, AXP2101_ADDR, 0x97, 0x1C); // BLDO2: 3.3V (LDO5 -> DRV2605L EN)
  writeRegI2C(Wire, AXP2101_ADDR, 0x99, 0x1C); // DLDO1: 3.3V (SPK_VDD, gated on demand)

  applyAxp2101PowerSettings();

  uint8_t ldoEn2 = readRegI2C(Wire, AXP2101_ADDR, 0x91);
  writeRegI2C(Wire, AXP2101_ADDR, 0x91, (uint8_t)(ldoEn2 | 0x01));

  writeRegI2C(Wire, AXP2101_ADDR, 0x30, 0xFF);

  delay(50);
}

// Configure DRV2605L registers silently in Standby (used after BLDO2 power-cycle on Light Sleep wake)
void reinitDRV2605LAfterSleep() {
  for (uint8_t attempt = 0; attempt < 5; attempt++) {
    Wire.beginTransmission(DRV2605_ADDR);
    if (Wire.endTransmission() == 0) break;
    delay(5);
  }
  writeRegI2C(Wire, DRV2605_ADDR, 0x01, 0x00); // Exit Standby
  writeRegI2C(Wire, DRV2605_ADDR, 0x02, 0x00); // RTP_INPUT = 0
  writeRegI2C(Wire, DRV2605_ADDR, 0x16, 0x8E); // RATED_VOLTAGE = 3.0 V
  writeRegI2C(Wire, DRV2605_ADDR, 0x17, 0xFF); // OD_CLAMP = max swing
  writeRegI2C(Wire, DRV2605_ADDR, 0x1A, 0x36); // ERM motor
  writeRegI2C(Wire, DRV2605_ADDR, 0x1B, 0x93); // STARTUP_BOOST = 1
  writeRegI2C(Wire, DRV2605_ADDR, 0x1C, 0xF5); // BIDIR_INPUT = 1
  writeRegI2C(Wire, DRV2605_ADDR, 0x1D, 0xB0); // ERM_OPEN_LOOP = 1
  writeRegI2C(Wire, DRV2605_ADDR, 0x01, 0x40); // Return to Standby
}

// Initialize DRV2605L haptic driver for 3.0V ERM 0827 motor (VC0827B005F) and buzz for 250 ms
void initDRV2605L() {
  // Wait for DRV2605L readiness on I2C bus after enabling BLDO2 rail
  for (uint8_t attempt = 0; attempt < 10; attempt++) {
    Wire.beginTransmission(DRV2605_ADDR);
    if (Wire.endTransmission() == 0) break;
    delay(15);
  }

  // Hardware reset and wait for DEV_RESET bit to clear
  writeRegI2C(Wire, DRV2605_ADDR, 0x01, 0x80);
  delay(15);
  for (uint8_t wait = 0; wait < 10; wait++) {
    if ((readRegI2C(Wire, DRV2605_ADDR, 0x01) & 0x80) == 0) break;
    delay(5);
  }

  // Exit Standby mode
  writeRegI2C(Wire, DRV2605_ADDR, 0x01, 0x00);
  delay(5);

  // ERM Open-Loop mode without voltage limiting or overdrive clamp compensation
  writeRegI2C(Wire, DRV2605_ADDR, 0x02, 0x00); // RTP_INPUT = 0
  writeRegI2C(Wire, DRV2605_ADDR, 0x16, 0x8E); // RATED_VOLTAGE = 3.0 V
  writeRegI2C(Wire, DRV2605_ADDR, 0x17, 0xFF); // OD_CLAMP = max swing for Open-Loop ERM
  writeRegI2C(Wire, DRV2605_ADDR, 0x1A, 0x36); // ERM motor (N_ERM_LRA = 0)
  writeRegI2C(Wire, DRV2605_ADDR, 0x1B, 0x93); // STARTUP_BOOST = 1
  writeRegI2C(Wire, DRV2605_ADDR, 0x1C, 0xF5); // Default BIDIR_INPUT = 1
  writeRegI2C(Wire, DRV2605_ADDR, 0x1D, 0xB0); // ERM_OPEN_LOOP = 1, SUPPLY_COMP_DIS = 1, signed RTP

  // Direct drive via Real-Time Playback (MODE = 5) at full amplitude 0x7F (+100%)
  writeRegI2C(Wire, DRV2605_ADDR, 0x01, 0x05); // MODE = 5 (RTP), STANDBY = 0
  delay(2);
  writeRegI2C(Wire, DRV2605_ADDR, 0x02, 0x7F); // +100% power to H-bridge
  delay(BOOT_VIBRATION_MS);                    // Buzz for 250 ms on power-on

  // Stop motor and enter Standby
  writeRegI2C(Wire, DRV2605_ADDR, 0x02, 0x00); // RTP_INPUT = 0
  writeRegI2C(Wire, DRV2605_ADDR, 0x01, 0x40); // STANDBY = 1
}

// Direct control of DRV2605L ERM haptic motor (used by continuous SOS signal)
void setMotorVibrate(bool on) {
  if (on) {
    writeRegI2C(Wire, DRV2605_ADDR, 0x01, 0x05); // MODE = 5 (RTP), STANDBY = 0
    writeRegI2C(Wire, DRV2605_ADDR, 0x02, 0x7F); // +100% drive to ERM motor
  } else {
    writeRegI2C(Wire, DRV2605_ADDR, 0x02, 0x00); // RTP_INPUT = 0
    writeRegI2C(Wire, DRV2605_ADDR, 0x01, 0x40); // STANDBY = 1
  }
}

// Immediately stop any active touch vibration pulse (e.g., before Wi-Fi/BLE scanning)
void stopTouchVibrationNow() {
  if (!sosActive) {
    setMotorVibrate(false);
  }
  touchVibActive = false;
}

// Start a short non-blocking haptic pulse on screen touch (DRV2605L RTP)
void triggerTouchVibration() {
  if (!settingTouchVibration || sosActive) return;
  if (currentScreen == SCREEN_WIFI_CONNECT && wifiView == WIFI_VIEW_SCANNING) return;
  if (currentScreen == SCREEN_BT_SCAN && (btView == BT_VIEW_SCANNING || btScanPending || btScanRunning)) return;
  if (currentScreen == SCREEN_WEATHER && (weatherView == WEATHER_VIEW_LOADING || weatherFetchPending)) return;
  if (currentScreen == SCREEN_SUN_ACTIVITY && (sunView == SUN_VIEW_LOADING || sunFetchPending)) return;
  setMotorVibrate(true);
  touchVibActive = true;
  touchVibEndMs  = millis() + TOUCH_VIB_DURATION_MS;
}

// End touch vibration pulse after TOUCH_VIB_DURATION_MS
void updateTouchVibration() {
  if (sosActive) return;
  if (touchVibActive && (int32_t)(millis() - touchVibEndMs) >= 0) {
    setMotorVibrate(false);
    touchVibActive = false;
  }
}

// Configure BMA423 INT1 (GPIO14) interrupt routing according to Wake-up condition settings
void configureBma423WakeInterrupt() {
  if (!bma423Ready) return;
  // Disable advanced power save briefly while modifying interrupt registers
  writeRegI2C(Wire, bma423Addr, BMA423_REG_PWR_CONF, 0x00);
  delay(2);
  // INT_LATCH (0x55) = 0x00 (non-latched pulse so Light Sleep does not re-trigger continuously)
  writeRegI2C(Wire, bma423Addr, BMA423_REG_INT_LATCH, 0x00);
  if (settingWakeOnBma423) {
    // INT1_IO_CTRL (0x53): output_en=1 (bit 3), push-pull (bit 2=0), active-LOW (bit 1=0), level (bit 0=0) -> 0x08
    writeRegI2C(Wire, bma423Addr, BMA423_REG_INT1_IO_CTRL, 0x08);
    // INT_MAP_1 (0x56): route step_cntr (0x02), tilt (0x08), wakeup/double-tap (0x10), any-motion (0x20) to INT1
    writeRegI2C(Wire, bma423Addr, BMA423_REG_INT_MAP_1, bma423HwPedometerReady ? 0x3A : 0x00);
  } else {
    writeRegI2C(Wire, bma423Addr, BMA423_REG_INT_MAP_1, 0x00);
    writeRegI2C(Wire, bma423Addr, BMA423_REG_INT1_IO_CTRL, 0x00);
  }
  // Clear any pending interrupt flags
  (void)readRegI2C(Wire, bma423Addr, BMA423_REG_INT_STATUS_0);
  // Re-enable BMA423 advanced low-power duty cycling
  writeRegI2C(Wire, bma423Addr, BMA423_REG_PWR_CONF, 0x01);
}

// Read 32-bit hardware step counter directly from BMA423 STEP_COUNTER_0..3 (0x1E..0x21)
uint32_t readBma423HardwareSteps() {
  if (!bma423Ready) return bma423LastHwSteps;
  Wire.beginTransmission(bma423Addr);
  Wire.write(BMA423_REG_STEP_COUNTER_0);
  if (Wire.endTransmission(false) != 0) return bma423LastHwSteps;
  if (Wire.requestFrom(bma423Addr, (uint8_t)4) != 4) return bma423LastHwSteps;
  uint32_t b0 = Wire.read();
  uint32_t b1 = Wire.read();
  uint32_t b2 = Wire.read();
  uint32_t b3 = Wire.read();
  return b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
}

// Synchronize global stepCount from BMA423 built-in hardware pedometer
void syncBma423HardwareSteps() {
  if (!bma423Ready || !bma423HwPedometerReady) return;
  uint32_t rawHwSteps = readBma423HardwareSteps();
  if (rawHwSteps < bma423HwStepZeroOffset) {
    bma423HwStepZeroOffset = 0;
  }
  bma423LastHwSteps = rawHwSteps - bma423HwStepZeroOffset;
  stepCount = stepCountBase + bma423LastHwSteps;
}

// Reset BMA423 built-in hardware step counter and persisted base offset
void resetBma423HardwareSteps() {
  stepCount = 0;
  stepCountBase = 0;
  bma423LastHwSteps = 0;
  bma423HwStepZeroOffset = 0;
#if HAS_SENSORLIB_BMA423
  if (bma423Ready && bma423HwPedometerReady) {
    bma423Sensor.resetPedometer();
  }
#endif
  if (bma423Ready && bma423HwPedometerReady) {
    bma423HwStepZeroOffset = readBma423HardwareSteps();
  }
}

// Initialize and configure BMA423 accelerometer and built-in hardware step counter
void initBMA423() {
  uint8_t candidates[2] = {BMA423_ADDR_LOW, BMA423_ADDR_HIGH};
  bma423Ready = false;
  bma423HwPedometerReady = false;

  for (uint8_t i = 0; i < 2; i++) {
    uint8_t chipId = readRegI2C(Wire, candidates[i], 0x00);
    if (chipId == 0x13) {
      bma423Addr = candidates[i];
      bma423Ready = true;
      break;
    }
  }

  if (!bma423Ready) {
    return;
  }

#if HAS_SENSORLIB_BMA423
  // Load official Bosch BMA423 config stream via SensorLib and enable built-in hardware pedometer
  if (bma423Sensor.begin(Wire, bma423Addr, I2C_SDA_PIN, I2C_SCL_PIN)) {
    bma423Sensor.enablePedometer(true);
    bma423Sensor.resetPedometer();
    uint8_t internalStatus = readRegI2C(Wire, bma423Addr, BMA423_REG_INTERNAL_STATUS);
    if ((internalStatus & 0x0F) == 0x01) {
      bma423HwPedometerReady = true;
    }
  }
#endif

  if (!bma423HwPedometerReady) {
    // Check if BMA423 ASIC feature engine is already initialized in RAM across warm reset
    uint8_t internalStatus = readRegI2C(Wire, bma423Addr, BMA423_REG_INTERNAL_STATUS);
    if ((internalStatus & 0x0F) == 0x01) {
      bma423HwPedometerReady = true;
    } else {
      // Soft-reset and configure low-power 50 Hz accelerometer mode
      writeRegI2C(Wire, bma423Addr, BMA423_REG_CMD, 0xB6);
      delay(20);
    }
  }

  // Configure 50 Hz ODR (0xA7), +/-4g range (0x01), accelerometer enabled (0x04)
  writeRegI2C(Wire, bma423Addr, BMA423_REG_PWR_CONF, 0x00);
  delay(2);
  writeRegI2C(Wire, bma423Addr, 0x40, 0xA7);
  writeRegI2C(Wire, bma423Addr, 0x41, 0x01);
  writeRegI2C(Wire, bma423Addr, BMA423_REG_PWR_CTRL, 0x04);
  delay(5);

  bma423LastHwSteps = bma423HwPedometerReady ? readBma423HardwareSteps() : 0;
  configureBma423WakeInterrupt();
}

// Read internal BMA423 temperature sensor (register 0x22: int8_t + 23 C, 0x80 = invalid)
bool readBma423TempC(float &outTempC) {
  if (!bma423Ready) return false;
  Wire.beginTransmission(bma423Addr);
  Wire.write(0x22);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(bma423Addr, (uint8_t)1) != 1) return false;
  uint8_t rawByte = Wire.read();
  if (rawByte == 0x80) return false;
  outTempC = 23.0f + (float)(int8_t)rawByte;
  return true;
}

// Record 3-axis acceleration sample (m/s^2) into Accel. sensor ring buffer
void recordAccelSample(float ax, float ay, float az) {
  currentAccelX = ax;
  currentAccelY = ay;
  currentAccelZ = az;

  if (accelHistoryCount < ACCEL_HISTORY_MAX) {
    accelHistoryX[accelHistoryCount] = ax;
    accelHistoryY[accelHistoryCount] = ay;
    accelHistoryZ[accelHistoryCount] = az;
    accelHistoryCount++;
  } else {
    for (uint8_t i = 1; i < ACCEL_HISTORY_MAX; i++) {
      accelHistoryX[i - 1] = accelHistoryX[i];
      accelHistoryY[i - 1] = accelHistoryY[i];
      accelHistoryZ[i - 1] = accelHistoryZ[i];
    }
    accelHistoryX[ACCEL_HISTORY_MAX - 1] = ax;
    accelHistoryY[ACCEL_HISTORY_MAX - 1] = ay;
    accelHistoryZ[ACCEL_HISTORY_MAX - 1] = az;
  }
}

// Background BMA423 step counter (hardware pedometer when active, low-power fallback otherwise)
void updateStepCounter() {
  if (!bma423Ready || currentScreen == SCREEN_ACCEL_SPEC) return;

  uint32_t now = millis();

  // When BMA423 built-in hardware pedometer is active, sync steps at 1 Hz without 50 Hz CPU polling
  if (bma423HwPedometerReady) {
    if (now - lastHwStepSyncMs >= 1000UL) {
      lastHwStepSyncMs = now;
      syncBma423HardwareSteps();
      if (now - lastStepSaveMs >= STEP_SAVE_INTERVAL_MS) {
        saveStepCountToFlash(false);
      }
    }
    // Only poll raw XYZ registers at high rate when Accel/Inclinometer screens require live vectors
    bool needLiveXyz = (currentScreen == SCREEN_ACCEL || currentScreen == SCREEN_INCLINOMETER);
    uint32_t xyzPollPeriod = needLiveXyz ? 25UL : 1000UL;
    if (now - lastStepPollMs < xyzPollPeriod) return;
    lastStepPollMs = now;
  } else {
    if (now - lastStepPollMs < 20UL) return;
    lastStepPollMs = now;
  }

  Wire.beginTransmission(bma423Addr);
  Wire.write(0x12);
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom(bma423Addr, (uint8_t)6) != 6) return;

  uint8_t xl = Wire.read();
  uint8_t xh = Wire.read();
  uint8_t yl = Wire.read();
  uint8_t yh = Wire.read();
  uint8_t zl = Wire.read();
  uint8_t zh = Wire.read();

  int16_t rawX = (int16_t)(((uint16_t)xh << 8) | xl) >> 4;
  int16_t rawY = (int16_t)(((uint16_t)yh << 8) | yl) >> 4;
  int16_t rawZ = (int16_t)(((uint16_t)zh << 8) | zl) >> 4;

  float gx = (float)rawX / 512.0f;
  float gy = (float)rawY / 512.0f;
  float gz = (float)rawZ / 512.0f;

  // Convert acceleration to m/s^2 and record history for Accel. sensor screen
  currentAccelX = gx * GRAVITY_MPS2;
  currentAccelY = gy * GRAVITY_MPS2;
  currentAccelZ = gz * GRAVITY_MPS2;
  uint32_t accelSamplePeriod = (currentScreen == SCREEN_ACCEL) ? 200UL : 1000UL;
  if (now - lastAccelSampleMs >= accelSamplePeriod) {
    lastAccelSampleMs = now;
    recordAccelSample(currentAccelX, currentAccelY, currentAccelZ);
  }

  // Smooth gravity vector and compute tilt angles for Inclinometer (X and Y swapped, X inverted)
  inclFiltGx = inclFiltGx * 0.78f + gx * 0.22f;
  inclFiltGy = inclFiltGy * 0.78f + gy * 0.22f;
  inclFiltGz = inclFiltGz * 0.78f + gz * 0.22f;
  inclAngleX = -atan2f(inclFiltGy, sqrtf(inclFiltGx * inclFiltGx + inclFiltGz * inclFiltGz)) * RAD_TO_DEG;
  inclAngleY = atan2f(inclFiltGx, sqrtf(inclFiltGy * inclFiltGy + inclFiltGz * inclFiltGz)) * RAD_TO_DEG;

  if (!bma423HwPedometerReady) {
    float mag = sqrtf(gx * gx + gy * gy + gz * gz);

    stepBaselineMag = stepBaselineMag * 0.96f + mag * 0.04f;
    stepFilteredMag = stepFilteredMag * 0.65f + mag * 0.35f;
    float dynAcc = stepFilteredMag - stepBaselineMag;

    if (!stepPeakActive && dynAcc > 0.18f && (now - lastStepDetectMs >= 280UL)) {
      stepCount++;
      stepPeakActive = true;
      lastStepDetectMs = now;
    } else if (stepPeakActive && dynAcc < 0.06f) {
      stepPeakActive = false;
    }

    // Automatically save step count to Flash every 10 minutes
    if (now - lastStepSaveMs >= STEP_SAVE_INTERVAL_MS) {
      saveStepCountToFlash(false);
    }
  }
}

// Read battery charge percentage from AXP2101 built-in Fuel Gauge
uint8_t readBatteryPercent() {
  uint8_t pct = readRegI2C(Wire, AXP2101_ADDR, 0xA4);
  if (pct > 100) {
    pct = 100;
  }
  return pct;
}

// Detect active battery charging state from AXP2101 status registers (0x00, 0x01)
bool readBatteryCharging() {
  uint8_t st1 = readRegI2C(Wire, AXP2101_ADDR, 0x00);
  uint8_t st2 = readRegI2C(Wire, AXP2101_ADDR, 0x01);
  bool vbusGood = ((st1 & 0x20) != 0);
  uint8_t chgDir = (uint8_t)((st2 >> 5) & 0x03); // 1 = Charging
  uint8_t chgState = (uint8_t)(st2 & 0x07);      // 0..3 = Tri/Pre/CC/CV active charging
  return vbusGood && (chgDir == 1 || chgState <= 3);
}

// Read 14-bit AXP2101 PMU ADC channel (1 mV/LSB)
uint16_t readAxp2101Adc14(uint8_t regHigh) {
  uint8_t hi = readRegI2C(Wire, AXP2101_ADDR, regHigh);
  uint8_t lo = readRegI2C(Wire, AXP2101_ADDR, (uint8_t)(regHigh + 1));
  return (uint16_t)(((uint16_t)(hi & 0x3F) << 8) | lo);
}

// Read complete battery and power rail telemetry from AXP2101
void readAxp2101PowerInfo(Axp2101PowerInfo &info) {
  uint8_t st1 = readRegI2C(Wire, AXP2101_ADDR, 0x00);
  uint8_t st2 = readRegI2C(Wire, AXP2101_ADDR, 0x01);

  info.vbusGood     = ((st1 & 0x20) != 0);
  info.batPresent   = ((st1 & 0x08) != 0);
  info.chgDirection = (uint8_t)((st2 >> 5) & 0x03);
  info.chgState     = (uint8_t)(st2 & 0x07);

  info.batPercent    = readBatteryPercent();
  info.batVoltageMv  = readAxp2101Adc14(0x34);
  info.vbusVoltageMv = readAxp2101Adc14(0x38);
  info.vsysVoltageMv = readAxp2101Adc14(0x3A);

  uint16_t rawTemp = readAxp2101Adc14(0x3C);
  float tDie = 22.0f + (7274.0f - (float)rawTemp) / 20.0f;
  if (tDie < -40.0f || tDie > 125.0f) tDie = 0.0f;
  info.dieTempC = tDie;

  info.dcdcMask = readRegI2C(Wire, AXP2101_ADDR, 0x80);
  info.ldoMask0 = readRegI2C(Wire, AXP2101_ADDR, 0x90);
  info.ldoMask1 = readRegI2C(Wire, AXP2101_ADDR, 0x91);
}

// Read AXP2101 PMU internal die temperature in deg C (14-bit ADC reg 0x3C/0x3D)
bool readAxp2101TempC(float &outTempC) {
  uint16_t rawTemp = readAxp2101Adc14(0x3C);
  if (rawTemp == 0 || rawTemp == 0x3FFF) return false;
  float tDie = 22.0f + (7274.0f - (float)rawTemp) / 20.0f;
  if (tDie < -40.0f || tDie > 125.0f) return false;
  outTempC = tDie;
  return true;
}

// BCD <-> DEC conversion for PCF8563 real-time clock
uint8_t bcdToDec(uint8_t val) { return ((val / 16) * 10) + (val % 16); }
uint8_t decToBcd(uint8_t val) { return ((val / 10) * 16) + (val % 10); }

// Read current time and date from PCF8563 RTC
void readRTC(DateTimeRTC &dt) {
  Wire.beginTransmission(PCF8563_ADDR);
  Wire.write(0x02);
  if (Wire.endTransmission(false) != 0) return;

  if (Wire.requestFrom((uint8_t)PCF8563_ADDR, (uint8_t)7) == 7) {
    dt.second    = bcdToDec(Wire.read() & 0x7F);
    dt.minute    = bcdToDec(Wire.read() & 0x7F);
    dt.hour      = bcdToDec(Wire.read() & 0x3F);
    dt.day       = bcdToDec(Wire.read() & 0x3F);
    dt.dayOfWeek = bcdToDec(Wire.read() & 0x07);
    uint8_t rawM = Wire.read();
    dt.month     = bcdToDec(rawM & 0x1F);
    dt.year      = 2000 + bcdToDec(Wire.read());

    if (dt.month < 1 || dt.month > 12) dt.month = 1;
    if (dt.day < 1 || dt.day > 31)     dt.day = 1;
    if (dt.hour > 23)                  dt.hour = 0;
    if (dt.minute > 59)                dt.minute = 0;
    if (dt.second > 59)                dt.second = 0;
    if (dt.dayOfWeek > 6)              dt.dayOfWeek = 0;
  }
}

// Write edited time and date to PCF8563 RTC
void writeRTC(const DateTimeRTC &dt) {
  Wire.beginTransmission(PCF8563_ADDR);
  Wire.write(0x02);
  Wire.write(decToBcd(dt.second));
  Wire.write(decToBcd(dt.minute));
  Wire.write(decToBcd(dt.hour));
  Wire.write(decToBcd(dt.day));
  Wire.write(decToBcd(dt.dayOfWeek));
  Wire.write(decToBcd(dt.month));
  Wire.write(decToBcd(dt.year % 100));
  Wire.endTransmission();
}

// ============================================================================
// 5. FT6236 / FT6336U TOUCHSCREEN AND IR TRANSMITTER (NEC / RC-5 / RC-6)
// ============================================================================

// Clamp touchscreen sensitivity directly to FT6236 / FT6336U register 0x80 (THGROUP: 30..80, default 50)
uint8_t touchSensitivityToThGroup(uint8_t sensVal) {
  if (sensVal < TOUCH_SENS_MIN_VAL) sensVal = TOUCH_SENS_MIN_VAL;
  if (sensVal > TOUCH_SENS_MAX_VAL) sensVal = TOUCH_SENS_MAX_VAL;
  return sensVal;
}

// Put FT6336U into full-rate Active Mode using saved sensitivity from Flash
void setTouchActiveMode() {
  uint8_t thGroup = touchSensitivityToThGroup(settingTouchSensitivity);
  writeRegI2C(Wire1, FT6336_ADDR, FT6336_REG_PMODE, FT6336_PMODE_ACTIVE); // PMODE (0xA5): Active Mode
  delay(2);
  writeRegI2C(Wire1, FT6336_ADDR, 0x80, thGroup);                         // THGROUP (0x80): sensitivity (30..80)
  writeRegI2C(Wire1, FT6336_ADDR, 0x85, 0xA0);                            // THDIFF: touch point jitter filter
  writeRegI2C(Wire1, FT6336_ADDR, FT6336_REG_CTRL, 0x00);                 // CTRL (0x86): stay in active mode
  writeRegI2C(Wire1, FT6336_ADDR, FT6336_REG_PERIODACTIVE, 0x06);         // PERIODACTIVE (0x88): 60 Hz scan
  writeRegI2C(Wire1, FT6336_ADDR, FT6336_REG_G_MODE, 0x00);               // G_MODE (0xA4): level trigger on TOUCH_INT
  touchInMonitorMode = false;
}

// Put FT6336U into low-power Monitor Mode while preserving TOUCH_INT_PIN pulses for 2x/3x tap wake
void setTouchMonitorMode() {
  uint8_t thGroup = touchSensitivityToThGroup(settingTouchSensitivity);
  writeRegI2C(Wire1, FT6336_ADDR, 0x80, thGroup);
  writeRegI2C(Wire1, FT6336_ADDR, FT6336_REG_G_MODE, 0x00);               // G_MODE (0xA4): LOW while finger touches
  writeRegI2C(Wire1, FT6336_ADDR, FT6336_REG_TIMEENTERMONITOR, 0x01);     // Enter monitor after 1 s idle
  writeRegI2C(Wire1, FT6336_ADDR, FT6336_REG_PERIODMONITOR, 0x19);        // Monitor scan period (low power, ~40 Hz)
  writeRegI2C(Wire1, FT6336_ADDR, FT6336_REG_CTRL, 0x01);                 // CTRL (0x86): allow switch to monitor
  writeRegI2C(Wire1, FT6336_ADDR, FT6336_REG_PMODE, FT6336_PMODE_MONITOR);// PMODE (0xA5): Monitor Mode (0x01)
  touchInMonitorMode = true;
}

// Put FT6336U into deepest Hibernate Mode when touch wake-up is disabled in Settings
void setTouchHibernateMode() {
  writeRegI2C(Wire1, FT6336_ADDR, FT6336_REG_PMODE, FT6336_PMODE_HIBERNATE); // PMODE (0xA5): Hibernate (0x03)
  touchInMonitorMode = true;
}

// Initialize FT6236 / FT6336U touchscreen controller using saved sensitivity from Flash
void initTouchScreen() {
  setTouchActiveMode();
}

// Detect 2 or 3 consecutive taps via TOUCH_INT_PIN (IO16) while FT6336U is in Monitor Mode
bool detectMultiTapOnTouchInt(uint8_t requiredTaps) {
  if (requiredTaps != 3) requiredTaps = 2;
  uint8_t tapCount = 1; // First tap already triggered the GPIO wakeup (TOUCH_INT_PIN == LOW)

  while (tapCount < requiredTaps) {
    // 1. Wait for current tap finger release (TOUCH_INT_PIN -> HIGH)
    uint32_t pressStart = millis();
    while (digitalRead(TOUCH_INT_PIN) == LOW) {
      if (millis() - pressStart > DOUBLE_TAP_MAX_MS) {
        return false; // Long press / pocket contact is not a tap
      }
      delay(4);
    }
    // Release debounce
    delay(20);

    // 2. Wait for next tap press edge (TOUCH_INT_PIN -> LOW) within DOUBLE_TAP_MAX_MS
    uint32_t gapStart = millis();
    bool gotNextTap = false;
    while (millis() - gapStart <= DOUBLE_TAP_MAX_MS) {
      if (digitalRead(TOUCH_INT_PIN) == LOW) {
        // Confirm stable touch pulse (>= 10 ms)
        delay(10);
        if (digitalRead(TOUCH_INT_PIN) == LOW) {
          gotNextTap = true;
          break;
        }
      }
      delay(4);
    }
    if (!gotNextTap) {
      return false;
    }
    tapCount++;
  }

  return true;
}

bool getTouchPoint(int16_t &tx, int16_t &ty) {
  Wire1.beginTransmission(FT6336_ADDR);
  Wire1.write(0x02);
  if (Wire1.endTransmission(false) != 0) return false;

  if (Wire1.requestFrom((uint8_t)FT6336_ADDR, (uint8_t)5) != 5) return false;

  uint8_t touches = Wire1.read() & 0x0F;
  if (touches == 0 || touches > 2) return false;

  uint8_t xh = Wire1.read();
  uint8_t xl = Wire1.read();
  uint8_t yh = Wire1.read();
  uint8_t yl = Wire1.read();

  int16_t rawX = ((int16_t)(xh & 0x0F) << 8) | xl;
  int16_t rawY = ((int16_t)(yh & 0x0F) << 8) | yl;

  tx = constrain(rawX, (int16_t)0, (int16_t)239);
  ty = constrain(rawY, (int16_t)0, (int16_t)239);
  return true;
}

// Configure hardware PWM (LEDC) for stable IR carrier (33, 36, or 38 kHz) on GPIO2
// Called strictly AFTER tft.init() so it does not hijack the shared LEDC clock from Light_PWM
void initIrLedc(uint8_t freqKhz) {
  if (freqKhz < 30 || freqKhz > 60) freqKhz = 38;
  if (irLedcInitialized) {
    if (irLedcFreqKhz != freqKhz) {
      ledc_set_freq(IR_LEDC_MODE, IR_LEDC_TIMER, (uint32_t)freqKhz * 1000UL);
      irLedcFreqKhz = freqKhz;
    }
    return;
  }

  ledc_timer_config_t timerCfg = {};
  timerCfg.speed_mode      = IR_LEDC_MODE;
  timerCfg.duty_resolution = IR_LEDC_DUTY_RES;
  timerCfg.timer_num       = IR_LEDC_TIMER;
  timerCfg.freq_hz         = (uint32_t)freqKhz * 1000UL;
  // Prefer APB_CLK (80 MHz), compatible with Light_PWM (12 kHz) on ESP32-S3;
  // if core already locked a different shared LEDC clock, fall back cleanly to LEDC_AUTO_CLK
  timerCfg.clk_cfg         = LEDC_USE_APB_CLK;
  if (ledc_timer_config(&timerCfg) != ESP_OK) {
    timerCfg.clk_cfg       = LEDC_AUTO_CLK;
    ledc_timer_config(&timerCfg);
  }

  ledc_channel_config_t chanCfg = {};
  chanCfg.gpio_num   = IR_SEND_PIN;
  chanCfg.speed_mode = IR_LEDC_MODE;
  chanCfg.channel    = IR_LEDC_CHANNEL;
  chanCfg.intr_type  = LEDC_INTR_DISABLE;
  chanCfg.timer_sel  = IR_LEDC_TIMER;
  chanCfg.duty       = 0;
  chanCfg.hpoint     = 0;
  ledc_channel_config(&chanCfg);

  irLedcInitialized = true;
  irLedcFreqKhz     = freqKhz;
}

// Generate IR burst using hardware PWM (33% duty cycle, exact duration via esp_timer_get_time)
void sendIrMark(uint32_t durationUs) {
  ledc_set_duty(IR_LEDC_MODE, IR_LEDC_CHANNEL, 85);
  ledc_update_duty(IR_LEDC_MODE, IR_LEDC_CHANNEL);
  int64_t endTime = esp_timer_get_time() + (int64_t)durationUs;
  while (esp_timer_get_time() < endTime) {
  }
  ledc_set_duty(IR_LEDC_MODE, IR_LEDC_CHANNEL, 0);
  ledc_update_duty(IR_LEDC_MODE, IR_LEDC_CHANNEL);
}

// Space (IR LED off) for the specified duration in microseconds
void sendIrSpace(uint32_t durationUs) {
  ledc_set_duty(IR_LEDC_MODE, IR_LEDC_CHANNEL, 0);
  ledc_update_duty(IR_LEDC_MODE, IR_LEDC_CHANNEL);
  int64_t endTime = esp_timer_get_time() + (int64_t)durationUs;
  while (esp_timer_get_time() < endTime) {
  }
}

// IR LED test: 5 crisp flashes (150 ms on / 150 ms off) visible on a phone camera
void testIrLedBlink() {
  initIrLedc(irCarrierKhz);
  for (uint8_t i = 0; i < 5; i++) {
    ledc_set_duty(IR_LEDC_MODE, IR_LEDC_CHANNEL, 128);
    ledc_update_duty(IR_LEDC_MODE, IR_LEDC_CHANNEL);
    delay(150);
    ledc_set_duty(IR_LEDC_MODE, IR_LEDC_CHANNEL, 0);
    ledc_update_duty(IR_LEDC_MODE, IR_LEDC_CHANNEL);
    delay(150);
  }
}

// Send 32-bit hex code via NEC protocol over IR LED
void sendIrNecCode(uint32_t code) {
  initIrLedc(irCarrierKhz);
  sendIrMark(9000UL);
  sendIrSpace(4500UL);

  for (int8_t bitIdx = 31; bitIdx >= 0; bitIdx--) {
    sendIrMark(562UL);
    if ((code >> bitIdx) & 1UL) {
      sendIrSpace(1687UL);
    } else {
      sendIrSpace(562UL);
    }
  }

  sendIrMark(562UL);
  sendIrSpace(40000UL);
}

// Send code via RC-5 protocol (Manchester encoding, 14-bit, 889 us half-bit)
void sendIrRc5Code(uint32_t code) {
  initIrLedc(irCarrierKhz);
  uint16_t frame;
  if (code > 0x7FFUL) {
    frame = (uint16_t)(code & 0x3FFFUL);
  } else {
    frame = (uint16_t)(0x3000U | (irToggleBit ? 0x0800U : 0x0000U) | (code & 0x07FFU));
  }
  irToggleBit = !irToggleBit;

  for (int8_t bitIdx = 13; bitIdx >= 0; bitIdx--) {
    if ((frame >> bitIdx) & 1U) {
      sendIrSpace(889UL);
      sendIrMark(889UL);
    } else {
      sendIrMark(889UL);
      sendIrSpace(889UL);
    }
  }
  sendIrSpace(25000UL);
}

// Send code via RC-6 protocol (Mode 0, 444 us base unit)
void sendIrRc6Code(uint32_t code) {
  initIrLedc(irCarrierKhz);
  sendIrMark(2666UL);
  sendIrSpace(889UL);

  sendIrMark(444UL);
  sendIrSpace(444UL);

  for (uint8_t m = 0; m < 3; m++) {
    sendIrSpace(444UL);
    sendIrMark(444UL);
  }

  if (irToggleBit) {
    sendIrMark(889UL);
    sendIrSpace(889UL);
  } else {
    sendIrSpace(889UL);
    sendIrMark(889UL);
  }
  irToggleBit = !irToggleBit;

  int8_t totalBits = 16;
  if (code > 0xFFFFFUL) {
    totalBits = 32;
  } else if (code > 0xFFFFUL) {
    totalBits = 20;
  }

  for (int8_t bitIdx = (int8_t)(totalBits - 1); bitIdx >= 0; bitIdx--) {
    if ((code >> bitIdx) & 1UL) {
      sendIrMark(444UL);
      sendIrSpace(444UL);
    } else {
      sendIrSpace(444UL);
      sendIrMark(444UL);
    }
  }

  sendIrSpace(2666UL);
}

// ============================================================================
// 6. GEOMETRY RENDERING: ROUNDED HEXAGONS AND VECTOR ICONS
// ============================================================================

// Draw filled rounded hexagon with border outline
void drawRoundedHexagon(int16_t cx, int16_t cy, int16_t radius, int16_t cornerR, uint16_t fillColor, uint16_t borderColor) {
  int16_t innerR = radius - cornerR;
  float vx[6], vy[6];

  for (int8_t i = 0; i < 6; i++) {
    float angle = (i * 60.0f - 90.0f) * DEG_TO_RAD;
    vx[i] = cx + innerR * cosf(angle);
    vy[i] = cy + innerR * sinf(angle);
  }

  for (int8_t i = 0; i < 6; i++) {
    int8_t next = (i + 1) % 6;
    canvas.fillTriangle(cx, cy, (int16_t)vx[i], (int16_t)vy[i], (int16_t)vx[next], (int16_t)vy[next], fillColor);
  }

  for (int8_t i = 0; i < 6; i++) {
    int8_t next = (i + 1) % 6;
    canvas.fillCircle((int16_t)vx[i], (int16_t)vy[i], cornerR, fillColor);

    float dx = vx[next] - vx[i];
    float dy = vy[next] - vy[i];
    float len = sqrtf(dx * dx + dy * dy);
    float nx = (dy / len) * cornerR;
    float ny = (-dx / len) * cornerR;

    int16_t x1 = (int16_t)(vx[i] + nx),    y1 = (int16_t)(vy[i] + ny);
    int16_t x2 = (int16_t)(vx[next] + nx), y2 = (int16_t)(vy[next] + ny);
    int16_t x3 = (int16_t)vx[next],        y3 = (int16_t)vy[next];
    int16_t x4 = (int16_t)vx[i],           y4 = (int16_t)vy[i];

    canvas.fillTriangle(x1, y1, x2, y2, x3, y3, fillColor);
    canvas.fillTriangle(x1, y1, x3, y3, x4, y4, fillColor);
    canvas.drawLine(x1, y1, x2, y2, borderColor);
  }

  for (int8_t i = 0; i < 6; i++) {
    float startAngle = i * 60.0f - 120.0f;
    for (int16_t a = 0; a <= 60; a += 6) {
      float rad1 = (startAngle + a) * DEG_TO_RAD;
      float rad2 = (startAngle + a + 6) * DEG_TO_RAD;
      int16_t ax1 = (int16_t)(vx[i] + cornerR * cosf(rad1));
      int16_t ay1 = (int16_t)(vy[i] + cornerR * sinf(rad1));
      int16_t ax2 = (int16_t)(vx[i] + cornerR * cosf(rad2));
      int16_t ay2 = (int16_t)(vy[i] + cornerR * sinf(rad2));
      canvas.drawLine(ax1, ay1, ax2, ay2, borderColor);
    }
  }
}

// Draw 9 unique vector icons for main menu page 1
void drawMenuIconSymbol(int8_t index, int16_t cx, int16_t cy) {
  switch (index) {
    case 0: { // Calendar: calendar sheet with header bar, rings, and date grid
      canvas.drawRoundRect(cx - 11, cy - 10, 22, 22, 3, COLOR_TEXT);
      canvas.fillRect(cx - 10, cy - 9, 20, 6, COLOR_RED);
      canvas.fillRect(cx - 7, cy - 13, 3, 5, COLOR_SUBTEXT);
      canvas.fillRect(cx + 4, cy - 13, 3, 5, COLOR_SUBTEXT);
      for (int8_t r = 0; r < 2; r++) {
        for (int8_t c = 0; c < 3; c++) {
          uint16_t dotCol = (r == 0 && c == 1) ? COLOR_ACCENT : COLOR_SUBTEXT;
          canvas.fillRect(cx - 7 + c * 6, cy + r * 5, 3, 3, dotCol);
        }
      }
      break;
    }
    case 1: { // Calc
      canvas.drawRoundRect(cx - 10, cy - 13, 20, 26, 3, COLOR_TEXT);
      canvas.fillRect(cx - 7, cy - 10, 14, 5, COLOR_ACCENT);
      for (int8_t r = 0; r < 2; r++) {
        for (int8_t c = 0; c < 2; c++) {
          canvas.fillRect(cx - 6 + c * 8, cy - 2 + r * 6, 4, 4, COLOR_SUBTEXT);
        }
      }
      break;
    }
    case 2: { // Clock
      canvas.drawCircle(cx, cy, 12, COLOR_ACCENT);
      canvas.drawCircle(cx, cy, 11, COLOR_ACCENT);
      canvas.drawLine(cx, cy, cx, cy - 7, COLOR_TEXT);
      canvas.drawLine(cx, cy, cx + 6, cy + 2, COLOR_WARN);
      canvas.fillCircle(cx, cy, 2, COLOR_TEXT);
      break;
    }
    case 3: { // GPS
      canvas.drawCircle(cx, cy - 3, 7, COLOR_GREEN);
      canvas.fillCircle(cx, cy - 3, 3, COLOR_TEXT);
      canvas.fillTriangle(cx - 6, cy, cx + 6, cy, cx, cy + 12, COLOR_GREEN);
      canvas.fillCircle(cx, cy - 3, 2, COLOR_BG);
      break;
    }
    case 4: { // IR (Infrared)
      canvas.fillRoundRect(cx - 9, cy - 5, 8, 10, 3, COLOR_RED);
      canvas.drawLine(cx - 12, cy - 2, cx - 9, cy - 2, COLOR_SUBTEXT);
      canvas.drawLine(cx - 12, cy + 2, cx - 9, cy + 2, COLOR_SUBTEXT);
      canvas.drawArc(cx - 2, cy, 7, 6, 315, 45, COLOR_WARN);
      canvas.drawArc(cx - 2, cy, 12, 11, 315, 45, COLOR_WARN);
      break;
    }
    case 5: { // Step counter
      canvas.fillEllipse(cx - 5, cy - 4, 3, 6, COLOR_ACCENT);
      canvas.fillCircle(cx - 5, cy + 5, 2, COLOR_ACCENT);
      canvas.fillEllipse(cx + 5, cy + 1, 3, 6, COLOR_GREEN);
      canvas.fillCircle(cx + 5, cy + 10, 2, COLOR_GREEN);
      break;
    }
    case 6: { // Inclinometer: level gauge and tilted artificial horizon with bubble
      canvas.drawCircle(cx, cy, 12, COLOR_ACCENT);
      canvas.drawLine(cx, cy - 12, cx, cy - 9, COLOR_TEXT);
      canvas.drawLine(cx - 12, cy, cx - 9, cy, COLOR_SUBTEXT);
      canvas.drawLine(cx + 9, cy, cx + 12, cy, COLOR_SUBTEXT);
      canvas.drawLine(cx - 9, cy + 4, cx + 9, cy - 4, COLOR_WARN);
      canvas.drawLine(cx - 9, cy + 5, cx + 9, cy - 3, COLOR_WARN);
      canvas.fillCircle(cx + 2, cy - 1, 2, COLOR_GREEN);
      break;
    }
    case 7: { // Planet Earth
      canvas.drawCircle(cx, cy, 12, COLOR_ACCENT);
      canvas.drawEllipse(cx, cy, 6, 12, COLOR_ACCENT);
      canvas.drawLine(cx - 12, cy, cx + 12, cy, COLOR_ACCENT);
      canvas.drawLine(cx - 10, cy - 6, cx + 10, cy - 6, COLOR_GREEN);
      canvas.drawLine(cx - 10, cy + 6, cx + 10, cy + 6, COLOR_GREEN);
      break;
    }
    case 8: { // Settings
      for (int8_t i = 0; i < 8; i++) {
        float ang = i * 45.0f * DEG_TO_RAD;
        int16_t tx = cx + (int16_t)(10.0f * cosf(ang));
        int16_t ty = cy + (int16_t)(10.0f * sinf(ang));
        canvas.fillCircle(tx, ty, 3, COLOR_SUBTEXT);
      }
      canvas.fillCircle(cx, cy, 9, COLOR_SUBTEXT);
      canvas.fillCircle(cx, cy, 4, COLOR_HEX_FILL);
      break;
    }
  }
}

// Draw 9 unique vector icons for main menu page 2
void drawMenuPage2IconSymbol(int8_t index, int16_t cx, int16_t cy) {
  switch (index) {
    case 0: { // Wi-Fi connect: Wi-Fi arcs and green connection checkmark
      canvas.fillCircle(cx - 2, cy + 7, 2, COLOR_ACCENT);
      canvas.drawArc(cx - 2, cy + 7, 7, 6, 225, 315, COLOR_ACCENT);
      canvas.drawArc(cx - 2, cy + 7, 12, 11, 225, 315, COLOR_ACCENT);
      canvas.drawLine(cx + 4, cy + 5, cx + 7, cy + 8, COLOR_GREEN);
      canvas.drawLine(cx + 7, cy + 8, cx + 12, cy + 1, COLOR_GREEN);
      canvas.drawLine(cx + 7, cy + 9, cx + 12, cy + 2, COLOR_GREEN);
      break;
    }
    case 1: { // Wi-Fi list: Wi-Fi symbol and network list
      canvas.fillCircle(cx - 7, cy + 4, 1, COLOR_ACCENT);
      canvas.drawArc(cx - 7, cy + 4, 5, 4, 225, 315, COLOR_ACCENT);
      canvas.drawArc(cx - 7, cy + 4, 9, 8, 225, 315, COLOR_ACCENT);
      canvas.fillRect(cx + 1, cy - 8, 10, 2, COLOR_TEXT);
      canvas.fillRect(cx + 1, cy - 2, 10, 2, COLOR_TEXT);
      canvas.fillRect(cx + 1, cy + 4, 10, 2, COLOR_SUBTEXT);
      canvas.fillRect(cx + 1, cy + 9, 7,  2, COLOR_SUBTEXT);
      break;
    }
    case 2: { // Wi-Fi analyzer: bar chart channel analyzer
      canvas.drawLine(cx - 12, cy + 10, cx + 12, cy + 10, COLOR_SUBTEXT);
      canvas.fillRect(cx - 10, cy + 2,  4, 8,  COLOR_ACCENT);
      canvas.fillRect(cx - 4,  cy - 8,  4, 18, COLOR_GREEN);
      canvas.fillRect(cx + 2,  cy - 2,  4, 12, COLOR_WARN);
      canvas.fillRect(cx + 8,  cy + 4,  4, 6,  COLOR_ACCENT);
      break;
    }
    case 3: { // Wi-Fi info: Wi-Fi arcs and info badge "i"
      canvas.fillCircle(cx - 5, cy + 6, 1, COLOR_ACCENT);
      canvas.drawArc(cx - 5, cy + 6, 5, 4, 225, 315, COLOR_ACCENT);
      canvas.drawArc(cx - 5, cy + 6, 9, 8, 225, 315, COLOR_ACCENT);
      canvas.drawCircle(cx + 6, cy + 1, 6, COLOR_GREEN);
      canvas.fillRect(cx + 5, cy - 2, 2, 2, COLOR_TEXT);
      canvas.fillRect(cx + 5, cy + 1, 2, 4, COLOR_TEXT);
      break;
    }
    case 4: { // Wi-Fi jam detect: alert triangle and interference indicator
      canvas.drawTriangle(cx, cy - 12, cx - 12, cy + 10, cx + 12, cy + 10, COLOR_WARN);
      canvas.fillRect(cx - 1, cy - 5, 2, 8, COLOR_RED);
      canvas.fillRect(cx - 1, cy + 5, 2, 2, COLOR_RED);
      break;
    }
    case 5: { // Hosts scan: network hub and 3 connected hosts
      canvas.drawLine(cx, cy - 4, cx - 9, cy + 5, COLOR_SUBTEXT);
      canvas.drawLine(cx, cy - 4, cx,     cy + 5, COLOR_SUBTEXT);
      canvas.drawLine(cx, cy - 4, cx + 9, cy + 5, COLOR_SUBTEXT);
      canvas.drawRoundRect(cx - 5, cy - 10, 10, 6, 2, COLOR_ACCENT);
      canvas.fillCircle(cx, cy - 7, 1, COLOR_GREEN);
      canvas.fillRoundRect(cx - 12, cy + 5, 6, 5, 1, COLOR_GREEN);
      canvas.fillRoundRect(cx - 3,  cy + 5, 6, 5, 1, COLOR_ACCENT);
      canvas.fillRoundRect(cx + 6,  cy + 5, 6, 5, 1, COLOR_WARN);
      break;
    }
    case 6: { // Bluetooth scan: Bluetooth rune symbol
      canvas.drawLine(cx, cy - 11, cx, cy + 11, COLOR_ACCENT);
      canvas.drawLine(cx, cy - 11, cx + 7, cy - 5, COLOR_ACCENT);
      canvas.drawLine(cx + 7, cy - 5, cx - 6, cy + 5, COLOR_ACCENT);
      canvas.drawLine(cx - 6, cy - 5, cx + 7, cy + 5, COLOR_ACCENT);
      canvas.drawLine(cx + 7, cy + 5, cx, cy + 11, COLOR_ACCENT);
      break;
    }
    case 7: { // Sound spectrometer: audio equalizer spectrum bars
      canvas.fillRect(cx - 11, cy + 1,  3, 9,  COLOR_ACCENT);
      canvas.fillRect(cx - 6,  cy - 7,  3, 17, COLOR_GREEN);
      canvas.fillRect(cx - 1,  cy - 11, 3, 21, COLOR_WARN);
      canvas.fillRect(cx + 4,  cy - 5,  3, 15, COLOR_GREEN);
      canvas.fillRect(cx + 9,  cy + 3,  3, 7,  COLOR_ACCENT);
      break;
    }
    case 8: { // Sound generator: speaker and sine wave
      canvas.fillRect(cx - 12, cy - 3, 4, 6, COLOR_TEXT);
      canvas.fillTriangle(cx - 8, cy - 3, cx - 8, cy + 3, cx - 3, cy - 8, COLOR_TEXT);
      canvas.fillTriangle(cx - 8, cy + 3, cx - 3, cy - 8, cx - 3, cy + 8, COLOR_TEXT);
      canvas.drawLine(cx,     cy,     cx + 3, cy - 7, COLOR_ACCENT);
      canvas.drawLine(cx + 3, cy - 7, cx + 6, cy,     COLOR_ACCENT);
      canvas.drawLine(cx + 6, cy,     cx + 9, cy + 7, COLOR_GREEN);
      canvas.drawLine(cx + 9, cy + 7, cx + 12, cy,    COLOR_GREEN);
      break;
    }
  }
}

// Draw 9 unique vector icons for main menu page 3
void drawMenuPage3IconSymbol(int8_t index, int16_t cx, int16_t cy) {
  switch (index) {
    case 0: { // ASCII table: grid card with letter A and hex code 41
      canvas.drawRoundRect(cx - 12, cy - 10, 24, 20, 2, COLOR_ACCENT);
      canvas.drawLine(cx - 12, cy - 3, cx + 12, cy - 3, COLOR_HEX_BORDER);
      canvas.drawLine(cx, cy - 10, cx, cy + 10, COLOR_HEX_BORDER);
      canvas.setTextDatum(middle_center);
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
      canvas.drawString("A", cx - 6, cy + 3);
      canvas.setTextColor(COLOR_WARN, COLOR_HEX_FILL);
      canvas.drawString("41", cx + 6, cy + 3);
      break;
    }
    case 1: { // Periodic table: Mendeleev cell grid (H/He, s/d/p blocks and f-row)
      int16_t x0 = cx - 13;
      int16_t y0 = cy - 11;
      // Period 1: H (left) and He (right)
      canvas.fillRect(x0 + 0 * 4, y0, 3, 3, COLOR_ACCENT);
      canvas.fillRect(x0 + 6 * 4, y0, 3, 3, COLOR_WARN);
      // Period 2: s-block (2 cells) + gap + p-block (3 cells)
      canvas.fillRect(x0 + 0 * 4, y0 + 4, 3, 3, COLOR_RED);
      canvas.fillRect(x0 + 1 * 4, y0 + 4, 3, 3, COLOR_WARN);
      canvas.fillRect(x0 + 4 * 4, y0 + 4, 3, 3, COLOR_ACCENT);
      canvas.fillRect(x0 + 5 * 4, y0 + 4, 3, 3, COLOR_ACCENT);
      canvas.fillRect(x0 + 6 * 4, y0 + 4, 3, 3, COLOR_WARN);
      // Periods 3 and 4: full cell rows (s + d + p blocks with dividers)
      for (int8_t r = 2; r <= 3; r++) {
        int16_t ry = y0 + r * 4;
        canvas.fillRect(x0 + 0 * 4, ry, 3, 3, COLOR_RED);
        canvas.fillRect(x0 + 1 * 4, ry, 3, 3, COLOR_WARN);
        canvas.fillRect(x0 + 2 * 4, ry, 3, 3, COLOR_GREEN);
        canvas.fillRect(x0 + 3 * 4, ry, 3, 3, COLOR_GREEN);
        canvas.fillRect(x0 + 4 * 4, ry, 3, 3, COLOR_GREEN);
        canvas.fillRect(x0 + 5 * 4, ry, 3, 3, COLOR_ACCENT);
        canvas.fillRect(x0 + 6 * 4, ry, 3, 3, COLOR_WARN);
      }
      // Bottom separated lanthanide / actinide strip (f-block)
      for (int8_t c = 1; c <= 5; c++) {
        canvas.fillRect(x0 + c * 4, y0 + 18, 3, 2, (c % 2 == 0) ? 0xF81F : COLOR_SUBTEXT);
      }
      break;
    }
    case 2: { // Food e-additives: badge circle and letter E
      canvas.drawCircle(cx, cy, 11, COLOR_GREEN);
      canvas.drawCircle(cx, cy, 10, COLOR_GREEN);
      canvas.setTextDatum(middle_center);
      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(COLOR_WARN, COLOR_HEX_FILL);
      canvas.drawString("E", cx, cy + 1);
      break;
    }
    case 3: { // Temp. sensor: thermometer with red mercury column
      canvas.drawRoundRect(cx - 3, cy - 12, 6, 17, 3, COLOR_TEXT);
      canvas.fillCircle(cx, cy + 7, 5, COLOR_RED);
      canvas.fillRect(cx - 1, cy - 4, 2, 10, COLOR_RED);
      canvas.drawLine(cx + 5, cy - 8, cx + 9, cy - 8, COLOR_SUBTEXT);
      canvas.drawLine(cx + 5, cy - 3, cx + 8, cy - 3, COLOR_SUBTEXT);
      canvas.drawLine(cx + 5, cy + 2, cx + 9, cy + 2, COLOR_SUBTEXT);
      break;
    }
    case 4: { // Accel. sensor: axes and vibration peak waveform
      canvas.drawLine(cx - 11, cy - 10, cx - 11, cy + 9, COLOR_SUBTEXT);
      canvas.drawLine(cx - 11, cy + 9,  cx + 11, cy + 9, COLOR_SUBTEXT);
      canvas.drawLine(cx - 10, cy + 6,  cx - 6,  cy + 6, COLOR_ACCENT);
      canvas.drawLine(cx - 6,  cy + 6,  cx - 3,  cy - 8, COLOR_WARN);
      canvas.drawLine(cx - 3,  cy - 8,  cx + 1,  cy + 7, COLOR_WARN);
      canvas.drawLine(cx + 1,  cy + 7,  cx + 5,  cy - 3, COLOR_GREEN);
      canvas.drawLine(cx + 5,  cy - 3,  cx + 9,  cy + 6, COLOR_ACCENT);
      break;
    }
    case 5: { // Sun activity: sun disc with rays
      canvas.fillCircle(cx, cy, 6, 0xFFE0);
      canvas.drawCircle(cx, cy, 7, COLOR_WARN);
      for (int8_t i = 0; i < 8; i++) {
        float ang = i * 45.0f * DEG_TO_RAD;
        int16_t x1 = cx + (int16_t)(9.0f * cosf(ang));
        int16_t y1 = cy + (int16_t)(9.0f * sinf(ang));
        int16_t x2 = cx + (int16_t)(12.0f * cosf(ang));
        int16_t y2 = cy + (int16_t)(12.0f * sinf(ang));
        canvas.drawLine(x1, y1, x2, y2, COLOR_WARN);
      }
      break;
    }
    case 6: { // Weather: sun, cloud, and raindrops
      canvas.fillCircle(cx - 5, cy - 5, 5, 0xFFE0);
      canvas.fillCircle(cx - 3, cy + 1, 5, COLOR_TEXT);
      canvas.fillCircle(cx + 3, cy - 1, 6, COLOR_TEXT);
      canvas.fillCircle(cx + 8, cy + 2, 4, COLOR_TEXT);
      canvas.fillRect(cx - 4, cy + 1, 13, 5, COLOR_TEXT);
      canvas.drawLine(cx - 4, cy + 8, cx - 6, cy + 11, COLOR_ACCENT);
      canvas.drawLine(cx + 2, cy + 8, cx,     cy + 11, COLOR_ACCENT);
      canvas.drawLine(cx + 8, cy + 8, cx + 6, cy + 11, COLOR_ACCENT);
      break;
    }
    case 7: { // System info: ESP32 IC package with pins
      canvas.drawRoundRect(cx - 7, cy - 7, 14, 14, 2, COLOR_ACCENT);
      canvas.fillRect(cx - 3, cy - 3, 6, 6, COLOR_GREEN);
      for (int8_t p = -4; p <= 4; p += 4) {
        canvas.drawLine(cx - 11, cy + p, cx - 8, cy + p, COLOR_SUBTEXT);
        canvas.drawLine(cx + 8,  cy + p, cx + 11, cy + p, COLOR_SUBTEXT);
        canvas.drawLine(cx + p, cy - 11, cx + p, cy - 8, COLOR_SUBTEXT);
        canvas.drawLine(cx + p, cy + 8,  cx + p, cy + 11, COLOR_SUBTEXT);
      }
      break;
    }
    case 8: { // About firmware: info circle with "i" symbol
      canvas.drawCircle(cx, cy, 11, COLOR_ACCENT);
      canvas.drawCircle(cx, cy, 10, COLOR_ACCENT);
      canvas.fillRect(cx - 1, cy - 6, 3, 3, COLOR_TEXT);
      canvas.fillRect(cx - 1, cy - 1, 3, 8, COLOR_TEXT);
      break;
    }
  }
}

// Vector icons for 9 items on main menu page 4
void drawMenuPage4IconSymbol(int8_t index, int16_t cx, int16_t cy) {
  switch (index) {
    case 0: { // Alarm: alarm clock with twin bells
      canvas.drawCircle(cx, cy + 1, 9, COLOR_ACCENT);
      canvas.fillCircle(cx - 7, cy - 8, 3, COLOR_WARN);
      canvas.fillCircle(cx + 7, cy - 8, 3, COLOR_WARN);
      canvas.drawLine(cx - 6, cy + 10, cx - 9, cy + 12, COLOR_SUBTEXT);
      canvas.drawLine(cx + 6, cy + 10, cx + 9, cy + 12, COLOR_SUBTEXT);
      canvas.drawLine(cx, cy + 1, cx, cy - 4, COLOR_TEXT);
      canvas.drawLine(cx, cy + 1, cx + 4, cy + 3, COLOR_WARN);
      canvas.fillCircle(cx, cy + 1, 1, COLOR_TEXT);
      break;
    }
    case 1: { // Stopwatch: stopwatch with top crown and side pusher
      canvas.fillRect(cx - 3, cy - 12, 6, 2, COLOR_ACCENT);
      canvas.drawLine(cx, cy - 10, cx, cy - 8, COLOR_ACCENT);
      canvas.drawLine(cx + 7, cy - 6, cx + 10, cy - 9, COLOR_WARN);
      canvas.drawCircle(cx, cy + 2, 9, COLOR_TEXT);
      canvas.drawLine(cx, cy + 2, cx + 5, cy - 3, COLOR_GREEN);
      canvas.fillCircle(cx, cy + 2, 2, COLOR_ACCENT);
      break;
    }
    case 2: { // Timer: hourglass
      canvas.fillRect(cx - 8, cy - 10, 16, 2, COLOR_ACCENT);
      canvas.fillRect(cx - 8, cy + 9, 16, 2, COLOR_ACCENT);
      canvas.drawLine(cx - 7, cy - 8, cx + 7, cy - 8, COLOR_TEXT);
      canvas.drawLine(cx - 7, cy - 8, cx, cy, COLOR_TEXT);
      canvas.drawLine(cx + 7, cy - 8, cx, cy, COLOR_TEXT);
      canvas.drawLine(cx, cy, cx - 7, cy + 8, COLOR_TEXT);
      canvas.drawLine(cx, cy, cx + 7, cy + 8, COLOR_TEXT);
      canvas.drawLine(cx - 7, cy + 8, cx + 7, cy + 8, COLOR_TEXT);
      canvas.fillTriangle(cx - 4, cy - 6, cx + 4, cy - 6, cx, cy - 1, COLOR_WARN);
      canvas.fillTriangle(cx, cy + 3, cx - 5, cy + 7, cx + 5, cy + 7, COLOR_WARN);
      break;
    }
    case 3: { // Flashlight: flashlight with upward light beams
      canvas.fillRect(cx - 3, cy, 6, 11, COLOR_SUBTEXT);
      canvas.fillTriangle(cx - 6, cy - 4, cx + 6, cy - 4, cx + 3, cy + 1, COLOR_ACCENT);
      canvas.fillTriangle(cx - 6, cy - 4, cx + 3, cy + 1, cx - 3, cy + 1, COLOR_ACCENT);
      canvas.fillRect(cx - 1, cy + 3, 2, 3, COLOR_WARN);
      canvas.drawLine(cx, cy - 6, cx, cy - 12, 0xFFE0);
      canvas.drawLine(cx - 4, cy - 6, cx - 8, cy - 11, 0xFFE0);
      canvas.drawLine(cx + 4, cy - 6, cx + 8, cy - 11, 0xFFE0);
      break;
    }
    case 4: { // SOS signal: red beacon with SOS label and radio waves
      canvas.fillRoundRect(cx - 11, cy - 6, 22, 13, 4, COLOR_RED);
      canvas.setTextDatum(middle_center);
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(COLOR_TEXT, COLOR_RED);
      canvas.drawString("SOS", cx, cy + 1);
      canvas.drawArc(cx, cy, 13, 12, 220, 320, COLOR_WARN);
      canvas.drawArc(cx, cy, 13, 12, 40, 140, COLOR_WARN);
      break;
    }
    case 5: { // Morse code: telegraph dots and dashes
      canvas.fillCircle(cx - 8, cy - 4, 2, COLOR_ACCENT);
      canvas.fillRoundRect(cx - 3, cy - 6, 8, 4, 1, COLOR_WARN);
      canvas.fillCircle(cx + 8, cy - 4, 2, COLOR_ACCENT);
      canvas.fillRoundRect(cx - 10, cy + 3, 7, 4, 1, COLOR_GREEN);
      canvas.fillCircle(cx, cy + 5, 2, COLOR_TEXT);
      canvas.fillRoundRect(cx + 4, cy + 3, 7, 4, 1, COLOR_GREEN);
      break;
    }
    case 6: { // Tetris: multicolored tetromino blocks
      canvas.fillRect(cx - 9, cy + 4, 5, 5, COLOR_ACCENT);
      canvas.fillRect(cx - 3, cy + 4, 5, 5, COLOR_ACCENT);
      canvas.fillRect(cx + 3, cy + 4, 5, 5, COLOR_WARN);
      canvas.fillRect(cx + 3, cy - 2, 5, 5, COLOR_WARN);
      canvas.fillRect(cx - 9, cy - 2, 5, 5, COLOR_GREEN);
      canvas.fillRect(cx - 3, cy - 2, 5, 5, 0xF81F);
      canvas.fillRect(cx - 3, cy - 8, 5, 5, 0xF81F);
      canvas.fillRect(cx + 3, cy - 8, 5, 5, 0xFFE0);
      break;
    }
    case 7: { // Paint: artist palette with paint dots and brush
      canvas.drawCircle(cx - 1, cy + 1, 10, COLOR_TEXT);
      canvas.fillCircle(cx - 5, cy - 3, 2, COLOR_RED);
      canvas.fillCircle(cx + 1, cy - 5, 2, 0xFFE0);
      canvas.fillCircle(cx + 5, cy - 1, 2, COLOR_GREEN);
      canvas.fillCircle(cx - 5, cy + 4, 2, COLOR_ACCENT);
      canvas.fillCircle(cx + 3, cy + 5, 2, COLOR_BG);
      canvas.drawLine(cx + 3, cy + 2, cx + 11, cy - 9, COLOR_WARN);
      break;
    }
    case 8: { // Accel. spectrometer: vibration spectrum bars + acceleration wave
      int16_t bars[5] = {6, 12, 16, 10, 7};
      uint16_t cols[5] = {COLOR_ACCENT, COLOR_GREEN, COLOR_WARN, COLOR_GREEN, COLOR_ACCENT};
      for (int8_t b = 0; b < 5; b++) {
        int16_t bx = cx - 10 + b * 4;
        canvas.fillRect(bx, cy + 9 - bars[b], 3, bars[b], cols[b]);
      }
      canvas.drawLine(cx - 10, cy - 7, cx - 5, cy - 10, COLOR_RED);
      canvas.drawLine(cx - 5, cy - 10, cx, cy - 5, COLOR_RED);
      canvas.drawLine(cx, cy - 5, cx + 5, cy - 10, COLOR_RED);
      canvas.drawLine(cx + 5, cy - 10, cx + 10, cy - 7, COLOR_RED);
      break;
    }
  }
}

// Draw icon symbols for main menu page 5 (Bluetooth keyboard, Bluetooth mouse, Bluetooth presenter)
void drawMenuPage5IconSymbol(int8_t index, int16_t cx, int16_t cy) {
  switch (index) {
    case 0: { // Bluetooth keyboard: compact keyboard chassis + keys + Bluetooth rune
      canvas.drawRoundRect(cx - 12, cy - 2, 24, 13, 2, COLOR_TEXT);
      for (int8_t k = 0; k < 5; k++) {
        canvas.fillRect(cx - 10 + k * 4, cy + 1, 3, 2, COLOR_ACCENT);
        canvas.fillRect(cx - 10 + k * 4, cy + 4, 3, 2, COLOR_SUBTEXT);
      }
      canvas.fillRect(cx - 6, cy + 8, 12, 2, COLOR_TEXT);
      // Mini Bluetooth rune above keyboard
      int16_t bx = cx;
      int16_t by = cy - 8;
      canvas.drawLine(bx, by - 4, bx, by + 4, COLOR_ACCENT);
      canvas.drawLine(bx, by - 4, bx + 3, by - 2, COLOR_ACCENT);
      canvas.drawLine(bx + 3, by - 2, bx - 3, by + 2, COLOR_ACCENT);
      canvas.drawLine(bx - 3, by - 2, bx + 3, by + 2, COLOR_ACCENT);
      canvas.drawLine(bx + 3, by + 2, bx, by + 4, COLOR_ACCENT);
      break;
    }
    case 1: { // Bluetooth mouse: rounded mouse shell + scroll wheel + Bluetooth rune
      canvas.drawRoundRect(cx - 7, cy - 5, 14, 17, 6, COLOR_TEXT);
      canvas.drawLine(cx - 6, cy + 1, cx + 6, cy + 1, COLOR_SUBTEXT);
      canvas.drawLine(cx, cy - 5, cx, cy + 1, COLOR_SUBTEXT);
      canvas.fillRect(cx - 1, cy - 3, 2, 3, COLOR_ACCENT);
      // Mini Bluetooth rune above mouse
      int16_t bx = cx;
      int16_t by = cy - 10;
      canvas.drawLine(bx, by - 3, bx, by + 3, COLOR_ACCENT);
      canvas.drawLine(bx, by - 3, bx + 3, by - 1, COLOR_ACCENT);
      canvas.drawLine(bx + 3, by - 1, bx - 2, by + 2, COLOR_ACCENT);
      canvas.drawLine(bx - 2, by - 1, bx + 3, by + 2, COLOR_ACCENT);
      canvas.drawLine(bx + 3, by + 2, bx, by + 3, COLOR_ACCENT);
      break;
    }
    case 2: { // Bluetooth presenter: handheld slide clicker + laser emitter + Bluetooth rune
      canvas.drawRoundRect(cx - 5, cy - 4, 10, 17, 4, COLOR_TEXT);
      // Red laser emitter dot & beam at top of clicker
      canvas.fillCircle(cx, cy - 4, 1, COLOR_RED);
      canvas.drawLine(cx, cy - 6, cx, cy - 11, COLOR_RED);
      canvas.drawLine(cx - 3, cy - 9, cx - 1, cy - 7, COLOR_WARN);
      canvas.drawLine(cx + 3, cy - 9, cx + 1, cy - 7, COLOR_WARN);
      // Slide prev/next buttons on presenter body
      canvas.fillRect(cx - 3, cy, 2, 3, COLOR_ACCENT);
      canvas.fillRect(cx + 1, cy, 2, 3, COLOR_ACCENT);
      canvas.fillCircle(cx, cy + 7, 1, COLOR_GREEN);
      // Mini Bluetooth rune to the right
      int16_t bx = cx + 9;
      int16_t by = cy - 6;
      canvas.drawLine(bx, by - 3, bx, by + 3, COLOR_ACCENT);
      canvas.drawLine(bx, by - 3, bx + 2, by - 1, COLOR_ACCENT);
      canvas.drawLine(bx + 2, by - 1, bx - 2, by + 1, COLOR_ACCENT);
      canvas.drawLine(bx - 2, by - 1, bx + 2, by + 1, COLOR_ACCENT);
      canvas.drawLine(bx + 2, by + 1, bx, by + 3, COLOR_ACCENT);
      break;
    }
    case 3: { // Color table: 3x2 vibrant RGB/CMYK swatch palette grid
      canvas.drawRoundRect(cx - 12, cy - 9, 24, 18, 3, COLOR_TEXT);
      canvas.fillRect(cx - 10, cy - 7, 6, 6, 0xF800); // Red
      canvas.fillRect(cx - 3,  cy - 7, 6, 6, 0x07E0); // Green
      canvas.fillRect(cx + 4,  cy - 7, 6, 6, 0x001F); // Blue
      canvas.fillRect(cx - 10, cy + 1, 6, 6, 0x07FF); // Cyan
      canvas.fillRect(cx - 3,  cy + 1, 6, 6, 0xF81F); // Magenta
      canvas.fillRect(cx + 4,  cy + 1, 6, 6, 0xFFE0); // Yellow
      break;
    }
    case 4: { // Resistor table: axial resistor body with metallic leads and 4 color bands
      canvas.drawFastHLine(cx - 14, cy, 5, COLOR_SUBTEXT);
      canvas.drawFastHLine(cx - 14, cy + 1, 5, COLOR_SUBTEXT);
      canvas.drawFastHLine(cx + 9, cy, 5, COLOR_SUBTEXT);
      canvas.drawFastHLine(cx + 9, cy + 1, 5, COLOR_SUBTEXT);
      canvas.fillRoundRect(cx - 9, cy - 5, 18, 11, 3, 0xD5B1); // Tan ceramic body
      canvas.drawRoundRect(cx - 9, cy - 5, 18, 11, 3, COLOR_TEXT);
      canvas.fillRect(cx - 6, cy - 4, 2, 9, 0xF800); // Band 1: Red
      canvas.fillRect(cx - 2, cy - 4, 2, 9, 0x981F); // Band 2: Violet
      canvas.fillRect(cx + 2, cy - 4, 2, 9, 0xFD20); // Multiplier: Orange
      canvas.fillRect(cx + 6, cy - 4, 2, 9, 0xFEA0); // Tolerance: Gold
      break;
    }
    case 5: { // Voltage divider: Vin -> R1 -> Vout tap -> R2 -> GND schematic symbol
      // Top Vin dot and lead
      canvas.fillCircle(cx - 3, cy - 11, 2, COLOR_WARN);
      canvas.drawFastVLine(cx - 3, cy - 9, 3, COLOR_TEXT);
      // Top resistor R1
      canvas.drawRect(cx - 6, cy - 6, 6, 5, COLOR_ACCENT);
      // Middle junction & Vout tap arrow right
      canvas.drawFastVLine(cx - 3, cy - 1, 3, COLOR_TEXT);
      canvas.fillCircle(cx - 3, cy, 2, COLOR_GREEN);
      canvas.drawFastHLine(cx - 1, cy, 9, COLOR_GREEN);
      canvas.fillCircle(cx + 8, cy, 2, COLOR_GREEN);
      // Bottom resistor R2
      canvas.drawRect(cx - 6, cy + 2, 6, 5, COLOR_ACCENT);
      // Ground bars
      canvas.drawFastVLine(cx - 3, cy + 7, 3, COLOR_TEXT);
      canvas.drawFastHLine(cx - 7, cy + 10, 9, COLOR_SUBTEXT);
      canvas.drawFastHLine(cx - 5, cy + 12, 5, COLOR_SUBTEXT);
      break;
    }
  }
}

// ============================================================================
// 7. COMMON UI ELEMENTS AND SCREEN RENDERING
// ============================================================================

// Top status bar (Wi-Fi icon left of time, HH:MM, and battery charge %)
void drawTopStatusBar() {
  canvas.fillRect(0, 0, 240, 28, COLOR_TOPBAR);

  // Wi-Fi status icon left of time (green = connected, gray crossed with red = disconnected)
  bool wifiConnected = (WiFi.status() == WL_CONNECTED);
  uint16_t wifiCol = wifiConnected ? COLOR_GREEN : COLOR_SUBTEXT;
  int16_t wx = 53;
  int16_t wy = 20;
  canvas.fillCircle(wx, wy, 1, wifiCol);
  canvas.drawArc(wx, wy, 5, 4, 225, 315, wifiCol);
  canvas.drawArc(wx, wy, 9, 8, 225, 315, wifiCol);
  canvas.drawArc(wx, wy, 13, 12, 225, 315, wifiCol);
  if (!wifiConnected) {
    canvas.drawLine(wx - 9, wy - 12, wx + 9, wy + 2, COLOR_RED);
    canvas.drawLine(wx - 8, wy - 12, wx + 10, wy + 2, COLOR_RED);
  }

  char statusBuf[24];
  snprintf(statusBuf, sizeof(statusBuf), "%02u:%02u  %u%%",
           rtcTime.hour, rtcTime.minute, batteryPercent);

  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_TEXT, COLOR_TOPBAR);
  canvas.drawString(statusBuf, 118, 14);

  int16_t bx = 166, by = 9;
  uint16_t batFrameCol = batteryCharging ? 0xFFE0 : COLOR_SUBTEXT;
  canvas.drawRect(bx, by, 16, 10, batFrameCol);
  canvas.fillRect(bx + 16, by + 3, 2, 4, batFrameCol);
  int16_t fillW = (batteryPercent * 12) / 100;
  uint16_t batColor = (batteryPercent > 20) ? COLOR_GREEN : COLOR_RED;
  if (fillW > 0) {
    canvas.fillRect(bx + 2, by + 2, fillW, 6, batColor);
  }
  // Lightning bolt overlay on battery icon while charging
  if (batteryCharging) {
    int16_t lx = bx + 8;
    int16_t ly = by + 5;
    // Dark outline for crisp contrast over the green battery fill
    canvas.drawLine(lx + 2, ly - 5, lx - 2, ly, COLOR_BG);
    canvas.drawLine(lx - 2, ly, lx + 2, ly, COLOR_BG);
    canvas.drawLine(lx + 2, ly, lx - 2, ly + 5, COLOR_BG);
    // Bright yellow lightning bolt (2 triangles)
    canvas.fillTriangle(lx + 1, ly - 4, lx - 3, ly + 1, lx + 1, ly + 1, 0xFFE0);
    canvas.fillTriangle(lx - 1, ly - 1, lx + 3, ly - 1, lx - 1, ly + 4, 0xFFE0);
  }
}

// Left arrow button in top-left corner of main menu
void drawTopLeftArrowIcon() {
  canvas.fillRoundRect(4, 2, 32, 24, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(4, 2, 32, 24, 6, COLOR_ACCENT);
  canvas.fillTriangle(12, 14, 25, 7, 25, 21, COLOR_ACCENT);
}

// Right arrow button in top-right corner of main menu
void drawTopRightArrowIcon() {
  canvas.fillRoundRect(204, 2, 32, 24, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(204, 2, 32, 24, 6, COLOR_ACCENT);
  canvas.fillTriangle(228, 14, 215, 7, 215, 21, COLOR_ACCENT);
}

// Close X icon in top-left corner to exit to main menu
void drawTopLeftCloseIcon() {
  canvas.fillRoundRect(4, 2, 32, 24, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(4, 2, 32, 24, 6, COLOR_RED);
  canvas.drawLine(14, 8, 26, 20, COLOR_RED);
  canvas.drawLine(15, 8, 26, 19, COLOR_RED);
  canvas.drawLine(26, 8, 14, 20, COLOR_RED);
  canvas.drawLine(25, 8, 14, 19, COLOR_RED);
}

// Gear icon in top-right corner (Clock and IR)
void drawTopRightGearIcon(bool active) {
  uint16_t borderCol = active ? COLOR_WARN : COLOR_ACCENT;
  uint16_t gearCol   = active ? COLOR_WARN : COLOR_TEXT;

  canvas.fillRoundRect(204, 2, 32, 24, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(204, 2, 32, 24, 6, borderCol);

  int16_t cx = 220, cy = 14;
  for (int8_t i = 0; i < 8; i++) {
    float a = i * 45.0f * DEG_TO_RAD;
    int16_t gx = cx + (int16_t)(7.0f * cosf(a));
    int16_t gy = cy + (int16_t)(7.0f * sinf(a));
    canvas.fillCircle(gx, gy, 2, gearCol);
  }
  canvas.fillCircle(cx, cy, 6, gearCol);
  canvas.fillCircle(cx, cy, 3, COLOR_HEX_FILL);
}

// Letter F icon in top-right corner to cycle 5 calculator pages
void drawTopRightFuncIcon(bool active) {
  uint16_t borderCol = COLOR_ACCENT;
  uint16_t textCol   = COLOR_TEXT;
  const char* label  = "F";
  if (active) {
    if (calcPage == 1) {
      borderCol = COLOR_WARN;
      textCol   = COLOR_WARN;
      label     = "F2";
    } else if (calcPage == 2) {
      borderCol = COLOR_GREEN;
      textCol   = COLOR_GREEN;
      label     = "F3";
    } else if (calcPage == 3) {
      borderCol = 0xF81F;
      textCol   = 0xF81F;
      label     = "F4";
    } else {
      borderCol = 0xFD20;
      textCol   = 0xFD20;
      label     = "F5";
    }
  }

  canvas.fillRoundRect(204, 2, 32, 24, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(204, 2, 32, 24, 6, borderCol);

  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(textCol, COLOR_HEX_FILL);
  canvas.drawString(label, 220, 14);
}

// Letter S icon in top-right corner of Earth screen to toggle satellite orbits
void drawTopRightSatIcon(bool active) {
  uint16_t borderCol = active ? COLOR_WARN : COLOR_ACCENT;
  uint16_t textCol   = active ? COLOR_WARN : COLOR_TEXT;

  canvas.fillRoundRect(204, 2, 32, 24, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(204, 2, 32, 24, 6, borderCol);

  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(textCol, COLOR_HEX_FILL);
  canvas.drawString("S", 220, 14);
}

// Digit 0 icon in top-right corner to reset step counter
void drawTopRightZeroIcon() {
  canvas.fillRoundRect(204, 2, 32, 24, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(204, 2, 32, 24, 6, COLOR_WARN);

  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_WARN, COLOR_HEX_FILL);
  canvas.drawString("0", 220, 14);
}

// 3-page toggle button in top-right corner of GPS screen (1/3, 2/3, 3/3)
void drawTopRightGpsPageIcon() {
  uint16_t col = COLOR_ACCENT;
  const char* lbl = "1/3";
  if (gpsPage == 1) {
    col = COLOR_GREEN;
    lbl = "2/3";
  } else if (gpsPage == 2) {
    col = COLOR_WARN;
    lbl = "3/3";
  }

  canvas.fillRoundRect(204, 2, 32, 24, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(204, 2, 32, 24, 6, col);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(col, COLOR_HEX_FILL);
  canvas.drawString(lbl, 220, 14);
}

// Get center coordinates of a hexagon in the 3x3 grid by index (0..8)
void getHexGridCenter(int8_t index, int16_t &cx, int16_t &cy) {
  int8_t col = index % 3;
  int8_t row = index / 3;
  cx = 44 + col * 76;  // 44, 120, 196
  cy = 62 + row * 68;  // 62, 130, 198
}

// Draw label below a hexagon (1 or 2 lines for longer titles)
void drawMenuLabel(const char* text, int16_t cx, int16_t y, uint16_t textColor) {
  canvas.setTextDatum(top_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(textColor, COLOR_BG);

  size_t len = strlen(text);
  const char* bestSp = NULL;
  if (len > 10) {
    size_t mid = len / 2;
    size_t bestDist = len;
    for (const char* p = text; *p != '\0'; p++) {
      if (*p == ' ') {
        size_t pos = (size_t)(p - text);
        size_t dist = (pos > mid) ? (pos - mid) : (mid - pos);
        if (dist < bestDist) {
          bestDist = dist;
          bestSp = p;
        }
      }
    }
  }

  if (bestSp != NULL) {
    char line1[24];
    uint8_t len1 = (uint8_t)(bestSp - text);
    if (len1 >= sizeof(line1)) len1 = (uint8_t)(sizeof(line1) - 1);
    memcpy(line1, text, len1);
    line1[len1] = '\0';

    canvas.drawString(line1, cx, y - 1);
    canvas.drawString(bestSp + 1, cx, y + 7);
  } else {
    canvas.drawString(text, cx, y + 1);
  }
}

// Draw main menu (5 pages of rounded hexagons with page navigation arrows)
void drawMainMenuScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftArrowIcon();  // Left arrow for switching main menu pages
  drawTopRightArrowIcon(); // Right arrow for switching main menu pages

  for (int8_t i = 0; i < 9; i++) {
    const char* labelText = MENU_LABELS[i];
    if (mainMenuPage == 0) {
      labelText = MENU_LABELS[i];
    } else if (mainMenuPage == 1) {
      labelText = MENU_LABELS_PAGE2[i];
    } else if (mainMenuPage == 2) {
      labelText = MENU_LABELS_PAGE3[i];
    } else if (mainMenuPage == 3) {
      labelText = MENU_LABELS_PAGE4[i];
    } else {
      labelText = MENU_LABELS_PAGE5[i];
    }
    if (labelText[0] == '\0') continue;

    int16_t cx, cy;
    getHexGridCenter(i, cx, cy);

    // Items on pages 1..4 (0..8) and Bluetooth/reference tools on page 5 (#0..#5) are active
    bool isImplemented = (mainMenuPage <= 3);
    if (mainMenuPage == 4 && i == 0) isImplemented = true;
    if (mainMenuPage == 4 && i == 1) isImplemented = true;
    if (mainMenuPage == 4 && i == 2) isImplemented = true;
    if (mainMenuPage == 4 && i == 3) isImplemented = true;
    if (mainMenuPage == 4 && i == 4) isImplemented = true;
    if (mainMenuPage == 4 && i == 5) isImplemented = true;
    uint16_t fillCol   = isImplemented ? COLOR_HEX_ACTIVE : COLOR_HEX_FILL;
    uint16_t borderCol = isImplemented ? COLOR_ACCENT     : COLOR_HEX_BORDER;

    drawRoundedHexagon(cx, cy - 4, 25, 5, fillCol, borderCol);

    if (mainMenuPage == 0) {
      drawMenuIconSymbol(i, cx, cy - 4);
    } else if (mainMenuPage == 1) {
      drawMenuPage2IconSymbol(i, cx, cy - 4);
    } else if (mainMenuPage == 2) {
      drawMenuPage3IconSymbol(i, cx, cy - 4);
    } else if (mainMenuPage == 3) {
      drawMenuPage4IconSymbol(i, cx, cy - 4);
    } else {
      drawMenuPage5IconSymbol(i, cx, cy - 4);
    }

    drawMenuLabel(labelText, cx, cy + 22, isImplemented ? COLOR_TEXT : COLOR_SUBTEXT);
  }
}

// Compute weekday for a given date (Sakamoto algorithm, 0 = Mon .. 6 = Sun)
uint8_t getCalendarWeekday(uint8_t day, uint8_t month, uint16_t year) {
  static const uint8_t t[12] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  uint16_t y = year;
  if (month < 3) {
    y = (uint16_t)(y - 1);
  }
  uint8_t dowSun0 = (uint8_t)((y + y / 4 - y / 100 + y / 400 + t[month - 1] + day) % 7);
  return (uint8_t)((dowSun0 + 6) % 7);
}

// Draw Calendar screen (SCREEN_CALENDAR) with year and month triangle buttons
void drawCalendarScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon(); // Top-left X button to exit to main menu

  // 1. Year selector row with left/right triangles
  canvas.fillRoundRect(12, 35, 44, 22, 5, COLOR_HEX_FILL);
  canvas.drawRoundRect(12, 35, 44, 22, 5, COLOR_ACCENT);
  canvas.fillTriangle(26, 46, 40, 40, 40, 52, COLOR_ACCENT);

  canvas.fillRoundRect(184, 35, 44, 22, 5, COLOR_HEX_FILL);
  canvas.drawRoundRect(184, 35, 44, 22, 5, COLOR_ACCENT);
  canvas.fillTriangle(214, 46, 200, 40, 200, 52, COLOR_ACCENT);

  char yearBuf[8];
  snprintf(yearBuf, sizeof(yearBuf), "%04u", (uint16_t)calYear);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
  canvas.drawString(yearBuf, 120, 46);

  // 2. Month selector row with left/right triangles
  canvas.fillRoundRect(12, 60, 44, 22, 5, COLOR_HEX_FILL);
  canvas.drawRoundRect(12, 60, 44, 22, 5, COLOR_ACCENT);
  canvas.fillTriangle(26, 71, 40, 65, 40, 77, COLOR_ACCENT);

  canvas.fillRoundRect(184, 60, 44, 22, 5, COLOR_HEX_FILL);
  canvas.drawRoundRect(184, 60, 44, 22, 5, COLOR_ACCENT);
  canvas.fillTriangle(214, 71, 200, 65, 200, 77, COLOR_ACCENT);

  uint8_t mIdx = (calMonth >= 1 && calMonth <= 12) ? (uint8_t)(calMonth - 1) : 0;
  canvas.setTextColor(COLOR_TEXT, COLOR_BG);
  canvas.drawString(CAL_MONTH_NAMES[mIdx], 120, 71);

  // 3. Weekday headers (Mo .. Su) and separator line
  canvas.setFont(&fonts::Font0);
  for (int8_t c = 0; c < 7; c++) {
    int16_t cx = (int16_t)(19 + c * 34);
    uint16_t hdrCol = (c >= 5) ? COLOR_RED : COLOR_SUBTEXT;
    canvas.setTextColor(hdrCol, COLOR_BG);
    canvas.drawString(CAL_WEEKDAYS[c], cx, 92);
  }
  canvas.drawLine(6, 100, 234, 100, COLOR_HEX_BORDER);

  // 4. Day grid (weekends highlighted in red, current date outlined)
  uint8_t firstCol = getCalendarWeekday(1, calMonth, calYear);
  uint8_t totalDays = getDaysInMonth(calMonth, calYear);
  canvas.setFont(&fonts::Font2);

  char dayBuf[4];
  for (uint8_t d = 1; d <= totalDays; d++) {
    uint8_t cellIdx = (uint8_t)(firstCol + d - 1);
    int8_t row = (int8_t)(cellIdx / 7);
    int8_t col = (int8_t)(cellIdx % 7);

    int16_t cx = (int16_t)(19 + col * 34);
    int16_t cy = (int16_t)(113 + row * 22);

    bool isToday = (calYear == rtcTime.year) &&
                   (calMonth == rtcTime.month) &&
                   (d == rtcTime.day);

    snprintf(dayBuf, sizeof(dayBuf), "%u", (uint16_t)d);

    if (isToday) {
      canvas.fillRoundRect(cx - 14, cy - 10, 28, 20, 4, COLOR_HEX_ACTIVE);
      canvas.drawRoundRect(cx - 14, cy - 10, 28, 20, 4, COLOR_ACCENT);
      canvas.setTextColor(COLOR_ACCENT, COLOR_HEX_ACTIVE);
      canvas.drawString(dayBuf, cx, cy);
    } else {
      uint16_t dayCol = (col >= 5) ? COLOR_RED : COLOR_TEXT;
      canvas.setTextColor(dayCol, COLOR_BG);
      canvas.drawString(dayBuf, cx, cy);
    }
  }
}

// Draw Clock screen (view and edit time/date)
void drawClockScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();
  drawTopRightGearIcon(isEditingClock);

  const DateTimeRTC &dt = isEditingClock ? editTime : rtcTime;

  canvas.setTextDatum(top_center);
  canvas.setFont(&fonts::Font2);
  if (isEditingClock) {
    if (clockSyncStatusMsg[0] != '\0' && (millis() - clockSyncStatusMs < 2500UL)) {
      bool syncOk = (strstr(clockSyncStatusMsg, "OK") != NULL);
      canvas.setTextColor(syncOk ? COLOR_GREEN : COLOR_WARN, COLOR_BG);
      canvas.drawString(clockSyncStatusMsg, 120, 38);
    } else {
      canvas.setTextColor(COLOR_WARN, COLOR_BG);
      canvas.drawString("EDIT MODE (Tap value)", 120, 38);
    }
  } else {
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
    canvas.drawString("REAL TIME CLOCK", 120, 38);
  }

  int16_t row1Y = 92;
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font4);
  canvas.setTextColor(COLOR_TEXT, COLOR_BG);

  char buf[16];
  snprintf(buf, sizeof(buf), "%02u", dt.hour);
  canvas.drawString(buf, 28, row1Y);
  canvas.drawString(":", 52, row1Y);

  snprintf(buf, sizeof(buf), "%02u", dt.minute);
  canvas.drawString(buf, 76, row1Y);
  canvas.drawString(":", 100, row1Y);

  snprintf(buf, sizeof(buf), "%02u", dt.second);
  canvas.drawString(buf, 124, row1Y);

  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
  snprintf(buf, sizeof(buf), "UTC%+d", (int16_t)utcOffset);
  canvas.drawString(buf, 192, row1Y);

  int16_t row2Y = 142;
  canvas.setFont(&fonts::Font4);
  canvas.setTextColor(COLOR_TEXT, COLOR_BG);

  snprintf(buf, sizeof(buf), "%02u", dt.day);
  canvas.drawString(buf, 26, row2Y);
  canvas.drawString(".", 48, row2Y);

  snprintf(buf, sizeof(buf), "%02u", dt.month);
  canvas.drawString(buf, 70, row2Y);
  canvas.drawString(".", 92, row2Y);

  snprintf(buf, sizeof(buf), "%04u", dt.year);
  canvas.drawString(buf, 132, row2Y);

  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_GREEN, COLOR_BG);
  canvas.drawString(DOW_NAMES[dt.dayOfWeek % 7], 204, row2Y);

  if (isEditingClock) {
    int16_t ulX = 0, ulW = 30, ulY = row1Y + 16;
    switch (activeField) {
      case FIELD_HOUR:  ulX = 10;  ulW = 36; ulY = row1Y + 16; break;
      case FIELD_MIN:   ulX = 58;  ulW = 36; ulY = row1Y + 16; break;
      case FIELD_SEC:   ulX = 106; ulW = 36; ulY = row1Y + 16; break;
      case FIELD_UTC:   ulX = 160; ulW = 64; ulY = row1Y + 14; break;
      case FIELD_DAY:   ulX = 8;   ulW = 36; ulY = row2Y + 16; break;
      case FIELD_MONTH: ulX = 52;  ulW = 36; ulY = row2Y + 16; break;
      case FIELD_YEAR:  ulX = 98;  ulW = 68; ulY = row2Y + 16; break;
      case FIELD_DOW:   ulX = 180; ulW = 48; ulY = row2Y + 14; break;
    }
    canvas.fillRect(ulX, ulY, ulW, 3, COLOR_WARN);

    // Decrement button (left triangle)
    canvas.fillRoundRect(8, 184, 50, 46, 8, COLOR_HEX_FILL);
    canvas.drawRoundRect(8, 184, 50, 46, 8, COLOR_ACCENT);
    canvas.fillTriangle(21, 207, 43, 194, 43, 220, COLOR_ACCENT);

    // GPS sync button (active when GPS fix is available)
    bool gpsReady = gps.location.isValid() && gps.date.isValid() && gps.time.isValid() && (gps.date.year() >= 2020);
    uint16_t gpsFill = gpsReady ? COLOR_HEX_ACTIVE : COLOR_TOPBAR;
    uint16_t gpsBord = gpsReady ? COLOR_GREEN : COLOR_HEX_BORDER;
    uint16_t gpsText = gpsReady ? COLOR_GREEN : COLOR_SUBTEXT;
    canvas.fillRoundRect(63, 184, 54, 46, 8, gpsFill);
    canvas.drawRoundRect(63, 184, 54, 46, 8, gpsBord);
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(gpsText, gpsFill);
    canvas.drawString("GPS", 90, 207);

    // NTP sync button (active when Wi-Fi is connected)
    bool ntpReady = (WiFi.status() == WL_CONNECTED);
    uint16_t ntpFill = ntpReady ? COLOR_HEX_ACTIVE : COLOR_TOPBAR;
    uint16_t ntpBord = ntpReady ? COLOR_ACCENT : COLOR_HEX_BORDER;
    uint16_t ntpText = ntpReady ? COLOR_ACCENT : COLOR_SUBTEXT;
    canvas.fillRoundRect(123, 184, 54, 46, 8, ntpFill);
    canvas.drawRoundRect(123, 184, 54, 46, 8, ntpBord);
    canvas.setTextColor(ntpText, ntpFill);
    canvas.drawString("NTP", 150, 207);

    // Increment button (right triangle)
    canvas.fillRoundRect(182, 184, 50, 46, 8, COLOR_HEX_FILL);
    canvas.drawRoundRect(182, 184, 50, 46, 8, COLOR_ACCENT);
    canvas.fillTriangle(219, 207, 197, 194, 197, 220, COLOR_ACCENT);
  }
}

// Satellite color by GNSS constellation (GPS, GLONASS, Galileo, BeiDou)
static uint16_t getGpsSystemColor(char sys) {
  if (sys == 'G') return COLOR_GREEN;
  if (sys == 'R') return COLOR_ACCENT;
  if (sys == 'E') return COLOR_WARN;
  return 0xF81F; // BeiDou / other
}

// Signal quality bar color by SNR value (dB-Hz)
static uint16_t getGpsSnrColor(uint8_t snr) {
  if (snr >= 35) return COLOR_GREEN;
  if (snr >= 25) return COLOR_ACCENT;
  if (snr >= 15) return COLOR_WARN;
  if (snr > 0)   return COLOR_RED;
  return COLOR_HEX_BORDER;
}

// Prune stale satellites (not received in NMEA GSV for over GPS_SAT_STALE_MS)
static void pruneStaleGpsSats() {
  uint32_t now = millis();
  uint8_t dst = 0;
  for (uint8_t i = 0; i < gpsSatCount; i++) {
    if ((uint32_t)(now - gpsSats[i].lastSeenMs) <= GPS_SAT_STALE_MS) {
      if (dst != i) gpsSats[dst] = gpsSats[i];
      dst++;
    }
  }
  gpsSatCount = dst;
}

// GPS page 2: polar satellite skyplot (N/E/S/W, elevation 0/30/60/90 deg)
void drawGpsSkyplotScreen() {
  pruneStaleGpsSats();

  const int16_t cx = 120;
  const int16_t cy = 128;
  const int16_t rOuter = 78;
  const int16_t r60    = 52; // 30 deg elevation
  const int16_t r30    = 26; // 60 deg elevation

  // Background and concentric horizon / 30 / 60 degree circles
  canvas.fillCircle(cx, cy, rOuter, COLOR_TOPBAR);
  canvas.drawCircle(cx, cy, rOuter, COLOR_ACCENT);
  canvas.drawCircle(cx, cy, r60, COLOR_HEX_BORDER);
  canvas.drawCircle(cx, cy, r30, COLOR_HEX_BORDER);

  // N-S and W-E crosshairs
  canvas.drawFastVLine(cx, cy - rOuter, rOuter * 2 + 1, COLOR_HEX_BORDER);
  canvas.drawFastHLine(cx - rOuter, cy, rOuter * 2 + 1, COLOR_HEX_BORDER);

  // Cardinal direction labels
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(middle_center);
  canvas.setTextColor(COLOR_WARN, COLOR_BG);
  canvas.drawString("N", cx, 39);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  canvas.drawString("S", cx, 215);
  canvas.drawString("W", 30, cy);
  canvas.drawString("E", 210, cy);

  // Plot each visible satellite on the polar projection
  uint8_t fixCount = 0;
  for (uint8_t i = 0; i < gpsSatCount; i++) {
    const GpsSatelliteInfo &s = gpsSats[i];
    if (s.usedInFix) fixCount++;
    int8_t elev = s.elevation;
    if (elev < 0) elev = 0;
    if (elev > 90) elev = 90;
    float radius = ((90.0f - (float)elev) / 90.0f) * (float)(rOuter - 6);
    float rad = (float)s.azimuth * DEG_TO_RAD;
    int16_t sx = cx + (int16_t)lroundf(radius * sinf(rad));
    int16_t sy = cy - (int16_t)lroundf(radius * cosf(rad));

    uint16_t col = getGpsSystemColor(s.system);
    if (s.snr > 0) {
      canvas.fillCircle(sx, sy, 8, col);
      if (s.usedInFix) {
        canvas.drawCircle(sx, sy, 9, COLOR_TEXT);
      }
      canvas.setTextDatum(middle_center);
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(COLOR_BG, col);
      char prnBuf[6];
      snprintf(prnBuf, sizeof(prnBuf), "%u", (unsigned)s.prn);
      canvas.drawString(prnBuf, sx, sy);
    } else {
      canvas.fillCircle(sx, sy, 7, COLOR_BG);
      canvas.drawCircle(sx, sy, 7, col);
      canvas.setTextDatum(middle_center);
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(col, COLOR_BG);
      char prnBuf[6];
      snprintf(prnBuf, sizeof(prnBuf), "%u", (unsigned)s.prn);
      canvas.drawString(prnBuf, sx, sy);
    }
  }

  if (gpsSatCount == 0) {
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font0);
    bool noUart = (gps.charsProcessed() == 0) || (millis() - lastGpsByteMs > 5000UL);
    canvas.setTextColor(noUart ? COLOR_RED : COLOR_WARN, COLOR_TOPBAR);
    canvas.drawString(noUart ? "No GPS UART" : "Searching sats...", cx, cy);
  }

  // Bottom summary line and constellation legend
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  char statBuf[26];
  snprintf(statBuf, sizeof(statBuf), "View:%u Fix:%u", (unsigned)gpsSatCount, (unsigned)fixCount);
  canvas.drawString(statBuf, 8, 231);

  canvas.setTextDatum(middle_right);
  canvas.setTextColor(COLOR_GREEN, COLOR_BG);
  canvas.drawString("G", 174, 231);
  canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
  canvas.drawString("R", 192, 231);
  canvas.setTextColor(COLOR_WARN, COLOR_BG);
  canvas.drawString("E", 210, 231);
  canvas.setTextColor(0xF81F, COLOR_BG);
  canvas.drawString("B", 228, 231);
}

// GPS page 3: satellite signal quality bar chart (SNR in dB-Hz)
void drawGpsSnrChartScreen() {
  pruneStaleGpsSats();

  // Sort satellite indices by descending SNR (display up to 12 strongest)
  uint8_t order[GPS_MAX_SATS];
  for (uint8_t i = 0; i < gpsSatCount; i++) order[i] = i;
  for (uint8_t i = 0; i < gpsSatCount; i++) {
    for (uint8_t j = (uint8_t)(i + 1); j < gpsSatCount; j++) {
      if (gpsSats[order[j]].snr > gpsSats[order[i]].snr) {
        uint8_t tmp = order[i];
        order[i] = order[j];
        order[j] = tmp;
      }
    }
  }

  uint8_t trackedCount = 0;
  uint16_t snrSum = 0;
  for (uint8_t i = 0; i < gpsSatCount; i++) {
    if (gpsSats[i].snr > 0) {
      snrSum += gpsSats[i].snr;
      trackedCount++;
    }
  }
  uint8_t avgSnr = (trackedCount > 0) ? (uint8_t)(snrSum / trackedCount) : 0;

  // Chart header
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
  canvas.drawString("GPS SNR (dB-Hz)", 8, 36);

  char avgBuf[24];
  snprintf(avgBuf, sizeof(avgBuf), "Avg:%u Trk:%u/%u",
           (unsigned)avgSnr, (unsigned)trackedCount, (unsigned)gpsSatCount);
  canvas.setTextDatum(middle_right);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  canvas.drawString(avgBuf, 234, 36);

  // Chart area (0..60 dB-Hz)
  const int16_t gx = 24;
  const int16_t gy = 46;
  const int16_t gw = 210;
  const int16_t gh = 148;
  canvas.fillRoundRect(gx, gy, gw, gh, 5, COLOR_TOPBAR);
  canvas.drawRoundRect(gx, gy, gw, gh, 5, COLOR_HEX_BORDER);

  const int16_t plotTop = gy + 12;
  const int16_t plotBot = gy + gh - 6;
  const int16_t plotH   = plotBot - plotTop;

  // Horizontal grid lines at 0, 20, 40, 60 dB-Hz
  const uint8_t ticks[4] = {0, 20, 40, 60};
  canvas.setTextDatum(middle_right);
  for (uint8_t t = 0; t < 4; t++) {
    int16_t ty = plotBot - (int16_t)(((int32_t)ticks[t] * plotH) / 60);
    if (t > 0) {
      for (int16_t dx = gx + 3; dx < gx + gw - 3; dx += 4) {
        canvas.drawPixel(dx, ty, COLOR_HEX_BORDER);
      }
    }
    char tb[6];
    snprintf(tb, sizeof(tb), "%u", (unsigned)ticks[t]);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
    canvas.drawString(tb, gx - 3, ty);
  }

  uint8_t showCount = (gpsSatCount < 12) ? gpsSatCount : 12;
  if (showCount == 0) {
    canvas.setTextDatum(middle_center);
    bool noUart = (gps.charsProcessed() == 0) || (millis() - lastGpsByteMs > 5000UL);
    canvas.setTextColor(noUart ? COLOR_RED : COLOR_WARN, COLOR_TOPBAR);
    canvas.drawString(noUart ? "No GPS UART" : "Waiting GSV data...", gx + gw / 2, gy + gh / 2);
  } else {
    const int16_t slotW = (gw - 8) / 12; // 16 px
    const int16_t barW  = 11;
    for (uint8_t i = 0; i < showCount; i++) {
      const GpsSatelliteInfo &s = gpsSats[order[i]];
      uint8_t snrClamped = (s.snr > 60) ? 60 : s.snr;
      int16_t bh = (int16_t)(((int32_t)snrClamped * plotH) / 60);
      int16_t bx = gx + 5 + i * slotW + 2;
      int16_t by = plotBot - bh;
      uint16_t col = getGpsSnrColor(s.snr);

      if (bh > 0) {
        canvas.fillRect(bx, by, barW, bh, col);
        if (s.usedInFix) {
          canvas.drawFastHLine(bx, by, barW, COLOR_TEXT);
        }
      } else {
        canvas.drawRect(bx, plotBot - 3, barW, 3, COLOR_HEX_BORDER);
      }

      // Numeric SNR value above bar
      if (s.snr > 0) {
        char sv[6];
        snprintf(sv, sizeof(sv), "%u", (unsigned)s.snr);
        canvas.setTextDatum(bottom_center);
        canvas.setTextColor(COLOR_TEXT, COLOR_TOPBAR);
        canvas.drawString(sv, bx + barW / 2, (by > gy + 11) ? (by - 1) : (gy + 11));
      }

      // Constellation prefix and PRN below X axis (2 compact lines)
      char sysBuf[2] = {s.system, '\0'};
      char prnBuf[5];
      snprintf(prnBuf, sizeof(prnBuf), "%02u", (unsigned)(s.prn % 100));
      canvas.setTextDatum(top_center);
      canvas.setTextColor(getGpsSystemColor(s.system), COLOR_BG);
      canvas.drawString(sysBuf, bx + barW / 2, gy + gh + 3);
      canvas.setTextColor(s.usedInFix ? COLOR_TEXT : COLOR_SUBTEXT, COLOR_BG);
      canvas.drawString(prnBuf, bx + barW / 2, gy + gh + 12);
    }
  }

  // Bottom signal quality scale
  canvas.setTextDatum(middle_left);
  canvas.fillRect(10, 227, 6, 6, COLOR_RED);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  canvas.drawString("<15", 19, 230);

  canvas.fillRect(54, 227, 6, 6, COLOR_WARN);
  canvas.drawString("15-24", 63, 230);

  canvas.fillRect(112, 227, 6, 6, COLOR_ACCENT);
  canvas.drawString("25-34", 121, 230);

  canvas.fillRect(170, 227, 6, 6, COLOR_GREEN);
  canvas.drawString("35+ dB", 179, 230);
}

// Draw GPS screen (3 pages: telemetry data, satellite skyplot, SNR chart)
void drawGpsScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();
  drawTopRightGpsPageIcon();

  if (gpsPage == 1) {
    drawGpsSkyplotScreen();
    return;
  }
  if (gpsPage == 2) {
    drawGpsSnrChartScreen();
    return;
  }

  canvas.setTextDatum(top_center);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
  canvas.drawString("GPS RECEIVER (MIA-M10Q)", 120, 32);

  canvas.setTextDatum(top_left);
  int16_t startY = 56;
  int16_t stepY  = 25;
  char lineBuf[44];

  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  canvas.drawString("Lat:", 12, startY);
  canvas.setTextColor(COLOR_TEXT, COLOR_BG);
  if (gps.location.isValid()) {
    snprintf(lineBuf, sizeof(lineBuf), "%.6f", gps.location.lat());
  } else {
    snprintf(lineBuf, sizeof(lineBuf), "---.------");
  }
  canvas.drawString(lineBuf, 88, startY);

  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  canvas.drawString("Lon:", 12, startY + stepY);
  canvas.setTextColor(COLOR_TEXT, COLOR_BG);
  if (gps.location.isValid()) {
    snprintf(lineBuf, sizeof(lineBuf), "%.6f", gps.location.lng());
  } else {
    snprintf(lineBuf, sizeof(lineBuf), "---.------");
  }
  canvas.drawString(lineBuf, 88, startY + stepY);

  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  canvas.drawString("Alt:", 12, startY + stepY * 2);
  canvas.setTextColor(COLOR_TEXT, COLOR_BG);
  if (gps.altitude.isValid()) {
    snprintf(lineBuf, sizeof(lineBuf), "%.1f m", gps.altitude.meters());
  } else {
    snprintf(lineBuf, sizeof(lineBuf), "--- m");
  }
  canvas.drawString(lineBuf, 88, startY + stepY * 2);

  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  canvas.drawString("Speed:", 12, startY + stepY * 3);
  canvas.setTextColor(COLOR_TEXT, COLOR_BG);
  if (gps.speed.isValid()) {
    snprintf(lineBuf, sizeof(lineBuf), "%.1f km/h", gps.speed.kmph());
  } else {
    snprintf(lineBuf, sizeof(lineBuf), "0.0 km/h");
  }
  canvas.drawString(lineBuf, 88, startY + stepY * 3);

  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  canvas.drawString("Sat Time:", 12, startY + stepY * 4);
  canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
  if (gps.time.isValid()) {
    snprintf(lineBuf, sizeof(lineBuf), "%02u:%02u:%02u UTC",
             gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    snprintf(lineBuf, sizeof(lineBuf), "--:--:-- UTC");
  }
  canvas.drawString(lineBuf, 88, startY + stepY * 4);

  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  canvas.drawString("Sats:", 12, startY + stepY * 5);
  uint32_t sats = gps.satellites.isValid() ? (uint32_t)gps.satellites.value() : 0UL;
  canvas.setTextColor((sats > 0) ? COLOR_GREEN : COLOR_WARN, COLOR_BG);
  snprintf(lineBuf, sizeof(lineBuf), "%" PRIu32, sats);
  canvas.drawString(lineBuf, 88, startY + stepY * 5);

  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  canvas.drawString("Status:", 12, startY + stepY * 6);
  bool noUartData = (gps.charsProcessed() == 0) || (millis() - lastGpsByteMs > 5000UL);

  if (noUartData) {
    canvas.setTextColor(COLOR_RED, COLOR_BG);
    snprintf(lineBuf, sizeof(lineBuf), "Error");
  } else if (gnggaHdop.isValid() && gnggaHdop.value()[0] != '\0') {
    canvas.setTextColor(COLOR_GREEN, COLOR_BG);
    snprintf(lineBuf, sizeof(lineBuf), "HDOP: %s", gnggaHdop.value());
  } else if (gps.hdop.isValid()) {
    canvas.setTextColor(COLOR_GREEN, COLOR_BG);
    snprintf(lineBuf, sizeof(lineBuf), "HDOP: %.2f", gps.hdop.hdop());
  } else {
    canvas.setTextColor(COLOR_WARN, COLOR_BG);
    snprintf(lineBuf, sizeof(lineBuf), "HDOP: --");
  }
  canvas.drawString(lineBuf, 88, startY + stepY * 6);
}

// Button labels for calculator page 1
const char* CALC_PAGE1_LABELS[5][4] = {
  {"C", "+/-", "%", "/"},
  {"7", "8",   "9", "*"},
  {"4", "5",   "6", "-"},
  {"1", "2",   "3", "+"},
  {"0", "",    ".", "="}
};

// 16 button labels for calculator page 2 (includes |x| and rnd)
const char* CALC_PAGE2_LABELS[16] = {
  "n!",  "sqrt", "ln",   "lg",
  "e^x", "2^x",  "10^x", "1/x",
  "x^2", "x^3",  "x^y",  "x^1/y",
  "pi",  "e",    "|x|",  "rnd"
};

// 14 button labels for calculator page 3 (trigonometric and hyperbolic functions)
const char* CALC_PAGE3_LABELS[14] = {
  "sin",  "cos",  "tan",  "cot",
  "asin", "acos", "atan", "acot",
  "sinh", "cosh", "tanh", "coth",
  "sech", "csch"
};

// 6 button labels for calculator page 4 (inverse hyperbolic functions)
const char* CALC_PAGE4_LABELS[6] = {
  "asinh", "acosh",
  "atanh", "acoth",
  "asech", "acsch"
};

// Base selector and 5x4 keypad for calculator page 5 (number base converter with fractional support)
const uint8_t CALC_PAGE5_BASE_VALS[4] = {2, 8, 10, 16};
const char* CALC_PAGE5_BASE_LABELS[4] = {"BIN", "OCT", "DEC", "HEX"};
const char* CALC_PAGE5_KEYS[5][4] = {
  {"A", "B", "C",   "D"},
  {"7", "8", "9",   "E"},
  {"4", "5", "6",   "F"},
  {"1", "2", "3",   "DEL"},
  {"0", ".", "+/-", "CLR"}
};

bool isCalcBaseKeyAllowed(const char* key, uint8_t base) {
  if (key == NULL || key[0] == '\0') return false;
  if (strcmp(key, ".") == 0 || strcmp(key, "+/-") == 0 ||
      strcmp(key, "DEL") == 0 || strcmp(key, "CLR") == 0) {
    return true;
  }
  if (key[1] == '\0') {
    int8_t d = -1;
    if (key[0] >= '0' && key[0] <= '9') d = (int8_t)(key[0] - '0');
    else if (key[0] >= 'A' && key[0] <= 'F') d = (int8_t)(10 + (key[0] - 'A'));
    return (d >= 0 && d < (int8_t)base);
  }
  return false;
}

// Vector rendering of mathematical symbols for calculator page 2 buttons
void drawCalcMathSymbol(int8_t funcIndex, int16_t cx, int16_t cy) {
  canvas.setTextDatum(middle_center);
  canvas.setTextColor(COLOR_TEXT, COLOR_HEX_ACTIVE);

  if (funcIndex == 1) {
    // Square root symbol: sqrt(x)
    canvas.drawLine(cx - 12, cy + 1, cx - 9,  cy + 1, COLOR_TEXT);
    canvas.drawLine(cx - 9,  cy + 1, cx - 6,  cy + 7, COLOR_TEXT);
    canvas.drawLine(cx - 6,  cy + 7, cx - 2,  cy - 7, COLOR_TEXT);
    canvas.drawLine(cx - 5,  cy + 7, cx - 1,  cy - 7, COLOR_TEXT);
    canvas.drawLine(cx - 2,  cy - 7, cx + 11, cy - 7, COLOR_TEXT);
    canvas.setFont(&fonts::Font2);
    canvas.drawString("x", cx + 5, cy + 2);
  } else if (funcIndex == 4) {
    // Exponential symbol: e^x
    canvas.setFont(&fonts::Font2);
    canvas.drawString("e", cx - 4, cy + 2);
    canvas.setFont(&fonts::Font0);
    canvas.drawString("x", cx + 6, cy - 5);
  } else if (funcIndex == 5) {
    // Power of two symbol: 2^x
    canvas.setFont(&fonts::Font2);
    canvas.drawString("2", cx - 4, cy + 2);
    canvas.setFont(&fonts::Font0);
    canvas.drawString("x", cx + 6, cy - 5);
  } else if (funcIndex == 6) {
    // Power of ten symbol: 10^x
    canvas.setFont(&fonts::Font2);
    canvas.drawString("10", cx - 5, cy + 2);
    canvas.setFont(&fonts::Font0);
    canvas.drawString("x", cx + 10, cy - 5);
  } else if (funcIndex == 8) {
    // Square symbol: x^2
    canvas.setFont(&fonts::Font2);
    canvas.drawString("x", cx - 4, cy + 2);
    canvas.setFont(&fonts::Font0);
    canvas.drawString("2", cx + 6, cy - 5);
  } else if (funcIndex == 9) {
    // Cube symbol: x^3
    canvas.setFont(&fonts::Font2);
    canvas.drawString("x", cx - 4, cy + 2);
    canvas.setFont(&fonts::Font0);
    canvas.drawString("3", cx + 6, cy - 5);
  } else if (funcIndex == 10) {
    // Arbitrary power symbol: x^y
    canvas.setFont(&fonts::Font2);
    canvas.drawString("x", cx - 4, cy + 2);
    canvas.setFont(&fonts::Font0);
    canvas.drawString("y", cx + 6, cy - 5);
  } else if (funcIndex == 11) {
    // Arbitrary root symbol: y-th root of x
    canvas.setFont(&fonts::Font0);
    canvas.drawString("y", cx - 11, cy - 5);
    canvas.drawLine(cx - 11, cy + 2, cx - 8,  cy + 2, COLOR_TEXT);
    canvas.drawLine(cx - 8,  cy + 2, cx - 5,  cy + 8, COLOR_TEXT);
    canvas.drawLine(cx - 5,  cy + 8, cx - 1,  cy - 7, COLOR_TEXT);
    canvas.drawLine(cx - 4,  cy + 8, cx,      cy - 7, COLOR_TEXT);
    canvas.drawLine(cx - 1,  cy - 7, cx + 12, cy - 7, COLOR_TEXT);
    canvas.setFont(&fonts::Font2);
    canvas.drawString("x", cx + 6, cy + 2);
  } else {
    canvas.setFont(&fonts::Font2);
    canvas.drawString(CALC_PAGE2_LABELS[funcIndex], cx, cy);
  }
}

// Draw infinity symbol (inf or -inf) in calculator result line
void drawCalcInfinitySymbol(int8_t sign) {
  int16_t cy = 48;
  canvas.drawCircle(206, cy, 6, COLOR_TEXT);
  canvas.drawCircle(206, cy, 5, COLOR_TEXT);
  canvas.drawCircle(218, cy, 6, COLOR_TEXT);
  canvas.drawCircle(218, cy, 5, COLOR_TEXT);

  if (sign < 0) {
    canvas.fillRect(184, cy - 1, 11, 2, COLOR_TEXT);
  }
}

// Draw Calculator screen (SCREEN_CALC, 5 pages)
void drawCalcScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();
  drawTopRightFuncIcon(calcSecondPage);

  canvas.fillRoundRect(6, 31, 228, 34, 5, COLOR_TOPBAR);
  canvas.drawRoundRect(6, 31, 228, 34, 5, (calcPage == 4) ? 0xFD20 : COLOR_HEX_BORDER);

  if (calcPage == 4) {
    // Active number base tag on left of display box
    const char* baseTag = "DEC";
    if (calcBaseMode == 2) baseTag = "BIN";
    else if (calcBaseMode == 8) baseTag = "OCT";
    else if (calcBaseMode == 16) baseTag = "HEX";
    canvas.setTextDatum(middle_left);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(0xFD20, COLOR_TOPBAR);
    canvas.drawString(baseTag, 11, 48);

    // Converted value right-aligned with dynamic font size for long binary/fractional strings
    canvas.setTextDatum(middle_right);
    uint16_t strLen = (uint16_t)strlen(calcBaseStr);
    if (strLen > 18) {
      canvas.setFont(&fonts::Font0);
    } else if (strLen > 10) {
      canvas.setFont(&fonts::Font2);
    } else {
      canvas.setFont(&fonts::Font4);
    }
    canvas.setTextColor(calcBaseError ? COLOR_RED : COLOR_TEXT, COLOR_TOPBAR);
    canvas.drawString(calcBaseStr, 228, 49);
  } else {
    if (calcPendingOp != 0) {
      const char* opLabel = (calcPendingOp == 'r') ? "rt" : NULL;
      char opStr[3];
      if (opLabel != NULL) {
        snprintf(opStr, sizeof(opStr), "%s", opLabel);
      } else {
        opStr[0] = calcPendingOp;
        opStr[1] = '\0';
      }
      canvas.setTextDatum(middle_left);
      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(COLOR_WARN, COLOR_TOPBAR);
      canvas.drawString(opStr, 10, 48);
    }

    if (calcInfSign != 0) {
      drawCalcInfinitySymbol(calcInfSign);
    } else {
      canvas.setTextDatum(middle_right);
      uint16_t strLen = (uint16_t)strlen(calcDisplayStr);
      if (strLen > 24) {
        canvas.setFont(&fonts::Font0);
      } else if (strLen > 11) {
        canvas.setFont(&fonts::Font2);
      } else {
        canvas.setFont(&fonts::Font4);
      }
      canvas.setTextColor(calcError ? COLOR_RED : COLOR_TEXT, COLOR_TOPBAR);
      canvas.drawString(calcDisplayStr, 228, 49);
    }
  }

  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font2);

  if (calcPage == 0) {
    for (int8_t r = 0; r < 5; r++) {
      for (int8_t c = 0; c < 4; c++) {
        if (r == 4 && c == 1) continue;

        int16_t bx = 6 + c * 57;
        int16_t by = 69 + r * 34;
        int16_t bw = (r == 4 && c == 0) ? 111 : 54;
        int16_t bh = 31;

        uint16_t fillCol = COLOR_HEX_FILL;
        uint16_t bordCol = COLOR_HEX_BORDER;
        if (c == 3) {
          fillCol = COLOR_HEX_ACTIVE;
          bordCol = COLOR_ACCENT;
        } else if (r == 0) {
          bordCol = COLOR_WARN;
        }

        canvas.fillRoundRect(bx, by, bw, bh, 5, fillCol);
        canvas.drawRoundRect(bx, by, bw, bh, 5, bordCol);
        canvas.setTextColor(COLOR_TEXT, fillCol);
        canvas.drawString(CALC_PAGE1_LABELS[r][c], bx + bw / 2, by + bh / 2);
      }
    }
  } else if (calcPage == 1) {
    for (int8_t i = 0; i < 16; i++) {
      int8_t r = i / 4;
      int8_t c = i % 4;
      int16_t bx = 6 + c * 57;
      int16_t by = 69 + r * 42;
      int16_t bw = 54;
      int16_t bh = 38;

      uint16_t bordCol = (i == 10 || i == 11) ? COLOR_WARN : ((i == 14 || i == 15) ? COLOR_GREEN : COLOR_ACCENT);
      canvas.fillRoundRect(bx, by, bw, bh, 5, COLOR_HEX_ACTIVE);
      canvas.drawRoundRect(bx, by, bw, bh, 5, bordCol);
      drawCalcMathSymbol(i, bx + bw / 2, by + bh / 2);
    }
  } else if (calcPage == 2) {
    for (int8_t i = 0; i < 14; i++) {
      int16_t bx, by, bw, bh;
      if (i < 12) {
        int8_t r = i / 4;
        int8_t c = i % 4;
        bx = 6 + c * 57;
        by = 69 + r * 42;
        bw = 54;
        bh = 38;
      } else if (i == 12) {
        bx = 6;
        by = 69 + 3 * 42;
        bw = 111;
        bh = 38;
      } else {
        bx = 120;
        by = 69 + 3 * 42;
        bw = 114;
        bh = 38;
      }

      uint16_t bordCol = (i >= 8) ? COLOR_GREEN : COLOR_ACCENT;
      canvas.fillRoundRect(bx, by, bw, bh, 5, COLOR_HEX_ACTIVE);
      canvas.drawRoundRect(bx, by, bw, bh, 5, bordCol);
      canvas.setTextColor(COLOR_TEXT, COLOR_HEX_ACTIVE);
      canvas.drawString(CALC_PAGE3_LABELS[i], bx + bw / 2, by + bh / 2);
    }
  } else if (calcPage == 3) {
    // Calculator page 4: 6 inverse hyperbolic functions (3x2 grid)
    for (int8_t i = 0; i < 6; i++) {
      int8_t r = i / 2;
      int8_t c = i % 2;
      int16_t bx = (int16_t)(6 + c * 117);
      int16_t by = (int16_t)(72 + r * 55);
      int16_t bw = (c == 0) ? 111 : 114;
      int16_t bh = 50;

      canvas.fillRoundRect(bx, by, bw, bh, 6, COLOR_HEX_ACTIVE);
      canvas.drawRoundRect(bx, by, bw, bh, 6, 0xF81F);
      canvas.setTextColor(COLOR_TEXT, COLOR_HEX_ACTIVE);
      canvas.drawString(CALC_PAGE4_LABELS[i], bx + bw / 2, by + bh / 2);
    }
  } else {
    // Calculator page 5: Number base converter (BIN / OCT / DEC / HEX + 0-9, A-F, ., +/-, DEL, CLR)
    for (int8_t c = 0; c < 4; c++) {
      int16_t bx = (int16_t)(6 + c * 57);
      int16_t by = 69;
      int16_t bw = 54;
      int16_t bh = 24;
      bool activeBase = (calcBaseMode == CALC_PAGE5_BASE_VALS[c]);
      uint16_t fillCol = activeBase ? COLOR_HEX_ACTIVE : COLOR_TOPBAR;
      uint16_t bordCol = activeBase ? 0xFD20 : COLOR_HEX_BORDER;
      uint16_t textCol = activeBase ? 0xFD20 : COLOR_SUBTEXT;
      canvas.fillRoundRect(bx, by, bw, bh, 5, fillCol);
      canvas.drawRoundRect(bx, by, bw, bh, 5, bordCol);
      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(textCol, fillCol);
      canvas.drawString(CALC_PAGE5_BASE_LABELS[c], bx + bw / 2, by + bh / 2);
    }

    for (int8_t r = 0; r < 5; r++) {
      for (int8_t c = 0; c < 4; c++) {
        const char* key = CALC_PAGE5_KEYS[r][c];
        int16_t bx = (int16_t)(6 + c * 57);
        int16_t by = (int16_t)(96 + r * 28);
        int16_t bw = 54;
        int16_t bh = 25;
        bool allowed = isCalcBaseKeyAllowed(key, calcBaseMode);
        bool isHexLetter = (key[1] == '\0' && key[0] >= 'A' && key[0] <= 'F');
        bool isResetKey  = (strcmp(key, "CLR") == 0 || strcmp(key, "DEL") == 0);
        bool isDotOrSign = (strcmp(key, ".") == 0 || strcmp(key, "+/-") == 0);

        uint16_t fillCol = COLOR_HEX_FILL;
        uint16_t bordCol = COLOR_HEX_BORDER;
        uint16_t textCol = COLOR_TEXT;

        if (!allowed) {
          fillCol = COLOR_TOPBAR;
          bordCol = 0x2104;
          textCol = 0x4A49;
        } else if (isResetKey) {
          fillCol = COLOR_HEX_ACTIVE;
          bordCol = (strcmp(key, "CLR") == 0) ? COLOR_RED : COLOR_WARN;
          textCol = (strcmp(key, "CLR") == 0) ? COLOR_RED : COLOR_WARN;
        } else if (isDotOrSign) {
          fillCol = COLOR_HEX_ACTIVE;
          bordCol = COLOR_ACCENT;
          textCol = COLOR_ACCENT;
        } else if (isHexLetter) {
          fillCol = COLOR_HEX_ACTIVE;
          bordCol = 0xFD20;
          textCol = COLOR_TEXT;
        }

        canvas.fillRoundRect(bx, by, bw, bh, 5, fillCol);
        canvas.drawRoundRect(bx, by, bw, bh, 5, bordCol);
        if (strlen(key) >= 3) {
          canvas.setFont(&fonts::Font0);
        } else {
          canvas.setFont(&fonts::Font2);
        }
        canvas.setTextColor(textCol, fillCol);
        canvas.drawString(key, bx + bw / 2, by + bh / 2);
      }
    }
  }
}

// Button labels for IR screen page 1 (0-9, A-F, CLR, Test LED, Send)
const char* IR_PAD_LABELS[5][4] = {
  {"A",   "B",        "C",    "D"},
  {"7",   "8",        "9",    "E"},
  {"4",   "5",        "6",    "F"},
  {"1",   "2",        "3",    "CLR"},
  {"0",   "Test LED", "Send", ""}
};

// Draw IR transmitter screen (SCREEN_IR: page 1 - code entry, page 2 - settings)
void drawIrScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();
  drawTopRightGearIcon(irSettingsPage);

  if (!irSettingsPage) {
    canvas.fillRoundRect(6, 31, 228, 34, 5, COLOR_TOPBAR);
    canvas.drawRoundRect(6, 31, 228, 34, 5, COLOR_HEX_BORDER);

    canvas.setTextDatum(middle_left);
    canvas.setFont(&fonts::Font2);
    if (irTestNoticeMs != 0 && (millis() - irTestNoticeMs < 1800UL)) {
      canvas.setTextColor(COLOR_WARN, COLOR_TOPBAR);
      canvas.drawString("TEST", 12, 48);
    } else if (irSentNoticeMs != 0 && (millis() - irSentNoticeMs < 1200UL)) {
      canvas.setTextColor(COLOR_GREEN, COLOR_TOPBAR);
      canvas.drawString("SENT", 12, 48);
    } else {
      canvas.setTextColor(COLOR_ACCENT, COLOR_TOPBAR);
      canvas.drawString("0x", 12, 48);
    }

    canvas.setTextDatum(middle_right);
    canvas.setFont(&fonts::Font4);
    canvas.setTextColor(COLOR_TEXT, COLOR_TOPBAR);
    canvas.drawString(irHexStr, 228, 49);

    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);

    for (int8_t r = 0; r < 5; r++) {
      for (int8_t c = 0; c < 4; c++) {
        if (r == 4 && c == 3) continue;

        int16_t bx = 6 + c * 57;
        int16_t by = 69 + r * 34;
        int16_t bw = 54;
        int16_t bh = 31;

        if (r == 4) {
          if (c == 0) {
            bx = 6;
            bw = 54;
          } else if (c == 1) {
            bx = 63;
            bw = 82;
          } else {
            bx = 148;
            bw = 83;
          }
        }

        const char* lbl = IR_PAD_LABELS[r][c];
        uint16_t fillCol = COLOR_HEX_FILL;
        uint16_t bordCol = COLOR_HEX_BORDER;

        if (strcmp(lbl, "Send") == 0) {
          fillCol = COLOR_HEX_ACTIVE;
          bordCol = COLOR_GREEN;
        } else if (strcmp(lbl, "Test LED") == 0) {
          fillCol = COLOR_HEX_ACTIVE;
          bordCol = COLOR_WARN;
        } else if (strcmp(lbl, "CLR") == 0) {
          bordCol = COLOR_RED;
        } else if (lbl[0] >= 'A' && lbl[0] <= 'F') {
          fillCol = COLOR_HEX_ACTIVE;
          bordCol = COLOR_ACCENT;
        }

        canvas.fillRoundRect(bx, by, bw, bh, 5, fillCol);
        canvas.drawRoundRect(bx, by, bw, bh, 5, bordCol);
        canvas.setTextColor(strcmp(lbl, "Test LED") == 0 ? COLOR_WARN : COLOR_TEXT, fillCol);
        canvas.drawString(lbl, bx + bw / 2, by + bh / 2);
      }
    }
  } else {
    canvas.setTextDatum(top_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
    canvas.drawString("IR SETTINGS", 120, 36);

    canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
    canvas.drawString("Protocol:", 120, 62);

    const char* protoLabels[3] = {"NEC", "RC-5", "RC-6"};
    canvas.setTextDatum(middle_center);
    for (int8_t i = 0; i < 3; i++) {
      int16_t bx = 10 + i * 76;
      int16_t by = 82;
      int16_t bw = 68;
      int16_t bh = 40;
      bool sel = ((uint8_t)irProtocol == (uint8_t)i);

      uint16_t fillCol = sel ? COLOR_HEX_ACTIVE : COLOR_HEX_FILL;
      uint16_t bordCol = sel ? COLOR_WARN       : COLOR_HEX_BORDER;
      canvas.fillRoundRect(bx, by, bw, bh, 6, fillCol);
      canvas.drawRoundRect(bx, by, bw, bh, 6, bordCol);
      canvas.setTextColor(sel ? COLOR_WARN : COLOR_TEXT, fillCol);
      canvas.drawString(protoLabels[i], bx + bw / 2, by + bh / 2);
    }

    canvas.setTextDatum(top_center);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
    canvas.drawString("Frequency:", 120, 134);

    const uint8_t freqVals[3] = {33, 36, 38};
    const char* freqLabels[3] = {"33 kHz", "36 kHz", "38 kHz"};
    canvas.setTextDatum(middle_center);
    for (int8_t i = 0; i < 3; i++) {
      int16_t bx = 10 + i * 76;
      int16_t by = 154;
      int16_t bw = 68;
      int16_t bh = 40;
      bool sel = (irCarrierKhz == freqVals[i]);

      uint16_t fillCol = sel ? COLOR_HEX_ACTIVE : COLOR_HEX_FILL;
      uint16_t bordCol = sel ? COLOR_GREEN      : COLOR_HEX_BORDER;
      canvas.fillRoundRect(bx, by, bw, bh, 6, fillCol);
      canvas.drawRoundRect(bx, by, bw, bh, 6, bordCol);
      canvas.setTextColor(sel ? COLOR_GREEN : COLOR_TEXT, fillCol);
      canvas.drawString(freqLabels[i], bx + bw / 2, by + bh / 2);
    }

    char infoBuf[32];
    snprintf(infoBuf, sizeof(infoBuf), "Active: %s  %u kHz",
             protoLabels[(uint8_t)irProtocol], (uint16_t)irCarrierKhz);
    canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
    canvas.drawString(infoBuf, 120, 216);
  }
}

// Helper to scale RGB565 color brightness for 3D Earth lighting
uint16_t shadeColor565(uint8_t r, uint8_t g, uint8_t b, float intensity) {
  uint8_t sr = (uint8_t)constrain((int16_t)(r * intensity), (int16_t)0, (int16_t)255);
  uint8_t sg = (uint8_t)constrain((int16_t)(g * intensity), (int16_t)0, (int16_t)255);
  uint8_t sb = (uint8_t)constrain((int16_t)(b * intensity), (int16_t)0, (int16_t)255);
  return ((uint16_t)(sr & 0xF8) << 8) | ((uint16_t)(sg & 0xFC) << 3) | (sb >> 3);
}

// Read single land bit from detailed EARTH_CONTINENTS bitmap (72x64)
float sampleEarthBit(int16_t row, int16_t col) {
  row = constrain(row, (int16_t)0, (int16_t)71);
  col = (col + 64) & 63;
  return (((EARTH_CONTINENTS[row] >> (63 - col)) & 1ULL) != 0) ? 1.0f : 0.0f;
}

// Convert local RTC date/time and timezone offset to UTC Julian Date
double computeJulianDateUTC(const DateTimeRTC &dt, int8_t tzOffsetHours) {
  int32_t y = (int32_t)dt.year;
  int32_t m = (int32_t)dt.month;
  int32_t d = (int32_t)dt.day;
  if (m <= 2) {
    y -= 1;
    m += 12;
  }
  int32_t a = y / 100;
  int32_t b = 2 - a + (a / 4);
  double jdDay = floor(365.25 * (double)(y + 4716)) +
                 floor(30.6001 * (double)(m + 1)) +
                 (double)d + (double)b - 1524.5;
  double fracDay = ((double)dt.hour + (double)dt.minute / 60.0 +
                    (double)dt.second / 3600.0 - (double)tzOffsetHours) / 24.0;
  return jdDay + fracDay;
}

// Compute Greenwich Mean Sidereal Time (GMST, radians)
double computeGmstRad(double jdUtc) {
  double d = jdUtc - 2451545.0;
  double t = d / 36525.0;
  double gmstDeg = 280.46061837 + 360.98564736629 * d +
                   0.000387933 * t * t - (t * t * t) / 38710000.0;
  gmstDeg = fmod(gmstDeg, 360.0);
  if (gmstDeg < 0.0) gmstDeg += 360.0;
  return gmstDeg * (M_PI / 180.0);
}

// Compute sub-satellite latitude and longitude on Kinosputnik orbit (with J2 secular precession)
void computeSatelliteLatLon(double jdUtc, double uOverrideRad, bool useOverrideU, float &outLatRad, float &outLonRad) {
  const double mu = 398600.4418;
  const double rEarth = 6378.137;
  const double j2 = 1.08262668e-3;

  double inc = TLE_INC_DEG * (M_PI / 180.0);
  double raan0 = TLE_RAAN_DEG * (M_PI / 180.0);
  double argp0 = TLE_ARG_P_DEG * (M_PI / 180.0);
  double m0 = TLE_M0_DEG * (M_PI / 180.0);
  double n = TLE_N0_REV_DAY * (2.0 * M_PI / 86400.0);

  double a = cbrt(mu / (n * n));
  double p = a * (1.0 - TLE_ECC * TLE_ECC);
  double reOverP = rEarth / p;
  double j2Factor = 1.5 * j2 * reOverP * reOverP * n;
  double cosI = cos(inc);
  double sinI = sin(inc);

  double raanDot = -j2Factor * cosI;
  double argpDot = 0.5 * j2Factor * (5.0 * cosI * cosI - 1.0);
  double mDotJ2  = 0.5 * j2Factor * sqrt(1.0 - TLE_ECC * TLE_ECC) * (3.0 * cosI * cosI - 1.0);

  double dtSec = (jdUtc - TLE_EPOCH_JD) * 86400.0;
  double raan = raan0 + raanDot * dtSec;
  double u;

  if (useOverrideU) {
    u = uOverrideRad;
  } else {
    double argp = argp0 + argpDot * dtSec;
    double meanAnom = fmod(m0 + (n + mDotJ2) * dtSec, 2.0 * M_PI);
    if (meanAnom < 0.0) meanAnom += 2.0 * M_PI;

    double eccAnom = meanAnom;
    for (uint8_t iter = 0; iter < 5; iter++) {
      eccAnom = eccAnom - (eccAnom - TLE_ECC * sin(eccAnom) - meanAnom) /
                          (1.0 - TLE_ECC * cos(eccAnom));
    }
    double trueAnom = 2.0 * atan2(sqrt(1.0 + TLE_ECC) * sin(eccAnom * 0.5),
                                  sqrt(1.0 - TLE_ECC) * cos(eccAnom * 0.5));
    u = argp + trueAnom;
  }

  double cosO = cos(raan);
  double sinO = sin(raan);
  double cosU = cos(u);
  double sinU = sin(u);

  double xEci = cosO * cosU - sinO * sinU * cosI;
  double yEci = sinO * cosU + cosO * sinU * cosI;
  double zEci = sinU * sinI;

  double gmst = computeGmstRad(jdUtc);
  double cosG = cos(gmst);
  double sinG = sin(gmst);

  double xEcef =  xEci * cosG + yEci * sinG;
  double yEcef = -xEci * sinG + yEci * cosG;
  double zEcef =  zEci;

  outLatRad = (float)atan2(zEcef, sqrt(xEcef * xEcef + yEcef * yEcef));
  outLonRad = (float)atan2(yEcef, xEcef);
}

// Compute sub-satellite latitude and longitude on ISS orbit (NORAD 25544, J2 + nDot/2 secular drift)
void computeIssLatLon(double jdUtc, double uOverrideRad, bool useOverrideU, float &outLatRad, float &outLonRad) {
  const double mu = 398600.4418;
  const double rEarth = 6378.137;
  const double j2 = 1.08262668e-3;

  double inc = ISS_TLE_INC_DEG * (M_PI / 180.0);
  double raan0 = ISS_TLE_RAAN_DEG * (M_PI / 180.0);
  double argp0 = ISS_TLE_ARG_P_DEG * (M_PI / 180.0);
  double m0 = ISS_TLE_M0_DEG * (M_PI / 180.0);
  double n = ISS_TLE_N0_REV_DAY * (2.0 * M_PI / 86400.0);

  double a = cbrt(mu / (n * n));
  double p = a * (1.0 - ISS_TLE_ECC * ISS_TLE_ECC);
  double reOverP = rEarth / p;
  double j2Factor = 1.5 * j2 * reOverP * reOverP * n;
  double cosI = cos(inc);
  double sinI = sin(inc);

  double raanDot = -j2Factor * cosI;
  double argpDot = 0.5 * j2Factor * (5.0 * cosI * cosI - 1.0);
  double mDotJ2  = 0.5 * j2Factor * sqrt(1.0 - ISS_TLE_ECC * ISS_TLE_ECC) * (3.0 * cosI * cosI - 1.0);

  double dtDays = jdUtc - ISS_TLE_EPOCH_JD;
  double dtSec = dtDays * 86400.0;
  double raan = raan0 + raanDot * dtSec;
  double u;

  if (useOverrideU) {
    u = uOverrideRad;
  } else {
    double argp = argp0 + argpDot * dtSec;
    double dragTerm = ISS_TLE_NDOT2 * dtDays * dtDays * (2.0 * M_PI);
    double meanAnom = fmod(m0 + (n + mDotJ2) * dtSec + dragTerm, 2.0 * M_PI);
    if (meanAnom < 0.0) meanAnom += 2.0 * M_PI;

    double eccAnom = meanAnom;
    for (uint8_t iter = 0; iter < 5; iter++) {
      eccAnom = eccAnom - (eccAnom - ISS_TLE_ECC * sin(eccAnom) - meanAnom) /
                          (1.0 - ISS_TLE_ECC * cos(eccAnom));
    }
    double trueAnom = 2.0 * atan2(sqrt(1.0 + ISS_TLE_ECC) * sin(eccAnom * 0.5),
                                  sqrt(1.0 - ISS_TLE_ECC) * cos(eccAnom * 0.5));
    u = argp + trueAnom;
  }

  double cosO = cos(raan);
  double sinO = sin(raan);
  double cosU = cos(u);
  double sinU = sin(u);

  double xEci = cosO * cosU - sinO * sinU * cosI;
  double yEci = sinO * cosU + cosO * sinU * cosI;
  double zEci = sinU * sinI;

  double gmst = computeGmstRad(jdUtc);
  double cosG = cos(gmst);
  double sinG = sin(gmst);

  double xEcef =  xEci * cosG + yEci * sinG;
  double yEcef = -xEci * sinG + yEci * cosG;
  double zEcef =  zEci;

  outLatRad = (float)atan2(zEcef, sqrt(xEcef * xEcef + yEcef * yEcef));
  outLonRad = (float)atan2(yEcef, xEcef);
}

// Project geographic coordinates (latRad, lonRad) into rotated Earth screen space
void projectEarthPoint(float latRad, float lonRad, float cosLat, float sinLat,
                       float cosLon, float sinLon, float &nx, float &ny, float &nz) {
  float x2 = cosf(latRad) * sinf(lonRad);
  float y2 = -sinf(latRad);
  float z2 = cosf(latRad) * cosf(lonRad);

  float x1 = x2 * cosLon - z2 * sinLon;
  float y1 = y2;
  float z1 = x2 * sinLon + z2 * cosLon;

  nx = x1;
  ny = y1 * cosLat + z1 * sinLat;
  nz = -y1 * sinLat + z1 * cosLat;
}

// Miniature Kinosputnik satellite icon
void drawKinosputnikIcon(int16_t sx, int16_t sy) {
  canvas.fillTriangle(sx - 8, sy + 5, sx + 4, sy - 7, sx + 8, sy - 3, 0x2124);
  canvas.fillTriangle(sx - 8, sy + 5, sx + 8, sy - 3, sx - 4, sy + 9, 0x2124);
  canvas.drawLine(sx - 7, sy + 5, sx + 5, sy - 6, 0x632C);
  canvas.drawLine(sx - 4, sy + 7, sx + 7, sy - 3, 0x632C);

  canvas.drawLine(sx + 4, sy - 4, sx + 9, sy - 9, 0xCE64);
  canvas.drawLine(sx + 7, sy - 7, sx + 10, sy - 10, 0xDEFB);
  canvas.drawLine(sx - 5, sy + 5, sx - 9, sy + 9, 0xCE64);
  canvas.drawLine(sx - 7, sy + 7, sx - 10, sy + 10, 0xDEFB);

  canvas.fillTriangle(sx - 5, sy - 2, sx - 1, sy - 6, sx + 3, sy - 2, 0xFFFF);
  canvas.fillTriangle(sx - 5, sy - 2, sx + 3, sy - 2, sx - 1, sy + 2, 0xFFFF);
  canvas.drawCircle(sx - 1, sy - 2, 2, 0x2104);

  canvas.fillTriangle(sx - 1, sy + 2, sx + 3, sy - 2, sx + 6, sy + 1, 0xBD66);
  canvas.fillTriangle(sx - 1, sy + 2, sx + 6, sy + 1, sx + 2, sy + 5, 0xBD66);

  canvas.fillCircle(sx - 5, sy + 3, 3, 0xEF7D);
  canvas.fillCircle(sx - 6, sy + 4, 2, 0x1082);
}

// Miniature International Space Station (ISS) icon
void drawIssIcon(int16_t sx, int16_t sy) {
  // Dark shadow outline under truss for legibility over bright continents
  canvas.drawLine(sx - 10, sy + 1, sx + 10, sy + 1, 0x0000);
  // 4 pairs of copper-gold ISS solar arrays (left and right of main truss)
  canvas.fillRect(sx - 10, sy - 7, 3, 6, 0xD462);
  canvas.fillRect(sx - 10, sy + 2, 3, 6, 0xD462);
  canvas.fillRect(sx - 6,  sy - 7, 3, 6, 0xFE60);
  canvas.fillRect(sx - 6,  sy + 2, 3, 6, 0xFE60);
  canvas.fillRect(sx + 4,  sy - 7, 3, 6, 0xFE60);
  canvas.fillRect(sx + 4,  sy + 2, 3, 6, 0xFE60);
  canvas.fillRect(sx + 8,  sy - 7, 3, 6, 0xD462);
  canvas.fillRect(sx + 8,  sy + 2, 3, 6, 0xD462);
  // Main integrated truss
  canvas.drawLine(sx - 10, sy, sx + 10, sy, 0xDEFB);
  // Pressurized central ISS core modules
  canvas.fillRect(sx - 2, sy - 4, 5, 9, 0xFFFF);
  canvas.drawRect(sx - 2, sy - 4, 5, 9, 0x2104);
  canvas.drawFastHLine(sx - 1, sy, 3, COLOR_WARN);
}

// Draw 3D Earth screen (SCREEN_EARTH): night shadow, cities, GPS pin, Kinosputnik, and ISS
void drawEarthScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();
  drawTopRightSatIcon(earthShowSatellite);

  int16_t cx = 120;
  int16_t cy = 134;
  int16_t radius = 82;
  int16_t haloR = radius + 10;
  int32_t rSq = (int32_t)radius * radius;
  int32_t haloRSq = (int32_t)haloR * haloR;
  float invR = 1.0f / (float)radius;

  float bgR = 8.0f;
  float bgG = 8.0f;
  float bgB = 16.0f;

  float sunX = -0.64f;
  float sunY = -0.34f;
  float sunZ =  0.69f;

  float cosLat = cosf(earthLat);
  float sinLat = sinf(earthLat);
  float cosLon = cosf(earthLon);
  float sinLon = sinf(earthLon);

  int16_t step = earthDragging ? 2 : 1;

  for (int16_t dy = -haloR; dy <= haloR; dy += step) {
    int32_t dySq = (int32_t)dy * dy;
    for (int16_t dx = -haloR; dx <= haloR; dx += step) {
      int32_t distSq = (int32_t)dx * dx + dySq;
      if (distSq > haloRSq) continue;

      if (distSq <= rSq) {
        float nx = (float)dx * invR;
        float ny = (float)dy * invR;
        float nz = sqrtf(fmaxf(0.0f, 1.0f - nx * nx - ny * ny));

        float y1 = ny * cosLat - nz * sinLat;
        float z1 = ny * sinLat + nz * cosLat;
        float x1 = nx;

        float x2 = x1 * cosLon + z1 * sinLon;
        float y2 = y1;
        float z2 = -x1 * sinLon + z1 * cosLon;

        float lat = asinf(constrain(-y2, -1.0f, 1.0f));
        float lon = atan2f(x2, z2);

        float fRow = ((float)M_PI_2 - lat) * (71.0f / (float)M_PI);
        float fCol = (lon + (float)M_PI) * (64.0f / (2.0f * (float)M_PI));
        int16_t r0 = (int16_t)floorf(fRow);
        int16_t c0 = (int16_t)floorf(fCol);
        float ty = fRow - (float)r0;
        float tx = fCol - (float)c0;

        float l00 = sampleEarthBit(r0,     c0);
        float l10 = sampleEarthBit(r0,     c0 + 1);
        float l01 = sampleEarthBit(r0 + 1, c0);
        float l11 = sampleEarthBit(r0 + 1, c0 + 1);
        float landVal = (l00 * (1.0f - tx) + l10 * tx) * (1.0f - ty) +
                        (l01 * (1.0f - tx) + l11 * tx) * ty;

        float baseR, baseG, baseB;
        if (landVal >= 0.45f) {
          if (fabsf(lat) > 1.18f) {
            baseR = 230.0f; baseG = 240.0f; baseB = 252.0f;
          } else if ((lat > 0.20f && lat < 0.56f && lon > -0.28f && lon < 1.05f) ||
                     (lat < -0.28f && lat > -0.54f && lon > 2.00f && lon < 2.62f)) {
            baseR = 194.0f; baseG = 168.0f; baseB = 112.0f;
          } else if (landVal > 0.85f) {
            baseR = 48.0f;  baseG = 148.0f; baseB = 68.0f;
          } else {
            baseR = 72.0f;  baseG = 182.0f; baseB = 86.0f;
          }
        } else {
          if (landVal > 0.14f) {
            baseR = 26.0f;  baseG = 128.0f; baseB = 205.0f;
          } else {
            baseR = 14.0f;  baseG = 72.0f;  baseB = 185.0f;
          }
        }

        float ndotl = x2 * sunX + y2 * sunY + z2 * sunZ;
        float light;
        if (ndotl > 0.0f) {
          light = 0.09f + 0.91f * ndotl;
        } else {
          light = 0.09f * expf(ndotl * 2.6f);
        }

        float outR = baseR * light;
        float outG = baseG * light;
        float outB = baseB * light;

        float twilight = fmaxf(0.0f, 1.0f - fabsf(ndotl) / 0.16f);
        if (twilight > 0.0f) {
          outR += 28.0f * twilight;
          outG += 12.0f * twilight;
          outB += 4.0f  * twilight;
        }

        float rim = 1.0f - nz;
        float atmHaze = rim * rim * rim * (0.14f + 0.86f * fmaxf(0.0f, ndotl + 0.25f));
        atmHaze = constrain(atmHaze, 0.0f, 0.80f);
        outR = outR * (1.0f - atmHaze) + 90.0f  * atmHaze;
        outG = outG * (1.0f - atmHaze) + 185.0f * atmHaze;
        outB = outB * (1.0f - atmHaze) + 255.0f * atmHaze;

        uint16_t pixelColor = shadeColor565(
          (uint8_t)constrain((int16_t)outR, (int16_t)0, (int16_t)255),
          (uint8_t)constrain((int16_t)outG, (int16_t)0, (int16_t)255),
          (uint8_t)constrain((int16_t)outB, (int16_t)0, (int16_t)255),
          1.0f
        );

        if (step == 1) {
          canvas.drawPixel(cx + dx, cy + dy, pixelColor);
        } else {
          canvas.fillRect(cx + dx, cy + dy, 2, 2, pixelColor);
        }
      } else {
        float dist = sqrtf((float)distSq);
        float t = 1.0f - (dist - (float)radius) / (float)(haloR - radius);
        if (t > 0.0f) {
          float ux = (float)dx / dist;
          float uy = (float)dy / dist;
          float hy1 = uy * cosLat;
          float hz1 = uy * sinLat;
          float hx2 = ux * cosLon + hz1 * sinLon;
          float hy2 = hy1;
          float hz2 = -ux * sinLon + hz1 * cosLon;
          float haloNdotL = hx2 * sunX + hy2 * sunY + hz2 * sunZ;

          float sunSide = 0.20f + 0.80f * fmaxf(0.0f, haloNdotL + 0.35f);
          float glow = t * t * sunSide * 0.82f;

          uint8_t hr = (uint8_t)constrain((int16_t)(bgR + 75.0f  * glow), (int16_t)0, (int16_t)255);
          uint8_t hg = (uint8_t)constrain((int16_t)(bgG + 175.0f * glow), (int16_t)0, (int16_t)255);
          uint8_t hb = (uint8_t)constrain((int16_t)(bgB + 255.0f * glow), (int16_t)0, (int16_t)255);
          uint16_t haloCol = shadeColor565(hr, hg, hb, 1.0f);

          if (step == 1) {
            canvas.drawPixel(cx + dx, cy + dy, haloCol);
          } else {
            canvas.fillRect(cx + dx, cy + dy, 2, 2, haloCol);
          }
        }
      }
    }
  }

  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font0);
  for (int8_t i = 0; i < 7; i++) {
    float nx, ny, nz;
    projectEarthPoint(EARTH_LABELS[i].latDeg * DEG_TO_RAD,
                      EARTH_LABELS[i].lonDeg * DEG_TO_RAD,
                      cosLat, sinLat, cosLon, sinLon, nx, ny, nz);

    if (nz > 0.25f) {
      int16_t sx = cx + (int16_t)(nx * (float)radius);
      int16_t sy = cy + (int16_t)(ny * (float)radius);

      canvas.setTextColor(0x0000);
      canvas.drawString(EARTH_LABELS[i].name, sx + 1, sy + 1);
      canvas.setTextColor(COLOR_SUBTEXT);
      canvas.drawString(EARTH_LABELS[i].name, sx, sy);
    }
  }

  for (int8_t i = 0; i < 7; i++) {
    float nx, ny, nz;
    projectEarthPoint(EARTH_CITIES[i].latDeg * DEG_TO_RAD,
                      EARTH_CITIES[i].lonDeg * DEG_TO_RAD,
                      cosLat, sinLat, cosLon, sinLon, nx, ny, nz);

    if (nz > 0.15f) {
      int16_t sx = cx + (int16_t)(nx * (float)radius);
      int16_t sy = cy + (int16_t)(ny * (float)radius);

      canvas.fillCircle(sx, sy, 3, 0x0000);
      canvas.fillCircle(sx, sy, 2, COLOR_RED);

      int16_t lx = constrain((int16_t)(sx + EARTH_CITIES[i].labelOffX), (int16_t)24, (int16_t)216);
      int16_t ly = constrain((int16_t)(sy + EARTH_CITIES[i].labelOffY), (int16_t)34, (int16_t)232);

      canvas.setTextColor(0x0000);
      canvas.drawString(EARTH_CITIES[i].name, lx + 1, ly + 1);
      canvas.setTextColor(COLOR_TEXT);
      canvas.drawString(EARTH_CITIES[i].name, lx, ly);
    }
  }

  if (gps.location.isValid()) {
    float gnx, gny, gnz;
    projectEarthPoint((float)gps.location.lat() * DEG_TO_RAD,
                      (float)gps.location.lng() * DEG_TO_RAD,
                      cosLat, sinLat, cosLon, sinLon, gnx, gny, gnz);
    if (gnz > 0.05f) {
      int16_t gx = cx + (int16_t)(gnx * (float)radius);
      int16_t gy = cy + (int16_t)(gny * (float)radius);
      canvas.fillCircle(gx, gy, 4, 0x0000);
      canvas.fillCircle(gx, gy, 3, 0xFFE0);
    }
  }

  if (earthShowSatellite) {
    double jdUtc = computeJulianDateUTC(rtcTime, utcOffset);
    float orbitScale = 1.15f;
    float rOrb = (float)radius * orbitScale;

    for (int16_t seg = 0; seg < 72; seg += 2) {
      double u1 = (double)seg * (2.0 * M_PI / 72.0);
      double u2 = (double)(seg + 1) * (2.0 * M_PI / 72.0);
      float lat1, lon1, lat2, lon2;
      computeSatelliteLatLon(jdUtc, u1, true, lat1, lon1);
      computeSatelliteLatLon(jdUtc, u2, true, lat2, lon2);

      float nx1, ny1, nz1, nx2, ny2, nz2;
      projectEarthPoint(lat1, lon1, cosLat, sinLat, cosLon, sinLon, nx1, ny1, nz1);
      projectEarthPoint(lat2, lon2, cosLat, sinLat, cosLon, sinLon, nx2, ny2, nz2);

      float mx = 0.5f * (nx1 + nx2);
      float my = 0.5f * (ny1 + ny2);
      float mz = 0.5f * (nz1 + nz2);
      float projSq = (mx * mx + my * my) * (orbitScale * orbitScale);

      if (mz >= 0.0f || projSq > 1.02f) {
        int16_t px1 = cx + (int16_t)(nx1 * rOrb);
        int16_t py1 = cy + (int16_t)(ny1 * rOrb);
        int16_t px2 = cx + (int16_t)(nx2 * rOrb);
        int16_t py2 = cy + (int16_t)(ny2 * rOrb);
        uint16_t dashCol = (mz >= 0.0f) ? COLOR_ACCENT : COLOR_SUBTEXT;
        canvas.drawLine(px1, py1, px2, py2, dashCol);
      }
    }

    float satLatRad, satLonRad;
    computeSatelliteLatLon(jdUtc, 0.0, false, satLatRad, satLonRad);

    float snx, sny, snz;
    projectEarthPoint(satLatRad, satLonRad, cosLat, sinLat, cosLon, sinLon, snx, sny, snz);
    float satProjSq = (snx * snx + sny * sny) * (orbitScale * orbitScale);

    if (snz >= 0.0f || satProjSq > 0.98f) {
      int16_t satX = cx + (int16_t)(snx * rOrb);
      int16_t satY = cy + (int16_t)(sny * rOrb);
      drawKinosputnikIcon(satX, satY);

      int16_t lblX = (satX > 145) ? (satX - 12) : (satX + 12);
      int16_t lblY = constrain(satY, (int16_t)36, (int16_t)230);
      canvas.setTextDatum((satX > 145) ? middle_right : middle_left);
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(0x0000);
      canvas.drawString("Kinosputnik", lblX + 1, lblY + 1);
      canvas.setTextColor(0xFFE0);
      canvas.drawString("Kinosputnik", lblX, lblY);
    }

    // ISS orbit track and current position (inclination 51.6320 deg, altitude ~415 km)
    float issOrbitScale = 1.10f;
    float rOrbIss = (float)radius * issOrbitScale;

    for (int16_t seg = 0; seg < 72; seg += 2) {
      double u1 = (double)seg * (2.0 * M_PI / 72.0);
      double u2 = (double)(seg + 1) * (2.0 * M_PI / 72.0);
      float lat1, lon1, lat2, lon2;
      computeIssLatLon(jdUtc, u1, true, lat1, lon1);
      computeIssLatLon(jdUtc, u2, true, lat2, lon2);

      float nx1, ny1, nz1, nx2, ny2, nz2;
      projectEarthPoint(lat1, lon1, cosLat, sinLat, cosLon, sinLon, nx1, ny1, nz1);
      projectEarthPoint(lat2, lon2, cosLat, sinLat, cosLon, sinLon, nx2, ny2, nz2);

      float mx = 0.5f * (nx1 + nx2);
      float my = 0.5f * (ny1 + ny2);
      float mz = 0.5f * (nz1 + nz2);
      float projSq = (mx * mx + my * my) * (issOrbitScale * issOrbitScale);

      if (mz >= 0.0f || projSq > 1.02f) {
        int16_t px1 = cx + (int16_t)(nx1 * rOrbIss);
        int16_t py1 = cy + (int16_t)(ny1 * rOrbIss);
        int16_t px2 = cx + (int16_t)(nx2 * rOrbIss);
        int16_t py2 = cy + (int16_t)(ny2 * rOrbIss);
        uint16_t dashCol = (mz >= 0.0f) ? COLOR_WARN : 0x7A20;
        canvas.drawLine(px1, py1, px2, py2, dashCol);
      }
    }

    float issLatRad, issLonRad;
    computeIssLatLon(jdUtc, 0.0, false, issLatRad, issLonRad);

    float inx, iny, inz;
    projectEarthPoint(issLatRad, issLonRad, cosLat, sinLat, cosLon, sinLon, inx, iny, inz);
    float issProjSq = (inx * inx + iny * iny) * (issOrbitScale * issOrbitScale);

    if (inz >= 0.0f || issProjSq > 0.98f) {
      int16_t issX = cx + (int16_t)(inx * rOrbIss);
      int16_t issY = cy + (int16_t)(iny * rOrbIss);
      drawIssIcon(issX, issY);

      int16_t issLblX = (issX > 188) ? (issX - 13) : (issX + 13);
      int16_t issLblY = constrain(issY, (int16_t)36, (int16_t)230);
      canvas.setTextDatum((issX > 188) ? middle_right : middle_left);
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(0x0000);
      canvas.drawString("ISS", issLblX + 1, issLblY + 1);
      canvas.setTextColor(COLOR_WARN);
      canvas.drawString("ISS", issLblX, issLblY);
    }
  }
}

// Draw Step counter screen (SCREEN_STEPS)
void drawStepsScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();
  drawTopRightZeroIcon();

  canvas.setTextDatum(top_center);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
  canvas.drawString("STEP COUNTER", 120, 42);

  canvas.drawCircle(120, 134, 62, COLOR_HEX_BORDER);
  canvas.drawCircle(120, 134, 58, COLOR_ACCENT);

  char stepBuf[16];
  snprintf(stepBuf, sizeof(stepBuf), "%" PRIu32, stepCount);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font6);
  canvas.setTextColor(COLOR_TEXT, COLOR_BG);
  canvas.drawString(stepBuf, 120, 128);

  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_GREEN, COLOR_BG);
  canvas.drawString("steps", 120, 166);

  canvas.setTextDatum(bottom_center);
  canvas.setFont(&fonts::Font0);
  if (bma423Ready) {
    canvas.setTextColor(bma423HwPedometerReady ? COLOR_GREEN : COLOR_SUBTEXT, COLOR_BG);
    canvas.drawString(bma423HwPedometerReady ? "BMA423 HW Pedometer" : "BMA423 Active", 120, 228);
  } else {
    canvas.setTextColor(COLOR_RED, COLOR_BG);
    canvas.drawString("BMA423 Error", 120, 228);
  }
}

// Draw bubble Inclinometer screen (SCREEN_INCLINOMETER)
void drawInclinometerScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  const int16_t cx = 120;
  const int16_t cy = 109;
  const int16_t outerR = 65;
  const int16_t innerR = 61;
  const int16_t bubbleR = 10;
  const int16_t maxTravel = innerR - bubbleR - 3;

  bool isLevel = bma423Ready && (fabsf(inclAngleX) <= 1.5f) && (fabsf(inclAngleY) <= 1.5f);

  // Header / level indicator below top status bar
  canvas.setTextDatum(top_center);
  canvas.setFont(&fonts::Font0);
  if (!bma423Ready) {
    canvas.setTextColor(COLOR_RED, COLOR_BG);
    canvas.drawString("BMA423 ERROR", 120, 32);
  } else if (isLevel) {
    canvas.setTextColor(COLOR_GREEN, COLOR_BG);
    canvas.drawString("LEVEL", 120, 32);
  } else {
    canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
    canvas.drawString("INCLINOMETER", 120, 32);
  }

  // Outer housing and fluid disc of bubble level
  uint16_t bezelCol = isLevel ? COLOR_GREEN : COLOR_ACCENT;
  canvas.fillCircle(cx, cy, outerR, COLOR_HEX_FILL);
  canvas.drawCircle(cx, cy, outerR, COLOR_HEX_BORDER);
  canvas.drawCircle(cx, cy, outerR - 1, bezelCol);
  canvas.fillCircle(cx, cy, innerR, 0x09C4);
  canvas.fillCircle(cx, cy, 44, 0x0AC5);

  // Concentric angle rings (15, 30, 45 deg) and crosshair
  canvas.drawCircle(cx, cy, 48, 0x1D48);
  canvas.drawCircle(cx, cy, 32, 0x1D48);
  canvas.drawCircle(cx, cy, 16, 0x2E8A);

  canvas.drawLine(cx - innerR + 3, cy, cx + innerR - 3, cy, 0x1D48);
  canvas.drawLine(cx, cy - innerR + 3, cx, cy + innerR - 3, 0x1D48);

  // Axis tick marks every 15 degrees
  for (int16_t step = 16; step <= 48; step += 16) {
    canvas.drawLine(cx - step, cy - 3, cx - step, cy + 3, 0x2E8A);
    canvas.drawLine(cx + step, cy - 3, cx + step, cy + 3, 0x2E8A);
    canvas.drawLine(cx - 3, cy - step, cx + 3, cy - step, 0x2E8A);
    canvas.drawLine(cx - 3, cy + step, cx + 3, cy + step, 0x2E8A);
  }

  // Air bubble position (bubble floats toward the raised side on tilt)
  float normX = -inclAngleX / 45.0f;
  float normY = -inclAngleY / 45.0f;
  float normDist = sqrtf(normX * normX + normY * normY);
  if (normDist > 1.0f) {
    normX /= normDist;
    normY /= normDist;
  }

  int16_t bx = cx + (int16_t)roundf(normX * (float)maxTravel);
  int16_t by = cy + (int16_t)roundf(normY * (float)maxTravel);

  // Draw bubble with drop shadow and specular highlight
  if (bma423Ready) {
    uint16_t bubbleFill = isLevel ? 0x67E8 : ((fabsf(inclAngleX) > 25.0f || fabsf(inclAngleY) > 25.0f) ? COLOR_WARN : 0x47FE);
    uint16_t bubbleCore = isLevel ? 0xAFE9 : ((fabsf(inclAngleX) > 25.0f || fabsf(inclAngleY) > 25.0f) ? 0xFE60 : 0x9FFF);

    canvas.fillCircle(bx + 2, by + 2, bubbleR, 0x0102);
    canvas.fillCircle(bx, by, bubbleR, bubbleFill);
    canvas.fillCircle(bx - 1, by - 1, bubbleR - 3, bubbleCore);
    canvas.drawCircle(bx, by, bubbleR, COLOR_TEXT);
    canvas.fillCircle(bx - 3, by - 3, 2, COLOR_TEXT);
    canvas.drawPixel(bx - 5, by - 5, COLOR_TEXT);
  }

  // Center bullseye reference ring over fluid and bubble
  uint16_t targetCol = isLevel ? COLOR_GREEN : COLOR_TEXT;
  canvas.drawCircle(cx, cy, bubbleR + 3, targetCol);
  canvas.drawLine(cx - 4, cy, cx + 4, cy, targetCol);
  canvas.drawLine(cx, cy - 4, cx, cy + 4, targetCol);

  // Bottom numeric readout panel for X and Y tilt angles
  int16_t boxY = 182;
  int16_t boxW = 108;
  int16_t boxH = 50;
  int16_t leftBoxX  = 8;
  int16_t rightBoxX = 124;

  bool xLevel = bma423Ready && (fabsf(inclAngleX) <= 1.5f);
  bool yLevel = bma423Ready && (fabsf(inclAngleY) <= 1.5f);

  canvas.fillRoundRect(leftBoxX, boxY, boxW, boxH, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(leftBoxX, boxY, boxW, boxH, 6, xLevel ? COLOR_GREEN : COLOR_ACCENT);

  canvas.fillRoundRect(rightBoxX, boxY, boxW, boxH, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(rightBoxX, boxY, boxW, boxH, 6, yLevel ? COLOR_GREEN : COLOR_ACCENT);

  canvas.setTextDatum(top_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
  canvas.drawString("X ANGLE", leftBoxX + boxW / 2, boxY + 5);
  canvas.drawString("Y ANGLE", rightBoxX + boxW / 2, boxY + 5);

  char bufX[16];
  char bufY[16];
  if (bma423Ready) {
    float dispX = (fabsf(inclAngleX) < 0.05f) ? 0.0f : inclAngleX;
    float dispY = (fabsf(inclAngleY) < 0.05f) ? 0.0f : inclAngleY;
    snprintf(bufX, sizeof(bufX), "%+.1f", dispX);
    snprintf(bufY, sizeof(bufY), "%+.1f", dispY);
  } else {
    snprintf(bufX, sizeof(bufX), "--.-");
    snprintf(bufY, sizeof(bufY), "--.-");
  }

  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font4);

  uint16_t colX = bma423Ready ? (xLevel ? COLOR_GREEN : COLOR_TEXT) : COLOR_RED;
  int16_t xCenter = leftBoxX + boxW / 2 - 4;
  canvas.setTextColor(colX, COLOR_HEX_FILL);
  canvas.drawString(bufX, xCenter, boxY + 32);
  int16_t degX1 = xCenter + canvas.textWidth(bufX) / 2 + 5;
  canvas.drawCircle(degX1, boxY + 23, 2, colX);

  uint16_t colY = bma423Ready ? (yLevel ? COLOR_GREEN : COLOR_TEXT) : COLOR_RED;
  int16_t yCenter = rightBoxX + boxW / 2 - 4;
  canvas.setTextColor(colY, COLOR_HEX_FILL);
  canvas.drawString(bufY, yCenter, boxY + 32);
  int16_t degX2 = yCenter + canvas.textWidth(bufY) / 2 + 5;
  canvas.drawCircle(degX2, boxY + 23, 2, colY);
}

// Load saved Wi-Fi networks (up to 64) from non-volatile Flash (Preferences)
void loadSavedWifiCredentials() {
  savedWifiCount = 0;
  if (!wifiPrefs.begin("wifi_creds", true)) return;
  uint8_t cnt = wifiPrefs.getUChar("count", 0);
  if (cnt > WIFI_SAVED_MAX) cnt = WIFI_SAVED_MAX;

  for (uint8_t i = 0; i < cnt; i++) {
    char keyS[8], keyP[8];
    snprintf(keyS, sizeof(keyS), "s%u", i);
    snprintf(keyP, sizeof(keyP), "p%u", i);
    String s = wifiPrefs.getString(keyS, "");
    String p = wifiPrefs.getString(keyP, "");
    if (s.length() > 0) {
      strncpy(savedWifiList[savedWifiCount].ssid, s.c_str(), 32);
      savedWifiList[savedWifiCount].ssid[32] = '\0';
      strncpy(savedWifiList[savedWifiCount].pass, p.c_str(), 64);
      savedWifiList[savedWifiCount].pass[64] = '\0';
      savedWifiCount++;
    }
  }
  wifiPrefs.end();
}

// Save SSID/password pair to Flash in LIFO order (up to 64 most recent networks)
void saveWifiCredentialLIFO(const char* ssid, const char* pass) {
  if (ssid == NULL || ssid[0] == '\0') return;

  // Remove duplicate if network was already saved
  int8_t existingIdx = -1;
  for (uint8_t i = 0; i < savedWifiCount; i++) {
    if (strcmp(savedWifiList[i].ssid, ssid) == 0) {
      existingIdx = (int8_t)i;
      break;
    }
  }
  if (existingIdx >= 0) {
    for (uint8_t i = (uint8_t)existingIdx; i + 1 < savedWifiCount; i++) {
      savedWifiList[i] = savedWifiList[i + 1];
    }
    savedWifiCount--;
  }

  // Shift down in LIFO order: newest entry is always inserted at index 0
  uint8_t shiftEnd = (savedWifiCount < WIFI_SAVED_MAX) ? savedWifiCount : (WIFI_SAVED_MAX - 1);
  for (int8_t i = (int8_t)shiftEnd; i > 0; i--) {
    savedWifiList[i] = savedWifiList[i - 1];
  }
  strncpy(savedWifiList[0].ssid, ssid, 32);
  savedWifiList[0].ssid[32] = '\0';
  strncpy(savedWifiList[0].pass, (pass != NULL) ? pass : "", 64);
  savedWifiList[0].pass[64] = '\0';

  if (savedWifiCount < WIFI_SAVED_MAX) {
    savedWifiCount++;
  }

  // Write updated list to Flash via ESP32 Preferences
  if (!wifiPrefs.begin("wifi_creds", false)) return;
  wifiPrefs.putUChar("count", savedWifiCount);
  for (uint8_t i = 0; i < savedWifiCount; i++) {
    char keyS[8], keyP[8];
    snprintf(keyS, sizeof(keyS), "s%u", i);
    snprintf(keyP, sizeof(keyP), "p%u", i);
    wifiPrefs.putString(keyS, savedWifiList[i].ssid);
    wifiPrefs.putString(keyP, savedWifiList[i].pass);
  }
  wifiPrefs.end();
}

// Delete a saved Wi-Fi network by index from RAM list and Flash
void deleteSavedWifiCredential(uint8_t index) {
  if (index >= savedWifiCount) return;

  for (uint8_t i = index; i + 1 < savedWifiCount; i++) {
    savedWifiList[i] = savedWifiList[i + 1];
  }
  savedWifiCount--;

  if (wifiPrefs.begin("wifi_creds", false)) {
    wifiPrefs.putUChar("count", savedWifiCount);
    for (uint8_t i = 0; i < savedWifiCount; i++) {
      char keyS[8], keyP[8];
      snprintf(keyS, sizeof(keyS), "s%u", i);
      snprintf(keyP, sizeof(keyP), "p%u", i);
      wifiPrefs.putString(keyS, savedWifiList[i].ssid);
      wifiPrefs.putString(keyP, savedWifiList[i].pass);
    }
    char staleS[8], staleP[8];
    snprintf(staleS, sizeof(staleS), "s%u", savedWifiCount);
    snprintf(staleP, sizeof(staleP), "p%u", savedWifiCount);
    wifiPrefs.remove(staleS);
    wifiPrefs.remove(staleP);
    wifiPrefs.end();
  }

  if (wifiListScrollOffset > 0 && (uint16_t)wifiListScrollOffset + WIFI_VISIBLE_ROWS > savedWifiCount) {
    wifiListScrollOffset = (savedWifiCount > WIFI_VISIBLE_ROWS)
                             ? (uint8_t)(savedWifiCount - WIFI_VISIBLE_ROWS)
                             : 0;
  }
}

// Brightness helpers (5..100% -> 13..255 PWM, preventing zero active brightness) and settings load/save
uint8_t getBrightnessPwmFromPercent(uint8_t pct) {
  if (pct < BRIGHTNESS_MIN_PCT) pct = BRIGHTNESS_MIN_PCT;
  if (pct > 100) pct = 100;
  uint8_t pwm = (uint8_t)(((uint16_t)pct * 255U + 50U) / 100U);
  if (pwm < BRIGHTNESS_MIN_PWM) pwm = BRIGHTNESS_MIN_PWM;
  return pwm;
}

uint8_t getFullBrightnessPwm() {
  return getBrightnessPwmFromPercent(settingDisplayBrightness);
}

uint8_t getHalfBrightnessPwm() {
  uint8_t halfPwm = (uint8_t)(getFullBrightnessPwm() / 2U);
  if (halfPwm < BRIGHTNESS_MIN_PWM) halfPwm = BRIGHTNESS_MIN_PWM;
  return halfPwm;
}

uint8_t findDisplayTimeOptionIdx(uint16_t sec) {
  for (uint8_t i = 0; i < DISPLAY_TIME_OPT_COUNT; i++) {
    if (DISPLAY_TIME_OPTIONS[i] == sec) return i;
  }
  return 4; // Default 30 seconds
}

void loadWatchSettings() {
  settingWifiEnabled       = true;
  settingWifiAutoConnect   = true;
  settingDisplayBrightness = 94;
  settingDisplayLightSec   = 30;
  settingTouchVibration    = true;
  settingTouchSensitivity  = TOUCH_SENS_DEFAULT_VAL;
  settingWakeTapCount      = 2;
  settingChargeCurrentMa   = 200;
  settingPowerGps          = true;
  settingPowerLora         = true;
  settingWakeOnTouch       = true;
  settingWakeOnBma423      = true;
  settingWakeOnButton      = true;

  if (!settingsPrefs.begin("watch_cfg", true)) return;
  settingWifiEnabled     = settingsPrefs.getBool("wifi_en", true);
  settingWifiAutoConnect = settingsPrefs.getBool("wifi_auto", true);
  uint8_t bri = settingsPrefs.getUChar("disp_bri", 94);
  if (bri < BRIGHTNESS_MIN_PCT) bri = BRIGHTNESS_MIN_PCT;
  if (bri > 100) bri = 100;
  settingDisplayBrightness = bri;

  uint16_t sec = settingsPrefs.getUShort("disp_sec", 30);
  settingDisplayLightSec = DISPLAY_TIME_OPTIONS[findDisplayTimeOptionIdx(sec)];
  settingTouchVibration  = settingsPrefs.getBool("touch_vib", true);

  uint8_t tSens = settingsPrefs.getUChar("touch_sens", TOUCH_SENS_DEFAULT_VAL);
  if (tSens < TOUCH_SENS_MIN_VAL || tSens > TOUCH_SENS_MAX_VAL) tSens = TOUCH_SENS_DEFAULT_VAL;
  settingTouchSensitivity = tSens;

  uint8_t wTaps = settingsPrefs.getUChar("wake_taps", 2);
  settingWakeTapCount = (wTaps == 3) ? 3 : 2;

  uint16_t chgMa = settingsPrefs.getUShort("chg_ma", 200);
  if (chgMa != 100 && chgMa != 200 && chgMa != 300 && chgMa != 400) chgMa = 200;
  settingChargeCurrentMa = chgMa;
  settingPowerGps        = settingsPrefs.getBool("pwr_gps", true);
  settingPowerLora       = settingsPrefs.getBool("pwr_lora", true);
  settingWakeOnTouch     = settingsPrefs.getBool("wake_tch", true);
  settingWakeOnBma423    = settingsPrefs.getBool("wake_bma", true);
  settingWakeOnButton    = settingsPrefs.getBool("wake_btn", true);
  if (!settingWakeOnTouch && !settingWakeOnBma423 && !settingWakeOnButton) {
    settingWakeOnButton = true;
  }

  stepCount          = settingsPrefs.getULong("step_cnt", 0);
  stepCountBase      = stepCount;
  lastSavedStepCount = stepCount;

  weatherAutoLocate      = settingsPrefs.getBool("w_auto", true);
  uint8_t cIdx           = settingsPrefs.getUChar("w_city", 0);
  weatherCityIdx         = (cIdx < WORLD_CAPITALS_COUNT) ? cIdx : 0;

  // Load settings for 2 alarms and timer ringtone
  alarms[0].hour     = settingsPrefs.getUChar("al0_h", 7) % 24;
  alarms[0].minute   = settingsPrefs.getUChar("al0_m", 0) % 60;
  alarms[0].enabled  = settingsPrefs.getBool("al0_on", false);
  alarms[0].ringtone = (settingsPrefs.getUChar("al0_t", 0) == 1) ? RINGTONE_BELL : RINGTONE_BEEP;
  uint8_t sn0        = settingsPrefs.getUChar("al0_sn", 5);
  alarms[0].snoozeMin = (sn0 >= 5 && sn0 <= 30) ? sn0 : 5;

  alarms[1].hour     = settingsPrefs.getUChar("al1_h", 8) % 24;
  alarms[1].minute   = settingsPrefs.getUChar("al1_m", 30) % 60;
  alarms[1].enabled  = settingsPrefs.getBool("al1_on", false);
  alarms[1].ringtone = (settingsPrefs.getUChar("al1_t", 1) == 1) ? RINGTONE_BELL : RINGTONE_BEEP;
  uint8_t sn1        = settingsPrefs.getUChar("al1_sn", 10);
  alarms[1].snoozeMin = (sn1 >= 5 && sn1 <= 30) ? sn1 : 10;

  timerRingtone      = (settingsPrefs.getUChar("tmr_t", 0) == 1) ? RINGTONE_BELL : RINGTONE_BEEP;
  settingsPrefs.end();
}

void saveWatchSettings() {
  if (settingDisplayBrightness < BRIGHTNESS_MIN_PCT) settingDisplayBrightness = BRIGHTNESS_MIN_PCT;
  if (settingDisplayBrightness > 100) settingDisplayBrightness = 100;
  if (settingTouchSensitivity < TOUCH_SENS_MIN_VAL) settingTouchSensitivity = TOUCH_SENS_MIN_VAL;
  if (settingTouchSensitivity > TOUCH_SENS_MAX_VAL) settingTouchSensitivity = TOUCH_SENS_MAX_VAL;
  if (settingWakeTapCount != 2 && settingWakeTapCount != 3) settingWakeTapCount = 2;
  if (settingChargeCurrentMa != 100 && settingChargeCurrentMa != 200 &&
      settingChargeCurrentMa != 300 && settingChargeCurrentMa != 400) {
    settingChargeCurrentMa = 200;
  }
  if (!settingWakeOnTouch && !settingWakeOnBma423 && !settingWakeOnButton) {
    settingWakeOnButton = true;
  }
  if (!settingsPrefs.begin("watch_cfg", false)) return;
  settingsPrefs.putBool("wifi_en", settingWifiEnabled);
  settingsPrefs.putBool("wifi_auto", settingWifiAutoConnect);
  settingsPrefs.putUChar("disp_bri", settingDisplayBrightness);
  settingsPrefs.putUShort("disp_sec", settingDisplayLightSec);
  settingsPrefs.putBool("touch_vib", settingTouchVibration);
  settingsPrefs.putUChar("touch_sens", settingTouchSensitivity);
  settingsPrefs.putUChar("wake_taps", settingWakeTapCount);
  settingsPrefs.putUShort("chg_ma", settingChargeCurrentMa);
  settingsPrefs.putBool("pwr_gps", settingPowerGps);
  settingsPrefs.putBool("pwr_lora", settingPowerLora);
  settingsPrefs.putBool("wake_tch", settingWakeOnTouch);
  settingsPrefs.putBool("wake_bma", settingWakeOnBma423);
  settingsPrefs.putBool("wake_btn", settingWakeOnButton);
  settingsPrefs.putBool("w_auto", weatherAutoLocate);
  settingsPrefs.putUChar("w_city", weatherCityIdx);

  settingsPrefs.putUChar("al0_h", alarms[0].hour);
  settingsPrefs.putUChar("al0_m", alarms[0].minute);
  settingsPrefs.putBool("al0_on", alarms[0].enabled);
  settingsPrefs.putUChar("al0_t", (uint8_t)alarms[0].ringtone);
  settingsPrefs.putUChar("al0_sn", alarms[0].snoozeMin);

  settingsPrefs.putUChar("al1_h", alarms[1].hour);
  settingsPrefs.putUChar("al1_m", alarms[1].minute);
  settingsPrefs.putBool("al1_on", alarms[1].enabled);
  settingsPrefs.putUChar("al1_t", (uint8_t)alarms[1].ringtone);
  settingsPrefs.putUChar("al1_sn", alarms[1].snoozeMin);

  settingsPrefs.putUChar("tmr_t", (uint8_t)timerRingtone);
  settingsPrefs.end();
}

// Periodic save of step counter to Flash (every 10 minutes or on reset)
void saveStepCountToFlash(bool force) {
  lastStepSaveMs = millis();
  if (!force && stepCount == lastSavedStepCount) return;
  if (!settingsPrefs.begin("watch_cfg", false)) return;
  settingsPrefs.putULong("step_cnt", stepCount);
  settingsPrefs.end();
  lastSavedStepCount = stepCount;
}

// Release BLE controller cleanly before Wi-Fi scans/connections so 2.4 GHz radio is free
void releaseBleRadioForWifi() {
  if (!bleHidKbActive && !bleHidMouseActive) {
    if (BLEDevice::getInitialized()) {
      BLEScan* pScan = BLEDevice::getScan();
      if (pScan != NULL) {
        pScan->stop();
        pScan->clearResults();
      }
      BLEDevice::stopAdvertising();
      BLEDevice::deinit(false);
      delay(50);
    }
    bleStackInitialized = false;
  }
  btWifiPausedForBle = false;
  if (btWifiSleepForBle) {
    btWifiSleepForBle = false;
  }
  WiFi.setSleep(false);
}

// Scan nearby Wi-Fi networks for Wi-Fi connect screen
void scanWifiNetworks() {
  stopTouchVibrationNow();
  if (!settingWifiEnabled) {
    wifiScanCount = 0;
    wifiScrollOffset = 0;
    return;
  }
  releaseBleRadioForWifi();
  wifiBgScanning = false;
  WiFi.scanDelete();
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect(false);
  }
  delay(60);

  wifiScanCount = 0;
  wifiScrollOffset = 0;

  int16_t n = WiFi.scanNetworks(false, false);
  if (n <= 0) {
    // One clean recovery retry in case the 2.4 GHz radio was just handed back from BLE
    WiFi.scanDelete();
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.disconnect(false);
    }
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    delay(120);
    n = WiFi.scanNetworks(false, false);
  }
  if (n > 0) {
    for (int16_t i = 0; i < n && wifiScanCount < WIFI_SCAN_MAX; i++) {
      String ssidStr = WiFi.SSID(i);
      if (ssidStr.length() == 0) continue;

      // Skip duplicate SSIDs (keep access point with strongest RSSI)
      bool duplicate = false;
      for (uint8_t j = 0; j < wifiScanCount; j++) {
        if (strcmp(wifiScanList[j].ssid, ssidStr.c_str()) == 0) {
          if (WiFi.RSSI(i) > wifiScanList[j].rssi) {
            wifiScanList[j].rssi = WiFi.RSSI(i);
          }
          duplicate = true;
          break;
        }
      }
      if (duplicate) continue;

      strncpy(wifiScanList[wifiScanCount].ssid, ssidStr.c_str(), 32);
      wifiScanList[wifiScanCount].ssid[32] = '\0';
      wifiScanList[wifiScanCount].rssi = WiFi.RSSI(i);
      wifiScanList[wifiScanCount].isLocked = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
      wifiScanCount++;
    }
  }
  WiFi.scanDelete();

  // Sort network list by descending signal strength (RSSI)
  for (uint8_t i = 0; i + 1 < wifiScanCount; i++) {
    for (uint8_t j = i + 1; j < wifiScanCount; j++) {
      if (wifiScanList[j].rssi > wifiScanList[i].rssi) {
        WifiScanItem tmp = wifiScanList[i];
        wifiScanList[i] = wifiScanList[j];
        wifiScanList[j] = tmp;
      }
    }
  }
}

// Start connection to selected Wi-Fi network
void startWifiConnectAttempt() {
  if (!settingWifiEnabled) {
    wifiConnectSuccess = false;
    wifiView = WIFI_VIEW_RESULT;
    return;
  }
  releaseBleRadioForWifi();
  wifiBgScanning = false;
  WiFi.scanDelete();
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(40);

  if (wifiSelectedLocked) {
    WiFi.begin(wifiSelectedSsid, wifiPasswordBuf);
  } else {
    WiFi.begin(wifiSelectedSsid);
  }

  wifiConnectStartMs = millis();
  wifiView = WIFI_VIEW_CONNECTING;
}

// Start connection to saved network from Wi-Fi list screen
void startWifiListConnectAttempt() {
  if (wifiListSelectedIdx < 0 || wifiListSelectedIdx >= savedWifiCount) return;
  const SavedWifiCred& cred = savedWifiList[wifiListSelectedIdx];

  strncpy(wifiListConnectSsid, cred.ssid, 32);
  wifiListConnectSsid[32] = '\0';

  if (!settingWifiEnabled) {
    wifiListConnectSuccess = false;
    wifiListView = WIFILIST_VIEW_RESULT;
    return;
  }

  releaseBleRadioForWifi();
  wifiBgScanning = false;
  WiFi.scanDelete();
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(40);

  if (cred.pass[0] != '\0') {
    WiFi.begin(cred.ssid, cred.pass);
  } else {
    WiFi.begin(cred.ssid);
  }

  wifiListConnectStartMs = millis();
  wifiListView = WIFILIST_VIEW_CONNECTING;
}

// Start asynchronous Wi-Fi scan for channel analyzer (channels 1..11)
void startWifiAnalyzerScan() {
  if (!settingWifiEnabled) {
    wifiAnalyzerScanning = false;
    return;
  }
  releaseBleRadioForWifi();
  wifiBgScanning = false;
  WiFi.scanDelete();
  WiFi.mode(WIFI_STA);
  int16_t res = WiFi.scanNetworks(true, true);
  wifiAnalyzerScanning = (res == WIFI_SCAN_RUNNING);
  lastWifiAnalyzerScanMs = millis();
}

// 802.11 promiscuous packet callback counting Deauth (0x0C) and Disassociation (0x0A) frames
void IRAM_ATTR wifiJamPromiscuousCb(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT || buf == nullptr) return;
  const wifi_promiscuous_pkt_t* pkt = (const wifi_promiscuous_pkt_t*)buf;
  if (pkt->rx_ctrl.sig_len < 2) return;
  uint8_t fc0 = pkt->payload[0];
  uint8_t frameType = (fc0 >> 2) & 0x03;
  uint8_t frameSub  = (fc0 >> 4) & 0x0F;
  // Management frame (0x00), Subtype 0x0C (Deauthentication) or 0x0A (Disassociation)
  if (frameType == 0x00 && (frameSub == 0x0C || frameSub == 0x0A)) {
    portENTER_CRITICAL_ISR(&wifiJamMux);
    wifiJamCurrPkts++;
    portEXIT_CRITICAL_ISR(&wifiJamMux);
  }
}

// Start Wi-Fi jam detector (hop channels 1..11 in promiscuous mode)
void startWifiJamDetect() {
  if (!settingWifiEnabled) {
    wifiJamActive = false;
    return;
  }
  releaseBleRadioForWifi();
  wifiBgScanning = false;
  WiFi.scanDelete();
  wifiJamWasConnected = (WiFi.status() == WL_CONNECTED);
  WiFi.mode(WIFI_STA);
  if (wifiJamWasConnected) {
    WiFi.disconnect(false);
  }
  memset(wifiJamPps, 0, sizeof(wifiJamPps));
  portENTER_CRITICAL(&wifiJamMux);
  wifiJamCurrPkts = 0;
  portEXIT_CRITICAL(&wifiJamMux);

  wifi_promiscuous_filter_t filt;
  filt.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
  esp_wifi_set_promiscuous_filter(&filt);
  esp_wifi_set_promiscuous_rx_cb(&wifiJamPromiscuousCb);
  esp_wifi_set_promiscuous(true);

  wifiJamCurrChan = 1;
  esp_wifi_set_channel(wifiJamCurrChan, WIFI_SECOND_CHAN_NONE);
  wifiJamChanStartMs = millis();
  wifiJamActive = true;
}

// Stop promiscuous mode and restore normal STA operation
void stopWifiJamDetect() {
  if (!wifiJamActive) return;
  esp_wifi_set_promiscuous(false);
  esp_wifi_set_promiscuous_rx_cb(nullptr);
  wifiJamActive = false;
  if (wifiJamWasConnected && settingWifiEnabled) {
    WiFi.reconnect();
  }
  wifiJamWasConnected = false;
}

// Apply global Wi-Fi enabled/disabled power state immediately
void applyWifiPowerState() {
  if (!settingWifiEnabled) {
    if (wifiJamActive) {
      stopWifiJamDetect();
    }
    if (hostsScanRunning) {
      stopHostsScan();
    }
    wifiAnalyzerScanning = false;
    wifiBgScanning = false;
    WiFi.scanDelete();
    WiFi.disconnect(true, false);
    esp_wifi_stop();
    WiFi.mode(WIFI_OFF);
    wifiInfoConnected = false;
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    if (settingWifiAutoConnect) {
      lastWifiAutoCheckMs = 0;
    }
  }
}

// Identify AP vendor from OUI (first 3 bytes of BSSID)
const char* lookupOuiVendor(const uint8_t bssid[6]) {
  for (uint16_t i = 0; i < WIFI_OUI_COUNT; i++) {
    if (WIFI_OUI_TABLE[i].b0 == bssid[0] &&
        WIFI_OUI_TABLE[i].b1 == bssid[1] &&
        WIFI_OUI_TABLE[i].b2 == bssid[2]) {
      return WIFI_OUI_TABLE[i].vendor;
    }
  }
  if (bssid[0] & 0x02) {
    return "Local / Random MAC";
  }
  return "Unknown";
}

// Human-readable 802.11 authentication mode name
const char* getWifiAuthModeStr(wifi_auth_mode_t mode) {
  switch (mode) {
    case WIFI_AUTH_OPEN:            return "OPEN";
    case WIFI_AUTH_WEP:             return "WEP";
    case WIFI_AUTH_WPA_PSK:         return "WPA-PSK";
    case WIFI_AUTH_WPA2_PSK:        return "WPA2-PSK";
    case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA/WPA2-PSK";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-EAP";
    case WIFI_AUTH_WPA3_PSK:        return "WPA3-SAE";
    case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2/WPA3";
    case WIFI_AUTH_WAPI_PSK:        return "WAPI-PSK";
    default:                        return "UNKNOWN";
  }
}

// Human-readable cipher name (unicast/group cipher)
const char* getWifiCipherStr(wifi_cipher_type_t cipher) {
  switch (cipher) {
    case WIFI_CIPHER_TYPE_NONE:        return "NONE";
    case WIFI_CIPHER_TYPE_WEP40:       return "WEP40";
    case WIFI_CIPHER_TYPE_WEP104:      return "WEP104";
    case WIFI_CIPHER_TYPE_TKIP:        return "TKIP";
    case WIFI_CIPHER_TYPE_CCMP:        return "AES-CCMP";
    case WIFI_CIPHER_TYPE_TKIP_CCMP:   return "TKIP+CCMP";
    case WIFI_CIPHER_TYPE_AES_CMAC128: return "AES-CMAC128";
    case WIFI_CIPHER_TYPE_SMS4:        return "SMS4";
    default:                           return "UNKNOWN";
  }
}

// Quick internet reachability check via short TCP connect to public DNS (1.1.1.1 / 8.8.8.8)
bool probeInternetReachable() {
  WiFiClient probeClient;
  if (probeClient.connect(IPAddress(1, 1, 1, 1), 53, 800)) {
    probeClient.stop();
    return true;
  }
  if (probeClient.connect(IPAddress(8, 8, 8, 8), 53, 800)) {
    probeClient.stop();
    return true;
  }
  return false;
}

// Populate all 14 parameters of the currently connected Wi-Fi access point
void refreshWifiInfoData() {
  wifiInfoConnected = (WiFi.status() == WL_CONNECTED);
  if (!wifiInfoConnected) {
    wifiInfoInternetOk = false;
    return;
  }

  wifi_ap_record_t apInfo;
  memset(&apInfo, 0, sizeof(apInfo));
  bool hasApInfo = (esp_wifi_sta_get_ap_info(&apInfo) == ESP_OK);

  String ssidStr = WiFi.SSID();
  if (ssidStr.length() == 0 && hasApInfo) {
    ssidStr = String((const char*)apInfo.ssid);
  }

  // 1. SSID
  strncpy(wifiInfoRows[0].label, "SSID", sizeof(wifiInfoRows[0].label) - 1);
  wifiInfoRows[0].label[sizeof(wifiInfoRows[0].label) - 1] = '\0';
  strncpy(wifiInfoRows[0].value, ssidStr.length() > 0 ? ssidStr.c_str() : "<hidden>", sizeof(wifiInfoRows[0].value) - 1);
  wifiInfoRows[0].value[sizeof(wifiInfoRows[0].value) - 1] = '\0';

  // 2. Saved password
  strncpy(wifiInfoRows[1].label, "SAVED PASSWORD", sizeof(wifiInfoRows[1].label) - 1);
  wifiInfoRows[1].label[sizeof(wifiInfoRows[1].label) - 1] = '\0';
  bool foundSaved = false;
  for (uint8_t i = 0; i < savedWifiCount; i++) {
    if (strcmp(savedWifiList[i].ssid, ssidStr.c_str()) == 0) {
      foundSaved = true;
      if (savedWifiList[i].pass[0] == '\0') {
        strncpy(wifiInfoRows[1].value, "<open network>", sizeof(wifiInfoRows[1].value) - 1);
      } else {
        strncpy(wifiInfoRows[1].value, savedWifiList[i].pass, sizeof(wifiInfoRows[1].value) - 1);
      }
      wifiInfoRows[1].value[sizeof(wifiInfoRows[1].value) - 1] = '\0';
      break;
    }
  }
  if (!foundSaved) {
    String pskStr = WiFi.psk();
    if (pskStr.length() > 0) {
      strncpy(wifiInfoRows[1].value, pskStr.c_str(), sizeof(wifiInfoRows[1].value) - 1);
    } else if (hasApInfo && apInfo.authmode == WIFI_AUTH_OPEN) {
      strncpy(wifiInfoRows[1].value, "<open network>", sizeof(wifiInfoRows[1].value) - 1);
    } else {
      strncpy(wifiInfoRows[1].value, "<not saved>", sizeof(wifiInfoRows[1].value) - 1);
    }
    wifiInfoRows[1].value[sizeof(wifiInfoRows[1].value) - 1] = '\0';
  }

  // 3. Internet reachable
  wifiInfoInternetOk = probeInternetReachable();
  strncpy(wifiInfoRows[2].label, "INTERNET", sizeof(wifiInfoRows[2].label) - 1);
  wifiInfoRows[2].label[sizeof(wifiInfoRows[2].label) - 1] = '\0';
  strncpy(wifiInfoRows[2].value, wifiInfoInternetOk ? "Reachable (Online)" : "Unreachable (Offline)", sizeof(wifiInfoRows[2].value) - 1);
  wifiInfoRows[2].value[sizeof(wifiInfoRows[2].value) - 1] = '\0';

  // 4. Supported PHY modes
  strncpy(wifiInfoRows[3].label, "PHY MODES", sizeof(wifiInfoRows[3].label) - 1);
  wifiInfoRows[3].label[sizeof(wifiInfoRows[3].label) - 1] = '\0';
  if (hasApInfo) {
    char phyBuf[32] = "802.11 ";
    bool firstMode = true;
    if (apInfo.phy_11b) { strcat(phyBuf, firstMode ? "b" : "/b"); firstMode = false; }
    if (apInfo.phy_11g) { strcat(phyBuf, firstMode ? "g" : "/g"); firstMode = false; }
    if (apInfo.phy_11n) { strcat(phyBuf, firstMode ? "n" : "/n"); firstMode = false; }
    if (apInfo.phy_lr)  { strcat(phyBuf, firstMode ? "LR" : "/LR"); firstMode = false; }
    if (firstMode) { strcpy(phyBuf, "Unknown"); }
    strncpy(wifiInfoRows[3].value, phyBuf, sizeof(wifiInfoRows[3].value) - 1);
  } else {
    strncpy(wifiInfoRows[3].value, "Unknown", sizeof(wifiInfoRows[3].value) - 1);
  }
  wifiInfoRows[3].value[sizeof(wifiInfoRows[3].value) - 1] = '\0';

  // 5. Signal strength (RSSI)
  int32_t rssi = hasApInfo ? (int32_t)apInfo.rssi : WiFi.RSSI();
  int32_t qual = 2 * (rssi + 100);
  if (qual < 0) qual = 0;
  if (qual > 100) qual = 100;
  strncpy(wifiInfoRows[4].label, "SIGNAL STRENGTH", sizeof(wifiInfoRows[4].label) - 1);
  wifiInfoRows[4].label[sizeof(wifiInfoRows[4].label) - 1] = '\0';
  snprintf(wifiInfoRows[4].value, sizeof(wifiInfoRows[4].value), "%ld dBm (%ld%%)", (long)rssi, (long)qual);

  // 6. Gateway IP
  strncpy(wifiInfoRows[5].label, "GATEWAY IP", sizeof(wifiInfoRows[5].label) - 1);
  wifiInfoRows[5].label[sizeof(wifiInfoRows[5].label) - 1] = '\0';
  String gwStr = WiFi.gatewayIP().toString();
  strncpy(wifiInfoRows[5].value, gwStr.c_str(), sizeof(wifiInfoRows[5].value) - 1);
  wifiInfoRows[5].value[sizeof(wifiInfoRows[5].value) - 1] = '\0';

  // 7. Local IP
  strncpy(wifiInfoRows[6].label, "LOCAL IP", sizeof(wifiInfoRows[6].label) - 1);
  wifiInfoRows[6].label[sizeof(wifiInfoRows[6].label) - 1] = '\0';
  String ipStr = WiFi.localIP().toString();
  strncpy(wifiInfoRows[6].value, ipStr.c_str(), sizeof(wifiInfoRows[6].value) - 1);
  wifiInfoRows[6].value[sizeof(wifiInfoRows[6].value) - 1] = '\0';

  // 8. Channel & channel width
  strncpy(wifiInfoRows[7].label, "CHANNEL / WIDTH", sizeof(wifiInfoRows[7].label) - 1);
  wifiInfoRows[7].label[sizeof(wifiInfoRows[7].label) - 1] = '\0';
  if (hasApInfo) {
    const char* bwStr = "20 MHz";
    if (apInfo.second == WIFI_SECOND_CHAN_ABOVE) bwStr = "40 MHz (+)";
    else if (apInfo.second == WIFI_SECOND_CHAN_BELOW) bwStr = "40 MHz (-)";
    snprintf(wifiInfoRows[7].value, sizeof(wifiInfoRows[7].value), "Ch %u (%s)", (unsigned)apInfo.primary, bwStr);
  } else {
    snprintf(wifiInfoRows[7].value, sizeof(wifiInfoRows[7].value), "Ch %ld (20 MHz)", (long)WiFi.channel());
  }

  // 9. BSSID
  uint8_t bssid[6] = {0};
  if (hasApInfo) {
    memcpy(bssid, apInfo.bssid, 6);
  } else {
    uint8_t* ptr = WiFi.BSSID();
    if (ptr != nullptr) memcpy(bssid, ptr, 6);
  }
  strncpy(wifiInfoRows[8].label, "BSSID", sizeof(wifiInfoRows[8].label) - 1);
  wifiInfoRows[8].label[sizeof(wifiInfoRows[8].label) - 1] = '\0';
  snprintf(wifiInfoRows[8].value, sizeof(wifiInfoRows[8].value), "%02X:%02X:%02X:%02X:%02X:%02X",
           bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);

  // 10. Manufacturer (OUI lookup from BSSID)
  strncpy(wifiInfoRows[9].label, "MANUFACTURER (OUI)", sizeof(wifiInfoRows[9].label) - 1);
  wifiInfoRows[9].label[sizeof(wifiInfoRows[9].label) - 1] = '\0';
  strncpy(wifiInfoRows[9].value, lookupOuiVendor(bssid), sizeof(wifiInfoRows[9].value) - 1);
  wifiInfoRows[9].value[sizeof(wifiInfoRows[9].value) - 1] = '\0';

  // 11. Auth mode
  strncpy(wifiInfoRows[10].label, "AUTH MODE", sizeof(wifiInfoRows[10].label) - 1);
  wifiInfoRows[10].label[sizeof(wifiInfoRows[10].label) - 1] = '\0';
  strncpy(wifiInfoRows[10].value, hasApInfo ? getWifiAuthModeStr(apInfo.authmode) : "UNKNOWN", sizeof(wifiInfoRows[10].value) - 1);
  wifiInfoRows[10].value[sizeof(wifiInfoRows[10].value) - 1] = '\0';

  // 12. WPS state
  strncpy(wifiInfoRows[11].label, "WPS STATE", sizeof(wifiInfoRows[11].label) - 1);
  wifiInfoRows[11].label[sizeof(wifiInfoRows[11].label) - 1] = '\0';
  strncpy(wifiInfoRows[11].value, (hasApInfo && apInfo.wps) ? "Supported / Enabled" : "Disabled / None", sizeof(wifiInfoRows[11].value) - 1);
  wifiInfoRows[11].value[sizeof(wifiInfoRows[11].value) - 1] = '\0';

  // 13. Unicast cipher
  strncpy(wifiInfoRows[12].label, "UNICAST CIPHER", sizeof(wifiInfoRows[12].label) - 1);
  wifiInfoRows[12].label[sizeof(wifiInfoRows[12].label) - 1] = '\0';
  strncpy(wifiInfoRows[12].value, hasApInfo ? getWifiCipherStr(apInfo.pairwise_cipher) : "UNKNOWN", sizeof(wifiInfoRows[12].value) - 1);
  wifiInfoRows[12].value[sizeof(wifiInfoRows[12].value) - 1] = '\0';

  // 14. Group cipher
  strncpy(wifiInfoRows[13].label, "GROUP CIPHER", sizeof(wifiInfoRows[13].label) - 1);
  wifiInfoRows[13].label[sizeof(wifiInfoRows[13].label) - 1] = '\0';
  strncpy(wifiInfoRows[13].value, hasApInfo ? getWifiCipherStr(apInfo.group_cipher) : "UNKNOWN", sizeof(wifiInfoRows[13].value) - 1);
  wifiInfoRows[13].value[sizeof(wifiInfoRows[13].value) - 1] = '\0';
}

void renderCurrentScreen();

// Background Wi-Fi connection monitoring and 1-minute auto-connect to saved networks
void updateWifiAutoAndConnect() {
  if (!settingWifiEnabled) {
    if (wifiBgScanning) {
      WiFi.scanDelete();
      wifiBgScanning = false;
    }
    wifiAnalyzerScanning = false;
    return;
  }
  uint32_t now = millis();

  // 1. Monitor active connection attempt from Wi-Fi connect screen
  if (currentScreen == SCREEN_WIFI_CONNECT && wifiView == WIFI_VIEW_CONNECTING) {
    wl_status_t st = WiFi.status();
    if (st == WL_CONNECTED) {
      wifiConnectSuccess = true;
      saveWifiCredentialLIFO(wifiSelectedSsid, wifiSelectedLocked ? wifiPasswordBuf : "");
      wifiView = WIFI_VIEW_RESULT;
      renderCurrentScreen();
      return;
    }
    if (st == WL_CONNECT_FAILED || st == WL_NO_SSID_AVAIL || (now - wifiConnectStartMs >= WIFI_CONNECT_TIMEOUT)) {
      WiFi.disconnect();
      wifiConnectSuccess = false;
      wifiView = WIFI_VIEW_RESULT;
      renderCurrentScreen();
      return;
    }
    return;
  }

  // 1b. Monitor active connection attempt from Wi-Fi list screen
  if (currentScreen == SCREEN_WIFI_LIST && wifiListView == WIFILIST_VIEW_CONNECTING) {
    wl_status_t st = WiFi.status();
    if (st == WL_CONNECTED) {
      wifiListConnectSuccess = true;
      wifiListView = WIFILIST_VIEW_RESULT;
      renderCurrentScreen();
      return;
    }
    if (st == WL_CONNECT_FAILED || st == WL_NO_SSID_AVAIL || (now - wifiListConnectStartMs >= WIFI_CONNECT_TIMEOUT)) {
      WiFi.disconnect();
      wifiListConnectSuccess = false;
      wifiListView = WIFILIST_VIEW_RESULT;
      renderCurrentScreen();
      return;
    }
    return;
  }

  // 1c. Asynchronous scan and occupancy calculation across 11 Wi-Fi channels in Wi-Fi analyzer
  if (currentScreen == SCREEN_WIFI_ANALYZER) {
    if (wifiAnalyzerScanning) {
      int16_t n = WiFi.scanComplete();
      if (n >= 0) {
        float rawLoad[WIFI_ANALYZER_CHANNELS];
        uint8_t apCounts[WIFI_ANALYZER_CHANNELS];
        for (uint8_t k = 0; k < WIFI_ANALYZER_CHANNELS; k++) {
          rawLoad[k] = 0.0f;
          apCounts[k] = 0;
        }
        for (int16_t i = 0; i < n; i++) {
          int32_t ch = WiFi.channel(i);
          if (ch < 1 || ch > WIFI_ANALYZER_CHANNELS) continue;
          if (apCounts[ch - 1] < 255) apCounts[ch - 1]++;

          int32_t rssi = WiFi.RSSI(i);
          if (rssi < -100) rssi = -100;
          if (rssi > -30)  rssi = -30;
          float sig = (float)(rssi + 100) / 70.0f;

          for (int8_t k = 0; k < WIFI_ANALYZER_CHANNELS; k++) {
            int32_t dist = abs((int32_t)(k + 1) - ch);
            if (dist == 0) {
              rawLoad[k] += 6.0f + sig * 32.0f;
            } else if (dist == 1) {
              rawLoad[k] += sig * 18.0f;
            } else if (dist == 2) {
              rawLoad[k] += sig * 8.0f;
            }
          }
        }
        for (uint8_t k = 0; k < WIFI_ANALYZER_CHANNELS; k++) {
          int32_t pct = (int32_t)lroundf(rawLoad[k]);
          if (pct < 0) pct = 0;
          if (pct > 100) pct = 100;
          wifiAnalyzerLoad[k] = (uint8_t)pct;
          wifiAnalyzerApCount[k] = apCounts[k];
        }
        WiFi.scanDelete();
        wifiAnalyzerScanning = false;
        wifiAnalyzerHasData = true;
        lastWifiAnalyzerScanMs = now;
        if (blState != BL_STATE_OFF) {
          renderCurrentScreen();
        }
      } else if (n == WIFI_SCAN_FAILED) {
        wifiAnalyzerScanning = false;
        lastWifiAnalyzerScanMs = now;
      }
    } else if (blState != BL_STATE_OFF && (now - lastWifiAnalyzerScanMs >= WIFI_ANALYZER_INTERVAL_MS)) {
      startWifiAnalyzerScan();
    }
    return;
  }

  // 1d. Channel hopping 1..11 and Deauth/Disassoc frames-per-second calculation for Wi-Fi jam detect
  if (currentScreen == SCREEN_WIFI_JAM && wifiJamActive) {
    uint32_t elapsed = now - wifiJamChanStartMs;
    if (elapsed >= WIFI_JAM_DWELL_MS) {
      uint32_t pkts = 0;
      portENTER_CRITICAL(&wifiJamMux);
      pkts = wifiJamCurrPkts;
      wifiJamCurrPkts = 0;
      portEXIT_CRITICAL(&wifiJamMux);

      uint32_t pps = (elapsed > 0) ? ((pkts * 1000UL + elapsed / 2UL) / elapsed) : 0;
      if (pps > 9999UL) pps = 9999UL;
      if (wifiJamCurrChan >= 1 && wifiJamCurrChan <= WIFI_JAM_CHANNELS) {
        wifiJamPps[wifiJamCurrChan - 1] = (uint16_t)pps;
      }

      wifiJamCurrChan++;
      if (wifiJamCurrChan > WIFI_JAM_CHANNELS) {
        wifiJamCurrChan = 1;
      }
      esp_wifi_set_channel(wifiJamCurrChan, WIFI_SECOND_CHAN_NONE);
      wifiJamChanStartMs = now;

      if (blState != BL_STATE_OFF) {
        renderCurrentScreen();
      }
    }
    return;
  }

  // 2. Complete background asynchronous scan and auto-connect in LIFO order
  if (wifiBgScanning) {
    if (!settingWifiAutoConnect) {
      WiFi.scanDelete();
      wifiBgScanning = false;
      return;
    }
    int16_t n = WiFi.scanComplete();
    if (n >= 0) {
      wifiBgScanning = false;
      if (WiFi.status() != WL_CONNECTED && settingWifiAutoConnect) {
        bool started = false;
        for (uint8_t s = 0; s < savedWifiCount && !started; s++) {
          for (int16_t i = 0; i < n; i++) {
            if (strcmp(savedWifiList[s].ssid, WiFi.SSID(i).c_str()) == 0) {
              if (savedWifiList[s].pass[0] != '\0') {
                WiFi.begin(savedWifiList[s].ssid, savedWifiList[s].pass);
              } else {
                WiFi.begin(savedWifiList[s].ssid);
              }
              started = true;
              break;
            }
          }
        }
      }
      WiFi.scanDelete();
    } else if (n == WIFI_SCAN_FAILED) {
      wifiBgScanning = false;
    }
    return;
  }

  // 3. Periodic 60-second check: if auto-connect enabled, Wi-Fi disconnected, and saved networks exist -> scan
  if (settingWifiAutoConnect && WiFi.status() != WL_CONNECTED && savedWifiCount > 0 &&
      !btScanPending && !btScanRunning &&
      currentScreen != SCREEN_WIFI_CONNECT &&
      currentScreen != SCREEN_WIFI_ANALYZER &&
      currentScreen != SCREEN_WIFI_JAM &&
      currentScreen != SCREEN_SOUND_SPEC &&
      currentScreen != SCREEN_ACCEL_SPEC &&
      currentScreen != SCREEN_SOUND_GEN &&
      currentScreen != SCREEN_HOSTS_SCAN &&
      currentScreen != SCREEN_BT_SCAN &&
      currentScreen != SCREEN_BT_KEYBOARD &&
      currentScreen != SCREEN_BT_MOUSE &&
      currentScreen != SCREEN_BT_PRESENTER &&
      currentScreen != SCREEN_SUN_ACTIVITY &&
      currentScreen != SCREEN_WEATHER &&
      (now - lastWifiAutoCheckMs >= WIFI_AUTO_CHECK_MS)) {
    lastWifiAutoCheckMs = now;
    WiFi.mode(WIFI_STA);
    WiFi.scanNetworks(true, false);
    wifiBgScanning = true;
  }
}

// Keyboard page badge (1..4) in top-right corner of Wi-Fi password screen
void drawTopRightKbPageIcon() {
  uint16_t borderCol = (wifiKbPage == 0) ? COLOR_ACCENT : ((wifiKbPage == 3) ? COLOR_GREEN : COLOR_WARN);
  canvas.fillRoundRect(204, 2, 32, 24, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(204, 2, 32, 24, 6, borderCol);

  char pageBuf[6];
  snprintf(pageBuf, sizeof(pageBuf), "%u/4", (unsigned)(wifiKbPage + 1));
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(borderCol, COLOR_HEX_FILL);
  canvas.drawString(pageBuf, 220, 14);
}

// Draw miniature padlock icon for encrypted Wi-Fi network
void drawWifiLockIcon(int16_t cx, int16_t cy, uint16_t color) {
  canvas.drawArc(cx, cy - 3, 4, 3, 180, 360, color);
  canvas.drawLine(cx - 3, cy - 3, cx - 3, cy, color);
  canvas.drawLine(cx + 3, cy - 3, cx + 3, cy, color);
  canvas.fillRoundRect(cx - 5, cy, 11, 8, 2, color);
  canvas.fillCircle(cx, cy + 4, 1, COLOR_HEX_FILL);
}

// Draw Wi-Fi connect screen (SCREEN_WIFI_CONNECT)
void drawWifiConnectScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  if (wifiView == WIFI_VIEW_SCANNING) {
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
    canvas.drawString("WI-FI CONNECT", 120, 96);
    canvas.setTextColor(COLOR_TEXT, COLOR_BG);
    canvas.drawString("Scanning networks...", 120, 130);
    return;
  }

  if (wifiView == WIFI_VIEW_LIST) {
    // Scroll-up button (top-right triangle below status bar)
    bool canUp = (wifiScrollOffset > 0);
    uint16_t upCol = canUp ? COLOR_ACCENT : COLOR_HEX_BORDER;
    canvas.fillRoundRect(200, 34, 34, 44, 6, COLOR_HEX_FILL);
    canvas.drawRoundRect(200, 34, 34, 44, 6, upCol);
    canvas.fillTriangle(217, 46, 207, 64, 227, 64, upCol);

    // Scroll-down button (bottom-right triangle)
    bool canDown = (wifiScrollOffset + WIFI_VISIBLE_ROWS < wifiScanCount);
    uint16_t downCol = canDown ? COLOR_ACCENT : COLOR_HEX_BORDER;
    canvas.fillRoundRect(200, 190, 34, 44, 6, COLOR_HEX_FILL);
    canvas.drawRoundRect(200, 190, 34, 44, 6, downCol);
    canvas.fillTriangle(217, 222, 207, 204, 227, 204, downCol);

    // Scroll indicator between triangles
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
    char idxBuf[12];
    if (wifiScanCount > 0) {
      snprintf(idxBuf, sizeof(idxBuf), "%u/%u", (unsigned)(wifiScrollOffset + 1), (unsigned)wifiScanCount);
    } else {
      snprintf(idxBuf, sizeof(idxBuf), "0/0");
    }
    canvas.drawString(idxBuf, 217, 134);

    if (wifiScanCount == 0) {
      canvas.setTextDatum(middle_center);
      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
      canvas.drawString("No Wi-Fi found", 100, 134);
      return;
    }

    String activeSsid = (WiFi.status() == WL_CONNECTED) ? WiFi.SSID() : "";

    for (uint8_t row = 0; row < WIFI_VISIBLE_ROWS; row++) {
      uint8_t idx = (uint8_t)(wifiScrollOffset + row);
      if (idx >= wifiScanCount) break;

      int16_t ry = (int16_t)(34 + row * 40);
      bool isCurr = (activeSsid.length() > 0 && strcmp(wifiScanList[idx].ssid, activeSsid.c_str()) == 0);
      uint16_t fillCol = isCurr ? COLOR_HEX_ACTIVE : COLOR_HEX_FILL;
      uint16_t bordCol = isCurr ? COLOR_GREEN : COLOR_ACCENT;

      canvas.fillRoundRect(6, ry, 188, 36, 5, fillCol);
      canvas.drawRoundRect(6, ry, 188, 36, 5, bordCol);

      // Padlock icon on the right for encrypted networks
      if (wifiScanList[idx].isLocked) {
        drawWifiLockIcon(180, ry + 18, COLOR_WARN);
      }

      // Network SSID (truncated to 18 chars to fit cleanly)
      char shortSsid[20];
      strncpy(shortSsid, wifiScanList[idx].ssid, 18);
      shortSsid[18] = '\0';

      canvas.setTextDatum(middle_left);
      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(isCurr ? COLOR_GREEN : COLOR_TEXT, fillCol);
      canvas.drawString(shortSsid, 12, ry + 18);
    }
    return;
  }

  if (wifiView == WIFI_VIEW_PASSWORD) {
    drawTopRightKbPageIcon();

    // SSID header and password entry box (calculator display style)
    canvas.fillRoundRect(6, 31, 228, 34, 5, COLOR_TOPBAR);
    canvas.drawRoundRect(6, 31, 228, 34, 5, COLOR_HEX_BORDER);

    char ssidCap[24];
    strncpy(ssidCap, wifiSelectedSsid, 22);
    ssidCap[22] = '\0';
    canvas.setTextDatum(top_left);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_ACCENT, COLOR_TOPBAR);
    canvas.drawString(ssidCap, 11, 34);

    // Show trailing characters if password exceeds 22 chars
    size_t passLen = strlen(wifiPasswordBuf);
    const char* dispPass = (passLen > 22) ? (wifiPasswordBuf + (passLen - 22)) : wifiPasswordBuf;
    canvas.setTextDatum(bottom_left);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_TEXT, COLOR_TOPBAR);
    canvas.drawString((passLen > 0) ? dispPass : "Password...", 11, 63);

    // On-screen keyboard (3 character rows + bottom Shift / <- / Enter row)
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);

    if (wifiKbPage == 0 || wifiKbPage == 3) {
      const char* keys = (wifiKbPage == 0)
                           ? (wifiKbShift ? WIFI_KB_P0_HI : WIFI_KB_P0_LO)
                           : (wifiKbShift ? WIFI_KB_P3_HI : WIFI_KB_P3_LO);
      for (uint8_t i = 0; i < 12; i++) {
        int16_t bx = (int16_t)(6 + (i % 4) * 57);
        int16_t by = (int16_t)(69 + (i / 4) * 42);
        canvas.fillRoundRect(bx, by, 54, 38, 5, COLOR_HEX_FILL);
        canvas.drawRoundRect(bx, by, 54, 38, 5, COLOR_HEX_BORDER);
        char chBuf[2] = {keys[i], '\0'};
        canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
        canvas.drawString(chBuf, bx + 27, by + 19);
      }
    } else if (wifiKbPage == 1) {
      const char* keys = wifiKbShift ? WIFI_KB_P1_HI : WIFI_KB_P1_LO;
      for (uint8_t i = 0; i < 8; i++) {
        int16_t bx = (int16_t)(6 + (i % 4) * 57);
        int16_t by = (int16_t)(69 + (i / 4) * 42);
        canvas.fillRoundRect(bx, by, 54, 38, 5, COLOR_HEX_FILL);
        canvas.drawRoundRect(bx, by, 54, 38, 5, COLOR_HEX_BORDER);
        char chBuf[2] = {keys[i], '\0'};
        canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
        canvas.drawString(chBuf, bx + 27, by + 19);
      }
      // Row 3: b n m (3 wide keys, 73 px each)
      for (uint8_t c = 0; c < 3; c++) {
        int16_t bx = (int16_t)(6 + c * 77);
        int16_t by = 153;
        canvas.fillRoundRect(bx, by, 74, 38, 5, COLOR_HEX_FILL);
        canvas.drawRoundRect(bx, by, 74, 38, 5, COLOR_HEX_BORDER);
        char chBuf[2] = {keys[8 + c], '\0'};
        canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
        canvas.drawString(chBuf, bx + 37, by + 19);
      }
    } else {
      // Page 3 (wifiKbPage == 2): o p [ ] / l ; ' , / . /
      const char* keys = wifiKbShift ? WIFI_KB_P2_HI : WIFI_KB_P2_LO;
      for (uint8_t i = 0; i < 8; i++) {
        int16_t bx = (int16_t)(6 + (i % 4) * 57);
        int16_t by = (int16_t)(69 + (i / 4) * 42);
        canvas.fillRoundRect(bx, by, 54, 38, 5, COLOR_HEX_FILL);
        canvas.drawRoundRect(bx, by, 54, 38, 5, COLOR_HEX_BORDER);
        char chBuf[2] = {keys[i], '\0'};
        canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
        canvas.drawString(chBuf, bx + 27, by + 19);
      }
      // Row 3: . and / (2 wide keys, 111 px each)
      for (uint8_t c = 0; c < 2; c++) {
        int16_t bx = (int16_t)(6 + c * 117);
        int16_t by = 153;
        canvas.fillRoundRect(bx, by, 111, 38, 5, COLOR_HEX_FILL);
        canvas.drawRoundRect(bx, by, 111, 38, 5, COLOR_HEX_BORDER);
        char chBuf[2] = {keys[8 + c], '\0'};
        canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
        canvas.drawString(chBuf, bx + 55, by + 19);
      }
    }

    // Bottom control row (y = 195): Shift | <- | Enter
    uint16_t shiftFill = wifiKbShift ? COLOR_HEX_ACTIVE : COLOR_HEX_FILL;
    uint16_t shiftBord = wifiKbShift ? COLOR_WARN : COLOR_ACCENT;
    canvas.fillRoundRect(6, 195, 54, 38, 5, shiftFill);
    canvas.drawRoundRect(6, 195, 54, 38, 5, shiftBord);
    canvas.setTextColor(wifiKbShift ? COLOR_WARN : COLOR_TEXT, shiftFill);
    canvas.drawString(wifiKbShift ? "ABC" : "abc", 33, 214);

    canvas.fillRoundRect(63, 195, 54, 38, 5, COLOR_HEX_FILL);
    canvas.drawRoundRect(63, 195, 54, 38, 5, COLOR_RED);
    canvas.setTextColor(COLOR_RED, COLOR_HEX_FILL);
    canvas.drawString("<-", 90, 214);

    canvas.fillRoundRect(120, 195, 114, 38, 5, COLOR_HEX_ACTIVE);
    canvas.drawRoundRect(120, 195, 114, 38, 5, COLOR_GREEN);
    canvas.setTextColor(COLOR_GREEN, COLOR_HEX_ACTIVE);
    canvas.drawString("Enter", 177, 214);
    return;
  }

  if (wifiView == WIFI_VIEW_CONNECTING) {
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
    canvas.drawString("Connecting to:", 120, 94);
    canvas.setTextColor(COLOR_TEXT, COLOR_BG);
    canvas.drawString(wifiSelectedSsid, 120, 124);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
    canvas.drawString("Please wait...", 120, 154);
    return;
  }

  // WIFI_VIEW_RESULT: success/failure status message and OK button
  canvas.setTextDatum(middle_center);
  if (wifiConnectSuccess) {
    canvas.drawCircle(120, 74, 22, COLOR_GREEN);
    canvas.drawLine(110, 74, 117, 82, COLOR_GREEN);
    canvas.drawLine(111, 74, 117, 81, COLOR_GREEN);
    canvas.drawLine(117, 82, 131, 66, COLOR_GREEN);
    canvas.drawLine(117, 81, 130, 66, COLOR_GREEN);

    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_GREEN, COLOR_BG);
    canvas.drawString("Connected!", 120, 114);

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_TEXT, COLOR_BG);
    canvas.drawString(wifiSelectedSsid, 120, 136);

    String ipStr = WiFi.localIP().toString();
    canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
    canvas.drawString(ipStr.c_str(), 120, 152);
  } else {
    canvas.drawCircle(120, 74, 22, COLOR_RED);
    canvas.drawLine(111, 65, 129, 83, COLOR_RED);
    canvas.drawLine(112, 65, 129, 82, COLOR_RED);
    canvas.drawLine(129, 65, 111, 83, COLOR_RED);
    canvas.drawLine(128, 65, 111, 82, COLOR_RED);

    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_RED, COLOR_BG);
    canvas.drawString("Connection Error", 120, 114);

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
    canvas.drawString("Wrong password or timeout", 120, 138);
  }

  // OK button -> return to main menu
  canvas.fillRoundRect(65, 178, 110, 42, 8, COLOR_HEX_ACTIVE);
  canvas.drawRoundRect(65, 178, 110, 42, 8, wifiConnectSuccess ? COLOR_GREEN : COLOR_ACCENT);
  canvas.setFont(&fonts::Font4);
  canvas.setTextColor(COLOR_TEXT, COLOR_HEX_ACTIVE);
  canvas.drawString("OK", 120, 200);
}

// Draw Wi-Fi list screen (saved networks list and view/connect/delete dialog)
void drawWifiListScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon(); // Top-left X icon -> return to main menu

  if (wifiListView == WIFILIST_VIEW_LIST) {
    if (savedWifiCount == 0) {
      canvas.setTextDatum(middle_center);
      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
      canvas.drawString("No saved networks", 120, 122);
      canvas.setFont(&fonts::Font0);
      canvas.drawString("Connect via Wi-Fi connect", 120, 146);
      return;
    }

    // List scroll triangle buttons on top-right and bottom-right
    bool canUp   = (wifiListScrollOffset > 0);
    bool canDown = ((uint16_t)wifiListScrollOffset + WIFI_VISIBLE_ROWS < savedWifiCount);

    canvas.fillRoundRect(199, 34, 36, 56, 6, COLOR_HEX_FILL);
    canvas.drawRoundRect(199, 34, 36, 56, 6, canUp ? COLOR_ACCENT : COLOR_HEX_BORDER);
    canvas.fillTriangle(217, 52, 208, 70, 226, 70, canUp ? COLOR_ACCENT : COLOR_SUBTEXT);

    canvas.fillRoundRect(199, 182, 36, 56, 6, COLOR_HEX_FILL);
    canvas.drawRoundRect(199, 182, 36, 56, 6, canDown ? COLOR_ACCENT : COLOR_HEX_BORDER);
    canvas.fillTriangle(217, 220, 208, 202, 226, 202, canDown ? COLOR_ACCENT : COLOR_SUBTEXT);

    char posBuf[12];
    snprintf(posBuf, sizeof(posBuf), "%u/%u", (unsigned)(wifiListScrollOffset + 1), (unsigned)savedWifiCount);
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
    canvas.drawString(posBuf, 217, 136);

    bool connected = (WiFi.status() == WL_CONNECTED);
    String currSsid = connected ? WiFi.SSID() : "";

    for (uint8_t r = 0; r < WIFI_VISIBLE_ROWS; r++) {
      uint8_t idx = (uint8_t)(wifiListScrollOffset + r);
      if (idx >= savedWifiCount) break;

      int16_t ry = (int16_t)(34 + r * 41);
      bool isCurr = connected && (strcmp(savedWifiList[idx].ssid, currSsid.c_str()) == 0);
      uint16_t fillCol = isCurr ? COLOR_HEX_ACTIVE : COLOR_HEX_FILL;
      uint16_t bordCol = isCurr ? COLOR_GREEN : COLOR_ACCENT;

      canvas.fillRoundRect(6, ry, 188, 37, 6, fillCol);
      canvas.drawRoundRect(6, ry, 188, 37, 6, bordCol);

      char shortSsid[20];
      strncpy(shortSsid, savedWifiList[idx].ssid, 18);
      shortSsid[18] = '\0';

      canvas.setTextDatum(middle_left);
      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(isCurr ? COLOR_GREEN : COLOR_TEXT, fillCol);
      canvas.drawString(shortSsid, 12, ry + 18);
    }
    return;
  }

  if (wifiListView == WIFILIST_VIEW_CONNECTING) {
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
    canvas.drawString("Connecting...", 120, 102);

    char shortSsid[23];
    strncpy(shortSsid, wifiListConnectSsid, 22);
    shortSsid[22] = '\0';
    canvas.setTextColor(COLOR_TEXT, COLOR_BG);
    canvas.drawString(shortSsid, 120, 130);

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
    canvas.drawString("Please wait", 120, 156);
    return;
  }

  if (wifiListView == WIFILIST_VIEW_RESULT) {
    canvas.setTextDatum(middle_center);
    if (wifiListConnectSuccess) {
      canvas.drawCircle(120, 74, 22, COLOR_GREEN);
      canvas.drawLine(109, 74, 117, 83, COLOR_GREEN);
      canvas.drawLine(109, 75, 117, 84, COLOR_GREEN);
      canvas.drawLine(117, 83, 132, 65, COLOR_GREEN);
      canvas.drawLine(117, 84, 132, 66, COLOR_GREEN);

      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(COLOR_GREEN, COLOR_BG);
      canvas.drawString("Connected!", 120, 114);

      char shortSsid[23];
      strncpy(shortSsid, wifiListConnectSsid, 22);
      shortSsid[22] = '\0';
      canvas.setTextColor(COLOR_TEXT, COLOR_BG);
      canvas.drawString(shortSsid, 120, 136);

      String ipStr = WiFi.localIP().toString();
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
      canvas.drawString(ipStr.c_str(), 120, 156);
    } else {
      canvas.drawCircle(120, 74, 22, COLOR_RED);
      canvas.drawLine(111, 65, 129, 83, COLOR_RED);
      canvas.drawLine(112, 65, 129, 82, COLOR_RED);
      canvas.drawLine(129, 65, 111, 83, COLOR_RED);
      canvas.drawLine(128, 65, 111, 82, COLOR_RED);

      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(COLOR_RED, COLOR_BG);
      canvas.drawString("Connection error", 120, 114);

      char shortSsid[23];
      strncpy(shortSsid, wifiListConnectSsid, 22);
      shortSsid[22] = '\0';
      canvas.setTextColor(COLOR_TEXT, COLOR_BG);
      canvas.drawString(shortSsid, 120, 136);

      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
      canvas.drawString("Network unavailable", 120, 156);
    }

    // OK button -> return to main menu
    canvas.fillRoundRect(65, 178, 110, 42, 8, COLOR_HEX_ACTIVE);
    canvas.drawRoundRect(65, 178, 110, 42, 8, wifiListConnectSuccess ? COLOR_GREEN : COLOR_ACCENT);
    canvas.setFont(&fonts::Font4);
    canvas.setTextColor(COLOR_TEXT, COLOR_HEX_ACTIVE);
    canvas.drawString("OK", 120, 200);
    return;
  }

  // Dialog showing SSID, password, and OK / Connect / Delete buttons
  if (wifiListSelectedIdx < 0 || wifiListSelectedIdx >= savedWifiCount) {
    wifiListView = WIFILIST_VIEW_LIST;
    return;
  }

  const SavedWifiCred& cred = savedWifiList[wifiListSelectedIdx];

  canvas.fillRoundRect(10, 38, 220, 124, 8, COLOR_TOPBAR);
  canvas.drawRoundRect(10, 38, 220, 124, 8, COLOR_ACCENT);

  canvas.setTextDatum(top_left);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
  canvas.drawString("NETWORK:", 18, 46);

  char ssidLine[23];
  strncpy(ssidLine, cred.ssid, 22);
  ssidLine[22] = '\0';
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_ACCENT, COLOR_TOPBAR);
  canvas.drawString(ssidLine, 18, 60);

  canvas.drawFastHLine(18, 86, 204, COLOR_HEX_BORDER);

  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
  canvas.drawString("PASSWORD:", 18, 94);

  if (cred.pass[0] == '\0') {
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
    canvas.drawString("(Open network)", 18, 108);
  } else {
    char passL1[23];
    strncpy(passL1, cred.pass, 22);
    passL1[22] = '\0';
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_TEXT, COLOR_TOPBAR);
    canvas.drawString(passL1, 18, 108);

    if (strlen(cred.pass) > 22) {
      char passL2[23];
      strncpy(passL2, cred.pass + 22, 22);
      passL2[22] = '\0';
      canvas.drawString(passL2, 18, 130);
    }
  }

  // Three buttons: OK (left), Connect (center), and Delete (right)
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font2);

  canvas.fillRoundRect(8, 176, 66, 46, 8, COLOR_HEX_ACTIVE);
  canvas.drawRoundRect(8, 176, 66, 46, 8, COLOR_GREEN);
  canvas.setTextColor(COLOR_TEXT, COLOR_HEX_ACTIVE);
  canvas.drawString("OK", 41, 199);

  canvas.fillRoundRect(80, 176, 80, 46, 8, COLOR_HEX_ACTIVE);
  canvas.drawRoundRect(80, 176, 80, 46, 8, COLOR_ACCENT);
  canvas.setTextColor(COLOR_ACCENT, COLOR_HEX_ACTIVE);
  canvas.drawString("Connect", 120, 199);

  canvas.fillRoundRect(166, 176, 66, 46, 8, COLOR_HEX_FILL);
  canvas.drawRoundRect(166, 176, 66, 46, 8, COLOR_RED);
  canvas.setTextColor(COLOR_RED, COLOR_HEX_FILL);
  canvas.drawString("Delete", 199, 199);
}

// Draw Settings screen (options list and editing sub-screens)
void drawSettingsScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  if (settingsView == SETTINGS_VIEW_LIST) {
    // Settings list (8 items) with vertical scrolling via right-side triangle buttons
    const char* itemTitles[SETTINGS_ITEM_COUNT] = {
      "Wi-Fi power",
      "Display light intensity",
      "Display light time",
      "Touch vibration",
      "Touch sensitivity",
      "Charge current",
      "Power settings",
      "Wake-up condition"
    };
    char valBufs[SETTINGS_ITEM_COUNT][24];
    if (!settingWifiEnabled) {
      snprintf(valBufs[0], sizeof(valBufs[0]), "OFF (Disabled)");
    } else {
      snprintf(valBufs[0], sizeof(valBufs[0]), "ON / Auto:%s", settingWifiAutoConnect ? "ON" : "OFF");
    }
    snprintf(valBufs[1], sizeof(valBufs[1]), "%u%%", (unsigned)settingDisplayBrightness);
    if (settingDisplayLightSec == 0) {
      snprintf(valBufs[2], sizeof(valBufs[2]), "0s (ON)");
    } else {
      snprintf(valBufs[2], sizeof(valBufs[2]), "%u s", (unsigned)settingDisplayLightSec);
    }
    snprintf(valBufs[3], sizeof(valBufs[3]), "%s", settingTouchVibration ? "ON" : "OFF");
    snprintf(valBufs[4], sizeof(valBufs[4]), "0x80:%u / %ux tap",
             (unsigned)settingTouchSensitivity, (unsigned)settingWakeTapCount);
    snprintf(valBufs[5], sizeof(valBufs[5]), "%u mA (AXP2101)", (unsigned)settingChargeCurrentMa);
    snprintf(valBufs[6], sizeof(valBufs[6]), "GPS:%s LoRa:%s",
             settingPowerGps ? "ON" : "OFF", settingPowerLora ? "ON" : "OFF");
    snprintf(valBufs[7], sizeof(valBufs[7]), "TCH:%s BMA:%s BTN:%s",
             settingWakeOnTouch ? "ON" : "OFF",
             settingWakeOnBma423 ? "ON" : "OFF",
             settingWakeOnButton ? "ON" : "OFF");

    uint8_t maxOffset = (SETTINGS_ITEM_COUNT > SETTINGS_VISIBLE_ROWS)
                          ? (uint8_t)(SETTINGS_ITEM_COUNT - SETTINGS_VISIBLE_ROWS)
                          : 0;
    if (settingsScrollOffset > maxOffset) settingsScrollOffset = maxOffset;

    for (uint8_t i = 0; i < SETTINGS_VISIBLE_ROWS; i++) {
      uint8_t itemIdx = (uint8_t)(settingsScrollOffset + i);
      if (itemIdx >= SETTINGS_ITEM_COUNT) break;
      int16_t ry = (int16_t)(36 + i * 50);
      canvas.fillRoundRect(6, ry, 188, 44, 8, COLOR_HEX_FILL);
      canvas.drawRoundRect(6, ry, 188, 44, 8, COLOR_ACCENT);

      canvas.setTextDatum(top_left);
      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
      canvas.drawString(itemTitles[itemIdx], 12, ry + 6);

      uint16_t valCol = COLOR_ACCENT;
      if (itemIdx == 0) valCol = settingWifiEnabled ? COLOR_GREEN : COLOR_SUBTEXT;
      else if (itemIdx == 2 && settingDisplayLightSec == 0) valCol = COLOR_GREEN;
      else if (itemIdx == 3) valCol = settingTouchVibration ? COLOR_GREEN : COLOR_SUBTEXT;
      else if (itemIdx == 4) valCol = COLOR_ACCENT;
      else if (itemIdx == 5) valCol = COLOR_WARN;
      else if (itemIdx == 6) valCol = (settingPowerGps || settingPowerLora) ? COLOR_GREEN : COLOR_SUBTEXT;
      else if (itemIdx == 7) valCol = (settingWakeOnTouch || settingWakeOnBma423 || settingWakeOnButton) ? COLOR_GREEN : COLOR_RED;

      canvas.setTextDatum(bottom_left);
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(valCol, COLOR_HEX_FILL);
      canvas.drawString(valBufs[itemIdx], 12, ry + 38);

      // Small right arrow indicating submenu navigation
      canvas.fillTriangle(187, ry + 22, 180, ry + 17, 180, ry + 27, COLOR_SUBTEXT);
    }

    // Vertical scroll buttons with triangles on top-right and bottom-right
    bool canUp   = (settingsScrollOffset > 0);
    bool canDown = (settingsScrollOffset < maxOffset);

    canvas.fillRoundRect(200, 36, 34, 44, 7, COLOR_HEX_FILL);
    canvas.drawRoundRect(200, 36, 34, 44, 7, canUp ? COLOR_ACCENT : COLOR_HEX_BORDER);
    canvas.fillTriangle(217, 48, 208, 66, 226, 66, canUp ? COLOR_ACCENT : COLOR_SUBTEXT);

    char pageBuf[10];
    snprintf(pageBuf, sizeof(pageBuf), "%u-%u/%u",
             (unsigned)(settingsScrollOffset + 1),
             (unsigned)(settingsScrollOffset + SETTINGS_VISIBLE_ROWS),
             (unsigned)SETTINGS_ITEM_COUNT);
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
    canvas.drawString(pageBuf, 217, 133);

    canvas.fillRoundRect(200, 186, 34, 44, 7, COLOR_HEX_FILL);
    canvas.drawRoundRect(200, 186, 34, 44, 7, canDown ? COLOR_ACCENT : COLOR_HEX_BORDER);
    canvas.fillTriangle(217, 218, 208, 200, 226, 200, canDown ? COLOR_ACCENT : COLOR_SUBTEXT);
    return;
  }

  if (settingsView == SETTINGS_VIEW_WIFI_AUTO) {
    // Wi-Fi power settings: global "Wi-Fi enabled" + "Wi-Fi autoconnect" toggles + OK / Cancel
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_TEXT, COLOR_BG);
    canvas.drawString("Wi-Fi power", 120, 46);

    // Row 1: Wi-Fi enabled (global radio power)
    int16_t r1Y = 64;
    canvas.fillRoundRect(12, r1Y, 216, 44, 8, COLOR_HEX_FILL);
    canvas.drawRoundRect(12, r1Y, 216, 44, 8, editWifiEnabled ? COLOR_GREEN : COLOR_HEX_BORDER);
    canvas.setTextDatum(top_left);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
    canvas.drawString("Wi-Fi enabled", 20, r1Y + 7);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(editWifiEnabled ? COLOR_GREEN : COLOR_SUBTEXT, COLOR_HEX_FILL);
    canvas.drawString(editWifiEnabled ? "Global radio ON" : "Global radio OFF", 20, r1Y + 29);

    int16_t sw1X = 166, sw1Y = r1Y + 10;
    canvas.fillRoundRect(sw1X, sw1Y, 52, 24, 12, editWifiEnabled ? COLOR_HEX_ACTIVE : COLOR_TOPBAR);
    canvas.drawRoundRect(sw1X, sw1Y, 52, 24, 12, editWifiEnabled ? COLOR_GREEN : COLOR_SUBTEXT);
    canvas.fillCircle(editWifiEnabled ? (sw1X + 40) : (sw1X + 12), sw1Y + 12, 9,
                      editWifiEnabled ? COLOR_GREEN : COLOR_SUBTEXT);

    // Row 2: Wi-Fi autoconnect
    int16_t r2Y = 116;
    bool autoEffOn = editWifiEnabled && editWifiAutoConnect;
    canvas.fillRoundRect(12, r2Y, 216, 44, 8, COLOR_HEX_FILL);
    canvas.drawRoundRect(12, r2Y, 216, 44, 8, autoEffOn ? COLOR_GREEN : COLOR_HEX_BORDER);
    canvas.setTextDatum(top_left);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(editWifiEnabled ? COLOR_TEXT : COLOR_SUBTEXT, COLOR_HEX_FILL);
    canvas.drawString("Wi-Fi autoconnect", 20, r2Y + 7);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(autoEffOn ? COLOR_GREEN : COLOR_SUBTEXT, COLOR_HEX_FILL);
    canvas.drawString(editWifiAutoConnect ? "Auto-join ON" : "Auto-join OFF", 20, r2Y + 29);

    int16_t sw2X = 166, sw2Y = r2Y + 10;
    canvas.fillRoundRect(sw2X, sw2Y, 52, 24, 12, autoEffOn ? COLOR_HEX_ACTIVE : COLOR_TOPBAR);
    canvas.drawRoundRect(sw2X, sw2Y, 52, 24, 12, autoEffOn ? COLOR_GREEN : COLOR_SUBTEXT);
    canvas.fillCircle(editWifiAutoConnect ? (sw2X + 40) : (sw2X + 12), sw2Y + 12, 9,
                      autoEffOn ? COLOR_GREEN : COLOR_SUBTEXT);
  }
  else if (settingsView == SETTINGS_VIEW_BRIGHTNESS) {
    // Backlight intensity slider (5-100%, min 5%) + OK / Cancel
    uint8_t shownBri = editDisplayBrightness;
    if (shownBri < BRIGHTNESS_MIN_PCT) shownBri = BRIGHTNESS_MIN_PCT;
    if (shownBri > 100) shownBri = 100;

    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_TEXT, COLOR_BG);
    canvas.drawString("Display light intensity", 120, 48);

    char briBuf[12];
    snprintf(briBuf, sizeof(briBuf), "%u%%", (unsigned)shownBri);
    canvas.setFont(&fonts::Font4);
    canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
    canvas.drawString(briBuf, 120, 84);

    // Slider track (x = 22..218, width 196 px, range 5..100%)
    int16_t trackX = 22, trackY = 122, trackW = 196, trackH = 14;
    canvas.fillRoundRect(trackX, trackY, trackW, trackH, 7, COLOR_TOPBAR);
    canvas.drawRoundRect(trackX, trackY, trackW, trackH, 7, COLOR_HEX_BORDER);

    int16_t fillW = (int16_t)(((uint32_t)(shownBri - BRIGHTNESS_MIN_PCT) * (uint32_t)trackW) / (100U - BRIGHTNESS_MIN_PCT));
    if (fillW > 2) {
      canvas.fillRoundRect(trackX, trackY, fillW, trackH, 7, COLOR_ACCENT);
    }

    int16_t knobX = (int16_t)(trackX + fillW);
    canvas.fillCircle(knobX, trackY + trackH / 2, 12, COLOR_TEXT);
    canvas.drawCircle(knobX, trackY + trackH / 2, 12, COLOR_ACCENT);
    canvas.fillCircle(knobX, trackY + trackH / 2, 5, COLOR_ACCENT);

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
    canvas.setTextDatum(top_left);
    canvas.drawString("5%", 20, 148);
    canvas.setTextDatum(top_right);
    canvas.drawString("100%", 220, 148);
  }
  else if (settingsView == SETTINGS_VIEW_LIGHT_TIME) {
    // Numeric indicator and left/right arrows (0,5,10,20,30,45,60,90,180 s) + OK / Cancel
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_TEXT, COLOR_BG);
    canvas.drawString("Display light time", 120, 48);

    bool canLeft  = (editDisplayTimeIdx > 0);
    bool canRight = (editDisplayTimeIdx + 1 < DISPLAY_TIME_OPT_COUNT);

    // Left arrow
    canvas.fillRoundRect(12, 84, 48, 52, 8, COLOR_HEX_FILL);
    canvas.drawRoundRect(12, 84, 48, 52, 8, canLeft ? COLOR_ACCENT : COLOR_HEX_BORDER);
    canvas.fillTriangle(26, 110, 44, 98, 44, 122, canLeft ? COLOR_ACCENT : COLOR_SUBTEXT);

    // Right arrow
    canvas.fillRoundRect(180, 84, 48, 52, 8, COLOR_HEX_FILL);
    canvas.drawRoundRect(180, 84, 48, 52, 8, canRight ? COLOR_ACCENT : COLOR_HEX_BORDER);
    canvas.fillTriangle(214, 110, 196, 98, 196, 122, canRight ? COLOR_ACCENT : COLOR_SUBTEXT);

    // Center numeric readout
    uint16_t secVal = DISPLAY_TIME_OPTIONS[editDisplayTimeIdx];
    canvas.fillRoundRect(68, 78, 104, 64, 8, COLOR_TOPBAR);
    canvas.drawRoundRect(68, 78, 104, 64, 8, (secVal == 0) ? COLOR_GREEN : COLOR_ACCENT);

    char secBuf[16];
    snprintf(secBuf, sizeof(secBuf), "%u s", (unsigned)secVal);
    canvas.setFont(&fonts::Font4);
    canvas.setTextColor((secVal == 0) ? COLOR_GREEN : COLOR_TEXT, COLOR_TOPBAR);
    canvas.drawString(secBuf, 120, (secVal == 0) ? 104 : 110);

    if (secVal == 0) {
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(COLOR_GREEN, COLOR_TOPBAR);
      canvas.drawString("NEVER OFF", 120, 129);
    }
  }
  else if (settingsView == SETTINGS_VIEW_TOUCH_VIB) {
    // Touch haptic feedback toggle + OK / Cancel
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_TEXT, COLOR_BG);
    canvas.drawString("Touch vibration", 120, 52);

    int16_t swX = 72, swY = 82, swW = 96, swH = 44;
    uint16_t trackFill = editTouchVibration ? COLOR_HEX_ACTIVE : COLOR_TOPBAR;
    uint16_t trackBord = editTouchVibration ? COLOR_GREEN : COLOR_SUBTEXT;
    canvas.fillRoundRect(swX, swY, swW, swH, 22, trackFill);
    canvas.drawRoundRect(swX, swY, swW, swH, 22, trackBord);

    int16_t knobX = editTouchVibration ? (swX + swW - 22) : (swX + 22);
    canvas.fillCircle(knobX, swY + 22, 16, editTouchVibration ? COLOR_GREEN : COLOR_SUBTEXT);
    canvas.drawCircle(knobX, swY + 22, 16, COLOR_TEXT);

    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(editTouchVibration ? COLOR_GREEN : COLOR_SUBTEXT, COLOR_BG);
    canvas.drawString(editTouchVibration ? "ENABLED" : "DISABLED", 120, 148);
  }
  else if (settingsView == SETTINGS_VIEW_TOUCH_SENS) {
    // Touchscreen sensitivity (FT6236 reg 0x80: 30..80, default 50) + wake gesture (2x / 3x tap)
    uint8_t shownSens = editTouchSensitivity;
    if (shownSens < TOUCH_SENS_MIN_VAL) shownSens = TOUCH_SENS_MIN_VAL;
    if (shownSens > TOUCH_SENS_MAX_VAL) shownSens = TOUCH_SENS_MAX_VAL;
    uint8_t thVal = touchSensitivityToThGroup(shownSens);

    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_TEXT, COLOR_BG);
    canvas.drawString("Touch sensitivity", 120, 40);

    char sensBuf[28];
    snprintf(sensBuf, sizeof(sensBuf), "Reg 0x80: %u (def 50)", (unsigned)thVal);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
    canvas.drawString(sensBuf, 120, 58);

    // Touch sensitivity slider (x = 22..218, width 196 px, scale 30..80)
    int16_t trackX = 22, trackY = 74, trackW = 196, trackH = 14;
    canvas.fillRoundRect(trackX, trackY, trackW, trackH, 7, COLOR_TOPBAR);
    canvas.drawRoundRect(trackX, trackY, trackW, trackH, 7, COLOR_HEX_BORDER);

    int16_t fillW = (int16_t)(((uint32_t)(shownSens - TOUCH_SENS_MIN_VAL) * (uint32_t)trackW) / (TOUCH_SENS_MAX_VAL - TOUCH_SENS_MIN_VAL));
    if (fillW > 2) {
      canvas.fillRoundRect(trackX, trackY, fillW, trackH, 7, COLOR_ACCENT);
    }

    int16_t knobX = (int16_t)(trackX + fillW);
    canvas.fillCircle(knobX, trackY + trackH / 2, 11, COLOR_TEXT);
    canvas.drawCircle(knobX, trackY + trackH / 2, 11, COLOR_ACCENT);
    canvas.fillCircle(knobX, trackY + trackH / 2, 4, COLOR_ACCENT);

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
    canvas.setTextDatum(top_left);
    canvas.drawString("30", 20, 95);
    canvas.setTextDatum(top_right);
    canvas.drawString("80", 220, 95);

    // Wake screen gesture selector: double or triple tap
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
    canvas.drawString("Wake screen gesture:", 120, 117);

    bool isTriple = (editWakeTapCount == 3);
    canvas.fillRoundRect(16, 128, 98, 34, 7, !isTriple ? COLOR_HEX_ACTIVE : COLOR_HEX_FILL);
    canvas.drawRoundRect(16, 128, 98, 34, 7, !isTriple ? COLOR_GREEN : COLOR_HEX_BORDER);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(!isTriple ? COLOR_GREEN : COLOR_SUBTEXT, !isTriple ? COLOR_HEX_ACTIVE : COLOR_HEX_FILL);
    canvas.drawString("2x TAP", 65, 145);

    canvas.fillRoundRect(126, 128, 98, 34, 7, isTriple ? COLOR_HEX_ACTIVE : COLOR_HEX_FILL);
    canvas.drawRoundRect(126, 128, 98, 34, 7, isTriple ? COLOR_GREEN : COLOR_HEX_BORDER);
    canvas.setTextColor(isTriple ? COLOR_GREEN : COLOR_SUBTEXT, isTriple ? COLOR_HEX_ACTIVE : COLOR_HEX_FILL);
    canvas.drawString("3x TAP", 175, 145);
  }
  else if (settingsView == SETTINGS_VIEW_CHARGE_CURR) {
    // AXP2101 charge current selector: 100 mA, 200 mA, 300 mA, 400 mA + OK / Cancel
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_TEXT, COLOR_BG);
    canvas.drawString("Charge current", 120, 44);

    char regBuf[28];
    snprintf(regBuf, sizeof(regBuf), "AXP2101 Reg 0x62: 0x%02X", (unsigned)chargeCurrentToAxpReg(editChargeCurrentMa));
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
    canvas.drawString(regBuf, 120, 62);

    for (uint8_t i = 0; i < CHARGE_CURRENT_OPT_COUNT; i++) {
      uint16_t ma = CHARGE_CURRENT_OPTIONS[i];
      bool sel = (editChargeCurrentMa == ma);
      int16_t bx = (i % 2 == 0) ? 14 : 124;
      int16_t by = (i < 2) ? 74 : 124;
      canvas.fillRoundRect(bx, by, 102, 42, 8, sel ? COLOR_HEX_ACTIVE : COLOR_HEX_FILL);
      canvas.drawRoundRect(bx, by, 102, 42, 8, sel ? COLOR_GREEN : COLOR_HEX_BORDER);
      char maBuf[12];
      snprintf(maBuf, sizeof(maBuf), "%u mA", (unsigned)ma);
      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(sel ? COLOR_GREEN : COLOR_TEXT, sel ? COLOR_HEX_ACTIVE : COLOR_HEX_FILL);
      canvas.drawString(maBuf, bx + 51, by + 21);
    }
  }
  else if (settingsView == SETTINGS_VIEW_POWER) {
    // AXP2101 GPS (BLDO1) and LoRa (ALDO4) power rail switches + OK / Cancel
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_TEXT, COLOR_BG);
    canvas.drawString("Power settings", 120, 44);

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
    canvas.drawString("AXP2101 Reg 0x90 LDO Rails", 120, 60);

    // Row 1: GPS power (BLDO1)
    canvas.fillRoundRect(12, 72, 216, 44, 8, COLOR_HEX_FILL);
    canvas.drawRoundRect(12, 72, 216, 44, 8, editPowerGps ? COLOR_GREEN : COLOR_HEX_BORDER);
    canvas.setTextDatum(top_left);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
    canvas.drawString("GPS power", 22, 78);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
    canvas.drawString("BLDO1 (3.3V)", 22, 100);
    canvas.fillRoundRect(156, 80, 62, 28, 14, editPowerGps ? COLOR_HEX_ACTIVE : COLOR_TOPBAR);
    canvas.drawRoundRect(156, 80, 62, 28, 14, editPowerGps ? COLOR_GREEN : COLOR_SUBTEXT);
    canvas.fillCircle(editPowerGps ? 204 : 170, 94, 10, editPowerGps ? COLOR_GREEN : COLOR_SUBTEXT);
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(editPowerGps ? COLOR_GREEN : COLOR_SUBTEXT, editPowerGps ? COLOR_HEX_ACTIVE : COLOR_TOPBAR);
    canvas.drawString(editPowerGps ? "ON" : "OFF", editPowerGps ? 176 : 196, 94);

    // Row 2: LoRa power (ALDO4)
    canvas.fillRoundRect(12, 122, 216, 44, 8, COLOR_HEX_FILL);
    canvas.drawRoundRect(12, 122, 216, 44, 8, editPowerLora ? COLOR_GREEN : COLOR_HEX_BORDER);
    canvas.setTextDatum(top_left);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
    canvas.drawString("LoRa power", 22, 128);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
    canvas.drawString("ALDO4 (3.3V)", 22, 150);
    canvas.fillRoundRect(156, 130, 62, 28, 14, editPowerLora ? COLOR_HEX_ACTIVE : COLOR_TOPBAR);
    canvas.drawRoundRect(156, 130, 62, 28, 14, editPowerLora ? COLOR_GREEN : COLOR_SUBTEXT);
    canvas.fillCircle(editPowerLora ? 204 : 170, 144, 10, editPowerLora ? COLOR_GREEN : COLOR_SUBTEXT);
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(editPowerLora ? COLOR_GREEN : COLOR_SUBTEXT, editPowerLora ? COLOR_HEX_ACTIVE : COLOR_TOPBAR);
    canvas.drawString(editPowerLora ? "ON" : "OFF", editPowerLora ? 176 : 196, 144);
  }
  else if (settingsView == SETTINGS_VIEW_WAKE_CONDITION) {
    // Wake-up condition switches: Touch (IO16), BMA423 (IO14), Button (IO0) + OK / Cancel
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_TEXT, COLOR_BG);
    canvas.drawString("Wake-up condition", 120, 38);

    const char* wakeTitles[3] = {"Touch", "BMA423", "Button"};
    const char* wakePins[3]   = {"TOUCH_INT (IO16)", "INT1 (IO14)", "BOOT (IO0)"};
    bool wakeVals[3]          = {editWakeOnTouch, editWakeOnBma423, editWakeOnButton};

    for (uint8_t r = 0; r < 3; r++) {
      int16_t ry = (int16_t)(52 + r * 40);
      bool on = wakeVals[r];
      canvas.fillRoundRect(12, ry, 216, 36, 7, COLOR_HEX_FILL);
      canvas.drawRoundRect(12, ry, 216, 36, 7, on ? COLOR_GREEN : COLOR_HEX_BORDER);
      canvas.setTextDatum(top_left);
      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
      canvas.drawString(wakeTitles[r], 20, ry + 3);
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
      canvas.drawString(wakePins[r], 20, ry + 23);

      // Compact pill toggle switch on right
      canvas.fillRoundRect(158, ry + 6, 60, 24, 12, on ? COLOR_HEX_ACTIVE : COLOR_TOPBAR);
      canvas.drawRoundRect(158, ry + 6, 60, 24, 12, on ? COLOR_GREEN : COLOR_SUBTEXT);
      canvas.fillCircle(on ? 206 : 170, ry + 18, 8, on ? COLOR_GREEN : COLOR_SUBTEXT);
      canvas.setTextDatum(middle_center);
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(on ? COLOR_GREEN : COLOR_SUBTEXT, on ? COLOR_HEX_ACTIVE : COLOR_TOPBAR);
      canvas.drawString(on ? "ON" : "OFF", on ? 178 : 196, ry + 18);
    }

    if (!editWakeOnTouch && !editWakeOnBma423 && !editWakeOnButton) {
      canvas.setTextDatum(middle_center);
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(COLOR_RED, COLOR_BG);
      canvas.drawString("Select at least 1 wake source", 120, 173);
    }
  }

  // Shared Cancel (left) and OK (right) buttons for all Settings sub-screens
  canvas.fillRoundRect(10, 178, 104, 44, 8, COLOR_HEX_FILL);
  canvas.drawRoundRect(10, 178, 104, 44, 8, COLOR_RED);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_RED, COLOR_HEX_FILL);
  canvas.drawString("Cancel", 62, 200);

  bool okDisabled = (settingsView == SETTINGS_VIEW_WAKE_CONDITION &&
                     !editWakeOnTouch && !editWakeOnBma423 && !editWakeOnButton);
  canvas.fillRoundRect(126, 178, 104, 44, 8, okDisabled ? COLOR_TOPBAR : COLOR_HEX_ACTIVE);
  canvas.drawRoundRect(126, 178, 104, 44, 8, okDisabled ? COLOR_HEX_BORDER : COLOR_GREEN);
  canvas.setTextColor(okDisabled ? COLOR_SUBTEXT : COLOR_TEXT, okDisabled ? COLOR_TOPBAR : COLOR_HEX_ACTIVE);
  canvas.drawString("OK", 178, 200);
}

// Format ASCII character label (0..127)
void formatAsciiCharName(uint8_t code, char* outBuf, size_t bufSize) {
  if (code < 32) {
    snprintf(outBuf, bufSize, "%s", ASCII_CTRL_NAMES[code]);
  } else if (code == 32) {
    snprintf(outBuf, bufSize, "SP");
  } else if (code == 127) {
    snprintf(outBuf, bufSize, "DEL");
  } else {
    snprintf(outBuf, bufSize, "%c", (char)code);
  }
}

// Draw CP1251 bitmap glyph (0x80..0xFF) from 5x7 matrix
void drawCp1251Glyph5x7(uint8_t code, int16_t x, int16_t y, uint16_t color, uint8_t scale) {
  if (code < 128) return;
  const uint8_t* cols = CP1251_GLYPHS_5X7[code - 128];
  for (uint8_t c = 0; c < 5; c++) {
    uint8_t bits = cols[c];
    for (uint8_t r = 0; r < 7; r++) {
      if (bits & (1U << r)) {
        canvas.fillRect((int16_t)(x + c * scale), (int16_t)(y + r * scale), scale, scale, color);
      }
    }
  }
}

// Draw ASCII table screen (ASCII 0..127 + CP1251 128..255: DEC, HEX, glyph + scroll triangles)
void drawAsciiTableScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  // Table header
  canvas.fillRect(6, 32, 188, 16, COLOR_TOPBAR);
  canvas.drawRect(6, 32, 188, 16, COLOR_HEX_BORDER);
  canvas.setTextDatum(middle_left);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_ACCENT, COLOR_TOPBAR);
  canvas.drawString("DEC", 16, 40);
  canvas.drawString("HEX", 72, 40);
  canvas.drawString("CHR", 136, 40);

  // Right vertical scroll bar (top and bottom triangles)
  bool canUp   = (asciiScrollOffset > 0);
  bool canDown = ((uint16_t)asciiScrollOffset + ASCII_VISIBLE_ROWS < ASCII_TOTAL_CODES);

  canvas.fillRoundRect(199, 34, 36, 56, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(199, 34, 36, 56, 6, canUp ? COLOR_ACCENT : COLOR_HEX_BORDER);
  canvas.fillTriangle(217, 52, 208, 70, 226, 70, canUp ? COLOR_ACCENT : COLOR_SUBTEXT);

  canvas.fillRoundRect(199, 182, 36, 56, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(199, 182, 36, 56, 6, canDown ? COLOR_ACCENT : COLOR_HEX_BORDER);
  canvas.fillTriangle(217, 220, 208, 202, 226, 202, canDown ? COLOR_ACCENT : COLOR_SUBTEXT);

  char pageBuf[12];
  snprintf(pageBuf, sizeof(pageBuf), "%u/32", (unsigned)(asciiScrollOffset / ASCII_VISIBLE_ROWS + 1));
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  canvas.drawString(pageBuf, 217, 136);

  // ASCII + CP1251 table rows (8 visible rows)
  canvas.setFont(&fonts::Font2);
  for (uint8_t r = 0; r < ASCII_VISIBLE_ROWS; r++) {
    uint16_t code16 = (uint16_t)asciiScrollOffset + r;
    if (code16 >= ASCII_TOTAL_CODES) break;
    uint8_t code = (uint8_t)code16;

    int16_t ry = (int16_t)(50 + r * 23);
    uint16_t rowBg = (r % 2 == 0) ? COLOR_HEX_FILL : COLOR_TOPBAR;
    canvas.fillRect(6, ry, 188, 22, rowBg);
    canvas.drawFastHLine(6, ry + 22, 188, COLOR_BG);

    char decBuf[8], hexBuf[8], chrBuf[8];
    snprintf(decBuf, sizeof(decBuf), "%3u", (unsigned)code);
    snprintf(hexBuf, sizeof(hexBuf), "0x%02X", (unsigned)code);

    canvas.setTextDatum(middle_left);
    canvas.setTextColor(COLOR_TEXT, rowBg);
    canvas.drawString(decBuf, 14, ry + 11);

    canvas.setTextColor(COLOR_ACCENT, rowBg);
    canvas.drawString(hexBuf, 66, ry + 11);

    if (code < 128) {
      formatAsciiCharName(code, chrBuf, sizeof(chrBuf));
      bool isControl = (code <= 32 || code == 127);
      canvas.setTextColor(isControl ? COLOR_WARN : COLOR_GREEN, rowBg);
      canvas.drawString(chrBuf, 136, ry + 11);
    } else if (code == 0x98) {
      canvas.setTextColor(COLOR_SUBTEXT, rowBg);
      canvas.drawString("---", 136, ry + 11);
    } else if (code == 0xA0) {
      canvas.setTextColor(COLOR_WARN, rowBg);
      canvas.drawString("NBS", 136, ry + 11);
    } else if (code == 0xAD) {
      canvas.setTextColor(COLOR_WARN, rowBg);
      canvas.drawString("SHY", 136, ry + 11);
    } else {
      drawCp1251Glyph5x7(code, 136, (int16_t)(ry + 4), COLOR_GREEN, 2);
    }
  }
}

// Read ESP32, BMA423, and AXP2101 temperatures (in deg C) and append to history (1 Hz)
void recordEsp32TempSample() {
  float tC = temperatureRead();
  if (isnan(tC) || tC < -60.0f || tC > 150.0f) {
    tC = currentTempC;
  }
  currentTempC = tC;

  float bmaC = currentBmaTempC;
  bool bmaOk = readBma423TempC(bmaC);
  if (bmaOk) {
    currentBmaTempC = bmaC;
    bmaTempValid = true;
  }

  float axpC = currentAxpTempC;
  bool axpOk = readAxp2101TempC(axpC);
  if (axpOk) {
    currentAxpTempC = axpC;
    axpTempValid = true;
  }

  if (tempHistoryCount < TEMP_HISTORY_MAX) {
    tempHistoryC[tempHistoryCount]        = tC;
    bmaTempHistoryC[tempHistoryCount]     = currentBmaTempC;
    bmaTempHistoryValid[tempHistoryCount] = bmaTempValid;
    axpTempHistoryC[tempHistoryCount]     = currentAxpTempC;
    axpTempHistoryValid[tempHistoryCount] = axpTempValid;
    tempHistoryCount++;
  } else {
    for (uint8_t i = 0; i + 1 < TEMP_HISTORY_MAX; i++) {
      tempHistoryC[i]        = tempHistoryC[i + 1];
      bmaTempHistoryC[i]     = bmaTempHistoryC[i + 1];
      bmaTempHistoryValid[i] = bmaTempHistoryValid[i + 1];
      axpTempHistoryC[i]     = axpTempHistoryC[i + 1];
      axpTempHistoryValid[i] = axpTempHistoryValid[i + 1];
    }
    tempHistoryC[TEMP_HISTORY_MAX - 1]        = tC;
    bmaTempHistoryC[TEMP_HISTORY_MAX - 1]     = currentBmaTempC;
    bmaTempHistoryValid[TEMP_HISTORY_MAX - 1] = bmaTempValid;
    axpTempHistoryC[TEMP_HISTORY_MAX - 1]     = currentAxpTempC;
    axpTempHistoryValid[TEMP_HISTORY_MAX - 1] = axpTempValid;
  }
}

// Convert temperature from deg C to selected display unit (C, K, F)
float convertTempFromC(float tempC, TempDisplayUnit unit) {
  if (unit == TEMP_UNIT_K) return tempC + 273.15f;
  if (unit == TEMP_UNIT_F) return tempC * 1.8f + 32.0f;
  return tempC;
}

// Draw Temp. sensor screen (ESP32 red + BMA423 orange + AXP2101 yellow traces + 3-temp C/K/F boxes)
void drawTempSensorScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  const char* unitTag = (tempGraphUnit == TEMP_UNIT_C) ? "C" : ((tempGraphUnit == TEMP_UNIT_K) ? "K" : "F");
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(middle_left);

  canvas.fillRect(42, 33, 5, 5, COLOR_RED);
  canvas.setTextColor(COLOR_TEXT, COLOR_BG);
  canvas.drawString("ESP32", 49, 35);

  canvas.fillRect(83, 33, 5, 5, COLOR_ORANGE);
  canvas.setTextColor(COLOR_ORANGE, COLOR_BG);
  canvas.drawString("BMA423", 90, 35);

  canvas.fillRect(130, 33, 5, 5, COLOR_YELLOW);
  canvas.setTextColor(COLOR_YELLOW, COLOR_BG);
  canvas.drawString("AXP2101", 137, 35);

  char unitBuf[8];
  snprintf(unitBuf, sizeof(unitBuf), "(%s)", unitTag);
  canvas.setTextDatum(middle_right);
  canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
  canvas.drawString(unitBuf, 234, 35);

  // Plot area in center of screen
  const int16_t gx = 42, gy = 43, gw = 192, gh = 117;
  canvas.fillRoundRect(gx, gy, gw, gh, 4, COLOR_TOPBAR);
  canvas.drawRoundRect(gx, gy, gw, gh, 4, COLOR_HEX_BORDER);

  // Auto-scale Y axis across all three curves (ESP32, BMA423, and AXP2101)
  float minVal = convertTempFromC(currentTempC, tempGraphUnit);
  float maxVal = minVal;
  if (bmaTempValid) {
    float bv = convertTempFromC(currentBmaTempC, tempGraphUnit);
    if (bv < minVal) minVal = bv;
    if (bv > maxVal) maxVal = bv;
  }
  if (axpTempValid) {
    float av = convertTempFromC(currentAxpTempC, tempGraphUnit);
    if (av < minVal) minVal = av;
    if (av > maxVal) maxVal = av;
  }
  for (uint8_t i = 0; i < tempHistoryCount; i++) {
    float v = convertTempFromC(tempHistoryC[i], tempGraphUnit);
    if (v < minVal) minVal = v;
    if (v > maxVal) maxVal = v;
    if (bmaTempHistoryValid[i]) {
      float bv = convertTempFromC(bmaTempHistoryC[i], tempGraphUnit);
      if (bv < minVal) minVal = bv;
      if (bv > maxVal) maxVal = bv;
    }
    if (axpTempHistoryValid[i]) {
      float av = convertTempFromC(axpTempHistoryC[i], tempGraphUnit);
      if (av < minVal) minVal = av;
      if (av > maxVal) maxVal = av;
    }
  }

  float minSpan = (tempGraphUnit == TEMP_UNIT_F) ? 1.8f : 1.0f;
  float span = maxVal - minVal;
  if (span < minSpan) {
    float mid = (maxVal + minVal) * 0.5f;
    minVal = mid - minSpan * 0.5f;
    maxVal = mid + minSpan * 0.5f;
  } else {
    float pad = span * 0.15f;
    minVal -= pad;
    maxVal += pad;
  }
  float midVal = (minVal + maxVal) * 0.5f;

  // Horizontal grid and Y-axis labels on left
  const int16_t plotTop = gy + 8;
  const int16_t plotBot = gy + gh - 8;
  const int16_t plotMid = (plotTop + plotBot) / 2;

  for (int16_t x = gx + 4; x < gx + gw - 4; x += 6) {
    canvas.drawPixel(x, plotTop, COLOR_HEX_BORDER);
    canvas.drawPixel(x, plotMid, COLOR_HEX_BORDER);
    canvas.drawPixel(x, plotBot, COLOR_HEX_BORDER);
  }

  char scaleBuf[12];
  canvas.setTextDatum(middle_right);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  snprintf(scaleBuf, sizeof(scaleBuf), "%.1f", maxVal);
  canvas.drawString(scaleBuf, gx - 4, plotTop);
  snprintf(scaleBuf, sizeof(scaleBuf), "%.1f", midVal);
  canvas.drawString(scaleBuf, gx - 4, plotMid);
  snprintf(scaleBuf, sizeof(scaleBuf), "%.1f", minVal);
  canvas.drawString(scaleBuf, gx - 4, plotBot);

  // Temperature trend lines: AXP2101 (yellow), BMA423 (orange), and ESP32 (red)
  if (tempHistoryCount > 0) {
    const int16_t plotLeft  = gx + 5;
    const int16_t plotRight = gx + gw - 5;
    const float denom = (maxVal - minVal);
    uint8_t denomIdx = (tempHistoryCount > 1) ? (uint8_t)(tempHistoryCount - 1) : 1;

    // 1. Third trace: AXP2101 PMU die temperature (yellow)
    bool hasPrevAxp = false;
    int16_t prevAxpPx = plotLeft;
    int16_t prevAxpPy = plotMid;
    for (uint8_t i = 0; i < tempHistoryCount; i++) {
      if (!axpTempHistoryValid[i]) continue;
      float av = convertTempFromC(axpTempHistoryC[i], tempGraphUnit);
      float norm = (denom > 1e-4f) ? ((av - minVal) / denom) : 0.5f;
      if (norm < 0.0f) norm = 0.0f;
      if (norm > 1.0f) norm = 1.0f;

      int16_t px = (tempHistoryCount == 1)
                     ? (int16_t)((plotLeft + plotRight) / 2)
                     : (int16_t)(plotLeft + ((int32_t)i * (plotRight - plotLeft)) / denomIdx);
      int16_t py = (int16_t)(plotBot - (int16_t)(norm * (float)(plotBot - plotTop) + 0.5f));

      if (hasPrevAxp) {
        canvas.drawLine(prevAxpPx, prevAxpPy, px, py, COLOR_YELLOW);
        canvas.drawLine(prevAxpPx, prevAxpPy - 1, px, py - 1, COLOR_YELLOW);
      }
      prevAxpPx = px;
      prevAxpPy = py;
      hasPrevAxp = true;
    }
    if (hasPrevAxp) {
      canvas.fillCircle(prevAxpPx, prevAxpPy, 3, COLOR_YELLOW);
    }

    // 2. Secondary trace: BMA423 (orange)
    bool hasPrevBma = false;
    int16_t prevBmaPx = plotLeft;
    int16_t prevBmaPy = plotMid;
    for (uint8_t i = 0; i < tempHistoryCount; i++) {
      if (!bmaTempHistoryValid[i]) continue;
      float bv = convertTempFromC(bmaTempHistoryC[i], tempGraphUnit);
      float norm = (denom > 1e-4f) ? ((bv - minVal) / denom) : 0.5f;
      if (norm < 0.0f) norm = 0.0f;
      if (norm > 1.0f) norm = 1.0f;

      int16_t px = (tempHistoryCount == 1)
                     ? (int16_t)((plotLeft + plotRight) / 2)
                     : (int16_t)(plotLeft + ((int32_t)i * (plotRight - plotLeft)) / denomIdx);
      int16_t py = (int16_t)(plotBot - (int16_t)(norm * (float)(plotBot - plotTop) + 0.5f));

      if (hasPrevBma) {
        canvas.drawLine(prevBmaPx, prevBmaPy, px, py, COLOR_ORANGE);
        canvas.drawLine(prevBmaPx, prevBmaPy - 1, px, py - 1, COLOR_ORANGE);
      }
      prevBmaPx = px;
      prevBmaPy = py;
      hasPrevBma = true;
    }
    if (hasPrevBma) {
      canvas.fillCircle(prevBmaPx, prevBmaPy, 3, COLOR_ORANGE);
    }

    // 3. Primary trace: ESP32 (red)
    int16_t prevPx = plotLeft;
    int16_t prevPy = plotMid;
    for (uint8_t i = 0; i < tempHistoryCount; i++) {
      float v = convertTempFromC(tempHistoryC[i], tempGraphUnit);
      float norm = (denom > 1e-4f) ? ((v - minVal) / denom) : 0.5f;
      if (norm < 0.0f) norm = 0.0f;
      if (norm > 1.0f) norm = 1.0f;

      int16_t px = (tempHistoryCount == 1)
                     ? (int16_t)((plotLeft + plotRight) / 2)
                     : (int16_t)(plotLeft + ((int32_t)i * (plotRight - plotLeft)) / denomIdx);
      int16_t py = (int16_t)(plotBot - (int16_t)(norm * (float)(plotBot - plotTop) + 0.5f));

      if (i > 0) {
        canvas.drawLine(prevPx, prevPy, px, py, COLOR_RED);
        canvas.drawLine(prevPx, prevPy - 1, px, py - 1, COLOR_RED);
      }
      prevPx = px;
      prevPy = py;
    }
    canvas.fillCircle(prevPx, prevPy, 3, COLOR_RED);
  }

  // Bottom 3 numeric boxes (C, K, F): three temperatures in each (ESP32, BMA423, AXP2101)
  float espVals[3] = {
    currentTempC,
    convertTempFromC(currentTempC, TEMP_UNIT_K),
    convertTempFromC(currentTempC, TEMP_UNIT_F)
  };
  float bmaVals[3] = {
    currentBmaTempC,
    convertTempFromC(currentBmaTempC, TEMP_UNIT_K),
    convertTempFromC(currentBmaTempC, TEMP_UNIT_F)
  };
  float axpVals[3] = {
    currentAxpTempC,
    convertTempFromC(currentAxpTempC, TEMP_UNIT_K),
    convertTempFromC(currentAxpTempC, TEMP_UNIT_F)
  };
  const char* labels[3] = {"C", "K", "F"};

  for (uint8_t u = 0; u < 3; u++) {
    int16_t bx = (int16_t)(6 + u * 77);
    int16_t by = 164;
    int16_t bw = 74;
    int16_t bh = 72;
    bool active = ((uint8_t)tempGraphUnit == u);
    uint16_t fillCol = active ? COLOR_HEX_ACTIVE : COLOR_HEX_FILL;
    uint16_t bordCol = active ? COLOR_ACCENT : COLOR_HEX_BORDER;

    canvas.fillRoundRect(bx, by, bw, bh, 6, fillCol);
    canvas.drawRoundRect(bx, by, bw, bh, 6, bordCol);

    canvas.setTextDatum(top_center);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(active ? COLOR_ACCENT : COLOR_SUBTEXT, fillCol);
    canvas.drawString(labels[u], bx + bw / 2, by + 3);
    if (u != 1) {
      canvas.drawCircle(bx + bw / 2 - 7, by + 5, 1, active ? COLOR_ACCENT : COLOR_SUBTEXT);
    }

    // Row 1: ESP32 temperature (red marker)
    char espBuf[12];
    snprintf(espBuf, sizeof(espBuf), "%.1f", espVals[u]);
    canvas.fillRect(bx + 6, by + 21, 4, 4, COLOR_RED);
    canvas.setTextDatum(middle_right);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(active ? COLOR_TEXT : COLOR_SUBTEXT, fillCol);
    canvas.drawString(espBuf, bx + bw - 5, by + 23);

    // Row 2: BMA423 temperature (orange marker)
    char bmaBuf[12];
    if (bmaTempValid) {
      snprintf(bmaBuf, sizeof(bmaBuf), "%.1f", bmaVals[u]);
    } else {
      snprintf(bmaBuf, sizeof(bmaBuf), "--.-");
    }
    canvas.fillRect(bx + 6, by + 39, 4, 4, COLOR_ORANGE);
    canvas.setTextDatum(middle_right);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(bmaTempValid ? COLOR_ORANGE : COLOR_SUBTEXT, fillCol);
    canvas.drawString(bmaBuf, bx + bw - 5, by + 41);

    // Row 3: AXP2101 temperature (yellow marker)
    char axpBuf[12];
    if (axpTempValid) {
      snprintf(axpBuf, sizeof(axpBuf), "%.1f", axpVals[u]);
    } else {
      snprintf(axpBuf, sizeof(axpBuf), "--.-");
    }
    canvas.fillRect(bx + 6, by + 57, 4, 4, COLOR_YELLOW);
    canvas.setTextDatum(middle_right);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(axpTempValid ? COLOR_YELLOW : COLOR_SUBTEXT, fillCol);
    canvas.drawString(axpBuf, bx + bw - 5, by + 59);
  }
}

// Chemical element category color (RGB565)
uint16_t getPeriodicCategoryColor(uint8_t cat) {
  switch (cat) {
    case 0:  return COLOR_GREEN;   // Nonmetal
    case 1:  return 0x07FF;        // Noble gas (cyan)
    case 2:  return COLOR_RED;     // Alkali metal
    case 3:  return COLOR_WARN;    // Alkaline earth
    case 4:  return 0x05F7;        // Metalloid (teal)
    case 5:  return 0xFFE0;        // Halogen (yellow)
    case 6:  return COLOR_ACCENT;  // Transition metal
    case 7:  return 0x7BEF;        // Post-transition (light steel)
    case 8:  return 0xFD20;        // Lanthanide (orange)
    default: return 0xF81F;        // Actinide (magenta)
  }
}

// Periodic table page switch button in top-right corner (1/4 .. 4/4)
void drawTopRightPeriodicPageIcon(uint8_t page) {
  uint16_t bordCol = (page == 3) ? 0xF81F : ((page == 0) ? COLOR_ACCENT : COLOR_WARN);
  canvas.fillRoundRect(204, 2, 32, 24, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(204, 2, 32, 24, 6, bordCol);
  char pBuf[8];
  snprintf(pBuf, sizeof(pBuf), "%u/%u", (unsigned)(page + 1), (unsigned)PERIODIC_PAGE_COUNT);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(bordCol, COLOR_HEX_FILL);
  canvas.drawString(pBuf, 220, 14);
}

// Compute element cell coordinates on the given Periodic table page (false if not on page)
bool getPeriodicElementSlot(uint8_t idx, uint8_t page, int16_t &rx, int16_t &ry, int16_t &rw, int16_t &rh) {
  if (idx >= PERIODIC_ELEMENT_COUNT) return false;
  const PeriodicElement &el = PERIODIC_ELEMENTS[idx];
  if (page <= 2) {
    uint8_t gStart = (uint8_t)(page * 6 + 1);
    uint8_t gEnd   = (uint8_t)(gStart + 5);
    int8_t row = -1;
    int8_t col = -1;
    if (el.period >= 1 && el.period <= 7 && el.group >= gStart && el.group <= gEnd) {
      row = (int8_t)(el.period - 1);
      col = (int8_t)(el.group - gStart);
    } else if (page == 0 && el.atomicNumber == 57) {
      row = 5;
      col = 2;
    } else if (page == 0 && el.atomicNumber == 89) {
      row = 6;
      col = 2;
    }
    if (row >= 0 && col >= 0) {
      rx = (int16_t)(16 + col * 37);
      ry = (int16_t)(54 + row * 26);
      rw = 35;
      rh = 24;
      return true;
    }
    return false;
  }

  // Page 4 (page == 3): lanthanides (57-71) and actinides (89-103)
  if (el.period == 8) {
    uint8_t pos = (uint8_t)(el.group - 1);
    rx = (int16_t)(6 + (pos % 5) * 46);
    ry = (int16_t)(52 + (pos / 5) * 27);
    rw = 44;
    rh = 25;
    return true;
  } else if (el.period == 9) {
    uint8_t pos = (uint8_t)(el.group - 1);
    rx = (int16_t)(6 + (pos % 5) * 46);
    ry = (int16_t)(148 + (pos / 5) * 27);
    rw = 44;
    rh = 25;
    return true;
  }
  return false;
}

// Draw Periodic table screen (4 pages of periodic table + element detail dialog)
void drawPeriodicTableScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  if (periodicView == PERIODIC_VIEW_TABLE) {
    drawTopRightPeriodicPageIcon(periodicPage);

    canvas.setTextDatum(top_center);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
    canvas.drawString(PERIODIC_PAGE_TITLES[periodicPage % PERIODIC_PAGE_COUNT], 120, 31);

    if (periodicPage <= 2) {
      // Group number headers (top) and period headers (left)
      uint8_t gStart = (uint8_t)(periodicPage * 6 + 1);
      canvas.setTextDatum(middle_center);
      canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
      for (uint8_t c = 0; c < 6; c++) {
        char gBuf[4];
        snprintf(gBuf, sizeof(gBuf), "%u", (unsigned)(gStart + c));
        canvas.drawString(gBuf, 16 + c * 37 + 17, 46);
      }
      for (uint8_t r = 0; r < 7; r++) {
        char pBuf[4];
        snprintf(pBuf, sizeof(pBuf), "%u", (unsigned)(r + 1));
        canvas.drawString(pBuf, 7, 54 + r * 26 + 12);
      }
    } else {
      canvas.setTextDatum(top_left);
      canvas.setTextColor(0xFD20, COLOR_BG);
      canvas.drawString("Lanthanides (57-71)", 8, 42);
      canvas.setTextColor(0xF81F, COLOR_BG);
      canvas.drawString("Actinides (89-103)", 8, 138);
    }

    // Chemical element cells on current page
    for (uint8_t i = 0; i < PERIODIC_ELEMENT_COUNT; i++) {
      int16_t bx, by, bw, bh;
      if (!getPeriodicElementSlot(i, periodicPage, bx, by, bw, bh)) continue;

      const PeriodicElement &el = PERIODIC_ELEMENTS[i];
      uint16_t catCol = getPeriodicCategoryColor(el.category);
      canvas.fillRoundRect(bx, by, bw, bh, 3, COLOR_HEX_FILL);
      canvas.drawRoundRect(bx, by, bw, bh, 3, catCol);

      char zBuf[6];
      snprintf(zBuf, sizeof(zBuf), "%u", (unsigned)el.atomicNumber);
      canvas.setTextDatum(top_left);
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
      canvas.drawString(zBuf, bx + 2, by + 2);

      canvas.setTextDatum(middle_center);
      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
      canvas.drawString(el.symbol, bx + bw / 2 + 2, by + bh / 2 + 4);
    }
    return;
  }

  // Element detail dialog with basic properties
  if (periodicSelectedIdx < 0 || periodicSelectedIdx >= PERIODIC_ELEMENT_COUNT) return;
  const PeriodicElement &el = PERIODIC_ELEMENTS[periodicSelectedIdx];
  uint16_t catCol = getPeriodicCategoryColor(el.category);

  canvas.fillRoundRect(8, 32, 224, 152, 8, COLOR_TOPBAR);
  canvas.drawRoundRect(8, 32, 224, 152, 8, catCol);

  // Symbol and atomic number badge at top-left
  canvas.fillRoundRect(15, 38, 46, 38, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(15, 38, 46, 38, 6, catCol);
  char zBuf[8];
  snprintf(zBuf, sizeof(zBuf), "%u", (unsigned)el.atomicNumber);
  canvas.setTextDatum(top_left);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(catCol, COLOR_HEX_FILL);
  canvas.drawString(zBuf, 19, 41);

  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font4);
  canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
  canvas.drawString(el.symbol, 38, 62);

  // Element name and category to the right of symbol
  canvas.setTextDatum(top_left);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_TEXT, COLOR_TOPBAR);
  canvas.drawString(el.name, 68, 41);

  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(catCol, COLOR_TOPBAR);
  canvas.drawString(PERIODIC_CATEGORIES[el.category % 10], 68, 63);

  canvas.drawFastHLine(14, 82, 212, COLOR_HEX_BORDER);

  // Key physical/chemical properties (atomic number, mass, STP density, MP and BP in deg C)
  char valBuf[32];
  const int16_t rowYs[5] = {91, 109, 127, 145, 163};
  const char* rowLabels[5] = {
    "Atomic num:",
    "Atomic mass:",
    "Density (STP):",
    "Melting pt:",
    "Boiling pt:"
  };

  for (uint8_t r = 0; r < 5; r++) {
    canvas.setTextDatum(middle_left);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
    canvas.drawString(rowLabels[r], 16, rowYs[r]);

    bool isTempRow = (r == 3 || r == 4);
    bool isNA = false;
    if (r == 0) {
      snprintf(valBuf, sizeof(valBuf), "%u", (unsigned)el.atomicNumber);
    } else if (r == 1) {
      snprintf(valBuf, sizeof(valBuf), "%s u", el.mass);
    } else if (r == 2) {
      if (strcmp(el.density, "N/A") == 0) {
        snprintf(valBuf, sizeof(valBuf), "N/A");
        isNA = true;
      } else {
        snprintf(valBuf, sizeof(valBuf), "%s g/cm3", el.density);
      }
    } else if (r == 3) {
      if (strcmp(el.meltC, "N/A") == 0) {
        snprintf(valBuf, sizeof(valBuf), "N/A");
        isNA = true;
      } else {
        snprintf(valBuf, sizeof(valBuf), "%s  C", el.meltC);
      }
    } else {
      if (strcmp(el.boilC, "N/A") == 0) {
        snprintf(valBuf, sizeof(valBuf), "N/A");
        isNA = true;
      } else {
        snprintf(valBuf, sizeof(valBuf), "%s  C", el.boilC);
      }
    }

    canvas.setTextDatum(middle_right);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(isNA ? COLOR_SUBTEXT : COLOR_TEXT, COLOR_TOPBAR);
    canvas.drawString(valBuf, 224, rowYs[r]);
    if (isTempRow && !isNA) {
      // Degree circle before letter "C"
      canvas.drawCircle(214, rowYs[r] - 2, 1, COLOR_TEXT);
    }
  }

  // OK button to return to Periodic table
  canvas.fillRoundRect(60, 192, 120, 40, 8, COLOR_HEX_ACTIVE);
  canvas.drawRoundRect(60, 192, 120, 40, 8, COLOR_GREEN);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_TEXT, COLOR_HEX_ACTIVE);
  canvas.drawString("OK", 120, 212);
}

// Hazard level color for food additive
uint16_t getEAddHazardColor(uint8_t level) {
  switch (level) {
    case 0:  return COLOR_GREEN;
    case 1:  return COLOR_ACCENT;
    case 2:  return COLOR_WARN;
    default: return COLOR_RED;
  }
}

// Draw string wrapped across up to 2 lines at spaces (up to maxChars on first line)
void drawWrappedText2Lines(const char* text, int16_t x, int16_t y, uint8_t maxChars, uint16_t fg, uint16_t bg) {
  size_t len = strlen(text);
  canvas.setTextDatum(top_left);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(fg, bg);
  if (len <= maxChars) {
    canvas.drawString(text, x, y);
    return;
  }
  size_t split = maxChars;
  for (size_t i = maxChars; i > 0; i--) {
    if (text[i] == ' ') {
      split = i;
      break;
    }
  }
  char line1[40];
  size_t copyLen = (split < sizeof(line1) - 1) ? split : (sizeof(line1) - 1);
  memcpy(line1, text, copyLen);
  line1[copyLen] = '\0';
  canvas.drawString(line1, x, y);

  const char* rest = text + split;
  while (*rest == ' ') rest++;
  if (*rest != '\0') {
    canvas.drawString(rest, x, y + 11);
  }
}

// Draw Food e-additives screen (E-number list + detail dialog)
void drawEAdditivesScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  if (eAddView == EADD_VIEW_LIST) {
    bool canUp   = (eAddScrollOffset > 0);
    bool canDown = ((uint16_t)eAddScrollOffset + EADD_VISIBLE_ROWS < EADD_COUNT);

    // Scroll-up triangle (top-right)
    canvas.fillRoundRect(199, 34, 36, 56, 6, COLOR_HEX_FILL);
    canvas.drawRoundRect(199, 34, 36, 56, 6, canUp ? COLOR_ACCENT : COLOR_HEX_BORDER);
    canvas.fillTriangle(217, 52, 208, 70, 226, 70, canUp ? COLOR_ACCENT : COLOR_SUBTEXT);

    // Scroll-down triangle (bottom-right)
    canvas.fillRoundRect(199, 182, 36, 56, 6, COLOR_HEX_FILL);
    canvas.drawRoundRect(199, 182, 36, 56, 6, canDown ? COLOR_ACCENT : COLOR_HEX_BORDER);
    canvas.fillTriangle(217, 220, 208, 202, 226, 202, canDown ? COLOR_ACCENT : COLOR_SUBTEXT);

    // Current list page indicator
    uint8_t totalPages = (uint8_t)((EADD_COUNT + EADD_VISIBLE_ROWS - 1) / EADD_VISIBLE_ROWS);
    uint8_t currPage   = (uint8_t)(eAddScrollOffset / EADD_VISIBLE_ROWS + 1);
    char pgBuf[12];
    snprintf(pgBuf, sizeof(pgBuf), "%u/%u", (unsigned)currPage, (unsigned)totalPages);
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
    canvas.drawString(pgBuf, 217, 136);

    // 5 visible list rows (E-code + short name)
    for (uint8_t r = 0; r < EADD_VISIBLE_ROWS; r++) {
      uint16_t idx = (uint16_t)eAddScrollOffset + r;
      if (idx >= EADD_COUNT) break;
      const FoodEAdditive &item = FOOD_E_ADDITIVES[idx];
      int16_t ry = (int16_t)(34 + r * 41);
      uint16_t hazCol = getEAddHazardColor(item.hazardLevel);

      canvas.fillRoundRect(6, ry, 188, 37, 6, COLOR_HEX_FILL);
      canvas.drawRoundRect(6, ry, 188, 37, 6, COLOR_HEX_BORDER);
      // Colored vertical hazard bar on left inside card
      canvas.fillRoundRect(8, ry + 5, 4, 27, 2, hazCol);

      canvas.setTextDatum(middle_left);
      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(COLOR_ACCENT, COLOR_HEX_FILL);
      canvas.drawString(item.code, 16, ry + 18);

      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
      canvas.drawString(item.shortName, 68, ry + 18);
    }
    return;
  }

  // Food additive detail dialog
  if (eAddSelectedIdx < 0 || eAddSelectedIdx >= EADD_COUNT) return;
  const FoodEAdditive &item = FOOD_E_ADDITIVES[eAddSelectedIdx];
  uint16_t hazCol = getEAddHazardColor(item.hazardLevel);

  canvas.fillRoundRect(8, 32, 224, 152, 8, COLOR_TOPBAR);
  canvas.drawRoundRect(8, 32, 224, 152, 8, hazCol);

  canvas.setTextDatum(middle_left);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_ACCENT, COLOR_TOPBAR);
  canvas.drawString(item.code, 16, 46);

  canvas.setTextDatum(middle_right);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(hazCol, COLOR_TOPBAR);
  canvas.drawString(EADD_HAZARD_LABELS[item.hazardLevel % 4], 224, 46);

  canvas.drawFastHLine(14, 59, 212, COLOR_HEX_BORDER);

  canvas.setTextDatum(top_left);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
  canvas.drawString("FULL NAME:", 16, 65);
  drawWrappedText2Lines(item.fullName, 16, 77, 33, COLOR_TEXT, COLOR_TOPBAR);

  canvas.setTextDatum(top_left);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
  canvas.drawString("TYPE:", 16, 104);
  canvas.setTextColor(COLOR_ACCENT, COLOR_TOPBAR);
  canvas.drawString(item.category, 16, 116);

  canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
  canvas.drawString("HAZARD:", 16, 136);
  canvas.setTextColor(hazCol, COLOR_TOPBAR);
  canvas.drawString(EADD_HAZARD_LABELS[item.hazardLevel % 4], 16, 148);
  drawWrappedText2Lines(item.hazardNote, 16, 160, 33, COLOR_TEXT, COLOR_TOPBAR);

  // OK button to return to Food e-additives list
  canvas.fillRoundRect(60, 192, 120, 40, 8, COLOR_HEX_ACTIVE);
  canvas.drawRoundRect(60, 192, 120, 40, 8, COLOR_GREEN);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_TEXT, COLOR_HEX_ACTIVE);
  canvas.drawString("OK", 120, 212);
}

// Draw Wi-Fi analyzer screen (11 Wi-Fi channels with occupancy bars)
void drawWifiAnalyzerScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  canvas.setTextDatum(top_center);
  canvas.setFont(&fonts::Font0);
  if (!wifiAnalyzerHasData && wifiAnalyzerScanning) {
    canvas.setTextColor(COLOR_WARN, COLOR_BG);
    canvas.drawString("WI-FI ANALYZER (SCAN...)", 120, 31);
  } else {
    canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
    canvas.drawString("WI-FI ANALYZER (CH 1-11)", 120, 31);
  }

  const int16_t startY  = 42;
  const int16_t rowStep = 18;
  const int16_t barX    = 38;
  const int16_t barW    = 164;
  const int16_t barH    = 14;

  char chBuf[8];
  char pctBuf[8];

  for (uint8_t ch = 0; ch < WIFI_ANALYZER_CHANNELS; ch++) {
    int16_t ry = (int16_t)(startY + ch * rowStep);
    int16_t midY = (int16_t)(ry + barH / 2);
    uint8_t load = wifiAnalyzerLoad[ch];
    if (load > 100) load = 100;

    // Channel number on left
    snprintf(chBuf, sizeof(chBuf), "Ch%2u", (unsigned)(ch + 1));
    canvas.setTextDatum(middle_left);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_TEXT, COLOR_BG);
    canvas.drawString(chBuf, 6, midY);

    // Channel occupancy bar
    canvas.fillRect(barX, ry, barW, barH, COLOR_TOPBAR);
    for (uint8_t q = 1; q <= 3; q++) {
      int16_t qx = (int16_t)(barX + (barW * q) / 4);
      canvas.drawFastVLine(qx, ry + 1, barH - 2, COLOR_HEX_FILL);
    }

    int16_t fillW = (int16_t)(((int32_t)(barW - 2) * load + 50) / 100);
    uint16_t barCol = (load < 35) ? COLOR_GREEN : ((load < 70) ? COLOR_WARN : COLOR_RED);
    if (fillW > 0) {
      canvas.fillRect(barX + 1, ry + 1, fillW, barH - 2, barCol);
    }
    canvas.drawRect(barX, ry, barW, barH, COLOR_HEX_BORDER);

    // Occupancy percentage on right
    snprintf(pctBuf, sizeof(pctBuf), "%3u%%", (unsigned)load);
    canvas.setTextDatum(middle_right);
    canvas.setTextColor((load == 0) ? COLOR_SUBTEXT : barCol, COLOR_BG);
    canvas.drawString(pctBuf, 235, midY);
  }
}

// Draw "About firmware" screen (name, version, device, author, emails)
void drawAboutFirmwareScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  canvas.setTextDatum(top_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
  canvas.drawString("ABOUT FIRMWARE", 120, 31);

  canvas.fillRoundRect(8, 44, 224, 188, 8, COLOR_TOPBAR);
  canvas.drawRoundRect(8, 44, 224, 188, 8, COLOR_ACCENT);

  // Firmware name
  canvas.setTextDatum(top_left);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
  canvas.drawString("FIRMWARE:", 16, 52);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_ACCENT, COLOR_TOPBAR);
  canvas.drawString(FW_NAME, 16, 62);

  canvas.drawFastHLine(14, 83, 212, COLOR_HEX_BORDER);

  // Firmware version (3 dot-separated numbers)
  char verBuf[24];
  snprintf(verBuf, sizeof(verBuf), "%u.%u.%u",
           (unsigned)FW_VERSION_MAJOR,
           (unsigned)FW_VERSION_MINOR,
           (unsigned)FW_VERSION_PATCH);
  canvas.setTextDatum(middle_left);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
  canvas.drawString("VERSION:", 16, 95);
  canvas.setTextDatum(middle_right);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_GREEN, COLOR_TOPBAR);
  canvas.drawString(verBuf, 224, 95);

  // Device name
  canvas.setTextDatum(top_left);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
  canvas.drawString("DEVICE:", 16, 108);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_TEXT, COLOR_TOPBAR);
  canvas.drawString(DEVICE_NAME, 16, 118);

  canvas.drawFastHLine(14, 139, 212, COLOR_HEX_BORDER);

  // Author
  canvas.setTextDatum(top_left);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
  canvas.drawString("AUTHOR:", 16, 145);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_TEXT, COLOR_TOPBAR);
  canvas.drawString(FW_AUTHOR, 16, 155);

  canvas.drawFastHLine(14, 176, 212, COLOR_HEX_BORDER);

  // Author and support email addresses
  canvas.setTextDatum(top_left);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
  canvas.drawString("EMAIL:", 16, 183);
  canvas.setTextColor(COLOR_ACCENT, COLOR_TOPBAR);
  canvas.drawString(FW_AUTHOR_EMAIL, 16, 196);
  canvas.setTextColor(COLOR_WARN, COLOR_TOPBAR);
  canvas.drawString(FW_SUPPORT_EMAIL, 16, 211);
}

// Draw Wi-Fi jam detect screen (Deauth/Disassoc frames/s, 0..250/s scale, channels 1..11)
void drawWifiJamDetectScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  uint16_t maxPps = 0;
  for (uint8_t ch = 0; ch < WIFI_JAM_CHANNELS; ch++) {
    if (wifiJamPps[ch] > maxPps) maxPps = wifiJamPps[ch];
  }

  canvas.setTextDatum(top_center);
  canvas.setFont(&fonts::Font0);
  if (maxPps >= 30) {
    canvas.setTextColor(COLOR_RED, COLOR_BG);
    canvas.drawString("WI-FI JAM ALERT (0-250/s)", 120, 31);
  } else {
    canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
    canvas.drawString("WI-FI JAM DETECT (0-250/s)", 120, 31);
  }

  const int16_t startY  = 42;
  const int16_t rowStep = 18;
  const int16_t barX    = 38;
  const int16_t barW    = 158;
  const int16_t barH    = 14;

  char chBuf[8];
  char ppsBuf[8];

  for (uint8_t ch = 0; ch < WIFI_JAM_CHANNELS; ch++) {
    int16_t ry   = (int16_t)(startY + ch * rowStep);
    int16_t midY = (int16_t)(ry + barH / 2);
    uint16_t pps = wifiJamPps[ch];
    uint16_t clamped = (pps > WIFI_JAM_MAX_PPS) ? WIFI_JAM_MAX_PPS : pps;
    bool isScanningCh = (wifiJamActive && wifiJamCurrChan == (uint8_t)(ch + 1));

    // Channel number on left
    snprintf(chBuf, sizeof(chBuf), "Ch%2u", (unsigned)(ch + 1));
    canvas.setTextDatum(middle_left);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(isScanningCh ? COLOR_ACCENT : COLOR_TEXT, COLOR_BG);
    canvas.drawString(chBuf, 6, midY);

    // Deauthentication frame rate bar (0..250 pkt/s)
    canvas.fillRect(barX, ry, barW, barH, COLOR_TOPBAR);
    for (uint8_t q = 1; q <= 3; q++) {
      int16_t qx = (int16_t)(barX + (barW * q) / 4);
      canvas.drawFastVLine(qx, ry + 1, barH - 2, COLOR_HEX_FILL);
    }

    int16_t fillW = (int16_t)(((int32_t)(barW - 2) * clamped + (WIFI_JAM_MAX_PPS / 2)) / WIFI_JAM_MAX_PPS);
    uint16_t barCol = (pps < 30) ? COLOR_GREEN : ((pps < 120) ? COLOR_WARN : COLOR_RED);
    if (fillW > 0) {
      canvas.fillRect(barX + 1, ry + 1, fillW, barH - 2, barCol);
    }
    canvas.drawRect(barX, ry, barW, barH, isScanningCh ? COLOR_ACCENT : COLOR_HEX_BORDER);

    // Numeric pkt/s value on right
    if (pps > WIFI_JAM_MAX_PPS) {
      snprintf(ppsBuf, sizeof(ppsBuf), "250+");
    } else {
      snprintf(ppsBuf, sizeof(ppsBuf), "%3u", (unsigned)pps);
    }
    canvas.setTextDatum(middle_right);
    canvas.setTextColor((pps == 0) ? COLOR_SUBTEXT : barCol, COLOR_BG);
    canvas.drawString(ppsBuf, 236, midY);
  }
}

// Draw Accel. sensor screen (3 colored X/Y/Z curves in m/s^2 and 3 numeric boxes below)
void drawAccelSensorScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  // 3-axis legend (X - red, Y - green, Z - cyan) and m/s^2 unit
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(middle_left);
  canvas.fillCircle(44, 35, 3, COLOR_RED);
  canvas.setTextColor(COLOR_RED, COLOR_BG);
  canvas.drawString("X", 50, 35);

  canvas.fillCircle(72, 35, 3, COLOR_GREEN);
  canvas.setTextColor(COLOR_GREEN, COLOR_BG);
  canvas.drawString("Y", 78, 35);

  canvas.fillCircle(100, 35, 3, COLOR_ACCENT);
  canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
  canvas.drawString("Z", 106, 35);

  canvas.setTextDatum(middle_right);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  canvas.drawString("ACCEL (m/s2)", 233, 35);

  // Plot area in center of screen
  const int16_t gx = 42, gy = 43, gw = 192, gh = 131;
  canvas.fillRoundRect(gx, gy, gw, gh, 4, COLOR_TOPBAR);
  canvas.drawRoundRect(gx, gy, gw, gh, 4, COLOR_HEX_BORDER);

  // Auto-scale Y axis across all three X, Y, Z curves
  float minVal = currentAccelX;
  float maxVal = currentAccelX;
  if (currentAccelY < minVal) minVal = currentAccelY;
  if (currentAccelY > maxVal) maxVal = currentAccelY;
  if (currentAccelZ < minVal) minVal = currentAccelZ;
  if (currentAccelZ > maxVal) maxVal = currentAccelZ;

  for (uint8_t i = 0; i < accelHistoryCount; i++) {
    float vx = accelHistoryX[i];
    float vy = accelHistoryY[i];
    float vz = accelHistoryZ[i];
    if (vx < minVal) minVal = vx;
    if (vx > maxVal) maxVal = vx;
    if (vy < minVal) minVal = vy;
    if (vy > maxVal) maxVal = vy;
    if (vz < minVal) minVal = vz;
    if (vz > maxVal) maxVal = vz;
  }

  float minSpan = 2.0f;
  float span = maxVal - minVal;
  if (span < minSpan) {
    float mid = (maxVal + minVal) * 0.5f;
    minVal = mid - minSpan * 0.5f;
    maxVal = mid + minSpan * 0.5f;
  } else {
    float pad = span * 0.15f;
    minVal -= pad;
    maxVal += pad;
  }
  float midVal = (minVal + maxVal) * 0.5f;

  // Horizontal grid and Y-axis labels on left
  const int16_t plotTop = gy + 8;
  const int16_t plotBot = gy + gh - 8;
  const int16_t plotMid = (plotTop + plotBot) / 2;

  for (int16_t x = gx + 4; x < gx + gw - 4; x += 6) {
    canvas.drawPixel(x, plotTop, COLOR_HEX_BORDER);
    canvas.drawPixel(x, plotMid, COLOR_HEX_BORDER);
    canvas.drawPixel(x, plotBot, COLOR_HEX_BORDER);
  }

  char scaleBuf[12];
  canvas.setTextDatum(middle_right);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  snprintf(scaleBuf, sizeof(scaleBuf), "%.1f", maxVal);
  canvas.drawString(scaleBuf, gx - 4, plotTop);
  snprintf(scaleBuf, sizeof(scaleBuf), "%.1f", midVal);
  canvas.drawString(scaleBuf, gx - 4, plotMid);
  snprintf(scaleBuf, sizeof(scaleBuf), "%.1f", minVal);
  canvas.drawString(scaleBuf, gx - 4, plotBot);

  // Draw 3 curves: X (red), Y (green), Z (cyan)
  if (accelHistoryCount > 0) {
    const int16_t plotLeft  = gx + 5;
    const int16_t plotRight = gx + gw - 5;
    const float denom = (maxVal - minVal);
    uint8_t denomIdx = (accelHistoryCount > 1) ? (uint8_t)(accelHistoryCount - 1) : 1;

    const float* series[3] = { accelHistoryX, accelHistoryY, accelHistoryZ };
    const uint16_t colors[3] = { COLOR_RED, COLOR_GREEN, COLOR_ACCENT };

    for (uint8_t axis = 0; axis < 3; axis++) {
      int16_t prevPx = plotLeft;
      int16_t prevPy = plotMid;
      for (uint8_t i = 0; i < accelHistoryCount; i++) {
        float v = series[axis][i];
        float norm = (denom > 1e-4f) ? ((v - minVal) / denom) : 0.5f;
        if (norm < 0.0f) norm = 0.0f;
        if (norm > 1.0f) norm = 1.0f;

        int16_t px = (accelHistoryCount == 1)
                       ? (int16_t)((plotLeft + plotRight) / 2)
                       : (int16_t)(plotLeft + ((int32_t)i * (plotRight - plotLeft)) / denomIdx);
        int16_t py = (int16_t)(plotBot - (int16_t)(norm * (float)(plotBot - plotTop) + 0.5f));

        if (i > 0) {
          canvas.drawLine(prevPx, prevPy, px, py, colors[axis]);
          canvas.drawLine(prevPx, prevPy - 1, px, py - 1, colors[axis]);
        }
        prevPx = px;
        prevPy = py;
      }
      canvas.fillCircle(prevPx, prevPy, 3, colors[axis]);
    }
  }

  // Bottom 3 numeric boxes for X, Y, Z axes in m/s^2
  const char* axisNames[3] = { "X", "Y", "Z" };
  const uint16_t axisCols[3] = { COLOR_RED, COLOR_GREEN, COLOR_ACCENT };
  float vals[3] = { currentAccelX, currentAccelY, currentAccelZ };

  char valBuf[16];
  for (uint8_t u = 0; u < 3; u++) {
    int16_t bx = (int16_t)(6 + u * 77);
    int16_t by = 179;
    int16_t bw = 74;
    int16_t bh = 56;

    canvas.fillRoundRect(bx, by, bw, bh, 6, COLOR_HEX_FILL);
    canvas.drawRoundRect(bx, by, bw, bh, 6, axisCols[u]);

    canvas.setTextDatum(top_center);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(axisCols[u], COLOR_HEX_FILL);
    canvas.drawString(axisNames[u], bx + bw / 2, by + 5);

    snprintf(valBuf, sizeof(valBuf), "%+.2f", vals[u]);
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
    canvas.drawString(valBuf, bx + bw / 2, by + 29);

    canvas.setTextDatum(bottom_center);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
    canvas.drawString("m/s2", bx + bw / 2, by + bh - 4);
  }
}

// Draw Wi-Fi info screen (14 parameters of current access point with vertical scroll)
void drawWifiInfoScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  if (!wifiInfoConnected) {
    canvas.fillRoundRect(14, 62, 212, 116, 8, COLOR_TOPBAR);
    canvas.drawRoundRect(14, 62, 212, 116, 8, COLOR_HEX_BORDER);
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_WARN, COLOR_TOPBAR);
    canvas.drawString("Wi-Fi not connected", 120, 102);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
    canvas.drawString("Connect via Wi-Fi connect", 120, 132);
    canvas.drawString("or Wi-Fi list first", 120, 148);
    return;
  }

  // Scroll buttons with triangles on top-right and bottom-right
  uint8_t maxOffset = (WIFI_INFO_ROW_COUNT > WIFI_INFO_VISIBLE_ROWS)
                        ? (uint8_t)(WIFI_INFO_ROW_COUNT - WIFI_INFO_VISIBLE_ROWS)
                        : 0;
  if (wifiInfoScrollOffset > maxOffset) wifiInfoScrollOffset = maxOffset;

  bool canUp   = (wifiInfoScrollOffset > 0);
  bool canDown = (wifiInfoScrollOffset < maxOffset);

  canvas.fillRoundRect(199, 34, 36, 56, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(199, 34, 36, 56, 6, canUp ? COLOR_ACCENT : COLOR_HEX_BORDER);
  canvas.fillTriangle(217, 52, 208, 70, 226, 70, canUp ? COLOR_ACCENT : COLOR_SUBTEXT);

  canvas.fillRoundRect(199, 182, 36, 56, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(199, 182, 36, 56, 6, canDown ? COLOR_ACCENT : COLOR_HEX_BORDER);
  canvas.fillTriangle(217, 220, 208, 202, 226, 202, canDown ? COLOR_ACCENT : COLOR_SUBTEXT);

  char posBuf[12];
  snprintf(posBuf, sizeof(posBuf), "%u/%u", (unsigned)(wifiInfoScrollOffset + 1), (unsigned)WIFI_INFO_ROW_COUNT);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  canvas.drawString(posBuf, 217, 136);

  for (uint8_t r = 0; r < WIFI_INFO_VISIBLE_ROWS; r++) {
    uint8_t idx = (uint8_t)(wifiInfoScrollOffset + r);
    if (idx >= WIFI_INFO_ROW_COUNT) break;

    int16_t ry = (int16_t)(34 + r * 41);
    canvas.fillRoundRect(6, ry, 188, 37, 6, COLOR_TOPBAR);
    canvas.drawRoundRect(6, ry, 188, 37, 6, COLOR_HEX_BORDER);

    // Parameter label on top
    canvas.setTextDatum(top_left);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
    canvas.drawString(wifiInfoRows[idx].label, 12, ry + 4);

    // Parameter value below
    uint16_t valCol = COLOR_TEXT;
    if (idx == 0) valCol = COLOR_ACCENT;
    else if (idx == 2) valCol = wifiInfoInternetOk ? COLOR_GREEN : COLOR_RED;
    else if (idx == 4) valCol = COLOR_GREEN;
    else if (idx == 10 || idx == 12 || idx == 13) valCol = COLOR_WARN;

    char shortVal[30];
    size_t vlen = strlen(wifiInfoRows[idx].value);
    if (vlen <= 20) {
      strncpy(shortVal, wifiInfoRows[idx].value, 20);
      shortVal[20] = '\0';
      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(valCol, COLOR_TOPBAR);
      canvas.drawString(shortVal, 12, ry + 16);
    } else {
      strncpy(shortVal, wifiInfoRows[idx].value, 29);
      shortVal[29] = '\0';
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(valCol, COLOR_TOPBAR);
      canvas.drawString(shortVal, 12, ry + 20);
    }
  }
}

// Fast iterative FFT (Radix-2 Cooley-Tukey, zero heap allocation)
void computeRadix2Fft(float* re, float* im, uint16_t n) {
  uint16_t j = 0;
  for (uint16_t i = 0; i < n; i++) {
    if (i < j) {
      float tr = re[i]; re[i] = re[j]; re[j] = tr;
      float ti = im[i]; im[i] = im[j]; im[j] = ti;
    }
    uint16_t m = (uint16_t)(n >> 1);
    while (m >= 1 && j >= m) {
      j = (uint16_t)(j - m);
      m = (uint16_t)(m >> 1);
    }
    j = (uint16_t)(j + m);
  }

  for (uint16_t len = 2; len <= n; len = (uint16_t)(len << 1)) {
    float ang = -6.283185307f / (float)len;
    float wLenRe = cosf(ang);
    float wLenIm = sinf(ang);
    uint16_t half = (uint16_t)(len >> 1);
    for (uint16_t i = 0; i < n; i = (uint16_t)(i + len)) {
      float wRe = 1.0f;
      float wIm = 0.0f;
      for (uint16_t k = 0; k < half; k++) {
        uint16_t uIdx = (uint16_t)(i + k);
        uint16_t vIdx = (uint16_t)(uIdx + half);
        float tRe = re[vIdx] * wRe - im[vIdx] * wIm;
        float tIm = re[vIdx] * wIm + im[vIdx] * wRe;
        re[vIdx] = re[uIdx] - tRe;
        im[vIdx] = im[uIdx] - tIm;
        re[uIdx] += tRe;
        im[uIdx] += tIm;
        float nextWRe = wRe * wLenRe - wIm * wLenIm;
        wIm = wRe * wLenIm + wIm * wLenRe;
        wRe = nextWRe;
      }
    }
  }
}

void stopSoundGenerator();

// Start PDM microphone SPM1423HM4H-B (GPIO44 CLK, GPIO47 DATA) for Sound spectrometer
void startSoundSpectrometer() {
  stopSoundGenerator();
  if (sndSpecActive) return;

  memset(sndSpecLevel, 0, sizeof(sndSpecLevel));
  memset(sndSpecPeak, 0, sizeof(sndSpecPeak));
  memset(sndSpecWaterfall, 0, sizeof(sndSpecWaterfall));
  sndSpecWfHead     = 0;
  sndSpecPeakFreqHz = 0.0f;
  sndSpecPeakDb     = -90.0f;

#if ESP_IDF_VERSION_MAJOR >= 5
  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chanCfg.dma_desc_num  = 4;
  chanCfg.dma_frame_num = SND_SPEC_FFT_N;
  if (i2s_new_channel(&chanCfg, NULL, &sndSpecRxChan) != ESP_OK) {
    sndSpecRxChan = NULL;
    return;
  }

  i2s_pdm_rx_config_t pdmCfg;
  memset(&pdmCfg, 0, sizeof(pdmCfg));
  pdmCfg.clk_cfg  = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SND_SPEC_SAMPLE_RATE);
  pdmCfg.slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
  pdmCfg.gpio_cfg.clk = (gpio_num_t)MIC_PDM_CLK_PIN;
  pdmCfg.gpio_cfg.din = (gpio_num_t)MIC_PDM_DATA_PIN;
  pdmCfg.gpio_cfg.invert_flags.clk_inv = false;

  if (i2s_channel_init_pdm_rx_mode(sndSpecRxChan, &pdmCfg) != ESP_OK ||
      i2s_channel_enable(sndSpecRxChan) != ESP_OK) {
    i2s_del_channel(sndSpecRxChan);
    sndSpecRxChan = NULL;
    return;
  }
#else
  i2s_config_t i2sCfg;
  memset(&i2sCfg, 0, sizeof(i2sCfg));
  i2sCfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
  i2sCfg.sample_rate          = SND_SPEC_SAMPLE_RATE;
  i2sCfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
  i2sCfg.channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT;
  i2sCfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2sCfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
  i2sCfg.dma_buf_count        = 4;
  i2sCfg.dma_buf_len          = SND_SPEC_FFT_N;
  i2sCfg.use_apll             = false;

  i2s_pin_config_t pinCfg;
  memset(&pinCfg, 0, sizeof(pinCfg));
  pinCfg.mck_io_pin   = I2S_PIN_NO_CHANGE;
  pinCfg.bck_io_pin   = I2S_PIN_NO_CHANGE;
  pinCfg.ws_io_pin    = MIC_PDM_CLK_PIN;
  pinCfg.data_out_num = I2S_PIN_NO_CHANGE;
  pinCfg.data_in_num  = MIC_PDM_DATA_PIN;

  if (i2s_driver_install(I2S_NUM_0, &i2sCfg, 0, NULL) != ESP_OK) {
    return;
  }
  i2s_set_pin(I2S_NUM_0, &pinCfg);
  i2s_zero_dma_buffer(I2S_NUM_0);
#endif

  sndSpecActive     = true;
  lastSndSpecDrawMs = 0;
}

// Stop PDM microphone and release I2S_NUM_0
void stopSoundSpectrometer() {
  if (!sndSpecActive) return;
  sndSpecActive = false;

#if ESP_IDF_VERSION_MAJOR >= 5
  if (sndSpecRxChan != NULL) {
    i2s_channel_disable(sndSpecRxChan);
    i2s_del_channel(sndSpecRxChan);
    sndSpecRxChan = NULL;
  }
#else
  i2s_driver_uninstall(I2S_NUM_0);
#endif
}

// Read PDM mic frame, Hann-windowed FFT, log frequency binning, and waterfall shift
void updateSoundSpectrometer() {
  if (!sndSpecActive) return;

  size_t bytesRead = 0;
#if ESP_IDF_VERSION_MAJOR >= 5
  if (sndSpecRxChan == NULL) return;
  if (i2s_channel_read(sndSpecRxChan, sndSpecRawSamples, sizeof(sndSpecRawSamples), &bytesRead, 40) != ESP_OK) {
    return;
  }
#else
  if (i2s_read(I2S_NUM_0, sndSpecRawSamples, sizeof(sndSpecRawSamples), &bytesRead, pdMS_TO_TICKS(40)) != ESP_OK) {
    return;
  }
#endif
  if (bytesRead < sizeof(sndSpecRawSamples)) return;

  // Remove DC offset
  float mean = 0.0f;
  for (uint16_t i = 0; i < SND_SPEC_FFT_N; i++) {
    mean += (float)sndSpecRawSamples[i];
  }
  mean /= (float)SND_SPEC_FFT_N;

  // Apply Hann window and normalize to [-1..1]
  for (uint16_t i = 0; i < SND_SPEC_FFT_N; i++) {
    float win = 0.5f * (1.0f - cosf(6.283185307f * (float)i / (float)(SND_SPEC_FFT_N - 1)));
    sndSpecRe[i] = (((float)sndSpecRawSamples[i] - mean) / 32768.0f) * win;
    sndSpecIm[i] = 0.0f;
  }

  computeRadix2Fft(sndSpecRe, sndSpecIm, SND_SPEC_FFT_N);

  const uint16_t halfN = SND_SPEC_FFT_N / 2;
  float binDb[SND_SPEC_FFT_N / 2];
  float bestMag = 0.0f;
  uint16_t bestBin = 1;

  for (uint16_t b = 1; b < halfN; b++) {
    float mag = sqrtf(sndSpecRe[b] * sndSpecRe[b] + sndSpecIm[b] * sndSpecIm[b]) / ((float)halfN * 0.5f);
    if (mag > bestMag) {
      bestMag = mag;
      bestBin = b;
    }
    float db = 20.0f * log10f(mag + 1e-5f);
    if (db < -90.0f) db = -90.0f;
    if (db > 0.0f)   db = 0.0f;
    binDb[b] = db;
  }
  binDb[0] = -90.0f;

  // Refine peak frequency using parabolic interpolation
  float peakBinF = (float)bestBin;
  if (bestBin > 1 && bestBin + 1 < halfN) {
    float a = binDb[bestBin - 1];
    float b = binDb[bestBin];
    float c = binDb[bestBin + 1];
    float denom = (a - 2.0f * b + c);
    if (fabsf(denom) > 1e-4f) {
      float delta = 0.5f * (a - c) / denom;
      if (delta > -0.5f && delta < 0.5f) peakBinF += delta;
    }
  }
  sndSpecPeakFreqHz = peakBinF * ((float)SND_SPEC_SAMPLE_RATE / (float)SND_SPEC_FFT_N);
  sndSpecPeakDb     = binDb[bestBin];

  // Logarithmic mapping of 62.5 Hz .. 8000 Hz onto 112 columns (Spectroid style)
  const float fMin = 62.5f;
  const float fMax = (float)SND_SPEC_SAMPLE_RATE * 0.5f;
  const float logRatio = logf(fMax / fMin);
  const float binHz = (float)SND_SPEC_SAMPLE_RATE / (float)SND_SPEC_FFT_N;

  sndSpecWfHead = (uint8_t)((sndSpecWfHead + SND_SPEC_WATERFALL_H - 1) % SND_SPEC_WATERFALL_H);

  for (uint16_t col = 0; col < SND_SPEC_BANDS; col++) {
    float f0 = fMin * expf(logRatio * (float)col / (float)SND_SPEC_BANDS);
    float f1 = fMin * expf(logRatio * (float)(col + 1) / (float)SND_SPEC_BANDS);
    int16_t b0 = (int16_t)floorf(f0 / binHz);
    int16_t b1 = (int16_t)ceilf(f1 / binHz);
    if (b0 < 1) b0 = 1;
    if (b1 < b0) b1 = b0;
    if (b1 >= (int16_t)halfN) b1 = (int16_t)(halfN - 1);

    float maxDb = -90.0f;
    for (int16_t b = b0; b <= b1; b++) {
      if (binDb[b] > maxDb) maxDb = binDb[b];
    }

    // Normalize -82..0 dB range to 0..100
    float norm = (maxDb + 82.0f) / 82.0f;
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;
    uint8_t targetLvl = (uint8_t)(norm * 100.0f + 0.5f);

    // Smooth spectrum envelope (fast attack, gentle decay)
    uint8_t prevLvl = sndSpecLevel[col];
    uint8_t smoothLvl = (targetLvl >= prevLvl)
                          ? (uint8_t)((prevLvl + targetLvl * 3U) / 4U)
                          : (uint8_t)((prevLvl * 3U + targetLvl) / 4U);
    sndSpecLevel[col] = smoothLvl;

    // Peak Hold decay curve (Spectroid style)
    if (smoothLvl >= sndSpecPeak[col]) {
      sndSpecPeak[col] = smoothLvl;
    } else if (sndSpecPeak[col] > 1) {
      sndSpecPeak[col] = (uint8_t)(sndSpecPeak[col] - 1);
    } else {
      sndSpecPeak[col] = 0;
    }

    uint8_t lutIdx = (uint8_t)(((uint16_t)smoothLvl * 31U) / 100U);
    if (lutIdx > 31) lutIdx = 31;
    sndSpecWaterfall[sndSpecWfHead][col] = lutIdx;
  }
}

// Draw Sound spectrometer screen (top: spectrum + Peak Hold, freq axis, bottom: Spectroid waterfall)
void drawSoundSpectrometerScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  // Title and peak frequency readout line
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(top_left);
  canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
  canvas.drawString("Spectrometer", 8, 31);

  char peakBuf[28];
  if (sndSpecPeakDb > -80.0f) {
    snprintf(peakBuf, sizeof(peakBuf), "%uHz %ddB",
             (unsigned)(sndSpecPeakFreqHz + 0.5f), (int)roundf(sndSpecPeakDb));
  } else {
    snprintf(peakBuf, sizeof(peakBuf), "Mic PDM 16k");
  }
  canvas.setTextDatum(top_right);
  canvas.setTextColor(COLOR_WARN, COLOR_BG);
  canvas.drawString(peakBuf, 232, 31);

  // Top spectrum plot area (x = 8..232, y = 43..111, width 224 px = 112 bands * 2 px)
  const int16_t plotX = 8;
  const int16_t plotY = 43;
  const int16_t plotW = SND_SPEC_BANDS * 2;
  const int16_t plotH = 68;
  canvas.fillRect(plotX, plotY, plotW, plotH, 0x0002);
  canvas.drawRect(plotX - 1, plotY - 1, plotW + 2, plotH + 2, COLOR_HEX_BORDER);

  // Horizontal dB grid lines (-20, -40, -60)
  for (uint8_t g = 1; g <= 3; g++) {
    int16_t gy = (int16_t)(plotY + (plotH * g) / 4);
    canvas.drawFastHLine(plotX, gy, plotW, 0x18E4);
  }

  // Vertical grid lines and labels for log frequency axis (100, 300, 1k, 3k, 8k Hz)
  const float fMin = 62.5f;
  const float fMax = (float)SND_SPEC_SAMPLE_RATE * 0.5f;
  const float logRatio = logf(fMax / fMin);
  const float markFreqs[5] = { 100.0f, 300.0f, 1000.0f, 3000.0f, 8000.0f };
  const char* markLabels[5] = { "100", "300", "1k", "3k", "8k" };

  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  for (uint8_t m = 0; m < 5; m++) {
    float frac = logf(markFreqs[m] / fMin) / logRatio;
    int16_t mx = (int16_t)(plotX + (int16_t)(frac * (float)(plotW - 1)));
    canvas.drawFastVLine(mx, plotY, plotH, 0x18E4);
    canvas.setTextDatum(m == 0 ? top_left : (m == 4 ? top_right : top_center));
    canvas.drawString(markLabels[m], mx, 114);
  }

  // Draw spectral bars and Peak Hold polyline
  int16_t prevPeakY = (int16_t)(plotY + plotH - 1);
  for (uint16_t col = 0; col < SND_SPEC_BANDS; col++) {
    int16_t bx = (int16_t)(plotX + col * 2);
    int16_t barH = (int16_t)(((uint16_t)sndSpecLevel[col] * (uint16_t)(plotH - 2)) / 100U);
    if (barH > 0) {
      uint8_t lutIdx = (uint8_t)(((uint16_t)sndSpecLevel[col] * 31U) / 100U);
      if (lutIdx > 31) lutIdx = 31;
      int16_t barY = (int16_t)(plotY + plotH - barH);
      canvas.fillRect(bx, barY, 2, barH, SPEC_COLOR_LUT[lutIdx]);
      canvas.drawFastHLine(bx, barY, 2, COLOR_ACCENT);
    }

    int16_t peakH = (int16_t)(((uint16_t)sndSpecPeak[col] * (uint16_t)(plotH - 2)) / 100U);
    int16_t peakY = (int16_t)(plotY + plotH - 1 - peakH);
    if (col > 0) {
      canvas.drawLine(bx - 2, prevPeakY, bx, peakY, COLOR_WARN);
    }
    prevPeakY = peakY;
  }

  // Bottom waterfall area (Spectrogram Waterfall, x = 8..232, y = 125..235, 55 rows * 2 px)
  const int16_t wfY = 125;
  const int16_t wfH = SND_SPEC_WATERFALL_H * 2;
  canvas.drawRect(plotX - 1, wfY - 1, plotW + 2, wfH + 2, COLOR_HEX_BORDER);
  for (uint8_t row = 0; row < SND_SPEC_WATERFALL_H; row++) {
    uint8_t ringIdx = (uint8_t)((sndSpecWfHead + row) % SND_SPEC_WATERFALL_H);
    int16_t ry = (int16_t)(wfY + row * 2);
    for (uint16_t col = 0; col < SND_SPEC_BANDS; col++) {
      uint8_t cIdx = sndSpecWaterfall[ringIdx][col];
      if (cIdx > 31) cIdx = 31;
      canvas.fillRect(plotX + col * 2, ry, 2, 2, SPEC_COLOR_LUT[cIdx]);
    }
  }
}

// FreeRTOS task (Core 0) for glitch-free continuous audio synthesis via MAX98357A I2S
void soundGeneratorAudioTask(void* arg) {
  (void)arg;
  int16_t stereoBuf[SND_GEN_CHUNK_SAMPLES * 2];
  float phase = 0.0f;
  uint32_t rng = 0x12345678UL;
  // Paul Kellet 7-pole pink noise filter state (-3 dB/octave)
  float b0 = 0.0f, b1 = 0.0f, b2 = 0.0f, b3 = 0.0f, b4 = 0.0f, b5 = 0.0f, b6 = 0.0f;

  while (!sndGenTaskStopReq) {
    uint16_t freq = sndGenFreqHz;
    if (freq < SND_GEN_FREQ_MIN) freq = SND_GEN_FREQ_MIN;
    if (freq > SND_GEN_FREQ_MAX) freq = SND_GEN_FREQ_MAX;
    uint8_t volPct = sndGenVolumePct;
    if (volPct > 100) volPct = 100;
    SoundWaveType wave = sndGenWave;

    // Safe speaker volume scaling (prevents clipping)
    float gain = ((float)volPct / 100.0f);
    gain = gain * gain * 18000.0f;
    float phaseInc = (float)freq / (float)SND_GEN_SAMPLE_RATE;

    for (uint16_t i = 0; i < SND_GEN_CHUNK_SAMPLES; i++) {
      float sampleF = 0.0f;
      switch (wave) {
        case SND_WAVE_SINE:
          sampleF = sinf(phase * 6.283185307f);
          break;
        case SND_WAVE_SQUARE:
          sampleF = (phase < 0.5f) ? 1.0f : -1.0f;
          break;
        case SND_WAVE_TRIANGLE:
          sampleF = 1.0f - 4.0f * fabsf(phase - 0.5f);
          break;
        case SND_WAVE_SAWTOOTH:
          sampleF = 2.0f * phase - 1.0f;
          break;
        case SND_WAVE_WHITE: {
          rng ^= (rng << 13);
          rng ^= (rng >> 17);
          rng ^= (rng << 5);
          sampleF = ((float)(int32_t)rng) * (1.0f / 2147483648.0f);
          break;
        }
        case SND_WAVE_PINK: {
          rng ^= (rng << 13);
          rng ^= (rng >> 17);
          rng ^= (rng << 5);
          float white = ((float)(int32_t)rng) * (1.0f / 2147483648.0f);
          b0 = 0.99886f * b0 + white * 0.0555179f;
          b1 = 0.99332f * b1 + white * 0.0750759f;
          b2 = 0.96900f * b2 + white * 0.1538520f;
          b3 = 0.86650f * b3 + white * 0.3104856f;
          b4 = 0.55000f * b4 + white * 0.5329522f;
          b5 = -0.7616f * b5 - white * 0.0168980f;
          float pink = (b0 + b1 + b2 + b3 + b4 + b5 + b6 + white * 0.5362f) * 0.14f;
          b6 = white * 0.115926f;
          if (pink > 1.0f)  pink = 1.0f;
          if (pink < -1.0f) pink = -1.0f;
          sampleF = pink;
          break;
        }
      }

      phase += phaseInc;
      if (phase >= 1.0f) phase -= 1.0f;

      int32_t s32 = (int32_t)(sampleF * gain);
      if (s32 > 32767)  s32 = 32767;
      if (s32 < -32767) s32 = -32767;
      int16_t s16 = (int16_t)s32;
      stereoBuf[i * 2]     = s16;
      stereoBuf[i * 2 + 1] = s16;
    }

    size_t written = 0;
#if ESP_IDF_VERSION_MAJOR >= 5
    if (sndGenTxChan != NULL) {
      i2s_channel_write(sndGenTxChan, stereoBuf, sizeof(stereoBuf), &written, 60);
    }
#else
    i2s_write(I2S_NUM_1, stereoBuf, sizeof(stereoBuf), &written, pdMS_TO_TICKS(60));
#endif
  }

  // Short silence frame before stop to avoid speaker pop
  memset(stereoBuf, 0, sizeof(stereoBuf));
  size_t written = 0;
#if ESP_IDF_VERSION_MAJOR >= 5
  if (sndGenTxChan != NULL) {
    i2s_channel_write(sndGenTxChan, stereoBuf, sizeof(stereoBuf), &written, 40);
  }
#else
  i2s_write(I2S_NUM_1, stereoBuf, sizeof(stereoBuf), &written, pdMS_TO_TICKS(40));
#endif

  sndGenTaskHandle = NULL;
  vTaskDelete(NULL);
}

// Start sound generation on MAX98357A I2S amplifier (BCK=48, WS=15, DOUT=46)
void startSoundGenerator() {
  stopAlertRingtone();
  stopCwToneAudio();
  stopSoundSpectrometer();
  if (sndGenPlaying) return;

  // Power ON MAX98357A speaker amplifier rail (AXP2101 DLDO1) only when audio starts
  setSpeakerAmpPower(true);
  delay(5);

#if ESP_IDF_VERSION_MAJOR >= 5
  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
  chanCfg.dma_desc_num  = 4;
  chanCfg.dma_frame_num = SND_GEN_CHUNK_SAMPLES;
  if (i2s_new_channel(&chanCfg, &sndGenTxChan, NULL) != ESP_OK) {
    sndGenTxChan = NULL;
    updateSpeakerAmpPower();
    return;
  }

  i2s_std_config_t stdCfg;
  memset(&stdCfg, 0, sizeof(stdCfg));
  stdCfg.clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SND_GEN_SAMPLE_RATE);
  stdCfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  stdCfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  stdCfg.gpio_cfg.bclk = (gpio_num_t)SPK_I2S_BCK_PIN;
  stdCfg.gpio_cfg.ws   = (gpio_num_t)SPK_I2S_WS_PIN;
  stdCfg.gpio_cfg.dout = (gpio_num_t)SPK_I2S_DOUT_PIN;
  stdCfg.gpio_cfg.din  = I2S_GPIO_UNUSED;
  stdCfg.gpio_cfg.invert_flags.mclk_inv = false;
  stdCfg.gpio_cfg.invert_flags.bclk_inv = false;
  stdCfg.gpio_cfg.invert_flags.ws_inv   = false;

  if (i2s_channel_init_std_mode(sndGenTxChan, &stdCfg) != ESP_OK ||
      i2s_channel_enable(sndGenTxChan) != ESP_OK) {
    i2s_del_channel(sndGenTxChan);
    sndGenTxChan = NULL;
    updateSpeakerAmpPower();
    return;
  }
#else
  i2s_config_t i2sCfg;
  memset(&i2sCfg, 0, sizeof(i2sCfg));
  i2sCfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  i2sCfg.sample_rate          = SND_GEN_SAMPLE_RATE;
  i2sCfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
  i2sCfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
  i2sCfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2sCfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
  i2sCfg.dma_buf_count        = 4;
  i2sCfg.dma_buf_len          = SND_GEN_CHUNK_SAMPLES;
  i2sCfg.use_apll             = false;
  i2sCfg.tx_desc_auto_clear   = true;

  i2s_pin_config_t pinCfg;
  memset(&pinCfg, 0, sizeof(pinCfg));
  pinCfg.mck_io_pin   = I2S_PIN_NO_CHANGE;
  pinCfg.bck_io_pin   = SPK_I2S_BCK_PIN;
  pinCfg.ws_io_pin    = SPK_I2S_WS_PIN;
  pinCfg.data_out_num = SPK_I2S_DOUT_PIN;
  pinCfg.data_in_num  = I2S_PIN_NO_CHANGE;

  if (i2s_driver_install(I2S_NUM_1, &i2sCfg, 0, NULL) != ESP_OK) {
    updateSpeakerAmpPower();
    return;
  }
  i2s_set_pin(I2S_NUM_1, &pinCfg);
  i2s_zero_dma_buffer(I2S_NUM_1);
#endif

  sndGenTaskStopReq = false;
  sndGenPlaying     = true;
  if (xTaskCreatePinnedToCore(soundGeneratorAudioTask, "snd_gen", 3072, NULL, 2, &sndGenTaskHandle, 0) != pdPASS) {
    sndGenTaskHandle = NULL;
    sndGenPlaying    = false;
#if ESP_IDF_VERSION_MAJOR >= 5
    if (sndGenTxChan != NULL) {
      i2s_channel_disable(sndGenTxChan);
      i2s_del_channel(sndGenTxChan);
      sndGenTxChan = NULL;
    }
#else
    i2s_driver_uninstall(I2S_NUM_1);
#endif
    updateSpeakerAmpPower();
  }
}

// Stop sound generation, release I2S_NUM_1, and power OFF MAX98357A DLDO1
void stopSoundGenerator() {
  if (!sndGenPlaying && sndGenTaskHandle == NULL) {
    updateSpeakerAmpPower();
    return;
  }
  sndGenTaskStopReq = true;
  sndGenPlaying     = false;

  uint32_t waitStart = millis();
  while (sndGenTaskHandle != NULL && (millis() - waitStart) < 300UL) {
    delay(2);
  }

#if ESP_IDF_VERSION_MAJOR >= 5
  if (sndGenTxChan != NULL) {
    i2s_channel_disable(sndGenTxChan);
    i2s_del_channel(sndGenTxChan);
    sndGenTxChan = NULL;
  }
#else
  i2s_driver_uninstall(I2S_NUM_1);
#endif
  updateSpeakerAmpPower();
}

// FreeRTOS task synthesizing alarm/timer ringtone (Beep or Bell) via MAX98357A I2S
void alertAudioTask(void* arg) {
  (void)arg;
  int16_t stereoBuf[SND_GEN_CHUNK_SAMPLES * 2];
  uint32_t sampleIdx = 0;

  while (!alertAudioStopReq) {
    AlertRingtone tone = activeAlertRingtone;
    for (uint16_t i = 0; i < SND_GEN_CHUNK_SAMPLES; i++) {
      float tSec = (float)sampleIdx / (float)SND_GEN_SAMPLE_RATE;
      float cycleT = fmodf(tSec, 1.0f);
      float sampleF = 0.0f;

      if (tone == RINGTONE_BEEP) {
        // Classic wristwatch beep pattern: 3 pulses of 100 ms (2400 Hz) + pause
        bool on = (cycleT < 0.10f) ||
                  (cycleT >= 0.18f && cycleT < 0.28f) ||
                  (cycleT >= 0.36f && cycleT < 0.46f);
        if (on) {
          float localT = (cycleT < 0.10f) ? cycleT : ((cycleT < 0.28f) ? (cycleT - 0.18f) : (cycleT - 0.36f));
          float env = 1.0f;
          if (localT < 0.006f) env = localT / 0.006f;
          else if (localT > 0.094f) env = (0.10f - localT) / 0.006f;
          float s = sinf(6.2831853f * 2400.0f * tSec) + 0.25f * sinf(6.2831853f * 4800.0f * tSec);
          sampleF = s * 0.72f * env;
        }
      } else {
        // Metallic alarm bell: 880 Hz + 1760/2637 Hz overtones with 14 Hz tremolo (650 ms ring + 350 ms pause)
        if (cycleT < 0.65f) {
          float env = expf(-cycleT * 2.2f);
          float clapper = 0.55f + 0.45f * sinf(6.2831853f * 14.0f * cycleT);
          float bell = 0.55f * sinf(6.2831853f * 880.0f * tSec) +
                       0.30f * sinf(6.2831853f * 1760.0f * tSec) +
                       0.15f * sinf(6.2831853f * 2637.0f * tSec);
          sampleF = bell * env * clapper;
        }
      }

      sampleIdx++;
      int32_t s32 = (int32_t)(sampleF * 15500.0f);
      if (s32 > 32767)  s32 = 32767;
      if (s32 < -32767) s32 = -32767;
      int16_t s16 = (int16_t)s32;
      stereoBuf[i * 2]     = s16;
      stereoBuf[i * 2 + 1] = s16;
    }

    size_t written = 0;
#if ESP_IDF_VERSION_MAJOR >= 5
    if (alertTxChan != NULL) {
      i2s_channel_write(alertTxChan, stereoBuf, sizeof(stereoBuf), &written, 60);
    }
#else
    i2s_write(I2S_NUM_1, stereoBuf, sizeof(stereoBuf), &written, pdMS_TO_TICKS(60));
#endif
  }

  memset(stereoBuf, 0, sizeof(stereoBuf));
  size_t written = 0;
#if ESP_IDF_VERSION_MAJOR >= 5
  if (alertTxChan != NULL) {
    i2s_channel_write(alertTxChan, stereoBuf, sizeof(stereoBuf), &written, 40);
  }
#else
  i2s_write(I2S_NUM_1, stereoBuf, sizeof(stereoBuf), &written, pdMS_TO_TICKS(40));
#endif

  alertAudioTaskHandle = NULL;
  vTaskDelete(NULL);
}

void startAlertRingtone(AlertRingtone tone) {
  stopSoundGenerator();
  stopCwToneAudio();
  stopSoundSpectrometer();
  stopAlertRingtone();

  // Power ON MAX98357A speaker amplifier rail (AXP2101 DLDO1) for Alarm/Timer ringtone
  setSpeakerAmpPower(true);
  delay(5);

  activeAlertRingtone = tone;
#if ESP_IDF_VERSION_MAJOR >= 5
  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
  chanCfg.dma_desc_num  = 4;
  chanCfg.dma_frame_num = SND_GEN_CHUNK_SAMPLES;
  if (i2s_new_channel(&chanCfg, &alertTxChan, NULL) != ESP_OK) {
    alertTxChan = NULL;
    updateSpeakerAmpPower();
    return;
  }

  i2s_std_config_t stdCfg;
  memset(&stdCfg, 0, sizeof(stdCfg));
  stdCfg.clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SND_GEN_SAMPLE_RATE);
  stdCfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  stdCfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  stdCfg.gpio_cfg.bclk = (gpio_num_t)SPK_I2S_BCK_PIN;
  stdCfg.gpio_cfg.ws   = (gpio_num_t)SPK_I2S_WS_PIN;
  stdCfg.gpio_cfg.dout = (gpio_num_t)SPK_I2S_DOUT_PIN;
  stdCfg.gpio_cfg.din  = I2S_GPIO_UNUSED;

  if (i2s_channel_init_std_mode(alertTxChan, &stdCfg) != ESP_OK ||
      i2s_channel_enable(alertTxChan) != ESP_OK) {
    i2s_del_channel(alertTxChan);
    alertTxChan = NULL;
    updateSpeakerAmpPower();
    return;
  }
#else
  i2s_config_t i2sCfg;
  memset(&i2sCfg, 0, sizeof(i2sCfg));
  i2sCfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  i2sCfg.sample_rate          = SND_GEN_SAMPLE_RATE;
  i2sCfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
  i2sCfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
  i2sCfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2sCfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
  i2sCfg.dma_buf_count        = 4;
  i2sCfg.dma_buf_len          = SND_GEN_CHUNK_SAMPLES;
  i2sCfg.use_apll             = false;
  i2sCfg.tx_desc_auto_clear   = true;

  i2s_pin_config_t pinCfg;
  memset(&pinCfg, 0, sizeof(pinCfg));
  pinCfg.mck_io_pin   = I2S_PIN_NO_CHANGE;
  pinCfg.bck_io_pin   = SPK_I2S_BCK_PIN;
  pinCfg.ws_io_pin    = SPK_I2S_WS_PIN;
  pinCfg.data_out_num = SPK_I2S_DOUT_PIN;
  pinCfg.data_in_num  = I2S_PIN_NO_CHANGE;

  if (i2s_driver_install(I2S_NUM_1, &i2sCfg, 0, NULL) != ESP_OK) {
    updateSpeakerAmpPower();
    return;
  }
  i2s_set_pin(I2S_NUM_1, &pinCfg);
  i2s_zero_dma_buffer(I2S_NUM_1);
#endif

  alertAudioStopReq = false;
  alertAudioPlaying = true;
  if (xTaskCreatePinnedToCore(alertAudioTask, "alert_snd", 3072, NULL, 2, &alertAudioTaskHandle, 0) != pdPASS) {
    alertAudioTaskHandle = NULL;
    alertAudioPlaying    = false;
#if ESP_IDF_VERSION_MAJOR >= 5
    if (alertTxChan != NULL) {
      i2s_channel_disable(alertTxChan);
      i2s_del_channel(alertTxChan);
      alertTxChan = NULL;
    }
#else
    i2s_driver_uninstall(I2S_NUM_1);
#endif
    updateSpeakerAmpPower();
  }
}

void stopAlertRingtone() {
  if (!alertAudioPlaying && alertAudioTaskHandle == NULL) {
    updateSpeakerAmpPower();
    return;
  }
  alertAudioStopReq = true;
  alertAudioPlaying = false;

  uint32_t waitStart = millis();
  while (alertAudioTaskHandle != NULL && (millis() - waitStart) < 300UL) {
    delay(2);
  }

#if ESP_IDF_VERSION_MAJOR >= 5
  if (alertTxChan != NULL) {
    i2s_channel_disable(alertTxChan);
    i2s_del_channel(alertTxChan);
    alertTxChan = NULL;
  }
#else
  i2s_driver_uninstall(I2S_NUM_1);
#endif
  updateSpeakerAmpPower();
}

// Wake display, start ringtone, and trigger vibration when alarm or timer fires
void wakeDisplayForAlert(AlertRingtone tone) {
  uint32_t now = millis();
  if (touchInMonitorMode) {
    setTouchActiveMode();
  }
  wakeDisplayFromSleep();
  blState          = BL_STATE_FULL;
  lastTouchMs      = now;
  waitingSecondTap = false;
  alertStartedMs   = now;
  lastAlertAnimMs  = now;
  lastAlertVibMs   = now;
  triggerTouchVibration();
  startAlertRingtone(tone);
  renderCurrentScreen();
  tft.setBrightness(getFullBrightnessPwm());
}

// Miniature waveform icon on Sound generator buttons
void drawWaveMiniIcon(SoundWaveType wave, int16_t x0, int16_t yMid, uint16_t col) {
  const int16_t w = 22;
  const int16_t amp = 6;
  switch (wave) {
    case SND_WAVE_SINE:
      for (int16_t i = 0; i < w - 1; i++) {
        int16_t y1 = (int16_t)(yMid - (int16_t)(sinf((float)i * 6.2831853f / (float)w) * (float)amp));
        int16_t y2 = (int16_t)(yMid - (int16_t)(sinf((float)(i + 1) * 6.2831853f / (float)w) * (float)amp));
        canvas.drawLine(x0 + i, y1, x0 + i + 1, y2, col);
      }
      break;
    case SND_WAVE_SQUARE:
      canvas.drawLine(x0, yMid, x0, yMid - amp, col);
      canvas.drawLine(x0, yMid - amp, x0 + 11, yMid - amp, col);
      canvas.drawLine(x0 + 11, yMid - amp, x0 + 11, yMid + amp, col);
      canvas.drawLine(x0 + 11, yMid + amp, x0 + w, yMid + amp, col);
      canvas.drawLine(x0 + w, yMid + amp, x0 + w, yMid, col);
      break;
    case SND_WAVE_TRIANGLE:
      canvas.drawLine(x0, yMid, x0 + 5, yMid - amp, col);
      canvas.drawLine(x0 + 5, yMid - amp, x0 + 16, yMid + amp, col);
      canvas.drawLine(x0 + 16, yMid + amp, x0 + w, yMid, col);
      break;
    case SND_WAVE_SAWTOOTH:
      canvas.drawLine(x0, yMid + amp, x0 + 11, yMid - amp, col);
      canvas.drawLine(x0 + 11, yMid - amp, x0 + 11, yMid + amp, col);
      canvas.drawLine(x0 + 11, yMid + amp, x0 + w, yMid - amp, col);
      canvas.drawLine(x0 + w, yMid - amp, x0 + w, yMid + amp, col);
      break;
    case SND_WAVE_WHITE: {
      static const int8_t wn[9] = { 0, -6, 5, -4, 6, -6, 4, -5, 1 };
      for (int16_t i = 0; i < 8; i++) {
        canvas.drawLine(x0 + i * 3, yMid + wn[i], x0 + (i + 1) * 3, yMid + wn[i + 1], col);
      }
      break;
    }
    case SND_WAVE_PINK: {
      static const int8_t pn[9] = { 2, -2, -5, -1, 3, 5, 2, -3, 0 };
      for (int16_t i = 0; i < 8; i++) {
        canvas.drawLine(x0 + i * 3, yMid + pn[i], x0 + (i + 1) * 3, yMid + pn[i + 1], col);
      }
      break;
    }
  }
}

// Draw Sound generator screen (frequency & volume SpinBoxes, 6 waveforms, ON/OFF, X)
void drawSoundGeneratorScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  // 1. Frequency SpinBox (y = 32..70): "-" / center value+step / "+" buttons
  canvas.fillRoundRect(8, 33, 40, 36, 7, COLOR_HEX_FILL);
  canvas.drawRoundRect(8, 33, 40, 36, 7, COLOR_HEX_BORDER);
  canvas.fillRoundRect(52, 33, 136, 36, 7, COLOR_TOPBAR);
  canvas.drawRoundRect(52, 33, 136, 36, 7, COLOR_ACCENT);
  canvas.fillRoundRect(192, 33, 40, 36, 7, COLOR_HEX_FILL);
  canvas.drawRoundRect(192, 33, 40, 36, 7, COLOR_HEX_BORDER);

  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font4);
  canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
  canvas.drawString("-", 28, 51);
  canvas.drawString("+", 212, 51);

  char freqStepBuf[24];
  snprintf(freqStepBuf, sizeof(freqStepBuf), "FREQ  +/-%u", (unsigned)SND_FREQ_STEPS[sndGenFreqStepIdx]);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
  canvas.drawString(freqStepBuf, 120, 41);

  char freqValBuf[20];
  snprintf(freqValBuf, sizeof(freqValBuf), "%u Hz", (unsigned)sndGenFreqHz);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_ACCENT, COLOR_TOPBAR);
  canvas.drawString(freqValBuf, 120, 57);

  // 2. Volume SpinBox (y = 74..110): "-" / value + level bar / "+" buttons
  canvas.fillRoundRect(8, 75, 40, 34, 7, COLOR_HEX_FILL);
  canvas.drawRoundRect(8, 75, 40, 34, 7, COLOR_HEX_BORDER);
  canvas.fillRoundRect(52, 75, 136, 34, 7, COLOR_TOPBAR);
  canvas.drawRoundRect(52, 75, 136, 34, 7, COLOR_HEX_BORDER);
  canvas.fillRoundRect(192, 75, 40, 34, 7, COLOR_HEX_FILL);
  canvas.drawRoundRect(192, 75, 40, 34, 7, COLOR_HEX_BORDER);

  canvas.setFont(&fonts::Font4);
  canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
  canvas.drawString("-", 28, 92);
  canvas.drawString("+", 212, 92);

  char volBuf[20];
  snprintf(volBuf, sizeof(volBuf), "VOL  %u%%", (unsigned)sndGenVolumePct);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_TEXT, COLOR_TOPBAR);
  canvas.drawString(volBuf, 120, 87);

  // Volume level bar inside SpinBox
  int16_t vTrackX = 62, vTrackY = 99, vTrackW = 116, vTrackH = 5;
  canvas.fillRoundRect(vTrackX, vTrackY, vTrackW, vTrackH, 2, COLOR_BG);
  int16_t vFillW = (int16_t)(((uint16_t)sndGenVolumePct * (uint16_t)vTrackW) / 100U);
  if (vFillW > 0) {
    canvas.fillRoundRect(vTrackX, vTrackY, vFillW, vTrackH, 2, COLOR_GREEN);
  }

  // 3. 6 waveform selection buttons (2 rows * 3 cols: Sine, Square, Tri / Saw, White, Pink)
  for (uint8_t idx = 0; idx < 6; idx++) {
    uint8_t row = (uint8_t)(idx / 3);
    uint8_t col = (uint8_t)(idx % 3);
    int16_t bx = (int16_t)(8 + col * 76);
    int16_t by = (int16_t)(116 + row * 35);
    bool sel = ((uint8_t)sndGenWave == idx);
    uint16_t bgCol   = sel ? COLOR_HEX_ACTIVE : COLOR_HEX_FILL;
    uint16_t bordCol = sel ? COLOR_ACCENT     : COLOR_HEX_BORDER;
    uint16_t txtCol  = sel ? COLOR_ACCENT     : COLOR_TEXT;

    canvas.fillRoundRect(bx, by, 72, 31, 6, bgCol);
    canvas.drawRoundRect(bx, by, 72, 31, 6, bordCol);

    uint16_t iconCol = (idx == 5 && sel) ? COLOR_RED : (sel ? COLOR_ACCENT : COLOR_SUBTEXT);
    drawWaveMiniIcon((SoundWaveType)idx, (int16_t)(bx + 25), (int16_t)(by + 10), iconCol);

    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(txtCol, bgCol);
    canvas.drawString(SND_WAVE_LABELS[idx], bx + 36, by + 23);
  }

  // 4. Sound ON/OFF toggle button (y = 190..234)
  uint16_t pwrFill = sndGenPlaying ? 0x0B44 : COLOR_HEX_FILL;
  uint16_t pwrBord = sndGenPlaying ? COLOR_GREEN : COLOR_HEX_BORDER;
  uint16_t pwrTxt  = sndGenPlaying ? COLOR_GREEN : COLOR_TEXT;
  canvas.fillRoundRect(8, 190, 224, 44, 10, pwrFill);
  canvas.drawRoundRect(8, 190, 224, 44, 10, pwrBord);

  // Speaker icon on left side of ON/OFF button
  int16_t sx = 46, sy = 212;
  canvas.fillRect(sx - 10, sy - 4, 5, 8, pwrTxt);
  canvas.fillTriangle(sx - 5, sy - 4, sx - 5, sy + 4, sx + 2, sy - 10, pwrTxt);
  canvas.fillTriangle(sx - 5, sy + 4, sx + 2, sy - 10, sx + 2, sy + 10, pwrTxt);
  if (sndGenPlaying) {
    canvas.drawArc(sx + 4, sy, 6, 5, 310, 50, COLOR_GREEN);
    canvas.drawArc(sx + 4, sy, 11, 10, 310, 50, COLOR_ACCENT);
  } else {
    canvas.drawLine(sx + 6, sy - 5, sx + 14, sy + 5, COLOR_RED);
    canvas.drawLine(sx + 14, sy - 5, sx + 6, sy + 5, COLOR_RED);
  }

  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(pwrTxt, pwrFill);
  canvas.drawString(sndGenPlaying ? "SOUND: ON" : "SOUND: OFF", 136, 212);
}

// ============================================================================
// 7.11. HOSTS SCAN (SUBNET ARP SCANNER) AND BLUETOOTH SCAN (BLE 5.0)
// ============================================================================

// Rescan button in top-right corner of Hosts scan and Bluetooth scan screens
void drawTopRightRescanIcon() {
  canvas.fillRoundRect(204, 2, 32, 24, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(204, 2, 32, 24, 6, COLOR_ACCENT);
  canvas.drawArc(220, 14, 7, 5, 35, 315, COLOR_ACCENT);
  canvas.fillTriangle(225, 7, 227, 13, 221, 11, COLOR_ACCENT);
}

// Quick NetBIOS NBSTAT query (UDP 137) to resolve host network name
bool queryNetbiosHostName(const uint8_t ip[4], char* outName, size_t outLen) {
  if (outLen < 2) return false;
  outName[0] = '\0';

  static const uint8_t nbQuery[50] = {
    0xA2, 0x48, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x20, 0x43, 0x4B, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41,
    0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41,
    0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x00, 0x00, 0x21,
    0x00, 0x01
  };

  WiFiUDP udp;
  if (!udp.begin(0)) return false;

  IPAddress target(ip[0], ip[1], ip[2], ip[3]);
  udp.beginPacket(target, 137);
  udp.write(nbQuery, sizeof(nbQuery));
  udp.endPacket();

  uint32_t t0 = millis();
  while ((millis() - t0) < 90UL) {
    int pktLen = udp.parsePacket();
    if (pktLen >= 75) {
      uint8_t buf[128];
      int r = udp.read(buf, (pktLen < (int)sizeof(buf)) ? pktLen : (int)sizeof(buf));
      if (r >= 75 && buf[56] > 0) {
        size_t pos = 0;
        for (int i = 0; i < 15 && pos + 1 < outLen; i++) {
          char ch = (char)buf[57 + i];
          if (ch >= 33 && ch <= 126) {
            outName[pos++] = ch;
          } else if (ch == ' ' && pos > 0) {
            break;
          }
        }
        outName[pos] = '\0';
        udp.stop();
        return (pos > 0);
      }
    }
    delay(2);
  }
  udp.stop();
  return false;
}

// Add discovered subnet host (with duplicate guard and OUI vendor lookup)
void addLanHostEntry(const uint8_t ip[4], const uint8_t mac[6], bool isGw, bool isSelf, int16_t rttMs) {
  if (lanHostCount >= HOSTS_SCAN_MAX) return;
  for (int16_t i = 0; i < lanHostCount; i++) {
    if (memcmp(lanHosts[i].ip, ip, 4) == 0) return;
  }

  LanHostEntry& h = lanHosts[lanHostCount];
  memset(&h, 0, sizeof(LanHostEntry));
  memcpy(h.ip, ip, 4);
  memcpy(h.mac, mac, 6);
  snprintf(h.ipStr, sizeof(h.ipStr), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  snprintf(h.macStr, sizeof(h.macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  strncpy(h.vendor, lookupOuiVendor(mac), sizeof(h.vendor) - 1);
  h.isGateway     = isGw;
  h.isSelf        = isSelf;
  h.rttMs         = rttMs;
  h.detailsProbed = isSelf;
  strncpy(h.openPorts, isSelf ? "Local Watch" : "Not probed", sizeof(h.openPorts) - 1);

  if (isSelf) {
    strncpy(h.name, "T-Watch S3 (Self)", sizeof(h.name) - 1);
    strncpy(h.role, "This Watch", sizeof(h.role) - 1);
  } else if (isGw) {
    if (strcmp(h.vendor, "Unknown") != 0) {
      snprintf(h.name, sizeof(h.name), "%s Router", h.vendor);
    } else {
      strncpy(h.name, "Gateway Router", sizeof(h.name) - 1);
    }
    strncpy(h.role, "Gateway", sizeof(h.role) - 1);
  } else {
    if (strcmp(h.vendor, "Unknown") != 0) {
      snprintf(h.name, sizeof(h.name), "%s-%u", h.vendor, (unsigned)ip[3]);
    } else {
      snprintf(h.name, sizeof(h.name), "Host-%u", (unsigned)ip[3]);
    }
    strncpy(h.role, "LAN Host", sizeof(h.role) - 1);
  }
  lanHostCount++;
}

// Sort hosts by last IP octet (gateway and own watch pinned at top)
void sortLanHosts() {
  for (int16_t i = 0; i < lanHostCount - 1; i++) {
    for (int16_t j = i + 1; j < lanHostCount; j++) {
      uint16_t keyI = lanHosts[i].isGateway ? 0 : (lanHosts[i].isSelf ? 1 : (uint16_t)(10 + lanHosts[i].ip[3]));
      uint16_t keyJ = lanHosts[j].isGateway ? 0 : (lanHosts[j].isSelf ? 1 : (uint16_t)(10 + lanHosts[j].ip[3]));
      if (keyJ < keyI) {
        LanHostEntry tmp = lanHosts[i];
        lanHosts[i] = lanHosts[j];
        lanHosts[j] = tmp;
      }
    }
  }
}

// Stop active host scan
void stopHostsScan() {
  hostsScanRunning    = false;
  hostsScanWaitingArp = false;
}

// Start non-blocking ARP scan of current Wi-Fi subnet
void startHostsScan() {
  stopHostsScan();
  lanHostCount      = 0;
  hostsScrollOffset = 0;
  hostsSelectedIdx  = -1;

  if (WiFi.status() != WL_CONNECTED) {
    hostsView = HOSTS_VIEW_NO_WIFI;
    hostsSubnetLabel[0] = '\0';
    return;
  }

  IPAddress localIp = WiFi.localIP();
  IPAddress gwIp    = WiFi.gatewayIP();
  hostsScanNetPrefix[0] = localIp[0];
  hostsScanNetPrefix[1] = localIp[1];
  hostsScanNetPrefix[2] = localIp[2];
  snprintf(hostsSubnetLabel, sizeof(hostsSubnetLabel), "%u.%u.%u.0/24",
           localIp[0], localIp[1], localIp[2]);

  // Add our own watch to the discovered hosts list
  uint8_t selfIp[4]  = { localIp[0], localIp[1], localIp[2], localIp[3] };
  uint8_t selfMac[6] = { 0 };
  WiFi.macAddress(selfMac);
  addLanHostEntry(selfIp, selfMac, false, true, 0);

  // Try reading gateway immediately from current lwIP ARP table
  if (gwIp[0] != 0) {
    ip4_addr_t gwAddr;
    IP4_ADDR(&gwAddr, gwIp[0], gwIp[1], gwIp[2], gwIp[3]);
    struct eth_addr* ethRet = NULL;
    const ip4_addr_t* ipRet = NULL;
    int8_t found = -1;
    uint8_t gwMac[6] = {0};
#if defined(LWIP_TCPIP_CORE_LOCKING) && LWIP_TCPIP_CORE_LOCKING
    LOCK_TCPIP_CORE();
#endif
    if (netif_default != NULL && netif_default->hwaddr_len == 6) {
      found = etharp_find_addr(netif_default, &gwAddr, &ethRet, &ipRet);
      if (found >= 0 && ethRet != NULL) {
        memcpy(gwMac, ethRet->addr, 6);
      }
    }
#if defined(LWIP_TCPIP_CORE_LOCKING) && LWIP_TCPIP_CORE_LOCKING
    UNLOCK_TCPIP_CORE();
#endif
    if (found >= 0) {
      uint8_t gwBytes[4] = { gwIp[0], gwIp[1], gwIp[2], gwIp[3] };
      addLanHostEntry(gwBytes, gwMac, true, false, 2);
    }
  }

  hostsScanNextHost   = 1;
  hostsScanBatchStart = 1;
  hostsScanBatchCount = 0;
  hostsScanWaitingArp = false;
  hostsScanRunning    = true;
  hostsView           = HOSTS_VIEW_SCANNING;
}

// Incremental subnet ARP scan step (batches of 6 IPs to avoid overflowing lwIP ARP table)
void updateHostsScan() {
  if (!hostsScanRunning || currentScreen != SCREEN_HOSTS_SCAN) return;

  if (WiFi.status() != WL_CONNECTED) {
    stopHostsScan();
    hostsView = HOSTS_VIEW_NO_WIFI;
    renderCurrentScreen();
    return;
  }

  uint32_t now = millis();
  IPAddress localIp = WiFi.localIP();
  IPAddress gwIp    = WiFi.gatewayIP();

  if (!hostsScanWaitingArp) {
    if (hostsScanNextHost > 254) {
      sortLanHosts();
      hostsScanRunning = false;
      hostsView = HOSTS_VIEW_LIST;
      renderCurrentScreen();
      return;
    }

    hostsScanBatchStart = hostsScanNextHost;
    hostsScanBatchCount = 0;

#if defined(LWIP_TCPIP_CORE_LOCKING) && LWIP_TCPIP_CORE_LOCKING
    LOCK_TCPIP_CORE();
#endif
    if (netif_default != NULL && netif_default->hwaddr_len == 6) {
      while (hostsScanNextHost <= 254 && hostsScanBatchCount < HOSTS_ARP_BATCH_SIZE) {
        uint8_t hOct = (uint8_t)hostsScanNextHost;
        hostsScanNextHost++;
        hostsScanBatchCount++;
        if (hOct == localIp[3]) continue;
        ip4_addr_t targetIp;
        IP4_ADDR(&targetIp, hostsScanNetPrefix[0], hostsScanNetPrefix[1], hostsScanNetPrefix[2], hOct);
        etharp_request(netif_default, &targetIp);
      }
    } else {
      hostsScanNextHost = 255;
    }
#if defined(LWIP_TCPIP_CORE_LOCKING) && LWIP_TCPIP_CORE_LOCKING
    UNLOCK_TCPIP_CORE();
#endif

    hostsScanWaitingArp  = true;
    hostsScanBatchSentMs = now;
    renderCurrentScreen();
    return;
  }

  // Wait 45 ms for ARP replies from the current batch of 6 addresses
  if ((now - hostsScanBatchSentMs) < 45UL) return;

  uint8_t foundIps[HOSTS_ARP_BATCH_SIZE][4];
  uint8_t foundMacs[HOSTS_ARP_BATCH_SIZE][6];
  uint8_t foundCnt = 0;

#if defined(LWIP_TCPIP_CORE_LOCKING) && LWIP_TCPIP_CORE_LOCKING
  LOCK_TCPIP_CORE();
#endif
  if (netif_default != NULL && netif_default->hwaddr_len == 6) {
    for (uint8_t k = 0; k < hostsScanBatchCount; k++) {
      uint16_t hostVal = (uint16_t)(hostsScanBatchStart + k);
      if (hostVal > 254 || hostVal == localIp[3]) continue;
      ip4_addr_t targetIp;
      IP4_ADDR(&targetIp, hostsScanNetPrefix[0], hostsScanNetPrefix[1], hostsScanNetPrefix[2], (uint8_t)hostVal);
      struct eth_addr* ethRet = NULL;
      const ip4_addr_t* ipRet = NULL;
      if (etharp_find_addr(netif_default, &targetIp, &ethRet, &ipRet) >= 0 && ethRet != NULL) {
        if (foundCnt < HOSTS_ARP_BATCH_SIZE) {
          foundIps[foundCnt][0] = hostsScanNetPrefix[0];
          foundIps[foundCnt][1] = hostsScanNetPrefix[1];
          foundIps[foundCnt][2] = hostsScanNetPrefix[2];
          foundIps[foundCnt][3] = (uint8_t)hostVal;
          memcpy(foundMacs[foundCnt], ethRet->addr, 6);
          foundCnt++;
        }
      }
    }
  }
#if defined(LWIP_TCPIP_CORE_LOCKING) && LWIP_TCPIP_CORE_LOCKING
  UNLOCK_TCPIP_CORE();
#endif

  int16_t approxRtt = (int16_t)(now - hostsScanBatchSentMs);
  if (approxRtt > 45) approxRtt = 45;
  for (uint8_t f = 0; f < foundCnt; f++) {
    bool isGw = (foundIps[f][0] == gwIp[0] && foundIps[f][1] == gwIp[1] &&
                 foundIps[f][2] == gwIp[2] && foundIps[f][3] == gwIp[3]);
    addLanHostEntry(foundIps[f], foundMacs[f], isGw, false, approxRtt);
  }

  hostsScanWaitingArp = false;
  if (hostsScanNextHost > 254) {
    sortLanHosts();
    hostsScanRunning = false;
    hostsView = HOSTS_VIEW_LIST;
  }
  renderCurrentScreen();
}

// Query NetBIOS name and common TCP ports of selected host when opening detail card
void probeSelectedHostDetails(int16_t idx) {
  if (idx < 0 || idx >= lanHostCount) return;
  LanHostEntry& h = lanHosts[idx];
  if (h.detailsProbed || h.isSelf || WiFi.status() != WL_CONNECTED) return;

  char nbName[24];
  if (queryNetbiosHostName(h.ip, nbName, sizeof(nbName))) {
    strncpy(h.name, nbName, sizeof(h.name) - 1);
    h.name[sizeof(h.name) - 1] = '\0';
  }

  static const uint16_t probePorts[5] = { 80, 443, 22, 53, 8080 };
  IPAddress ip(h.ip[0], h.ip[1], h.ip[2], h.ip[3]);
  char portBuf[24] = "";
  int16_t bestRtt = -1;

  for (uint8_t p = 0; p < 5; p++) {
    WiFiClient cli;
    uint32_t t0 = millis();
    if (cli.connect(ip, probePorts[p], 65)) {
      int16_t dt = (int16_t)(millis() - t0);
      if (bestRtt < 0 || dt < bestRtt) bestRtt = dt;
      cli.stop();
      char tmp[8];
      snprintf(tmp, sizeof(tmp), "%s%u", (portBuf[0] != '\0') ? "," : "", (unsigned)probePorts[p]);
      if (strlen(portBuf) + strlen(tmp) < sizeof(portBuf)) {
        strcat(portBuf, tmp);
      }
    }
  }

  if (bestRtt >= 0) h.rttMs = bestRtt;
  if (portBuf[0] != '\0') {
    strncpy(h.openPorts, portBuf, sizeof(h.openPorts) - 1);
  } else {
    strncpy(h.openPorts, "None (22/53/80/443)", sizeof(h.openPorts) - 1);
  }
  h.detailsProbed = true;
}

// Draw Hosts scan screen
void drawHostsScanScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  if (hostsView == HOSTS_VIEW_NO_WIFI) {
    canvas.fillRoundRect(10, 46, 220, 174, 8, COLOR_HEX_FILL);
    canvas.drawRoundRect(10, 46, 220, 174, 8, COLOR_WARN);
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_WARN, COLOR_HEX_FILL);
    canvas.drawString("Hosts scan", 120, 82);
    canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
    canvas.drawString("Wi-Fi not connected", 120, 120);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
    canvas.drawString("Connect to Wi-Fi network first", 120, 152);
    canvas.drawString("Press X to return to menu", 120, 174);
    return;
  }

  if (hostsView == HOSTS_VIEW_SCANNING) {
    uint16_t done = (hostsScanNextHost > 1) ? (uint16_t)(hostsScanNextHost - 1) : 0;
    if (done > 254) done = 254;
    uint8_t pct = (uint8_t)(((uint32_t)done * 100UL) / 254UL);

    canvas.fillRoundRect(10, 46, 220, 174, 8, COLOR_HEX_FILL);
    canvas.drawRoundRect(10, 46, 220, 174, 8, COLOR_ACCENT);

    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_ACCENT, COLOR_HEX_FILL);
    canvas.drawString("Scanning subnet...", 120, 74);

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
    canvas.drawString(hostsSubnetLabel, 120, 100);

    // ARP scan progress bar
    int16_t barX = 24, barY = 122, barW = 192, barH = 14;
    canvas.fillRoundRect(barX, barY, barW, barH, 4, COLOR_BG);
    canvas.drawRoundRect(barX, barY, barW, barH, 4, COLOR_HEX_BORDER);
    int16_t fillW = (int16_t)(((uint32_t)pct * (uint32_t)(barW - 2)) / 100UL);
    if (fillW > 0) {
      canvas.fillRoundRect(barX + 1, barY + 1, fillW, barH - 2, 3, COLOR_ACCENT);
    }

    char statBuf[32];
    snprintf(statBuf, sizeof(statBuf), "%u%% (%u/254)  Found: %d",
             (unsigned)pct, (unsigned)done, (int)lanHostCount);
    canvas.setTextColor(COLOR_GREEN, COLOR_HEX_FILL);
    canvas.drawString(statBuf, 120, 156);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
    canvas.drawString("Press X to cancel", 120, 186);
    return;
  }

  if (hostsView == HOSTS_VIEW_LIST) {
    drawTopRightRescanIcon();

    // Scroll triangle buttons on top-right and bottom-right
    bool canUp = (hostsScrollOffset > 0);
    uint16_t upCol = canUp ? COLOR_ACCENT : COLOR_HEX_BORDER;
    canvas.fillRoundRect(200, 34, 34, 44, 6, COLOR_HEX_FILL);
    canvas.drawRoundRect(200, 34, 34, 44, 6, upCol);
    canvas.fillTriangle(217, 46, 207, 64, 227, 64, upCol);

    bool canDown = (hostsScrollOffset + HOSTS_VISIBLE_ROWS < lanHostCount);
    uint16_t downCol = canDown ? COLOR_ACCENT : COLOR_HEX_BORDER;
    canvas.fillRoundRect(200, 190, 34, 44, 6, COLOR_HEX_FILL);
    canvas.drawRoundRect(200, 190, 34, 44, 6, downCol);
    canvas.fillTriangle(217, 222, 207, 204, 227, 204, downCol);

    char cntBuf[16];
    snprintf(cntBuf, sizeof(cntBuf), "%d/%d",
             (lanHostCount > 0) ? (int)(hostsScrollOffset + 1) : 0, (int)lanHostCount);
    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(middle_center);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
    canvas.drawString(cntBuf, 217, 134);

    if (lanHostCount == 0) {
      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
      canvas.drawString("No hosts found", 100, 134);
      return;
    }

    for (int16_t row = 0; row < HOSTS_VISIBLE_ROWS; row++) {
      int16_t idx = hostsScrollOffset + row;
      if (idx >= lanHostCount) break;
      const LanHostEntry& h = lanHosts[idx];
      int16_t ry = (int16_t)(34 + row * 41);
      uint16_t fillCol = (h.isGateway || h.isSelf) ? COLOR_HEX_ACTIVE : COLOR_HEX_FILL;
      uint16_t bordCol = h.isGateway ? COLOR_WARN : (h.isSelf ? COLOR_GREEN : COLOR_ACCENT);

      canvas.fillRoundRect(6, ry, 188, 37, 5, fillCol);
      canvas.drawRoundRect(6, ry, 188, 37, 5, bordCol);

      char nameCut[22];
      strncpy(nameCut, h.name, 20);
      nameCut[20] = '\0';
      canvas.setTextDatum(top_left);
      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(h.isGateway ? COLOR_WARN : (h.isSelf ? COLOR_GREEN : COLOR_TEXT), fillCol);
      canvas.drawString(nameCut, 12, ry + 3);

      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(COLOR_ACCENT, fillCol);
      canvas.drawString(h.ipStr, 12, ry + 23);

      canvas.setTextDatum(top_right);
      canvas.setTextColor(COLOR_SUBTEXT, fillCol);
      canvas.drawString(h.role, 188, ry + 23);
    }
    return;
  }

  // HOSTS_VIEW_DETAIL: basic information about selected network host
  if (hostsSelectedIdx >= 0 && hostsSelectedIdx < lanHostCount) {
    const LanHostEntry& h = lanHosts[hostsSelectedIdx];
    canvas.fillRoundRect(8, 33, 224, 151, 8, COLOR_HEX_FILL);
    canvas.drawRoundRect(8, 33, 224, 151, 8, h.isGateway ? COLOR_WARN : COLOR_ACCENT);

    canvas.setTextDatum(top_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(h.isGateway ? COLOR_WARN : COLOR_ACCENT, COLOR_HEX_FILL);
    char titleCut[22];
    strncpy(titleCut, h.name, 20);
    titleCut[20] = '\0';
    canvas.drawString(titleCut, 120, 39);
    canvas.drawFastHLine(16, 58, 208, COLOR_HEX_BORDER);

    const char* labels[6] = { "IP:", "MAC:", "Vendor:", "Role:", "RTT:", "Ports:" };
    char rttBuf[16];
    snprintf(rttBuf, sizeof(rttBuf), "%d ms", (int)h.rttMs);
    const char* vals[6] = { h.ipStr, h.macStr, h.vendor, h.role, rttBuf, h.openPorts };

    canvas.setFont(&fonts::Font0);
    for (uint8_t r = 0; r < 6; r++) {
      int16_t ly = (int16_t)(66 + r * 19);
      canvas.setTextDatum(top_left);
      canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
      canvas.drawString(labels[r], 16, ly);

      canvas.setTextDatum(top_right);
      canvas.setTextColor((r == 0) ? COLOR_ACCENT : ((r == 2) ? COLOR_GREEN : COLOR_TEXT), COLOR_HEX_FILL);
      canvas.drawString(vals[r], 224, ly);
    }
  }

  // OK button to return to hosts list
  canvas.fillRoundRect(56, 192, 128, 40, 8, COLOR_HEX_ACTIVE);
  canvas.drawRoundRect(56, 192, 128, 40, 8, COLOR_GREEN);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_GREEN, COLOR_HEX_ACTIVE);
  canvas.drawString("OK", 120, 212);
}

// Decode BLE manufacturer from 16-bit Bluetooth SIG Company Identifier
const char* lookupBleCompanyName(uint16_t companyId) {
  switch (companyId) {
    case 0x004C: return "Apple";
    case 0x0075: return "Samsung";
    case 0x0006: return "Microsoft";
    case 0x00E0: return "Google";
    case 0x038F: return "Xiaomi";
    case 0x0059: return "Nordic Semi";
    case 0x02E5: return "Espressif";
    case 0x0157: return "Huawei";
    case 0x012D: return "Sony";
    case 0x0087: return "Garmin";
    case 0x009E: return "Bose";
    case 0x0057: return "Harman/JBL";
    case 0x00C4: return "LG Electronics";
    case 0x0002: return "Intel";
    case 0x000A: return "Qualcomm";
    case 0x0131: return "Cypress/Inf";
    case 0x00D2: return "Dialog Semi";
    case 0x0171: return "Amazon";
    default:     return "Unknown";
  }
}

// Decode BLE GAP Appearance to human-readable device type
const char* lookupBleAppearance(uint16_t app) {
  uint16_t cat = (uint16_t)(app >> 6);
  switch (cat) {
    case 1:  return "Phone";
    case 2:  return "Computer";
    case 3:  return "Watch";
    case 4:  return "Clock";
    case 13: return "Heart Rate";
    case 15: return "HID Input";
    case 16: return "Running Sens";
    case 18: return "Cycling Sens";
    case 21: return "Audio/Headset";
    case 49: return "Beacon/Tag";
    default: return "BLE Device";
  }
}

// Parse textual MAC "aa:bb:cc:dd:ee:ff" into 6 bytes and uppercase string
void formatBleMacUpper(const char* src, char* outStr, size_t outLen, uint8_t outBytes[6]) {
  memset(outBytes, 0, 6);
  unsigned int b[6] = {0, 0, 0, 0, 0, 0};
  if (sscanf(src, "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) == 6) {
    for (int i = 0; i < 6; i++) outBytes[i] = (uint8_t)b[i];
    snprintf(outStr, outLen, "%02X:%02X:%02X:%02X:%02X:%02X",
             outBytes[0], outBytes[1], outBytes[2], outBytes[3], outBytes[4], outBytes[5]);
  } else {
    strncpy(outStr, src, outLen - 1);
    outStr[outLen - 1] = '\0';
    for (size_t i = 0; outStr[i] != '\0'; i++) {
      if (outStr[i] >= 'a' && outStr[i] <= 'z') outStr[i] = (char)(outStr[i] - 32);
    }
  }
}

// Shut down BLE scan controller and restore Wi-Fi STA mode cleanly (prevents Wi-Fi scan starvation)
void restoreWifiAfterBleScan() {
  if (!bleHidKbActive && !bleHidMouseActive && BLEDevice::getInitialized()) {
    BLEDevice::deinit(false);
    delay(40);
    bleStackInitialized = false;
  }
  if (btWifiPausedForBle) {
    btWifiPausedForBle = false;
    if (settingWifiEnabled) {
      WiFi.mode(WIFI_STA);
      WiFi.setSleep(false);
      delay(40);
    }
  } else if (btWifiSleepForBle) {
    btWifiSleepForBle = false;
    if (settingWifiEnabled) {
      WiFi.setSleep(false);
    }
  }
}

// Non-blocking Bluedroid completion callback (runs on Core 0 BTC task; only sets flag)
void onBleScanComplete(BLEScanResults scanResults) {
  (void)scanResults;
  btScanDoneFlag = true;
}

// Stop BLE scan, release results buffer, deinit BLE radio, and restore Wi-Fi STA
void stopBluetoothScan() {
  btScanPending  = false;
  btScanRunning  = false;
  btScanDoneFlag = false;
  if (bleStackInitialized && BLEDevice::getInitialized()) {
    BLEScan* pScan = BLEDevice::getScan();
    if (pScan != NULL) {
      pScan->stop();
      pScan->clearResults();
    }
  }
  restoreWifiAfterBleScan();
}

// Prepare and start scanning nearby Bluetooth devices (vibration silenced)
void requestBluetoothScan() {
  stopTouchVibrationNow();
  if (btScanRunning) {
    stopBluetoothScan();
  }
  btDeviceCount  = 0;
  btScrollOffset = 0;
  btSelectedIdx  = -1;
  btView         = BT_VIEW_SCANNING;
  btScanDoneFlag = false;
  btScanPending  = true;
}

// Start non-blocking asynchronous BLE scan with Wi-Fi/BLE coexistence protection
void runBluetoothScanNow() {
  btScanPending = false;
  stopTouchVibrationNow();
  if (currentScreen != SCREEN_BT_SCAN) return;

  // Cancel any active Wi-Fi background scan to avoid 2.4 GHz radio deadlock
  if (wifiBgScanning) {
    WiFi.scanDelete();
    wifiBgScanning = false;
  }

  // Handle ESP32-S3 shared 2.4 GHz Wi-Fi/BLE coexistence cleanly
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.setSleep(true); // Required by ESP-IDF when Wi-Fi and BLE share the radio
    btWifiSleepForBle = true;
  } else {
    WiFi.mode(WIFI_OFF);
    btWifiPausedForBle = true;
    btWifiSleepForBle  = false;
    delay(20);
  }

  if (!BLEDevice::getInitialized()) {
    BLEDevice::init("");
    delay(30);
  }
  bleStackInitialized = BLEDevice::getInitialized();
  if (!bleStackInitialized) {
    restoreWifiAfterBleScan();
    btView = BT_VIEW_LIST;
    renderCurrentScreen();
    return;
  }

  BLEScan* pScan = BLEDevice::getScan();
  if (pScan == NULL) {
    restoreWifiAfterBleScan();
    btView = BT_VIEW_LIST;
    renderCurrentScreen();
    return;
  }

  // Coexistence-safe 50% duty cycle (100 ms interval, 50 ms window) prevents RF starvation
  pScan->stop();
  pScan->clearResults();
  pScan->setActiveScan(true);
  pScan->setInterval(160);
  pScan->setWindow(80);

  btScanDoneFlag = false;
  btScanStartMs  = millis();
  btScanRunning  = true;

  // Asynchronous non-blocking scan (never blocks loopTask on infinite semaphore wait)
  bool started = pScan->start(BT_SCAN_DURATION_SEC, onBleScanComplete, false);
  if (!started) {
    btScanRunning = false;
    pScan->clearResults();
    restoreWifiAfterBleScan();
    btView = BT_VIEW_LIST;
    renderCurrentScreen();
  }
}

// Poll asynchronous BLE scan in loop(), collect results on completion or watchdog timeout
void updateBluetoothScan() {
  if (!btScanRunning) return;

  uint32_t elapsed = millis() - btScanStartMs;
  bool timedOut = (elapsed >= BT_SCAN_TIMEOUT_MS);
  if (!btScanDoneFlag && !timedOut) return;

  btScanRunning = false;
  BLEScan* pScan = (bleStackInitialized && BLEDevice::getInitialized()) ? BLEDevice::getScan() : NULL;
  if (pScan != NULL && timedOut && !btScanDoneFlag) {
    pScan->stop();
    delay(10);
  }
  btScanDoneFlag = false;

  btDeviceCount = 0;
  if (pScan != NULL) {
#if ESP_IDF_VERSION_MAJOR >= 5
    BLEScanResults* scanRes = pScan->getResults();
    int foundCount = (scanRes != NULL) ? scanRes->getCount() : 0;
#else
    BLEScanResults scanResVal = pScan->getResults();
    BLEScanResults* scanRes = &scanResVal;
    int foundCount = scanRes->getCount();
#endif

    for (int i = 0; i < foundCount && btDeviceCount < BT_SCAN_MAX; i++) {
      BLEAdvertisedDevice dev = scanRes->getDevice(i);
      BtDeviceEntry& d = btDevices[btDeviceCount];
      memset(&d, 0, sizeof(BtDeviceEntry));

      formatBleMacUpper(dev.getAddress().toString().c_str(), d.macStr, sizeof(d.macStr), d.mac);

      if (dev.haveName()) {
        strncpy(d.name, dev.getName().c_str(), sizeof(d.name) - 1);
        d.name[sizeof(d.name) - 1] = '\0';
      } else {
        d.name[0] = '\0';
      }

      d.rssi = (int8_t)dev.getRSSI();
      if (dev.haveTXPower()) {
        d.txPower    = dev.getTXPower();
        d.hasTxPower = true;
      } else {
        d.txPower    = -127;
        d.hasTxPower = false;
      }
      d.addrType = (uint8_t)dev.getAddressType();

      if (dev.haveManufacturerData() && dev.getManufacturerData().length() >= 2) {
        const uint8_t* mfg = (const uint8_t*)dev.getManufacturerData().c_str();
        d.companyId    = (uint16_t)((uint16_t)mfg[0] | ((uint16_t)mfg[1] << 8));
        d.hasCompanyId = true;
        strncpy(d.vendor, lookupBleCompanyName(d.companyId), sizeof(d.vendor) - 1);
      } else {
        d.hasCompanyId = false;
        strncpy(d.vendor, lookupOuiVendor(d.mac), sizeof(d.vendor) - 1);
      }

      if (dev.haveAppearance()) {
        d.appearance = dev.getAppearance();
        strncpy(d.devType, lookupBleAppearance(d.appearance), sizeof(d.devType) - 1);
      } else {
        d.appearance = 0;
        strncpy(d.devType, "BLE Device", sizeof(d.devType) - 1);
      }

      if (dev.haveServiceUUID()) {
        strncpy(d.serviceUuid, dev.getServiceUUID().toString().c_str(), sizeof(d.serviceUuid) - 1);
        d.serviceUuid[sizeof(d.serviceUuid) - 1] = '\0';
      } else {
        strncpy(d.serviceUuid, "None", sizeof(d.serviceUuid) - 1);
      }

      btDeviceCount++;
    }

    pScan->clearResults();
  }

  restoreWifiAfterBleScan();

  // Sort by RSSI signal strength (nearest devices at top)
  for (int16_t i = 0; i < btDeviceCount - 1; i++) {
    for (int16_t j = i + 1; j < btDeviceCount; j++) {
      if (btDevices[j].rssi > btDevices[i].rssi) {
        BtDeviceEntry tmp = btDevices[i];
        btDevices[i] = btDevices[j];
        btDevices[j] = tmp;
      }
    }
  }

  btView = BT_VIEW_LIST;
  if (currentScreen == SCREEN_BT_SCAN) {
    renderCurrentScreen();
  }
}

// Draw Bluetooth scan screen
void drawBluetoothScanScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  if (btView == BT_VIEW_SCANNING) {
    canvas.fillRoundRect(10, 56, 220, 148, 8, COLOR_HEX_FILL);
    canvas.drawRoundRect(10, 56, 220, 148, 8, COLOR_ACCENT);

    // Bluetooth icon centered on scanning card
    int16_t cx = 120, cy = 94;
    canvas.drawLine(cx, cy - 14, cx, cy + 14, COLOR_ACCENT);
    canvas.drawLine(cx, cy - 14, cx + 9, cy - 6, COLOR_ACCENT);
    canvas.drawLine(cx + 9, cy - 6, cx - 7, cy + 6, COLOR_ACCENT);
    canvas.drawLine(cx - 7, cy - 6, cx + 9, cy + 6, COLOR_ACCENT);
    canvas.drawLine(cx + 9, cy + 6, cx, cy + 14, COLOR_ACCENT);

    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_ACCENT, COLOR_HEX_FILL);
    canvas.drawString("Scanning BLE...", 120, 134);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
    canvas.drawString("Searching nearby devices (3s)", 120, 166);
    return;
  }

  if (btView == BT_VIEW_LIST) {
    drawTopRightRescanIcon();

    // Scroll triangle buttons on top-right and bottom-right
    bool canUp = (btScrollOffset > 0);
    uint16_t upCol = canUp ? COLOR_ACCENT : COLOR_HEX_BORDER;
    canvas.fillRoundRect(200, 34, 34, 44, 6, COLOR_HEX_FILL);
    canvas.drawRoundRect(200, 34, 34, 44, 6, upCol);
    canvas.fillTriangle(217, 46, 207, 64, 227, 64, upCol);

    bool canDown = (btScrollOffset + BT_VISIBLE_ROWS < btDeviceCount);
    uint16_t downCol = canDown ? COLOR_ACCENT : COLOR_HEX_BORDER;
    canvas.fillRoundRect(200, 190, 34, 44, 6, COLOR_HEX_FILL);
    canvas.drawRoundRect(200, 190, 34, 44, 6, downCol);
    canvas.fillTriangle(217, 222, 207, 204, 227, 204, downCol);

    char cntBuf[16];
    snprintf(cntBuf, sizeof(cntBuf), "%d/%d",
             (btDeviceCount > 0) ? (int)(btScrollOffset + 1) : 0, (int)btDeviceCount);
    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(middle_center);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
    canvas.drawString(cntBuf, 217, 134);

    if (btDeviceCount == 0) {
      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
      canvas.drawString("No BLE devices", 100, 134);
      return;
    }

    for (int16_t row = 0; row < BT_VISIBLE_ROWS; row++) {
      int16_t idx = btScrollOffset + row;
      if (idx >= btDeviceCount) break;
      const BtDeviceEntry& d = btDevices[idx];
      int16_t ry = (int16_t)(34 + row * 41);
      bool hasName = (d.name[0] != '\0');
      uint16_t rssiCol = (d.rssi >= -60) ? COLOR_GREEN : ((d.rssi >= -78) ? COLOR_WARN : COLOR_RED);

      canvas.fillRoundRect(6, ry, 188, 37, 5, COLOR_HEX_FILL);
      canvas.drawRoundRect(6, ry, 188, 37, 5, hasName ? COLOR_ACCENT : COLOR_HEX_BORDER);

      char topLine[22];
      if (hasName) {
        strncpy(topLine, d.name, 19);
        topLine[19] = '\0';
      } else {
        strncpy(topLine, d.macStr, 17);
        topLine[17] = '\0';
      }

      canvas.setTextDatum(top_left);
      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(hasName ? COLOR_TEXT : COLOR_ACCENT, COLOR_HEX_FILL);
      canvas.drawString(topLine, 12, ry + 3);

      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
      canvas.drawString(hasName ? d.macStr : d.vendor, 12, ry + 23);

      char rssiBuf[12];
      snprintf(rssiBuf, sizeof(rssiBuf), "%ddB", (int)d.rssi);
      canvas.setTextDatum(top_right);
      canvas.setTextColor(rssiCol, COLOR_HEX_FILL);
      canvas.drawString(rssiBuf, 188, ry + 23);
    }
    return;
  }

  // BT_VIEW_DETAIL: basic information about selected Bluetooth device
  if (btSelectedIdx >= 0 && btSelectedIdx < btDeviceCount) {
    const BtDeviceEntry& d = btDevices[btSelectedIdx];
    canvas.fillRoundRect(8, 33, 224, 151, 8, COLOR_HEX_FILL);
    canvas.drawRoundRect(8, 33, 224, 151, 8, COLOR_ACCENT);

    canvas.setTextDatum(top_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_ACCENT, COLOR_HEX_FILL);
    char titleCut[22];
    strncpy(titleCut, (d.name[0] != '\0') ? d.name : "<Unnamed BLE>", 20);
    titleCut[20] = '\0';
    canvas.drawString(titleCut, 120, 39);
    canvas.drawFastHLine(16, 58, 208, COLOR_HEX_BORDER);

    char rssiVal[24];
    const char* sigQual = (d.rssi >= -60) ? "Strong" : ((d.rssi >= -78) ? "Medium" : "Weak");
    snprintf(rssiVal, sizeof(rssiVal), "%d dBm (%s)", (int)d.rssi, sigQual);

    char txVal[16];
    if (d.hasTxPower) snprintf(txVal, sizeof(txVal), "%d dBm", (int)d.txPower);
    else strncpy(txVal, "N/A", sizeof(txVal) - 1);

    const char* addrTypeStr = (d.addrType == 0) ? "Public" : ((d.addrType == 1) ? "Random" : "RPA");
    char typeBuf[28];
    snprintf(typeBuf, sizeof(typeBuf), "%s / %s", addrTypeStr, d.devType);

    char uuidShort[22];
    strncpy(uuidShort, d.serviceUuid, 20);
    uuidShort[20] = '\0';

    const char* labels[6] = { "MAC:", "Vendor:", "RSSI:", "TX Pwr:", "Type:", "UUID:" };
    const char* vals[6]   = { d.macStr, d.vendor, rssiVal, txVal, typeBuf, uuidShort };

    canvas.setFont(&fonts::Font0);
    for (uint8_t r = 0; r < 6; r++) {
      int16_t ly = (int16_t)(66 + r * 19);
      canvas.setTextDatum(top_left);
      canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
      canvas.drawString(labels[r], 16, ly);

      canvas.setTextDatum(top_right);
      uint16_t valCol = (r == 0) ? COLOR_ACCENT : ((r == 2) ? ((d.rssi >= -60) ? COLOR_GREEN : COLOR_WARN) : COLOR_TEXT);
      canvas.setTextColor(valCol, COLOR_HEX_FILL);
      canvas.drawString(vals[r], 224, ly);
    }
  }

  // OK button to return to Bluetooth devices list
  canvas.fillRoundRect(56, 192, 128, 40, 8, COLOR_HEX_ACTIVE);
  canvas.drawRoundRect(56, 192, 128, 40, 8, COLOR_GREEN);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_GREEN, COLOR_HEX_ACTIVE);
  canvas.drawString("OK", 120, 212);
}

// ============================================================================
// 7.11B. BLUETOOTH HID KEYBOARD (SCREEN_BT_KEYBOARD)
// Full 104-key PC keyboard split across 8 pages (Wi-Fi connect style)
// ============================================================================

#define BT_KB_PAGE_COUNT            8
#define BT_KB_MOD_CTRL              0x01
#define BT_KB_MOD_SHIFT             0x02
#define BT_KB_MOD_ALT               0x04
#define BT_KB_MOD_GUI               0x08
#define BLE_HID_APPEARANCE_KEYBOARD 0x03C1

#define BT_KB_FLAG_ALPHA            0x01
#define BT_KB_FLAG_FN               0x02
#define BT_KB_FLAG_NAV              0x04
#define BT_KB_FLAG_LOCK             0x08
#define BT_KB_FLAG_NUM              0x10

// struct BtKbKeyDef is declared in Section 3 before top-level functions for Arduino IDE auto-prototypes

// Page 1 (1/8): QWERTY left block (identical to Wi-Fi connect page 1)
static const BtKbKeyDef BT_KB_PAGE0[12] = {
  {"q", "Q", 0x14, BT_KB_FLAG_ALPHA}, {"w", "W", 0x1A, BT_KB_FLAG_ALPHA},
  {"e", "E", 0x08, BT_KB_FLAG_ALPHA}, {"r", "R", 0x15, BT_KB_FLAG_ALPHA},
  {"a", "A", 0x04, BT_KB_FLAG_ALPHA}, {"s", "S", 0x16, BT_KB_FLAG_ALPHA},
  {"d", "D", 0x07, BT_KB_FLAG_ALPHA}, {"f", "F", 0x09, BT_KB_FLAG_ALPHA},
  {"z", "Z", 0x1D, BT_KB_FLAG_ALPHA}, {"x", "X", 0x1B, BT_KB_FLAG_ALPHA},
  {"c", "C", 0x06, BT_KB_FLAG_ALPHA}, {"v", "V", 0x19, BT_KB_FLAG_ALPHA}
};

// Page 2 (2/8): QWERTY middle block (11 keys, 3 wide keys on row 3 like Wi-Fi connect page 2)
static const BtKbKeyDef BT_KB_PAGE1[11] = {
  {"t", "T", 0x17, BT_KB_FLAG_ALPHA}, {"y", "Y", 0x1C, BT_KB_FLAG_ALPHA},
  {"u", "U", 0x18, BT_KB_FLAG_ALPHA}, {"i", "I", 0x0C, BT_KB_FLAG_ALPHA},
  {"g", "G", 0x0A, BT_KB_FLAG_ALPHA}, {"h", "H", 0x0B, BT_KB_FLAG_ALPHA},
  {"j", "J", 0x0D, BT_KB_FLAG_ALPHA}, {"k", "K", 0x0E, BT_KB_FLAG_ALPHA},
  {"b", "B", 0x05, BT_KB_FLAG_ALPHA}, {"n", "N", 0x11, BT_KB_FLAG_ALPHA},
  {"m", "M", 0x10, BT_KB_FLAG_ALPHA}
};

// Page 3 (3/8): QWERTY right block + full punctuation (including \ and `)
static const BtKbKeyDef BT_KB_PAGE2[12] = {
  {"o", "O", 0x12, BT_KB_FLAG_ALPHA}, {"p", "P", 0x13, BT_KB_FLAG_ALPHA},
  {"[", "{", 0x2F, 0},                {"]", "}", 0x30, 0},
  {"l", "L", 0x0F, BT_KB_FLAG_ALPHA}, {";", ":", 0x33, 0},
  {"'", "\"", 0x34, 0},               {",", "<", 0x36, 0},
  {".", ">", 0x37, 0},                {"/", "?", 0x38, 0},
  {"\\", "|", 0x31, 0},               {"`", "~", 0x35, 0}
};

// Page 4 (4/8): Digits & shifted symbols (identical to Wi-Fi connect page 4)
static const BtKbKeyDef BT_KB_PAGE3[12] = {
  {"1", "!", 0x1E, 0}, {"2", "@", 0x1F, 0}, {"3", "#", 0x20, 0}, {"4", "$", 0x21, 0},
  {"5", "%", 0x22, 0}, {"6", "^", 0x23, 0}, {"7", "&", 0x24, 0}, {"8", "*", 0x25, 0},
  {"9", "(", 0x26, 0}, {"0", ")", 0x27, 0}, {"-", "_", 0x2D, 0}, {"=", "+", 0x2E, 0}
};

// Page 5 (5/8): Function keys F1-F12
static const BtKbKeyDef BT_KB_PAGE4[12] = {
  {"F1",  NULL, 0x3A, BT_KB_FLAG_FN}, {"F2",  NULL, 0x3B, BT_KB_FLAG_FN},
  {"F3",  NULL, 0x3C, BT_KB_FLAG_FN}, {"F4",  NULL, 0x3D, BT_KB_FLAG_FN},
  {"F5",  NULL, 0x3E, BT_KB_FLAG_FN}, {"F6",  NULL, 0x3F, BT_KB_FLAG_FN},
  {"F7",  NULL, 0x40, BT_KB_FLAG_FN}, {"F8",  NULL, 0x41, BT_KB_FLAG_FN},
  {"F9",  NULL, 0x42, BT_KB_FLAG_FN}, {"F10", NULL, 0x43, BT_KB_FLAG_FN},
  {"F11", NULL, 0x44, BT_KB_FLAG_FN}, {"F12", NULL, 0x45, BT_KB_FLAG_FN}
};

// Page 6 (6/8): Navigation, arrows, and editing keys
static const BtKbKeyDef BT_KB_PAGE5[12] = {
  {"Esc",  NULL, 0x29, BT_KB_FLAG_NAV}, {"Tab",  NULL, 0x2B, BT_KB_FLAG_NAV},
  {"Ins",  NULL, 0x49, BT_KB_FLAG_NAV}, {"Del",  NULL, 0x4C, BT_KB_FLAG_NAV},
  {"Home", NULL, 0x4A, BT_KB_FLAG_NAV}, {"Up",   NULL, 0x52, BT_KB_FLAG_NAV},
  {"End",  NULL, 0x4D, BT_KB_FLAG_NAV}, {"PgUp", NULL, 0x4B, BT_KB_FLAG_NAV},
  {"Left", NULL, 0x50, BT_KB_FLAG_NAV}, {"Down", NULL, 0x51, BT_KB_FLAG_NAV},
  {"Right",NULL, 0x4F, BT_KB_FLAG_NAV}, {"PgDn", NULL, 0x4E, BT_KB_FLAG_NAV}
};

// Page 7 (7/8): System keys, Locks, Menu, and Numpad operators
static const BtKbKeyDef BT_KB_PAGE6[12] = {
  {"Caps",  NULL, 0x39, BT_KB_FLAG_LOCK}, {"NumLk", NULL, 0x53, BT_KB_FLAG_LOCK},
  {"ScrLk", NULL, 0x47, BT_KB_FLAG_LOCK}, {"PrtSc", NULL, 0x46, BT_KB_FLAG_LOCK},
  {"Pause", NULL, 0x48, BT_KB_FLAG_LOCK}, {"Menu",  NULL, 0x65, BT_KB_FLAG_LOCK},
  {"KP/",   NULL, 0x54, BT_KB_FLAG_NUM},  {"KP*",   NULL, 0x55, BT_KB_FLAG_NUM},
  {"KP-",   NULL, 0x56, BT_KB_FLAG_NUM},  {"KP+",   NULL, 0x57, BT_KB_FLAG_NUM},
  {"KP.",   NULL, 0x63, BT_KB_FLAG_NUM},  {"KPEnt", NULL, 0x58, BT_KB_FLAG_NUM}
};

// Page 8 (8/8): Full numeric keypad (0-9, ., Enter)
static const BtKbKeyDef BT_KB_PAGE7[12] = {
  {"KP7", NULL, 0x5F, BT_KB_FLAG_NUM}, {"KP8", NULL, 0x60, BT_KB_FLAG_NUM},
  {"KP9", NULL, 0x61, BT_KB_FLAG_NUM}, {"KP0", NULL, 0x62, BT_KB_FLAG_NUM},
  {"KP4", NULL, 0x5C, BT_KB_FLAG_NUM}, {"KP5", NULL, 0x5D, BT_KB_FLAG_NUM},
  {"KP6", NULL, 0x5E, BT_KB_FLAG_NUM}, {"KP.", NULL, 0x63, BT_KB_FLAG_NUM},
  {"KP1", NULL, 0x59, BT_KB_FLAG_NUM}, {"KP2", NULL, 0x5A, BT_KB_FLAG_NUM},
  {"KP3", NULL, 0x5B, BT_KB_FLAG_NUM}, {"KPEnt",NULL, 0x58, BT_KB_FLAG_NUM}
};

// Standard USB HID 1.11 Keyboard Report Descriptor (Report ID 1, raw hex to avoid Arduino macro conflicts)
static const uint8_t BLE_HID_KB_REPORT_MAP[] = {
  0x05, 0x01,       // Usage Page (Generic Desktop)
  0x09, 0x06,       // Usage (Keyboard)
  0xA1, 0x01,       // Collection (Application)
  0x85, 0x01,       //   Report ID (1)
  0x05, 0x07,       //   Usage Page (Key Codes)
  0x19, 0xE0,       //   Usage Minimum (224 - Left Ctrl)
  0x29, 0xE7,       //   Usage Maximum (231 - Right GUI)
  0x15, 0x00,       //   Logical Minimum (0)
  0x25, 0x01,       //   Logical Maximum (1)
  0x75, 0x01,       //   Report Size (1)
  0x95, 0x08,       //   Report Count (8)
  0x81, 0x02,       //   Input (Data, Variable, Absolute) - Modifier byte
  0x95, 0x01,       //   Report Count (1)
  0x75, 0x08,       //   Report Size (8)
  0x81, 0x01,       //   Input (Constant) - Reserved byte
  0x95, 0x05,       //   Report Count (5)
  0x75, 0x01,       //   Report Size (1)
  0x05, 0x08,       //   Usage Page (LEDs)
  0x19, 0x01,       //   Usage Minimum (1 - Num Lock)
  0x29, 0x05,       //   Usage Maximum (5 - Kana)
  0x91, 0x02,       //   Output (Data, Variable, Absolute) - LED report
  0x95, 0x01,       //   Report Count (1)
  0x75, 0x03,       //   Report Size (3)
  0x91, 0x01,       //   Output (Constant) - LED padding
  0x95, 0x06,       //   Report Count (6)
  0x75, 0x08,       //   Report Size (8)
  0x15, 0x00,       //   Logical Minimum (0)
  0x25, 0x65,       //   Logical Maximum (101)
  0x05, 0x07,       //   Usage Page (Key Codes)
  0x19, 0x00,       //   Usage Minimum (0)
  0x29, 0x65,       //   Usage Maximum (101)
  0x81, 0x00,       //   Input (Data, Array) - 6-key rollover array
  0xC0              // End Collection
};

BLEServer* bleHidServer          = NULL;
BLEHIDDevice* bleHidDevice       = NULL;
BLECharacteristic* bleHidKbInput = NULL;
BLECharacteristic* bleHidKbOutput = NULL;
volatile bool bleHidKbConnected  = false;
volatile bool bleHidKbUiDirty    = false;
volatile uint8_t bleHidHostLeds  = 0;
uint8_t btKbPage                 = 0;
uint8_t btKbMods                 = 0;
char btKbLastSent[24]            = "";
bool btKbWifiPaused              = false;
bool btKbWifiWasConnected        = false;

class BleHidKbServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    (void)pServer;
    bleHidKbConnected = true;
#if defined(CONFIG_BLUEDROID_ENABLED)
    if (bleHidKbInput != NULL) {
      BLE2902* desc = (BLE2902*)bleHidKbInput->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
      if (desc != NULL) {
        desc->setNotifications(true);
      }
    }
#endif
    bleHidKbUiDirty = true;
  }

  void onDisconnect(BLEServer* pServer) {
    bleHidKbConnected = false;
    bleHidKbUiDirty = true;
    if (bleHidKbActive && pServer != NULL) {
      pServer->startAdvertising();
    }
  }
};

class BleHidKbOutputCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) {
    if (pChar != NULL && pChar->getLength() > 0) {
      const uint8_t* data = pChar->getData();
      if (data != NULL) {
        bleHidHostLeds = data[0];
        bleHidKbUiDirty = true;
      }
    }
  }
};

static BleHidKbServerCallbacks bleHidKbServerCb;
static BleHidKbOutputCallbacks bleHidKbOutputCb;
static BLESecurity bleHidKbSecurity;

// Safely disconnect any host, stop BLE advertising, deinit BLE HID, and restore Wi-Fi STA
void stopBleHidKeyboard() {
  bleHidKbStartPending = false;
  bleHidKbActive = false;

  if (bleHidKbConnected && bleHidKbInput != NULL) {
    uint8_t emptyReport[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    bleHidKbInput->setValue(emptyReport, sizeof(emptyReport));
    bleHidKbInput->notify();
    delay(15);
  }

  if (BLEDevice::getInitialized()) {
    BLEDevice::stopAdvertising();
    if (bleHidServer != NULL && bleHidServer->getConnectedCount() > 0) {
      bleHidServer->disconnect(bleHidServer->getConnId());
      delay(60);
    }
  }

  if (bleHidServer != NULL) {
    bleHidServer->setCallbacks(NULL);
  }
  if (bleHidKbOutput != NULL) {
    bleHidKbOutput->setCallbacks(NULL);
  }
  bleHidServer      = NULL;
  bleHidKbInput     = NULL;
  bleHidKbOutput    = NULL;
  bleHidKbConnected = false;
  bleHidKbUiDirty   = false;
  bleHidHostLeds    = 0;
  btKbMods          = 0;

  if (BLEDevice::getInitialized()) {
    BLEDevice::deinit(false);
    delay(80);
  }
  bleStackInitialized = false;

  if (bleHidDevice != NULL) {
    delete bleHidDevice;
    bleHidDevice = NULL;
  }

  if (btKbWifiPaused) {
    btKbWifiPaused = false;
    if (settingWifiEnabled) {
      WiFi.mode(WIFI_STA);
      WiFi.setSleep(false);
      delay(40);
      if (btKbWifiWasConnected) {
        btKbWifiWasConnected = false;
        WiFi.reconnect();
      }
    } else {
      btKbWifiWasConnected = false;
    }
  }
  bleHidPresenterMode = false;
}

// Schedule deferred BLE HID Keyboard bring-up after stopping touch vibration and painting UI
void requestBleHidKeyboardStart() {
  stopTouchVibrationNow();
  bleHidPresenterMode  = false;
  bleHidKbActive       = false;
  bleHidKbConnected    = false;
  bleHidKbUiDirty      = false;
  bleHidHostLeds       = 0;
  btKbPage             = 0;
  btKbMods             = 0;
  btKbLastSent[0]      = '\0';
  bleHidKbStartPending = true;
}

// Schedule deferred BLE HID Presenter bring-up (uses BLE HID Keyboard profile on a single slide-control screen)
void requestBleHidPresenterStart() {
  stopTouchVibrationNow();
  bleHidPresenterMode  = true;
  bleHidKbActive       = false;
  bleHidKbConnected    = false;
  bleHidKbUiDirty      = false;
  bleHidHostLeds       = 0;
  btKbPage             = 0;
  btKbMods             = 0;
  btKbLastSent[0]      = '\0';
  bleHidKbStartPending = true;
}

// Start Bluetooth HID Keyboard / Presenter mode (pauses Wi-Fi for DRAM/RF safety, full NULL guards)
void startBleHidKeyboard() {
  bleHidKbStartPending = false;
  stopTouchVibrationNow();
  if (currentScreen != SCREEN_BT_KEYBOARD && currentScreen != SCREEN_BT_PRESENTER) return;

  stopBluetoothScan();
  if (bleHidMouseActive || bleHidMouseStartPending) {
    stopBleHidMouse();
  }
  if (wifiBgScanning) {
    WiFi.scanDelete();
    wifiBgScanning = false;
  }

  // Pause Wi-Fi STA while BLE HID is active: frees ~65 KB internal DRAM and avoids radio/coex aborts
  if (!btKbWifiPaused) {
    btKbWifiWasConnected = (WiFi.status() == WL_CONNECTED);
    WiFi.disconnect(false, false);
    WiFi.mode(WIFI_OFF);
    btKbWifiPaused = true;
    delay(50);
  }

  // Re-initialize BLE with proper HID Keyboard GAP name
  if (BLEDevice::getInitialized()) {
    BLEDevice::stopAdvertising();
    BLEDevice::deinit(false);
    delay(80);
  }

  if (bleHidPresenterMode) {
    BLEDevice::init("T-Watch S3 Pres");
  } else {
    BLEDevice::init("T-Watch S3 KB");
  }
  delay(50);
  bleStackInitialized = BLEDevice::getInitialized();
  if (!bleStackInitialized) {
    stopBleHidKeyboard();
    strncpy(btKbLastSent, "BLE Init Err", sizeof(btKbLastSent) - 1);
    btKbLastSent[sizeof(btKbLastSent) - 1] = '\0';
    if (currentScreen == SCREEN_BT_KEYBOARD || currentScreen == SCREEN_BT_PRESENTER) renderCurrentScreen();
    return;
  }

  // Enable LE Secure Connections + Bonding required by Android 13/14, Windows, iOS, macOS, and Linux HID hosts
#if defined(ESP_LE_AUTH_REQ_SC_BOND)
  bleHidKbSecurity.setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
#else
  bleHidKbSecurity.setAuthenticationMode(ESP_LE_AUTH_BOND);
#endif
  bleHidKbSecurity.setCapability(ESP_IO_CAP_NONE);
  bleHidKbSecurity.setKeySize(16);
  bleHidKbSecurity.setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  bleHidKbSecurity.setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
#if defined(CONFIG_BLUEDROID_ENABLED)
  esp_ble_gap_config_local_icon(BLE_HID_APPEARANCE_KEYBOARD);
#endif

  bleHidServer = BLEDevice::createServer();
  if (bleHidServer == NULL) {
    stopBleHidKeyboard();
    strncpy(btKbLastSent, "BLE Srv Err", sizeof(btKbLastSent) - 1);
    btKbLastSent[sizeof(btKbLastSent) - 1] = '\0';
    if (currentScreen == SCREEN_BT_KEYBOARD || currentScreen == SCREEN_BT_PRESENTER) renderCurrentScreen();
    return;
  }
  bleHidServer->setCallbacks(&bleHidKbServerCb);

  bleHidDevice = new BLEHIDDevice(bleHidServer);
  if (bleHidDevice == NULL) {
    stopBleHidKeyboard();
    strncpy(btKbLastSent, "BLE HID Err", sizeof(btKbLastSent) - 1);
    btKbLastSent[sizeof(btKbLastSent) - 1] = '\0';
    if (currentScreen == SCREEN_BT_KEYBOARD || currentScreen == SCREEN_BT_PRESENTER) renderCurrentScreen();
    return;
  }

  bleHidKbInput  = bleHidDevice->inputReport(1);
  bleHidKbOutput = bleHidDevice->outputReport(1);
  if (bleHidKbInput == NULL) {
    stopBleHidKeyboard();
    strncpy(btKbLastSent, "BLE Rep Err", sizeof(btKbLastSent) - 1);
    btKbLastSent[sizeof(btKbLastSent) - 1] = '\0';
    if (currentScreen == SCREEN_BT_KEYBOARD || currentScreen == SCREEN_BT_PRESENTER) renderCurrentScreen();
    return;
  }
  if (bleHidKbOutput != NULL) {
    bleHidKbOutput->setCallbacks(&bleHidKbOutputCb);
  }

  // Version-agnostic raw buffer setValue avoids Core 2.x std::string vs Core 3.x String conflicts
  BLECharacteristic* pMfg = bleHidDevice->manufacturer();
  if (pMfg != NULL) {
    pMfg->setValue((uint8_t*)"VRXFILE", 7);
  }
  bleHidDevice->pnp(0x02, 0xE502, 0xA111, 0x0210);
  bleHidDevice->hidInfo(0x00, 0x01);
  bleHidDevice->reportMap((uint8_t*)BLE_HID_KB_REPORT_MAP, sizeof(BLE_HID_KB_REPORT_MAP));
  // Set initial battery level BEFORE startServices() to avoid premature notify() during startup
  bleHidDevice->setBatteryLevel((uint8_t)constrain(batteryPercent, 0, 100));
  bleHidDevice->startServices();
  delay(30);

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  BLEService* pHidSvc = bleHidDevice->hidService();
  if (pAdvertising == NULL || pHidSvc == NULL) {
    stopBleHidKeyboard();
    strncpy(btKbLastSent, "BLE Adv Err", sizeof(btKbLastSent) - 1);
    btKbLastSent[sizeof(btKbLastSent) - 1] = '\0';
    if (currentScreen == SCREEN_BT_KEYBOARD || currentScreen == SCREEN_BT_PRESENTER) renderCurrentScreen();
    return;
  }
  pAdvertising->setAppearance(BLE_HID_APPEARANCE_KEYBOARD);
  pAdvertising->addServiceUUID(pHidSvc->getUUID());
  pAdvertising->setScanResponse(true);
#if defined(CONFIG_BLUEDROID_ENABLED)
  pAdvertising->setAdvertisementType(ADV_TYPE_IND);
#endif
  pAdvertising->setMinInterval(0x20);
  pAdvertising->setMaxInterval(0x40);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);

  // Explicit <=31-byte Android/iOS-compliant primary ADV (26B KB / 28B Pres) + scan response (23B KB / 25B Pres):
  // Flags(0x06, 3B) + Appearance(0x03C1, 4B) + 16-bit HID UUID(0x1812, 4B) + Name(15B/17B) <= 31B
  BLEAdvertisementData advData;
  advData.setFlags(0x06);
  advData.setAppearance(BLE_HID_APPEARANCE_KEYBOARD);
  advData.setCompleteServices(BLEUUID((uint16_t)0x1812));

  BLEAdvertisementData scanRespData;
  if (bleHidPresenterMode) {
    advData.setName("T-Watch S3 Pres");
    scanRespData.setName("T-Watch S3 Pres");
  } else {
    advData.setName("T-Watch S3 KB");
    scanRespData.setName("T-Watch S3 KB");
  }
  scanRespData.setAppearance(BLE_HID_APPEARANCE_KEYBOARD);
  scanRespData.setCompleteServices(BLEUUID((uint16_t)0x1812));

  pAdvertising->setAdvertisementData(advData);
  pAdvertising->setScanResponseData(scanRespData);
  delay(20);
  pAdvertising->start();

  bleHidKbActive    = true;
  bleHidKbConnected = false;
  bleHidKbUiDirty   = false;
  bleHidHostLeds    = 0;
  btKbPage          = 0;
  btKbMods          = 0;
  btKbLastSent[0]   = '\0';
  if (currentScreen == SCREEN_BT_KEYBOARD || currentScreen == SCREEN_BT_PRESENTER) {
    renderCurrentScreen();
  }
}

// Send one HID keyboard press + release report over BLE GATT notification
void sendBleHidKeyReport(uint8_t mods, uint8_t hidCode, const char* keyLabel) {
  char combo[24];
  combo[0] = '\0';
  if (mods & BT_KB_MOD_CTRL)  strncat(combo, "Ctrl+", sizeof(combo) - strlen(combo) - 1);
  if (mods & BT_KB_MOD_SHIFT) strncat(combo, "Sh+",   sizeof(combo) - strlen(combo) - 1);
  if (mods & BT_KB_MOD_ALT)   strncat(combo, "Alt+",  sizeof(combo) - strlen(combo) - 1);
  if (mods & BT_KB_MOD_GUI)   strncat(combo, "Win+",  sizeof(combo) - strlen(combo) - 1);
  if (keyLabel != NULL && keyLabel[0] != '\0') {
    strncat(combo, keyLabel, sizeof(combo) - strlen(combo) - 1);
  } else if (strlen(combo) > 0 && combo[strlen(combo) - 1] == '+') {
    combo[strlen(combo) - 1] = '\0';
  }
  strncpy(btKbLastSent, combo, sizeof(btKbLastSent) - 1);
  btKbLastSent[sizeof(btKbLastSent) - 1] = '\0';

  if (bleHidKbActive && bleHidKbConnected && bleHidKbInput != NULL) {
    uint8_t pressRep[8]   = { (uint8_t)(mods & 0x0F), 0x00, hidCode, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t releaseRep[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    bleHidKbInput->setValue(pressRep, sizeof(pressRep));
    bleHidKbInput->notify();
    delay(15);
    bleHidKbInput->setValue(releaseRep, sizeof(releaseRep));
    bleHidKbInput->notify();
  }
}

// Lookup key table pointer and key count for current Bluetooth keyboard page (0..7)
const BtKbKeyDef* getBtKbPageKeys(uint8_t page, uint8_t& outCount) {
  outCount = 12;
  switch (page % BT_KB_PAGE_COUNT) {
    case 0: return BT_KB_PAGE0;
    case 1: outCount = 11; return BT_KB_PAGE1;
    case 2: return BT_KB_PAGE2;
    case 3: return BT_KB_PAGE3;
    case 4: return BT_KB_PAGE4;
    case 5: return BT_KB_PAGE5;
    case 6: return BT_KB_PAGE6;
    default: return BT_KB_PAGE7;
  }
}

// Resolve displayed text for a key considering Shift & host Caps Lock LED
const char* getBtKbKeyDisplayLabel(const BtKbKeyDef& key) {
  if (key.shiftLabel == NULL) return key.label;
  bool shiftOn = ((btKbMods & BT_KB_MOD_SHIFT) != 0);
  bool capsOn  = ((bleHidHostLeds & 0x02) != 0);
  bool useUpper = ((key.flags & BT_KB_FLAG_ALPHA) != 0) ? (shiftOn != capsOn) : shiftOn;
  return useUpper ? key.shiftLabel : key.label;
}

// Top-right page badge (1/8..8/8) for Bluetooth keyboard screen
void drawTopRightBtKbPageIcon() {
  uint16_t borderCol = COLOR_ACCENT;
  if (btKbPage == 3) borderCol = COLOR_GREEN;
  else if (btKbPage >= 4) borderCol = COLOR_WARN;

  canvas.fillRoundRect(204, 2, 32, 24, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(204, 2, 32, 24, 6, borderCol);

  char pageBuf[6];
  snprintf(pageBuf, sizeof(pageBuf), "%u/%u", (unsigned)(btKbPage + 1), (unsigned)BT_KB_PAGE_COUNT);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(borderCol, COLOR_HEX_FILL);
  canvas.drawString(pageBuf, 220, 14);
}

// Render Bluetooth HID Keyboard screen (SCREEN_BT_KEYBOARD)
void drawBluetoothKeyboardScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();
  drawTopRightBtKbPageIcon();

  // Header status & sticky modifier bar (y = 31..65, matches Wi-Fi password display box)
  canvas.fillRoundRect(6, 31, 228, 34, 5, COLOR_TOPBAR);
  canvas.drawRoundRect(6, 31, 228, 34, 5, bleHidKbConnected ? COLOR_GREEN : COLOR_HEX_BORDER);

  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(top_left);
  if (bleHidKbConnected) {
    canvas.setTextColor(COLOR_GREEN, COLOR_TOPBAR);
    canvas.drawString("BLE HID: Connected", 11, 34);
  } else if (bleHidKbStartPending) {
    canvas.setTextColor(COLOR_WARN, COLOR_TOPBAR);
    canvas.drawString("BLE HID: Starting...", 11, 34);
  } else if (bleHidKbActive) {
    canvas.setTextColor(COLOR_WARN, COLOR_TOPBAR);
    canvas.drawString("BLE HID: Pairing", 11, 34);
  } else {
    canvas.setTextColor(COLOR_RED, COLOR_TOPBAR);
    canvas.drawString("BLE HID: Error", 11, 34);
  }

  canvas.setTextDatum(top_right);
  if (btKbLastSent[0] != '\0') {
    canvas.setTextColor(COLOR_ACCENT, COLOR_TOPBAR);
    canvas.drawString(btKbLastSent, 229, 34);
  } else {
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
    canvas.drawString("T-Watch S3 KB", 229, 34);
  }

  // 4 sticky modifier chips inside bottom half of header box: Ctrl | Shift | Alt | Win
  const char* modNames[4] = {"Ctrl", "Shift", "Alt", "Win"};
  const uint8_t modBits[4] = {BT_KB_MOD_CTRL, BT_KB_MOD_SHIFT, BT_KB_MOD_ALT, BT_KB_MOD_GUI};
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font0);
  for (uint8_t m = 0; m < 4; m++) {
    int16_t mx = (int16_t)(10 + m * 56);
    bool active = ((btKbMods & modBits[m]) != 0);
    uint16_t mFill = active ? COLOR_HEX_ACTIVE : COLOR_BG;
    uint16_t mBord = active ? COLOR_WARN : COLOR_HEX_BORDER;
    uint16_t mTxt  = active ? COLOR_WARN : COLOR_SUBTEXT;
    canvas.fillRoundRect(mx, 46, 52, 16, 3, mFill);
    canvas.drawRoundRect(mx, 46, 52, 16, 3, mBord);
    canvas.setTextColor(mTxt, mFill);
    canvas.drawString(modNames[m], mx + 26, 54);
  }

  // Main key grid (y = 69..192, 3 rows of big 54x38 touch keys matching Wi-Fi connect)
  uint8_t keyCount = 0;
  const BtKbKeyDef* keys = getBtKbPageKeys(btKbPage, keyCount);
  canvas.setTextDatum(middle_center);

  for (uint8_t i = 0; i < keyCount; i++) {
    int16_t bx = 0;
    int16_t by = 0;
    int16_t bw = 54;
    if (btKbPage == 1 && i >= 8) {
      bx = (int16_t)(6 + (i - 8) * 77);
      by = 153;
      bw = 74;
    } else {
      bx = (int16_t)(6 + (i % 4) * 57);
      by = (int16_t)(69 + (i / 4) * 42);
      bw = 54;
    }

    const BtKbKeyDef& k = keys[i];
    const char* dispLbl = getBtKbKeyDisplayLabel(k);

    bool lockLedOn = false;
    if (k.hidCode == 0x39 && (bleHidHostLeds & 0x02)) lockLedOn = true; // Caps Lock LED
    if (k.hidCode == 0x53 && (bleHidHostLeds & 0x01)) lockLedOn = true; // Num Lock LED
    if (k.hidCode == 0x47 && (bleHidHostLeds & 0x04)) lockLedOn = true; // Scroll Lock LED

    uint16_t kFill = lockLedOn ? COLOR_HEX_ACTIVE : COLOR_HEX_FILL;
    uint16_t kBord = COLOR_HEX_BORDER;
    uint16_t kTxt  = COLOR_TEXT;
    if (lockLedOn) {
      kBord = COLOR_GREEN;
      kTxt  = COLOR_GREEN;
    } else if (k.flags & BT_KB_FLAG_FN) {
      kBord = COLOR_ACCENT;
      kTxt  = COLOR_ACCENT;
    } else if (k.flags & BT_KB_FLAG_NAV) {
      kBord = COLOR_WARN;
      kTxt  = COLOR_TEXT;
    } else if (k.flags & BT_KB_FLAG_LOCK) {
      kBord = COLOR_HEX_BORDER;
      kTxt  = COLOR_WARN;
    }

    canvas.fillRoundRect(bx, by, bw, 38, 5, kFill);
    canvas.drawRoundRect(bx, by, bw, 38, 5, kBord);

    if (strlen(dispLbl) >= 4) {
      canvas.setFont(&fonts::Font0);
    } else {
      canvas.setFont(&fonts::Font2);
    }
    canvas.setTextColor(kTxt, kFill);
    canvas.drawString(dispLbl, bx + bw / 2, by + 19);
  }

  // Bottom control row (y = 195..233): Shift | Space | <- | Enter
  bool shiftOn = ((btKbMods & BT_KB_MOD_SHIFT) != 0);
  uint16_t shiftFill = shiftOn ? COLOR_HEX_ACTIVE : COLOR_HEX_FILL;
  uint16_t shiftBord = shiftOn ? COLOR_WARN : COLOR_ACCENT;
  canvas.fillRoundRect(6, 195, 54, 38, 5, shiftFill);
  canvas.drawRoundRect(6, 195, 54, 38, 5, shiftBord);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(shiftOn ? COLOR_WARN : COLOR_TEXT, shiftFill);
  canvas.drawString(shiftOn ? "ABC" : "abc", 33, 214);

  canvas.fillRoundRect(63, 195, 54, 38, 5, COLOR_HEX_FILL);
  canvas.drawRoundRect(63, 195, 54, 38, 5, COLOR_HEX_BORDER);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
  canvas.drawString("Space", 90, 214);

  canvas.fillRoundRect(120, 195, 54, 38, 5, COLOR_HEX_FILL);
  canvas.drawRoundRect(120, 195, 54, 38, 5, COLOR_RED);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_RED, COLOR_HEX_FILL);
  canvas.drawString("<-", 147, 214);

  canvas.fillRoundRect(177, 195, 57, 38, 5, COLOR_HEX_ACTIVE);
  canvas.drawRoundRect(177, 195, 57, 38, 5, COLOR_GREEN);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_GREEN, COLOR_HEX_ACTIVE);
  canvas.drawString("Enter", 205, 214);
}

// ============================================================================
// 7.11B2. BLUETOOTH HID PRESENTER (SCREEN_BT_PRESENTER)
// Single-screen presentation clicker: Left/Right, Page Up/Down, F5, Shift+F5, Esc, B, .
// ============================================================================

#define BT_PRES_BTN_COUNT  9
#define BT_PRES_STYLE_NAV  0
#define BT_PRES_STYLE_NEXT 1
#define BT_PRES_STYLE_PLAY 2
#define BT_PRES_STYLE_ESC  3
#define BT_PRES_STYLE_DIM  4

static const BtPresenterBtn BT_PRESENTER_BTNS[BT_PRES_BTN_COUNT] = {
  {  6,  61, 111, 43, "< Left",     "Left",  0x00,            0x50, BT_PRES_STYLE_NAV  },
  {123,  61, 111, 43, "Right >",    "Right", 0x00,            0x4F, BT_PRES_STYLE_NEXT },
  {  6, 109, 111, 39, "Page Up",    "PgUp",  0x00,            0x4B, BT_PRES_STYLE_NAV  },
  {123, 109, 111, 39, "Page Down",  "PgDn",  0x00,            0x4E, BT_PRES_STYLE_NAV  },
  {  6, 153, 111, 39, "F5",         "F5",    0x00,            0x3E, BT_PRES_STYLE_PLAY },
  {123, 153, 111, 39, "Shift + F5", "F5",    BT_KB_MOD_SHIFT, 0x3E, BT_PRES_STYLE_PLAY },
  {  6, 197,  73, 38, "Esc",        "Esc",   0x00,            0x29, BT_PRES_STYLE_ESC  },
  { 84, 197,  73, 38, "B",          "B",     0x00,            0x05, BT_PRES_STYLE_DIM  },
  {162, 197,  72, 38, ".",          ".",     0x00,            0x37, BT_PRES_STYLE_DIM  }
};

// Render single-screen Bluetooth HID Presenter (SCREEN_BT_PRESENTER)
void drawBluetoothPresenterScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  // Status banner (y = 31..56): BLE HID connection state + last sent presenter key
  canvas.fillRoundRect(6, 31, 228, 25, 5, COLOR_TOPBAR);
  canvas.drawRoundRect(6, 31, 228, 25, 5, bleHidKbConnected ? COLOR_GREEN : COLOR_HEX_BORDER);

  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(middle_left);
  if (bleHidKbConnected) {
    canvas.setTextColor(COLOR_GREEN, COLOR_TOPBAR);
    canvas.drawString("BLE HID: Connected", 11, 43);
  } else if (bleHidKbStartPending) {
    canvas.setTextColor(COLOR_WARN, COLOR_TOPBAR);
    canvas.drawString("BLE HID: Starting...", 11, 43);
  } else if (bleHidKbActive) {
    canvas.setTextColor(COLOR_WARN, COLOR_TOPBAR);
    canvas.drawString("BLE HID: Pairing", 11, 43);
  } else {
    canvas.setTextColor(COLOR_RED, COLOR_TOPBAR);
    canvas.drawString("BLE HID: Error", 11, 43);
  }

  canvas.setTextDatum(middle_right);
  if (btKbLastSent[0] != '\0') {
    canvas.setTextColor(COLOR_ACCENT, COLOR_TOPBAR);
    canvas.drawString(btKbLastSent, 229, 43);
  } else {
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
    canvas.drawString("T-Watch S3 Pres", 229, 43);
  }

  // 9 large single-screen presenter buttons (4 ergonomic rows: y = 61..235)
  canvas.setTextDatum(middle_center);
  for (uint8_t i = 0; i < BT_PRES_BTN_COUNT; i++) {
    const BtPresenterBtn& b = BT_PRESENTER_BTNS[i];
    uint16_t bFill = COLOR_HEX_FILL;
    uint16_t bBord = COLOR_ACCENT;
    uint16_t bTxt  = COLOR_TEXT;

    if (b.style == BT_PRES_STYLE_NEXT) {
      bFill = COLOR_HEX_ACTIVE;
      bBord = COLOR_GREEN;
      bTxt  = COLOR_GREEN;
    } else if (b.style == BT_PRES_STYLE_PLAY) {
      bFill = COLOR_HEX_ACTIVE;
      bBord = COLOR_WARN;
      bTxt  = COLOR_WARN;
    } else if (b.style == BT_PRES_STYLE_ESC) {
      bFill = COLOR_HEX_FILL;
      bBord = COLOR_RED;
      bTxt  = COLOR_RED;
    } else if (b.style == BT_PRES_STYLE_DIM) {
      bFill = COLOR_HEX_FILL;
      bBord = COLOR_HEX_BORDER;
      bTxt  = COLOR_TEXT;
    }

    canvas.fillRoundRect(b.x, b.y, b.w, b.h, 6, bFill);
    canvas.drawRoundRect(b.x, b.y, b.w, b.h, 6, bBord);

    if (i == 8) {
      canvas.setFont(&fonts::Font4);
      canvas.setTextColor(bTxt, bFill);
      canvas.drawString(b.label, b.x + b.w / 2, b.y + b.h / 2 - 4);
    } else {
      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(bTxt, bFill);
      canvas.drawString(b.label, b.x + b.w / 2, b.y + b.h / 2);
    }
  }
}

// ============================================================================
// 7.11C. BLUETOOTH HID MOUSE (SCREEN_BT_MOUSE)
// BMA423 accelerometer cursor + Left / Double-Left / Middle / Right + Scroll Wheel
// ============================================================================

#define BLE_HID_APPEARANCE_MOUSE    0x03C2
#define BT_MOUSE_BTN_LEFT           0x01
#define BT_MOUSE_BTN_RIGHT          0x02
#define BT_MOUSE_BTN_MIDDLE         0x04
#define BT_MOUSE_DEADZONE_DEG       5.0f
#define BT_MOUSE_REPORT_INTERVAL_MS 25UL
#define BT_MOUSE_UI_INTERVAL_MS     80UL
#define BT_MOUSE_WHEEL_DRAG_STEP_PX 12

// Standard USB HID 1.11 Mouse Report Descriptor (52 bytes: 3 buttons + X + Y + Wheel, Report ID 1)
static const uint8_t BLE_HID_MOUSE_REPORT_MAP[] = {
  0x05, 0x01,       // Usage Page (Generic Desktop)
  0x09, 0x02,       // Usage (Mouse)
  0xA1, 0x01,       // Collection (Application)
  0x85, 0x01,       //   Report ID (1)
  0x09, 0x01,       //   Usage (Pointer)
  0xA1, 0x00,       //   Collection (Physical)
  0x05, 0x09,       //     Usage Page (Buttons)
  0x19, 0x01,       //     Usage Minimum (1 - Left)
  0x29, 0x03,       //     Usage Maximum (3 - Middle)
  0x15, 0x00,       //     Logical Minimum (0)
  0x25, 0x01,       //     Logical Maximum (1)
  0x95, 0x03,       //     Report Count (3)
  0x75, 0x01,       //     Report Size (1)
  0x81, 0x02,       //     Input (Data, Variable, Absolute)
  0x95, 0x01,       //     Report Count (1)
  0x75, 0x05,       //     Report Size (5)
  0x81, 0x01,       //     Input (Constant) - 5-bit button padding
  0x05, 0x01,       //     Usage Page (Generic Desktop)
  0x09, 0x30,       //     Usage (X)
  0x09, 0x31,       //     Usage (Y)
  0x09, 0x38,       //     Usage (Wheel)
  0x15, 0x81,       //     Logical Minimum (-127)
  0x25, 0x7F,       //     Logical Maximum (+127)
  0x75, 0x08,       //     Report Size (8)
  0x95, 0x03,       //     Report Count (3)
  0x81, 0x06,       //     Input (Data, Variable, Relative)
  0xC0,             //   End Collection (Physical)
  0xC0              // End Collection (Application)
};

BLEServer* bleHidMouseServer          = NULL;
BLEHIDDevice* bleHidMouseDevice       = NULL;
BLECharacteristic* bleHidMouseInput   = NULL;
volatile bool bleHidMouseConnected    = false;
volatile bool bleHidMouseUiDirty      = false;
bool btMouseWifiPaused                = false;
bool btMouseWifiWasConnected          = false;
float btMouseZeroX                    = 0.0f;
float btMouseZeroY                    = 0.0f;
int8_t btMouseLastDx                  = 0;
int8_t btMouseLastDy                  = 0;
int8_t btMouseLastWheel               = 0;
uint8_t btMouseLastButtons            = 0;
char btMouseLastAction[20]            = "Ready";
uint32_t lastBtMouseReportMs          = 0;
uint32_t lastBtMouseUiDrawMs          = 0;
bool btMouseWheelDragging             = false;
int16_t btMouseWheelLastY             = -1;
int16_t btMouseWheelAccumY            = 0;
bool btMouseWheelDraggedSteps         = false;
int16_t btMouseWheelVisualPos         = 0;
int32_t btMouseWheelTotal             = 0;

class BleHidMouseServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    (void)pServer;
    bleHidMouseConnected = true;
#if defined(CONFIG_BLUEDROID_ENABLED)
    if (bleHidMouseInput != NULL) {
      BLE2902* desc = (BLE2902*)bleHidMouseInput->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
      if (desc != NULL) {
        desc->setNotifications(true);
      }
    }
#endif
    bleHidMouseUiDirty = true;
  }

  void onDisconnect(BLEServer* pServer) {
    bleHidMouseConnected = false;
    bleHidMouseUiDirty = true;
    if (bleHidMouseActive && pServer != NULL) {
      pServer->startAdvertising();
    }
  }
};

static BleHidMouseServerCallbacks bleHidMouseServerCb;
static BLESecurity bleHidMouseSecurity;

// Convert relative BMA423 tilt angle (degrees) into smooth signed HID mouse delta
int8_t computeBtMouseAxisDelta(float tiltDeg) {
  float absTilt = fabsf(tiltDeg);
  if (absTilt <= BT_MOUSE_DEADZONE_DEG) return 0;
  float eff = absTilt - BT_MOUSE_DEADZONE_DEG;
  if (eff > 40.0f) eff = 40.0f;
  float speed = eff * 0.28f + eff * eff * 0.008f;
  int16_t delta = (int16_t)lroundf(speed);
  if (delta < 1) delta = 1;
  if (delta > 24) delta = 24;
  return (tiltDeg >= 0.0f) ? (int8_t)delta : (int8_t)(-delta);
}

// Re-center BMA423 accelerometer neutral angle for cursor movement
void recenterBtMouseTilt() {
  btMouseZeroX = inclAngleX;
  btMouseZeroY = inclAngleY;
  btMouseLastDx = 0;
  btMouseLastDy = 0;
  strncpy(btMouseLastAction, "Zeroed", sizeof(btMouseLastAction) - 1);
  btMouseLastAction[sizeof(btMouseLastAction) - 1] = '\0';
}

// Send raw 4-byte Bluetooth HID Mouse report: [buttons, dx, dy, wheel]
void sendBleHidMouseReport(uint8_t buttons, int8_t dx, int8_t dy, int8_t wheel) {
  btMouseLastButtons = buttons;
  btMouseLastDx      = dx;
  btMouseLastDy      = dy;
  btMouseLastWheel   = wheel;
  if (bleHidMouseActive && bleHidMouseConnected && bleHidMouseInput != NULL) {
    uint8_t rep[4] = {
      (uint8_t)(buttons & 0x07),
      (uint8_t)dx,
      (uint8_t)dy,
      (uint8_t)wheel
    };
    bleHidMouseInput->setValue(rep, sizeof(rep));
    bleHidMouseInput->notify();
  }
}

// Send single mouse button click (Left / Middle / Right)
void sendBleHidMouseClick(uint8_t buttonMask, const char* label) {
  strncpy(btMouseLastAction, label, sizeof(btMouseLastAction) - 1);
  btMouseLastAction[sizeof(btMouseLastAction) - 1] = '\0';
  sendBleHidMouseReport(buttonMask, 0, 0, 0);
  delay(18);
  sendBleHidMouseReport(0x00, 0, 0, 0);
}

// Send double Left mouse click (two crisp press-release pulses)
void sendBleHidMouseDoubleClickLeft() {
  strncpy(btMouseLastAction, "2x Left", sizeof(btMouseLastAction) - 1);
  btMouseLastAction[sizeof(btMouseLastAction) - 1] = '\0';
  sendBleHidMouseReport(BT_MOUSE_BTN_LEFT, 0, 0, 0);
  delay(18);
  sendBleHidMouseReport(0x00, 0, 0, 0);
  delay(65);
  sendBleHidMouseReport(BT_MOUSE_BTN_LEFT, 0, 0, 0);
  delay(18);
  sendBleHidMouseReport(0x00, 0, 0, 0);
}

// Send scroll wheel step (+1 = up, -1 = down) and advance visual wheel ridges
void sendBleHidMouseWheel(int8_t wheelStep) {
  if (wheelStep == 0) return;
  btMouseWheelTotal += wheelStep;
  btMouseWheelVisualPos = (int16_t)((btMouseWheelVisualPos - wheelStep * 3) % 12);
  if (btMouseWheelVisualPos < 0) btMouseWheelVisualPos += 12;
  snprintf(btMouseLastAction, sizeof(btMouseLastAction),
           (wheelStep > 0) ? "Wheel +%d" : "Wheel %d", (int)wheelStep);
  sendBleHidMouseReport(0x00, 0, 0, wheelStep);
}

// Disconnect host, stop advertising, deinit BLE HID Mouse, and restore Wi-Fi STA
void stopBleHidMouse() {
  bleHidMouseStartPending  = false;
  bleHidMouseActive        = false;
  btMouseWheelDragging     = false;
  btMouseWheelLastY        = -1;
  btMouseWheelAccumY       = 0;
  btMouseWheelDraggedSteps = false;

  if (bleHidMouseConnected && bleHidMouseInput != NULL) {
    uint8_t releaseRep[4] = {0, 0, 0, 0};
    bleHidMouseInput->setValue(releaseRep, sizeof(releaseRep));
    bleHidMouseInput->notify();
    delay(15);
  }

  if (BLEDevice::getInitialized()) {
    BLEDevice::stopAdvertising();
    if (bleHidMouseServer != NULL && bleHidMouseServer->getConnectedCount() > 0) {
      bleHidMouseServer->disconnect(bleHidMouseServer->getConnId());
      delay(60);
    }
  }

  if (bleHidMouseServer != NULL) {
    bleHidMouseServer->setCallbacks(NULL);
  }
  bleHidMouseServer    = NULL;
  bleHidMouseInput     = NULL;
  bleHidMouseConnected = false;
  bleHidMouseUiDirty   = false;
  btMouseLastButtons   = 0;
  btMouseLastDx        = 0;
  btMouseLastDy        = 0;
  btMouseLastWheel     = 0;

  if (BLEDevice::getInitialized()) {
    BLEDevice::deinit(false);
    delay(80);
  }
  bleStackInitialized = false;

  if (bleHidMouseDevice != NULL) {
    delete bleHidMouseDevice;
    bleHidMouseDevice = NULL;
  }

  if (btMouseWifiPaused) {
    btMouseWifiPaused = false;
    if (settingWifiEnabled) {
      WiFi.mode(WIFI_STA);
      WiFi.setSleep(false);
      delay(40);
      if (btMouseWifiWasConnected) {
        btMouseWifiWasConnected = false;
        WiFi.reconnect();
      }
    } else {
      btMouseWifiWasConnected = false;
    }
  }
}

// Schedule deferred BLE HID Mouse bring-up after stopping touch vibration and painting UI
void requestBleHidMouseStart() {
  stopTouchVibrationNow();
  bleHidMouseActive        = false;
  bleHidMouseConnected     = false;
  bleHidMouseUiDirty       = false;
  btMouseWheelDragging     = false;
  btMouseWheelLastY        = -1;
  btMouseWheelAccumY       = 0;
  btMouseWheelDraggedSteps = false;
  btMouseWheelVisualPos    = 0;
  btMouseWheelTotal        = 0;
  btMouseLastButtons       = 0;
  btMouseLastDx            = 0;
  btMouseLastDy            = 0;
  btMouseLastWheel         = 0;
  btMouseZeroX             = inclAngleX;
  btMouseZeroY             = inclAngleY;
  strncpy(btMouseLastAction, "Starting...", sizeof(btMouseLastAction) - 1);
  btMouseLastAction[sizeof(btMouseLastAction) - 1] = '\0';
  bleHidMouseStartPending  = true;
}

// Start Bluetooth HID Mouse mode (pauses Wi-Fi for DRAM/RF safety, full NULL guards)
void startBleHidMouse() {
  bleHidMouseStartPending = false;
  stopTouchVibrationNow();
  if (currentScreen != SCREEN_BT_MOUSE) return;

  stopBluetoothScan();
  if (bleHidKbActive || bleHidKbStartPending) {
    stopBleHidKeyboard();
  }
  if (wifiBgScanning) {
    WiFi.scanDelete();
    wifiBgScanning = false;
  }

  // Pause Wi-Fi STA while BLE HID is active: frees ~65 KB internal DRAM and avoids radio/coex aborts
  if (!btMouseWifiPaused) {
    btMouseWifiWasConnected = (WiFi.status() == WL_CONNECTED);
    WiFi.disconnect(false, false);
    WiFi.mode(WIFI_OFF);
    btMouseWifiPaused = true;
    delay(50);
  }

  if (BLEDevice::getInitialized()) {
    BLEDevice::stopAdvertising();
    BLEDevice::deinit(false);
    delay(80);
  }

  BLEDevice::init("T-Watch S3 Mouse");
  delay(50);
  bleStackInitialized = BLEDevice::getInitialized();
  if (!bleStackInitialized) {
    stopBleHidMouse();
    strncpy(btMouseLastAction, "BLE Init Err", sizeof(btMouseLastAction) - 1);
    btMouseLastAction[sizeof(btMouseLastAction) - 1] = '\0';
    if (currentScreen == SCREEN_BT_MOUSE) renderCurrentScreen();
    return;
  }

  // Enable LE Secure Connections + Bonding required by Android 13/14, Windows, iOS, macOS, and Linux HID hosts
#if defined(ESP_LE_AUTH_REQ_SC_BOND)
  bleHidMouseSecurity.setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
#else
  bleHidMouseSecurity.setAuthenticationMode(ESP_LE_AUTH_BOND);
#endif
  bleHidMouseSecurity.setCapability(ESP_IO_CAP_NONE);
  bleHidMouseSecurity.setKeySize(16);
  bleHidMouseSecurity.setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  bleHidMouseSecurity.setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
#if defined(CONFIG_BLUEDROID_ENABLED)
  esp_ble_gap_config_local_icon(BLE_HID_APPEARANCE_MOUSE);
#endif

  bleHidMouseServer = BLEDevice::createServer();
  if (bleHidMouseServer == NULL) {
    stopBleHidMouse();
    strncpy(btMouseLastAction, "BLE Srv Err", sizeof(btMouseLastAction) - 1);
    btMouseLastAction[sizeof(btMouseLastAction) - 1] = '\0';
    if (currentScreen == SCREEN_BT_MOUSE) renderCurrentScreen();
    return;
  }
  bleHidMouseServer->setCallbacks(&bleHidMouseServerCb);

  bleHidMouseDevice = new BLEHIDDevice(bleHidMouseServer);
  if (bleHidMouseDevice == NULL) {
    stopBleHidMouse();
    strncpy(btMouseLastAction, "BLE HID Err", sizeof(btMouseLastAction) - 1);
    btMouseLastAction[sizeof(btMouseLastAction) - 1] = '\0';
    if (currentScreen == SCREEN_BT_MOUSE) renderCurrentScreen();
    return;
  }

  bleHidMouseInput = bleHidMouseDevice->inputReport(1);
  if (bleHidMouseInput == NULL) {
    stopBleHidMouse();
    strncpy(btMouseLastAction, "BLE Rep Err", sizeof(btMouseLastAction) - 1);
    btMouseLastAction[sizeof(btMouseLastAction) - 1] = '\0';
    if (currentScreen == SCREEN_BT_MOUSE) renderCurrentScreen();
    return;
  }

  // Version-agnostic raw buffer setValue avoids Core 2.x std::string vs Core 3.x String conflicts
  BLECharacteristic* pMfg = bleHidMouseDevice->manufacturer();
  if (pMfg != NULL) {
    pMfg->setValue((uint8_t*)"VRXFILE", 7);
  }
  bleHidMouseDevice->pnp(0x02, 0xE502, 0xA112, 0x0210);
  bleHidMouseDevice->hidInfo(0x00, 0x01);
  bleHidMouseDevice->reportMap((uint8_t*)BLE_HID_MOUSE_REPORT_MAP, sizeof(BLE_HID_MOUSE_REPORT_MAP));
  // Set initial battery level BEFORE startServices() to avoid premature notify() during startup
  bleHidMouseDevice->setBatteryLevel((uint8_t)constrain(batteryPercent, 0, 100));
  bleHidMouseDevice->startServices();
  delay(30);

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  BLEService* pHidSvc = bleHidMouseDevice->hidService();
  if (pAdvertising == NULL || pHidSvc == NULL) {
    stopBleHidMouse();
    strncpy(btMouseLastAction, "BLE Adv Err", sizeof(btMouseLastAction) - 1);
    btMouseLastAction[sizeof(btMouseLastAction) - 1] = '\0';
    if (currentScreen == SCREEN_BT_MOUSE) renderCurrentScreen();
    return;
  }
  pAdvertising->setAppearance(BLE_HID_APPEARANCE_MOUSE);
  pAdvertising->addServiceUUID(pHidSvc->getUUID());
  pAdvertising->setScanResponse(true);
#if defined(CONFIG_BLUEDROID_ENABLED)
  pAdvertising->setAdvertisementType(ADV_TYPE_IND);
#endif
  pAdvertising->setMinInterval(0x20);
  pAdvertising->setMaxInterval(0x40);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);

  // Explicit <=31-byte Android/iOS-compliant primary ADV (29 bytes) + scan response (26 bytes):
  // Flags(0x06, 3B) + Appearance(0x03C2, 4B) + 16-bit HID UUID(0x1812, 4B) + Name("T-Watch S3 Mouse", 18B) = 29B
  BLEAdvertisementData advData;
  advData.setFlags(0x06);
  advData.setAppearance(BLE_HID_APPEARANCE_MOUSE);
  advData.setCompleteServices(BLEUUID((uint16_t)0x1812));
  advData.setName("T-Watch S3 Mouse");

  BLEAdvertisementData scanRespData;
  scanRespData.setName("T-Watch S3 Mouse");
  scanRespData.setAppearance(BLE_HID_APPEARANCE_MOUSE);
  scanRespData.setCompleteServices(BLEUUID((uint16_t)0x1812));

  pAdvertising->setAdvertisementData(advData);
  pAdvertising->setScanResponseData(scanRespData);
  delay(20);
  pAdvertising->start();

  bleHidMouseActive        = true;
  bleHidMouseConnected     = false;
  bleHidMouseUiDirty       = false;
  btMouseZeroX             = inclAngleX;
  btMouseZeroY             = inclAngleY;
  lastBtMouseReportMs      = millis();
  lastBtMouseUiDrawMs      = millis();
  strncpy(btMouseLastAction, "Tilt to move", sizeof(btMouseLastAction) - 1);
  btMouseLastAction[sizeof(btMouseLastAction) - 1] = '\0';
  if (currentScreen == SCREEN_BT_MOUSE) {
    renderCurrentScreen();
  }
}

// Periodic 40 Hz BMA423 accelerometer-to-HID-mouse cursor update
void updateBluetoothMouse() {
  if (!bleHidMouseActive || currentScreen != SCREEN_BT_MOUSE) return;

  uint32_t now = millis();
  if (now - lastBtMouseReportMs >= BT_MOUSE_REPORT_INTERVAL_MS) {
    lastBtMouseReportMs = now;
    int8_t dx = 0;
    int8_t dy = 0;
    if (bma423Ready) {
      dx = computeBtMouseAxisDelta(inclAngleX - btMouseZeroX);
      dy = computeBtMouseAxisDelta(inclAngleY - btMouseZeroY);
    }
    btMouseLastDx = dx;
    btMouseLastDy = dy;

    if (dx != 0 || dy != 0) {
      lastTouchMs = now; // Keep display awake while actively tilting the mouse
      if (bleHidMouseConnected) {
        sendBleHidMouseReport(0x00, dx, dy, 0);
      }
      bleHidMouseUiDirty = true;
    }
  }

  if (bleHidMouseUiDirty && blState != BL_STATE_OFF && (now - lastBtMouseUiDrawMs >= BT_MOUSE_UI_INTERVAL_MS)) {
    lastBtMouseUiDrawMs = now;
    bleHidMouseUiDirty  = false;
    renderCurrentScreen();
  }
}

// Top-right ZERO calibration button for Bluetooth mouse BMA423 neutral tilt
void drawTopRightBtMouseZeroIcon() {
  canvas.fillRoundRect(202, 2, 34, 24, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(202, 2, 34, 24, 6, COLOR_ACCENT);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_ACCENT, COLOR_HEX_FILL);
  canvas.drawString("ZERO", 219, 14);
}

// Render Bluetooth HID Mouse screen (SCREEN_BT_MOUSE)
void drawBluetoothMouseScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();
  drawTopRightBtMouseZeroIcon();

  // Status & live BMA423 tilt telemetry card (y = 31..65)
  canvas.fillRoundRect(6, 31, 228, 34, 5, COLOR_TOPBAR);
  canvas.drawRoundRect(6, 31, 228, 34, 5, bleHidMouseConnected ? COLOR_GREEN : COLOR_HEX_BORDER);

  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(top_left);
  if (bleHidMouseConnected) {
    canvas.setTextColor(COLOR_GREEN, COLOR_TOPBAR);
    canvas.drawString("BLE HID: Connected", 11, 35);
  } else if (bleHidMouseStartPending) {
    canvas.setTextColor(COLOR_WARN, COLOR_TOPBAR);
    canvas.drawString("BLE HID: Starting...", 11, 35);
  } else if (bleHidMouseActive) {
    canvas.setTextColor(COLOR_WARN, COLOR_TOPBAR);
    canvas.drawString("BLE HID: Pairing", 11, 35);
  } else {
    canvas.setTextColor(COLOR_RED, COLOR_TOPBAR);
    canvas.drawString("BLE HID: Error", 11, 35);
  }

  char accelBuf[26];
  if (bma423Ready) {
    snprintf(accelBuf, sizeof(accelBuf), "BMA423 dx:%+d dy:%+d", (int)btMouseLastDx, (int)btMouseLastDy);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
  } else {
    strncpy(accelBuf, "BMA423: Not found", sizeof(accelBuf) - 1);
    accelBuf[sizeof(accelBuf) - 1] = '\0';
    canvas.setTextColor(COLOR_RED, COLOR_TOPBAR);
  }
  canvas.drawString(accelBuf, 11, 50);

  // Last mouse action readout (left of tilt radar)
  canvas.setTextDatum(top_right);
  canvas.setTextColor(COLOR_ACCENT, COLOR_TOPBAR);
  canvas.drawString(btMouseLastAction, 198, 35);

  char whlBuf[14];
  snprintf(whlBuf, sizeof(whlBuf), "W:%+ld", (long)btMouseWheelTotal);
  canvas.setTextColor(COLOR_WARN, COLOR_TOPBAR);
  canvas.drawString(whlBuf, 198, 50);

  // Mini live BMA423 tilt crosshair indicator (cx = 217, cy = 48, r = 13)
  int16_t rcx = 217, rcy = 48, rr = 13;
  canvas.drawCircle(rcx, rcy, rr, COLOR_HEX_BORDER);
  canvas.drawFastHLine(rcx - 9, rcy, 19, COLOR_HEX_FILL);
  canvas.drawFastVLine(rcx, rcy - 9, 19, COLOR_HEX_FILL);
  float relX = bma423Ready ? constrain(inclAngleX - btMouseZeroX, -35.0f, 35.0f) : 0.0f;
  float relY = bma423Ready ? constrain(inclAngleY - btMouseZeroY, -35.0f, 35.0f) : 0.0f;
  int16_t dotX = (int16_t)(rcx + (int16_t)lroundf((relX / 35.0f) * 9.0f));
  int16_t dotY = (int16_t)(rcy + (int16_t)lroundf((relY / 35.0f) * 9.0f));
  uint16_t dotCol = (btMouseLastDx != 0 || btMouseLastDy != 0) ? COLOR_GREEN : COLOR_ACCENT;
  canvas.fillCircle(dotX, dotY, 3, dotCol);

  // Left Column (x = 6..84): Single Left Click (top) & Double Left Click (bottom)
  canvas.fillRoundRect(6, 70, 78, 79, 8, COLOR_HEX_FILL);
  canvas.drawRoundRect(6, 70, 78, 79, 8, COLOR_ACCENT);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
  canvas.drawString("L CLICK", 45, 102);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_ACCENT, COLOR_HEX_FILL);
  canvas.drawString("1x LEFT", 45, 124);

  canvas.fillRoundRect(6, 154, 78, 79, 8, COLOR_HEX_ACTIVE);
  canvas.drawRoundRect(6, 154, 78, 79, 8, COLOR_GREEN);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_GREEN, COLOR_HEX_ACTIVE);
  canvas.drawString("2x L", 45, 186);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_TEXT, COLOR_HEX_ACTIVE);
  canvas.drawString("DBL CLICK", 45, 208);

  // Center Column (x = 90..150, y = 70..233): Interactive Scroll Wheel (drag or tap up/down)
  canvas.fillRoundRect(90, 70, 60, 163, 12, COLOR_TOPBAR);
  canvas.drawRoundRect(90, 70, 60, 163, 12, btMouseWheelDragging ? COLOR_GREEN : COLOR_ACCENT);

  // Upper scroll-up cap (y = 70..103)
  canvas.fillTriangle(120, 78, 107, 94, 133, 94, COLOR_ACCENT);
  canvas.drawFastHLine(96, 103, 48, COLOR_HEX_BORDER);

  // Ribbed cylindrical scroll wheel drum (y = 106..196)
  canvas.fillRoundRect(97, 107, 46, 88, 8, COLOR_HEX_FILL);
  canvas.drawRoundRect(97, 107, 46, 88, 8, COLOR_HEX_BORDER);
  for (int16_t ry = (int16_t)(111 + (btMouseWheelVisualPos % 12)); ry <= 190; ry += 12) {
    uint16_t ridgeCol = (ry > 134 && ry < 168) ? COLOR_ACCENT : COLOR_SUBTEXT;
    canvas.drawFastHLine(104, ry, 32, ridgeCol);
    canvas.drawFastHLine(106, ry + 1, 28, COLOR_BG);
  }
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
  canvas.drawString("WHL", 120, 151);

  // Lower scroll-down cap (y = 199..233)
  canvas.drawFastHLine(96, 199, 48, COLOR_HEX_BORDER);
  canvas.fillTriangle(120, 224, 107, 208, 133, 208, COLOR_ACCENT);

  // Right Column (x = 156..234): Single Right Click (top) & Single Middle Click (bottom)
  canvas.fillRoundRect(156, 70, 78, 79, 8, COLOR_HEX_FILL);
  canvas.drawRoundRect(156, 70, 78, 79, 8, COLOR_ACCENT);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
  canvas.drawString("R CLICK", 195, 102);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_ACCENT, COLOR_HEX_FILL);
  canvas.drawString("1x RIGHT", 195, 124);

  canvas.fillRoundRect(156, 154, 78, 79, 8, COLOR_HEX_FILL);
  canvas.drawRoundRect(156, 154, 78, 79, 8, COLOR_WARN);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_WARN, COLOR_HEX_FILL);
  canvas.drawString("M CLICK", 195, 186);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
  canvas.drawString("1x MID", 195, 208);
}

// ============================================================================
// 7.11D. COLOR TABLE (SCREEN_COLOR_TABLE: RGB / CMYK + HEX)
// ============================================================================

#define COLOR_TABLE_COUNT   32
#define COLOR_TABLE_VISIBLE 6

static const ColorTableEntry COLOR_TABLE_DATA[COLOR_TABLE_COUNT] = {
  {"Red",        255,   0,   0},
  {"Crimson",    220,  20,  60},
  {"Coral",      255, 127,  80},
  {"Orange",     255, 165,   0},
  {"Gold",       255, 215,   0},
  {"Yellow",     255, 255,   0},
  {"Chartreuse", 127, 255,   0},
  {"Lime",         0, 255,   0},
  {"Green",        0, 128,   0},
  {"Emerald",     46, 204, 113},
  {"Spring",       0, 255, 127},
  {"Teal",         0, 128, 128},
  {"Cyan",         0, 255, 255},
  {"Sky Blue",   135, 206, 235},
  {"Azure",        0, 127, 255},
  {"Blue",         0,   0, 255},
  {"Navy",         0,   0, 128},
  {"Indigo",      75,   0, 130},
  {"Violet",     138,  43, 226},
  {"Purple",     128,   0, 128},
  {"Magenta",    255,   0, 255},
  {"Deep Pink",  255,  20, 147},
  {"Pink",       255, 192, 203},
  {"Salmon",     250, 128, 114},
  {"Brown",      165,  42,  42},
  {"Maroon",     128,   0,   0},
  {"Olive",      128, 128,   0},
  {"Beige",      245, 245, 220},
  {"White",      255, 255, 255},
  {"Silver",     192, 192, 192},
  {"Gray",       128, 128, 128},
  {"Black",        0,   0,   0}
};

bool colorTableCmykMode    = false;
int16_t colorTableScroll   = 0;
int16_t colorTableSelected = -1;

// Convert 8-bit RGB888 to 16-bit RGB565 for LovyanGFX sprite
uint16_t colorRgb888To565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((uint16_t)(r & 0xF8) << 8) | ((uint16_t)(g & 0xFC) << 3) | ((uint16_t)(b >> 3)));
}

// Convert RGB888 (0..255) to CMYK percentages (0..100)
void colorRgbToCmyk(uint8_t r, uint8_t g, uint8_t b, uint8_t &c, uint8_t &m, uint8_t &y, uint8_t &k) {
  if (r == 0 && g == 0 && b == 0) {
    c = 0; m = 0; y = 0; k = 100;
    return;
  }
  uint8_t maxV = r;
  if (g > maxV) maxV = g;
  if (b > maxV) maxV = b;
  float rf = (float)r / 255.0f;
  float gf = (float)g / 255.0f;
  float bf = (float)b / 255.0f;
  float kf = 1.0f - ((float)maxV / 255.0f);
  float denom = 1.0f - kf;
  if (denom <= 0.0001f) {
    c = 0; m = 0; y = 0; k = 100;
    return;
  }
  c = (uint8_t)constrain((int)lroundf(((1.0f - rf - kf) / denom) * 100.0f), 0, 100);
  m = (uint8_t)constrain((int)lroundf(((1.0f - gf - kf) / denom) * 100.0f), 0, 100);
  y = (uint8_t)constrain((int)lroundf(((1.0f - bf - kf) / denom) * 100.0f), 0, 100);
  k = (uint8_t)constrain((int)lroundf(kf * 100.0f), 0, 100);
}

// Top-right RGB / CMYK toggle button for Color table
void drawTopRightColorModeIcon() {
  uint16_t bordCol = colorTableCmykMode ? COLOR_WARN : COLOR_ACCENT;
  canvas.fillRoundRect(198, 2, 38, 24, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(198, 2, 38, 24, 6, bordCol);
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(middle_center);
  canvas.setTextColor(bordCol, COLOR_HEX_FILL);
  canvas.drawString(colorTableCmykMode ? "CMYK" : "RGB", 217, 14);
}

// Render Color table screen (SCREEN_COLOR_TABLE)
void drawColorTableScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();
  drawTopRightColorModeIcon();

  // Detail modal when user taps a color row
  if (colorTableSelected >= 0 && colorTableSelected < COLOR_TABLE_COUNT) {
    const ColorTableEntry& e = COLOR_TABLE_DATA[colorTableSelected];
    uint16_t col565 = colorRgb888To565(e.r, e.g, e.b);
    uint8_t c = 0, m = 0, y = 0, k = 0;
    colorRgbToCmyk(e.r, e.g, e.b, c, m, y, k);

    canvas.fillRoundRect(10, 34, 220, 198, 8, COLOR_TOPBAR);
    canvas.drawRoundRect(10, 34, 220, 198, 8, COLOR_ACCENT);

    // Big color preview swatch
    canvas.fillRoundRect(20, 44, 200, 48, 6, col565);
    canvas.drawRoundRect(20, 44, 200, 48, 6, COLOR_TEXT);

    char hexBuf[12];
    char rgbBuf[28];
    char cmykBuf[32];
    char c565Buf[24];
    snprintf(hexBuf, sizeof(hexBuf), "#%02X%02X%02X", e.r, e.g, e.b);
    snprintf(rgbBuf, sizeof(rgbBuf), "RGB:  %u, %u, %u", e.r, e.g, e.b);
    snprintf(cmykBuf, sizeof(cmykBuf), "CMYK: %u%% %u%% %u%% %u%%", c, m, y, k);
    snprintf(c565Buf, sizeof(c565Buf), "RGB565: 0x%04X", col565);

    canvas.setFont(&fonts::Font2);
    canvas.setTextDatum(top_left);
    canvas.setTextColor(COLOR_TEXT, COLOR_TOPBAR);
    canvas.drawString(e.name, 20, 100);
    canvas.setTextDatum(top_right);
    canvas.setTextColor(COLOR_ACCENT, COLOR_TOPBAR);
    canvas.drawString(hexBuf, 220, 100);

    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(top_left);
    canvas.setTextColor(COLOR_TEXT, COLOR_TOPBAR);
    canvas.drawString(rgbBuf, 20, 126);
    canvas.setTextColor(COLOR_WARN, COLOR_TOPBAR);
    canvas.drawString(cmykBuf, 20, 144);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
    canvas.drawString(c565Buf, 20, 162);

    canvas.fillRoundRect(70, 192, 100, 30, 6, COLOR_HEX_ACTIVE);
    canvas.drawRoundRect(70, 192, 100, 30, 6, COLOR_GREEN);
    canvas.setFont(&fonts::Font2);
    canvas.setTextDatum(middle_center);
    canvas.setTextColor(COLOR_GREEN, COLOR_HEX_ACTIVE);
    canvas.drawString("OK", 120, 207);
    return;
  }

  // Subtitle bar (y = 30..45)
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(colorTableCmykMode ? COLOR_WARN : COLOR_ACCENT, COLOR_BG);
  canvas.drawString(colorTableCmykMode ? "Color table [CMYK]" : "Color table [RGB]", 8, 38);

  char rangeBuf[16];
  int16_t firstNum = (int16_t)(colorTableScroll + 1);
  int16_t lastNum = (int16_t)(colorTableScroll + COLOR_TABLE_VISIBLE);
  if (lastNum > COLOR_TABLE_COUNT) lastNum = COLOR_TABLE_COUNT;
  snprintf(rangeBuf, sizeof(rangeBuf), "%d-%d/%d", (int)firstNum, (int)lastNum, COLOR_TABLE_COUNT);
  canvas.setTextDatum(middle_right);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  canvas.drawString(rangeBuf, 232, 38);

  // 6 visible color rows (y = 48..234, height 30, stride 31, width 197)
  for (int16_t r = 0; r < COLOR_TABLE_VISIBLE; r++) {
    int16_t idx = (int16_t)(colorTableScroll + r);
    if (idx >= COLOR_TABLE_COUNT) break;
    const ColorTableEntry& e = COLOR_TABLE_DATA[idx];
    int16_t ry = (int16_t)(48 + r * 31);
    uint16_t col565 = colorRgb888To565(e.r, e.g, e.b);

    canvas.fillRoundRect(6, ry, 197, 29, 4, COLOR_HEX_FILL);
    canvas.drawRoundRect(6, ry, 197, 29, 4, COLOR_HEX_BORDER);

    // Color swatch
    canvas.fillRoundRect(10, ry + 4, 22, 21, 3, col565);
    canvas.drawRoundRect(10, ry + 4, 22, 21, 3, COLOR_SUBTEXT);

    char hexBuf[10];
    snprintf(hexBuf, sizeof(hexBuf), "#%02X%02X%02X", e.r, e.g, e.b);

    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(top_left);
    canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
    canvas.drawString(e.name, 37, ry + 4);

    canvas.setTextDatum(top_right);
    canvas.setTextColor(COLOR_ACCENT, COLOR_HEX_FILL);
    canvas.drawString(hexBuf, 198, ry + 4);

    char valBuf[32];
    if (colorTableCmykMode) {
      uint8_t c = 0, m = 0, y = 0, k = 0;
      colorRgbToCmyk(e.r, e.g, e.b, c, m, y, k);
      snprintf(valBuf, sizeof(valBuf), "C:%u M:%u Y:%u K:%u%%", c, m, y, k);
      canvas.setTextColor(COLOR_WARN, COLOR_HEX_FILL);
    } else {
      snprintf(valBuf, sizeof(valBuf), "R:%u G:%u B:%u", e.r, e.g, e.b);
      canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
    }
    canvas.setTextDatum(top_left);
    canvas.drawString(valBuf, 37, ry + 16);
  }

  // Right scrollbar with Up / Down triangles (x = 207..235)
  bool canUp = (colorTableScroll > 0);
  bool canDn = (colorTableScroll + COLOR_TABLE_VISIBLE < COLOR_TABLE_COUNT);
  canvas.fillRoundRect(207, 48, 28, 46, 5, COLOR_HEX_FILL);
  canvas.drawRoundRect(207, 48, 28, 46, 5, canUp ? COLOR_ACCENT : COLOR_HEX_BORDER);
  canvas.fillTriangle(221, 61, 213, 79, 229, 79, canUp ? COLOR_ACCENT : COLOR_SUBTEXT);

  canvas.fillRoundRect(217, 100, 8, 82, 3, COLOR_TOPBAR);
  int16_t maxScroll = COLOR_TABLE_COUNT - COLOR_TABLE_VISIBLE;
  int16_t thumbH = 22;
  int16_t thumbY = (maxScroll > 0) ? (int16_t)(100 + ((int32_t)colorTableScroll * (82 - thumbH)) / maxScroll) : 100;
  canvas.fillRoundRect(217, thumbY, 8, thumbH, 3, COLOR_ACCENT);

  canvas.fillRoundRect(207, 188, 28, 46, 5, COLOR_HEX_FILL);
  canvas.drawRoundRect(207, 188, 28, 46, 5, canDn ? COLOR_ACCENT : COLOR_HEX_BORDER);
  canvas.fillTriangle(221, 221, 213, 203, 229, 203, canDn ? COLOR_ACCENT : COLOR_SUBTEXT);
}

// ============================================================================
// 7.11E. RESISTOR COLOR CODE TABLE (SCREEN_RESISTOR_TABLE: 4-BAND / 5-BAND)
// ============================================================================

#define RES_COLOR_COUNT   12
#define RES_TABLE_VISIBLE 6

static const ResistorColorEntry RES_COLOR_DATA[RES_COLOR_COUNT] = {
  {"Black",   20,  20,  20,  0, "x1",    1.0,         "-"},
  {"Brown",  139,  69,  19,  1, "x10",   10.0,        "1%"},
  {"Red",    220,  38,  38,  2, "x100",  100.0,       "2%"},
  {"Orange", 249, 115,  22,  3, "x1k",   1000.0,      "-"},
  {"Yellow", 250, 204,  21,  4, "x10k",  10000.0,     "-"},
  {"Green",   34, 197,  94,  5, "x100k", 100000.0,    "0.5%"},
  {"Blue",    59, 130, 246,  6, "x1M",   1000000.0,   "0.25%"},
  {"Violet", 168,  85, 247,  7, "x10M",  10000000.0,  "0.1%"},
  {"Gray",   148, 163, 184,  8, "x100M", 100000000.0, "0.05%"},
  {"White",  248, 250, 252,  9, "x1G",   1000000000.0,"-"},
  {"Gold",   212, 175,  55, -1, "x0.1",  0.1,         "5%"},
  {"Silver", 192, 192, 192, -1, "x0.01", 0.01,        "10%"}
};

bool resistorFiveBand        = false;
uint8_t resistorBands[5]     = {2, 7, 0, 3, 10}; // 4B: Red-Violet-Orange-Gold (27 kOhm +/-5%); 5B: Red-Violet-Black-Orange-Gold (270 kOhm +/-5%)
uint8_t resistorSelectedBand = 0;
int16_t resistorTableScroll  = 0;

// Check if color index (0..11) is valid for a given band slot in 4-band or 5-band mode
bool isResistorColorValidForSlot(uint8_t colorIdx, uint8_t slot, bool fiveBand) {
  if (colorIdx >= RES_COLOR_COUNT) return false;
  uint8_t multSlot = fiveBand ? 3 : 2;
  uint8_t tolSlot  = fiveBand ? 4 : 3;
  if (slot < multSlot) {
    // Significant digit band (0..9 only, non-zero preferred for B1 though 0 is allowed on B2/B3)
    if (RES_COLOR_DATA[colorIdx].digit < 0) return false;
    if (slot == 0 && colorIdx == 0) return false; // 1st band is non-zero (1..9)
    return true;
  }
  if (slot == multSlot) return true; // All 12 colors have IEC 60062 multipliers
  if (slot == tolSlot) {
    return (RES_COLOR_DATA[colorIdx].tolLabel[0] != '-');
  }
  return false;
}

// Format decoded resistor value (Ohms / kOhm / MOhm / GOhm) + tolerance into pure ASCII string
void formatResistorValue(char* outBuf, size_t outSize) {
  double sig = 0.0;
  uint8_t multIdx = resistorBands[3];
  uint8_t tolIdx  = resistorBands[4];
  if (multIdx >= RES_COLOR_COUNT) multIdx = 0;
  if (tolIdx >= RES_COLOR_COUNT) tolIdx = 10;

  int d1 = RES_COLOR_DATA[resistorBands[0]].digit;
  int d2 = RES_COLOR_DATA[resistorBands[1]].digit;
  int d3 = RES_COLOR_DATA[resistorBands[2]].digit;
  if (d1 < 0) d1 = 1;
  if (d2 < 0) d2 = 0;
  if (d3 < 0) d3 = 0;

  if (resistorFiveBand) {
    sig = (double)(d1 * 100 + d2 * 10 + d3);
  } else {
    sig = (double)(d1 * 10 + d2);
  }

  double ohms = sig * RES_COLOR_DATA[multIdx].multValue;
  const char* unit = "Ohm";
  double scaled = ohms;
  if (ohms >= 1e9) {
    scaled = ohms / 1e9;
    unit = "GOhm";
  } else if (ohms >= 1e6) {
    scaled = ohms / 1e6;
    unit = "MOhm";
  } else if (ohms >= 1e3) {
    scaled = ohms / 1e3;
    unit = "kOhm";
  }

  char numBuf[16];
  if (fabs(scaled - round(scaled)) < 0.005) {
    snprintf(numBuf, sizeof(numBuf), "%.0f", round(scaled));
  } else if (fabs(scaled * 10.0 - round(scaled * 10.0)) < 0.05) {
    snprintf(numBuf, sizeof(numBuf), "%.1f", scaled);
  } else {
    snprintf(numBuf, sizeof(numBuf), "%.2f", scaled);
  }
  const char* tolStr = RES_COLOR_DATA[tolIdx].tolLabel;
  if (tolStr[0] == '-') tolStr = "5%";
  snprintf(outBuf, outSize, "%s %s +/-%s", numBuf, unit, tolStr);
}

// Top-right 4B / 5B toggle button for Resistor table
void drawTopRightResistorModeIcon() {
  uint16_t bordCol = resistorFiveBand ? COLOR_GREEN : COLOR_ACCENT;
  canvas.fillRoundRect(202, 2, 34, 24, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(202, 2, 34, 24, 6, bordCol);
  canvas.setFont(&fonts::Font2);
  canvas.setTextDatum(middle_center);
  canvas.setTextColor(bordCol, COLOR_HEX_FILL);
  canvas.drawString(resistorFiveBand ? "5B" : "4B", 219, 14);
}

// Render Resistor color code table & interactive 4-band/5-band decoder (SCREEN_RESISTOR_TABLE)
void drawResistorTableScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();
  drawTopRightResistorModeIcon();

  // Top interactive resistor diagram + live resistance readout (y = 31..89)
  canvas.fillRoundRect(6, 31, 228, 58, 5, COLOR_TOPBAR);
  canvas.drawRoundRect(6, 31, 228, 58, 5, resistorFiveBand ? COLOR_GREEN : COLOR_ACCENT);

  char valBuf[32];
  formatResistorValue(valBuf, sizeof(valBuf));

  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(top_left);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
  canvas.drawString(resistorFiveBand ? "5-Band IEC" : "4-Band IEC", 11, 34);
  canvas.setTextDatum(top_right);
  canvas.setTextColor(COLOR_GREEN, COLOR_TOPBAR);
  canvas.drawString(valBuf, 229, 34);

  // Axial metallic leads and resistor ceramic body (y = 48..68)
  canvas.fillRect(16, 56, 30, 4, COLOR_SUBTEXT);
  canvas.fillRect(194, 56, 30, 4, COLOR_SUBTEXT);
  uint16_t bodyCol = resistorFiveBand ? 0x3C79 : 0xD5B1; // Blue metal-film (5B) vs Tan carbon-film (4B)
  canvas.fillRoundRect(46, 48, 148, 20, 6, bodyCol);
  canvas.drawRoundRect(46, 48, 148, 20, 6, COLOR_TEXT);

  uint8_t bandCount = resistorFiveBand ? 5 : 4;
  if (resistorSelectedBand >= bandCount) resistorSelectedBand = 0;

  static const int16_t BAND4_X[4] = {64, 90, 116, 162};
  static const int16_t BAND5_X[5] = {58, 80, 102, 124, 166};
  static const char* BAND4_LBL[4] = {"B1", "B2", "Mul", "Tol"};
  static const char* BAND5_LBL[5] = {"B1", "B2", "B3", "Mul", "Tol"};

  for (uint8_t b = 0; b < bandCount; b++) {
    int16_t bx = resistorFiveBand ? BAND5_X[b] : BAND4_X[b];
    uint8_t cIdx = resistorFiveBand ? resistorBands[b] : ((b < 2) ? resistorBands[b] : resistorBands[b + 1]);
    if (cIdx >= RES_COLOR_COUNT) cIdx = 0;
    uint16_t bCol = colorRgb888To565(RES_COLOR_DATA[cIdx].r, RES_COLOR_DATA[cIdx].g, RES_COLOR_DATA[cIdx].b);
    canvas.fillRect(bx, 49, 12, 18, bCol);
    if (b == resistorSelectedBand) {
      canvas.drawRect(bx - 1, 47, 14, 22, COLOR_WARN);
    }

    // Band selector chip below resistor body (y = 71..86)
    int16_t chipW = resistorFiveBand ? 40 : 50;
    int16_t chipX = resistorFiveBand ? (int16_t)(12 + b * 44) : (int16_t)(12 + b * 55);
    bool sel = (b == resistorSelectedBand);
    uint16_t cFill = sel ? COLOR_HEX_ACTIVE : COLOR_BG;
    uint16_t cBord = sel ? COLOR_WARN : COLOR_HEX_BORDER;
    uint16_t cTxt  = sel ? COLOR_WARN : COLOR_SUBTEXT;
    canvas.fillRoundRect(chipX, 71, chipW, 15, 3, cFill);
    canvas.drawRoundRect(chipX, 71, chipW, 15, 3, cBord);
    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(middle_center);
    canvas.setTextColor(cTxt, cFill);
    canvas.drawString(resistorFiveBand ? BAND5_LBL[b] : BAND4_LBL[b], chipX + chipW / 2, 78);
  }

  // Table column header (y = 92..104)
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
  canvas.drawString("Color", 10, 98);
  canvas.drawString(resistorFiveBand ? "D1-3" : "D1-2", 86, 98);
  canvas.drawString("Mult", 124, 98);
  canvas.drawString("Tol", 172, 98);

  // Currently selected band's assigned color index
  uint8_t activeColorIdx = resistorFiveBand
    ? resistorBands[resistorSelectedBand]
    : ((resistorSelectedBand < 2) ? resistorBands[resistorSelectedBand] : resistorBands[resistorSelectedBand + 1]);

  // 6 visible IEC 60062 color rows (y = 107..233, height 20, stride 21)
  for (int16_t r = 0; r < RES_TABLE_VISIBLE; r++) {
    int16_t idx = (int16_t)(resistorTableScroll + r);
    if (idx >= RES_COLOR_COUNT) break;
    const ResistorColorEntry& e = RES_COLOR_DATA[idx];
    int16_t ry = (int16_t)(107 + r * 21);
    bool isAssigned = (idx == (int16_t)activeColorIdx);
    bool validForSlot = isResistorColorValidForSlot((uint8_t)idx, resistorSelectedBand, resistorFiveBand);
    uint16_t rFill = isAssigned ? COLOR_HEX_ACTIVE : COLOR_HEX_FILL;
    uint16_t rBord = isAssigned ? COLOR_WARN : COLOR_HEX_BORDER;

    canvas.fillRoundRect(6, ry, 197, 19, 3, rFill);
    canvas.drawRoundRect(6, ry, 197, 19, 3, rBord);

    uint16_t swCol = colorRgb888To565(e.r, e.g, e.b);
    canvas.fillRoundRect(9, ry + 3, 14, 13, 2, swCol);
    canvas.drawRoundRect(9, ry + 3, 14, 13, 2, COLOR_SUBTEXT);

    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(middle_left);
    canvas.setTextColor(validForSlot ? COLOR_TEXT : COLOR_SUBTEXT, rFill);
    canvas.drawString(e.name, 27, ry + 10);

    char digBuf[4];
    if (e.digit >= 0) {
      snprintf(digBuf, sizeof(digBuf), "%d", (int)e.digit);
    } else {
      snprintf(digBuf, sizeof(digBuf), "-");
    }
    canvas.setTextDatum(middle_center);
    canvas.setTextColor(COLOR_ACCENT, rFill);
    canvas.drawString(digBuf, 98, ry + 10);
    canvas.setTextColor(COLOR_TEXT, rFill);
    canvas.drawString(e.multLabel, 142, ry + 10);
    canvas.setTextColor(e.tolLabel[0] != '-' ? COLOR_GREEN : COLOR_SUBTEXT, rFill);
    canvas.drawString(e.tolLabel, 184, ry + 10);
  }

  // Right scrollbar with Up / Down triangles (x = 207..235, y = 107..232)
  bool canUp = (resistorTableScroll > 0);
  bool canDn = (resistorTableScroll + RES_TABLE_VISIBLE < RES_COLOR_COUNT);
  canvas.fillRoundRect(207, 107, 28, 36, 5, COLOR_HEX_FILL);
  canvas.drawRoundRect(207, 107, 28, 36, 5, canUp ? COLOR_ACCENT : COLOR_HEX_BORDER);
  canvas.fillTriangle(221, 117, 213, 133, 229, 133, canUp ? COLOR_ACCENT : COLOR_SUBTEXT);

  canvas.fillRoundRect(217, 148, 8, 42, 3, COLOR_TOPBAR);
  int16_t maxScroll = RES_COLOR_COUNT - RES_TABLE_VISIBLE;
  int16_t thumbH = 18;
  int16_t thumbY = (maxScroll > 0) ? (int16_t)(148 + ((int32_t)resistorTableScroll * (42 - thumbH)) / maxScroll) : 148;
  canvas.fillRoundRect(217, thumbY, 8, thumbH, 3, COLOR_ACCENT);

  canvas.fillRoundRect(207, 196, 28, 36, 5, COLOR_HEX_FILL);
  canvas.drawRoundRect(207, 196, 28, 36, 5, canDn ? COLOR_ACCENT : COLOR_HEX_BORDER);
  canvas.fillTriangle(221, 222, 213, 206, 229, 206, canDn ? COLOR_ACCENT : COLOR_SUBTEXT);
}

// ============================================================================
// 7.11F. VOLTAGE DIVIDER CALCULATOR (SCREEN_VOLTAGE_DIVIDER: SpinBox Vin/R1/R2)
// ============================================================================

#define VDIV_RES_VAL_COUNT 48
#define VDIV_VIN_MIN_MV    100L
#define VDIV_VIN_MAX_MV    100000L

static const float VDIV_RES_VALS[VDIV_RES_VAL_COUNT] = {
  1.0f, 1.2f, 1.5f, 1.8f, 2.0f, 2.2f, 2.7f, 3.0f, 3.3f, 3.9f, 4.7f, 5.0f,
  5.1f, 5.6f, 6.8f, 8.2f, 10.0f, 12.0f, 15.0f, 18.0f, 20.0f, 22.0f, 27.0f, 30.0f,
  33.0f, 39.0f, 47.0f, 50.0f, 51.0f, 56.0f, 68.0f, 82.0f, 100.0f, 120.0f, 150.0f, 180.0f,
  200.0f, 220.0f, 270.0f, 300.0f, 330.0f, 390.0f, 470.0f, 500.0f, 510.0f, 560.0f, 680.0f, 820.0f
};
static const char* VDIV_UNIT_NAMES[3] = {"Ohm", "kOhm", "MOhm"};
static const double VDIV_UNIT_MULTS[3] = {1.0, 1000.0, 1000000.0};
static const int32_t VDIV_VIN_STEPS_MV[4] = {100L, 500L, 1000L, 5000L};
static const char* VDIV_VIN_STEP_LABELS[4] = {"0.1V", "0.5V", "1V", "5V"};

int32_t vdivVinMv      = 5000L; // Default 5.0 V (5V -> 3.33V classic ESP32 divider)
uint8_t vdivVinStepIdx = 1;     // +/-0.5 V
uint8_t vdivR1ValIdx   = 16;    // 10
uint8_t vdivR1Unit     = 1;     // kOhm -> 10 kOhm
uint8_t vdivR2ValIdx   = 20;    // 20
uint8_t vdivR2Unit     = 1;     // kOhm -> 20 kOhm

double getVdivResistanceOhms(uint8_t valIdx, uint8_t unitIdx) {
  if (valIdx >= VDIV_RES_VAL_COUNT) valIdx = 0;
  if (unitIdx > 2) unitIdx = 1;
  return (double)VDIV_RES_VALS[valIdx] * VDIV_UNIT_MULTS[unitIdx];
}

void stepVdivResistor(uint8_t &valIdx, uint8_t &unitIdx, int8_t delta) {
  if (delta > 0) {
    if (valIdx + 1 < VDIV_RES_VAL_COUNT) {
      valIdx++;
    } else if (unitIdx < 2) {
      unitIdx++;
      valIdx = 0;
    }
  } else if (delta < 0) {
    if (valIdx > 0) {
      valIdx--;
    } else if (unitIdx > 0) {
      unitIdx--;
      valIdx = VDIV_RES_VAL_COUNT - 1;
    }
  }
}

void formatVdivResistorLabel(uint8_t valIdx, uint8_t unitIdx, char* outBuf, size_t outSize) {
  if (valIdx >= VDIV_RES_VAL_COUNT) valIdx = 0;
  if (unitIdx > 2) unitIdx = 1;
  float v = VDIV_RES_VALS[valIdx];
  if (v < 10.0f) {
    snprintf(outBuf, outSize, "%.1f %s", v, VDIV_UNIT_NAMES[unitIdx]);
  } else {
    snprintf(outBuf, outSize, "%.0f %s", v, VDIV_UNIT_NAMES[unitIdx]);
  }
}

void formatVdivVoltage(double volts, char* outBuf, size_t outSize) {
  if (volts < 1.0) {
    double mv = volts * 1000.0;
    if (mv >= 100.0) {
      snprintf(outBuf, outSize, "%.0f mV", mv);
    } else {
      snprintf(outBuf, outSize, "%.1f mV", mv);
    }
  } else if (volts < 10.0) {
    snprintf(outBuf, outSize, "%.2f V", volts);
  } else {
    snprintf(outBuf, outSize, "%.1f V", volts);
  }
}

void formatVdivCurrent(double amps, char* outBuf, size_t outSize) {
  if (amps < 1e-3) {
    double ua = amps * 1e6;
    if (ua >= 100.0) {
      snprintf(outBuf, outSize, "%.0f uA", ua);
    } else {
      snprintf(outBuf, outSize, "%.1f uA", ua);
    }
  } else if (amps < 1.0) {
    double ma = amps * 1e3;
    if (ma >= 10.0) {
      snprintf(outBuf, outSize, "%.1f mA", ma);
    } else {
      snprintf(outBuf, outSize, "%.2f mA", ma);
    }
  } else {
    snprintf(outBuf, outSize, "%.2f A", amps);
  }
}

void formatVdivPower(double watts, char* outBuf, size_t outSize) {
  if (watts < 1e-3) {
    double uw = watts * 1e6;
    if (uw >= 100.0) {
      snprintf(outBuf, outSize, "%.0f uW", uw);
    } else {
      snprintf(outBuf, outSize, "%.1f uW", uw);
    }
  } else if (watts < 1.0) {
    double mw = watts * 1e3;
    if (mw >= 10.0) {
      snprintf(outBuf, outSize, "%.1f mW", mw);
    } else {
      snprintf(outBuf, outSize, "%.2f mW", mw);
    }
  } else {
    snprintf(outBuf, outSize, "%.2f W", watts);
  }
}

// Helper to render one SpinBox row ("-" | header + value | "+")
void drawVdivSpinBox(int16_t y, const char* topLabel, const char* valStr, uint16_t valColor, uint16_t bordColor) {
  canvas.fillRoundRect(8,   y, 40, 37, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(8,   y, 40, 37, 6, COLOR_HEX_BORDER);
  canvas.fillRoundRect(52,  y, 136, 37, 6, COLOR_TOPBAR);
  canvas.drawRoundRect(52,  y, 136, 37, 6, bordColor);
  canvas.fillRoundRect(192, y, 40, 37, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(192, y, 40, 37, 6, COLOR_HEX_BORDER);

  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font4);
  canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
  canvas.drawString("-", 28,  y + 19);
  canvas.drawString("+", 212, y + 19);

  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
  canvas.drawString(topLabel, 120, y + 9);

  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(valColor, COLOR_TOPBAR);
  canvas.drawString(valStr, 120, y + 25);
}

// Render Voltage divider calculator screen (SCREEN_VOLTAGE_DIVIDER)
void drawVoltageDividerScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  double vinV = (double)vdivVinMv / 1000.0;
  double r1Ohms = getVdivResistanceOhms(vdivR1ValIdx, vdivR1Unit);
  double r2Ohms = getVdivResistanceOhms(vdivR2ValIdx, vdivR2Unit);
  double rTot = r1Ohms + r2Ohms;
  double ratio = (rTot > 0.0) ? (r2Ohms / rTot) : 0.0;
  double voutV = vinV * ratio;
  double currentA = (rTot > 0.0) ? (vinV / rTot) : 0.0;
  double powerW = (rTot > 0.0) ? ((vinV * vinV) / rTot) : 0.0;

  // Top result + mini schematic card (y = 31..107, height 76)
  canvas.fillRoundRect(6, 31, 228, 76, 6, COLOR_TOPBAR);
  canvas.drawRoundRect(6, 31, 228, 76, 6, COLOR_GREEN);

  // Left mini voltage divider schematic (x = 12..82, y = 35..103)
  int16_t sx = 34;
  canvas.fillCircle(sx, 40, 2, COLOR_WARN);
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(middle_right);
  canvas.setTextColor(COLOR_WARN, COLOR_TOPBAR);
  canvas.drawString("Vin", sx - 5, 40);

  canvas.drawFastVLine(sx, 42, 6, COLOR_TEXT);
  canvas.fillRect(sx - 5, 48, 11, 15, COLOR_HEX_FILL);
  canvas.drawRect(sx - 5, 48, 11, 15, COLOR_ACCENT);
  canvas.setTextDatum(middle_right);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
  canvas.drawString("R1", sx - 8, 55);

  canvas.drawFastVLine(sx, 63, 11, COLOR_TEXT);
  canvas.fillCircle(sx, 68, 2, COLOR_GREEN);
  canvas.drawFastHLine(sx + 2, 68, 20, COLOR_GREEN);
  canvas.fillCircle(sx + 23, 68, 2, COLOR_GREEN);
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(COLOR_GREEN, COLOR_TOPBAR);
  canvas.drawString("Vout", sx + 27, 68);

  canvas.fillRect(sx - 5, 74, 11, 15, COLOR_HEX_FILL);
  canvas.drawRect(sx - 5, 74, 11, 15, COLOR_ACCENT);
  canvas.setTextDatum(middle_right);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
  canvas.drawString("R2", sx - 8, 81);

  canvas.drawFastVLine(sx, 89, 6, COLOR_TEXT);
  canvas.drawFastHLine(sx - 8, 95, 17, COLOR_SUBTEXT);
  canvas.drawFastHLine(sx - 5, 98, 11, COLOR_SUBTEXT);
  canvas.drawFastHLine(sx - 2, 101, 5, COLOR_SUBTEXT);

  // Right Vout, Ratio %, Current I, Power P readout (x = 92..228)
  char voutBuf[20];
  char ratioBuf[16];
  char curBuf[20];
  char pwrBuf[20];
  formatVdivVoltage(voutV, voutBuf, sizeof(voutBuf));
  snprintf(ratioBuf, sizeof(ratioBuf), "%.1f%%", ratio * 100.0);
  formatVdivCurrent(currentA, curBuf, sizeof(curBuf));
  formatVdivPower(powerW, pwrBuf, sizeof(pwrBuf));

  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(top_left);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
  canvas.drawString("Vout", 96, 36);
  canvas.setTextDatum(top_right);
  canvas.setTextColor(COLOR_ACCENT, COLOR_TOPBAR);
  canvas.drawString(ratioBuf, 226, 36);

  canvas.setFont(&fonts::Font4);
  canvas.setTextDatum(middle_right);
  canvas.setTextColor(COLOR_GREEN, COLOR_TOPBAR);
  canvas.drawString(voutBuf, 226, 62);

  char iLine[24];
  char pLine[24];
  snprintf(iLine, sizeof(iLine), "I:%s", curBuf);
  snprintf(pLine, sizeof(pLine), "P:%s", pwrBuf);
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(bottom_left);
  canvas.setTextColor(COLOR_TEXT, COLOR_TOPBAR);
  canvas.drawString(iLine, 96, 102);
  canvas.setTextDatum(bottom_right);
  canvas.setTextColor(COLOR_WARN, COLOR_TOPBAR);
  canvas.drawString(pLine, 226, 102);

  // 3 SpinBoxes: Vin (y = 112), R1 (y = 154), R2 (y = 196)
  char vinTop[24], vinVal[16];
  snprintf(vinTop, sizeof(vinTop), "Vin  +/-%s", VDIV_VIN_STEP_LABELS[vdivVinStepIdx]);
  snprintf(vinVal, sizeof(vinVal), "%.1f V", vinV);
  drawVdivSpinBox(112, vinTop, vinVal, COLOR_WARN, COLOR_WARN);

  char r1Top[24], r1Val[20];
  snprintf(r1Top, sizeof(r1Top), "R1 (top) [%s]", VDIV_UNIT_NAMES[vdivR1Unit]);
  formatVdivResistorLabel(vdivR1ValIdx, vdivR1Unit, r1Val, sizeof(r1Val));
  drawVdivSpinBox(154, r1Top, r1Val, COLOR_ACCENT, COLOR_ACCENT);

  char r2Top[24], r2Val[20];
  snprintf(r2Top, sizeof(r2Top), "R2 (bot) [%s]", VDIV_UNIT_NAMES[vdivR2Unit]);
  formatVdivResistorLabel(vdivR2ValIdx, vdivR2Unit, r2Val, sizeof(r2Val));
  drawVdivSpinBox(196, r2Top, r2Val, COLOR_ACCENT, COLOR_ACCENT);
}

// ============================================================================
// 7.12. WEATHER (WeatherAPI) AND SUN ACTIVITY (NOAA SWPC) SCREENS
// ============================================================================

// Perform HTTPS GET request without external JSON libraries (WiFiClientSecure + HTTPClient)
// If keepTail == true and response exceeds maxBytes, keep the trailing tail (for NOAA arrays)
bool httpsGetPayload(const char* url, String& outPayload, size_t maxBytes, bool keepTail, int* outCode) {
  outPayload = "";
  if (outCode) *outCode = 0;
  if (WiFi.status() != WL_CONNECTED) return false;

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(12);

  HTTPClient http;
  http.setConnectTimeout(10000);
  http.setTimeout(12000);
  http.setReuse(false);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.useHTTP10(true);
  http.setUserAgent("TWatchS3-FW/2.0");

  if (!http.begin(client, url)) {
    client.stop();
    return false;
  }
  http.addHeader("Accept", "application/json");
  http.addHeader("Accept-Encoding", "identity");
  http.addHeader("Connection", "close");

  int code = http.GET();
  if (outCode) *outCode = code;
  if (code != HTTP_CODE_OK) {
    http.end();
    client.stop();
    return false;
  }

  WiFiClient* stream = http.getStreamPtr();
  int remaining = http.getSize(); // -1 if Content-Length is absent
  size_t reserveCap = (maxBytes < 12288) ? (maxBytes + 512) : 12288;
  outPayload.reserve(reserveCap);
  char buf[513];
  uint32_t lastDataMs = millis();

  while (http.connected() || (stream && stream->available() > 0)) {
    int avail = stream ? stream->available() : 0;
    if (avail > 0) {
      int toRead = (avail > 512) ? 512 : avail;
      if (remaining > 0 && toRead > remaining) toRead = remaining;
      int n = stream->readBytes(buf, toRead);
      if (n > 0) {
        buf[n] = '\0';
        outPayload += buf;
        if (remaining > 0) {
          remaining -= n;
        }
        if (outPayload.length() > maxBytes + (keepTail ? 2048U : 0U)) {
          if (keepTail) {
            outPayload.remove(0, outPayload.length() - maxBytes);
          } else {
            break;
          }
        }
        lastDataMs = millis();
        if (remaining == 0) break;
      }
    } else {
      if (remaining == 0) break;
      if (millis() - lastDataMs > 3500UL) break;
      delay(5);
    }
  }

  if (keepTail && outPayload.length() > maxBytes) {
    outPayload.remove(0, outPayload.length() - maxBytes);
  }

  http.end();
  client.stop();
  delay(20);
  return outPayload.length() > 2;
}

// Extract JSON string field "key":"value" starting at fromPos
bool jsonExtractString(const String& src, const char* key, int fromPos, int limitPos, char* outBuf, size_t outSize, int* nextPos) {
  if (!outBuf || outSize == 0) return false;
  outBuf[0] = '\0';
  String pat = String("\"") + key + "\"";
  int idx = src.indexOf(pat, fromPos);
  if (idx < 0 || (limitPos > 0 && idx >= limitPos)) return false;

  int colon = src.indexOf(':', idx + pat.length());
  if (colon < 0 || (limitPos > 0 && colon >= limitPos)) return false;

  int p = colon + 1;
  int len = src.length();
  while (p < len && (src[p] == ' ' || src[p] == '\t' || src[p] == '\r' || src[p] == '\n')) p++;
  if (p >= len) return false;

  if (src[p] == 'n') {
    // JSON null
    if (nextPos) *nextPos = p + 4;
    return false;
  }
  if (src[p] != '"') return false;
  p++;

  size_t w = 0;
  while (p < len && src[p] != '"') {
    char c = src[p];
    if (c == '\\' && (p + 1) < len) {
      p++;
      c = src[p];
    }
    if ((uint8_t)c >= 32 && (uint8_t)c <= 126 && w + 1 < outSize) {
      outBuf[w++] = c;
    }
    p++;
  }
  outBuf[w] = '\0';
  if (nextPos) *nextPos = (p < len) ? (p + 1) : len;
  return true;
}

// Extract JSON numeric field "key":12.3 or "key":"12.3"
bool jsonExtractFloat(const String& src, const char* key, int fromPos, int limitPos, float* outVal, int* nextPos) {
  String pat = String("\"") + key + "\"";
  int idx = src.indexOf(pat, fromPos);
  if (idx < 0 || (limitPos > 0 && idx >= limitPos)) return false;

  int colon = src.indexOf(':', idx + pat.length());
  if (colon < 0 || (limitPos > 0 && colon >= limitPos)) return false;

  int p = colon + 1;
  int len = src.length();
  while (p < len && (src[p] == ' ' || src[p] == '\t' || src[p] == '\r' || src[p] == '\n' || src[p] == '"')) p++;
  if (p >= len) return false;
  if (src[p] == 'n') return false;

  char numBuf[24];
  size_t w = 0;
  while (p < len && w + 1 < sizeof(numBuf)) {
    char c = src[p];
    if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.') {
      numBuf[w++] = c;
      p++;
    } else {
      break;
    }
  }
  numBuf[w] = '\0';
  if (w == 0) return false;
  if (outVal) *outVal = (float)atof(numBuf);
  if (nextPos) *nextPos = p;
  return true;
}

// Abbreviated weekday label for YYYY-MM-DD date (reuses getCalendarWeekday)
void formatShortDateWithDow(const char* isoDate, char* outBuf, size_t outSize) {
  static const char* const DOW_SHORT[7] = {"Mo", "Tu", "We", "Th", "Fr", "Sa", "Su"};
  if (!isoDate || strlen(isoDate) < 10) {
    snprintf(outBuf, outSize, "--");
    return;
  }
  uint16_t yr = (uint16_t)atoi(isoDate);
  uint8_t mo  = (uint8_t)atoi(isoDate + 5);
  uint8_t da  = (uint8_t)atoi(isoDate + 8);
  uint8_t dow = (mo >= 1 && mo <= 12 && da >= 1 && da <= 31) ? getCalendarWeekday(da, mo, yr) : 0;
  snprintf(outBuf, outSize, "%s %02u/%02u", DOW_SHORT[dow % 7], da, mo);
}

// Mode switch button in top-right corner ("7d" / "Now" / "3d" / "<")
void drawTopRightModeBadge(const char* label, uint16_t borderCol) {
  canvas.fillRoundRect(204, 2, 32, 24, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(204, 2, 32, 24, 6, borderCol);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(borderCol, COLOR_HEX_FILL);
  canvas.drawString(label, 220, 14);
}

// Parse WeatherAPI forecast.json response (current + astro + up to 7 forecast days)
bool parseWeatherApiResponse(const String& body, const char* sourceTag) {
  int locPos = body.indexOf("\"location\"");
  int curPos = body.indexOf("\"current\"");
  if (locPos < 0 || curPos < 0) return false;

  char tmp[32];
  if (jsonExtractString(body, "name", locPos, curPos, tmp, sizeof(tmp), nullptr)) {
    strncpy(weatherData.city, tmp, sizeof(weatherData.city) - 1);
    weatherData.city[sizeof(weatherData.city) - 1] = '\0';
  }
  if (jsonExtractString(body, "country", locPos, curPos, tmp, sizeof(tmp), nullptr)) {
    strncpy(weatherData.country, tmp, sizeof(weatherData.country) - 1);
    weatherData.country[sizeof(weatherData.country) - 1] = '\0';
  }
  strncpy(weatherData.sourceLabel, sourceTag, sizeof(weatherData.sourceLabel) - 1);
  weatherData.sourceLabel[sizeof(weatherData.sourceLabel) - 1] = '\0';

  int fcstPos = body.indexOf("\"forecast\"", curPos);
  int curEnd  = (fcstPos > curPos) ? fcstPos : body.length();

  float fval = 0.0f;
  if (jsonExtractFloat(body, "temp_c", curPos, curEnd, &fval, nullptr)) weatherData.tempC = fval;
  if (jsonExtractFloat(body, "feelslike_c", curPos, curEnd, &fval, nullptr)) weatherData.feelsLikeC = fval;
  if (jsonExtractFloat(body, "humidity", curPos, curEnd, &fval, nullptr)) weatherData.humidity = (uint8_t)constrain((int)fval, 0, 100);
  if (jsonExtractFloat(body, "wind_kph", curPos, curEnd, &fval, nullptr)) weatherData.windKph = fval;
  if (jsonExtractFloat(body, "pressure_mb", curPos, curEnd, &fval, nullptr)) weatherData.pressureMb = fval;
  if (jsonExtractFloat(body, "is_day", curPos, curEnd, &fval, nullptr)) weatherData.isDay = (fval >= 0.5f);

  int condPos = body.indexOf("\"condition\"", curPos);
  if (condPos > curPos && condPos < curEnd) {
    if (jsonExtractString(body, "text", condPos, curEnd, tmp, sizeof(tmp), nullptr)) {
      strncpy(weatherData.conditionText, tmp, sizeof(weatherData.conditionText) - 1);
      weatherData.conditionText[sizeof(weatherData.conditionText) - 1] = '\0';
    }
    if (jsonExtractFloat(body, "code", condPos, curEnd, &fval, nullptr)) {
      weatherData.conditionCode = (uint16_t)fval;
    }
  }

  weatherForecastCount = 0;
  if (fcstPos > 0) {
    int scanPos = body.indexOf("\"forecastday\"", fcstPos);
    while (scanPos > 0 && weatherForecastCount < WEATHER_FORECAST_MAX) {
      int datePos = body.indexOf("\"date\"", scanPos);
      if (datePos < 0) break;
      int nextDatePos = body.indexOf("\"date\"", datePos + 10);
      int dayEnd = (nextDatePos > datePos) ? nextDatePos : body.length();

      WeatherForecastDay& d = weatherForecast[weatherForecastCount];
      memset(&d, 0, sizeof(d));
      jsonExtractString(body, "date", datePos, dayEnd, d.date, sizeof(d.date), nullptr);

      int dayBlock = body.indexOf("\"day\"", datePos);
      int astroBlock = body.indexOf("\"astro\"", datePos);
      int dayBlockEnd = (astroBlock > dayBlock && astroBlock < dayEnd) ? astroBlock : dayEnd;

      if (dayBlock > 0 && dayBlock < dayEnd) {
        if (jsonExtractFloat(body, "maxtemp_c", dayBlock, dayBlockEnd, &fval, nullptr)) d.maxTempC = fval;
        if (jsonExtractFloat(body, "mintemp_c", dayBlock, dayBlockEnd, &fval, nullptr)) d.minTempC = fval;
        if (jsonExtractFloat(body, "daily_chance_of_rain", dayBlock, dayBlockEnd, &fval, nullptr)) {
          d.chanceOfRain = (uint8_t)constrain((int)fval, 0, 100);
        }
        int dCond = body.indexOf("\"condition\"", dayBlock);
        if (dCond > dayBlock && dCond < dayBlockEnd) {
          jsonExtractString(body, "text", dCond, dayBlockEnd, d.conditionText, sizeof(d.conditionText), nullptr);
          if (jsonExtractFloat(body, "code", dCond, dayBlockEnd, &fval, nullptr)) {
            d.conditionCode = (uint16_t)fval;
          }
        }
      }

      // Take astronomy data (sunrise, sunset, moon phase) from first forecast day (today)
      if (weatherForecastCount == 0 && astroBlock > 0 && astroBlock < dayEnd) {
        jsonExtractString(body, "sunrise", astroBlock, dayEnd, weatherData.sunrise, sizeof(weatherData.sunrise), nullptr);
        jsonExtractString(body, "sunset", astroBlock, dayEnd, weatherData.sunset, sizeof(weatherData.sunset), nullptr);
        jsonExtractString(body, "moon_phase", astroBlock, dayEnd, weatherData.moonPhase, sizeof(weatherData.moonPhase), nullptr);
        if (jsonExtractFloat(body, "moon_illumination", astroBlock, dayEnd, &fval, nullptr)) {
          weatherData.moonIllum = (uint8_t)constrain((int)fval, 0, 100);
        }
      }

      weatherForecastCount++;
      scanPos = (nextDatePos > 0) ? nextDatePos : -1;
    }
  }

  weatherData.valid = true;
  weatherData.errorMsg[0] = '\0';
  return true;
}

// Request weather update (deferred execution in loop after rendering Loading screen)
void requestWeatherFetch() {
  stopTouchVibrationNow();
  if (WiFi.status() != WL_CONNECTED) {
    weatherData.valid = false;
    snprintf(weatherData.errorMsg, sizeof(weatherData.errorMsg), "Wi-Fi not connected");
    weatherView = WEATHER_VIEW_ERROR;
    weatherFetchPending = false;
    return;
  }
  weatherView = WEATHER_VIEW_LOADING;
  weatherFetchPending = true;
}

// Execute WeatherAPI HTTPS request with priority: Manual -> Auto Wi-Fi (auto:ip) -> GPS fallback
void fetchWeatherDataNow() {
  stopTouchVibrationNow();
  weatherFetchPending = false;
  if (currentScreen != SCREEN_WEATHER) return;

  if (!settingWifiEnabled || WiFi.status() != WL_CONNECTED) {
    weatherData.valid = false;
    snprintf(weatherData.errorMsg, sizeof(weatherData.errorMsg),
             settingWifiEnabled ? "Wi-Fi not connected" : "Wi-Fi disabled");
    weatherView = WEATHER_VIEW_ERROR;
    renderCurrentScreen();
    return;
  }

  String payload;
  int httpCode = 0;
  char url[220];
  bool ok = false;

  if (weatherAutoLocate) {
    // Auto-locate via Wi-Fi (auto:ip) and fetch 7-day forecast (without hourly blocks hour=25)
    snprintf(url, sizeof(url),
             "https://api.weatherapi.com/v1/forecast.json?key=%s&q=auto:ip&days=7&hour=25&aqi=no&alerts=no",
             WEATHER_API_KEY);
    if (httpsGetPayload(url, payload, 12288, false, &httpCode)) {
      ok = parseWeatherApiResponse(payload, "Wi-Fi");
    }

    // If Wi-Fi auto-location fails, fall back to GPS coordinates
    if (!ok && gps.location.isValid()) {
      snprintf(url, sizeof(url),
               "https://api.weatherapi.com/v1/forecast.json?key=%s&q=%.4f,%.4f&days=7&hour=25&aqi=no&alerts=no",
               WEATHER_API_KEY, gps.location.lat(), gps.location.lng());
      if (httpsGetPayload(url, payload, 12288, false, &httpCode)) {
        ok = parseWeatherApiResponse(payload, "GPS");
      }
    }

    if (!ok) {
      weatherData.valid = false;
      snprintf(weatherData.errorMsg, sizeof(weatherData.errorMsg),
               gps.location.isValid() ? "WeatherAPI HTTP %d" : "No Wi-Fi/GPS geo", httpCode);
      weatherView = WEATHER_VIEW_ERROR;
      renderCurrentScreen();
      return;
    }
  } else {
    // Manual mode: selected capital from WORLD_CAPITALS
    uint8_t idx = (weatherCityIdx < WORLD_CAPITALS_COUNT) ? weatherCityIdx : 0;
    snprintf(url, sizeof(url),
             "https://api.weatherapi.com/v1/forecast.json?key=%s&q=%s&days=7&hour=25&aqi=no&alerts=no",
             WEATHER_API_KEY, WORLD_CAPITALS[idx].query);
    if (httpsGetPayload(url, payload, 12288, false, &httpCode)) {
      ok = parseWeatherApiResponse(payload, "Manual");
    }
    if (!ok) {
      weatherData.valid = false;
      snprintf(weatherData.errorMsg, sizeof(weatherData.errorMsg), "WeatherAPI HTTP %d", httpCode);
      weatherView = WEATHER_VIEW_ERROR;
      renderCurrentScreen();
      return;
    }
  }

  weatherView = WEATHER_VIEW_CURRENT;
  renderCurrentScreen();
}

// Miniature weather condition icon by WeatherAPI code (1000=Clear, 1003..1009=Cloudy, 1063..1201=Rain, 1210..1258=Snow, 1087/1273+=Storm)
void drawWeatherConditionIcon(int16_t cx, int16_t cy, uint16_t code, bool isDay, bool smallIcon) {
  int16_t r = smallIcon ? 5 : 10;
  if (code == 1000) {
    // Clear: sun or crescent moon
    if (isDay) {
      canvas.fillCircle(cx, cy, r - 2, 0xFFE0);
      for (int8_t k = 0; k < 8; k++) {
        float ang = k * 0.7854f;
        int16_t x1 = cx + (int16_t)(cosf(ang) * (r + 1));
        int16_t y1 = cy + (int16_t)(sinf(ang) * (r + 1));
        int16_t x2 = cx + (int16_t)(cosf(ang) * (r + (smallIcon ? 3 : 5)));
        int16_t y2 = cy + (int16_t)(sinf(ang) * (r + (smallIcon ? 3 : 5)));
        canvas.drawLine(x1, y1, x2, y2, COLOR_WARN);
      }
    } else {
      canvas.fillCircle(cx, cy, r - 1, 0xFFE0);
      canvas.fillCircle(cx + (smallIcon ? 3 : 5), cy - 2, r - 2, COLOR_HEX_FILL);
    }
    return;
  }

  // Cloud (base for overcast/rain/snow/storm)
  if (code == 1003) {
    canvas.fillCircle(cx - r / 2, cy - r / 2, r / 2 + 1, 0xFFE0);
  }
  uint16_t cloudCol = (code >= 1063) ? 0x9CF3 : 0xDEFB;
  canvas.fillCircle(cx - r / 2, cy, r / 2 + 1, cloudCol);
  canvas.fillCircle(cx + r / 2, cy, r / 2, cloudCol);
  canvas.fillCircle(cx, cy - r / 3, r / 2 + 2, cloudCol);
  canvas.fillRect(cx - r / 2, cy, r, r / 2 + 1, cloudCol);

  if (code == 1087 || code >= 1273) {
    // Lightning bolt
    canvas.drawLine(cx, cy + r / 2, cx - 2, cy + r + 1, 0xFFE0);
    canvas.drawLine(cx - 2, cy + r + 1, cx + 2, cy + r + 1, 0xFFE0);
    canvas.drawLine(cx + 2, cy + r + 1, cx - 1, cy + r + (smallIcon ? 4 : 6), COLOR_WARN);
  } else if ((code >= 1210 && code <= 1225) || code == 1066 || code == 1114 || code == 1255 || code == 1258) {
    // Snowflakes
    canvas.fillCircle(cx - 4, cy + r + 2, 1, COLOR_TEXT);
    canvas.fillCircle(cx, cy + r + 3, 1, COLOR_ACCENT);
    canvas.fillCircle(cx + 4, cy + r + 2, 1, COLOR_TEXT);
  } else if (code >= 1063 && code != 1135 && code != 1147) {
    // Raindrops
    int16_t dLen = smallIcon ? 3 : 5;
    canvas.drawLine(cx - 4, cy + r / 2 + 2, cx - 5, cy + r / 2 + 2 + dLen, COLOR_ACCENT);
    canvas.drawLine(cx, cy + r / 2 + 3, cx - 1, cy + r / 2 + 3 + dLen, COLOR_ACCENT);
    canvas.drawLine(cx + 4, cy + r / 2 + 2, cx + 3, cy + r / 2 + 2 + dLen, COLOR_ACCENT);
  }
}

// Graphical moon phase icon
void drawMoonPhaseIcon(int16_t cx, int16_t cy, int16_t r, const char* phaseName, uint8_t illumPct) {
  canvas.fillCircle(cx, cy, r, 0x18E3);
  canvas.drawCircle(cx, cy, r, COLOR_SUBTEXT);
  bool waning = (phaseName && strstr(phaseName, "Waning") != nullptr) ||
                (phaseName && strstr(phaseName, "Last") != nullptr);
  if (illumPct >= 92) {
    canvas.fillCircle(cx, cy, r - 1, 0xFFE0);
  } else if (illumPct > 8) {
    // Illuminated portion of lunar disc
    int16_t xStart = waning ? (cx - r + 1) : (cx - (int16_t)((illumPct - 50) * (r - 1) / 50));
    int16_t xEnd   = waning ? (cx + (int16_t)((illumPct - 50) * (r - 1) / 50)) : (cx + r - 1);
    for (int16_t dy = -(r - 1); dy <= (r - 1); dy++) {
      int16_t halfW = (int16_t)sqrtf((float)((r - 1) * (r - 1) - dy * dy));
      int16_t lx = (xStart > cx - halfW) ? xStart : (cx - halfW);
      int16_t rx = (xEnd < cx + halfW) ? xEnd : (cx + halfW);
      if (rx >= lx) {
        canvas.drawFastHLine(lx, cy + dy, rx - lx + 1, 0xFFE0);
      }
    }
  }
}

// Bottom bar for Auto/Manual toggle and world capital selector
void drawWeatherBottomControls() {
  uint16_t modeBg = weatherAutoLocate ? COLOR_HEX_ACTIVE : COLOR_HEX_FILL;
  uint16_t modeBd = weatherAutoLocate ? COLOR_GREEN : COLOR_WARN;
  canvas.fillRoundRect(6, 191, 111, 42, 7, modeBg);
  canvas.drawRoundRect(6, 191, 111, 42, 7, modeBd);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, modeBg);
  canvas.drawString("Geolocation", 61, 203);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(modeBd, modeBg);
  canvas.drawString(weatherAutoLocate ? "AUTO" : "MANUAL", 61, 221);

  canvas.fillRoundRect(123, 191, 111, 42, 7, COLOR_HEX_FILL);
  canvas.drawRoundRect(123, 191, 111, 42, 7, COLOR_ACCENT);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
  canvas.drawString("Capital list", 178, 203);
  char capShort[14];
  uint8_t cIdx = (weatherCityIdx < WORLD_CAPITALS_COUNT) ? weatherCityIdx : 0;
  strncpy(capShort, WORLD_CAPITALS[cIdx].name, sizeof(capShort) - 1);
  capShort[sizeof(capShort) - 1] = '\0';
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
  canvas.drawString(capShort, 178, 221);
}

// Draw Weather screen
void drawWeatherScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  if (weatherView == WEATHER_VIEW_LOADING) {
    drawTopRightModeBadge("7d", COLOR_SUBTEXT);
    canvas.fillRoundRect(14, 54, 212, 118, 10, COLOR_HEX_FILL);
    canvas.drawRoundRect(14, 54, 212, 118, 10, COLOR_ACCENT);
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_ACCENT, COLOR_HEX_FILL);
    canvas.drawString("Loading weather...", 120, 96);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
    if (weatherAutoLocate) {
      canvas.drawString("Mode: Auto (Wi-Fi / GPS)", 120, 124);
    } else {
      uint8_t cIdx = (weatherCityIdx < WORLD_CAPITALS_COUNT) ? weatherCityIdx : 0;
      char sub[32];
      snprintf(sub, sizeof(sub), "Capital: %s", WORLD_CAPITALS[cIdx].name);
      canvas.drawString(sub, 120, 124);
    }
    canvas.drawString("api.weatherapi.com", 120, 146);
    drawWeatherBottomControls();
    return;
  }

  if (weatherView == WEATHER_VIEW_ERROR) {
    drawTopRightModeBadge("7d", COLOR_SUBTEXT);
    canvas.fillRoundRect(14, 46, 212, 134, 10, COLOR_HEX_FILL);
    canvas.drawRoundRect(14, 46, 212, 134, 10, COLOR_WARN);
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_WARN, COLOR_HEX_FILL);
    canvas.drawString("Weather", 120, 72);
    canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
    canvas.drawString(weatherData.errorMsg[0] ? weatherData.errorMsg : "Request failed", 120, 102);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
    canvas.drawString("Tap card to retry", 120, 132);
    canvas.fillRoundRect(74, 146, 92, 24, 6, COLOR_HEX_ACTIVE);
    canvas.drawRoundRect(74, 146, 92, 24, 6, COLOR_ACCENT);
    canvas.setTextColor(COLOR_ACCENT, COLOR_HEX_ACTIVE);
    canvas.drawString("RETRY", 120, 158);
    drawWeatherBottomControls();
    return;
  }

  if (weatherView == WEATHER_VIEW_CITIES) {
    // Scrollable list of world capitals
    drawTopRightModeBadge("<", COLOR_ACCENT);

    for (uint8_t row = 0; row < WEATHER_CITIES_VISIBLE; row++) {
      int16_t idx = (int16_t)(weatherCityScroll + row);
      if (idx >= WORLD_CAPITALS_COUNT) break;
      int16_t y = 34 + row * 41;
      bool isSelected = (!weatherAutoLocate && idx == weatherCityIdx);
      uint16_t bgCol = isSelected ? COLOR_HEX_ACTIVE : COLOR_HEX_FILL;
      uint16_t bdCol = isSelected ? COLOR_GREEN : COLOR_HEX_BORDER;
      canvas.fillRoundRect(6, y, 188, 37, 6, bgCol);
      canvas.drawRoundRect(6, y, 188, 37, 6, bdCol);

      canvas.setTextDatum(middle_left);
      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(COLOR_TEXT, bgCol);
      canvas.drawString(WORLD_CAPITALS[idx].name, 14, y + 19);

      canvas.setTextDatum(middle_right);
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(isSelected ? COLOR_GREEN : COLOR_ACCENT, bgCol);
      canvas.drawString(WORLD_CAPITALS[idx].country, 186, y + 19);
    }

    // Scroll up/down buttons on right
    bool canUp = (weatherCityScroll > 0);
    bool canDn = (weatherCityScroll + WEATHER_CITIES_VISIBLE < WORLD_CAPITALS_COUNT);
    canvas.fillRoundRect(200, 34, 34, 54, 6, COLOR_HEX_FILL);
    canvas.drawRoundRect(200, 34, 34, 54, 6, canUp ? COLOR_ACCENT : COLOR_HEX_BORDER);
    canvas.fillTriangle(217, 50, 207, 70, 227, 70, canUp ? COLOR_ACCENT : COLOR_HEX_BORDER);

    char cntBuf[12];
    snprintf(cntBuf, sizeof(cntBuf), "%d/%d", (int)(weatherCityScroll + 1), WORLD_CAPITALS_COUNT);
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
    canvas.drawString(cntBuf, 217, 134);

    canvas.fillRoundRect(200, 182, 34, 54, 6, COLOR_HEX_FILL);
    canvas.drawRoundRect(200, 182, 34, 54, 6, canDn ? COLOR_ACCENT : COLOR_HEX_BORDER);
    canvas.fillTriangle(217, 220, 207, 200, 227, 200, canDn ? COLOR_ACCENT : COLOR_HEX_BORDER);
    return;
  }

  if (weatherView == WEATHER_VIEW_FORECAST) {
    // 7-day weather forecast screen
    drawTopRightModeBadge("Now", COLOR_GREEN);

    char hdr[36];
    snprintf(hdr, sizeof(hdr), "7-Day: %.16s", weatherData.city);
    canvas.setTextDatum(middle_left);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
    canvas.drawString(hdr, 8, 37);

    char cnt[12];
    snprintf(cnt, sizeof(cnt), "%u d", weatherForecastCount);
    canvas.setTextDatum(middle_right);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
    canvas.drawString(cnt, 232, 37);

    if (weatherForecastCount == 0) {
      canvas.setTextDatum(middle_center);
      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
      canvas.drawString("No forecast data", 120, 132);
      return;
    }

    uint8_t rows = (weatherForecastCount <= WEATHER_FORECAST_MAX) ? weatherForecastCount : WEATHER_FORECAST_MAX;
    for (uint8_t i = 0; i < rows; i++) {
      int16_t y = 46 + i * 27;
      uint16_t bgCol = (i == 0) ? COLOR_HEX_ACTIVE : COLOR_HEX_FILL;
      uint16_t bdCol = (i == 0) ? COLOR_ACCENT : COLOR_HEX_BORDER;
      canvas.fillRoundRect(6, y, 228, 25, 5, bgCol);
      canvas.drawRoundRect(6, y, 228, 25, 5, bdCol);

      char dtStr[12];
      formatShortDateWithDow(weatherForecast[i].date, dtStr, sizeof(dtStr));
      canvas.setTextDatum(middle_left);
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(i == 0 ? COLOR_ACCENT : COLOR_TEXT, bgCol);
      canvas.drawString(dtStr, 12, y + 12);

      drawWeatherConditionIcon(76, y + 12, weatherForecast[i].conditionCode, true, true);

      char rainBuf[8];
      snprintf(rainBuf, sizeof(rainBuf), "%u%%", weatherForecast[i].chanceOfRain);
      canvas.setTextDatum(middle_center);
      canvas.setTextColor((weatherForecast[i].chanceOfRain >= 50) ? COLOR_ACCENT : COLOR_SUBTEXT, bgCol);
      canvas.drawString(rainBuf, 114, y + 12);

      char tempRng[20];
      snprintf(tempRng, sizeof(tempRng), "%+.0f / %+.0f C", weatherForecast[i].minTempC, weatherForecast[i].maxTempC);
      canvas.setTextDatum(middle_right);
      canvas.setTextColor(COLOR_TEXT, bgCol);
      canvas.drawString(tempRng, 228, y + 12);
    }
    return;
  }

  // WEATHER_VIEW_CURRENT
  drawTopRightModeBadge("7d", COLOR_ACCENT);

  // City title and geolocation source line (Wi-Fi / GPS / Manual)
  canvas.setTextDatum(middle_left);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_TEXT, COLOR_BG);
  canvas.drawString(weatherData.city, 8, 39);

  canvas.setTextDatum(middle_right);
  canvas.setFont(&fonts::Font0);
  uint16_t srcCol = (strcmp(weatherData.sourceLabel, "Manual") == 0) ? COLOR_WARN : COLOR_GREEN;
  canvas.setTextColor(srcCol, COLOR_BG);
  canvas.drawString(weatherData.sourceLabel, 232, 39);

  // Main current weather card
  canvas.fillRoundRect(6, 50, 228, 72, 8, COLOR_HEX_FILL);
  canvas.drawRoundRect(6, 50, 228, 72, 8, COLOR_ACCENT);

  drawWeatherConditionIcon(34, 78, weatherData.conditionCode, weatherData.isDay, false);

  char tempBuf[16];
  snprintf(tempBuf, sizeof(tempBuf), "%+.1f C", weatherData.tempC);
  canvas.setTextDatum(middle_left);
  canvas.setFont(&fonts::Font4);
  canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
  canvas.drawString(tempBuf, 64, 70);

  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_ACCENT, COLOR_HEX_FILL);
  canvas.drawString(weatherData.conditionText, 64, 90);

  char metricsBuf[42];
  snprintf(metricsBuf, sizeof(metricsBuf), "Fl:%+.0fC  H:%u%%  W:%.0fkm/h  %.0fhPa",
           weatherData.feelsLikeC, weatherData.humidity, weatherData.windKph, weatherData.pressureMb);
  canvas.setTextDatum(middle_center);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
  canvas.drawString(metricsBuf, 120, 110);

  // Astronomy card: sunrise, sunset, and moon phase
  canvas.fillRoundRect(6, 126, 228, 60, 8, COLOR_HEX_FILL);
  canvas.drawRoundRect(6, 126, 228, 60, 8, COLOR_HEX_BORDER);

  canvas.setTextDatum(middle_left);
  canvas.setFont(&fonts::Font0);
  char sunLine1[26], sunLine2[26];
  snprintf(sunLine1, sizeof(sunLine1), "Rise: %s", weatherData.sunrise);
  snprintf(sunLine2, sizeof(sunLine2), "Set:  %s", weatherData.sunset);
  canvas.setTextColor(COLOR_WARN, COLOR_HEX_FILL);
  canvas.drawString(sunLine1, 12, 143);
  canvas.setTextColor(0xFD60, COLOR_HEX_FILL);
  canvas.drawString(sunLine2, 12, 166);

  drawMoonPhaseIcon(124, 156, 12, weatherData.moonPhase, weatherData.moonIllum);
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
  char moonLbl[16];
  strncpy(moonLbl, weatherData.moonPhase, sizeof(moonLbl) - 1);
  moonLbl[sizeof(moonLbl) - 1] = '\0';
  canvas.drawString(moonLbl, 142, 147);
  char illumBuf[16];
  snprintf(illumBuf, sizeof(illumBuf), "Moon %u%%", weatherData.moonIllum);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
  canvas.drawString(illumBuf, 142, 165);

  // Auto/Manual and Capital list buttons
  drawWeatherBottomControls();
}

// ============================================================================
// SUN ACTIVITY SCREEN LOGIC AND RENDERING (NOAA SWPC)
// ============================================================================

uint16_t getKpColor(float kp) {
  if (kp >= 7.0f) return COLOR_RED;
  if (kp >= 5.0f) return COLOR_WARN;
  if (kp >= 4.0f) return 0xFFE0;
  return COLOR_GREEN;
}

uint16_t getNoaaScaleColor(uint8_t scale) {
  if (scale >= 3) return COLOR_RED;
  if (scale == 2) return COLOR_WARN;
  if (scale == 1) return 0xFFE0;
  return COLOR_GREEN;
}

const char* getKpStormText(float kp) {
  if (kp >= 8.6f) return "G5 Extreme";
  if (kp >= 7.6f) return "G4 Severe";
  if (kp >= 6.6f) return "G3 Strong";
  if (kp >= 5.6f) return "G2 Moderate";
  if (kp >= 4.6f) return "G1 Minor";
  if (kp >= 3.6f) return "Unsettled";
  return "Quiet";
}

// Color for Kyoto Dst geomagnetic index (in nT, negative values = storm)
uint16_t getDstColor(int16_t dstNt) {
  if (dstNt <= -100) return COLOR_RED;
  if (dstNt <= -50)  return COLOR_WARN;
  if (dstNt <= -30)  return 0xFFE0;
  return COLOR_GREEN;
}

// Universal NOAA SWPC time-series parser (supports both 2D arrays ["2026-...", "val"] and object arrays {"time_tag":..., "key":val})
uint8_t parseNoaaTimeSeriesRows(const String& src, float* outVals, uint8_t maxVals, bool takeTail,
                                bool onlyPredicted, const char* key1, const char* key2, const char* key3,
                                char* lastStamp, size_t stampSize) {
  float ring[64];
  uint16_t total = 0;
  int len = src.length();
  if (len < 6 || maxVals == 0) return 0;

  // 1. Try parsing 2D array format: [ "2026-...", "val", ... ]
  int pos = 0;
  while (pos < len) {
    int lb = src.indexOf('[', pos);
    if (lb < 0) break;
    int p = lb + 1;
    while (p < len && (src[p] == ' ' || src[p] == '\t' || src[p] == '\r' || src[p] == '\n')) p++;
    if (p + 3 >= len || src[p] != '"' || src[p + 1] != '2' || src[p + 2] != '0') {
      pos = lb + 1;
      continue;
    }
    int rb = src.indexOf(']', p);
    if (rb < 0) break;

    if (onlyPredicted) {
      String rowSlice = src.substring(lb, rb);
      if (rowSlice.indexOf("predicted") < 0 && rowSlice.indexOf("estimated") < 0) {
        pos = rb + 1;
        continue;
      }
    }

    int q1 = p + 1;
    int q2 = src.indexOf('"', q1);
    if (q2 > q1 && q2 < rb) {
      int comma = src.indexOf(',', q2);
      if (comma > 0 && comma < rb) {
        int vp = comma + 1;
        while (vp < rb && (src[vp] == ' ' || src[vp] == '\t' || src[vp] == '"')) vp++;
        char nbuf[16];
        size_t w = 0;
        while (vp < rb && w + 1 < sizeof(nbuf)) {
          char c = src[vp];
          if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+') {
            nbuf[w++] = c;
            vp++;
          } else {
            break;
          }
        }
        nbuf[w] = '\0';
        if (w > 0 && !(w == 1 && (nbuf[0] == '-' || nbuf[0] == '+' || nbuf[0] == '.'))) {
          float kv = (float)atof(nbuf);
          if (takeTail) {
            ring[total % 64] = kv;
            total++;
            if (lastStamp && stampSize > 10) {
              int slen = q2 - q1;
              if (slen >= (int)stampSize) slen = (int)stampSize - 1;
              for (int k = 0; k < slen; k++) lastStamp[k] = src[q1 + k];
              lastStamp[slen] = '\0';
            }
          } else if (total < maxVals) {
            outVals[total++] = kv;
          }
        }
      }
    }
    pos = rb + 1;
  }

  // 2. If no 2D rows found, parse JSON object array format: {"time_tag":"2026-...", "Kp":3.33, ...}
  if (total == 0 && key1 != nullptr) {
    pos = 0;
    while (pos < len) {
      int ob = src.indexOf('{', pos);
      if (ob < 0) break;
      int cb = src.indexOf('}', ob);
      if (cb < 0) break;

      if (onlyPredicted) {
        String objSlice = src.substring(ob, cb);
        if (objSlice.indexOf("predicted") < 0 && objSlice.indexOf("estimated") < 0) {
          pos = cb + 1;
          continue;
        }
      }

      float kv = 0.0f;
      bool gotVal = jsonExtractFloat(src, key1, ob, cb, &kv, nullptr);
      if (!gotVal && key2) gotVal = jsonExtractFloat(src, key2, ob, cb, &kv, nullptr);
      if (!gotVal && key3) gotVal = jsonExtractFloat(src, key3, ob, cb, &kv, nullptr);

      if (gotVal) {
        if (takeTail) {
          ring[total % 64] = kv;
          total++;
          if (lastStamp && stampSize > 10) {
            jsonExtractString(src, "time_tag", ob, cb, lastStamp, stampSize, nullptr);
          }
        } else if (total < maxVals) {
          outVals[total++] = kv;
        }
      }
      pos = cb + 1;
    }
  }

  if (!takeTail) return (uint8_t)total;
  uint8_t count = (total < (uint16_t)maxVals) ? (uint8_t)total : maxVals;
  if (count > 64) count = 64;
  uint16_t startIdx = (total > count) ? (uint16_t)(total - count) : 0;
  for (uint8_t i = 0; i < count; i++) {
    outVals[i] = ring[(startIdx + i) % 64];
  }
  return count;
}

// Parse latest Kp value from NOAA JSON array (noaa-planetary-k-index.json)
uint8_t parseNoaaKpArrayRows(const String& src, float* outVals, uint8_t maxVals, bool takeTail, bool onlyPredicted, char* lastStamp, size_t stampSize) {
  return parseNoaaTimeSeriesRows(src, outVals, maxVals, takeTail, onlyPredicted, "Kp", "kp", "kp_index", lastStamp, stampSize);
}

// Find exact position of top-level object key "slotKey": { in noaa-scales.json
static int findNoaaScaleSlotPos(const String& body, const char* slotKey) {
  String pat = String("\"") + slotKey + "\"";
  int len = body.length();
  int searchPos = 0;
  while (searchPos < len) {
    int idx = body.indexOf(pat, searchPos);
    if (idx < 0) return -1;
    int p = idx + pat.length();
    while (p < len && (body[p] == ' ' || body[p] == '\t' || body[p] == '\r' || body[p] == '\n')) p++;
    if (p < len && body[p] == ':') {
      p++;
      while (p < len && (body[p] == ' ' || body[p] == '\t' || body[p] == '\r' || body[p] == '\n')) p++;
      if (p < len && body[p] == '{') {
        return idx;
      }
    }
    searchPos = idx + pat.length();
  }
  return -1;
}

// Parse slot "0" / "1" / "2" / "3" in noaa-scales.json
void parseNoaaScaleSlot(const String& body, const char* slotKey, char* dateOut, size_t dateSize,
                        uint8_t* rScale, uint8_t* sScale, uint8_t* gScale,
                        uint8_t* rMinorProb, uint8_t* sProb, char* gTextOut, size_t gTextSize) {
  int sPos = findNoaaScaleSlotPos(body, slotKey);
  if (sPos < 0) return;

  // End of block is start of next slot or +720 bytes
  int endPos = sPos + 720;
  if (endPos > (int)body.length()) endPos = body.length();

  if (dateOut && dateSize > 0) {
    jsonExtractString(body, "DateStamp", sPos, endPos, dateOut, dateSize, nullptr);
  }
  float fv = 0.0f;
  int rPos = body.indexOf("\"R\"", sPos);
  int sBlock = body.indexOf("\"S\"", sPos);
  int gPos = body.indexOf("\"G\"", sPos);

  if (rPos > sPos && rPos < endPos) {
    int rEnd = (sBlock > rPos && sBlock < endPos) ? sBlock : endPos;
    if (rScale && jsonExtractFloat(body, "Scale", rPos, rEnd, &fv, nullptr)) *rScale = (uint8_t)constrain((int)fv, 0, 5);
    if (rMinorProb && jsonExtractFloat(body, "MinorProb", rPos, rEnd, &fv, nullptr)) *rMinorProb = (uint8_t)constrain((int)fv, 0, 100);
  }
  if (sBlock > sPos && sBlock < endPos) {
    int sEnd = (gPos > sBlock && gPos < endPos) ? gPos : endPos;
    if (sScale && jsonExtractFloat(body, "Scale", sBlock, sEnd, &fv, nullptr)) *sScale = (uint8_t)constrain((int)fv, 0, 5);
    if (sProb && jsonExtractFloat(body, "Prob", sBlock, sEnd, &fv, nullptr)) *sProb = (uint8_t)constrain((int)fv, 0, 100);
  }
  if (gPos > sPos && gPos < endPos) {
    if (gScale && jsonExtractFloat(body, "Scale", gPos, endPos, &fv, nullptr)) *gScale = (uint8_t)constrain((int)fv, 0, 5);
    if (gTextOut && gTextSize > 0) jsonExtractString(body, "Text", gPos, endPos, gTextOut, gTextSize, nullptr);
  }
}

void requestSunActivityFetch() {
  stopTouchVibrationNow();
  if (WiFi.status() != WL_CONNECTED) {
    sunData.valid = false;
    snprintf(sunData.errorMsg, sizeof(sunData.errorMsg), "Wi-Fi not connected");
    sunView = SUN_VIEW_ERROR;
    sunFetchPending = false;
    return;
  }
  sunView = SUN_VIEW_LOADING;
  sunFetchPending = true;
}

void fetchSunActivityNow() {
  stopTouchVibrationNow();
  sunFetchPending = false;
  if (currentScreen != SCREEN_SUN_ACTIVITY) return;

  if (!settingWifiEnabled || WiFi.status() != WL_CONNECTED) {
    sunData.valid = false;
    snprintf(sunData.errorMsg, sizeof(sunData.errorMsg),
             settingWifiEnabled ? "Wi-Fi not connected" : "Wi-Fi disabled");
    sunView = SUN_VIEW_ERROR;
    renderCurrentScreen();
    return;
  }

  String payload;
  int httpCode = 0;
  bool anyOk = false;
  float fv = 0.0f;

  // 1. Current and forecast NOAA R/S/G scales (noaa-scales.json)
  if (httpsGetPayload("https://services.swpc.noaa.gov/products/noaa-scales.json", payload, 6144, false, &httpCode)) {
    anyOk = true;
    char dummyDate[14];
    parseNoaaScaleSlot(payload, "0", dummyDate, sizeof(dummyDate),
                       &sunData.rScale, &sunData.sScale, &sunData.gScale,
                       nullptr, nullptr, sunData.gText, sizeof(sunData.gText));
    const char* fKeys[SUN_FORECAST_DAYS] = {"1", "2", "3"};
    for (uint8_t d = 0; d < SUN_FORECAST_DAYS; d++) {
      sunData.days[d].gScale = 0;
      sunData.days[d].rMinorProb = 0;
      sunData.days[d].sProb = 0;
      sunData.days[d].maxKp = 2.0f;
      parseNoaaScaleSlot(payload, fKeys[d], sunData.days[d].date, sizeof(sunData.days[d].date),
                         nullptr, nullptr, &sunData.days[d].gScale,
                         &sunData.days[d].rMinorProb, &sunData.days[d].sProb, nullptr, 0);
    }
  }

  // 2. Observed planetary Kp index (last 8 three-hour intervals)
  float obsKp[8];
  uint8_t obsCnt = 0;
  if (httpsGetPayload("https://services.swpc.noaa.gov/products/noaa-planetary-k-index.json", payload, 8192, true, &httpCode)) {
    anyOk = true;
    obsCnt = parseNoaaKpArrayRows(payload, obsKp, 8, true, false, sunData.stamp, sizeof(sunData.stamp));
    if (obsCnt > 0) {
      sunData.kpCurrent = obsKp[obsCnt - 1];
    }
  }

  // 3. 3-day Kp forecast (keepTail=true to avoid truncating forecast rows at file end)
  float fcstKp[8];
  uint8_t fcstCnt = 0;
  if (httpsGetPayload("https://services.swpc.noaa.gov/products/noaa-planetary-k-index-forecast.json", payload, 10240, true, &httpCode)) {
    anyOk = true;
    fcstCnt = parseNoaaKpArrayRows(payload, fcstKp, 8, false, true, nullptr, 0);
  }

  sunData.kpObsCount = obsCnt;
  sunData.kpBarCount = 0;
  for (uint8_t i = 0; i < obsCnt && sunData.kpBarCount < SUN_KP_BARS_MAX; i++) {
    sunData.kpBars[sunData.kpBarCount++] = obsKp[i];
  }
  for (uint8_t i = 0; i < fcstCnt && sunData.kpBarCount < SUN_KP_BARS_MAX; i++) {
    sunData.kpBars[sunData.kpBarCount++] = fcstKp[i];
  }
  // Compute estimated peak Kp for 3 forecast days
  for (uint8_t d = 0; d < SUN_FORECAST_DAYS; d++) {
    float gBase = (sunData.days[d].gScale > 0) ? (4.0f + (float)sunData.days[d].gScale) : 2.7f;
    float pMax = gBase;
    uint8_t startBar = (uint8_t)(d * 3);
    for (uint8_t k = 0; k < 3 && (startBar + k) < fcstCnt; k++) {
      if (fcstKp[startBar + k] > pMax) pMax = fcstKp[startBar + k];
    }
    sunData.days[d].maxKp = pMax;
  }

  // 4. Solar wind speed (km/s)
  if (httpsGetPayload("https://services.swpc.noaa.gov/products/summary/solar-wind-speed.json", payload, 512, false, &httpCode)) {
    anyOk = true;
    if (jsonExtractFloat(payload, "WindSpeed", 0, payload.length(), &fv, nullptr)) sunData.windSpeedKms = fv;
  }

  // 5. Interplanetary magnetic field Bt / Bz (nT)
  if (httpsGetPayload("https://services.swpc.noaa.gov/products/summary/solar-wind-mag-field.json", payload, 512, false, &httpCode)) {
    anyOk = true;
    if (jsonExtractFloat(payload, "Bt", 0, payload.length(), &fv, nullptr)) sunData.imfBt = fv;
    if (jsonExtractFloat(payload, "Bz", 0, payload.length(), &fv, nullptr)) sunData.imfBz = fv;
  }

  // 6. F10.7 solar radio flux (sfu)
  if (httpsGetPayload("https://services.swpc.noaa.gov/products/summary/10cm-flux.json", payload, 512, false, &httpCode)) {
    anyOk = true;
    if (jsonExtractFloat(payload, "Flux", 0, payload.length(), &fv, nullptr)) sunData.f107Flux = fv;
  }

  // 7. 30-day 10.7 cm solar radio flux series (10cm-flux-30-day.json)
  if (httpsGetPayload("https://services.swpc.noaa.gov/products/10cm-flux-30-day.json", payload, 8192, true, &httpCode)) {
    anyOk = true;
    uint8_t fCnt = parseNoaaTimeSeriesRows(payload, sunData.flux30d, SUN_FLUX30_MAX, true, false,
                                           "flux", "Flux", "f107", nullptr, 0);
    sunData.flux30dCount = fCnt;
    if (fCnt > 0) {
      float fMin = sunData.flux30d[0];
      float fMax = sunData.flux30d[0];
      float fSum = 0.0f;
      for (uint8_t i = 0; i < fCnt; i++) {
        float v = sunData.flux30d[i];
        if (v < fMin) fMin = v;
        if (v > fMax) fMax = v;
        fSum += v;
      }
      sunData.flux30dMin = fMin;
      sunData.flux30dMax = fMax;
      sunData.flux30dAvg = fSum / (float)fCnt;
      sunData.flux30dLatest = sunData.flux30d[fCnt - 1];
      if (sunData.f107Flux <= 0.0f) {
        sunData.f107Flux = sunData.flux30dLatest;
      }
    }
  }

  // 8. Kyoto Dst geomagnetic index (kyoto-dst.json, signed nT values)
  if (httpsGetPayload("https://services.swpc.noaa.gov/products/kyoto-dst.json", payload, 8192, true, &httpCode)) {
    anyOk = true;
    float dstTmp[SUN_DST_BARS_MAX];
    uint8_t dCnt = parseNoaaTimeSeriesRows(payload, dstTmp, SUN_DST_BARS_MAX, true, false,
                                           "dst", "Dst", "dst_index", sunData.dstStamp, sizeof(sunData.dstStamp));
    sunData.dstCount = dCnt;
    if (dCnt > 0) {
      int16_t dMin = (int16_t)lroundf(dstTmp[0]);
      for (uint8_t i = 0; i < dCnt; i++) {
        int16_t dv = (int16_t)lroundf(dstTmp[i]);
        sunData.dstBars[i] = dv;
        if (dv < dMin) dMin = dv;
      }
      sunData.dstCurrent = sunData.dstBars[dCnt - 1];
      sunData.dstMin24h  = dMin;
    }
  }

  if (!anyOk) {
    sunData.valid = false;
    snprintf(sunData.errorMsg, sizeof(sunData.errorMsg), "NOAA SWPC HTTP %d", httpCode);
    sunView = SUN_VIEW_ERROR;
    renderCurrentScreen();
    return;
  }

  sunData.valid = true;
  sunData.errorMsg[0] = '\0';
  sunView = SUN_VIEW_CURRENT;
  renderCurrentScreen();
}

// Draw Sun activity screen (current data, 3-day forecast, 10cm-flux-30-day, and kyoto-dst)
void drawSunActivityScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  if (sunView == SUN_VIEW_LOADING) {
    drawTopRightModeBadge("3d", COLOR_SUBTEXT);
    canvas.fillRoundRect(14, 62, 212, 118, 10, COLOR_HEX_FILL);
    canvas.drawRoundRect(14, 62, 212, 118, 10, COLOR_WARN);
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_WARN, COLOR_HEX_FILL);
    canvas.drawString("Loading NOAA...", 120, 100);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
    canvas.drawString("services.swpc.noaa.gov", 120, 128);
    canvas.drawString("Kp, R/S/G, Wind, IMF", 120, 146);
    canvas.drawString("10cm-flux-30d & Kyoto Dst", 120, 162);
    return;
  }

  if (sunView == SUN_VIEW_ERROR) {
    drawTopRightModeBadge("3d", COLOR_SUBTEXT);
    canvas.fillRoundRect(14, 58, 212, 132, 10, COLOR_HEX_FILL);
    canvas.drawRoundRect(14, 58, 212, 132, 10, COLOR_RED);
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_RED, COLOR_HEX_FILL);
    canvas.drawString("Sun activity", 120, 86);
    canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
    canvas.drawString(sunData.errorMsg[0] ? sunData.errorMsg : "NOAA SWPC error", 120, 118);
    canvas.fillRoundRect(74, 148, 92, 28, 6, COLOR_HEX_ACTIVE);
    canvas.drawRoundRect(74, 148, 92, 28, 6, COLOR_ACCENT);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_ACCENT, COLOR_HEX_ACTIVE);
    canvas.drawString("RETRY", 120, 162);
    return;
  }

  if (sunView == SUN_VIEW_FORECAST) {
    drawTopRightModeBadge("Dst", COLOR_WARN);

    // Top 3-hour Kp bar chart (observed + forecast)
    canvas.fillRoundRect(6, 32, 228, 96, 8, COLOR_HEX_FILL);
    canvas.drawRoundRect(6, 32, 228, 96, 8, COLOR_HEX_BORDER);
    canvas.setTextDatum(middle_left);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_ACCENT, COLOR_HEX_FILL);
    canvas.drawString("Kp Timeline (Obs | Fcst)", 12, 42);
    canvas.setTextDatum(middle_right);
    canvas.setTextColor(COLOR_WARN, COLOR_HEX_FILL);
    canvas.drawString("Storm>=5", 228, 42);

    // Geomagnetic storm threshold line Kp = 5
    int16_t plotY0 = 120;
    int16_t plotH  = 66;
    int16_t stormY = plotY0 - (int16_t)((5.0f / 9.0f) * plotH);
    for (int16_t dx = 14; dx < 226; dx += 6) {
      canvas.drawFastHLine(dx, stormY, 3, COLOR_WARN);
    }

    uint8_t cnt = sunData.kpBarCount;
    for (uint8_t i = 0; i < cnt && i < SUN_KP_BARS_MAX; i++) {
      float kv = constrain(sunData.kpBars[i], 0.0f, 9.0f);
      int16_t bh = (int16_t)((kv / 9.0f) * plotH);
      if (bh < 3) bh = 3;
      int16_t bx = 14 + i * 13;
      uint16_t bc = getKpColor(kv);
      canvas.fillRect(bx, plotY0 - bh, 10, bh, bc);
      if (i >= sunData.kpObsCount) {
        canvas.drawRect(bx, plotY0 - bh, 10, bh, COLOR_TEXT);
      }
    }
    if (sunData.kpObsCount > 0 && sunData.kpObsCount < cnt) {
      int16_t divX = 12 + sunData.kpObsCount * 13;
      canvas.drawFastVLine(divX, 52, 70, COLOR_ACCENT);
    }

    // Bottom 3-day NOAA forecast table (G/R/S)
    canvas.setTextDatum(middle_left);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
    canvas.drawString("3-Day NOAA Storm Forecast", 8, 138);

    for (uint8_t d = 0; d < SUN_FORECAST_DAYS; d++) {
      int16_t y = 147 + d * 30;
      uint16_t gCol = getNoaaScaleColor(sunData.days[d].gScale);
      canvas.fillRoundRect(6, y, 228, 27, 6, COLOR_HEX_FILL);
      canvas.drawRoundRect(6, y, 228, 27, 6, gCol);

      char dtStr[12];
      formatShortDateWithDow(sunData.days[d].date, dtStr, sizeof(dtStr));
      canvas.setTextDatum(middle_left);
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
      canvas.drawString(dtStr, 12, y + 13);

      char gBadge[10];
      snprintf(gBadge, sizeof(gBadge), "G%u", sunData.days[d].gScale);
      canvas.fillRoundRect(74, y + 5, 28, 17, 4, gCol);
      canvas.setTextDatum(middle_center);
      canvas.setTextColor(COLOR_BG, gCol);
      canvas.drawString(gBadge, 88, y + 13);

      char kpMaxBuf[14];
      snprintf(kpMaxBuf, sizeof(kpMaxBuf), "Kp %.1f", sunData.days[d].maxKp);
      canvas.setTextDatum(middle_left);
      canvas.setTextColor(getKpColor(sunData.days[d].maxKp), COLOR_HEX_FILL);
      canvas.drawString(kpMaxBuf, 108, y + 13);

      char probBuf[20];
      snprintf(probBuf, sizeof(probBuf), "R:%u%% S:%u%%", sunData.days[d].rMinorProb, sunData.days[d].sProb);
      canvas.setTextDatum(middle_right);
      canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
      canvas.drawString(probBuf, 228, y + 13);
    }
    return;
  }

  if (sunView == SUN_VIEW_FLUX_DST) {
    // Page 3: 30-day F10.7 flux chart (10cm-flux-30-day.json) and Kyoto Dst index (kyoto-dst.json)
    drawTopRightModeBadge("Now", COLOR_GREEN);

    // Top card: 10.7cm Solar Radio Flux (30 days)
    canvas.fillRoundRect(6, 32, 228, 98, 8, COLOR_HEX_FILL);
    canvas.drawRoundRect(6, 32, 228, 98, 8, COLOR_ACCENT);
    canvas.setTextDatum(middle_left);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_ACCENT, COLOR_HEX_FILL);
    canvas.drawString("10cm Flux (30d)", 12, 42);

    char fHdr[24];
    snprintf(fHdr, sizeof(fHdr), "%.0f sfu (avg %.0f)", sunData.flux30dLatest, sunData.flux30dAvg);
    canvas.setTextDatum(middle_right);
    canvas.setTextColor(COLOR_WARN, COLOR_HEX_FILL);
    canvas.drawString(fHdr, 228, 42);

    if (sunData.flux30dCount > 0) {
      float lo = sunData.flux30dMin - 10.0f;
      if (lo < 50.0f) lo = 50.0f;
      float hi = sunData.flux30dMax + 10.0f;
      if (hi <= lo + 10.0f) hi = lo + 20.0f;
      int16_t plotBottom = 114;
      int16_t plotH = 56;
      uint8_t n = sunData.flux30dCount;
      for (uint8_t i = 0; i < n && i < SUN_FLUX30_MAX; i++) {
        float norm = (sunData.flux30d[i] - lo) / (hi - lo);
        norm = constrain(norm, 0.05f, 1.0f);
        int16_t bh = (int16_t)(norm * plotH);
        if (bh < 3) bh = 3;
        int16_t bx = (int16_t)(14 + (int32_t)i * 212 / (n > 1 ? n : 1));
        int16_t bw = (int16_t)(212 / (n > 1 ? n : 1) - 1);
        if (bw < 3) bw = 3;
        uint16_t fc = (sunData.flux30d[i] >= 180.0f) ? COLOR_WARN : COLOR_ACCENT;
        canvas.fillRect(bx, plotBottom - bh, bw, bh, fc);
      }
      char fRange[28];
      snprintf(fRange, sizeof(fRange), "Min:%.0f  Max:%.0f  %ud", sunData.flux30dMin, sunData.flux30dMax, (unsigned)n);
      canvas.setTextDatum(middle_left);
      canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
      canvas.drawString(fRange, 12, 123);
    } else {
      canvas.setTextDatum(middle_center);
      canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
      canvas.drawString("No 10cm-flux-30-day data", 120, 84);
    }

    // Bottom card: Kyoto Dst Index (24h, signed nT)
    uint16_t dstCol = getDstColor(sunData.dstCurrent);
    canvas.fillRoundRect(6, 136, 228, 98, 8, COLOR_HEX_FILL);
    canvas.drawRoundRect(6, 136, 228, 98, 8, dstCol);
    canvas.setTextDatum(middle_left);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(dstCol, COLOR_HEX_FILL);
    canvas.drawString("Kyoto Dst (nT)", 12, 146);

    char dHdr[24];
    snprintf(dHdr, sizeof(dHdr), "Now:%+d  Min:%+d", (int)sunData.dstCurrent, (int)sunData.dstMin24h);
    canvas.setTextDatum(middle_right);
    canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
    canvas.drawString(dHdr, 228, 146);

    if (sunData.dstCount > 0) {
      int16_t zeroY = 170; // 0 nT baseline near top (Dst is usually negative)
      canvas.drawFastHLine(12, zeroY, 216, COLOR_SUBTEXT);
      int16_t stormY = 192; // -50 nT moderate storm threshold
      for (int16_t dx = 12; dx < 226; dx += 6) {
        canvas.drawFastHLine(dx, stormY, 3, COLOR_WARN);
      }

      int16_t maxNeg = (sunData.dstMin24h < -100) ? (int16_t)(-sunData.dstMin24h) : 100;
      uint8_t dn = sunData.dstCount;
      for (uint8_t i = 0; i < dn && i < SUN_DST_BARS_MAX; i++) {
        int16_t dv = sunData.dstBars[i];
        int16_t bx = (int16_t)(14 + (int32_t)i * 212 / (dn > 1 ? dn : 1));
        int16_t bw = (int16_t)(212 / (dn > 1 ? dn : 1) - 1);
        if (bw < 4) bw = 4;
        uint16_t dc = getDstColor(dv);
        if (dv < 0) {
          int16_t bh = (int16_t)(((int32_t)(-dv) * 44L) / maxNeg);
          if (bh < 3) bh = 3;
          if (bh > 46) bh = 46;
          canvas.fillRect(bx, zeroY + 1, bw, bh, dc);
        } else if (dv > 0) {
          int16_t bh = (int16_t)(((int32_t)dv * 14L) / 40L);
          if (bh < 2) bh = 2;
          if (bh > 14) bh = 14;
          canvas.fillRect(bx, zeroY - bh, bw, bh, COLOR_ACCENT);
        } else {
          canvas.fillRect(bx, zeroY - 1, bw, 2, COLOR_GREEN);
        }
      }
      canvas.setTextDatum(middle_left);
      canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
      char dstSt[28];
      snprintf(dstSt, sizeof(dstSt), "0nT / -50nT  %.12s", sunData.dstStamp);
      canvas.drawString(dstSt, 12, 226);
    } else {
      canvas.setTextDatum(middle_center);
      canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
      canvas.drawString("No kyoto-dst.json data", 120, 188);
    }
    return;
  }

  // SUN_VIEW_CURRENT: current geomagnetic and solar conditions
  drawTopRightModeBadge("3d", COLOR_ACCENT);

  // Kp index and geomagnetic storm status card
  uint16_t kpCol = getKpColor(sunData.kpCurrent);
  canvas.fillRoundRect(6, 32, 228, 63, 8, COLOR_HEX_FILL);
  canvas.drawRoundRect(6, 32, 228, 63, 8, kpCol);

  char kpStr[12];
  snprintf(kpStr, sizeof(kpStr), "Kp %.1f", sunData.kpCurrent);
  canvas.setTextDatum(middle_left);
  canvas.setFont(&fonts::Font4);
  canvas.setTextColor(kpCol, COLOR_HEX_FILL);
  canvas.drawString(kpStr, 14, 52);

  canvas.setTextDatum(middle_right);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(kpCol, COLOR_HEX_FILL);
  canvas.drawString(getKpStormText(sunData.kpCurrent), 226, 52);

  // 9-segment Kp bar scale (1..9)
  for (uint8_t seg = 1; seg <= 9; seg++) {
    int16_t sx = 14 + (seg - 1) * 24;
    bool filled = (sunData.kpCurrent >= (float)seg - 0.35f);
    uint16_t sc = getKpColor((float)seg);
    canvas.fillRoundRect(sx, 74, 20, 12, 3, filled ? sc : COLOR_BG);
    canvas.drawRoundRect(sx, 74, 20, 12, 3, filled ? sc : COLOR_HEX_BORDER);
  }

  // NOAA scales strip: R (Radio), S (Solar Radiation), G (Geomagnetic Storms)
  const char* scaleTitles[3] = {"Radio", "Rad", "Geomag"};
  const char scalePrefix[3]  = {'R', 'S', 'G'};
  const uint8_t scaleVals[3] = {sunData.rScale, sunData.sScale, sunData.gScale};
  for (uint8_t k = 0; k < 3; k++) {
    int16_t bx = 6 + k * 78;
    uint16_t sc = getNoaaScaleColor(scaleVals[k]);
    canvas.fillRoundRect(bx, 100, 72, 42, 7, COLOR_HEX_FILL);
    canvas.drawRoundRect(bx, 100, 72, 42, 7, sc);
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
    canvas.drawString(scaleTitles[k], bx + 36, 111);
    char sv[6];
    snprintf(sv, sizeof(sv), "%c%u", scalePrefix[k], scaleVals[k]);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(sc, COLOR_HEX_FILL);
    canvas.drawString(sv, bx + 36, 129);
  }

  // Solar wind, IMF Bt/Bz, F10.7 flux, and Kyoto Dst card
  canvas.fillRoundRect(6, 147, 228, 56, 8, COLOR_HEX_FILL);
  canvas.drawRoundRect(6, 147, 228, 56, 8, COLOR_HEX_BORDER);

  char wLine[34], bLine[34], fLine[34];
  snprintf(wLine, sizeof(wLine), "Solar Wind: %.0f km/s", sunData.windSpeedKms);
  snprintf(bLine, sizeof(bLine), "IMF Bt: %.1f nT   Bz: %+.1f nT", sunData.imfBt, sunData.imfBz);
  snprintf(fLine, sizeof(fLine), "F10.7: %.0f sfu   Dst: %+d nT", sunData.f107Flux, (int)sunData.dstCurrent);
  canvas.setTextDatum(middle_left);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(sunData.windSpeedKms >= 550.0f ? COLOR_WARN : COLOR_ACCENT, COLOR_HEX_FILL);
  canvas.drawString(wLine, 14, 160);
  canvas.setTextColor(sunData.imfBz <= -5.0f ? COLOR_RED : COLOR_TEXT, COLOR_HEX_FILL);
  canvas.drawString(bLine, 14, 176);
  canvas.setTextColor(getDstColor(sunData.dstCurrent), COLOR_HEX_FILL);
  canvas.drawString(fLine, 14, 192);

  // Bottom bar: NOAA SWPC label and refresh button
  canvas.setTextDatum(middle_left);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  char stBuf[24];
  snprintf(stBuf, sizeof(stBuf), "NOAA %.16s", sunData.stamp);
  canvas.drawString(stBuf, 8, 221);

  canvas.fillRoundRect(160, 208, 74, 26, 6, COLOR_HEX_ACTIVE);
  canvas.drawRoundRect(160, 208, 74, 26, 6, COLOR_ACCENT);
  canvas.setTextDatum(middle_center);
  canvas.setTextColor(COLOR_ACCENT, COLOR_HEX_ACTIVE);
  canvas.drawString("REFRESH", 197, 221);
}

// ============================================================================
// 7.13. SYSTEM INFO SCREEN (DEVICE, FIRMWARE, ESP32, SENSORS, AXP2101)
// ============================================================================

// Populate a single System info list row
static void setSysInfoRow(uint8_t idx, const char* label, const char* value, uint16_t color) {
  if (idx >= SYS_INFO_ROW_COUNT) return;
  strncpy(sysInfoRows[idx].label, label, sizeof(sysInfoRows[idx].label) - 1);
  sysInfoRows[idx].label[sizeof(sysInfoRows[idx].label) - 1] = '\0';
  strncpy(sysInfoRows[idx].value, value, sizeof(sysInfoRows[idx].value) - 1);
  sysInfoRows[idx].value[sizeof(sysInfoRows[idx].value) - 1] = '\0';
  sysInfoRows[idx].color = color;
}

// Gather live system telemetry for System info screen
void updateSystemInfoRows() {
  static const char* const DOW_NAMES[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
  char buf[36];

  // 0..2: Device, firmware name, and version (3 dot-separated numbers)
  setSysInfoRow(0, "Device", DEVICE_NAME, COLOR_ACCENT);
  setSysInfoRow(1, "Firmware", FW_NAME, COLOR_TEXT);
  snprintf(buf, sizeof(buf), "%d.%d.%d", FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH);
  setSysInfoRow(2, "Version", buf, COLOR_GREEN);

  // 3..5: Time, date, and time zone
  snprintf(buf, sizeof(buf), "%02u:%02u:%02u", rtcTime.hour, rtcTime.minute, rtcTime.second);
  setSysInfoRow(3, "Time", buf, COLOR_TEXT);

  const char* dow = (rtcTime.dayOfWeek < 7) ? DOW_NAMES[rtcTime.dayOfWeek] : "---";
  snprintf(buf, sizeof(buf), "%02u.%02u.%04u (%s)", rtcTime.day, rtcTime.month, rtcTime.year, dow);
  setSysInfoRow(4, "Date", buf, COLOR_TEXT);

  snprintf(buf, sizeof(buf), "UTC%+d", (int)utcOffset);
  setSysInfoRow(5, "Time zone", buf, COLOR_ACCENT);

  // 6..7: Wi-Fi status
  bool wifiOk = (WiFi.status() == WL_CONNECTED);
  if (wifiOk) {
    String ssid = WiFi.SSID();
    snprintf(buf, sizeof(buf), "ON: %.24s", ssid.c_str());
    setSysInfoRow(6, "Wi-Fi status", buf, COLOR_GREEN);

    IPAddress ip = WiFi.localIP();
    snprintf(buf, sizeof(buf), "%u.%u.%u.%u (%lddBm)",
             ip[0], ip[1], ip[2], ip[3], (long)WiFi.RSSI());
    setSysInfoRow(7, "Wi-Fi IP / RSSI", buf, COLOR_ACCENT);
  } else {
    wifi_mode_t wMode = WiFi.getMode();
    setSysInfoRow(6, "Wi-Fi status", (wMode == WIFI_MODE_NULL) ? "OFF" : "Disconnected", COLOR_WARN);
    setSysInfoRow(7, "Wi-Fi IP / RSSI", "No connection", COLOR_SUBTEXT);
  }

  // 8: ESP32 serial number (from eFuse MAC / Chip ID)
  uint64_t efuseMac = ESP.getEfuseMac();
  snprintf(buf, sizeof(buf), "%04" PRIX32 "%08" PRIX32,
           (uint32_t)((efuseMac >> 32) & 0xFFFFUL),
           (uint32_t)(efuseMac & 0xFFFFFFFFUL));
  setSysInfoRow(8, "ESP32 Serial", buf, COLOR_ACCENT);

  // 9..12: ESP32 CPU frequency and memory (Heap, PSRAM, Flash)
  snprintf(buf, sizeof(buf), "%s %uC @ %luMHz",
           ESP.getChipModel(), (unsigned)ESP.getChipCores(), (unsigned long)ESP.getCpuFreqMHz());
  setSysInfoRow(9, "CPU & Freq", buf, COLOR_TEXT);

  snprintf(buf, sizeof(buf), "%lu / %lu KB free",
           (unsigned long)(ESP.getFreeHeap() / 1024UL),
           (unsigned long)(ESP.getHeapSize() / 1024UL));
  setSysInfoRow(10, "Heap RAM", buf, COLOR_GREEN);

  uint32_t psramTotalKb = (uint32_t)(ESP.getPsramSize() / 1024UL);
  if (psramTotalKb > 0) {
    snprintf(buf, sizeof(buf), "%lu / %lu KB free",
             (unsigned long)(ESP.getFreePsram() / 1024UL),
             (unsigned long)psramTotalKb);
  } else {
    snprintf(buf, sizeof(buf), "Not detected");
  }
  setSysInfoRow(11, "PSRAM", buf, (psramTotalKb > 0) ? COLOR_GREEN : COLOR_SUBTEXT);

  snprintf(buf, sizeof(buf), "%lu MB @ %lu MHz",
           (unsigned long)(ESP.getFlashChipSize() / (1024UL * 1024UL)),
           (unsigned long)(ESP.getFlashChipSpeed() / 1000000UL));
  setSysInfoRow(12, "Flash memory", buf, COLOR_TEXT);

  // 13..14: ESP32 and BMA423 temperatures
  snprintf(buf, sizeof(buf), "%.1f C", currentTempC);
  setSysInfoRow(13, "ESP32 Temp", buf, COLOR_RED);

  if (bmaTempValid) {
    snprintf(buf, sizeof(buf), "%.1f C", currentBmaTempC);
    setSysInfoRow(14, "BMA423 Temp", buf, 0xFD60);
  } else {
    setSysInfoRow(14, "BMA423 Temp", "N/A", COLOR_SUBTEXT);
  }

  // 15..19: Battery state and AXP2101 power rails
  Axp2101PowerInfo pmu;
  readAxp2101PowerInfo(pmu);
  batteryPercent = pmu.batPercent;

  const char* batStateStr = "Standby";
  uint16_t batCol = COLOR_GREEN;
  if (!pmu.batPresent) {
    batStateStr = "No battery";
    batCol = COLOR_RED;
  } else if (pmu.chgDirection == 1) {
    if (pmu.chgState == 2)      batStateStr = "Charging (CC)";
    else if (pmu.chgState == 3) batStateStr = "Charging (CV)";
    else if (pmu.chgState == 1) batStateStr = "Pre-charge";
    else                        batStateStr = "Charging";
    batCol = COLOR_ACCENT;
  } else if (pmu.chgState == 4) {
    batStateStr = "Full";
    batCol = COLOR_GREEN;
  } else if (pmu.chgDirection == 2) {
    batStateStr = "Discharging";
    batCol = (pmu.batPercent > 20) ? COLOR_GREEN : COLOR_RED;
  }
  snprintf(buf, sizeof(buf), "%u%% %s", (unsigned)pmu.batPercent, batStateStr);
  setSysInfoRow(15, "Battery", buf, batCol);

  snprintf(buf, sizeof(buf), "%.2f V (%u mV)", (float)pmu.batVoltageMv / 1000.0f, (unsigned)pmu.batVoltageMv);
  setSysInfoRow(16, "AXP2101 VBAT", buf, (pmu.batVoltageMv >= 3500) ? COLOR_GREEN : COLOR_WARN);

  if (pmu.vbusGood) {
    snprintf(buf, sizeof(buf), "USB %.2f V (Good)", (float)pmu.vbusVoltageMv / 1000.0f);
    setSysInfoRow(17, "AXP2101 VBUS", buf, COLOR_ACCENT);
  } else {
    setSysInfoRow(17, "AXP2101 VBUS", "Disconnected", COLOR_SUBTEXT);
  }

  snprintf(buf, sizeof(buf), "%.2f V (%u mV)", (float)pmu.vsysVoltageMv / 1000.0f, (unsigned)pmu.vsysVoltageMv);
  setSysInfoRow(18, "AXP2101 VSYS", buf, COLOR_TEXT);

  uint8_t railsOn = 0;
  for (uint8_t b = 0; b < 5; b++) if (pmu.dcdcMask & (1U << b)) railsOn++;
  for (uint8_t b = 0; b < 8; b++) if (pmu.ldoMask0 & (1U << b)) railsOn++;
  if (pmu.ldoMask1 & 0x01) railsOn++;
  snprintf(buf, sizeof(buf), "%.1f C  Rails:%u ON", pmu.dieTempC, (unsigned)railsOn);
  setSysInfoRow(19, "AXP2101 PMU", buf, COLOR_WARN);
}

// Draw System info screen
void drawSystemInfoScreen() {
  updateSystemInfoRows();

  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  uint8_t maxOffset = (SYS_INFO_ROW_COUNT > SYS_INFO_VISIBLE_ROWS)
                        ? (uint8_t)(SYS_INFO_ROW_COUNT - SYS_INFO_VISIBLE_ROWS)
                        : 0;
  if (sysInfoScrollOffset > maxOffset) sysInfoScrollOffset = maxOffset;

  bool canUp   = (sysInfoScrollOffset > 0);
  bool canDown = (sysInfoScrollOffset < maxOffset);

  // Scroll triangle buttons on top-right and bottom-right
  canvas.fillRoundRect(199, 34, 36, 56, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(199, 34, 36, 56, 6, canUp ? COLOR_ACCENT : COLOR_HEX_BORDER);
  canvas.fillTriangle(217, 52, 208, 70, 226, 70, canUp ? COLOR_ACCENT : COLOR_SUBTEXT);

  canvas.fillRoundRect(199, 182, 36, 56, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(199, 182, 36, 56, 6, canDown ? COLOR_ACCENT : COLOR_HEX_BORDER);
  canvas.fillTriangle(217, 220, 208, 202, 226, 202, canDown ? COLOR_ACCENT : COLOR_SUBTEXT);

  char posBuf[12];
  snprintf(posBuf, sizeof(posBuf), "%u/%u", (unsigned)(sysInfoScrollOffset + 1), (unsigned)SYS_INFO_ROW_COUNT);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  canvas.drawString(posBuf, 217, 136);

  for (uint8_t r = 0; r < SYS_INFO_VISIBLE_ROWS; r++) {
    uint8_t idx = (uint8_t)(sysInfoScrollOffset + r);
    if (idx >= SYS_INFO_ROW_COUNT) break;

    int16_t ry = (int16_t)(34 + r * 41);
    canvas.fillRoundRect(6, ry, 188, 37, 6, COLOR_TOPBAR);
    canvas.drawRoundRect(6, ry, 188, 37, 6, COLOR_HEX_BORDER);

    canvas.setTextDatum(top_left);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
    canvas.drawString(sysInfoRows[idx].label, 12, ry + 4);

    char shortVal[30];
    size_t vlen = strlen(sysInfoRows[idx].value);
    if (vlen <= 20) {
      strncpy(shortVal, sysInfoRows[idx].value, 20);
      shortVal[20] = '\0';
      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(sysInfoRows[idx].color, COLOR_TOPBAR);
      canvas.drawString(shortVal, 12, ry + 16);
    } else {
      strncpy(shortVal, sysInfoRows[idx].value, 29);
      shortVal[29] = '\0';
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(sysInfoRows[idx].color, COLOR_TOPBAR);
      canvas.drawString(shortVal, 12, ry + 20);
    }
  }
}

// ============================================================================
// 7.14. ALARM (2 ALARMS), STOPWATCH, TIMER, AND MODAL ALERT DIALOGS
// ============================================================================

uint32_t getStopwatchElapsedMs() {
  if (swRunning) {
    return swAccumulatedMs + (millis() - swStartMs);
  }
  return swAccumulatedMs;
}

uint32_t getTimerSetTotalMs() {
  return ((uint32_t)timerSetH * 3600UL + (uint32_t)timerSetM * 60UL + (uint32_t)timerSetS) * 1000UL;
}

uint32_t getTimerDisplayRemainingMs() {
  if (timerRunning) {
    int32_t diff = (int32_t)(timerEndMs - millis());
    return (diff > 0) ? (uint32_t)diff : 0UL;
  }
  if (timerPaused) {
    return timerRemainingMs;
  }
  return getTimerSetTotalMs();
}

// Draw Alarm screen (2 alarms, Clock-style time edit, ON/OFF, ringtone, Snooze)
void drawAlarmScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  // Alarm 1 / Alarm 2 selector tabs
  for (uint8_t idx = 0; idx < ALARM_COUNT; idx++) {
    int16_t bx = (idx == 0) ? 8 : 124;
    bool sel = (selectedAlarmIdx == idx);
    uint16_t fillCol = sel ? COLOR_HEX_ACTIVE : COLOR_HEX_FILL;
    uint16_t bordCol = sel ? COLOR_ACCENT : COLOR_HEX_BORDER;
    canvas.fillRoundRect(bx, 32, 108, 24, 6, fillCol);
    canvas.drawRoundRect(bx, 32, 108, 24, 6, bordCol);

    uint16_t dotCol = alarms[idx].snoozePending ? COLOR_WARN : (alarms[idx].enabled ? COLOR_GREEN : COLOR_HEX_BORDER);
    canvas.fillCircle(bx + 14, 44, 4, dotCol);

    char tabBuf[16];
    snprintf(tabBuf, sizeof(tabBuf), "ALARM %u", (unsigned)(idx + 1));
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(sel ? COLOR_TEXT : COLOR_SUBTEXT, fillCol);
    canvas.drawString(tabBuf, bx + 60, 44);
  }

  AlarmEntry &al = alarms[selectedAlarmIdx];

  // Clock-style HH:MM block with < / > triangles and orange underline
  canvas.fillRoundRect(8, 62, 46, 46, 7, COLOR_HEX_FILL);
  canvas.drawRoundRect(8, 62, 46, 46, 7, COLOR_ACCENT);
  canvas.fillTriangle(20, 85, 42, 73, 42, 97, COLOR_ACCENT);

  canvas.fillRoundRect(186, 62, 46, 46, 7, COLOR_HEX_FILL);
  canvas.drawRoundRect(186, 62, 46, 46, 7, COLOR_ACCENT);
  canvas.fillTriangle(220, 85, 198, 73, 198, 97, COLOR_ACCENT);

  canvas.fillRoundRect(58, 62, 124, 46, 7, COLOR_TOPBAR);
  canvas.drawRoundRect(58, 62, 124, 46, 7, COLOR_HEX_BORDER);

  char hBuf[6], mBuf[6];
  snprintf(hBuf, sizeof(hBuf), "%02u", (unsigned)al.hour);
  snprintf(mBuf, sizeof(mBuf), "%02u", (unsigned)al.minute);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font4);
  canvas.setTextColor(al.enabled ? COLOR_TEXT : COLOR_SUBTEXT, COLOR_TOPBAR);
  canvas.drawString(hBuf, 90, 84);
  canvas.drawString(":", 120, 82);
  canvas.drawString(mBuf, 150, 84);

  // Active field underline (Clock style)
  int16_t ulX = (alarmEditField == 0) ? 72 : 132;
  canvas.fillRect(ulX, 102, 36, 3, COLOR_WARN);

  // Row 1: ON / OFF toggle
  canvas.fillRoundRect(8, 116, 224, 34, 6, COLOR_TOPBAR);
  canvas.drawRoundRect(8, 116, 224, 34, 6, al.enabled ? COLOR_GREEN : COLOR_HEX_BORDER);
  canvas.setTextDatum(middle_left);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
  canvas.drawString(al.snoozePending ? "Status (Snooze)" : "Status", 16, 133);

  uint16_t pillBg = al.enabled ? COLOR_GREEN : COLOR_HEX_FILL;
  uint16_t pillFg = al.enabled ? COLOR_BG : COLOR_SUBTEXT;
  canvas.fillRoundRect(166, 122, 58, 22, 5, pillBg);
  if (!al.enabled) canvas.drawRoundRect(166, 122, 58, 22, 5, COLOR_HEX_BORDER);
  canvas.setTextDatum(middle_center);
  canvas.setTextColor(pillFg, pillBg);
  canvas.drawString(al.enabled ? "ON" : "OFF", 195, 133);

  // Row 2: Ringtone selector (Beep / Bell)
  canvas.fillRoundRect(8, 156, 224, 34, 6, COLOR_TOPBAR);
  canvas.drawRoundRect(8, 156, 224, 34, 6, COLOR_HEX_BORDER);
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
  canvas.drawString("Ringtone", 16, 173);

  const char* toneLabel = (al.ringtone == RINGTONE_BELL) ? "Bell" : "Beep";
  uint16_t toneCol = (al.ringtone == RINGTONE_BELL) ? COLOR_WARN : COLOR_ACCENT;
  canvas.fillRoundRect(150, 162, 74, 22, 5, COLOR_HEX_ACTIVE);
  canvas.drawRoundRect(150, 162, 74, 22, 5, toneCol);
  canvas.setTextDatum(middle_center);
  canvas.setTextColor(toneCol, COLOR_HEX_ACTIVE);
  canvas.drawString(toneLabel, 187, 173);

  // Row 3: Snooze duration selector
  canvas.fillRoundRect(8, 196, 224, 34, 6, COLOR_TOPBAR);
  canvas.drawRoundRect(8, 196, 224, 34, 6, COLOR_HEX_BORDER);
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
  canvas.drawString("Snooze", 16, 213);

  char snzBuf[12];
  snprintf(snzBuf, sizeof(snzBuf), "%u min", (unsigned)al.snoozeMin);
  canvas.fillRoundRect(150, 202, 74, 22, 5, COLOR_HEX_ACTIVE);
  canvas.drawRoundRect(150, 202, 74, 22, 5, COLOR_ACCENT);
  canvas.setTextDatum(middle_center);
  canvas.setTextColor(COLOR_TEXT, COLOR_HEX_ACTIVE);
  canvas.drawString(snzBuf, 187, 213);
}

// Draw Stopwatch screen (Start, Stop, Reset buttons)
void drawStopwatchScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  uint32_t elapsed = getStopwatchElapsedMs();
  uint32_t totalSec = elapsed / 1000UL;
  uint8_t cs  = (uint8_t)((elapsed % 1000UL) / 10UL);
  uint8_t sec = (uint8_t)(totalSec % 60UL);
  uint8_t mn  = (uint8_t)((totalSec / 60UL) % 60UL);
  uint8_t hr  = (uint8_t)((totalSec / 3600UL) % 100UL);

  canvas.setTextDatum(top_center);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
  canvas.drawString("STOPWATCH", 120, 34);

  const char* stateLabel = swRunning ? "RUNNING" : ((elapsed > 0) ? "STOPPED" : "READY");
  uint16_t stateCol = swRunning ? COLOR_GREEN : ((elapsed > 0) ? COLOR_WARN : COLOR_SUBTEXT);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(stateCol, COLOR_BG);
  canvas.drawString(stateLabel, 120, 56);

  // Time display card
  canvas.fillRoundRect(10, 74, 220, 74, 8, COLOR_TOPBAR);
  canvas.drawRoundRect(10, 74, 220, 74, 8, swRunning ? COLOR_GREEN : COLOR_HEX_BORDER);

  char mainBuf[16];
  char csBuf[8];
  if (hr > 0) {
    snprintf(mainBuf, sizeof(mainBuf), "%02u:%02u:%02u", (unsigned)hr, (unsigned)mn, (unsigned)sec);
  } else {
    snprintf(mainBuf, sizeof(mainBuf), "%02u:%02u", (unsigned)mn, (unsigned)sec);
  }
  snprintf(csBuf, sizeof(csBuf), ".%02u", (unsigned)cs);

  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font4);
  canvas.setTextColor(COLOR_TEXT, COLOR_TOPBAR);
  canvas.drawString(mainBuf, (hr > 0) ? 94 : 96, 106);

  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_ACCENT, COLOR_TOPBAR);
  canvas.drawString(csBuf, (hr > 0) ? 186 : 168, 110);

  // Seconds progress bar (0..60 s)
  int16_t barW = (int16_t)((((uint32_t)sec * 100UL + (uint32_t)cs) * 196UL) / 6000UL);
  canvas.fillRoundRect(22, 134, 196, 5, 2, COLOR_HEX_FILL);
  if (barW > 0) {
    canvas.fillRoundRect(22, 134, barW, 5, 2, swRunning ? COLOR_GREEN : COLOR_ACCENT);
  }

  // Three bottom buttons: Start / Stop / Reset
  uint16_t startFill = swRunning ? COLOR_TOPBAR : COLOR_HEX_ACTIVE;
  uint16_t startBord = swRunning ? COLOR_HEX_BORDER : COLOR_GREEN;
  uint16_t startText = swRunning ? COLOR_SUBTEXT : COLOR_GREEN;
  canvas.fillRoundRect(8, 168, 71, 56, 8, startFill);
  canvas.drawRoundRect(8, 168, 71, 56, 8, startBord);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(startText, startFill);
  canvas.drawString("Start", 43, 196);

  uint16_t stopFill = swRunning ? COLOR_HEX_ACTIVE : COLOR_TOPBAR;
  uint16_t stopBord = swRunning ? COLOR_RED : COLOR_HEX_BORDER;
  uint16_t stopText = swRunning ? COLOR_RED : COLOR_SUBTEXT;
  canvas.fillRoundRect(84, 168, 71, 56, 8, stopFill);
  canvas.drawRoundRect(84, 168, 71, 56, 8, stopBord);
  canvas.setTextColor(stopText, stopFill);
  canvas.drawString("Stop", 119, 196);

  bool canReset = (elapsed > 0) || swRunning;
  uint16_t rstFill = canReset ? COLOR_HEX_FILL : COLOR_TOPBAR;
  uint16_t rstBord = canReset ? COLOR_ACCENT : COLOR_HEX_BORDER;
  uint16_t rstText = canReset ? COLOR_TEXT : COLOR_SUBTEXT;
  canvas.fillRoundRect(160, 168, 72, 56, 8, rstFill);
  canvas.drawRoundRect(160, 168, 72, 56, 8, rstBord);
  canvas.setTextColor(rstText, rstFill);
  canvas.drawString("Reset", 196, 196);
}

// Draw Timer screen (Clock-style time edit, Start/Pause/Reset, ringtone selector)
void drawTimerScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  bool isIdle = (!timerRunning && !timerPaused);
  uint32_t remMs = getTimerDisplayRemainingMs();
  uint32_t totalSec = (remMs + 999UL) / 1000UL;
  if (isIdle) totalSec = getTimerSetTotalMs() / 1000UL;

  uint8_t hr  = (uint8_t)((totalSec / 3600UL) % 100UL);
  uint8_t mn  = (uint8_t)((totalSec / 60UL) % 60UL);
  uint8_t sec = (uint8_t)(totalSec % 60UL);

  canvas.setTextDatum(top_center);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(timerRunning ? COLOR_GREEN : (timerPaused ? COLOR_WARN : COLOR_ACCENT), COLOR_BG);
  canvas.drawString(timerRunning ? "TIMER RUNNING" : (timerPaused ? "TIMER PAUSED" : "COUNTDOWN TIMER"), 120, 32);

  // HH:MM:SS time card with active field underline (Clock style)
  canvas.fillRoundRect(8, 52, 224, 54, 7, COLOR_TOPBAR);
  canvas.drawRoundRect(8, 52, 224, 54, 7, timerRunning ? COLOR_GREEN : COLOR_HEX_BORDER);

  char hBuf[6], mBuf[6], sBuf[6];
  snprintf(hBuf, sizeof(hBuf), "%02u", (unsigned)hr);
  snprintf(mBuf, sizeof(mBuf), "%02u", (unsigned)mn);
  snprintf(sBuf, sizeof(sBuf), "%02u", (unsigned)sec);

  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font4);
  canvas.setTextColor(COLOR_TEXT, COLOR_TOPBAR);
  canvas.drawString(hBuf, 50, 77);
  canvas.drawString(":", 85, 75);
  canvas.drawString(mBuf, 120, 77);
  canvas.drawString(":", 155, 75);
  canvas.drawString(sBuf, 190, 77);

  if (isIdle) {
    int16_t ulX = (timerEditField == 0) ? 32 : ((timerEditField == 1) ? 102 : 172);
    canvas.fillRect(ulX, 96, 36, 3, COLOR_WARN);
  } else {
    uint32_t setMs = getTimerSetTotalMs();
    int16_t progW = (setMs > 0) ? (int16_t)((remMs * 204UL) / setMs) : 0;
    if (progW > 204) progW = 204;
    canvas.fillRoundRect(18, 97, 204, 5, 2, COLOR_HEX_FILL);
    if (progW > 0) {
      canvas.fillRoundRect(18, 97, progW, 5, 2, timerRunning ? COLOR_GREEN : COLOR_WARN);
    }
  }

  // Middle row: < / > arrows (Clock style) + Beep / Bell ringtone selector
  canvas.fillRoundRect(8, 114, 50, 36, 7, isIdle ? COLOR_HEX_FILL : COLOR_TOPBAR);
  canvas.drawRoundRect(8, 114, 50, 36, 7, isIdle ? COLOR_ACCENT : COLOR_HEX_BORDER);
  canvas.fillTriangle(21, 132, 41, 122, 41, 142, isIdle ? COLOR_ACCENT : COLOR_HEX_BORDER);

  uint16_t toneCol = (timerRingtone == RINGTONE_BELL) ? COLOR_WARN : COLOR_ACCENT;
  canvas.fillRoundRect(64, 114, 112, 36, 7, COLOR_HEX_ACTIVE);
  canvas.drawRoundRect(64, 114, 112, 36, 7, toneCol);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(toneCol, COLOR_HEX_ACTIVE);
  canvas.drawString((timerRingtone == RINGTONE_BELL) ? "Tone: Bell" : "Tone: Beep", 120, 132);

  canvas.fillRoundRect(182, 114, 50, 36, 7, isIdle ? COLOR_HEX_FILL : COLOR_TOPBAR);
  canvas.drawRoundRect(182, 114, 50, 36, 7, isIdle ? COLOR_ACCENT : COLOR_HEX_BORDER);
  canvas.fillTriangle(219, 132, 199, 122, 199, 142, isIdle ? COLOR_ACCENT : COLOR_HEX_BORDER);

  // Bottom row: Start / Pause / Reset
  bool canStart = (!timerRunning) && (getTimerDisplayRemainingMs() > 0);
  uint16_t startFill = canStart ? COLOR_HEX_ACTIVE : COLOR_TOPBAR;
  uint16_t startBord = canStart ? COLOR_GREEN : COLOR_HEX_BORDER;
  uint16_t startText = canStart ? COLOR_GREEN : COLOR_SUBTEXT;
  canvas.fillRoundRect(8, 160, 71, 56, 8, startFill);
  canvas.drawRoundRect(8, 160, 71, 56, 8, startBord);
  canvas.setTextColor(startText, startFill);
  canvas.drawString("Start", 43, 188);

  uint16_t pauseFill = timerRunning ? COLOR_HEX_ACTIVE : COLOR_TOPBAR;
  uint16_t pauseBord = timerRunning ? COLOR_WARN : COLOR_HEX_BORDER;
  uint16_t pauseText = timerRunning ? COLOR_WARN : COLOR_SUBTEXT;
  canvas.fillRoundRect(84, 160, 71, 56, 8, pauseFill);
  canvas.drawRoundRect(84, 160, 71, 56, 8, pauseBord);
  canvas.setTextColor(pauseText, pauseFill);
  canvas.drawString("Pause", 119, 188);

  uint16_t rstFill = COLOR_HEX_FILL;
  uint16_t rstBord = COLOR_ACCENT;
  canvas.fillRoundRect(160, 160, 72, 56, 8, rstFill);
  canvas.drawRoundRect(160, 160, 72, 56, 8, rstBord);
  canvas.setTextColor(COLOR_TEXT, rstFill);
  canvas.drawString("Reset", 196, 188);

  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  canvas.drawString(isIdle ? "Tap HH/MM/SS + </> to set" : "Countdown active", 120, 228);
}

// Modal alert dialog when alarm (Cancel / Snooze) or timer (OK) triggers
void drawAlertModalOverlay() {
  if (activeAlertType == ALERT_NONE) return;

  bool pulseOn = (((millis() - alertStartedMs) / 450UL) % 2UL) == 0;
  uint16_t borderCol = (activeAlertType == ALERT_ALARM)
                         ? (pulseOn ? COLOR_WARN : COLOR_RED)
                         : (pulseOn ? COLOR_ACCENT : COLOR_GREEN);

  canvas.fillRoundRect(14, 34, 212, 186, 10, COLOR_TOPBAR);
  canvas.drawRoundRect(14, 34, 212, 186, 10, borderCol);
  canvas.drawRoundRect(15, 35, 210, 184, 9, borderCol);

  canvas.setTextDatum(middle_center);

  if (activeAlertType == ALERT_ALARM) {
    uint8_t idx = (activeAlertAlarmIdx < ALARM_COUNT) ? activeAlertAlarmIdx : 0;
    AlarmEntry &al = alarms[idx];

    // Bell icon at top of dialog
    canvas.fillCircle(120, 56, 9, COLOR_WARN);
    canvas.fillRect(111, 56, 18, 8, COLOR_WARN);
    canvas.fillCircle(120, 67, 3, COLOR_TEXT);

    char hdrBuf[20];
    snprintf(hdrBuf, sizeof(hdrBuf), "ALARM %u", (unsigned)(idx + 1));
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_WARN, COLOR_TOPBAR);
    canvas.drawString(hdrBuf, 120, 82);

    char tBuf[10];
    snprintf(tBuf, sizeof(tBuf), "%02u:%02u", (unsigned)al.hour, (unsigned)al.minute);
    canvas.setFont(&fonts::Font4);
    canvas.setTextColor(COLOR_TEXT, COLOR_TOPBAR);
    canvas.drawString(tBuf, 120, 114);

    char infoBuf[28];
    snprintf(infoBuf, sizeof(infoBuf), "%s | Snooze %um",
             (al.ringtone == RINGTONE_BELL) ? "Bell" : "Beep",
             (unsigned)al.snoozeMin);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
    canvas.drawString(infoBuf, 120, 140);

    // Cancel (left) and Snooze (right) buttons
    canvas.fillRoundRect(22, 158, 92, 48, 8, COLOR_HEX_FILL);
    canvas.drawRoundRect(22, 158, 92, 48, 8, COLOR_RED);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_RED, COLOR_HEX_FILL);
    canvas.drawString("Cancel", 68, 182);

    canvas.fillRoundRect(126, 158, 92, 48, 8, COLOR_HEX_ACTIVE);
    canvas.drawRoundRect(126, 158, 92, 48, 8, COLOR_WARN);
    canvas.setTextColor(COLOR_WARN, COLOR_HEX_ACTIVE);
    canvas.drawString("Snooze", 172, 182);
  } else {
    // Modal timer completion dialog with OK button
    canvas.fillCircle(120, 56, 10, COLOR_ACCENT);
    canvas.fillCircle(120, 56, 7, COLOR_TOPBAR);
    canvas.drawLine(120, 56, 120, 50, COLOR_ACCENT);
    canvas.drawLine(120, 56, 125, 56, COLOR_GREEN);

    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_ACCENT, COLOR_TOPBAR);
    canvas.drawString("TIMER FINISHED", 120, 82);

    canvas.setFont(&fonts::Font4);
    canvas.setTextColor(COLOR_TEXT, COLOR_TOPBAR);
    canvas.drawString("00:00:00", 120, 114);

    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_TOPBAR);
    canvas.drawString((timerRingtone == RINGTONE_BELL) ? "Ringtone: Bell" : "Ringtone: Beep", 120, 140);

    canvas.fillRoundRect(56, 158, 128, 48, 8, COLOR_HEX_ACTIVE);
    canvas.drawRoundRect(56, 158, 128, 48, 8, COLOR_GREEN);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_GREEN, COLOR_HEX_ACTIVE);
    canvas.drawString("OK", 120, 182);
  }
}

// Background monitor for 2 alarms, Snooze, and countdown timer
void updateAlarmAndTimer() {
  uint32_t now = millis();

  // 1. Check countdown timer completion (from any screen, including display off)
  if (timerRunning && (int32_t)(timerEndMs - now) <= 0) {
    timerRunning     = false;
    timerPaused      = false;
    timerRemainingMs = 0;
    activeAlertType  = ALERT_TIMER;
    wakeDisplayForAlert(timerRingtone);
    return;
  }

  // 2. Check both alarms and pending Snooze
  if (activeAlertType == ALERT_NONE) {
    int32_t minuteKey = ((int32_t)rtcTime.day * 1440L) + ((int32_t)rtcTime.hour * 60L) + (int32_t)rtcTime.minute;
    for (uint8_t i = 0; i < ALARM_COUNT; i++) {
      if (!alarms[i].enabled) continue;
      if (alarms[i].snoozePending && (int32_t)(alarms[i].snoozeTargetMs - now) <= 0) {
        alarms[i].snoozePending = false;
        activeAlertType     = ALERT_ALARM;
        activeAlertAlarmIdx = i;
        wakeDisplayForAlert(alarms[i].ringtone);
        return;
      }
      if (!alarms[i].snoozePending &&
          rtcTime.hour == alarms[i].hour &&
          rtcTime.minute == alarms[i].minute &&
          rtcTime.second <= 2 &&
          alarms[i].lastTriggeredMin != minuteKey) {
        alarms[i].lastTriggeredMin = minuteKey;
        activeAlertType     = ALERT_ALARM;
        activeAlertAlarmIdx = i;
        wakeDisplayForAlert(alarms[i].ringtone);
        return;
      }
    }
  } else {
    // Keep screen awake and pulse border/haptics while alarm/timer dialog is open
    if (now - lastAlertVibMs >= 1600UL) {
      lastAlertVibMs = now;
      triggerTouchVibration();
    }
    if (now - lastAlertAnimMs >= 450UL) {
      lastAlertAnimMs = now;
      renderCurrentScreen();
    }
  }
}

// ============================================================================
// 7.15. FLASHLIGHT, SOS SIGNAL, AND MORSE CODE (EN + RU) SCREENS & CW AUDIO
// ============================================================================

// FreeRTOS task synthesizing gated CW tone (max-volume SOS 2400 Hz or Morse 800 Hz) via MAX98357A I2S
void cwToneAudioTask(void* arg) {
  (void)arg;
  int16_t stereoBuf[SND_GEN_CHUNK_SAMPLES * 2];
  float phase = 0.0f;
  float env   = 0.0f;

  while (!cwAudioStopReq) {
    bool gate   = cwToneGateOn;
    float freq  = cwToneFreqHz;
    float gain  = cwToneGain;
    float step  = 6.2831853f * freq / (float)SND_GEN_SAMPLE_RATE;

    for (uint16_t i = 0; i < SND_GEN_CHUNK_SAMPLES; i++) {
      // Smooth 2.5 ms attack/release envelope to avoid speaker clicks
      if (gate) {
        env += 0.015f;
        if (env > 1.0f) env = 1.0f;
      } else {
        env -= 0.015f;
        if (env < 0.0f) env = 0.0f;
      }

      float s = 0.0f;
      if (env > 0.001f) {
        if (freq > 1500.0f) {
          // Piercing dual-harmonic tone for maximum-volume SOS beacon
          s = (0.78f * sinf(phase) + 0.22f * sinf(phase * 2.0f)) * env * gain;
        } else {
          // Clean CW telegraph sine tone for Morse code table
          s = sinf(phase) * env * gain;
        }
        phase += step;
        if (phase >= 6.2831853f) phase -= 6.2831853f;
      }

      int32_t s32 = (int32_t)(s * 29500.0f);
      if (s32 > 32767)  s32 = 32767;
      if (s32 < -32767) s32 = -32767;
      int16_t s16 = (int16_t)s32;
      stereoBuf[i * 2]     = s16;
      stereoBuf[i * 2 + 1] = s16;
    }

    size_t written = 0;
#if ESP_IDF_VERSION_MAJOR >= 5
    if (cwTxChan != NULL) {
      i2s_channel_write(cwTxChan, stereoBuf, sizeof(stereoBuf), &written, 60);
    }
#else
    i2s_write(I2S_NUM_1, stereoBuf, sizeof(stereoBuf), &written, pdMS_TO_TICKS(60));
#endif
  }

  memset(stereoBuf, 0, sizeof(stereoBuf));
  size_t written = 0;
#if ESP_IDF_VERSION_MAJOR >= 5
  if (cwTxChan != NULL) {
    i2s_channel_write(cwTxChan, stereoBuf, sizeof(stereoBuf), &written, 40);
  }
#else
  i2s_write(I2S_NUM_1, stereoBuf, sizeof(stereoBuf), &written, pdMS_TO_TICKS(40));
#endif

  cwAudioTaskHandle = NULL;
  vTaskDelete(NULL);
}

void startCwToneAudio(float freqHz, float gain) {
  stopSoundGenerator();
  stopSoundSpectrometer();
  stopAlertRingtone();

  cwToneFreqHz = freqHz;
  cwToneGain   = gain;
  if (cwAudioPlaying && cwAudioTaskHandle != NULL) {
    setSpeakerAmpPower(true);
    return;
  }

  // Power ON MAX98357A speaker amplifier rail (AXP2101 DLDO1) for SOS / Morse audio
  setSpeakerAmpPower(true);
  delay(5);

#if ESP_IDF_VERSION_MAJOR >= 5
  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
  chanCfg.dma_desc_num  = 4;
  chanCfg.dma_frame_num = SND_GEN_CHUNK_SAMPLES;
  if (i2s_new_channel(&chanCfg, &cwTxChan, NULL) != ESP_OK) {
    cwTxChan = NULL;
    updateSpeakerAmpPower();
    return;
  }

  i2s_std_config_t stdCfg;
  memset(&stdCfg, 0, sizeof(stdCfg));
  stdCfg.clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SND_GEN_SAMPLE_RATE);
  stdCfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  stdCfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  stdCfg.gpio_cfg.bclk = (gpio_num_t)SPK_I2S_BCK_PIN;
  stdCfg.gpio_cfg.ws   = (gpio_num_t)SPK_I2S_WS_PIN;
  stdCfg.gpio_cfg.dout = (gpio_num_t)SPK_I2S_DOUT_PIN;
  stdCfg.gpio_cfg.din  = I2S_GPIO_UNUSED;

  if (i2s_channel_init_std_mode(cwTxChan, &stdCfg) != ESP_OK ||
      i2s_channel_enable(cwTxChan) != ESP_OK) {
    i2s_del_channel(cwTxChan);
    cwTxChan = NULL;
    updateSpeakerAmpPower();
    return;
  }
#else
  i2s_config_t i2sCfg;
  memset(&i2sCfg, 0, sizeof(i2sCfg));
  i2sCfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  i2sCfg.sample_rate          = SND_GEN_SAMPLE_RATE;
  i2sCfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
  i2sCfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
  i2sCfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2sCfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
  i2sCfg.dma_buf_count        = 4;
  i2sCfg.dma_buf_len          = SND_GEN_CHUNK_SAMPLES;
  i2sCfg.use_apll             = false;
  i2sCfg.tx_desc_auto_clear   = true;

  i2s_pin_config_t pinCfg;
  memset(&pinCfg, 0, sizeof(pinCfg));
  pinCfg.mck_io_pin   = I2S_PIN_NO_CHANGE;
  pinCfg.bck_io_pin   = SPK_I2S_BCK_PIN;
  pinCfg.ws_io_pin    = SPK_I2S_WS_PIN;
  pinCfg.data_out_num = SPK_I2S_DOUT_PIN;
  pinCfg.data_in_num  = I2S_PIN_NO_CHANGE;

  if (i2s_driver_install(I2S_NUM_1, &i2sCfg, 0, NULL) != ESP_OK) {
    updateSpeakerAmpPower();
    return;
  }
  i2s_set_pin(I2S_NUM_1, &pinCfg);
  i2s_zero_dma_buffer(I2S_NUM_1);
#endif

  cwAudioStopReq = false;
  cwAudioPlaying = true;
  if (xTaskCreatePinnedToCore(cwToneAudioTask, "cw_snd", 3072, NULL, 2, &cwAudioTaskHandle, 0) != pdPASS) {
    cwAudioTaskHandle = NULL;
    cwAudioPlaying    = false;
#if ESP_IDF_VERSION_MAJOR >= 5
    if (cwTxChan != NULL) {
      i2s_channel_disable(cwTxChan);
      i2s_del_channel(cwTxChan);
      cwTxChan = NULL;
    }
#else
    i2s_driver_uninstall(I2S_NUM_1);
#endif
    updateSpeakerAmpPower();
  }
}

void stopCwToneAudio() {
  cwToneGateOn = false;
  if (!cwAudioPlaying && cwAudioTaskHandle == NULL) {
    updateSpeakerAmpPower();
    return;
  }
  cwAudioStopReq = true;
  cwAudioPlaying = false;

  uint32_t waitStart = millis();
  while (cwAudioTaskHandle != NULL && (millis() - waitStart) < 300UL) {
    delay(2);
  }

#if ESP_IDF_VERSION_MAJOR >= 5
  if (cwTxChan != NULL) {
    i2s_channel_disable(cwTxChan);
    i2s_del_channel(cwTxChan);
    cwTxChan = NULL;
  }
#else
  i2s_driver_uninstall(I2S_NUM_1);
#endif
  updateSpeakerAmpPower();
}

// Start continuous emergency SOS beacon (max volume beep, max brightness strobe, DRV2605L vibration)
void startSosSignal() {
  stopMorsePlayback();
  sosActive      = true;
  sosStepIdx     = 0;
  sosStepStartMs = millis();
  sosFlashOn     = (SOS_STEP_ACTIVE[0] != 0);
  blState        = BL_STATE_FULL;
  tft.setBrightness(255); // Maximum display brightness
  startCwToneAudio(2400.0f, 1.0f);
  cwToneGateOn = sosFlashOn;
  setMotorVibrate(sosFlashOn);
}

void stopSosSignal() {
  sosActive    = false;
  sosFlashOn   = false;
  cwToneGateOn = false;
  setMotorVibrate(false);
  stopCwToneAudio();
  tft.setBrightness(getFullBrightnessPwm());
}

void updateSosSignal() {
  if (!sosActive) return;
  uint32_t now = millis();
  uint32_t stepDur = (uint32_t)SOS_STEP_UNITS[sosStepIdx] * SOS_UNIT_MS;
  if (now - sosStepStartMs >= stepDur) {
    sosStepStartMs = now;
    sosStepIdx = (uint8_t)((sosStepIdx + 1) % SOS_STEP_COUNT);
    bool nextOn = (SOS_STEP_ACTIVE[sosStepIdx] != 0);
    if (nextOn != sosFlashOn) {
      sosFlashOn   = nextOn;
      cwToneGateOn = nextOn;
      setMotorVibrate(nextOn);
    }
    tft.setBrightness(255);
    renderCurrentScreen();
  }
}

// Start playing a single Morse code entry from MORSE_TABLE[]
void startMorsePlayback(uint8_t idx) {
  if (idx >= MORSE_TOTAL_COUNT) return;
  if (sosActive) stopSosSignal();
  morsePlayingIdx  = (int8_t)idx;
  morsePlayElemIdx = 0;
  morsePlayToneOn  = true;
  const char* code = MORSE_TABLE[idx].code;
  uint32_t dur     = (code[0] == '-') ? (MORSE_UNIT_MS * 3UL) : MORSE_UNIT_MS;
  morsePlayNextMs  = millis() + dur;
  startCwToneAudio(800.0f, 0.72f);
  cwToneGateOn = true;
}

void stopMorsePlayback() {
  morsePlayingIdx  = -1;
  morsePlayElemIdx = 0;
  morsePlayToneOn  = false;
  cwToneGateOn     = false;
  stopCwToneAudio();
}

void updateMorsePlayback() {
  if (morsePlayingIdx < 0 || morsePlayingIdx >= MORSE_TOTAL_COUNT) return;
  uint32_t now = millis();
  if ((int32_t)(now - morsePlayNextMs) < 0) return;

  const char* code = MORSE_TABLE[morsePlayingIdx].code;
  uint8_t len      = (uint8_t)strlen(code);

  if (morsePlayToneOn) {
    // Finished current dot/dash; enter intra-character silence gap or stop
    cwToneGateOn    = false;
    morsePlayToneOn = false;
    if (morsePlayElemIdx + 1 < len) {
      morsePlayNextMs = now + MORSE_UNIT_MS;
      renderCurrentScreen();
    } else {
      stopMorsePlayback();
      renderCurrentScreen();
    }
  } else {
    // Finished silence gap; play next dot or dash
    morsePlayElemIdx++;
    if (morsePlayElemIdx < len) {
      morsePlayToneOn = true;
      cwToneGateOn    = true;
      uint32_t dur    = (code[morsePlayElemIdx] == '-') ? (MORSE_UNIT_MS * 3UL) : MORSE_UNIT_MS;
      morsePlayNextMs = now + dur;
      renderCurrentScreen();
    } else {
      stopMorsePlayback();
      renderCurrentScreen();
    }
  }
}

// Draw Flashlight screen (ON/OFF, 8 colors: white..cyan, brightness slider 5..100%, top-left X)
void drawFlashlightScreen() {
  const FlashlightColorItem& col = FLASHLIGHT_COLORS[flashlightColorIdx % FLASHLIGHT_COLOR_COUNT];
  uint8_t shownBri = flashlightBrightness;
  if (shownBri < BRIGHTNESS_MIN_PCT) shownBri = BRIGHTNESS_MIN_PCT;
  if (shownBri > 100) shownBri = 100;

  if (flashlightOn) {
    // Full-screen solid floodlight illumination in selected color
    canvas.fillScreen(col.rgb565);

    // High-contrast top-left close X button (always visible on any flood color)
    canvas.fillRoundRect(4, 4, 34, 24, 6, COLOR_BG);
    canvas.drawRoundRect(4, 4, 34, 24, 6, COLOR_RED);
    canvas.drawLine(15, 10, 27, 22, COLOR_RED);
    canvas.drawLine(27, 10, 15, 22, COLOR_RED);

    // Compact dark dock at bottom so upper area stays pure floodlight
    canvas.fillRoundRect(8, 126, 224, 108, 10, COLOR_BG);
    canvas.drawRoundRect(8, 126, 224, 108, 10, COLOR_HEX_BORDER);
  } else {
    canvas.fillScreen(COLOR_BG);
    drawTopStatusBar();
    drawTopLeftCloseIcon();

    // Lamp status and selected color preview card
    canvas.fillRoundRect(12, 36, 216, 80, 10, COLOR_HEX_FILL);
    canvas.drawRoundRect(12, 36, 216, 80, 10, COLOR_HEX_BORDER);

    // Lamp swatch circle
    canvas.fillCircle(48, 76, 22, col.rgb565);
    canvas.drawCircle(48, 76, 22, COLOR_TEXT);
    canvas.drawCircle(48, 76, 24, COLOR_HEX_BORDER);

    canvas.setTextDatum(middle_left);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
    canvas.drawString("Flashlight", 82, 60);

    char stBuf[24];
    snprintf(stBuf, sizeof(stBuf), "OFF  |  %u%%", (unsigned)shownBri);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
    canvas.drawString(stBuf, 82, 82);

    canvas.setTextColor(COLOR_ACCENT, COLOR_HEX_FILL);
    canvas.drawString(col.name, 82, 98);
  }

  // Brightness slider (x = 22..218, y = 138, 5..100%)
  int16_t trackX = 22, trackY = 138, trackW = 196, trackH = 12;
  canvas.fillRoundRect(trackX, trackY, trackW, trackH, 6, COLOR_TOPBAR);
  canvas.drawRoundRect(trackX, trackY, trackW, trackH, 6, COLOR_HEX_BORDER);

  int16_t fillW = (int16_t)(((uint32_t)(shownBri - BRIGHTNESS_MIN_PCT) * (uint32_t)trackW) / (100U - BRIGHTNESS_MIN_PCT));
  if (fillW > 2) {
    canvas.fillRoundRect(trackX, trackY, fillW, trackH, 6, COLOR_ACCENT);
  }
  int16_t knobX = (int16_t)(trackX + fillW);
  canvas.fillCircle(knobX, trackY + trackH / 2, 9, COLOR_TEXT);
  canvas.drawCircle(knobX, trackY + trackH / 2, 9, COLOR_ACCENT);

  char briLbl[18];
  snprintf(briLbl, sizeof(briLbl), "Bri: %u%%", (unsigned)shownBri);
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(top_center);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  canvas.drawString(briLbl, 120, 154);

  // Color selector button (left, y = 166..194) and ON/OFF button (bottom, y = 198..230)
  canvas.fillRoundRect(14, 164, 212, 30, 7, COLOR_HEX_FILL);
  canvas.drawRoundRect(14, 164, 212, 30, 7, COLOR_ACCENT);
  canvas.fillCircle(32, 179, 8, col.rgb565);
  canvas.drawCircle(32, 179, 8, COLOR_TEXT);
  char colBuf[24];
  snprintf(colBuf, sizeof(colBuf), "Color: %s", col.name);
  canvas.setTextDatum(middle_left);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
  canvas.drawString(colBuf, 48, 179);
  canvas.fillTriangle(216, 179, 208, 174, 208, 184, COLOR_ACCENT);

  // ON / OFF primary toggle button
  uint16_t btnFill = flashlightOn ? COLOR_RED : COLOR_GREEN;
  uint16_t btnText = flashlightOn ? COLOR_TEXT : COLOR_BG;
  canvas.fillRoundRect(14, 198, 212, 32, 8, btnFill);
  canvas.drawRoundRect(14, 198, 212, 32, 8, COLOR_TEXT);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(btnText, btnFill);
  canvas.drawString(flashlightOn ? "OFF  (TURN OFF)" : "ON  (TURN ON)", 120, 214);
}

// Draw SOS signal screen (continuous max-volume beep, max-brightness strobe, motor vibration)
void drawSosScreen() {
  uint16_t bgCol   = sosFlashOn ? 0xFFFF : 0x0000;
  uint16_t sosCol  = sosFlashOn ? COLOR_RED : 0x7800;
  canvas.fillScreen(bgCol);

  // Always-visible top-left close X button to stop SOS and exit to main menu
  canvas.fillRoundRect(4, 4, 34, 24, 6, COLOR_BG);
  canvas.drawRoundRect(4, 4, 34, 24, 6, COLOR_RED);
  canvas.drawLine(15, 10, 27, 22, COLOR_RED);
  canvas.drawLine(27, 10, 15, 22, COLOR_RED);

  // Active status badge top-right
  canvas.fillRoundRect(148, 6, 86, 20, 5, sosFlashOn ? COLOR_RED : COLOR_TOPBAR);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_TEXT, sosFlashOn ? COLOR_RED : COLOR_TOPBAR);
  canvas.drawString(sosFlashOn ? "TX ACTIVE" : "STANDBY", 191, 16);

  // Large SOS beacon text
  canvas.setFont(&fonts::Font4);
  canvas.setTextSize(2);
  canvas.setTextColor(sosCol, bgCol);
  canvas.drawString("SOS", 120, 86);
  canvas.setTextSize(1);

  // 9-element Morse visual bar: ... --- ...
  int8_t curSym = SOS_STEP_SYM[sosStepIdx % SOS_STEP_COUNT];
  int16_t sx = 18;
  int16_t sy = 148;
  for (int8_t e = 0; e < 9; e++) {
    bool isDash = (e >= 3 && e <= 5);
    bool isLit  = (e == curSym && sosFlashOn);
    uint16_t elCol = isLit ? COLOR_RED : (sosFlashOn ? 0xC618 : 0x39E7);
    if (isDash) {
      canvas.fillRoundRect(sx, sy - 5, 20, 10, 3, elCol);
      sx += 25;
    } else {
      canvas.fillCircle(sx + 5, sy, 5, elCol);
      sx += 15;
    }
    if (e == 2 || e == 5) sx += 10; // Inter-letter spacing
  }

  // Hardware telemetry footer card
  canvas.fillRoundRect(12, 182, 216, 46, 8, COLOR_BG);
  canvas.drawRoundRect(12, 182, 216, 46, 8, sosFlashOn ? COLOR_RED : COLOR_HEX_BORDER);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_TEXT, COLOR_BG);
  canvas.drawString("MAX VOL 2400Hz | MAX BRI 100%", 120, 197);
  canvas.setTextColor(sosFlashOn ? COLOR_WARN : COLOR_SUBTEXT, COLOR_BG);
  canvas.drawString(sosFlashOn ? "BUZZ + FLASH: ON  (Tap X to stop)" : "MOTOR + LED: GAP  (Tap X to stop)", 120, 215);
}

// Draw Morse code table screen (analogous to ASCII table: EN + RU + Digits + Punctuation, tap row to play)
void drawMorseCodeScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  // Top-right quick section jump button (EN -> RU -> 0-9 -> SYM -> EN)
  const char* nextSecLbl = "RU";
  if (morseScrollOffset >= MORSE_SYM_OFFSET) {
    nextSecLbl = "EN";
  } else if (morseScrollOffset >= MORSE_NUM_OFFSET) {
    nextSecLbl = "SYM";
  } else if (morseScrollOffset >= MORSE_EN_COUNT) {
    nextSecLbl = "0-9";
  }
  canvas.fillRoundRect(198, 4, 38, 22, 5, COLOR_HEX_FILL);
  canvas.drawRoundRect(198, 4, 38, 22, 5, COLOR_ACCENT);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_ACCENT, COLOR_HEX_FILL);
  canvas.drawString(nextSecLbl, 217, 15);

  // Table header
  canvas.fillRect(6, 32, 188, 16, COLOR_TOPBAR);
  canvas.drawRect(6, 32, 188, 16, COLOR_HEX_BORDER);
  canvas.setTextDatum(middle_left);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_ACCENT, COLOR_TOPBAR);
  canvas.drawString("CHR", 12, 40);
  canvas.drawString("CAT", 46, 40);
  canvas.drawString("MORSE (TAP)", 82, 40);

  // Right vertical scroll bar (top and bottom triangles, analogous to ASCII table)
  bool canUp   = (morseScrollOffset > 0);
  bool canDown = ((uint16_t)morseScrollOffset + MORSE_VISIBLE_ROWS < MORSE_TOTAL_COUNT);

  canvas.fillRoundRect(199, 34, 36, 56, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(199, 34, 36, 56, 6, canUp ? COLOR_ACCENT : COLOR_HEX_BORDER);
  canvas.fillTriangle(217, 52, 208, 70, 226, 70, canUp ? COLOR_ACCENT : COLOR_SUBTEXT);

  canvas.fillRoundRect(199, 182, 36, 56, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(199, 182, 36, 56, 6, canDown ? COLOR_ACCENT : COLOR_HEX_BORDER);
  canvas.fillTriangle(217, 220, 208, 202, 226, 202, canDown ? COLOR_ACCENT : COLOR_SUBTEXT);

  uint8_t totalPages = (uint8_t)((MORSE_TOTAL_COUNT + MORSE_VISIBLE_ROWS - 1) / MORSE_VISIBLE_ROWS);
  uint8_t curPage    = (uint8_t)(morseScrollOffset / MORSE_VISIBLE_ROWS + 1);
  if (curPage > totalPages) curPage = totalPages;
  char pageBuf[12];
  snprintf(pageBuf, sizeof(pageBuf), "%u/%u", (unsigned)curPage, (unsigned)totalPages);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  canvas.drawString(pageBuf, 217, 136);

  // 7 visible table rows (y = 50 + r * 26, height 24 px each)
  for (uint8_t r = 0; r < MORSE_VISIBLE_ROWS; r++) {
    uint8_t idx = (uint8_t)(morseScrollOffset + r);
    if (idx >= MORSE_TOTAL_COUNT) break;
    const MorseCodeEntry& entry = MORSE_TABLE[idx];
    bool isPlaying = (morsePlayingIdx == (int8_t)idx);

    int16_t ry = (int16_t)(50 + r * 26);
    uint16_t rowBg = isPlaying ? COLOR_HEX_ACTIVE : ((r % 2 == 0) ? COLOR_HEX_FILL : COLOR_TOPBAR);
    canvas.fillRect(6, ry, 188, 24, rowBg);
    if (isPlaying) {
      canvas.drawRect(6, ry, 188, 24, COLOR_ACCENT);
    } else {
      canvas.drawFastHLine(6, ry + 24, 188, COLOR_BG);
    }

    // Column 1: Glyph (ASCII Font2 for EN/Digits/Punctuation, CP1251 5x7 bitmap for RU)
    uint16_t chrCol = isPlaying ? COLOR_WARN : COLOR_GREEN;
    if (entry.isRu != 1) {
      char chBuf[2] = {(char)entry.cp1251, '\0'};
      canvas.setTextDatum(middle_left);
      canvas.setFont(&fonts::Font2);
      canvas.setTextColor(chrCol, rowBg);
      canvas.drawString(chBuf, 14, ry + 12);
    } else {
      drawCp1251Glyph5x7(entry.cp1251, 14, (int16_t)(ry + 5), chrCol, 2);
    }

    // Column 2: Category tag (EN / RU / 0-9 / SYM)
    const char* tagStr = "EN";
    uint16_t tagCol    = COLOR_ACCENT;
    if (entry.isRu == 1) {
      tagStr = "RU";
      tagCol = COLOR_WARN;
    } else if (entry.isRu == 2) {
      tagStr = "0-9";
      tagCol = COLOR_GREEN;
    } else if (entry.isRu == 3) {
      tagStr = "SYM";
      tagCol = 0xF81F;
    }
    canvas.setTextDatum(middle_left);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(tagCol, rowBg);
    canvas.drawString(tagStr, 46, ry + 12);

    // Column 3: Graphical Morse dots and dashes (highlights active sounding element)
    int16_t mx = 80;
    int16_t my = (int16_t)(ry + 12);
    for (uint8_t c = 0; entry.code[c] != '\0'; c++) {
      bool elemLit = (isPlaying && morsePlayToneOn && c == morsePlayElemIdx);
      uint16_t symCol = elemLit ? 0xFFE0 : (isPlaying ? COLOR_ACCENT : COLOR_TEXT);
      if (entry.code[c] == '-') {
        canvas.fillRoundRect(mx, my - 3, 14, 6, 2, symCol);
        mx += 18;
      } else {
        canvas.fillCircle(mx + 3, my, 3, symCol);
        mx += 10;
      }
    }
  }
}

// ----------------------------------------------------------------------------
// TETRIS GAME ENGINE & RENDERER (10x20 well, 7 tetrominoes, wall-kick, score)
// ----------------------------------------------------------------------------
bool tetrisCanPlace(uint8_t type, uint8_t rot, int8_t px, int8_t py) {
  uint16_t mask = TETRIS_SHAPES[type % 7][rot & 3];
  for (int8_t r = 0; r < 4; r++) {
    for (int8_t c = 0; c < 4; c++) {
      uint16_t bit = 0x8000u >> (r * 4 + c);
      if (!(mask & bit)) continue;
      int8_t gx = px + c;
      int8_t gy = py + r;
      if (gx < 0 || gx >= TETRIS_COLS || gy >= TETRIS_ROWS) return false;
      if (gy >= 0 && tetrisBoard[gy][gx] != 0) return false;
    }
  }
  return true;
}

int8_t tetrisComputeGhostY() {
  int8_t gy = tetrisCurY;
  while (tetrisCanPlace(tetrisCurType, tetrisCurRot, tetrisCurX, gy + 1)) {
    gy++;
  }
  return gy;
}

uint32_t tetrisDropIntervalMs() {
  int32_t ms = 700 - (int32_t)(tetrisLevel - 1) * 55;
  return (uint32_t)((ms < 120) ? 120 : ms);
}

void tetrisResetGame() {
  memset(tetrisBoard, 0, sizeof(tetrisBoard));
  tetrisCurType  = (uint8_t)(esp_random() % 7);
  tetrisNextType = (uint8_t)(esp_random() % 7);
  tetrisCurRot   = 0;
  tetrisCurX     = 3;
  tetrisCurY     = 0;
  tetrisScore    = 0;
  tetrisLines    = 0;
  tetrisLevel    = 1;
  tetrisPaused   = false;
  tetrisGameOver = false;
  lastTetrisDropMs = millis();
}

void tetrisLockAndSpawn() {
  uint16_t mask = TETRIS_SHAPES[tetrisCurType % 7][tetrisCurRot & 3];
  for (int8_t r = 0; r < 4; r++) {
    for (int8_t c = 0; c < 4; c++) {
      uint16_t bit = 0x8000u >> (r * 4 + c);
      if (!(mask & bit)) continue;
      int8_t gx = tetrisCurX + c;
      int8_t gy = tetrisCurY + r;
      if (gy >= 0 && gy < TETRIS_ROWS && gx >= 0 && gx < TETRIS_COLS) {
        tetrisBoard[gy][gx] = tetrisCurType + 1;
      }
    }
  }

  // Clear any completed horizontal rows
  uint8_t cleared = 0;
  for (int8_t row = TETRIS_ROWS - 1; row >= 0; row--) {
    bool full = true;
    for (int8_t col = 0; col < TETRIS_COLS; col++) {
      if (tetrisBoard[row][col] == 0) {
        full = false;
        break;
      }
    }
    if (full) {
      cleared++;
      for (int8_t shift = row; shift > 0; shift--) {
        memcpy(tetrisBoard[shift], tetrisBoard[shift - 1], TETRIS_COLS);
      }
      memset(tetrisBoard[0], 0, TETRIS_COLS);
      row++; // re-check shifted row
    }
  }

  if (cleared > 0) {
    static const uint16_t LINE_PTS[5] = {0, 100, 300, 500, 800};
    uint8_t idx = (cleared > 4) ? 4 : cleared;
    tetrisScore += (uint32_t)LINE_PTS[idx] * (uint32_t)tetrisLevel;
    tetrisLines += cleared;
    uint16_t lvl = 1 + (tetrisLines / 10);
    tetrisLevel = (uint8_t)((lvl > 15) ? 15 : lvl);
  }

  // Spawn next tetromino
  tetrisCurType  = tetrisNextType;
  tetrisNextType = (uint8_t)(esp_random() % 7);
  tetrisCurRot   = 0;
  tetrisCurX     = 3;
  tetrisCurY     = 0;
  lastTetrisDropMs = millis();

  if (!tetrisCanPlace(tetrisCurType, tetrisCurRot, tetrisCurX, tetrisCurY)) {
    tetrisGameOver = true;
  }
}

void tetrisMoveHoriz(int8_t dx) {
  if (tetrisPaused || tetrisGameOver) return;
  if (tetrisCanPlace(tetrisCurType, tetrisCurRot, tetrisCurX + dx, tetrisCurY)) {
    tetrisCurX += dx;
  }
}

void tetrisRotateCW() {
  if (tetrisPaused || tetrisGameOver) return;
  uint8_t nextRot = (uint8_t)((tetrisCurRot + 1) & 3);
  static const int8_t KICKS[5] = {0, -1, 1, -2, 2};
  for (uint8_t k = 0; k < 5; k++) {
    if (tetrisCanPlace(tetrisCurType, nextRot, tetrisCurX + KICKS[k], tetrisCurY)) {
      tetrisCurRot = nextRot;
      tetrisCurX += KICKS[k];
      return;
    }
  }
}

void tetrisStepDown(bool softDrop) {
  if (tetrisPaused || tetrisGameOver) return;
  if (tetrisCanPlace(tetrisCurType, tetrisCurRot, tetrisCurX, tetrisCurY + 1)) {
    tetrisCurY++;
    if (softDrop) tetrisScore += 1;
  } else {
    tetrisLockAndSpawn();
  }
  lastTetrisDropMs = millis();
}

void updateTetrisGame() {
  if (tetrisPaused || tetrisGameOver) return;
  uint32_t now = millis();
  if (now - lastTetrisDropMs >= tetrisDropIntervalMs()) {
    tetrisStepDown(false);
    renderCurrentScreen();
  }
}

void drawTetrisScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  // Top-right Restart button (x=198..236, y=4..26)
  canvas.fillRoundRect(198, 4, 38, 22, 5, COLOR_HEX_FILL);
  canvas.drawRoundRect(198, 4, 38, 22, 5, COLOR_ACCENT);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_ACCENT, COLOR_HEX_FILL);
  canvas.drawString("RST", 217, 15);

  // 10x20 Playing field (bx=16, by=31, cell=8 -> 80x160 px, y=31..191)
  const int16_t bx = 16;
  const int16_t by = 31;
  const int16_t cs = 8;
  canvas.fillRect(bx - 1, by - 1, TETRIS_COLS * cs + 2, TETRIS_ROWS * cs + 2, 0x0000);
  canvas.drawRect(bx - 2, by - 2, TETRIS_COLS * cs + 4, TETRIS_ROWS * cs + 4, COLOR_HEX_BORDER);

  // Subtle grid lines
  for (int8_t c = 1; c < TETRIS_COLS; c++) {
    canvas.drawFastVLine(bx + c * cs, by, TETRIS_ROWS * cs, 0x1082);
  }
  for (int8_t r = 1; r < TETRIS_ROWS; r++) {
    canvas.drawFastHLine(bx, by + r * cs, TETRIS_COLS * cs, 0x1082);
  }

  // Locked blocks
  for (int8_t r = 0; r < TETRIS_ROWS; r++) {
    for (int8_t c = 0; c < TETRIS_COLS; c++) {
      uint8_t cell = tetrisBoard[r][c];
      if (cell > 0 && cell <= 7) {
        uint16_t col = TETRIS_COLORS[cell - 1];
        canvas.fillRect(bx + c * cs + 1, by + r * cs + 1, cs - 2, cs - 2, col);
      }
    }
  }

  // Ghost piece preview + active falling piece
  if (!tetrisGameOver) {
    uint16_t mask = TETRIS_SHAPES[tetrisCurType % 7][tetrisCurRot & 3];
    int8_t ghostY = tetrisComputeGhostY();
    uint16_t activeCol = TETRIS_COLORS[tetrisCurType % 7];
    for (int8_t r = 0; r < 4; r++) {
      for (int8_t c = 0; c < 4; c++) {
        uint16_t bit = 0x8000u >> (r * 4 + c);
        if (!(mask & bit)) continue;
        int8_t gx = tetrisCurX + c;
        int8_t gyGhost = ghostY + r;
        if (gx >= 0 && gx < TETRIS_COLS && gyGhost >= 0 && gyGhost < TETRIS_ROWS) {
          canvas.drawRect(bx + gx * cs + 1, by + gyGhost * cs + 1, cs - 2, cs - 2, COLOR_HEX_BORDER);
        }
        int8_t gyCur = tetrisCurY + r;
        if (gx >= 0 && gx < TETRIS_COLS && gyCur >= 0 && gyCur < TETRIS_ROWS) {
          canvas.fillRect(bx + gx * cs + 1, by + gyCur * cs + 1, cs - 2, cs - 2, activeCol);
        }
      }
    }
  }

  // Right HUD panel (x=106..232): SCORE, NEXT, LVL/LINES
  // SCORE card (y=31..73)
  canvas.fillRoundRect(106, 31, 126, 42, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(106, 31, 126, 42, 6, COLOR_HEX_BORDER);
  canvas.setTextDatum(top_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
  canvas.drawString("SCORE", 169, 36);
  char scoreBuf[16];
  snprintf(scoreBuf, sizeof(scoreBuf), "%lu", (unsigned long)tetrisScore);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COLOR_ACCENT, COLOR_HEX_FILL);
  canvas.drawString(scoreBuf, 169, 49);

  // NEXT card (y=78..136)
  canvas.fillRoundRect(106, 78, 126, 58, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(106, 78, 126, 58, 6, COLOR_HEX_BORDER);
  canvas.setTextDatum(top_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
  canvas.drawString("NEXT", 169, 83);
  uint16_t nextMask = TETRIS_SHAPES[tetrisNextType % 7][0];
  uint16_t nextCol  = TETRIS_COLORS[tetrisNextType % 7];
  const int16_t nx0 = 151;
  const int16_t ny0 = 96;
  const int16_t ncs = 9;
  for (int8_t r = 0; r < 4; r++) {
    for (int8_t c = 0; c < 4; c++) {
      if (nextMask & (0x8000u >> (r * 4 + c))) {
        canvas.fillRect(nx0 + c * ncs + 1, ny0 + r * ncs + 1, ncs - 2, ncs - 2, nextCol);
      }
    }
  }

  // LVL & LINES card (y=141..191)
  canvas.fillRoundRect(106, 141, 126, 50, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(106, 141, 126, 50, 6, tetrisPaused ? COLOR_WARN : COLOR_HEX_BORDER);
  canvas.setTextDatum(middle_left);
  canvas.setFont(&fonts::Font0);
  char statBuf[24];
  snprintf(statBuf, sizeof(statBuf), "LVL:   %u", (unsigned)tetrisLevel);
  canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
  canvas.drawString(statBuf, 116, 155);
  snprintf(statBuf, sizeof(statBuf), "LINES: %u", (unsigned)tetrisLines);
  canvas.drawString(statBuf, 116, 169);
  canvas.setTextDatum(middle_center);
  if (tetrisPaused) {
    canvas.setTextColor(COLOR_WARN, COLOR_HEX_FILL);
    canvas.drawString("PAUSED", 169, 182);
  } else {
    canvas.setTextColor(COLOR_GREEN, COLOR_HEX_FILL);
    canvas.drawString("PLAYING", 169, 182);
  }

  // Bottom 5 control buttons (y=196..236): Left, Right, Down, Rotate, Pause
  const int16_t btnY = 196;
  const int16_t btnH = 40;
  for (uint8_t b = 0; b < 5; b++) {
    int16_t bxBtn = 4 + b * 47;
    int16_t bwBtn = 44;
    bool isPauseActive = (b == 4 && tetrisPaused);
    uint16_t bgCol = isPauseActive ? COLOR_WARN : COLOR_HEX_FILL;
    uint16_t bdCol = isPauseActive ? COLOR_WARN : COLOR_HEX_BORDER;
    uint16_t fgCol = isPauseActive ? COLOR_BG : COLOR_TEXT;
    canvas.fillRoundRect(bxBtn, btnY, bwBtn, btnH, 6, bgCol);
    canvas.drawRoundRect(bxBtn, btnY, bwBtn, btnH, 6, bdCol);
    int16_t cx = bxBtn + bwBtn / 2;
    int16_t cy = btnY + btnH / 2;
    if (b == 0) {
      // Left arrow
      canvas.fillTriangle(cx - 8, cy, cx + 6, cy - 8, cx + 6, cy + 8, fgCol);
    } else if (b == 1) {
      // Right arrow
      canvas.fillTriangle(cx + 8, cy, cx - 6, cy - 8, cx - 6, cy + 8, fgCol);
    } else if (b == 2) {
      // Down arrow
      canvas.fillTriangle(cx, cy + 8, cx - 8, cy - 6, cx + 8, cy - 6, COLOR_ACCENT);
    } else if (b == 3) {
      // Rotate icon (circular arrow)
      canvas.drawCircle(cx, cy, 8, COLOR_ACCENT);
      canvas.fillRect(cx + 2, cy - 10, 8, 8, bgCol);
      canvas.fillTriangle(cx + 9, cy - 5, cx + 3, cy - 9, cx + 4, cy - 2, COLOR_ACCENT);
    } else {
      // Pause / Play icon
      if (tetrisPaused) {
        canvas.fillTriangle(cx + 7, cy, cx - 5, cy - 7, cx - 5, cy + 7, fgCol);
      } else {
        canvas.fillRect(cx - 6, cy - 7, 4, 14, fgCol);
        canvas.fillRect(cx + 2, cy - 7, 4, 14, fgCol);
      }
    }
  }

  // Game Over modal dialog with OK button to restart
  if (tetrisGameOver) {
    canvas.fillRoundRect(28, 62, 184, 118, 10, COLOR_TOPBAR);
    canvas.drawRoundRect(28, 62, 184, 118, 10, COLOR_RED);
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font4);
    canvas.setTextColor(COLOR_RED, COLOR_TOPBAR);
    canvas.drawString("Game over", 120, 86);
    char goBuf[24];
    snprintf(goBuf, sizeof(goBuf), "Score: %lu", (unsigned long)tetrisScore);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_TEXT, COLOR_TOPBAR);
    canvas.drawString(goBuf, 120, 112);

    canvas.fillRoundRect(64, 132, 112, 36, 8, COLOR_GREEN);
    canvas.drawRoundRect(64, 132, 112, 36, 8, COLOR_TEXT);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(COLOR_BG, COLOR_GREEN);
    canvas.drawString("OK", 120, 150);
  }
}

// ----------------------------------------------------------------------------
// PAINT DRAWING PROGRAM (120x86 4-bit canvas, 4 brushes, Flash save/auto-load)
// ----------------------------------------------------------------------------
inline uint8_t paintGetPixel(int16_t lx, int16_t ly) {
  if (lx < 0 || lx >= PAINT_BUF_W || ly < 0 || ly >= PAINT_BUF_H) return 0;
  uint32_t idx = (uint32_t)ly * PAINT_BUF_W + (uint32_t)lx;
  uint8_t b = paintCanvas[idx >> 1];
  return (idx & 1u) ? (uint8_t)((b >> 4) & 0x0F) : (uint8_t)(b & 0x0F);
}

inline void paintSetPixel(int16_t lx, int16_t ly, uint8_t colorId) {
  if (lx < 0 || lx >= PAINT_BUF_W || ly < 0 || ly >= PAINT_BUF_H) return;
  uint32_t idx = (uint32_t)ly * PAINT_BUF_W + (uint32_t)lx;
  uint8_t &b = paintCanvas[idx >> 1];
  if (idx & 1u) {
    b = (uint8_t)((b & 0x0F) | ((colorId & 0x0F) << 4));
  } else {
    b = (uint8_t)((b & 0xF0) | (colorId & 0x0F));
  }
}

void paintClearCanvas() {
  memset(paintCanvas, 0, sizeof(paintCanvas));
  paintStrokeStep = 0;
  paintSaveBannerMs = 0;
}

void savePaintToFlash() {
  if (!paintPrefs.begin("paint_img", false)) return;
  paintPrefs.putUChar("ver", 1);
  paintPrefs.putUChar("fg", paintColorIdx % PAINT_COLOR_COUNT);
  paintPrefs.putUChar("bg", paintBgIdx % PAINT_BG_COUNT);
  paintPrefs.putUChar("sz", paintSizeIdx % PAINT_SIZE_COUNT);
  paintPrefs.putUChar("br", paintBrushIdx % PAINT_BRUSH_COUNT);
  paintPrefs.putBytes("p0", paintCanvas, PAINT_CHUNK_BYTES);
  paintPrefs.putBytes("p1", paintCanvas + PAINT_CHUNK_BYTES, PAINT_BUF_BYTES - PAINT_CHUNK_BYTES);
  paintPrefs.end();
}

void loadPaintFromFlash() {
  paintClearCanvas();
  if (!paintPrefs.begin("paint_img", true)) return;
  uint8_t ver = paintPrefs.getUChar("ver", 0);
  if (ver == 1) {
    paintColorIdx = paintPrefs.getUChar("fg", 5) % PAINT_COLOR_COUNT;
    paintBgIdx    = paintPrefs.getUChar("bg", 0) % PAINT_BG_COUNT;
    paintSizeIdx  = paintPrefs.getUChar("sz", 1) % PAINT_SIZE_COUNT;
    paintBrushIdx = paintPrefs.getUChar("br", 0) % PAINT_BRUSH_COUNT;
    size_t n0 = paintPrefs.getBytesLength("p0");
    size_t n1 = paintPrefs.getBytesLength("p1");
    if (n0 == PAINT_CHUNK_BYTES && n1 == (PAINT_BUF_BYTES - PAINT_CHUNK_BYTES)) {
      paintPrefs.getBytes("p0", paintCanvas, PAINT_CHUNK_BYTES);
      paintPrefs.getBytes("p1", paintCanvas + PAINT_CHUNK_BYTES, PAINT_BUF_BYTES - PAINT_CHUNK_BYTES);
    }
  }
  paintPrefs.end();
}

static uint32_t paintSprayHash(int32_t x, int32_t y, uint32_t step) {
  uint32_t h = (uint32_t)(x * 374761393L + y * 668265263L) + step * 1442695041UL;
  h = (h ^ (h >> 13)) * 1274126177UL;
  return h ^ (h >> 16);
}

void paintStampBrush(int16_t lx, int16_t ly, uint8_t colorId, uint8_t size, uint8_t brushIdx, uint32_t step) {
  // Brush 3 = Dashed: skip alternating 4-step segments
  if (brushIdx == 3 && ((step / 4UL) & 1UL) == 1UL) return;

  int16_t radius = (size > 0) ? (int16_t)(size - 1) : 0;

  if (brushIdx == 2) {
    // Brush 2 = Spray airbrush scatter
    int16_t sprayR = (size + 2 < 2) ? 2 : (int16_t)(size + 2);
    uint8_t dots   = (size * 3 < 4) ? 4 : (uint8_t)(size * 3);
    for (uint8_t i = 0; i < dots; i++) {
      uint32_t h1 = paintSprayHash(lx + i, ly, step + (uint32_t)i * 7UL);
      uint32_t h2 = paintSprayHash(lx, ly + i, step + (uint32_t)i * 13UL + 3UL);
      int16_t ox = (int16_t)(h1 % (uint32_t)(sprayR * 2 + 1)) - sprayR;
      int16_t oy = (int16_t)(h2 % (uint32_t)(sprayR * 2 + 1)) - sprayR;
      if (ox * ox + oy * oy <= sprayR * sprayR) {
        paintSetPixel(lx + ox, ly + oy, colorId);
      }
    }
    return;
  }

  for (int16_t dy = -radius; dy <= radius; dy++) {
    for (int16_t dx = -radius; dx <= radius; dx++) {
      if (brushIdx == 0 || brushIdx == 3) {
        // Round / Dashed circular nib
        if (dx * dx + dy * dy <= radius * radius + radius) {
          paintSetPixel(lx + dx, ly + dy, colorId);
        }
      } else {
        // Square nib
        paintSetPixel(lx + dx, ly + dy, colorId);
      }
    }
  }
}

void paintDrawSegment(int16_t sx0, int16_t sy0, int16_t sx1, int16_t sy1) {
  if (sy0 < PAINT_CANVAS_Y || sy0 >= PAINT_CANVAS_Y + PAINT_CANVAS_H) return;
  if (sy1 < PAINT_CANVAS_Y || sy1 >= PAINT_CANVAS_Y + PAINT_CANVAS_H) return;

  int16_t x0 = constrain(sx0 / 2, 0, PAINT_BUF_W - 1);
  int16_t y0 = constrain((sy0 - PAINT_CANVAS_Y) / 2, 0, PAINT_BUF_H - 1);
  int16_t x1 = constrain(sx1 / 2, 0, PAINT_BUF_W - 1);
  int16_t y1 = constrain((sy1 - PAINT_CANVAS_Y) / 2, 0, PAINT_BUF_H - 1);

  uint8_t colorId = (uint8_t)((paintColorIdx % PAINT_COLOR_COUNT) + 1);
  uint8_t sz      = PAINT_SIZES[paintSizeIdx % PAINT_SIZE_COUNT];
  uint8_t br      = paintBrushIdx % PAINT_BRUSH_COUNT;

  int16_t dx = abs(x1 - x0);
  int16_t dy = abs(y1 - y0);
  int16_t sx = (x0 < x1) ? 1 : -1;
  int16_t sy = (y0 < y1) ? 1 : -1;
  int16_t err = dx - dy;

  for (int16_t guard = 0; guard < 512; guard++) {
    paintStampBrush(x0, y0, colorId, sz, br, paintStrokeStep);
    paintStrokeStep++;
    if (x0 == x1 && y0 == y1) break;
    int16_t e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }
}

void drawPaintScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  // Top-right Clear canvas button (x=198..236, y=4..26)
  canvas.fillRoundRect(198, 4, 38, 22, 5, COLOR_HEX_FILL);
  canvas.drawRoundRect(198, 4, 38, 22, 5, COLOR_RED);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_RED, COLOR_HEX_FILL);
  canvas.drawString("CLR", 217, 15);

  // Drawing canvas region (x=0..239, y=28..199 -> 120x86 logical pixels scaled 2x2)
  uint16_t bgCol = PAINT_BG_COLORS[paintBgIdx % PAINT_BG_COUNT];
  canvas.fillRect(0, PAINT_CANVAS_Y, 240, PAINT_CANVAS_H, bgCol);

  for (int16_t ly = 0; ly < PAINT_BUF_H; ly++) {
    int16_t sy = PAINT_CANVAS_Y + ly * 2;
    for (int16_t lx = 0; lx < PAINT_BUF_W; lx++) {
      uint8_t pix = paintGetPixel(lx, ly);
      if (pix > 0 && pix <= PAINT_COLOR_COUNT) {
        canvas.fillRect(lx * 2, sy, 2, 2, PAINT_COLORS[pix - 1]);
      }
    }
  }

  // Subtle divider above bottom toolbar
  canvas.drawFastHLine(0, 199, 240, COLOR_HEX_BORDER);
  canvas.fillRect(0, 200, 240, 40, COLOR_TOPBAR);

  // Bottom 5 buttons (y=202..238, w=44 each): Color, BG, Size, Brush, Save
  const int16_t btnY = 202;
  const int16_t btnH = 36;
  const int16_t btnW = 44;
  uint16_t fgCol = PAINT_COLORS[paintColorIdx % PAINT_COLOR_COUNT];
  uint8_t szVal  = PAINT_SIZES[paintSizeIdx % PAINT_SIZE_COUNT];
  uint8_t brIdx  = paintBrushIdx % PAINT_BRUSH_COUNT;

  // 0: Line Color button
  canvas.fillRoundRect(4, btnY, btnW, btnH, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(4, btnY, btnW, btnH, 6, COLOR_ACCENT);
  canvas.fillRoundRect(11, btnY + 5, 30, 14, 3, fgCol);
  canvas.drawRoundRect(11, btnY + 5, 30, 14, 3, COLOR_HEX_BORDER);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
  canvas.drawString("COL", 26, btnY + 28);

  // 1: Background Color button
  canvas.fillRoundRect(51, btnY, btnW, btnH, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(51, btnY, btnW, btnH, 6, COLOR_HEX_BORDER);
  canvas.fillRoundRect(58, btnY + 5, 30, 14, 3, bgCol);
  canvas.drawRoundRect(58, btnY + 5, 30, 14, 3, COLOR_ACCENT);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_HEX_FILL);
  canvas.drawString("BG", 73, btnY + 28);

  // 2: Line Thickness (Size) button
  canvas.fillRoundRect(98, btnY, btnW, btnH, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(98, btnY, btnW, btnH, 6, COLOR_HEX_BORDER);
  uint8_t dotR = (szVal == 1) ? 2 : (szVal == 2) ? 3 : (szVal == 4) ? 5 : 7;
  canvas.fillCircle(120, btnY + 12, dotR, COLOR_ACCENT);
  char szBuf[8];
  snprintf(szBuf, sizeof(szBuf), "%upx", (unsigned)(szVal * 2));
  canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
  canvas.drawString(szBuf, 120, btnY + 28);

  // 3: Line / Brush Type button (RND / SQR / SPR / DSH)
  canvas.fillRoundRect(145, btnY, btnW, btnH, 6, COLOR_HEX_FILL);
  canvas.drawRoundRect(145, btnY, btnW, btnH, 6, COLOR_HEX_BORDER);
  if (brIdx == 0) {
    canvas.fillCircle(167, btnY + 12, 5, COLOR_TEXT);
  } else if (brIdx == 1) {
    canvas.fillRect(162, btnY + 7, 10, 10, COLOR_TEXT);
  } else if (brIdx == 2) {
    canvas.fillCircle(163, btnY + 9, 1, COLOR_ACCENT);
    canvas.fillCircle(170, btnY + 8, 1, COLOR_ACCENT);
    canvas.fillCircle(167, btnY + 12, 1, COLOR_TEXT);
    canvas.fillCircle(162, btnY + 15, 1, COLOR_ACCENT);
    canvas.fillCircle(172, btnY + 15, 1, COLOR_TEXT);
  } else {
    canvas.fillRect(154, btnY + 11, 7, 3, COLOR_ACCENT);
    canvas.fillRect(164, btnY + 11, 7, 3, COLOR_ACCENT);
    canvas.fillRect(174, btnY + 11, 7, 3, COLOR_ACCENT);
  }
  canvas.setTextColor(COLOR_TEXT, COLOR_HEX_FILL);
  canvas.drawString(PAINT_BRUSH_NAMES[brIdx], 167, btnY + 28);

  // 4: Save to Flash button
  bool justSaved = (paintSaveBannerMs != 0 && (millis() - paintSaveBannerMs < 1600UL));
  canvas.fillRoundRect(192, btnY, btnW, btnH, 6, justSaved ? COLOR_GREEN : COLOR_HEX_ACTIVE);
  canvas.drawRoundRect(192, btnY, btnW, btnH, 6, COLOR_GREEN);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(justSaved ? COLOR_BG : COLOR_GREEN, justSaved ? COLOR_GREEN : COLOR_HEX_ACTIVE);
  canvas.drawString(justSaved ? "OK!" : "SAVE", 214, btnY + 18);

  // Brief SAVED TO FLASH badge overlay on top of canvas
  if (justSaved) {
    canvas.fillRoundRect(68, 34, 104, 20, 6, COLOR_TOPBAR);
    canvas.drawRoundRect(68, 34, 104, 20, 6, COLOR_GREEN);
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(COLOR_GREEN, COLOR_TOPBAR);
    canvas.drawString("SAVED TO FLASH", 120, 44);
  }
}

// ============================================================================
// 7L. ACCEL. SPECTROMETER (BMA423 200 HZ VIBRATION FFT + SPECTROID WATERFALL)
// ============================================================================

void startAccelSpectrometer() {
  memset(accelSpecRing, 0, sizeof(accelSpecRing));
  memset(accelSpecLevel, 0, sizeof(accelSpecLevel));
  memset(accelSpecPeak, 0, sizeof(accelSpecPeak));
  memset(accelSpecWaterfall, 0, sizeof(accelSpecWaterfall));
  accelSpecRingPos = 0;
  accelSpecSampleCount = 0;
  accelSpecWfHead = 0;
  accelSpecPeakFreqHz = 0.0f;
  accelSpecPeakG = 0.0f;
  accelSpecPeakDb = -80.0f;
  lastAccelSpecSampleUs = micros();
  lastAccelSpecDrawMs = millis();
  accelSpecActive = true;

  // Switch BMA423 ACC_CONF (0x40) to 200 Hz ODR (perf=1, avg4, ODR=0x09 -> 0xA9, Nyquist=100 Hz)
  if (bma423Ready) {
    writeRegI2C(Wire, bma423Addr, 0x40, 0xA9);
  }
}

void stopAccelSpectrometer() {
  accelSpecActive = false;
  // Restore BMA423 ACC_CONF (0x40) to default 50 Hz ODR (0xA7) for normal low-power operation
  if (bma423Ready) {
    writeRegI2C(Wire, bma423Addr, 0x40, 0xA7);
  }
}

// Poll one BMA423 sample at 200 Hz (5000 us period) into circular ring buffer
void sampleAccelSpectrometerOnce() {
  if (!accelSpecActive || !bma423Ready) return;
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastAccelSpecSampleUs) < ACC_SPEC_SAMPLE_PERIOD_US) return;
  lastAccelSpecSampleUs = nowUs;

  Wire.beginTransmission(bma423Addr);
  Wire.write(0x12);
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom(bma423Addr, (uint8_t)6) != 6) return;

  uint8_t xl = Wire.read();
  uint8_t xh = Wire.read();
  uint8_t yl = Wire.read();
  uint8_t yh = Wire.read();
  uint8_t zl = Wire.read();
  uint8_t zh = Wire.read();

  int16_t rawX = (int16_t)(((uint16_t)xh << 8) | xl) >> 4;
  int16_t rawY = (int16_t)(((uint16_t)yh << 8) | yl) >> 4;
  int16_t rawZ = (int16_t)(((uint16_t)zh << 8) | zl) >> 4;

  float gx = (float)rawX / 512.0f;
  float gy = (float)rawY / 512.0f;
  float gz = (float)rawZ / 512.0f;

  float sampleG = 0.0f;
  if (accelSpecAxis == 1) {
    sampleG = gx;
  } else if (accelSpecAxis == 2) {
    sampleG = gy;
  } else if (accelSpecAxis == 3) {
    sampleG = gz;
  } else {
    sampleG = sqrtf(gx * gx + gy * gy + gz * gz);
  }

  accelSpecRing[accelSpecRingPos] = sampleG;
  accelSpecRingPos = (uint16_t)((accelSpecRingPos + 1U) % ACC_SPEC_FFT_N);
  if (accelSpecSampleCount < ACC_SPEC_FFT_N) {
    accelSpecSampleCount++;
  }
}

// Compute Hann-windowed 128-pt Radix-2 FFT of BMA423 vibrations and push Spectroid waterfall row
void updateAccelSpectrometer() {
  if (!accelSpecActive || !bma423Ready) return;
  if (accelSpecSampleCount < 16) return;

  // Read chronological 128-sample window and compute DC mean (removes static 1g gravity)
  float mean = 0.0f;
  for (uint16_t i = 0; i < ACC_SPEC_FFT_N; i++) {
    uint16_t idx = (uint16_t)((accelSpecRingPos + i) % ACC_SPEC_FFT_N);
    accelSpecRe[i] = accelSpecRing[idx];
    mean += accelSpecRe[i];
  }
  mean /= (float)ACC_SPEC_FFT_N;

  // Apply Hann window
  for (uint16_t i = 0; i < ACC_SPEC_FFT_N; i++) {
    float win = 0.5f * (1.0f - cosf(6.283185307f * (float)i / (float)(ACC_SPEC_FFT_N - 1)));
    accelSpecRe[i] = (accelSpecRe[i] - mean) * win;
    accelSpecIm[i] = 0.0f;
  }

  computeRadix2Fft(accelSpecRe, accelSpecIm, ACC_SPEC_FFT_N);

  const uint16_t halfN = ACC_SPEC_FFT_N / 2;
  float binDb[ACC_SPEC_FFT_N / 2];
  float bestMag = 0.0f;
  uint16_t bestBin = 1;

  for (uint16_t b = 1; b < halfN; b++) {
    float mag = sqrtf(accelSpecRe[b] * accelSpecRe[b] + accelSpecIm[b] * accelSpecIm[b]) / ((float)halfN * 0.5f);
    if (mag > bestMag) {
      bestMag = mag;
      bestBin = b;
    }
    float db = 20.0f * log10f(mag + 1e-4f);
    if (db < -80.0f) db = -80.0f;
    if (db > 0.0f)   db = 0.0f;
    binDb[b] = db;
  }
  binDb[0] = -80.0f;

  // Parabolic interpolation for sub-bin vibration peak frequency
  float peakBinF = (float)bestBin;
  if (bestBin > 1 && bestBin + 1 < halfN) {
    float a = binDb[bestBin - 1];
    float b = binDb[bestBin];
    float c = binDb[bestBin + 1];
    float denom = (a - 2.0f * b + c);
    if (fabsf(denom) > 1e-4f) {
      float delta = 0.5f * (a - c) / denom;
      if (delta > -0.5f && delta < 0.5f) peakBinF += delta;
    }
  }

  const float binHz = (float)ACC_SPEC_SAMPLE_RATE / (float)ACC_SPEC_FFT_N;
  accelSpecPeakFreqHz = peakBinF * binHz;
  accelSpecPeakG      = bestMag;
  accelSpecPeakDb     = binDb[bestBin];

  // Logarithmic mapping of 1.56 Hz .. 100 Hz onto 112 Spectroid bands
  const float fMin = binHz;
  const float fMax = (float)ACC_SPEC_SAMPLE_RATE * 0.5f;
  const float logRatio = logf(fMax / fMin);

  accelSpecWfHead = (uint8_t)((accelSpecWfHead + ACC_SPEC_WATERFALL_H - 1) % ACC_SPEC_WATERFALL_H);

  for (uint16_t col = 0; col < ACC_SPEC_BANDS; col++) {
    float f0 = fMin * expf(logRatio * (float)col / (float)ACC_SPEC_BANDS);
    float f1 = fMin * expf(logRatio * (float)(col + 1) / (float)ACC_SPEC_BANDS);
    int16_t b0 = (int16_t)floorf(f0 / binHz);
    int16_t b1 = (int16_t)ceilf(f1 / binHz);
    if (b0 < 1) b0 = 1;
    if (b1 < b0) b1 = b0;
    if (b1 >= (int16_t)halfN) b1 = (int16_t)(halfN - 1);

    float maxDb = -80.0f;
    for (int16_t b = b0; b <= b1; b++) {
      if (binDb[b] > maxDb) maxDb = binDb[b];
    }

    // Normalize -70..0 dBg range to 0..100
    float norm = (maxDb + 70.0f) / 70.0f;
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;
    uint8_t targetLvl = (uint8_t)(norm * 100.0f + 0.5f);

    uint8_t prevLvl = accelSpecLevel[col];
    uint8_t smoothLvl = (targetLvl >= prevLvl)
                          ? (uint8_t)((prevLvl + targetLvl * 3U) / 4U)
                          : (uint8_t)((prevLvl * 3U + targetLvl) / 4U);
    accelSpecLevel[col] = smoothLvl;

    if (smoothLvl >= accelSpecPeak[col]) {
      accelSpecPeak[col] = smoothLvl;
    } else if (accelSpecPeak[col] > 1) {
      accelSpecPeak[col] = (uint8_t)(accelSpecPeak[col] - 1);
    } else {
      accelSpecPeak[col] = 0;
    }

    uint8_t lutIdx = (uint8_t)(((uint16_t)smoothLvl * 31U) / 100U);
    if (lutIdx > 31) lutIdx = 31;
    accelSpecWaterfall[accelSpecWfHead][col] = lutIdx;
  }
}

// Draw Accel. spectrometer screen (identical Spectroid style to Sound spectrometer, 1.56..100 Hz)
void drawAccelSpectrometerScreen() {
  canvas.fillScreen(COLOR_BG);
  drawTopStatusBar();
  drawTopLeftCloseIcon();

  // Top-right axis selector badge (MAG / X / Y / Z, x=198..236, y=4..26)
  canvas.fillRoundRect(198, 4, 38, 22, 5, COLOR_HEX_FILL);
  canvas.drawRoundRect(198, 4, 38, 22, 5, COLOR_ACCENT);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_ACCENT, COLOR_HEX_FILL);
  canvas.drawString(ACC_SPEC_AXIS_LABELS[accelSpecAxis % 4], 217, 15);

  // Header row: title + live peak vibration Hz / g
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(top_left);
  canvas.setTextColor(COLOR_ACCENT, COLOR_BG);
  canvas.drawString("Accel. Spec", 8, 31);

  char peakBuf[28];
  uint16_t peakCol = COLOR_WARN;
  if (!bma423Ready) {
    snprintf(peakBuf, sizeof(peakBuf), "BMA423 ERR");
    peakCol = COLOR_RED;
  } else if (accelSpecPeakG >= 0.005f) {
    snprintf(peakBuf, sizeof(peakBuf), "%.1fHz %.2fg", accelSpecPeakFreqHz, accelSpecPeakG);
  } else {
    snprintf(peakBuf, sizeof(peakBuf), "BMA423 200Hz");
  }
  canvas.setTextDatum(top_right);
  canvas.setTextColor(peakCol, COLOR_BG);
  canvas.drawString(peakBuf, 232, 31);

  // Upper spectrum plot area (x = 8..232, y = 43..111, w = 224 = 112 bands * 2 px, h = 68)
  const int16_t plotX = 8;
  const int16_t plotY = 43;
  const int16_t plotW = ACC_SPEC_BANDS * 2;
  const int16_t plotH = 68;
  canvas.fillRect(plotX, plotY, plotW, plotH, 0x0002);
  canvas.drawRect(plotX - 1, plotY - 1, plotW + 2, plotH + 2, COLOR_HEX_BORDER);

  // Horizontal dBg grid lines
  for (uint8_t g = 1; g <= 3; g++) {
    int16_t gy = (int16_t)(plotY + (plotH * g) / 4);
    canvas.drawFastHLine(plotX, gy, plotW, 0x18E4);
  }

  // Vertical grid lines and labels for log vibration frequency axis (2, 5, 15, 40, 100 Hz)
  const float fMin = (float)ACC_SPEC_SAMPLE_RATE / (float)ACC_SPEC_FFT_N;
  const float fMax = (float)ACC_SPEC_SAMPLE_RATE * 0.5f;
  const float logRatio = logf(fMax / fMin);
  const float markFreqs[5] = { 2.0f, 5.0f, 15.0f, 40.0f, 100.0f };
  const char* markLabels[5] = { "2Hz", "5", "15", "40", "100" };

  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(COLOR_SUBTEXT, COLOR_BG);
  for (uint8_t m = 0; m < 5; m++) {
    float frac = logf(markFreqs[m] / fMin) / logRatio;
    int16_t mx = (int16_t)(plotX + (int16_t)(frac * (float)(plotW - 1)));
    canvas.drawFastVLine(mx, plotY, plotH, 0x18E4);
    canvas.setTextDatum(m == 0 ? top_left : (m == 4 ? top_right : top_center));
    canvas.drawString(markLabels[m], mx, 114);
  }

  // Draw spectral bars and Peak Hold polyline
  int16_t prevPeakY = (int16_t)(plotY + plotH - 1);
  for (uint16_t col = 0; col < ACC_SPEC_BANDS; col++) {
    int16_t bx = (int16_t)(plotX + col * 2);
    int16_t barH = (int16_t)(((uint16_t)accelSpecLevel[col] * (uint16_t)(plotH - 2)) / 100U);
    if (barH > 0) {
      uint8_t lutIdx = (uint8_t)(((uint16_t)accelSpecLevel[col] * 31U) / 100U);
      if (lutIdx > 31) lutIdx = 31;
      int16_t barY = (int16_t)(plotY + plotH - barH);
      canvas.fillRect(bx, barY, 2, barH, SPEC_COLOR_LUT[lutIdx]);
      canvas.drawFastHLine(bx, barY, 2, COLOR_ACCENT);
    }

    int16_t peakH = (int16_t)(((uint16_t)accelSpecPeak[col] * (uint16_t)(plotH - 2)) / 100U);
    int16_t peakY = (int16_t)(plotY + plotH - 1 - peakH);
    if (col > 0) {
      canvas.drawLine(bx - 2, prevPeakY, bx, peakY, COLOR_WARN);
    }
    prevPeakY = peakY;
  }

  // Lower scrolling Spectrogram Waterfall (x = 8..232, y = 125..235, 55 rows * 2 px)
  const int16_t wfY = 125;
  const int16_t wfH = ACC_SPEC_WATERFALL_H * 2;
  canvas.drawRect(plotX - 1, wfY - 1, plotW + 2, wfH + 2, COLOR_HEX_BORDER);
  for (uint8_t row = 0; row < ACC_SPEC_WATERFALL_H; row++) {
    uint8_t ringIdx = (uint8_t)((accelSpecWfHead + row) % ACC_SPEC_WATERFALL_H);
    int16_t ry = (int16_t)(wfY + row * 2);
    for (uint16_t col = 0; col < ACC_SPEC_BANDS; col++) {
      uint8_t cIdx = accelSpecWaterfall[ringIdx][col];
      if (cIdx > 31) cIdx = 31;
      canvas.fillRect(plotX + col * 2, ry, 2, 2, SPEC_COLOR_LUT[cIdx]);
    }
  }
}

// Master screen render dispatch from offscreen buffer
void renderCurrentScreen() {
  switch (currentScreen) {
    case SCREEN_MAIN_MENU:
      drawMainMenuScreen();
      break;
    case SCREEN_CLOCK:
      drawClockScreen();
      break;
    case SCREEN_GPS:
      drawGpsScreen();
      break;
    case SCREEN_CALC:
      drawCalcScreen();
      break;
    case SCREEN_EARTH:
      drawEarthScreen();
      break;
    case SCREEN_STEPS:
      drawStepsScreen();
      break;
    case SCREEN_IR:
      drawIrScreen();
      break;
    case SCREEN_CALENDAR:
      drawCalendarScreen();
      break;
    case SCREEN_INCLINOMETER:
      drawInclinometerScreen();
      break;
    case SCREEN_WIFI_CONNECT:
      drawWifiConnectScreen();
      break;
    case SCREEN_WIFI_LIST:
      drawWifiListScreen();
      break;
    case SCREEN_SETTINGS:
      drawSettingsScreen();
      break;
    case SCREEN_ASCII:
      drawAsciiTableScreen();
      break;
    case SCREEN_TEMP:
      drawTempSensorScreen();
      break;
    case SCREEN_PERIODIC:
      drawPeriodicTableScreen();
      break;
    case SCREEN_E_ADDITIVES:
      drawEAdditivesScreen();
      break;
    case SCREEN_WIFI_ANALYZER:
      drawWifiAnalyzerScreen();
      break;
    case SCREEN_ABOUT_FW:
      drawAboutFirmwareScreen();
      break;
    case SCREEN_WIFI_INFO:
      drawWifiInfoScreen();
      break;
    case SCREEN_WIFI_JAM:
      drawWifiJamDetectScreen();
      break;
    case SCREEN_ACCEL:
      drawAccelSensorScreen();
      break;
    case SCREEN_SOUND_SPEC:
      drawSoundSpectrometerScreen();
      break;
    case SCREEN_SOUND_GEN:
      drawSoundGeneratorScreen();
      break;
    case SCREEN_HOSTS_SCAN:
      drawHostsScanScreen();
      break;
    case SCREEN_BT_SCAN:
      drawBluetoothScanScreen();
      break;
    case SCREEN_SUN_ACTIVITY:
      drawSunActivityScreen();
      break;
    case SCREEN_WEATHER:
      drawWeatherScreen();
      break;
    case SCREEN_SYSTEM_INFO:
      drawSystemInfoScreen();
      break;
    case SCREEN_ALARM:
      drawAlarmScreen();
      break;
    case SCREEN_STOPWATCH:
      drawStopwatchScreen();
      break;
    case SCREEN_TIMER:
      drawTimerScreen();
      break;
    case SCREEN_FLASHLIGHT:
      drawFlashlightScreen();
      break;
    case SCREEN_SOS:
      drawSosScreen();
      break;
    case SCREEN_MORSE:
      drawMorseCodeScreen();
      break;
    case SCREEN_TETRIS:
      drawTetrisScreen();
      break;
    case SCREEN_PAINT:
      drawPaintScreen();
      break;
    case SCREEN_ACCEL_SPEC:
      drawAccelSpectrometerScreen();
      break;
    case SCREEN_BT_KEYBOARD:
      drawBluetoothKeyboardScreen();
      break;
    case SCREEN_BT_MOUSE:
      drawBluetoothMouseScreen();
      break;
    case SCREEN_BT_PRESENTER:
      drawBluetoothPresenterScreen();
      break;
    case SCREEN_COLOR_TABLE:
      drawColorTableScreen();
      break;
    case SCREEN_RESISTOR_TABLE:
      drawResistorTableScreen();
      break;
    case SCREEN_VOLTAGE_DIVIDER:
      drawVoltageDividerScreen();
      break;
  }
  if (activeAlertType != ALERT_NONE) {
    drawAlertModalOverlay();
  }
  canvas.pushSprite(0, 0);
}

// ============================================================================
// 8. CALCULATOR, IR, CLOCK EDITING, AND TOUCH DISPATCH LOGIC
// ============================================================================

// Helper to format a real number (up to 12 decimal digits) into buffer
void formatRealPart(double val, char* outBuf, uint8_t bufSize, uint8_t decimals) {
  if (fabs(val) < 1e-13) {
    snprintf(outBuf, bufSize, "0");
    return;
  }

  if (fabs(val) >= 1e12 || fabs(val) < 1e-9) {
    snprintf(outBuf, bufSize, "%.12g", val);
    return;
  }

  char fmt[8];
  snprintf(fmt, sizeof(fmt), "%%.%uf", (uint16_t)decimals);
  snprintf(outBuf, bufSize, fmt, val);

  char* dotPos = strchr(outBuf, '.');
  if (dotPos != NULL) {
    int16_t endIdx = (int16_t)strlen(outBuf) - 1;
    while (endIdx > 0 && outBuf[endIdx] == '0') {
      outBuf[endIdx] = '\0';
      endIdx--;
    }
    if (endIdx > 0 && outBuf[endIdx] == '.') {
      outBuf[endIdx] = '\0';
    }
  }
}

// Format double value into calculator string (up to 12 decimals + inf / -inf)
void formatCalcNumber(double val) {
  if (isnan(val)) {
    snprintf(calcDisplayStr, sizeof(calcDisplayStr), "Error");
    calcInfSign = 0;
    calcError = true;
    calcNewInput = true;
    return;
  }

  if (isinf(val)) {
    calcError = false;
    calcInfSign = (val > 0.0) ? 1 : -1;
    snprintf(calcDisplayStr, sizeof(calcDisplayStr), (val > 0.0) ? "INF" : "-INF");
    calcNewInput = true;
    return;
  }

  calcInfSign = 0;
  calcError = false;
  formatRealPart(val, calcDisplayStr, (uint8_t)sizeof(calcDisplayStr), 12);
}

// Format complex number re + im*i into calculator string
void formatCalcComplex(double re, double im) {
  calcInfSign = 0;
  calcError = false;

  if (isnan(re) || isnan(im) || isinf(re) || isinf(im)) {
    formatCalcNumber(NAN);
    return;
  }

  if (fabs(im) < 1e-12) {
    formatCalcNumber(re);
    return;
  }

  char reBuf[24];
  char imBuf[24];

  if (fabs(re) < 1e-12) {
    formatRealPart(fabs(im), imBuf, (uint8_t)sizeof(imBuf), 12);
    if (im < 0.0) {
      snprintf(calcDisplayStr, sizeof(calcDisplayStr), "-%si", imBuf);
    } else {
      snprintf(calcDisplayStr, sizeof(calcDisplayStr), "%si", imBuf);
    }
  } else {
    formatRealPart(re, reBuf, (uint8_t)sizeof(reBuf), 10);
    formatRealPart(fabs(im), imBuf, (uint8_t)sizeof(imBuf), 10);
    snprintf(calcDisplayStr, sizeof(calcDisplayStr), "%s%c%si",
             reBuf, (im >= 0.0) ? '+' : '-', imBuf);
  }
}

// Execute pending calculator binary operation (+, -, *, /, x^y, x^1/y)
double evaluateBinaryOp(double a, double b, char op) {
  switch (op) {
    case '+': return a + b;
    case '-': return a - b;
    case '*': return a * b;
    case '/':
      if (fabs(b) < 1e-15) {
        if (a > 0.0) return INFINITY;
        if (a < 0.0) return -INFINITY;
        calcError = true;
        return NAN;
      }
      return a / b;
    case '^':
      return pow(a, b);
    case 'r':
      if (fabs(b) < 1e-15) {
        calcError = true;
        return NAN;
      }
      if (a < 0.0) {
        double rB = round(b);
        if (fabs(b - rB) < 1e-9 && ((int32_t)rB % 2 != 0)) {
          return -pow(-a, 1.0 / b);
        }
        return NAN;
      }
      return pow(a, 1.0 / b);
  }
  return b;
}

// Handle button press on calculator page 1
void handleCalcPage1Button(const char* label) {
  if (strcmp(label, "C") == 0) {
    snprintf(calcDisplayStr, sizeof(calcDisplayStr), "0");
    calcStoredVal = 0.0;
    calcPendingOp = 0;
    calcNewInput  = true;
    calcError     = false;
    calcInfSign   = 0;
    return;
  }

  if (calcError || calcInfSign != 0 || strchr(calcDisplayStr, 'i') != NULL) {
    snprintf(calcDisplayStr, sizeof(calcDisplayStr), "0");
    calcError = false;
    calcInfSign = 0;
    calcNewInput = true;
  }

  if (label[0] >= '0' && label[0] <= '9' && label[1] == '\0') {
    if (calcNewInput || strcmp(calcDisplayStr, "0") == 0) {
      calcDisplayStr[0] = label[0];
      calcDisplayStr[1] = '\0';
      calcNewInput = false;
    } else if (strlen(calcDisplayStr) < 18) {
      uint8_t len = (uint8_t)strlen(calcDisplayStr);
      calcDisplayStr[len] = label[0];
      calcDisplayStr[len + 1] = '\0';
    }
    return;
  }

  if (strcmp(label, ".") == 0) {
    if (calcNewInput) {
      snprintf(calcDisplayStr, sizeof(calcDisplayStr), "0.");
      calcNewInput = false;
    } else if (strchr(calcDisplayStr, '.') == NULL && strlen(calcDisplayStr) < 17) {
      strcat(calcDisplayStr, ".");
    }
    return;
  }

  double currentVal = atof(calcDisplayStr);

  if (strcmp(label, "+/-") == 0) {
    if (fabs(currentVal) > 0.0) {
      formatCalcNumber(-currentVal);
    }
    return;
  }

  if (strcmp(label, "%") == 0) {
    if (calcPendingOp == '+' || calcPendingOp == '-') {
      currentVal = calcStoredVal * (currentVal / 100.0);
    } else {
      currentVal = currentVal / 100.0;
    }
    formatCalcNumber(currentVal);
    calcNewInput = true;
    return;
  }

  if (strcmp(label, "=") == 0) {
    if (calcPendingOp != 0) {
      if (calcPendingOp == 'r' && calcStoredVal < 0.0) {
        if (fabs(currentVal) < 1e-15) {
          formatCalcNumber(NAN);
        } else {
          double rB = round(currentVal);
          if (fabs(currentVal - rB) < 1e-9 && ((int32_t)rB % 2 != 0)) {
            formatCalcNumber(-pow(-calcStoredVal, 1.0 / currentVal));
          } else {
            double mag = pow(-calcStoredVal, 1.0 / currentVal);
            double angle = M_PI / currentVal;
            formatCalcComplex(mag * cos(angle), mag * sin(angle));
          }
        }
      } else {
        double res = evaluateBinaryOp(calcStoredVal, currentVal, calcPendingOp);
        formatCalcNumber(res);
      }
      calcPendingOp = 0;
      calcNewInput = true;
    }
    return;
  }

  if (label[0] == '+' || label[0] == '-' || label[0] == '*' || label[0] == '/') {
    if (calcPendingOp != 0 && !calcNewInput) {
      calcStoredVal = evaluateBinaryOp(calcStoredVal, currentVal, calcPendingOp);
      formatCalcNumber(calcStoredVal);
    } else {
      calcStoredVal = currentVal;
    }
    calcPendingOp = label[0];
    calcNewInput = true;
  }
}

// Handle function button press on calculator page 2
void handleCalcPage2Button(int8_t funcIndex) {
  if (calcError && funcIndex != 12 && funcIndex != 13 && funcIndex != 15) {
    return;
  }
  if (calcInfSign != 0 && funcIndex != 12 && funcIndex != 13 && funcIndex != 14 && funcIndex != 15) {
    return;
  }
  if (funcIndex == 14 && calcInfSign != 0) {
    calcInfSign = 1;
    calcNewInput = true;
    return;
  }

  double x = atof(calcDisplayStr);

  if (funcIndex == 10 || funcIndex == 11) {
    char op = (funcIndex == 10) ? '^' : 'r';
    if (calcPendingOp != 0 && !calcNewInput) {
      calcStoredVal = evaluateBinaryOp(calcStoredVal, x, calcPendingOp);
      formatCalcNumber(calcStoredVal);
    } else {
      calcStoredVal = x;
    }
    calcPendingOp = op;
    calcNewInput = true;
    calcPage = 0;
    calcSecondPage = false;
    return;
  }

  if (funcIndex == 0) {
    if (x < 0.0 || floor(x) != x || x > 10000000.0) {
      formatCalcNumber(NAN);
    } else if (x <= 170.0) {
      double factRes = 1.0;
      int32_t n = (int32_t)x;
      for (int32_t i = 2; i <= n; i++) {
        factRes *= (double)i;
      }
      formatCalcNumber(factRes);
    } else {
      double log10Val = lgamma(x + 1.0) / log(10.0);
      double expPart  = floor(log10Val);
      double mantissa = pow(10.0, log10Val - expPart);
      calcInfSign = 0;
      calcError = false;
      snprintf(calcDisplayStr, sizeof(calcDisplayStr), "%.12fe+%" PRId64,
               mantissa, (int64_t)expPart);
    }
    calcNewInput = true;
    return;
  }

  if (funcIndex == 1) {
    if (x < 0.0) {
      formatCalcComplex(0.0, sqrt(-x));
    } else {
      formatCalcNumber(sqrt(x));
    }
    calcNewInput = true;
    return;
  }

  double res = 0.0;
  switch (funcIndex) {
    case 2: res = (x <= 0.0) ? NAN : log(x); break;
    case 3: res = (x <= 0.0) ? NAN : log10(x); break;
    case 4: res = exp(x); break;
    case 5: res = pow(2.0, x); break;
    case 6: res = pow(10.0, x); break;
    case 7: res = (fabs(x) < 1e-15) ? INFINITY : (1.0 / x); break;
    case 8: res = x * x; break;
    case 9: res = x * x * x; break;
    case 12:
      res = M_PI;
      calcError = false;
      calcInfSign = 0;
      break;
    case 13:
      res = M_E;
      calcError = false;
      calcInfSign = 0;
      break;
    case 14:
      res = fabs(x);
      break;
    case 15:
      res = (double)esp_random() / 4294967296.0;
      calcError = false;
      calcInfSign = 0;
      break;
  }

  formatCalcNumber(res);
  calcNewInput = true;
}

// Handle function button press on calculator page 3
void handleCalcPage3Button(int8_t funcIndex) {
  if (calcError || calcInfSign != 0) {
    return;
  }

  double x = atof(calcDisplayStr);
  double res = 0.0;

  switch (funcIndex) {
    case 0: res = sin(x); break;
    case 1: res = cos(x); break;
    case 2: {
      double c = cos(x);
      res = (fabs(c) < 1e-15) ? ((sin(x) >= 0.0) ? INFINITY : -INFINITY) : tan(x);
      break;
    }
    case 3: {
      double s = sin(x);
      res = (fabs(s) < 1e-15) ? ((cos(x) >= 0.0) ? INFINITY : -INFINITY) : (cos(x) / s);
      break;
    }
    case 4: res = (x < -1.0 || x > 1.0) ? NAN : asin(x); break;
    case 5: res = (x < -1.0 || x > 1.0) ? NAN : acos(x); break;
    case 6: res = atan(x); break;
    case 7: res = atan2(1.0, x); break;
    case 8: res = sinh(x); break;
    case 9: res = cosh(x); break;
    case 10: res = tanh(x); break;
    case 11: {
      double sh = sinh(x);
      res = (fabs(sh) < 1e-15) ? INFINITY : (cosh(x) / sh);
      break;
    }
    case 12: res = 1.0 / cosh(x); break;
    case 13: {
      double sh = sinh(x);
      res = (fabs(sh) < 1e-15) ? INFINITY : (1.0 / sh);
      break;
    }
  }

  formatCalcNumber(res);
  calcNewInput = true;
}

// Handle function button press on calculator page 4 (inverse hyperbolic functions)
void handleCalcPage4Button(int8_t funcIndex) {
  if (calcError || calcInfSign != 0) {
    return;
  }

  double x = atof(calcDisplayStr);
  double res = 0.0;

  switch (funcIndex) {
    case 0: // asinh(x)
      res = asinh(x);
      break;
    case 1: // acosh(x), x >= 1
      res = (x < 1.0) ? NAN : acosh(x);
      break;
    case 2: // atanh(x), |x| < 1
      if (fabs(x - 1.0) < 1e-15) res = INFINITY;
      else if (fabs(x + 1.0) < 1e-15) res = -INFINITY;
      else if (fabs(x) > 1.0) res = NAN;
      else res = atanh(x);
      break;
    case 3: // acoth(x) = 0.5 * ln((x + 1)/(x - 1)), |x| > 1
      if (fabs(x - 1.0) < 1e-15) res = INFINITY;
      else if (fabs(x + 1.0) < 1e-15) res = -INFINITY;
      else if (fabs(x) < 1.0) res = NAN;
      else res = 0.5 * log((x + 1.0) / (x - 1.0));
      break;
    case 4: // asech(x) = acosh(1 / x), 0 < x <= 1
      if (fabs(x) < 1e-15) res = INFINITY;
      else if (x < 0.0 || x > 1.0) res = NAN;
      else res = acosh(1.0 / x);
      break;
    case 5: // acsch(x) = asinh(1 / x), x != 0
      if (fabs(x) < 1e-15) res = INFINITY;
      else res = asinh(1.0 / x);
      break;
  }

  formatCalcNumber(res);
  calcNewInput = true;
}

// Convert ASCII digit/hex char to numeric value (0..15) or -1 if invalid
int8_t calcBaseCharToVal(char ch) {
  if (ch >= '0' && ch <= '9') return (int8_t)(ch - '0');
  if (ch >= 'A' && ch <= 'F') return (int8_t)(10 + (ch - 'A'));
  if (ch >= 'a' && ch <= 'f') return (int8_t)(10 + (ch - 'a'));
  return -1;
}

// Parse signed integer or fractional number string in arbitrary base (2, 8, 10, 16)
double parseCalcBaseNumber(const char* str, uint8_t base) {
  if (str == NULL || str[0] == '\0' || base < 2 || base > 16) return 0.0;
  bool neg = false;
  size_t idx = 0;
  if (str[idx] == '-') {
    neg = true;
    idx++;
  } else if (str[idx] == '+') {
    idx++;
  }

  double intVal  = 0.0;
  double fracVal = 0.0;
  double weight  = 1.0 / (double)base;
  bool inFrac    = false;

  for (; str[idx] != '\0'; idx++) {
    char c = str[idx];
    if (c == '.') {
      if (inFrac) break;
      inFrac = true;
      continue;
    }
    int8_t d = calcBaseCharToVal(c);
    if (d < 0 || d >= (int8_t)base) continue;
    if (!inFrac) {
      intVal = intVal * (double)base + (double)d;
    } else {
      fracVal += (double)d * weight;
      weight  /= (double)base;
    }
  }

  double total = intVal + fracVal;
  return (neg && total > 0.0) ? -total : total;
}

// Format signed integer or fractional double value in target base (2, 8, 10, 16)
void formatCalcBaseNumber(double val, uint8_t base, char* outBuf, size_t outSize) {
  static const char DIGITS[] = "0123456789ABCDEF";
  if (outBuf == NULL || outSize < 4) return;
  if (!isfinite(val) || fabs(val) >= 4503599627370496.0) {
    calcBaseError = true;
    snprintf(outBuf, outSize, "Overflow");
    return;
  }

  calcBaseError = false;
  bool neg = (val < 0.0);
  double absVal = fabs(val);
  uint8_t maxFrac = (base == 2) ? 12 : 8;

  uint64_t intPart = (uint64_t)floor(absVal);
  double frac = absVal - (double)intPart;

  uint8_t fracDigits[16];
  for (uint8_t i = 0; i < maxFrac; i++) {
    frac *= (double)base;
    int8_t d = (int8_t)floor(frac);
    if (d < 0) d = 0;
    if (d >= (int8_t)base) d = (int8_t)(base - 1);
    fracDigits[i] = (uint8_t)d;
    frac -= (double)d;
  }

  // Round to nearest at maxFrac-th digit and propagate carry into integer part if needed
  if (frac >= 0.5 - 1e-9) {
    bool carry = true;
    for (int8_t i = (int8_t)maxFrac - 1; i >= 0 && carry; i--) {
      fracDigits[i]++;
      if (fracDigits[i] >= base) {
        fracDigits[i] = 0;
      } else {
        carry = false;
      }
    }
    if (carry) {
      intPart++;
    }
  }

  // Trim trailing fractional zeros
  int8_t fracLen = (int8_t)maxFrac;
  while (fracLen > 0 && fracDigits[fracLen - 1] == 0) {
    fracLen--;
  }

  // Convert integer part to reversed digits
  char intRev[64];
  uint8_t intLen = 0;
  if (intPart == 0) {
    intRev[intLen++] = '0';
  } else {
    uint64_t tmp = intPart;
    while (tmp > 0 && intLen < 60) {
      intRev[intLen++] = DIGITS[tmp % base];
      tmp /= base;
    }
  }

  // Avoid "-0"
  if (intPart == 0 && fracLen == 0) {
    neg = false;
  }

  size_t pos = 0;
  if (neg && pos + 1 < outSize) {
    outBuf[pos++] = '-';
  }
  for (int8_t i = (int8_t)intLen - 1; i >= 0 && pos + 1 < outSize; i--) {
    outBuf[pos++] = intRev[i];
  }
  if (fracLen > 0 && pos + 2 < outSize) {
    outBuf[pos++] = '.';
    for (int8_t i = 0; i < fracLen && pos + 1 < outSize; i++) {
      outBuf[pos++] = DIGITS[fracDigits[i]];
    }
  }
  outBuf[pos] = '\0';
}

// Sync current decimal Calc value into Page 5 base converter
void syncCalcToBaseConverter() {
  if (calcError || calcInfSign != 0 || strchr(calcDisplayStr, 'i') != NULL) {
    calcBaseExactVal = 0.0;
    calcBaseError    = false;
    snprintf(calcBaseStr, sizeof(calcBaseStr), "0");
    return;
  }
  calcBaseExactVal = atof(calcDisplayStr);
  formatCalcBaseNumber(calcBaseExactVal, calcBaseMode, calcBaseStr, sizeof(calcBaseStr));
}

// Sync Page 5 base converter value back to decimal Calc display
void syncBaseConverterToCalc() {
  if (calcBaseError) {
    formatCalcNumber(NAN);
    return;
  }
  formatCalcNumber(calcBaseExactVal);
  calcNewInput = true;
}

// Switch active number system on calculator page 5 (BIN / OCT / DEC / HEX)
void handleCalcPage5BaseSelect(uint8_t newBase) {
  if (newBase != 2 && newBase != 8 && newBase != 10 && newBase != 16) return;
  if (newBase == calcBaseMode) return;
  calcBaseMode = newBase;
  if (calcBaseError) {
    calcBaseExactVal = 0.0;
    calcBaseError    = false;
    snprintf(calcBaseStr, sizeof(calcBaseStr), "0");
    return;
  }
  formatCalcBaseNumber(calcBaseExactVal, calcBaseMode, calcBaseStr, sizeof(calcBaseStr));
}

// Handle keypad press on calculator page 5 (0-9, A-F, ., +/-, DEL, CLR)
void handleCalcPage5Key(const char* key) {
  if (!isCalcBaseKeyAllowed(key, calcBaseMode)) return;

  if (strcmp(key, "CLR") == 0) {
    snprintf(calcBaseStr, sizeof(calcBaseStr), "0");
    calcBaseExactVal = 0.0;
    calcBaseError    = false;
    return;
  }

  if (calcBaseError) {
    snprintf(calcBaseStr, sizeof(calcBaseStr), "0");
    calcBaseExactVal = 0.0;
    calcBaseError    = false;
  }

  if (strcmp(key, "DEL") == 0) {
    size_t len = strlen(calcBaseStr);
    if (len <= 1 || (len == 2 && calcBaseStr[0] == '-')) {
      snprintf(calcBaseStr, sizeof(calcBaseStr), "0");
    } else {
      calcBaseStr[len - 1] = '\0';
      if (strcmp(calcBaseStr, "-") == 0) {
        snprintf(calcBaseStr, sizeof(calcBaseStr), "0");
      }
    }
    calcBaseExactVal = parseCalcBaseNumber(calcBaseStr, calcBaseMode);
    return;
  }

  if (strcmp(key, "+/-") == 0) {
    if (strcmp(calcBaseStr, "0") == 0 || strcmp(calcBaseStr, "0.") == 0) return;
    if (calcBaseStr[0] == '-') {
      memmove(calcBaseStr, calcBaseStr + 1, strlen(calcBaseStr));
    } else if (strlen(calcBaseStr) + 1 < sizeof(calcBaseStr)) {
      memmove(calcBaseStr + 1, calcBaseStr, strlen(calcBaseStr) + 1);
      calcBaseStr[0] = '-';
    }
    calcBaseExactVal = parseCalcBaseNumber(calcBaseStr, calcBaseMode);
    return;
  }

  if (strcmp(key, ".") == 0) {
    if (strchr(calcBaseStr, '.') == NULL && strlen(calcBaseStr) < 26) {
      strcat(calcBaseStr, ".");
    }
    calcBaseExactVal = parseCalcBaseNumber(calcBaseStr, calcBaseMode);
    return;
  }

  // Valid digit or hex letter (0-9, A-F)
  char ch = key[0];
  uint8_t maxLen = (calcBaseMode == 2) ? 26 : 18;
  if (strcmp(calcBaseStr, "0") == 0) {
    calcBaseStr[0] = ch;
    calcBaseStr[1] = '\0';
  } else if (strcmp(calcBaseStr, "-0") == 0) {
    calcBaseStr[0] = '-';
    calcBaseStr[1] = ch;
    calcBaseStr[2] = '\0';
  } else if (strlen(calcBaseStr) < maxLen) {
    size_t len = strlen(calcBaseStr);
    calcBaseStr[len]     = ch;
    calcBaseStr[len + 1] = '\0';
  }
  calcBaseExactVal = parseCalcBaseNumber(calcBaseStr, calcBaseMode);
}

// Handle button press on IR screen page 1
void handleIrButton(const char* label) {
  if (strcmp(label, "CLR") == 0) {
    snprintf(irHexStr, sizeof(irHexStr), "0");
    irSentNoticeMs = 0;
    irTestNoticeMs = 0;
    return;
  }

  if (strcmp(label, "Test LED") == 0) {
    irSentNoticeMs = 0;
    irTestNoticeMs = millis();
    renderCurrentScreen();
    testIrLedBlink();
    irTestNoticeMs = millis();
    return;
  }

  if (strcmp(label, "Send") == 0) {
    irTestNoticeMs = 0;
    uint32_t code = (uint32_t)strtoul(irHexStr, NULL, 16);
    if (irProtocol == IR_PROTO_NEC) {
      sendIrNecCode(code);
    } else if (irProtocol == IR_PROTO_RC5) {
      sendIrRc5Code(code);
    } else {
      sendIrRc6Code(code);
    }
    irSentNoticeMs = millis();
    return;
  }

  char ch = label[0];
  if ((ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'F')) {
    irSentNoticeMs = 0;
    irTestNoticeMs = 0;
    if (strcmp(irHexStr, "0") == 0) {
      irHexStr[0] = ch;
      irHexStr[1] = '\0';
    } else if (strlen(irHexStr) < 8) {
      uint8_t len = (uint8_t)strlen(irHexStr);
      irHexStr[len] = ch;
      irHexStr[len + 1] = '\0';
    }
  }
}

// Number of days in a month accounting for leap years
uint8_t getDaysInMonth(uint8_t month, uint16_t year) {
  const uint8_t days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (month < 1 || month > 12) return 31;
  if (month == 2) {
    bool leap = ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
    return leap ? 29 : 28;
  }
  return days[month - 1];
}

// Compute weekday from date (0 = Sun, 1 = Mon .. 6 = Sat) and apply UTC offset
void applyUtcOffsetToDateTime(uint16_t year, uint8_t month, uint8_t day,
                              uint8_t hour, uint8_t minute, uint8_t second,
                              int8_t offsetHours, DateTimeRTC &out) {
  int16_t h = (int16_t)hour + (int16_t)offsetHours;
  int16_t d = (int16_t)day;
  int16_t m = (int16_t)month;
  int16_t y = (int16_t)year;

  while (h >= 24) {
    h -= 24;
    d++;
    uint8_t dim = getDaysInMonth((uint8_t)m, (uint16_t)y);
    if (d > dim) {
      d = 1;
      m++;
      if (m > 12) {
        m = 1;
        y++;
      }
    }
  }
  while (h < 0) {
    h += 24;
    d--;
    if (d < 1) {
      m--;
      if (m < 1) {
        m = 12;
        y--;
      }
      d = getDaysInMonth((uint8_t)m, (uint16_t)y);
    }
  }

  static const uint8_t sakamoto[12] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  int16_t sy = (m < 3) ? (y - 1) : y;
  uint8_t dow = (uint8_t)((sy + sy / 4 - sy / 100 + sy / 400 + sakamoto[m - 1] + d) % 7);

  out.year      = (uint16_t)y;
  out.month     = (uint8_t)m;
  out.day       = (uint8_t)d;
  out.hour      = (uint8_t)h;
  out.minute    = minute;
  out.second    = second;
  out.dayOfWeek = dow;
}

// Sync RTC clock from GPS (when valid GPS fix is available)
bool syncClockFromGPS() {
  if (!(gps.location.isValid() && gps.date.isValid() && gps.time.isValid() && gps.date.year() >= 2020)) {
    return false;
  }
  applyUtcOffsetToDateTime((uint16_t)gps.date.year(),
                           (uint8_t)gps.date.month(),
                           (uint8_t)gps.date.day(),
                           (uint8_t)gps.time.hour(),
                           (uint8_t)gps.time.minute(),
                           (uint8_t)gps.time.second(),
                           utcOffset,
                           editTime);
  writeRTC(editTime);
  rtcTime = editTime;
  snprintf(clockSyncStatusMsg, sizeof(clockSyncStatusMsg), "GPS SYNC OK");
  clockSyncStatusMs = millis();
  return true;
}

// Sync RTC clock from standard NTP server (when Wi-Fi is connected)
bool syncClockFromNTP() {
  if (!settingWifiEnabled || WiFi.status() != WL_CONNECTED) {
    return false;
  }
  snprintf(clockSyncStatusMsg, sizeof(clockSyncStatusMsg), "SYNCING NTP...");
  clockSyncStatusMs = millis();
  renderCurrentScreen();

  configTime((long)utcOffset * 3600L, 0, NTP_SERVER_PRIMARY, NTP_SERVER_SECONDARY);
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 4000)) {
    editTime.year      = (uint16_t)(timeinfo.tm_year + 1900);
    editTime.month     = (uint8_t)(timeinfo.tm_mon + 1);
    editTime.day       = (uint8_t)timeinfo.tm_mday;
    editTime.hour      = (uint8_t)timeinfo.tm_hour;
    editTime.minute    = (uint8_t)timeinfo.tm_min;
    editTime.second    = (uint8_t)timeinfo.tm_sec;
    editTime.dayOfWeek = (uint8_t)(timeinfo.tm_wday % 7);
    writeRTC(editTime);
    rtcTime = editTime;
    snprintf(clockSyncStatusMsg, sizeof(clockSyncStatusMsg), "NTP SYNC OK");
    clockSyncStatusMs = millis();
    return true;
  }
  snprintf(clockSyncStatusMsg, sizeof(clockSyncStatusMsg), "NTP SYNC ERR");
  clockSyncStatusMs = millis();
  return false;
}

// Adjust selected Clock field by delta (-1 or +1)
void adjustClockField(int8_t delta) {
  switch (activeField) {
    case FIELD_HOUR:
      editTime.hour = (editTime.hour + 24 + delta) % 24;
      break;
    case FIELD_MIN:
      editTime.minute = (editTime.minute + 60 + delta) % 60;
      break;
    case FIELD_SEC:
      editTime.second = (editTime.second + 60 + delta) % 60;
      break;
    case FIELD_UTC:
      utcOffset += delta;
      if (utcOffset > 14)  utcOffset = -12;
      if (utcOffset < -12) utcOffset = 14;
      break;
    case FIELD_MONTH: {
      int16_t m = (int16_t)editTime.month + delta;
      if (m > 12) m = 1;
      if (m < 1)  m = 12;
      editTime.month = (uint8_t)m;
      uint8_t maxD = getDaysInMonth(editTime.month, editTime.year);
      if (editTime.day > maxD) editTime.day = maxD;
      break;
    }
    case FIELD_YEAR: {
      int16_t y = (int16_t)editTime.year + delta;
      if (y > 2099) y = 2000;
      if (y < 2000) y = 2099;
      editTime.year = (uint16_t)y;
      uint8_t maxD = getDaysInMonth(editTime.month, editTime.year);
      if (editTime.day > maxD) editTime.day = maxD;
      break;
    }
    case FIELD_DAY: {
      uint8_t maxD = getDaysInMonth(editTime.month, editTime.year);
      int16_t d = (int16_t)editTime.day + delta;
      if (d > maxD) d = 1;
      if (d < 1)    d = maxD;
      editTime.day = (uint8_t)d;
      break;
    }
    case FIELD_DOW:
      editTime.dayOfWeek = (editTime.dayOfWeek + 7 + delta) % 7;
      break;
  }
}

// Handle single tap at coordinates (x, y) while screen is active
void handleScreenTap(int16_t x, int16_t y) {
  // Intercept taps on active alarm (Cancel / Snooze) or timer (OK) alert dialog
  if (activeAlertType != ALERT_NONE) {
    if (activeAlertType == ALERT_ALARM) {
      if (y >= 152 && y <= 214) {
        uint8_t idx = (activeAlertAlarmIdx < ALARM_COUNT) ? activeAlertAlarmIdx : 0;
        if (x >= 18 && x <= 118) {
          // Cancel button
          alarms[idx].snoozePending = false;
          activeAlertType = ALERT_NONE;
          stopAlertRingtone();
          renderCurrentScreen();
          return;
        }
        if (x >= 122 && x <= 222) {
          // Snooze button
          alarms[idx].snoozePending  = true;
          alarms[idx].snoozeTargetMs = millis() + (uint32_t)alarms[idx].snoozeMin * 60000UL;
          activeAlertType = ALERT_NONE;
          stopAlertRingtone();
          renderCurrentScreen();
          return;
        }
      }
    } else if (activeAlertType == ALERT_TIMER) {
      if (x >= 48 && x <= 192 && y >= 152 && y <= 214) {
        // OK button
        activeAlertType = ALERT_NONE;
        stopAlertRingtone();
        timerRemainingMs = getTimerSetTotalMs();
        renderCurrentScreen();
        return;
      }
    }
    return;
  }

  if (currentScreen == SCREEN_MAIN_MENU) {
    // Top-left arrow -> previous main menu page (0..3)
    if (x <= 44 && y <= 34) {
      mainMenuPage = (uint8_t)((mainMenuPage + MAIN_MENU_PAGE_COUNT - 1) % MAIN_MENU_PAGE_COUNT);
      renderCurrentScreen();
      return;
    }

    // Top-right arrow -> next main menu page (0..3)
    if (x >= 196 && y <= 34) {
      mainMenuPage = (uint8_t)((mainMenuPage + 1) % MAIN_MENU_PAGE_COUNT);
      renderCurrentScreen();
      return;
    }

    // Handle taps on 9 icons of main menu page 1
    if (mainMenuPage == 0) {
      for (int8_t i = 0; i < 9; i++) {
        int16_t cx, cy;
        getHexGridCenter(i, cx, cy);
        int32_t dx = x - cx;
        int32_t dy = y - (cy - 4);
        if ((dx * dx + dy * dy) <= (28 * 28)) {
          if (i == 0) { // Calendar icon
            calMonth = rtcTime.month;
            calYear  = rtcTime.year;
            currentScreen = SCREEN_CALENDAR;
            renderCurrentScreen();
          } else if (i == 1) { // Calc icon
            currentScreen = SCREEN_CALC;
            renderCurrentScreen();
          } else if (i == 2) { // Clock icon
            isEditingClock = false;
            currentScreen = SCREEN_CLOCK;
            renderCurrentScreen();
          } else if (i == 3) { // GPS icon
            gpsPage = 0;
            currentScreen = SCREEN_GPS;
            renderCurrentScreen();
          } else if (i == 4) { // IR icon
            irSettingsPage = false;
            currentScreen = SCREEN_IR;
            renderCurrentScreen();
          } else if (i == 5) { // Step counter icon
            currentScreen = SCREEN_STEPS;
            renderCurrentScreen();
          } else if (i == 6) { // Inclinometer icon
            currentScreen = SCREEN_INCLINOMETER;
            lastInclDrawMs = millis();
            renderCurrentScreen();
          } else if (i == 7) { // Earth / Satellites icon
            earthDragging = false;
            currentScreen = SCREEN_EARTH;
            renderCurrentScreen();
          } else if (i == 8) { // Settings icon
            settingsView = SETTINGS_VIEW_LIST;
            settingsScrollOffset = 0;
            settingsSliderDragging = false;
            touchSensSliderDragging = false;
            currentScreen = SCREEN_SETTINGS;
            renderCurrentScreen();
          }
          break;
        }
      }
    }
    // Handle taps on all 9 icons of main menu page 2 (Wi-Fi..Hosts scan, Bluetooth scan, Sound spec/gen)
    else if (mainMenuPage == 1) {
      for (int8_t i = 0; i < 9; i++) {
        int16_t cx, cy;
        getHexGridCenter(i, cx, cy);
        int32_t dx = x - cx;
        int32_t dy = y - (cy - 4);
        if ((dx * dx + dy * dy) <= (28 * 28)) {
          if (i == 0) {
            stopTouchVibrationNow();
            currentScreen = SCREEN_WIFI_CONNECT;
            wifiView = WIFI_VIEW_SCANNING;
            renderCurrentScreen();
            scanWifiNetworks();
            wifiView = WIFI_VIEW_LIST;
            renderCurrentScreen();
          } else if (i == 1) {
            wifiListView = WIFILIST_VIEW_LIST;
            wifiListScrollOffset = 0;
            wifiListSelectedIdx = -1;
            currentScreen = SCREEN_WIFI_LIST;
            renderCurrentScreen();
          } else if (i == 2) {
            memset(wifiAnalyzerLoad, 0, sizeof(wifiAnalyzerLoad));
            memset(wifiAnalyzerApCount, 0, sizeof(wifiAnalyzerApCount));
            wifiAnalyzerHasData = false;
            currentScreen = SCREEN_WIFI_ANALYZER;
            startWifiAnalyzerScan();
            renderCurrentScreen();
          } else if (i == 3) {
            wifiInfoScrollOffset = 0;
            refreshWifiInfoData();
            currentScreen = SCREEN_WIFI_INFO;
            renderCurrentScreen();
          } else if (i == 4) {
            currentScreen = SCREEN_WIFI_JAM;
            startWifiJamDetect();
            renderCurrentScreen();
          } else if (i == 5) {
            currentScreen = SCREEN_HOSTS_SCAN;
            startHostsScan();
            renderCurrentScreen();
          } else if (i == 6) {
            stopTouchVibrationNow();
            currentScreen = SCREEN_BT_SCAN;
            requestBluetoothScan();
            renderCurrentScreen();
          } else if (i == 7) {
            currentScreen = SCREEN_SOUND_SPEC;
            startSoundSpectrometer();
            renderCurrentScreen();
          } else if (i == 8) {
            currentScreen = SCREEN_SOUND_GEN;
            renderCurrentScreen();
          }
          break;
        }
      }
    }
    // Handle taps on all 9 icons of main menu page 3
    else if (mainMenuPage == 2) {
      for (int8_t i = 0; i < 9; i++) {
        int16_t cx, cy;
        getHexGridCenter(i, cx, cy);
        int32_t dx = x - cx;
        int32_t dy = y - (cy - 4);
        if ((dx * dx + dy * dy) <= (28 * 28)) {
          if (i == 0) {
            asciiScrollOffset = 0;
            currentScreen = SCREEN_ASCII;
            renderCurrentScreen();
          } else if (i == 1) {
            periodicView = PERIODIC_VIEW_TABLE;
            periodicSelectedIdx = -1;
            currentScreen = SCREEN_PERIODIC;
            renderCurrentScreen();
          } else if (i == 2) {
            eAddView = EADD_VIEW_LIST;
            eAddSelectedIdx = -1;
            currentScreen = SCREEN_E_ADDITIVES;
            renderCurrentScreen();
          } else if (i == 3) {
            if (tempHistoryCount == 0) {
              recordEsp32TempSample();
            }
            currentScreen = SCREEN_TEMP;
            renderCurrentScreen();
          } else if (i == 4) {
            if (accelHistoryCount == 0) {
              recordAccelSample(currentAccelX, currentAccelY, currentAccelZ);
            }
            currentScreen = SCREEN_ACCEL;
            renderCurrentScreen();
          } else if (i == 5) {
            stopTouchVibrationNow();
            currentScreen = SCREEN_SUN_ACTIVITY;
            requestSunActivityFetch();
            renderCurrentScreen();
          } else if (i == 6) {
            stopTouchVibrationNow();
            currentScreen = SCREEN_WEATHER;
            requestWeatherFetch();
            renderCurrentScreen();
          } else if (i == 7) {
            sysInfoScrollOffset = 0;
            currentScreen = SCREEN_SYSTEM_INFO;
            renderCurrentScreen();
          } else if (i == 8) {
            currentScreen = SCREEN_ABOUT_FW;
            renderCurrentScreen();
          }
          break;
        }
      }
    }
    // Handle taps on main menu page 4 items (Alarm, Stopwatch, Timer, Flashlight, SOS signal, Morse code)
    else if (mainMenuPage == 3) {
      for (int8_t i = 0; i < 9; i++) {
        int16_t cx, cy;
        getHexGridCenter(i, cx, cy);
        int32_t dx = x - cx;
        int32_t dy = y - (cy - 4);
        if ((dx * dx + dy * dy) <= (28 * 28)) {
          if (i == 0) {
            currentScreen = SCREEN_ALARM;
            renderCurrentScreen();
          } else if (i == 1) {
            currentScreen = SCREEN_STOPWATCH;
            lastSwDrawMs = millis();
            renderCurrentScreen();
          } else if (i == 2) {
            currentScreen = SCREEN_TIMER;
            lastTimerDrawMs = millis();
            renderCurrentScreen();
          } else if (i == 3) {
            currentScreen = SCREEN_FLASHLIGHT;
            flashlightSliderDragging = false;
            renderCurrentScreen();
          } else if (i == 4) {
            currentScreen = SCREEN_SOS;
            startSosSignal();
            renderCurrentScreen();
          } else if (i == 5) {
            currentScreen = SCREEN_MORSE;
            stopMorsePlayback();
            renderCurrentScreen();
          } else if (i == 6) {
            currentScreen = SCREEN_TETRIS;
            tetrisResetGame();
            renderCurrentScreen();
          } else if (i == 7) {
            currentScreen = SCREEN_PAINT;
            paintDrawing = false;
            paintLastX = -1;
            paintLastY = -1;
            loadPaintFromFlash();
            renderCurrentScreen();
          } else if (i == 8) {
            currentScreen = SCREEN_ACCEL_SPEC;
            startAccelSpectrometer();
            renderCurrentScreen();
          }
          break;
        }
      }
    }
    // Handle taps on main menu page 5 items (Bluetooth keyboard, Bluetooth mouse, Bluetooth presenter)
    else if (mainMenuPage == 4) {
      for (int8_t i = 0; i < 9; i++) {
        if (MENU_LABELS_PAGE5[i][0] == '\0') continue;
        int16_t cx, cy;
        getHexGridCenter(i, cx, cy);
        int32_t dx = x - cx;
        int32_t dy = y - (cy - 4);
        if ((dx * dx + dy * dy) <= (28 * 28)) {
          if (i == 0) {
            currentScreen = SCREEN_BT_KEYBOARD;
            requestBleHidKeyboardStart();
            renderCurrentScreen();
          } else if (i == 1) {
            currentScreen = SCREEN_BT_MOUSE;
            requestBleHidMouseStart();
            renderCurrentScreen();
          } else if (i == 2) {
            currentScreen = SCREEN_BT_PRESENTER;
            requestBleHidPresenterStart();
            renderCurrentScreen();
          } else if (i == 3) {
            colorTableSelected = -1;
            currentScreen = SCREEN_COLOR_TABLE;
            renderCurrentScreen();
          } else if (i == 4) {
            currentScreen = SCREEN_RESISTOR_TABLE;
            renderCurrentScreen();
          } else if (i == 5) {
            currentScreen = SCREEN_VOLTAGE_DIVIDER;
            renderCurrentScreen();
          }
          break;
        }
      }
    }
  }
  else if (currentScreen == SCREEN_CALENDAR) {
    // Exit to main menu on top-left X tap
    if (x <= 44 && y <= 34) {
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }

    // Change year via triangle buttons
    if (y >= 35 && y <= 57) {
      if (x >= 10 && x <= 60) {
        calYear = (calYear > 2000) ? (uint16_t)(calYear - 1) : 2099;
        renderCurrentScreen();
        return;
      } else if (x >= 180 && x <= 230) {
        calYear = (calYear < 2099) ? (uint16_t)(calYear + 1) : 2000;
        renderCurrentScreen();
        return;
      }
    }

    // Change month via triangle buttons
    if (y >= 60 && y <= 82) {
      if (x >= 10 && x <= 60) {
        if (calMonth > 1) {
          calMonth--;
        } else {
          calMonth = 12;
          calYear = (calYear > 2000) ? (uint16_t)(calYear - 1) : 2099;
        }
        renderCurrentScreen();
        return;
      } else if (x >= 180 && x <= 230) {
        if (calMonth < 12) {
          calMonth++;
        } else {
          calMonth = 1;
          calYear = (calYear < 2099) ? (uint16_t)(calYear + 1) : 2000;
        }
        renderCurrentScreen();
        return;
      }
    }
  }
  else if (currentScreen == SCREEN_CLOCK) {
    if (x <= 44 && y <= 34) {
      isEditingClock = false;
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }

    if (x >= 196 && y <= 34) {
      if (!isEditingClock) {
        editTime = rtcTime;
        activeField = FIELD_HOUR;
        isEditingClock = true;
      } else {
        writeRTC(editTime);
        rtcTime = editTime;
        isEditingClock = false;
      }
      renderCurrentScreen();
      return;
    }

    if (isEditingClock) {
      if (y >= 70 && y <= 114) {
        if (x < 52)                   activeField = FIELD_HOUR;
        else if (x >= 52 && x < 100)  activeField = FIELD_MIN;
        else if (x >= 100 && x < 152) activeField = FIELD_SEC;
        else                          activeField = FIELD_UTC;
        renderCurrentScreen();
        return;
      }

      if (y >= 120 && y <= 165) {
        if (x < 48)                  activeField = FIELD_DAY;
        else if (x >= 48 && x < 92)  activeField = FIELD_MONTH;
        else if (x >= 92 && x < 172) activeField = FIELD_YEAR;
        else                         activeField = FIELD_DOW;
        renderCurrentScreen();
        return;
      }

      if (y >= 180 && x >= 6 && x <= 60) {
        adjustClockField(-1);
        renderCurrentScreen();
        return;
      }

      // GPS clock sync button
      if (y >= 180 && x >= 61 && x <= 119) {
        syncClockFromGPS();
        renderCurrentScreen();
        return;
      }

      // NTP clock sync button
      if (y >= 180 && x >= 121 && x <= 179) {
        syncClockFromNTP();
        renderCurrentScreen();
        return;
      }

      if (y >= 180 && x >= 180 && x <= 234) {
        adjustClockField(+1);
        renderCurrentScreen();
        return;
      }
    }
  }
  else if (currentScreen == SCREEN_GPS) {
    if (x <= 44 && y <= 34) {
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }
    // Top-right button cycles GPS pages (1/3 -> 2/3 -> 3/3)
    if (x >= 196 && y <= 34) {
      gpsPage = (uint8_t)((gpsPage + 1) % GPS_PAGE_COUNT);
      renderCurrentScreen();
      return;
    }
  }
  else if (currentScreen == SCREEN_CALC) {
    if (x <= 44 && y <= 34) {
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }

    if (x >= 196 && y <= 34) {
      uint8_t prevPage = calcPage;
      calcPage = (uint8_t)((calcPage + 1) % 5);
      calcSecondPage = (calcPage != 0);
      if (calcPage == 4 && prevPage != 4) {
        syncCalcToBaseConverter();
      } else if (prevPage == 4 && calcPage == 0) {
        syncBaseConverterToCalc();
      }
      renderCurrentScreen();
      return;
    }

    if (y >= 68) {
      if (calcPage == 0) {
        int8_t row = (int8_t)((y - 69) / 34);
        int8_t col = (int8_t)((x - 6) / 57);
        row = constrain(row, (int8_t)0, (int8_t)4);
        col = constrain(col, (int8_t)0, (int8_t)3);
        if (row == 4 && col == 1) col = 0;
        handleCalcPage1Button(CALC_PAGE1_LABELS[row][col]);
      } else if (calcPage == 1) {
        int8_t row = (int8_t)((y - 69) / 42);
        int8_t col = (int8_t)((x - 6) / 57);
        row = constrain(row, (int8_t)0, (int8_t)3);
        col = constrain(col, (int8_t)0, (int8_t)3);
        handleCalcPage2Button((int8_t)(row * 4 + col));
      } else if (calcPage == 2) {
        int8_t row = (int8_t)((y - 69) / 42);
        int8_t col = (int8_t)((x - 6) / 57);
        row = constrain(row, (int8_t)0, (int8_t)3);
        col = constrain(col, (int8_t)0, (int8_t)3);
        if (row < 3) {
          handleCalcPage3Button((int8_t)(row * 4 + col));
        } else {
          handleCalcPage3Button((col < 2) ? 12 : 13);
        }
      } else if (calcPage == 3) {
        // Page 4 (inverse hyperbolic functions, 3x2 grid)
        int8_t row = (int8_t)((y - 72) / 55);
        int8_t col = (x < 120) ? 0 : 1;
        row = constrain(row, (int8_t)0, (int8_t)2);
        handleCalcPage4Button((int8_t)(row * 2 + col));
      } else {
        // Page 5 (number base converter: top row BIN/OCT/DEC/HEX, 5x4 keypad below)
        int8_t col = (int8_t)((x - 6) / 57);
        col = constrain(col, (int8_t)0, (int8_t)3);
        if (y <= 94) {
          handleCalcPage5BaseSelect(CALC_PAGE5_BASE_VALS[col]);
        } else {
          int8_t row = (int8_t)((y - 96) / 28);
          row = constrain(row, (int8_t)0, (int8_t)4);
          handleCalcPage5Key(CALC_PAGE5_KEYS[row][col]);
        }
      }
      renderCurrentScreen();
    }
  }
  else if (currentScreen == SCREEN_EARTH) {
    if (x <= 44 && y <= 34) {
      earthDragging = false;
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }

    if (x >= 196 && y <= 34) {
      earthShowSatellite = !earthShowSatellite;
      renderCurrentScreen();
      return;
    }
  }
  else if (currentScreen == SCREEN_STEPS) {
    if (x <= 44 && y <= 34) {
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }

    if (x >= 196 && y <= 34) {
      resetBma423HardwareSteps();
      saveStepCountToFlash(true);
      renderCurrentScreen();
      return;
    }
  }
  else if (currentScreen == SCREEN_IR) {
    if (x <= 44 && y <= 34) {
      irSettingsPage = false;
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }

    if (x >= 196 && y <= 34) {
      irSettingsPage = !irSettingsPage;
      renderCurrentScreen();
      return;
    }

    if (!irSettingsPage) {
      if (y >= 68) {
        int8_t row = (int8_t)((y - 69) / 34);
        row = constrain(row, (int8_t)0, (int8_t)4);
        if (row == 4) {
          if (x < 62) {
            handleIrButton("0");
          } else if (x < 147) {
            handleIrButton("Test LED");
          } else {
            handleIrButton("Send");
          }
        } else {
          int8_t col = (int8_t)((x - 6) / 57);
          col = constrain(col, (int8_t)0, (int8_t)3);
          handleIrButton(IR_PAD_LABELS[row][col]);
        }
        renderCurrentScreen();
      }
    } else {
      if (y >= 80 && y <= 126) {
        if (x >= 10 && x <= 78)        irProtocol = IR_PROTO_NEC;
        else if (x >= 86 && x <= 154)  irProtocol = IR_PROTO_RC5;
        else if (x >= 162 && x <= 230) irProtocol = IR_PROTO_RC6;
        renderCurrentScreen();
      } else if (y >= 152 && y <= 198) {
        if (x >= 10 && x <= 78)        irCarrierKhz = 33;
        else if (x >= 86 && x <= 154)  irCarrierKhz = 36;
        else if (x >= 162 && x <= 230) irCarrierKhz = 38;
        initIrLedc(irCarrierKhz);
        renderCurrentScreen();
      }
    }
  }
  else if (currentScreen == SCREEN_INCLINOMETER) {
    // Exit to main menu on top-left X tap
    if (x <= 44 && y <= 34) {
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }
  }
  else if (currentScreen == SCREEN_WIFI_CONNECT) {
    // Exit to main menu on top-left X tap
    if (x <= 44 && y <= 34) {
      if (wifiView == WIFI_VIEW_CONNECTING) {
        WiFi.disconnect();
      }
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }

    if (wifiView == WIFI_VIEW_LIST) {
      // Scroll-up button (top-right)
      if (x >= 196 && y >= 34 && y <= 84) {
        if (wifiScrollOffset > 0) {
          wifiScrollOffset--;
          renderCurrentScreen();
        }
        return;
      }
      // Scroll-down button (bottom-right)
      if (x >= 196 && y >= 184 && y <= 238) {
        if (wifiScrollOffset + WIFI_VISIBLE_ROWS < wifiScanCount) {
          wifiScrollOffset++;
          renderCurrentScreen();
        }
        return;
      }
      // Tap on a network row in the list
      if (x >= 6 && x <= 194 && y >= 34 && y < 234) {
        uint8_t row = (uint8_t)((y - 34) / 40);
        uint8_t idx = (uint8_t)(wifiScrollOffset + row);
        if (row < WIFI_VISIBLE_ROWS && idx < wifiScanCount) {
          strncpy(wifiSelectedSsid, wifiScanList[idx].ssid, 32);
          wifiSelectedSsid[32] = '\0';
          wifiSelectedLocked = wifiScanList[idx].isLocked;
          wifiPasswordBuf[0] = '\0';
          wifiKbPage = 0;
          wifiKbShift = false;

          if (wifiSelectedLocked) {
            wifiView = WIFI_VIEW_PASSWORD;
            renderCurrentScreen();
          } else {
            startWifiConnectAttempt();
            renderCurrentScreen();
          }
        }
        return;
      }
    }
    else if (wifiView == WIFI_VIEW_PASSWORD) {
      // Top-right button -> cycle 4 keyboard character pages
      if (x >= 196 && y <= 34) {
        wifiKbPage = (uint8_t)((wifiKbPage + 1) % 4);
        renderCurrentScreen();
        return;
      }

      // Bottom row (Shift / <- / Enter)
      if (y >= 194 && y <= 236) {
        if (x >= 6 && x <= 60) {
          wifiKbShift = !wifiKbShift;
          renderCurrentScreen();
          return;
        }
        if (x >= 62 && x <= 117) {
          size_t len = strlen(wifiPasswordBuf);
          if (len > 0) {
            wifiPasswordBuf[len - 1] = '\0';
            renderCurrentScreen();
          }
          return;
        }
        if (x >= 120 && x <= 234) {
          startWifiConnectAttempt();
          renderCurrentScreen();
          return;
        }
      }

      // 3 character key rows (y = 69..192)
      if (y >= 68 && y < 194) {
        int8_t row = (int8_t)((y - 69) / 42);
        row = constrain(row, (int8_t)0, (int8_t)2);
        char ch = 0;

        if (wifiKbPage == 0 || wifiKbPage == 3) {
          int8_t col = constrain((int8_t)((x - 6) / 57), (int8_t)0, (int8_t)3);
          const char* keys = (wifiKbPage == 0)
                               ? (wifiKbShift ? WIFI_KB_P0_HI : WIFI_KB_P0_LO)
                               : (wifiKbShift ? WIFI_KB_P3_HI : WIFI_KB_P3_LO);
          ch = keys[row * 4 + col];
        } else if (wifiKbPage == 1) {
          const char* keys = wifiKbShift ? WIFI_KB_P1_HI : WIFI_KB_P1_LO;
          if (row < 2) {
            int8_t col = constrain((int8_t)((x - 6) / 57), (int8_t)0, (int8_t)3);
            ch = keys[row * 4 + col];
          } else {
            int8_t col = constrain((int8_t)((x - 6) / 77), (int8_t)0, (int8_t)2);
            ch = keys[8 + col];
          }
        } else {
          const char* keys = wifiKbShift ? WIFI_KB_P2_HI : WIFI_KB_P2_LO;
          if (row < 2) {
            int8_t col = constrain((int8_t)((x - 6) / 57), (int8_t)0, (int8_t)3);
            ch = keys[row * 4 + col];
          } else {
            int8_t col = (x < 120) ? 0 : 1;
            ch = keys[8 + col];
          }
        }

        if (ch != 0) {
          size_t len = strlen(wifiPasswordBuf);
          if (len < 63) {
            wifiPasswordBuf[len] = ch;
            wifiPasswordBuf[len + 1] = '\0';
            renderCurrentScreen();
          }
        }
        return;
      }
    }
    else if (wifiView == WIFI_VIEW_RESULT) {
      // Tap OK button -> return to main menu
      if (x >= 55 && x <= 185 && y >= 170 && y <= 230) {
        currentScreen = SCREEN_MAIN_MENU;
        renderCurrentScreen();
        return;
      }
    }
  }
  else if (currentScreen == SCREEN_WIFI_LIST) {
    // Top-left X -> return to main menu
    if (x <= 44 && y <= 34) {
      wifiListView = WIFILIST_VIEW_LIST;
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }

    if (wifiListView == WIFILIST_VIEW_LIST) {
      // Scroll up/down via right-side triangles
      if (x >= 196 && y >= 32 && y <= 94) {
        if (wifiListScrollOffset > 0) {
          wifiListScrollOffset--;
          renderCurrentScreen();
        }
        return;
      }
      if (x >= 196 && y >= 178 && y <= 239) {
        if ((uint16_t)wifiListScrollOffset + WIFI_VISIBLE_ROWS < savedWifiCount) {
          wifiListScrollOffset++;
          renderCurrentScreen();
        }
        return;
      }

      // Tap saved SSID -> dialog with SSID, password, and OK / Connect / Delete buttons
      if (x >= 4 && x <= 195 && y >= 34 && y < 34 + WIFI_VISIBLE_ROWS * 41) {
        uint8_t row = (uint8_t)((y - 34) / 41);
        uint8_t idx = (uint8_t)(wifiListScrollOffset + row);
        if (idx < savedWifiCount) {
          wifiListSelectedIdx = (int8_t)idx;
          wifiListView = WIFILIST_VIEW_DIALOG;
          renderCurrentScreen();
        }
        return;
      }
    }
    else if (wifiListView == WIFILIST_VIEW_DIALOG) {
      // OK button (left) -> return to saved networks list
      if (x >= 6 && x <= 76 && y >= 172 && y <= 228) {
        wifiListView = WIFILIST_VIEW_LIST;
        renderCurrentScreen();
        return;
      }
      // Connect button (center) -> connect to selected network using saved password
      if (x >= 78 && x <= 162 && y >= 172 && y <= 228) {
        startWifiListConnectAttempt();
        renderCurrentScreen();
        return;
      }
      // Delete button (right) -> remove network from list and Flash, then return to list
      if (x >= 164 && x <= 234 && y >= 172 && y <= 228) {
        if (wifiListSelectedIdx >= 0 && wifiListSelectedIdx < savedWifiCount) {
          deleteSavedWifiCredential((uint8_t)wifiListSelectedIdx);
        }
        wifiListSelectedIdx = -1;
        wifiListView = WIFILIST_VIEW_LIST;
        renderCurrentScreen();
        return;
      }
    }
    else if (wifiListView == WIFILIST_VIEW_RESULT) {
      // OK button after connection result -> return to main menu
      if (x >= 55 && x <= 185 && y >= 170 && y <= 230) {
        wifiListView = WIFILIST_VIEW_LIST;
        wifiListSelectedIdx = -1;
        currentScreen = SCREEN_MAIN_MENU;
        renderCurrentScreen();
        return;
      }
    }
  }
  else if (currentScreen == SCREEN_SETTINGS) {
    // Top-left X -> return to main menu (reverting unsaved brightness)
    if (x <= 44 && y <= 34) {
      if (settingsView == SETTINGS_VIEW_BRIGHTNESS) {
        tft.setBrightness(getFullBrightnessPwm());
      }
      settingsSliderDragging = false;
      touchSensSliderDragging = false;
      settingsView = SETTINGS_VIEW_LIST;
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }

    if (settingsView == SETTINGS_VIEW_LIST) {
      uint8_t maxOffset = (SETTINGS_ITEM_COUNT > SETTINGS_VISIBLE_ROWS)
                            ? (uint8_t)(SETTINGS_ITEM_COUNT - SETTINGS_VISIBLE_ROWS)
                            : 0;
      // Scroll UP button on top-right
      if (x >= 196 && x <= 239 && y >= 32 && y <= 96) {
        if (settingsScrollOffset > 0) {
          settingsScrollOffset--;
          renderCurrentScreen();
        }
        return;
      }
      // Scroll DOWN button on bottom-right
      if (x >= 196 && x <= 239 && y >= 174 && y <= 239) {
        if (settingsScrollOffset < maxOffset) {
          settingsScrollOffset++;
          renderCurrentScreen();
        }
        return;
      }

      // Select settings item accounting for scroll offset (4 visible rows: y = 36 + i * 50)
      if (x >= 6 && x <= 194 && y >= 36 && y < 236) {
        uint8_t row = (uint8_t)((y - 36) / 50);
        uint8_t itemIdx = (uint8_t)(settingsScrollOffset + row);
        if (itemIdx == 0) {
          editWifiEnabled     = settingWifiEnabled;
          editWifiAutoConnect = settingWifiAutoConnect;
          settingsView = SETTINGS_VIEW_WIFI_AUTO;
          renderCurrentScreen();
          return;
        }
        if (itemIdx == 1) {
          editDisplayBrightness = (settingDisplayBrightness < BRIGHTNESS_MIN_PCT)
                                    ? BRIGHTNESS_MIN_PCT
                                    : settingDisplayBrightness;
          settingsView = SETTINGS_VIEW_BRIGHTNESS;
          renderCurrentScreen();
          return;
        }
        if (itemIdx == 2) {
          editDisplayTimeIdx = findDisplayTimeOptionIdx(settingDisplayLightSec);
          settingsView = SETTINGS_VIEW_LIGHT_TIME;
          renderCurrentScreen();
          return;
        }
        if (itemIdx == 3) {
          editTouchVibration = settingTouchVibration;
          settingsView = SETTINGS_VIEW_TOUCH_VIB;
          renderCurrentScreen();
          return;
        }
        if (itemIdx == 4) {
          editTouchSensitivity = settingTouchSensitivity;
          editWakeTapCount = settingWakeTapCount;
          touchSensSliderDragging = false;
          settingsView = SETTINGS_VIEW_TOUCH_SENS;
          renderCurrentScreen();
          return;
        }
        if (itemIdx == 5) {
          editChargeCurrentMa = settingChargeCurrentMa;
          settingsView = SETTINGS_VIEW_CHARGE_CURR;
          renderCurrentScreen();
          return;
        }
        if (itemIdx == 6) {
          editPowerGps  = settingPowerGps;
          editPowerLora = settingPowerLora;
          settingsView = SETTINGS_VIEW_POWER;
          renderCurrentScreen();
          return;
        }
        if (itemIdx == 7) {
          editWakeOnTouch  = settingWakeOnTouch;
          editWakeOnBma423 = settingWakeOnBma423;
          editWakeOnButton = settingWakeOnButton;
          settingsView = SETTINGS_VIEW_WAKE_CONDITION;
          renderCurrentScreen();
          return;
        }
      }
      return;
    }

    if (settingsView == SETTINGS_VIEW_WIFI_AUTO) {
      // Row 1: Global "Wi-Fi enabled" toggle
      if (x >= 12 && x <= 228 && y >= 64 && y <= 112) {
        editWifiEnabled = !editWifiEnabled;
        renderCurrentScreen();
        return;
      }
      // Row 2: "Wi-Fi autoconnect" toggle
      if (x >= 12 && x <= 228 && y >= 114 && y <= 164) {
        editWifiAutoConnect = !editWifiAutoConnect;
        renderCurrentScreen();
        return;
      }
      // Cancel -> return to list without saving
      if (x >= 8 && x <= 116 && y >= 174 && y <= 228) {
        settingsView = SETTINGS_VIEW_LIST;
        renderCurrentScreen();
        return;
      }
      // OK -> save to Flash, apply Wi-Fi power state, and return to list
      if (x >= 124 && x <= 232 && y >= 174 && y <= 228) {
        settingWifiEnabled     = editWifiEnabled;
        settingWifiAutoConnect = editWifiAutoConnect;
        saveWatchSettings();
        applyWifiPowerState();
        settingsView = SETTINGS_VIEW_LIST;
        renderCurrentScreen();
        return;
      }
      return;
    }

    if (settingsView == SETTINGS_VIEW_BRIGHTNESS) {
      // Tap brightness slider (5-100%, min 5%)
      if (x >= 10 && x <= 230 && y >= 102 && y <= 162) {
        int32_t pct = (int32_t)BRIGHTNESS_MIN_PCT + ((int32_t)(x - 22) * (100L - (int32_t)BRIGHTNESS_MIN_PCT) + 98L) / 196L;
        if (pct < BRIGHTNESS_MIN_PCT) pct = BRIGHTNESS_MIN_PCT;
        if (pct > 100) pct = 100;
        editDisplayBrightness = (uint8_t)pct;
        settingsSliderDragging = true;
        tft.setBrightness(getBrightnessPwmFromPercent(editDisplayBrightness));
        renderCurrentScreen();
        return;
      }
      // Cancel -> restore previous brightness and return to list
      if (x >= 8 && x <= 116 && y >= 174 && y <= 228) {
        settingsSliderDragging = false;
        tft.setBrightness(getFullBrightnessPwm());
        settingsView = SETTINGS_VIEW_LIST;
        renderCurrentScreen();
        return;
      }
      // OK -> save brightness to Flash (>= 5%), apply, and return to list
      if (x >= 124 && x <= 232 && y >= 174 && y <= 228) {
        settingsSliderDragging = false;
        if (editDisplayBrightness < BRIGHTNESS_MIN_PCT) editDisplayBrightness = BRIGHTNESS_MIN_PCT;
        settingDisplayBrightness = editDisplayBrightness;
        saveWatchSettings();
        tft.setBrightness(getFullBrightnessPwm());
        settingsView = SETTINGS_VIEW_LIST;
        renderCurrentScreen();
        return;
      }
      return;
    }

    if (settingsView == SETTINGS_VIEW_LIGHT_TIME) {
      // Left/right arrows for display backlight timeout
      if (x >= 8 && x <= 64 && y >= 78 && y <= 144) {
        if (editDisplayTimeIdx > 0) {
          editDisplayTimeIdx--;
          renderCurrentScreen();
        }
        return;
      }
      if (x >= 176 && x <= 232 && y >= 78 && y <= 144) {
        if (editDisplayTimeIdx + 1 < DISPLAY_TIME_OPT_COUNT) {
          editDisplayTimeIdx++;
          renderCurrentScreen();
        }
        return;
      }
      // Cancel -> return to list without saving
      if (x >= 8 && x <= 116 && y >= 174 && y <= 228) {
        settingsView = SETTINGS_VIEW_LIST;
        renderCurrentScreen();
        return;
      }
      // OK -> save to Flash and apply to backlight timer
      if (x >= 124 && x <= 232 && y >= 174 && y <= 228) {
        settingDisplayLightSec = DISPLAY_TIME_OPTIONS[editDisplayTimeIdx];
        saveWatchSettings();
        lastTouchMs = millis();
        settingsView = SETTINGS_VIEW_LIST;
        renderCurrentScreen();
        return;
      }
      return;
    }

    if (settingsView == SETTINGS_VIEW_TOUCH_VIB) {
      // Toggle touch vibration switch
      if (x >= 56 && x <= 184 && y >= 72 && y <= 140) {
        editTouchVibration = !editTouchVibration;
        renderCurrentScreen();
        return;
      }
      // Cancel -> return to list without saving
      if (x >= 8 && x <= 116 && y >= 174 && y <= 228) {
        settingsView = SETTINGS_VIEW_LIST;
        renderCurrentScreen();
        return;
      }
      // OK -> save toggle state to Flash and return to list
      if (x >= 124 && x <= 232 && y >= 174 && y <= 228) {
        settingTouchVibration = editTouchVibration;
        saveWatchSettings();
        settingsView = SETTINGS_VIEW_LIST;
        renderCurrentScreen();
        return;
      }
      return;
    }

    if (settingsView == SETTINGS_VIEW_TOUCH_SENS) {
      // Touch sensitivity slider (FT6236 reg 0x80: 100..180)
      if (x >= 10 && x <= 230 && y >= 62 && y <= 106) {
        int32_t val = (int32_t)TOUCH_SENS_MIN_VAL + ((int32_t)(x - 22) * (int32_t)(TOUCH_SENS_MAX_VAL - TOUCH_SENS_MIN_VAL) + 98L) / 196L;
        if (val < TOUCH_SENS_MIN_VAL) val = TOUCH_SENS_MIN_VAL;
        if (val > TOUCH_SENS_MAX_VAL) val = TOUCH_SENS_MAX_VAL;
        editTouchSensitivity = (uint8_t)val;
        touchSensSliderDragging = true;
        renderCurrentScreen();
        return;
      }
      // Screen wake gesture selector: 2x TAP (left) / 3x TAP (right)
      if (y >= 122 && y <= 168) {
        if (x >= 10 && x <= 118) {
          editWakeTapCount = 2;
          renderCurrentScreen();
          return;
        }
        if (x >= 122 && x <= 230) {
          editWakeTapCount = 3;
          renderCurrentScreen();
          return;
        }
      }
      // Cancel -> return to list without saving
      if (x >= 8 && x <= 116 && y >= 174 && y <= 228) {
        touchSensSliderDragging = false;
        settingsView = SETTINGS_VIEW_LIST;
        renderCurrentScreen();
        return;
      }
      // OK -> save settings to Flash, apply to FT6236/FT6336U, and return to list
      if (x >= 124 && x <= 232 && y >= 174 && y <= 228) {
        touchSensSliderDragging = false;
        settingTouchSensitivity = editTouchSensitivity;
        settingWakeTapCount = (editWakeTapCount == 3) ? 3 : 2;
        saveWatchSettings();
        initTouchScreen();
        settingsView = SETTINGS_VIEW_LIST;
        renderCurrentScreen();
        return;
      }
      return;
    }

    if (settingsView == SETTINGS_VIEW_CHARGE_CURR) {
      // 4 charge current option buttons: 100, 200, 300, 400 mA
      for (uint8_t i = 0; i < CHARGE_CURRENT_OPT_COUNT; i++) {
        int16_t bx = (i % 2 == 0) ? 14 : 124;
        int16_t by = (i < 2) ? 74 : 124;
        if (x >= bx && x <= bx + 102 && y >= by && y <= by + 42) {
          editChargeCurrentMa = CHARGE_CURRENT_OPTIONS[i];
          renderCurrentScreen();
          return;
        }
      }
      // Cancel -> return to list without saving
      if (x >= 8 && x <= 116 && y >= 174 && y <= 228) {
        settingsView = SETTINGS_VIEW_LIST;
        renderCurrentScreen();
        return;
      }
      // OK -> save charge current to Flash, apply to AXP2101, and return to list
      if (x >= 124 && x <= 232 && y >= 174 && y <= 228) {
        settingChargeCurrentMa = editChargeCurrentMa;
        saveWatchSettings();
        applyAxp2101PowerSettings();
        settingsView = SETTINGS_VIEW_LIST;
        renderCurrentScreen();
        return;
      }
      return;
    }

    if (settingsView == SETTINGS_VIEW_POWER) {
      // Row 1: GPS power (BLDO1) toggle
      if (x >= 12 && x <= 228 && y >= 72 && y <= 116) {
        editPowerGps = !editPowerGps;
        renderCurrentScreen();
        return;
      }
      // Row 2: LoRa power (ALDO4) toggle
      if (x >= 12 && x <= 228 && y >= 122 && y <= 166) {
        editPowerLora = !editPowerLora;
        renderCurrentScreen();
        return;
      }
      // Cancel -> return to list without saving
      if (x >= 8 && x <= 116 && y >= 174 && y <= 228) {
        settingsView = SETTINGS_VIEW_LIST;
        renderCurrentScreen();
        return;
      }
      // OK -> save GPS/LoRa power enables to Flash, set GPS UART Hi-Z if disabled, apply AXP2101 rails, return to list
      if (x >= 124 && x <= 232 && y >= 174 && y <= 228) {
        settingPowerGps  = editPowerGps;
        settingPowerLora = editPowerLora;
        saveWatchSettings();
        if (!settingPowerGps) {
          applyGpsUartState(); // Put GPS RX/TX into high-impedance INPUT before cutting BLDO1
        }
        applyAxp2101PowerSettings();
        if (settingPowerGps) {
          delay(10);
          applyGpsUartState(); // Start GPS UART after BLDO1 power rail stabilizes
        }
        settingsView = SETTINGS_VIEW_LIST;
        renderCurrentScreen();
        return;
      }
      return;
    }

    if (settingsView == SETTINGS_VIEW_WAKE_CONDITION) {
      // Row 1: Touch wake (TOUCH_INT_PIN / IO16)
      if (x >= 12 && x <= 228 && y >= 50 && y <= 89) {
        editWakeOnTouch = !editWakeOnTouch;
        renderCurrentScreen();
        return;
      }
      // Row 2: BMA423 interrupt wake (IO14)
      if (x >= 12 && x <= 228 && y >= 90 && y <= 129) {
        editWakeOnBma423 = !editWakeOnBma423;
        renderCurrentScreen();
        return;
      }
      // Row 3: Button wake (IO0)
      if (x >= 12 && x <= 228 && y >= 130 && y <= 169) {
        editWakeOnButton = !editWakeOnButton;
        renderCurrentScreen();
        return;
      }
      // Cancel -> discard edits and return to Settings list
      if (x >= 8 && x <= 116 && y >= 174 && y <= 228) {
        settingsView = SETTINGS_VIEW_LIST;
        renderCurrentScreen();
        return;
      }
      // OK -> save wake-up conditions to Flash (requires >= 1 enabled source), apply BMA423 INT1, return to list
      if (x >= 124 && x <= 232 && y >= 174 && y <= 228) {
        if (!editWakeOnTouch && !editWakeOnBma423 && !editWakeOnButton) {
          return;
        }
        settingWakeOnTouch  = editWakeOnTouch;
        settingWakeOnBma423 = editWakeOnBma423;
        settingWakeOnButton = editWakeOnButton;
        saveWatchSettings();
        configureBma423WakeInterrupt();
        settingsView = SETTINGS_VIEW_LIST;
        renderCurrentScreen();
        return;
      }
      return;
    }
  }
  else if (currentScreen == SCREEN_ASCII) {
    // Top-left X -> return to main menu
    if (x <= 44 && y <= 34) {
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }
    // Table scroll triangles on top-right and bottom-right (ASCII + CP1251, 0..255)
    if (x >= 196 && y >= 32 && y <= 94) {
      if (asciiScrollOffset >= ASCII_VISIBLE_ROWS) {
        asciiScrollOffset = (uint8_t)(asciiScrollOffset - ASCII_VISIBLE_ROWS);
      } else {
        asciiScrollOffset = 0;
      }
      renderCurrentScreen();
      return;
    }
    if (x >= 196 && y >= 178 && y <= 239) {
      if ((uint16_t)asciiScrollOffset + ASCII_VISIBLE_ROWS < ASCII_TOTAL_CODES) {
        uint16_t nextOff = (uint16_t)asciiScrollOffset + ASCII_VISIBLE_ROWS;
        if (nextOff > ASCII_TOTAL_CODES - ASCII_VISIBLE_ROWS) {
          nextOff = ASCII_TOTAL_CODES - ASCII_VISIBLE_ROWS;
        }
        asciiScrollOffset = (uint8_t)nextOff;
        renderCurrentScreen();
      }
      return;
    }
  }
  else if (currentScreen == SCREEN_TEMP) {
    // Top-left X -> return to main menu
    if (x <= 44 && y <= 34) {
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }
    // Tap C, K, or F at bottom to switch chart temperature units
    if (y >= 164) {
      if (x < 82) {
        tempGraphUnit = TEMP_UNIT_C;
      } else if (x < 158) {
        tempGraphUnit = TEMP_UNIT_K;
      } else {
        tempGraphUnit = TEMP_UNIT_F;
      }
      renderCurrentScreen();
      return;
    }
  }
  else if (currentScreen == SCREEN_PERIODIC) {
    // Top-left X -> return to main menu
    if (x <= 44 && y <= 34) {
      periodicView = PERIODIC_VIEW_TABLE;
      periodicSelectedIdx = -1;
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }

    if (periodicView == PERIODIC_VIEW_TABLE) {
      // Top-right button -> cycle Periodic table pages
      if (x >= 196 && y <= 34) {
        periodicPage = (uint8_t)((periodicPage + 1) % PERIODIC_PAGE_COUNT);
        renderCurrentScreen();
        return;
      }

      // Tap chemical element -> open properties dialog
      for (uint8_t i = 0; i < PERIODIC_ELEMENT_COUNT; i++) {
        int16_t bx, by, bw, bh;
        if (!getPeriodicElementSlot(i, periodicPage, bx, by, bw, bh)) continue;
        if (x >= bx && x <= bx + bw && y >= by && y <= by + bh) {
          periodicSelectedIdx = (int16_t)i;
          periodicView = PERIODIC_VIEW_DIALOG;
          renderCurrentScreen();
          return;
        }
      }
      return;
    }

    // OK button in dialog -> return to Periodic table
    if (x >= 52 && x <= 188 && y >= 186 && y <= 236) {
      periodicView = PERIODIC_VIEW_TABLE;
      periodicSelectedIdx = -1;
      renderCurrentScreen();
      return;
    }
  }
  else if (currentScreen == SCREEN_E_ADDITIVES) {
    // Top-left X -> return to main menu
    if (x <= 44 && y <= 34) {
      eAddView = EADD_VIEW_LIST;
      eAddSelectedIdx = -1;
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }

    if (eAddView == EADD_VIEW_LIST) {
      // Scroll list via top-right and bottom-right triangles
      if (x >= 196 && y >= 32 && y <= 94) {
        if (eAddScrollOffset >= EADD_VISIBLE_ROWS) {
          eAddScrollOffset = (uint8_t)(eAddScrollOffset - EADD_VISIBLE_ROWS);
        } else {
          eAddScrollOffset = 0;
        }
        renderCurrentScreen();
        return;
      }
      if (x >= 196 && y >= 178 && y <= 239) {
        if ((uint16_t)eAddScrollOffset + EADD_VISIBLE_ROWS < EADD_COUNT) {
          eAddScrollOffset = (uint8_t)(eAddScrollOffset + EADD_VISIBLE_ROWS);
          if (eAddScrollOffset > EADD_COUNT - EADD_VISIBLE_ROWS) {
            eAddScrollOffset = (uint8_t)(EADD_COUNT - EADD_VISIBLE_ROWS);
          }
          renderCurrentScreen();
        }
        return;
      }

      // Tap list item -> open food additive detail dialog
      if (x >= 4 && x <= 195 && y >= 34 && y < 34 + EADD_VISIBLE_ROWS * 41) {
        uint8_t row = (uint8_t)((y - 34) / 41);
        uint16_t idx = (uint16_t)eAddScrollOffset + row;
        if (idx < EADD_COUNT) {
          eAddSelectedIdx = (int16_t)idx;
          eAddView = EADD_VIEW_DIALOG;
          renderCurrentScreen();
        }
      }
      return;
    }

    // OK button in dialog -> return to list
    if (x >= 52 && x <= 188 && y >= 186 && y <= 236) {
      eAddView = EADD_VIEW_LIST;
      eAddSelectedIdx = -1;
      renderCurrentScreen();
      return;
    }
  }
  else if (currentScreen == SCREEN_WIFI_ANALYZER) {
    // Top-left X -> return to main menu
    if (x <= 44 && y <= 34) {
      if (wifiAnalyzerScanning) {
        WiFi.scanDelete();
        wifiAnalyzerScanning = false;
      }
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }
  }
  else if (currentScreen == SCREEN_ABOUT_FW) {
    // Top-left X -> return to main menu
    if (x <= 44 && y <= 34) {
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }
  }
  else if (currentScreen == SCREEN_WIFI_JAM) {
    // Top-left X -> stop promiscuous mode and return to main menu
    if (x <= 44 && y <= 34) {
      stopWifiJamDetect();
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }
  }
  else if (currentScreen == SCREEN_ACCEL) {
    // Top-left X -> return to main menu
    if (x <= 44 && y <= 34) {
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }
  }
  else if (currentScreen == SCREEN_WIFI_INFO) {
    // Top-left X -> return to main menu
    if (x <= 44 && y <= 34) {
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }
    if (!wifiInfoConnected) return;

    uint8_t maxOffset = (WIFI_INFO_ROW_COUNT > WIFI_INFO_VISIBLE_ROWS)
                          ? (uint8_t)(WIFI_INFO_ROW_COUNT - WIFI_INFO_VISIBLE_ROWS)
                          : 0;
    // Scroll UP button on top-right
    if (x >= 196 && x <= 239 && y >= 32 && y <= 100) {
      if (wifiInfoScrollOffset > 0) {
        wifiInfoScrollOffset--;
        renderCurrentScreen();
      }
      return;
    }
    // Scroll DOWN button on bottom-right
    if (x >= 196 && x <= 239 && y >= 174 && y <= 239) {
      if (wifiInfoScrollOffset < maxOffset) {
        wifiInfoScrollOffset++;
        renderCurrentScreen();
      }
      return;
    }
  }
  else if (currentScreen == SCREEN_SOUND_SPEC) {
    // Top-left X -> stop PDM microphone and return to main menu
    if (x <= 44 && y <= 34) {
      stopSoundSpectrometer();
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }
  }
  else if (currentScreen == SCREEN_SOUND_GEN) {
    // Top-left X -> stop audio output and return to main menu
    if (x <= 44 && y <= 34) {
      stopSoundGenerator();
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }

    // Frequency SpinBox (y = 32..72)
    if (y >= 32 && y <= 72) {
      uint16_t step = SND_FREQ_STEPS[sndGenFreqStepIdx];
      if (x >= 8 && x <= 50) {
        int32_t nextF = (int32_t)sndGenFreqHz - (int32_t)step;
        if (nextF < SND_GEN_FREQ_MIN) nextF = SND_GEN_FREQ_MIN;
        sndGenFreqHz = (uint16_t)nextF;
        renderCurrentScreen();
        return;
      }
      if (x >= 190 && x <= 232) {
        int32_t nextF = (int32_t)sndGenFreqHz + (int32_t)step;
        if (nextF > SND_GEN_FREQ_MAX) nextF = SND_GEN_FREQ_MAX;
        sndGenFreqHz = (uint16_t)nextF;
        renderCurrentScreen();
        return;
      }
      if (x >= 52 && x <= 188) {
        sndGenFreqStepIdx = (uint8_t)((sndGenFreqStepIdx + 1) % 4);
        renderCurrentScreen();
        return;
      }
    }

    // Volume SpinBox (y = 74..112, step 5%)
    if (y >= 74 && y <= 112) {
      if (x >= 8 && x <= 50) {
        int16_t nextV = (int16_t)sndGenVolumePct - 5;
        if (nextV < 0) nextV = 0;
        sndGenVolumePct = (uint8_t)nextV;
        renderCurrentScreen();
        return;
      }
      if (x >= 190 && x <= 232) {
        int16_t nextV = (int16_t)sndGenVolumePct + 5;
        if (nextV > 100) nextV = 100;
        sndGenVolumePct = (uint8_t)nextV;
        renderCurrentScreen();
        return;
      }
    }

    // 6 waveform selection buttons (2 rows * 3 cols, y = 116..184)
    if (y >= 116 && y <= 184 && x >= 8 && x <= 232) {
      uint8_t row = (y >= 150) ? 1 : 0;
      uint8_t col = 0;
      if (x >= 160) col = 2;
      else if (x >= 84) col = 1;
      sndGenWave = (SoundWaveType)(row * 3 + col);
      renderCurrentScreen();
      return;
    }

    // Sound ON/OFF toggle button (y = 188..236)
    if (x >= 8 && x <= 232 && y >= 188 && y <= 236) {
      if (sndGenPlaying) {
        stopSoundGenerator();
      } else {
        startSoundGenerator();
      }
      renderCurrentScreen();
      return;
    }
  }
  else if (currentScreen == SCREEN_HOSTS_SCAN) {
    // Top-left X -> stop scan and return to main menu
    if (x <= 44 && y <= 34) {
      stopHostsScan();
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }

    if (hostsView == HOSTS_VIEW_LIST) {
      // Rescan button in top-right corner
      if (x >= 196 && y <= 34) {
        startHostsScan();
        renderCurrentScreen();
        return;
      }
      // Scroll up (top-right triangle)
      if (x >= 196 && y >= 32 && y <= 94) {
        if (hostsScrollOffset > 0) {
          hostsScrollOffset--;
          renderCurrentScreen();
        }
        return;
      }
      // Scroll down (bottom-right triangle)
      if (x >= 196 && y >= 178 && y <= 239) {
        if (hostsScrollOffset + HOSTS_VISIBLE_ROWS < lanHostCount) {
          hostsScrollOffset++;
          renderCurrentScreen();
        }
        return;
      }
      // Select host -> probe details and show host info
      if (x >= 4 && x <= 195 && y >= 34 && y < 34 + HOSTS_VISIBLE_ROWS * 41) {
        int16_t row = (int16_t)((y - 34) / 41);
        int16_t idx = (int16_t)(hostsScrollOffset + row);
        if (idx >= 0 && idx < lanHostCount) {
          hostsSelectedIdx = idx;
          probeSelectedHostDetails(idx);
          hostsView = HOSTS_VIEW_DETAIL;
          renderCurrentScreen();
        }
        return;
      }
    } else if (hostsView == HOSTS_VIEW_DETAIL) {
      if (x >= 52 && x <= 188 && y >= 186 && y <= 236) {
        hostsView = HOSTS_VIEW_LIST;
        hostsSelectedIdx = -1;
        renderCurrentScreen();
        return;
      }
    }
  }
  else if (currentScreen == SCREEN_BT_SCAN) {
    // Top-left X -> stop BLE scan and return to main menu
    if (x <= 44 && y <= 34) {
      stopBluetoothScan();
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }

    if (btView == BT_VIEW_LIST) {
      // BLE rescan button in top-right corner (vibration silenced)
      if (x >= 196 && y <= 34) {
        stopTouchVibrationNow();
        requestBluetoothScan();
        renderCurrentScreen();
        return;
      }
      // Scroll up (top-right triangle)
      if (x >= 196 && y >= 32 && y <= 94) {
        if (btScrollOffset > 0) {
          btScrollOffset--;
          renderCurrentScreen();
        }
        return;
      }
      // Scroll down (bottom-right triangle)
      if (x >= 196 && y >= 178 && y <= 239) {
        if (btScrollOffset + BT_VISIBLE_ROWS < btDeviceCount) {
          btScrollOffset++;
          renderCurrentScreen();
        }
        return;
      }
      // Select Bluetooth device -> show device details
      if (x >= 4 && x <= 195 && y >= 34 && y < 34 + BT_VISIBLE_ROWS * 41) {
        int16_t row = (int16_t)((y - 34) / 41);
        int16_t idx = (int16_t)(btScrollOffset + row);
        if (idx >= 0 && idx < btDeviceCount) {
          btSelectedIdx = idx;
          btView = BT_VIEW_DETAIL;
          renderCurrentScreen();
        }
        return;
      }
    } else if (btView == BT_VIEW_DETAIL) {
      if (x >= 52 && x <= 188 && y >= 186 && y <= 236) {
        btView = BT_VIEW_LIST;
        btSelectedIdx = -1;
        renderCurrentScreen();
        return;
      }
    }
  }
  else if (currentScreen == SCREEN_WEATHER) {
    // Top-left X -> return to main menu
    if (x <= 44 && y <= 34) {
      weatherFetchPending = false;
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }

    if (weatherView == WEATHER_VIEW_CITIES) {
      // Back "<" button in top-right corner
      if (x >= 196 && y <= 34) {
        weatherView = weatherData.valid ? WEATHER_VIEW_CURRENT : WEATHER_VIEW_ERROR;
        renderCurrentScreen();
        return;
      }
      // Scroll up
      if (x >= 196 && y >= 34 && y <= 94) {
        if (weatherCityScroll > 0) {
          weatherCityScroll--;
          renderCurrentScreen();
        }
        return;
      }
      // Scroll down
      if (x >= 196 && y >= 178 && y <= 239) {
        if (weatherCityScroll + WEATHER_CITIES_VISIBLE < WORLD_CAPITALS_COUNT) {
          weatherCityScroll++;
          renderCurrentScreen();
        }
        return;
      }
      // Select capital from list (switch to manual mode and fetch weather immediately)
      if (x >= 4 && x <= 195 && y >= 34 && y < 34 + WEATHER_CITIES_VISIBLE * 41) {
        int16_t row = (int16_t)((y - 34) / 41);
        int16_t idx = (int16_t)(weatherCityScroll + row);
        if (idx >= 0 && idx < WORLD_CAPITALS_COUNT) {
          weatherCityIdx = (uint8_t)idx;
          weatherAutoLocate = false;
          saveWatchSettings();
          requestWeatherFetch();
          renderCurrentScreen();
        }
        return;
      }
      return;
    }

    // Top-right button -> toggle between current weather and 7-day forecast
    if (x >= 196 && y <= 34 && weatherData.valid) {
      weatherView = (weatherView == WEATHER_VIEW_CURRENT) ? WEATHER_VIEW_FORECAST : WEATHER_VIEW_CURRENT;
      renderCurrentScreen();
      return;
    }

    // Retry request on error screen
    if (weatherView == WEATHER_VIEW_ERROR && y >= 46 && y <= 180) {
      requestWeatherFetch();
      renderCurrentScreen();
      return;
    }

    // Bottom Auto/Manual and Capital list buttons available on Current / Error / Loading views
    if (weatherView != WEATHER_VIEW_FORECAST && y >= 188 && y <= 238) {
      if (x >= 4 && x <= 118) {
        weatherAutoLocate = !weatherAutoLocate;
        saveWatchSettings();
        requestWeatherFetch();
        renderCurrentScreen();
        return;
      }
      if (x >= 122 && x <= 236) {
        weatherFetchPending = false;
        int16_t targetScroll = (int16_t)weatherCityIdx - 2;
        if (targetScroll < 0) targetScroll = 0;
        if (targetScroll > WORLD_CAPITALS_COUNT - WEATHER_CITIES_VISIBLE) {
          targetScroll = WORLD_CAPITALS_COUNT - WEATHER_CITIES_VISIBLE;
        }
        weatherCityScroll = targetScroll;
        weatherView = WEATHER_VIEW_CITIES;
        renderCurrentScreen();
        return;
      }
    }
  }
  else if (currentScreen == SCREEN_SUN_ACTIVITY) {
    // Top-left X -> return to main menu
    if (x <= 44 && y <= 34) {
      sunFetchPending = false;
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }

    // Top-right button -> cycle screens: Now -> 3d -> Flux/Dst -> Now
    if (x >= 196 && y <= 34 && sunData.valid) {
      if (sunView == SUN_VIEW_CURRENT) sunView = SUN_VIEW_FORECAST;
      else if (sunView == SUN_VIEW_FORECAST) sunView = SUN_VIEW_FLUX_DST;
      else sunView = SUN_VIEW_CURRENT;
      renderCurrentScreen();
      return;
    }

    // Retry request on error screen
    if (sunView == SUN_VIEW_ERROR && y >= 58 && y <= 192) {
      requestSunActivityFetch();
      renderCurrentScreen();
      return;
    }

    // REFRESH button at bottom of current solar activity screen
    if (sunView == SUN_VIEW_CURRENT && x >= 156 && x <= 236 && y >= 204 && y <= 238) {
      requestSunActivityFetch();
      renderCurrentScreen();
      return;
    }
  }
  else if (currentScreen == SCREEN_SYSTEM_INFO) {
    // Top-left X -> return to main menu
    if (x <= 44 && y <= 34) {
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }

    uint8_t maxOffset = (SYS_INFO_ROW_COUNT > SYS_INFO_VISIBLE_ROWS)
                          ? (uint8_t)(SYS_INFO_ROW_COUNT - SYS_INFO_VISIBLE_ROWS)
                          : 0;
    // Scroll UP button on top-right
    if (x >= 196 && x <= 239 && y >= 32 && y <= 100) {
      if (sysInfoScrollOffset > 0) {
        sysInfoScrollOffset--;
        renderCurrentScreen();
      }
      return;
    }
    // Scroll DOWN button on bottom-right
    if (x >= 196 && x <= 239 && y >= 174 && y <= 239) {
      if (sysInfoScrollOffset < maxOffset) {
        sysInfoScrollOffset++;
        renderCurrentScreen();
      }
      return;
    }
  }
  else if (currentScreen == SCREEN_ALARM) {
    // Top-left X -> return to main menu
    if (x <= 44 && y <= 34) {
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }

    // Switch between Alarm 1 and Alarm 2
    if (y >= 30 && y <= 58) {
      if (x >= 6 && x <= 118) {
        selectedAlarmIdx = 0;
        renderCurrentScreen();
        return;
      }
      if (x >= 122 && x <= 234) {
        selectedAlarmIdx = 1;
        renderCurrentScreen();
        return;
      }
    }

    AlarmEntry &al = alarms[selectedAlarmIdx];

    // Select time field (HH or MM) and < / > arrows (Clock style)
    if (y >= 60 && y <= 112) {
      if (x >= 6 && x <= 56) {
        if (alarmEditField == 0) {
          al.hour = (uint8_t)((al.hour + 23) % 24);
        } else {
          al.minute = (uint8_t)((al.minute + 59) % 60);
        }
        al.snoozePending = false;
        al.lastTriggeredMin = -1;
        saveWatchSettings();
        renderCurrentScreen();
        return;
      }
      if (x >= 184 && x <= 234) {
        if (alarmEditField == 0) {
          al.hour = (uint8_t)((al.hour + 1) % 24);
        } else {
          al.minute = (uint8_t)((al.minute + 1) % 60);
        }
        al.snoozePending = false;
        al.lastTriggeredMin = -1;
        saveWatchSettings();
        renderCurrentScreen();
        return;
      }
      if (x >= 58 && x < 120) {
        alarmEditField = 0;
        renderCurrentScreen();
        return;
      }
      if (x >= 120 && x <= 182) {
        alarmEditField = 1;
        renderCurrentScreen();
        return;
      }
    }

    // ON / OFF toggle
    if (x >= 8 && x <= 232 && y >= 115 && y <= 152) {
      al.enabled = !al.enabled;
      al.snoozePending = false;
      al.lastTriggeredMin = -1;
      saveWatchSettings();
      renderCurrentScreen();
      return;
    }

    // Ringtone selector: Beep / Bell
    if (x >= 8 && x <= 232 && y >= 155 && y <= 192) {
      al.ringtone = (al.ringtone == RINGTONE_BEEP) ? RINGTONE_BELL : RINGTONE_BEEP;
      saveWatchSettings();
      renderCurrentScreen();
      return;
    }

    // Snooze duration selector (5 -> 10 -> 15 -> 20 -> 30 min)
    if (x >= 8 && x <= 232 && y >= 195 && y <= 234) {
      uint8_t nextIdx = 0;
      for (uint8_t k = 0; k < ALARM_SNOOZE_OPT_COUNT; k++) {
        if (ALARM_SNOOZE_OPTIONS[k] == al.snoozeMin) {
          nextIdx = (uint8_t)((k + 1) % ALARM_SNOOZE_OPT_COUNT);
          break;
        }
      }
      al.snoozeMin = ALARM_SNOOZE_OPTIONS[nextIdx];
      saveWatchSettings();
      renderCurrentScreen();
      return;
    }
  }
  else if (currentScreen == SCREEN_STOPWATCH) {
    // Top-left X -> return to main menu
    if (x <= 44 && y <= 34) {
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }

    // Start / Stop / Reset buttons
    if (y >= 164 && y <= 230) {
      if (x >= 6 && x <= 81) {
        if (!swRunning) {
          swRunning    = true;
          swStartMs    = millis();
          lastSwDrawMs = swStartMs;
          renderCurrentScreen();
        }
        return;
      }
      if (x >= 82 && x <= 157) {
        if (swRunning) {
          swAccumulatedMs += (millis() - swStartMs);
          swRunning = false;
          renderCurrentScreen();
        }
        return;
      }
      if (x >= 158 && x <= 234) {
        swRunning       = false;
        swAccumulatedMs = 0;
        swStartMs       = millis();
        renderCurrentScreen();
        return;
      }
    }
  }
  else if (currentScreen == SCREEN_TIMER) {
    // Top-left X -> return to main menu
    if (x <= 44 && y <= 34) {
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }

    bool isIdle = (!timerRunning && !timerPaused);

    // Select HH / MM / SS field by tapping (Clock style)
    if (isIdle && y >= 50 && y <= 108) {
      if (x >= 8 && x < 85) {
        timerEditField = 0;
        renderCurrentScreen();
        return;
      }
      if (x >= 85 && x < 155) {
        timerEditField = 1;
        renderCurrentScreen();
        return;
      }
      if (x >= 155 && x <= 232) {
        timerEditField = 2;
        renderCurrentScreen();
        return;
      }
    }

    // Middle row: < / > arrows (set time) and Beep / Bell ringtone selector
    if (y >= 112 && y <= 152) {
      if (isIdle && x >= 6 && x <= 60) {
        if (timerEditField == 0) timerSetH = (uint8_t)((timerSetH + 23) % 24);
        else if (timerEditField == 1) timerSetM = (uint8_t)((timerSetM + 59) % 60);
        else timerSetS = (uint8_t)((timerSetS + 59) % 60);
        timerRemainingMs = getTimerSetTotalMs();
        renderCurrentScreen();
        return;
      }
      if (isIdle && x >= 180 && x <= 234) {
        if (timerEditField == 0) timerSetH = (uint8_t)((timerSetH + 1) % 24);
        else if (timerEditField == 1) timerSetM = (uint8_t)((timerSetM + 1) % 60);
        else timerSetS = (uint8_t)((timerSetS + 1) % 60);
        timerRemainingMs = getTimerSetTotalMs();
        renderCurrentScreen();
        return;
      }
      if (x >= 62 && x <= 178) {
        timerRingtone = (timerRingtone == RINGTONE_BEEP) ? RINGTONE_BELL : RINGTONE_BEEP;
        saveWatchSettings();
        renderCurrentScreen();
        return;
      }
    }

    // Bottom row: Start / Pause / Reset
    if (y >= 158 && y <= 224) {
      if (x >= 6 && x <= 81) {
        if (!timerRunning) {
          uint32_t startMs = timerPaused ? timerRemainingMs : getTimerSetTotalMs();
          if (startMs > 0) {
            timerRemainingMs = startMs;
            timerEndMs       = millis() + startMs;
            timerRunning     = true;
            timerPaused      = false;
            lastTimerDrawMs  = millis();
            renderCurrentScreen();
          }
        }
        return;
      }
      if (x >= 82 && x <= 157) {
        if (timerRunning) {
          timerRemainingMs = getTimerDisplayRemainingMs();
          timerRunning     = false;
          timerPaused      = true;
          renderCurrentScreen();
        }
        return;
      }
      if (x >= 158 && x <= 234) {
        timerRunning     = false;
        timerPaused      = false;
        timerRemainingMs = getTimerSetTotalMs();
        renderCurrentScreen();
        return;
      }
    }
  }
  else if (currentScreen == SCREEN_FLASHLIGHT) {
    // Top-left X -> turn off flashlight, restore user brightness, exit to main menu
    if (x <= 44 && y <= 34) {
      flashlightOn             = false;
      flashlightSliderDragging = false;
      tft.setBrightness(getFullBrightnessPwm());
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }
    // Brightness slider (x = 10..230, y = 126..160)
    if (x >= 10 && x <= 230 && y >= 126 && y <= 160) {
      int32_t pct = (int32_t)BRIGHTNESS_MIN_PCT + ((int32_t)(x - 22) * (100L - (int32_t)BRIGHTNESS_MIN_PCT) + 98L) / 196L;
      if (pct < BRIGHTNESS_MIN_PCT) pct = BRIGHTNESS_MIN_PCT;
      if (pct > 100) pct = 100;
      flashlightBrightness     = (uint8_t)pct;
      flashlightSliderDragging = true;
      if (flashlightOn) {
        tft.setBrightness(getBrightnessPwmFromPercent(flashlightBrightness));
      }
      renderCurrentScreen();
      return;
    }
    // Color selector button (y = 162..195)
    if (x >= 10 && x <= 230 && y >= 162 && y <= 195) {
      flashlightColorIdx = (uint8_t)((flashlightColorIdx + 1) % FLASHLIGHT_COLOR_COUNT);
      renderCurrentScreen();
      return;
    }
    // ON / OFF toggle button (y = 196..236)
    if (x >= 10 && x <= 230 && y >= 196 && y <= 236) {
      flashlightOn = !flashlightOn;
      if (flashlightOn) {
        tft.setBrightness(getBrightnessPwmFromPercent(flashlightBrightness));
      } else {
        tft.setBrightness(getFullBrightnessPwm());
      }
      renderCurrentScreen();
      return;
    }
  }
  else if (currentScreen == SCREEN_SOS) {
    // Top-left X -> stop SOS audio, strobe, and vibration, exit to main menu
    if (x <= 44 && y <= 34) {
      stopSosSignal();
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }
  }
  else if (currentScreen == SCREEN_MORSE) {
    // Top-left X -> stop Morse playback and exit to main menu
    if (x <= 44 && y <= 34) {
      stopMorsePlayback();
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }
    // Top-right quick section jump button (EN -> RU -> 0-9 -> SYM -> EN)
    if (x >= 194 && y <= 32) {
      if (morseScrollOffset >= MORSE_SYM_OFFSET) {
        morseScrollOffset = 0;
      } else if (morseScrollOffset >= MORSE_NUM_OFFSET) {
        morseScrollOffset = MORSE_SYM_OFFSET;
      } else if (morseScrollOffset >= MORSE_EN_COUNT) {
        morseScrollOffset = MORSE_NUM_OFFSET;
      } else {
        morseScrollOffset = MORSE_EN_COUNT;
      }
      renderCurrentScreen();
      return;
    }
    // Right UP scroll triangle
    if (x >= 196 && y >= 32 && y <= 94) {
      if (morseScrollOffset >= MORSE_VISIBLE_ROWS) {
        morseScrollOffset = (uint8_t)(morseScrollOffset - MORSE_VISIBLE_ROWS);
      } else {
        morseScrollOffset = 0;
      }
      renderCurrentScreen();
      return;
    }
    // Right DOWN scroll triangle
    if (x >= 196 && y >= 178 && y <= 239) {
      uint8_t maxOff = (MORSE_TOTAL_COUNT > MORSE_VISIBLE_ROWS)
                         ? (uint8_t)(MORSE_TOTAL_COUNT - MORSE_VISIBLE_ROWS)
                         : 0;
      if ((uint16_t)morseScrollOffset + MORSE_VISIBLE_ROWS <= maxOff) {
        morseScrollOffset = (uint8_t)(morseScrollOffset + MORSE_VISIBLE_ROWS);
      } else {
        morseScrollOffset = maxOff;
      }
      renderCurrentScreen();
      return;
    }
    // Tap any visible table row (x = 6..194, y = 50..232) -> play its Morse code
    if (x >= 6 && x <= 194 && y >= 50 && y < 50 + MORSE_VISIBLE_ROWS * 26) {
      uint8_t row = (uint8_t)((y - 50) / 26);
      uint8_t idx = (uint8_t)(morseScrollOffset + row);
      if (idx < MORSE_TOTAL_COUNT) {
        startMorsePlayback(idx);
        renderCurrentScreen();
        return;
      }
    }
  }
  else if (currentScreen == SCREEN_TETRIS) {
    // Top-left X -> return to main menu
    if (x <= 44 && y <= 34) {
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }
    // Top-right Restart button
    if (x >= 194 && y <= 34) {
      tetrisResetGame();
      renderCurrentScreen();
      return;
    }
    // Game Over dialog OK button -> restart game
    if (tetrisGameOver) {
      if (x >= 50 && x <= 190 && y >= 124 && y <= 178) {
        tetrisResetGame();
        renderCurrentScreen();
      }
      return;
    }
    // Bottom 5 buttons (y >= 192): Left, Right, Down, Rotate, Pause
    if (y >= 192) {
      if (x < 48) {
        tetrisMoveHoriz(-1);
      } else if (x < 96) {
        tetrisMoveHoriz(1);
      } else if (x < 144) {
        tetrisStepDown(true);
      } else if (x < 192) {
        tetrisRotateCW();
      } else {
        tetrisPaused = !tetrisPaused;
        lastTetrisDropMs = millis();
      }
      renderCurrentScreen();
      return;
    }
  }
  else if (currentScreen == SCREEN_PAINT) {
    // Top-left X -> return to main menu
    if (x <= 44 && y <= 34) {
      paintDrawing = false;
      paintLastX = -1;
      paintLastY = -1;
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }
    // Top-right CLR button -> clear canvas
    if (x >= 194 && y <= 34) {
      paintDrawing = false;
      paintClearCanvas();
      renderCurrentScreen();
      return;
    }
    // Bottom toolbar (y >= 200): Color, BG, Size, Brush, Save
    if (y >= 200) {
      paintDrawing = false;
      if (x < 48) {
        paintColorIdx = (paintColorIdx + 1) % PAINT_COLOR_COUNT;
      } else if (x < 96) {
        paintBgIdx = (paintBgIdx + 1) % PAINT_BG_COUNT;
      } else if (x < 144) {
        paintSizeIdx = (paintSizeIdx + 1) % PAINT_SIZE_COUNT;
      } else if (x < 192) {
        paintBrushIdx = (paintBrushIdx + 1) % PAINT_BRUSH_COUNT;
      } else {
        savePaintToFlash();
        paintSaveBannerMs = millis();
      }
      renderCurrentScreen();
      return;
    }
    // Drawing canvas area (y = 28..199)
    if (y >= PAINT_CANVAS_Y && y < PAINT_CANVAS_Y + PAINT_CANVAS_H) {
      paintDrawing = true;
      paintLastX = x;
      paintLastY = y;
      paintSaveBannerMs = 0;
      paintDrawSegment(x, y, x, y);
      renderCurrentScreen();
      return;
    }
  }
  else if (currentScreen == SCREEN_ACCEL_SPEC) {
    // Top-left X -> stop 200 Hz capture (restore 50 Hz BMA423) and return to main menu
    if (x <= 44 && y <= 34) {
      stopAccelSpectrometer();
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }
    // Top-right axis badge -> cycle MAG / X / Y / Z and clear Peak Hold
    if (x >= 194 && y <= 34) {
      accelSpecAxis = (uint8_t)((accelSpecAxis + 1U) % 4U);
      memset(accelSpecPeak, 0, sizeof(accelSpecPeak));
      memset(accelSpecRing, 0, sizeof(accelSpecRing));
      accelSpecRingPos = 0;
      accelSpecSampleCount = 0;
      renderCurrentScreen();
      return;
    }
  }
  else if (currentScreen == SCREEN_BT_KEYBOARD) {
    // Top-left X -> disconnect and disable Bluetooth HID, then return to main menu
    if (x <= 44 && y <= 34) {
      stopBleHidKeyboard();
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }
    // Top-right page badge -> cycle 8 keyboard screens (1/8..8/8)
    if (x >= 196 && y <= 34) {
      btKbPage = (uint8_t)((btKbPage + 1U) % BT_KB_PAGE_COUNT);
      renderCurrentScreen();
      return;
    }
    // Sticky modifier chips inside header bar (y = 46..66): Ctrl | Shift | Alt | Win
    if (y >= 46 && y <= 66) {
      if (x >= 8 && x <= 63) {
        btKbMods ^= BT_KB_MOD_CTRL;
      } else if (x >= 64 && x <= 119) {
        btKbMods ^= BT_KB_MOD_SHIFT;
      } else if (x >= 120 && x <= 175) {
        btKbMods ^= BT_KB_MOD_ALT;
      } else if (x >= 176 && x <= 232) {
        if (btKbMods & BT_KB_MOD_GUI) {
          // Second tap on armed Win chip sends standalone Left GUI key (Start / Spotlight)
          sendBleHidKeyReport(BT_KB_MOD_GUI, 0x00, "Win");
          btKbMods = (uint8_t)(btKbMods & ~BT_KB_MOD_GUI);
        } else {
          btKbMods |= BT_KB_MOD_GUI;
        }
      }
      renderCurrentScreen();
      return;
    }
    // Bottom control row (y = 194..236): Shift | Space | <- | Enter
    if (y >= 194 && y <= 236) {
      if (x >= 6 && x <= 60) {
        btKbMods ^= BT_KB_MOD_SHIFT;
        renderCurrentScreen();
        return;
      }
      if (x >= 62 && x <= 117) {
        sendBleHidKeyReport(btKbMods, 0x2C, "Space");
        btKbMods = 0;
        renderCurrentScreen();
        return;
      }
      if (x >= 119 && x <= 174) {
        sendBleHidKeyReport(btKbMods, 0x2A, "Bksp");
        btKbMods = 0;
        renderCurrentScreen();
        return;
      }
      if (x >= 176 && x <= 234) {
        sendBleHidKeyReport(btKbMods, 0x28, "Enter");
        btKbMods = 0;
        renderCurrentScreen();
        return;
      }
    }
    // Main key grid (y = 68..193)
    if (y >= 68 && y < 194) {
      int8_t row = constrain((int8_t)((y - 69) / 42), (int8_t)0, (int8_t)2);
      uint8_t keyCount = 0;
      const BtKbKeyDef* keys = getBtKbPageKeys(btKbPage, keyCount);
      int8_t idx = -1;
      if (btKbPage == 1 && row == 2) {
        int8_t col = constrain((int8_t)((x - 6) / 77), (int8_t)0, (int8_t)2);
        idx = (int8_t)(8 + col);
      } else {
        int8_t col = constrain((int8_t)((x - 6) / 57), (int8_t)0, (int8_t)3);
        idx = (int8_t)(row * 4 + col);
      }
      if (idx >= 0 && idx < (int8_t)keyCount) {
        const BtKbKeyDef& k = keys[idx];
        const char* dispLbl = getBtKbKeyDisplayLabel(k);
        sendBleHidKeyReport(btKbMods, k.hidCode, dispLbl);
        btKbMods = 0;
        renderCurrentScreen();
      }
      return;
    }
  }
  else if (currentScreen == SCREEN_BT_PRESENTER) {
    // Top-left X -> disconnect and disable Bluetooth HID Presenter, then return to main menu
    if (x <= 44 && y <= 34) {
      stopBleHidKeyboard();
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }
    // Check 9 single-screen presenter buttons
    for (uint8_t i = 0; i < BT_PRES_BTN_COUNT; i++) {
      const BtPresenterBtn& b = BT_PRESENTER_BTNS[i];
      if (x >= b.x && x < (b.x + b.w) && y >= b.y && y < (b.y + b.h)) {
        sendBleHidKeyReport(b.mods, b.hidCode, b.sentLabel);
        renderCurrentScreen();
        return;
      }
    }
  }
  else if (currentScreen == SCREEN_COLOR_TABLE) {
    // Top-left X -> return to main menu
    if (x <= 44 && y <= 34) {
      colorTableSelected = -1;
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }
    // Top-right RGB / CMYK toggle button
    if (x >= 194 && y <= 34) {
      colorTableCmykMode = !colorTableCmykMode;
      renderCurrentScreen();
      return;
    }
    // Detail modal dismiss (OK button or tap inside modal)
    if (colorTableSelected >= 0) {
      if (y >= 184 || x < 10 || x > 230) {
        colorTableSelected = -1;
        renderCurrentScreen();
      }
      return;
    }
    // Right scrollbar Up / Down buttons
    if (x >= 204 && y >= 46 && y <= 104) {
      if (colorTableScroll > 0) {
        colorTableScroll -= 3;
        if (colorTableScroll < 0) colorTableScroll = 0;
        renderCurrentScreen();
      }
      return;
    }
    if (x >= 204 && y >= 178 && y <= 239) {
      int16_t maxScroll = COLOR_TABLE_COUNT - COLOR_TABLE_VISIBLE;
      if (colorTableScroll < maxScroll) {
        colorTableScroll += 3;
        if (colorTableScroll > maxScroll) colorTableScroll = maxScroll;
        renderCurrentScreen();
      }
      return;
    }
    // Row tap -> open full color detail modal
    if (x >= 6 && x <= 203 && y >= 48 && y < 48 + COLOR_TABLE_VISIBLE * 31) {
      int16_t row = (int16_t)((y - 48) / 31);
      int16_t idx = (int16_t)(colorTableScroll + row);
      if (idx >= 0 && idx < COLOR_TABLE_COUNT) {
        colorTableSelected = idx;
        renderCurrentScreen();
      }
      return;
    }
  }
  else if (currentScreen == SCREEN_RESISTOR_TABLE) {
    // Top-left X -> return to main menu
    if (x <= 44 && y <= 34) {
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }
    // Top-right 4B / 5B toggle button
    if (x >= 196 && y <= 34) {
      resistorFiveBand = !resistorFiveBand;
      if (!resistorFiveBand && resistorSelectedBand >= 4) {
        resistorSelectedBand = 3;
      }
      renderCurrentScreen();
      return;
    }
    // Band selector tap on resistor card (y = 46..89)
    if (y >= 46 && y <= 89 && x >= 10 && x <= 230) {
      if (resistorFiveBand) {
        int8_t b = constrain((int8_t)((x - 12) / 44), (int8_t)0, (int8_t)4);
        resistorSelectedBand = (uint8_t)b;
      } else {
        int8_t b = constrain((int8_t)((x - 12) / 55), (int8_t)0, (int8_t)3);
        resistorSelectedBand = (uint8_t)b;
      }
      renderCurrentScreen();
      return;
    }
    // Right scrollbar Up / Down buttons
    if (x >= 204 && y >= 105 && y <= 148) {
      if (resistorTableScroll > 0) {
        resistorTableScroll -= 3;
        if (resistorTableScroll < 0) resistorTableScroll = 0;
        renderCurrentScreen();
      }
      return;
    }
    if (x >= 204 && y >= 192 && y <= 239) {
      int16_t maxScroll = RES_COLOR_COUNT - RES_TABLE_VISIBLE;
      if (resistorTableScroll < maxScroll) {
        resistorTableScroll += 3;
        if (resistorTableScroll > maxScroll) resistorTableScroll = maxScroll;
        renderCurrentScreen();
      }
      return;
    }
    // Table row tap -> assign color to selected resistor band if valid per IEC 60062
    if (x >= 6 && x <= 203 && y >= 107 && y < 107 + RES_TABLE_VISIBLE * 21) {
      int16_t row = (int16_t)((y - 107) / 21);
      int16_t idx = (int16_t)(resistorTableScroll + row);
      if (idx >= 0 && idx < RES_COLOR_COUNT &&
          isResistorColorValidForSlot((uint8_t)idx, resistorSelectedBand, resistorFiveBand)) {
        if (resistorFiveBand) {
          resistorBands[resistorSelectedBand] = (uint8_t)idx;
        } else {
          uint8_t storageSlot = (resistorSelectedBand < 2) ? resistorSelectedBand : (uint8_t)(resistorSelectedBand + 1);
          resistorBands[storageSlot] = (uint8_t)idx;
        }
        renderCurrentScreen();
      }
      return;
    }
  }
  else if (currentScreen == SCREEN_VOLTAGE_DIVIDER) {
    // Top-left X -> return to main menu
    if (x <= 44 && y <= 34) {
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }
    // 1. Vin SpinBox (y = 110..151)
    if (y >= 110 && y <= 151) {
      int32_t stepMv = VDIV_VIN_STEPS_MV[vdivVinStepIdx];
      if (x <= 50) {
        vdivVinMv -= stepMv;
        if (vdivVinMv < VDIV_VIN_MIN_MV) vdivVinMv = VDIV_VIN_MIN_MV;
        renderCurrentScreen();
      } else if (x >= 190) {
        vdivVinMv += stepMv;
        if (vdivVinMv > VDIV_VIN_MAX_MV) vdivVinMv = VDIV_VIN_MAX_MV;
        renderCurrentScreen();
      } else {
        vdivVinStepIdx = (uint8_t)((vdivVinStepIdx + 1) % 4);
        renderCurrentScreen();
      }
      return;
    }
    // 2. R1 SpinBox (y = 152..193)
    if (y >= 152 && y <= 193) {
      if (x <= 50) {
        stepVdivResistor(vdivR1ValIdx, vdivR1Unit, -1);
        renderCurrentScreen();
      } else if (x >= 190) {
        stepVdivResistor(vdivR1ValIdx, vdivR1Unit, 1);
        renderCurrentScreen();
      } else {
        vdivR1Unit = (uint8_t)((vdivR1Unit + 1) % 3);
        renderCurrentScreen();
      }
      return;
    }
    // 3. R2 SpinBox (y = 194..238)
    if (y >= 194 && y <= 238) {
      if (x <= 50) {
        stepVdivResistor(vdivR2ValIdx, vdivR2Unit, -1);
        renderCurrentScreen();
      } else if (x >= 190) {
        stepVdivResistor(vdivR2ValIdx, vdivR2Unit, 1);
        renderCurrentScreen();
      } else {
        vdivR2Unit = (uint8_t)((vdivR2Unit + 1) % 3);
        renderCurrentScreen();
      }
      return;
    }
  }
  else if (currentScreen == SCREEN_BT_MOUSE) {
    // Top-left X -> disconnect and disable Bluetooth HID Mouse, then return to main menu
    if (x <= 44 && y <= 34) {
      stopBleHidMouse();
      currentScreen = SCREEN_MAIN_MENU;
      renderCurrentScreen();
      return;
    }
    // Top-right ZERO badge (or tapping the tilt radar) -> re-center BMA423 neutral tilt
    if ((x >= 196 && y <= 34) || (x >= 198 && y >= 31 && y <= 66)) {
      recenterBtMouseTilt();
      renderCurrentScreen();
      return;
    }
    // Left column (x = 6..84): Single Left Click (top) and Double Left Click (bottom)
    if (x >= 4 && x <= 86 && y >= 68 && y <= 151) {
      sendBleHidMouseClick(BT_MOUSE_BTN_LEFT, "1x Left");
      renderCurrentScreen();
      return;
    }
    if (x >= 4 && x <= 86 && y >= 152 && y <= 236) {
      sendBleHidMouseDoubleClickLeft();
      renderCurrentScreen();
      return;
    }
    // Center column (x = 88..152, y = 68..236): Interactive Scroll Wheel (drag or tap up/down)
    if (x >= 88 && x <= 152 && y >= 68 && y <= 236) {
      btMouseWheelDragging     = true;
      btMouseWheelLastY        = y;
      btMouseWheelAccumY       = 0;
      btMouseWheelDraggedSteps = false;
      if (y <= 104) {
        sendBleHidMouseWheel(1);
        btMouseWheelDraggedSteps = true;
      } else if (y >= 199) {
        sendBleHidMouseWheel(-1);
        btMouseWheelDraggedSteps = true;
      }
      renderCurrentScreen();
      return;
    }
    // Right column (x = 154..236): Single Right Click (top) and Single Middle Click (bottom)
    if (x >= 154 && x <= 236 && y >= 68 && y <= 151) {
      sendBleHidMouseClick(BT_MOUSE_BTN_RIGHT, "1x Right");
      renderCurrentScreen();
      return;
    }
    if (x >= 154 && x <= 236 && y >= 152 && y <= 236) {
      sendBleHidMouseClick(BT_MOUSE_BTN_MIDDLE, "1x Mid");
      renderCurrentScreen();
      return;
    }
  }
}

// ============================================================================
// 9. BACKLIGHT, LIGHT SLEEP POWER MANAGEMENT, AND TOUCHSCREEN
// ============================================================================

// Put ST7789V display controller into sleep mode (DISPOFF + SLPIN)
void putDisplayToSleep() {
  tft.setBrightness(BRIGHTNESS_OFF);
  if (!displaySleeping) {
    tft.sleep();
    displaySleeping = true;
  }
}

// Wake ST7789V display controller from sleep mode (SLPOUT + DISPON)
void wakeDisplayFromSleep() {
  if (displaySleeping) {
    tft.wakeup();
    delay(20);
    displaySleeping = false;
  }
}

// Power down Wi-Fi and Bluetooth radios cleanly before ESP32 Light Sleep
void disableRadiosForLightSleep() {
  if (bleHidKbActive) {
    stopBleHidKeyboard();
  }
  if (bleHidMouseActive) {
    stopBleHidMouse();
  }
  if (btScanRunning) {
    btScanRunning = false;
    btScanPending = false;
  }
  if (BLEDevice::getInitialized()) {
    BLEScan* pScan = BLEDevice::getScan();
    if (pScan != NULL) {
      pScan->stop();
      pScan->clearResults();
    }
    BLEDevice::stopAdvertising();
    BLEDevice::deinit(false);
    delay(20);
  }
  bleStackInitialized = false;
  btStop();

  wifiBgScanning = false;
  WiFi.scanDelete();
  WiFi.disconnect(true, false);
  esp_wifi_stop();
  WiFi.mode(WIFI_OFF);
  wifiInfoConnected = false;
}

// Compute microseconds until next pending Timer or Alarm check (0 if none active)
uint64_t computeNextAlertWakeUs() {
  uint32_t now = millis();
  uint64_t bestMs = 0;

  if (timerRunning) {
    int32_t rem = (int32_t)(timerEndMs - now);
    uint64_t remMs = (rem > 250) ? (uint64_t)rem : 250ULL;
    bestMs = remMs;
  }

  for (uint8_t i = 0; i < ALARM_COUNT; i++) {
    if (!alarms[i].enabled) continue;
    uint64_t candMs = 30000ULL; // Wake every 30 s to verify RTC minute match when an alarm is armed
    if (alarms[i].snoozePending) {
      int32_t rem = (int32_t)(alarms[i].snoozeTargetMs - now);
      candMs = (rem > 250) ? (uint64_t)rem : 250ULL;
    }
    if (bestMs == 0 || candMs < bestMs) {
      bestMs = candMs;
    }
  }

  return bestMs * 1000ULL;
}

// Enter ESP32-S3 Light Sleep in BL_STATE_OFF with display sleep, FT6336U Monitor Mode,
// Wi-Fi/BT off, USB Serial off, and AXP2101 DLDO1/ALDO4/BLDO2 off (preserving GPS BLDO1 if enabled)
void enterDisplayOffLightSleep() {
  stopTouchVibrationNow();
  saveStepCountToFlash(false);

  // 1. Put ST7789V display controller into sleep mode
  putDisplayToSleep();

  // 2. Disable Wi-Fi and Bluetooth completely for lowest Light Sleep current
  disableRadiosForLightSleep();

  // 3. Switch FT6336U into Monitor Mode (or Hibernate if touch wake is disabled)
  bool useTouchWake  = settingWakeOnTouch;
  bool useBmaWake    = settingWakeOnBma423;
  bool useButtonWake = settingWakeOnButton;
  if (!useTouchWake && !useBmaWake && !useButtonWake) {
    useButtonWake = true;
  }

  if (useTouchWake) {
    setTouchMonitorMode();
  } else {
    setTouchHibernateMode();
  }

  // 4. Configure BMA423 INT1 (IO14) and clear any latched status
  configureBma423WakeInterrupt();
  if (bma423Ready) {
    (void)readRegI2C(Wire, bma423Addr, BMA423_REG_INT_STATUS_0);
  }

  // 5. Power down AXP2101 DLDO1, ALDO4, BLDO2 rails (preserving GPS BLDO1 if enabled in Settings)
  applyAxp2101LightSleepRails();

  // 6. Disable USB Serial before Light Sleep
  Serial.flush();
  Serial.end();

  // 7. Configure GPIO Light Sleep wake-up lines (all active LOW with internal pull-up)
  gpio_wakeup_disable((gpio_num_t)TOUCH_INT_PIN);
  gpio_wakeup_disable((gpio_num_t)BMA423_INT_PIN);
  gpio_wakeup_disable((gpio_num_t)WAKE_BUTTON_PIN);

  if (useTouchWake) {
    pinMode(TOUCH_INT_PIN, INPUT_PULLUP);
    gpio_wakeup_enable((gpio_num_t)TOUCH_INT_PIN, GPIO_INTR_LOW_LEVEL);
  }
  if (useBmaWake) {
    pinMode(BMA423_INT_PIN, INPUT_PULLUP);
    gpio_wakeup_enable((gpio_num_t)BMA423_INT_PIN, GPIO_INTR_LOW_LEVEL);
  }
  if (useButtonWake) {
    pinMode(WAKE_BUTTON_PIN, INPUT_PULLUP);
    gpio_wakeup_enable((gpio_num_t)WAKE_BUTTON_PIN, GPIO_INTR_LOW_LEVEL);
  }
  esp_sleep_enable_gpio_wakeup();

  // 8. Sleep loop: remain in Light Sleep until a valid wake condition occurs
  bool wokeValid = false;
  while (!wokeValid) {
    // Wait briefly for any currently-held wake pin to release HIGH before arming LOW-level sleep
    uint32_t settleStart = millis();
    while (millis() - settleStart < 350UL) {
      bool anyLow = false;
      if (useTouchWake && digitalRead(TOUCH_INT_PIN) == LOW) anyLow = true;
      if (useButtonWake && digitalRead(WAKE_BUTTON_PIN) == LOW) anyLow = true;
      if (!anyLow) break;
      delay(5);
    }

    // Arm timer wakeup if a countdown Timer or Alarm is active
    uint64_t nextAlertUs = computeNextAlertWakeUs();
    if (nextAlertUs > 0) {
      esp_sleep_enable_timer_wakeup(nextAlertUs);
    } else {
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    }

    esp_light_sleep_start();

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_TIMER) {
      readRTC(rtcTime);
      updateAlarmAndTimer();
      if (activeAlertType != ALERT_NONE) {
        wokeValid = true;
      }
    }
    else if (cause == ESP_SLEEP_WAKEUP_GPIO) {
      if (useButtonWake && digitalRead(WAKE_BUTTON_PIN) == LOW) {
        delay(25);
        wokeValid = true;
      }
      else if (useBmaWake && digitalRead(BMA423_INT_PIN) == LOW) {
        if (bma423Ready) {
          (void)readRegI2C(Wire, bma423Addr, BMA423_REG_INT_STATUS_0);
        }
        wokeValid = true;
      }
      else if (useTouchWake && digitalRead(TOUCH_INT_PIN) == LOW) {
        uint8_t requiredTaps = (settingWakeTapCount == 3) ? 3 : 2;
        if (detectMultiTapOnTouchInt(requiredTaps)) {
          wokeValid = true;
        }
      }
    }
  }

  // 9. Restore active state, AXP2101 rails, DRV2605L, and USB Serial after valid wake-up
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
  gpio_wakeup_disable((gpio_num_t)TOUCH_INT_PIN);
  gpio_wakeup_disable((gpio_num_t)BMA423_INT_PIN);
  gpio_wakeup_disable((gpio_num_t)WAKE_BUTTON_PIN);

  applyAxp2101PowerSettings();
  delay(15);
  reinitDRV2605LAfterSleep();
  Serial.begin(115200);

  setTouchActiveMode();
  wakeDisplayFromSleep();

  // Sync RTC, battery, and BMA423 hardware pedometer accumulated during Light Sleep
  readRTC(rtcTime);
  batteryPercent  = readBatteryPercent();
  batteryCharging = readBatteryCharging();
  syncBma423HardwareSteps();

  blState          = BL_STATE_FULL;
  lastTouchMs      = millis();
  lastRtcReadMs    = lastTouchMs;
  wakeTapCounter   = 0;
  waitingSecondTap = false;
  touchWasPressed  = (digitalRead(TOUCH_INT_PIN) == LOW);

  renderCurrentScreen();
  tft.setBrightness(getFullBrightnessPwm());

  // Re-enable Wi-Fi STA for background auto-reconnect only when Wi-Fi is globally enabled
  if (settingWifiEnabled && settingWifiAutoConnect && savedWifiCount > 0) {
    WiFi.mode(WIFI_STA);
    lastWifiAutoCheckMs = 0;
  }
}

void updateTouchAndBacklight() {
  uint32_t now = millis();
  int16_t tx = 0, ty = 0;
  bool isPressed = getTouchPoint(tx, ty);

  // 1. Touch press edge (new contact)
  if (isPressed && !touchWasPressed) {
    touchWasPressed = true;
    prevTouchX = tx;
    prevTouchY = ty;

    if (blState == BL_STATE_OFF) {
      uint8_t requiredTaps = (settingWakeTapCount == 3) ? 3 : 2;
      if (wakeTapCounter == 0 || (now - lastWakeTapMs > DOUBLE_TAP_MAX_MS)) {
        wakeTapCounter = 1;
        firstWakeTapMs = now;
        lastWakeTapMs = now;
        waitingSecondTap = true;
      } else {
        uint32_t dt = now - lastWakeTapMs;
        if (dt >= DOUBLE_TAP_MIN_MS && dt <= DOUBLE_TAP_MAX_MS) {
          wakeTapCounter++;
          lastWakeTapMs = now;
          if (wakeTapCounter >= requiredTaps) {
            if (touchInMonitorMode) setTouchActiveMode();
            wakeDisplayFromSleep();
            blState = BL_STATE_FULL;
            lastTouchMs = now;
            wakeTapCounter = 0;
            waitingSecondTap = false;
            renderCurrentScreen();
            tft.setBrightness(getFullBrightnessPwm());
          }
        } else if (dt > DOUBLE_TAP_MAX_MS) {
          wakeTapCounter = 1;
          firstWakeTapMs = now;
          lastWakeTapMs = now;
          waitingSecondTap = true;
        }
      }
    }
    else if (blState == BL_STATE_HALF) {
      blState = BL_STATE_FULL;
      tft.setBrightness(getFullBrightnessPwm());
      lastTouchMs = now;
      triggerTouchVibration();
      handleScreenTap(tx, ty);
    }
    else {
      lastTouchMs = now;
      triggerTouchVibration();
      handleScreenTap(tx, ty);
    }
  }
  // 2. Touch hold and drag (3D Earth rotation and Settings sliders)
  else if (isPressed && touchWasPressed && blState != BL_STATE_OFF) {
    lastTouchMs = now;
    if (blState == BL_STATE_HALF) {
      blState = BL_STATE_FULL;
      tft.setBrightness(getFullBrightnessPwm());
    }

    if (currentScreen == SCREEN_EARTH && ty > 36) {
      int16_t dx = tx - prevTouchX;
      int16_t dy = ty - prevTouchY;
      if (abs(dx) >= 2 || abs(dy) >= 2) {
        earthDragging = true;
        earthLon -= (float)dx * 0.025f;
        earthLat = constrain(earthLat + (float)dy * 0.025f, -1.45f, 1.45f);
        prevTouchX = tx;
        prevTouchY = ty;
        renderCurrentScreen();
      }
    }
    else if (currentScreen == SCREEN_SETTINGS && settingsView == SETTINGS_VIEW_BRIGHTNESS && settingsSliderDragging) {
      int32_t pct = (int32_t)BRIGHTNESS_MIN_PCT + ((int32_t)(tx - 22) * (100L - (int32_t)BRIGHTNESS_MIN_PCT) + 98L) / 196L;
      if (pct < BRIGHTNESS_MIN_PCT) pct = BRIGHTNESS_MIN_PCT;
      if (pct > 100) pct = 100;
      if ((uint8_t)pct != editDisplayBrightness) {
        editDisplayBrightness = (uint8_t)pct;
        tft.setBrightness(getBrightnessPwmFromPercent(editDisplayBrightness));
        renderCurrentScreen();
      }
    }
    else if (currentScreen == SCREEN_SETTINGS && settingsView == SETTINGS_VIEW_TOUCH_SENS && touchSensSliderDragging) {
      int32_t val = (int32_t)TOUCH_SENS_MIN_VAL + ((int32_t)(tx - 22) * (int32_t)(TOUCH_SENS_MAX_VAL - TOUCH_SENS_MIN_VAL) + 98L) / 196L;
      if (val < TOUCH_SENS_MIN_VAL) val = TOUCH_SENS_MIN_VAL;
      if (val > TOUCH_SENS_MAX_VAL) val = TOUCH_SENS_MAX_VAL;
      if ((uint8_t)val != editTouchSensitivity) {
        editTouchSensitivity = (uint8_t)val;
        renderCurrentScreen();
      }
    }
    else if (currentScreen == SCREEN_FLASHLIGHT && flashlightSliderDragging) {
      int32_t pct = (int32_t)BRIGHTNESS_MIN_PCT + ((int32_t)(tx - 22) * (100L - (int32_t)BRIGHTNESS_MIN_PCT) + 98L) / 196L;
      if (pct < BRIGHTNESS_MIN_PCT) pct = BRIGHTNESS_MIN_PCT;
      if (pct > 100) pct = 100;
      if ((uint8_t)pct != flashlightBrightness) {
        flashlightBrightness = (uint8_t)pct;
        if (flashlightOn) {
          tft.setBrightness(getBrightnessPwmFromPercent(flashlightBrightness));
        }
        renderCurrentScreen();
      }
    }
    else if (currentScreen == SCREEN_PAINT && paintDrawing) {
      if (ty >= PAINT_CANVAS_Y && ty < PAINT_CANVAS_Y + PAINT_CANVAS_H) {
        if (paintLastX >= 0 && paintLastY >= 0) {
          paintDrawSegment(paintLastX, paintLastY, tx, ty);
        } else {
          paintDrawSegment(tx, ty, tx, ty);
        }
        paintLastX = tx;
        paintLastY = ty;
        renderCurrentScreen();
      }
    }
    else if (currentScreen == SCREEN_BT_MOUSE && btMouseWheelDragging) {
      int16_t dy = ty - btMouseWheelLastY;
      btMouseWheelLastY = ty;
      btMouseWheelAccumY += dy;
      bool wheelMoved = false;
      while (btMouseWheelAccumY <= -BT_MOUSE_WHEEL_DRAG_STEP_PX) {
        btMouseWheelAccumY += BT_MOUSE_WHEEL_DRAG_STEP_PX;
        sendBleHidMouseWheel(1);
        btMouseWheelDraggedSteps = true;
        wheelMoved = true;
      }
      while (btMouseWheelAccumY >= BT_MOUSE_WHEEL_DRAG_STEP_PX) {
        btMouseWheelAccumY -= BT_MOUSE_WHEEL_DRAG_STEP_PX;
        sendBleHidMouseWheel(-1);
        btMouseWheelDraggedSteps = true;
        wheelMoved = true;
      }
      if (wheelMoved) {
        renderCurrentScreen();
      }
    }
  }
  // 3. Finger release
  else if (!isPressed && touchWasPressed) {
    if (currentScreen == SCREEN_BT_MOUSE && btMouseWheelDragging) {
      btMouseWheelDragging = false;
      if (!btMouseWheelDraggedSteps && prevTouchY >= 105 && prevTouchY <= 198) {
        sendBleHidMouseWheel((prevTouchY < 151) ? 1 : -1);
        renderCurrentScreen();
      } else {
        renderCurrentScreen();
      }
      btMouseWheelLastY        = -1;
      btMouseWheelAccumY       = 0;
      btMouseWheelDraggedSteps = false;
    }
    touchWasPressed = false;
    prevTouchX = -1;
    prevTouchY = -1;
    settingsSliderDragging   = false;
    touchSensSliderDragging  = false;
    flashlightSliderDragging = false;
    paintDrawing = false;
    paintLastX   = -1;
    paintLastY   = -1;
    if (currentScreen == SCREEN_EARTH && earthDragging) {
      earthDragging = false;
      renderCurrentScreen();
    }
  }

  if (waitingSecondTap && (now - lastWakeTapMs > DOUBLE_TAP_MAX_MS)) {
    waitingSecondTap = false;
    wakeTapCounter = 0;
  }

  // Keep display awake at max brightness during active SOS emergency beacon
  if (currentScreen == SCREEN_SOS && sosActive) {
    lastTouchMs = now;
    blState     = BL_STATE_FULL;
    tft.setBrightness(255);
    return;
  }

  // Keep display awake at chosen flashlight brightness while Flashlight is ON
  if (currentScreen == SCREEN_FLASHLIGHT && flashlightOn) {
    lastTouchMs = now;
    blState     = BL_STATE_FULL;
    tft.setBrightness(getBrightnessPwmFromPercent(flashlightBrightness));
    return;
  }

  // Prevent display from sleeping while alarm or timer alert is ringing
  if (activeAlertType != ALERT_NONE) {
    lastTouchMs = now;
    if (blState != BL_STATE_FULL) {
      wakeDisplayFromSleep();
      blState = BL_STATE_FULL;
      tft.setBrightness(getFullBrightnessPwm());
    }
    return;
  }

  // If 0 seconds selected, keep backlight on permanently
  if (settingDisplayLightSec == 0) {
    if (blState != BL_STATE_FULL) {
      wakeDisplayFromSleep();
      blState = BL_STATE_FULL;
      tft.setBrightness(getFullBrightnessPwm());
    }
    return;
  }

  uint32_t idleMs = now - lastTouchMs;
  uint32_t offMs  = (uint32_t)settingDisplayLightSec * 1000UL;
  uint32_t dimMs  = (offMs > TIMEOUT_DIM_MS) ? TIMEOUT_DIM_MS : (offMs / 2UL);

  if (blState == BL_STATE_FULL && idleMs >= dimMs && idleMs < offMs) {
    blState = BL_STATE_HALF;
    tft.setBrightness(getHalfBrightnessPwm());
  }
  else if (blState != BL_STATE_OFF && idleMs >= offMs) {
    blState = BL_STATE_OFF;
    waitingSecondTap = false;
    wakeTapCounter = 0;
    enterDisplayOffLightSleep();
  }
}

// ============================================================================
// 10. MIA-M10Q GPS RECEIVER DATA STREAM (FIXED 38400 BAUD)
// ============================================================================

// Update or add satellite entry in gpsSats[] from NMEA $xxGSV
static void upsertGpsSatellite(char sys, uint8_t prn, int8_t elev, int16_t azim, uint8_t snr) {
  if (prn == 0) return;
  uint32_t now = millis();
  bool inFix = false;
  for (uint8_t k = 0; k < gpsFixPrnCount; k++) {
    if (gpsFixPrns[k] == prn) {
      inFix = true;
      break;
    }
  }

  for (uint8_t i = 0; i < gpsSatCount; i++) {
    if (gpsSats[i].prn == prn && gpsSats[i].system == sys) {
      gpsSats[i].elevation  = elev;
      gpsSats[i].azimuth    = azim;
      gpsSats[i].snr        = snr;
      gpsSats[i].usedInFix  = inFix;
      gpsSats[i].lastSeenMs = now;
      return;
    }
  }

  if (gpsSatCount < GPS_MAX_SATS) {
    gpsSats[gpsSatCount].prn        = prn;
    gpsSats[gpsSatCount].system     = sys;
    gpsSats[gpsSatCount].elevation  = elev;
    gpsSats[gpsSatCount].azimuth    = azim;
    gpsSats[gpsSatCount].snr        = snr;
    gpsSats[gpsSatCount].usedInFix  = inFix;
    gpsSats[gpsSatCount].lastSeenMs = now;
    gpsSatCount++;
  }
}

// Parse complete NMEA 0183 sentence ($xxGSV and $xxGSA) with checksum verification
static void parseGpsNmeaSentence(const char* line) {
  if (!line || line[0] != '$') return;
  const char* star = strchr(line, '*');
  if (!star || star[1] == '\0' || star[2] == '\0') return;

  uint8_t calcCs = 0;
  for (const char* p = line + 1; p < star; p++) {
    calcCs ^= (uint8_t)(*p);
  }
  char csStr[3] = {star[1], star[2], '\0'};
  uint8_t givenCs = (uint8_t)strtoul(csStr, NULL, 16);
  if (calcCs != givenCs) return;

  // Copy payload without "$" and "*CS" for comma tokenization
  char body[96];
  size_t bodyLen = (size_t)(star - (line + 1));
  if (bodyLen < 6 || bodyLen >= sizeof(body)) return;
  memcpy(body, line + 1, bodyLen);
  body[bodyLen] = '\0';

  char* fields[24];
  uint8_t fCount = 0;
  char* cur = body;
  while (fCount < 24) {
    fields[fCount++] = cur;
    char* comma = strchr(cur, ',');
    if (!comma) break;
    *comma = '\0';
    cur = comma + 1;
  }
  if (fCount < 4) return;

  const char* talker = fields[0];
  size_t tLen = strlen(talker);
  if (tLen < 5) return;

  // Determine GNSS constellation from talker ID (GP, GL, GA, GB/BD)
  char sys = 'G';
  if (talker[0] == 'G' && talker[1] == 'L') sys = 'R';
  else if (talker[0] == 'G' && talker[1] == 'A') sys = 'E';
  else if ((talker[0] == 'G' && talker[1] == 'B') || (talker[0] == 'B' && talker[1] == 'D')) sys = 'B';

  // $xxGSA: active satellite PRNs used in fix (fields 3..14)
  if (strcmp(talker + 2, "GSA") == 0) {
    uint8_t nFix = 0;
    for (uint8_t i = 3; i <= 14 && i < fCount && nFix < 12; i++) {
      if (fields[i][0] != '\0') {
        uint8_t prn = (uint8_t)atoi(fields[i]);
        if (prn > 0) gpsFixPrns[nFix++] = prn;
      }
    }
    gpsFixPrnCount = nFix;
    for (uint8_t s = 0; s < gpsSatCount; s++) {
      bool found = false;
      for (uint8_t k = 0; k < gpsFixPrnCount; k++) {
        if (gpsSats[s].prn == gpsFixPrns[k]) {
          found = true;
          break;
        }
      }
      gpsSats[s].usedInFix = found;
    }
    return;
  }

  // $xxGSV: satellites in view (groups of 4 fields: PRN, Elev, Azim, SNR)
  if (strcmp(talker + 2, "GSV") == 0) {
    for (uint8_t i = 4; (uint8_t)(i + 3) < fCount; i += 4) {
      if (fields[i][0] == '\0') continue;
      uint8_t prn   = (uint8_t)atoi(fields[i]);
      int8_t elev   = (int8_t)atoi(fields[i + 1]);
      int16_t azim  = (int16_t)atoi(fields[i + 2]);
      uint8_t snr   = (fields[i + 3][0] != '\0') ? (uint8_t)atoi(fields[i + 3]) : 0;
      if (elev < 0) elev = 0;
      if (elev > 90) elev = 90;
      if (azim < 0) azim = 0;
      if (azim >= 360) azim = (int16_t)(azim % 360);
      if (snr > 99) snr = 99;
      upsertGpsSatellite(sys, prn, elev, azim, snr);
    }
  }
}

// Character-by-character NMEA sentence assembly alongside TinyGPSPlus
static void feedGpsNmeaChar(char ch) {
  if (ch == '$') {
    gpsNmeaPos = 0;
    gpsNmeaLine[gpsNmeaPos++] = ch;
    return;
  }
  if (gpsNmeaPos == 0) return;
  if (ch == '\r' || ch == '\n') {
    gpsNmeaLine[gpsNmeaPos] = '\0';
    parseGpsNmeaSentence(gpsNmeaLine);
    gpsNmeaPos = 0;
    return;
  }
  if (gpsNmeaPos < sizeof(gpsNmeaLine) - 1) {
    gpsNmeaLine[gpsNmeaPos++] = ch;
  } else {
    gpsNmeaPos = 0;
  }
}

void updateGPS() {
  if (!settingPowerGps || !gpsUartActive) return;
  while (GPSSerial.available() > 0) {
    char ch = (char)GPSSerial.read();
    lastGpsByteMs = millis();
    gps.encode(ch);
    feedGpsNmeaChar(ch);
  }
}

// ============================================================================
// 11. BOOT SPLASH SCREEN, SETUP, AND LOOP
// ============================================================================

// Embedded 240x240 baseline JPEG boot splash (from Manul-Picture.png, decoded by LovyanGFX TJpgDec)
const uint8_t BOOT_SPLASH_MANUL_JPG[15325] PROGMEM = {
  0xFF,0xD8,0xFF,0xE0,0x00,0x10,0x4A,0x46,0x49,0x46,0x00,0x01,0x01,0x02,0x00,0x25,0x00,0x25,0x00,0x00,0xFF,0xDB,0x00,0x43,
  0x00,0x06,0x04,0x05,0x06,0x05,0x04,0x06,0x06,0x05,0x06,0x07,0x07,0x06,0x08,0x0A,0x10,0x0A,0x0A,0x09,0x09,0x0A,0x14,0x0E,
  0x0F,0x0C,0x10,0x17,0x14,0x18,0x18,0x17,0x14,0x16,0x16,0x1A,0x1D,0x25,0x1F,0x1A,0x1B,0x23,0x1C,0x16,0x16,0x20,0x2C,0x20,
  0x23,0x26,0x27,0x29,0x2A,0x29,0x19,0x1F,0x2D,0x30,0x2D,0x28,0x30,0x25,0x28,0x29,0x28,0xFF,0xDB,0x00,0x43,0x01,0x07,0x07,
  0x07,0x0A,0x08,0x0A,0x13,0x0A,0x0A,0x13,0x28,0x1A,0x16,0x1A,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,
  0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,
  0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0xFF,0xC0,0x00,0x11,0x08,0x00,0xF0,0x00,0xF0,0x03,
  0x01,0x22,0x00,0x02,0x11,0x01,0x03,0x11,0x01,0xFF,0xC4,0x00,0x1C,0x00,0x00,0x02,0x03,0x01,0x01,0x01,0x01,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x05,0x02,0x03,0x06,0x07,0x01,0x00,0x08,0xFF,0xC4,0x00,0x3F,0x10,0x00,0x02,0x01,0x03,
  0x03,0x02,0x05,0x02,0x03,0x07,0x03,0x04,0x01,0x03,0x05,0x00,0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,0x21,0x31,0x41,0x06,
  0x13,0x22,0x51,0x61,0x14,0x71,0x32,0x81,0x91,0x07,0x23,0x42,0xA1,0xB1,0xD1,0xF0,0x15,0x52,0xC1,0x33,0x72,0xE1,0xF1,0x24,
  0x08,0x16,0x62,0x17,0x25,0x34,0x43,0x73,0xFF,0xC4,0x00,0x19,0x01,0x00,0x03,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x03,0x00,0x04,0x05,0xFF,0xC4,0x00,0x22,0x11,0x00,0x02,0x02,0x02,0x03,0x01,0x00,0x03,
  0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x11,0x03,0x21,0x12,0x31,0x41,0x51,0x04,0x13,0x22,0x61,0x32,0xFF,
  0xDA,0x00,0x0C,0x03,0x01,0x00,0x02,0x11,0x03,0x11,0x00,0x3F,0x00,0xCA,0x5B,0xE9,0x87,0x77,0x2B,0xD7,0xA5,0x39,0xB7,0xD3,
  0x76,0xA7,0x3D,0x4F,0xBD,0x19,0x12,0xB2,0xF0,0xC9,0xF6,0xA3,0x8A,0xB1,0x40,0x7B,0xD7,0x15,0x9D,0x14,0xC5,0x6B,0x60,0x50,
  0xFB,0x51,0xD6,0x81,0xE2,0x70,0x3F,0xA5,0x42,0x69,0x1D,0xB8,0xC1,0xC8,0xAB,0xAC,0x37,0xE7,0xD4,0x0F,0x1D,0x2B,0x36,0x1A,
  0x66,0x9F,0x4E,0x5D,0xC0,0x16,0xEB,0x4E,0x15,0x91,0x53,0xB5,0x20,0xB5,0x94,0x85,0x03,0x07,0x23,0xDA,0x8C,0x12,0x96,0x3D,
  0xF1,0xDE,0x96,0x32,0x1B,0x88,0x64,0xF2,0x27,0x55,0xEB,0xED,0x5F,0x5A,0x30,0x2C,0x42,0x83,0x55,0x22,0xAA,0xFA,0x8F,0x34,
  0x54,0x4C,0x07,0x4A,0xBC,0x68,0x9B,0x0A,0xB7,0x3C,0xE3,0xA0,0xA3,0x10,0x0E,0xE7,0x8A,0x12,0x16,0xC9,0xC5,0x12,0xA5,0x79,
  0xCF,0x14,0x49,0xBB,0x2F,0x41,0x81,0xC5,0x5A,0x1B,0xD3,0xCD,0x0A,0x8F,0x83,0xCF,0x4A,0xB3,0xCE,0x03,0xDA,0xB0,0x53,0x3E,
  0x93,0x38,0x39,0x14,0xBA,0xE2,0x40,0x4E,0x0D,0x1B,0x24,0xA5,0xC7,0x4C,0x66,0x86,0x68,0x77,0x75,0x15,0xA8,0x6E,0x40,0xA9,
  0x97,0x6C,0x0E,0x94,0x52,0x43,0x81,0xC5,0x79,0x1C,0x67,0x3C,0x0A,0x21,0x4E,0xD1,0x45,0x68,0x56,0xEC,0xAD,0x62,0xF7,0xA8,
  0x4B,0x10,0x53,0x91,0xD6,0x88,0x2D,0x8E,0x95,0x00,0x77,0x1A,0xC6,0x40,0xDB,0x73,0x1B,0x13,0xF6,0xA8,0x2C,0x7E,0xAC,0x37,
  0x5A,0x2D,0xD7,0xCB,0x01,0x7D,0xF9,0x35,0xF2,0xA6,0x4E,0x6B,0x36,0x3A,0x21,0x1D,0xA8,0x27,0x83,0x57,0x0B,0x5C,0x71,0x57,
  0xC3,0x85,0xE2,0x8A,0x51,0x95,0xE8,0x29,0x47,0x15,0xCB,0x0B,0x05,0xE0,0x70,0x28,0x47,0x84,0x37,0x38,0xE6,0x9E,0x32,0xD0,
  0xD2,0xC4,0xBF,0xC2,0x28,0xA1,0x5A,0x13,0x0B,0x7C,0x37,0xC5,0x7D,0x24,0x7C,0x74,0xA6,0x66,0x2E,0x3A,0x55,0x66,0x22,0x0F,
  0x4A,0x6B,0x13,0x88,0xA9,0xAD,0xFB,0xD5,0x4E,0xBE,0x5F,0x5E,0x94,0xD2,0x44,0x23,0x8C,0x1C,0x1A,0x87,0x94,0x8C,0x08,0x65,
  0xCD,0x6B,0x0F,0x16,0x03,0x13,0xE0,0xF3,0xD2,0xAF,0x27,0x3D,0xEB,0xE3,0x0E,0xC6,0xC6,0xDA,0x90,0x41,0xB7,0x15,0xAC,0x15,
  0x46,0x0E,0x78,0x4E,0xE2,0x14,0x1C,0x8A,0xBE,0xD1,0x19,0x86,0x19,0x5B,0xF4,0xA7,0x16,0xF1,0xA4,0x8D,0x9D,0xA2,0x8A,0xF2,
  0xA2,0xC7,0x00,0x0A,0xE3,0x8F,0xC2,0x96,0x2A,0x16,0x01,0xC6,0x71,0xC5,0x5E,0xB6,0xA1,0x40,0x38,0xA3,0xD4,0x2A,0xE7,0x3D,
  0x2A,0x4A,0xAA,0xC4,0x60,0xD5,0x14,0x44,0xE4,0x00,0x11,0x90,0xE7,0x18,0xCD,0x10,0x88,0xE4,0x63,0x80,0x4F,0x5A,0x3B,0xC9,
  0x18,0xC9,0x34,0x0C,0xCC,0x62,0x6C,0x21,0xAD,0x54,0x1E,0x4D,0x8C,0x23,0x40,0xD1,0x81,0x90,0x18,0x76,0x35,0x28,0x58,0x06,
  0x21,0xBA,0xFB,0x52,0xED,0xF2,0x3A,0xE0,0x1E,0x3D,0xE8,0x98,0x71,0xB0,0x09,0xCE,0x47,0x66,0x07,0x91,0x46,0xE8,0x29,0x58,
  0xCA,0x32,0x10,0xE7,0x39,0x06,0xA4,0xF7,0x0A,0x38,0x06,0x95,0xB3,0x3C,0x4B,0xBC,0x37,0x9B,0x06,0x70,0x58,0x76,0xFB,0xFB,
  0x1A,0x1E,0xE2,0x46,0x8F,0xD4,0x1B,0x28,0xDF,0x84,0xFB,0xD6,0xE6,0x07,0x11,0xC9,0xBB,0xCF,0x18,0xFC,0xEB,0xC4,0x98,0x13,
  0xEA,0x6E,0x94,0x8D,0x24,0x7F,0x73,0xF9,0xD5,0xA2,0xE0,0xED,0xC0,0x07,0x34,0xF1,0x76,0x23,0x54,0x3C,0x17,0x0A,0xBC,0xEE,
  0xC8,0xA9,0x0B,0xBC,0x8E,0x31,0x49,0xA1,0x6C,0xFE,0x2E,0xF5,0x29,0x26,0x45,0x5F,0x4B,0x77,0xC5,0x55,0x45,0x89,0x68,0x6D,
  0xF5,0xAA,0x3A,0xE2,0xA1,0x25,0xFC,0x64,0x70,0x6B,0x35,0x7B,0x78,0x15,0x4E,0x1A,0x94,0x7F,0xA8,0xB7,0x9A,0x17,0x71,0xE7,
  0xA5,0x24,0x9D,0x76,0x56,0x11,0x4C,0xDE,0xC1,0x76,0x8D,0xD4,0x8A,0x22,0x19,0x11,0xDF,0x83,0xC0,0xE4,0xFD,0xAB,0x0F,0x05,
  0xF3,0x75,0x07,0xA5,0x3E,0x86,0x72,0x91,0x2C,0x67,0x89,0x1B,0x0D,0x27,0xC7,0xB2,0xFF,0x00,0xC9,0xFB,0xFC,0x52,0xC6,0x49,
  0x86,0x51,0xA1,0xFB,0x10,0xE4,0x96,0x23,0x26,0xA2,0x24,0x51,0x59,0xCB,0x8D,0x4C,0xA6,0x55,0x4F,0x3D,0xAA,0xCD,0x32,0xEA,
  0x4B,0xA0,0x76,0x87,0x7C,0x1C,0x65,0x41,0x3C,0xE6,0x8B,0x60,0x4A,0x8D,0x22,0x48,0xA2,0xAD,0x17,0x00,0x77,0xA5,0x3F,0xBC,
  0xF3,0x14,0x65,0x46,0x46,0x79,0x61,0x56,0x21,0xF3,0x14,0x80,0xCA,0x7E,0xC6,0xB2,0x0F,0x2A,0x18,0xBC,0xDC,0x12,0x0D,0x56,
  0x24,0xDC,0x0F,0xBD,0x2C,0x12,0x32,0x9C,0x6F,0x06,0xA6,0x92,0x9C,0xE3,0xBD,0x10,0x72,0x18,0xAB,0x0E,0xF8,0xAF,0x1D,0xFD,
  0xA8,0x3F,0x32,0xAD,0xC8,0x09,0xD6,0xB0,0x51,0x77,0x0E,0x31,0x50,0x68,0x80,0x1D,0x30,0x6A,0x31,0x31,0xC7,0x15,0x70,0x7C,
  0x8C,0x1A,0xC3,0x58,0x39,0x8C,0x63,0x26,0xA9,0x91,0x07,0x5A,0xBE,0x52,0x43,0x71,0xD2,0xA8,0x32,0x12,0x4E,0x68,0x74,0x66,
  0x65,0x2D,0x2E,0x23,0xC9,0x55,0x3D,0x6A,0xD9,0xDD,0xD5,0x33,0x1F,0x35,0xED,0xBD,0x86,0x78,0xFE,0x23,0x45,0xC5,0x68,0xEA,
  0x4A,0xB8,0xCD,0x4A,0x30,0xF4,0x93,0x98,0xB1,0x25,0x9A,0x55,0xC1,0xEB,0x9E,0x82,0x98,0x59,0x41,0x20,0xE5,0xB3,0x91,0x46,
  0x41,0x65,0xB0,0xEE,0xC7,0x3F,0xD2,0x8C,0x48,0xF0,0x0E,0x45,0x56,0x84,0xB0,0x19,0x32,0x47,0x14,0x23,0x20,0x2E,0x41,0x3D,
  0x69,0xC7,0x91,0xBC,0x64,0x55,0x1F,0x44,0x32,0x59,0x87,0x26,0x83,0x43,0x26,0x06,0xB0,0x90,0x3A,0xF1,0x54,0xCC,0xA4,0x12,
  0xB9,0xC6,0x7A,0x1F,0x7A,0x67,0xE5,0x75,0x5A,0x0B,0x52,0x89,0xCD,0xAB,0x48,0x84,0x2B,0xC7,0xCE,0x31,0x53,0x71,0xB2,0x8A,
  0x74,0x07,0x6B,0x34,0xB1,0x6F,0x65,0x24,0x16,0x18,0xE3,0x9E,0x9E,0xE3,0xBD,0x46,0x69,0x62,0x70,0x36,0xED,0x45,0x77,0xDA,
  0x53,0x77,0xA5,0xCF,0x6D,0x84,0xFE,0x16,0xF8,0x3D,0x7A,0x52,0xC4,0xD5,0xD7,0xCB,0x6C,0xB2,0xEE,0x38,0x3C,0x76,0x23,0xB5,
  0x0F,0x1A,0xDC,0xEA,0x77,0x52,0xA5,0x98,0x48,0x62,0x67,0x06,0x4F,0xA8,0x03,0x63,0x63,0xA8,0x19,0xEB,0xFF,0x00,0x15,0x48,
  0x62,0x04,0xA6,0x69,0xF6,0x18,0x98,0xA9,0xE5,0xC9,0xC0,0xC8,0xC6,0x7F,0x2E,0xC7,0xB1,0x14,0x23,0x5E,0xA4,0x60,0x48,0x54,
  0x95,0xF3,0x7C,0xA6,0xE3,0xA6,0x7F,0xC0,0x6B,0x5F,0xA7,0xE8,0x11,0xDA,0xD8,0xC4,0x97,0x57,0x0D,0x2B,0x23,0x79,0x91,0x36,
  0x72,0x54,0x0E,0xD9,0xFE,0x21,0xFC,0xFF,0x00,0x4A,0xCF,0xEB,0x36,0x97,0x97,0x56,0xB7,0x27,0x46,0x8C,0x46,0xD2,0x3A,0x22,
  0xC6,0xF8,0xFD,0xDF,0x23,0x0F,0xFA,0x8F,0xE5,0x4F,0x15,0x14,0xE8,0x9B,0xB6,0x80,0x6F,0xE7,0x7B,0x53,0x17,0x9B,0x19,0x08,
  0xC0,0xE1,0xBB,0x7F,0x98,0xA5,0xCD,0x70,0x5D,0xBC,0xC5,0xDD,0xE5,0x11,0xBF,0x70,0xE4,0x05,0xE9,0x9F,0xB5,0x68,0xB5,0x77,
  0x8E,0x4D,0x35,0xAC,0xEE,0x26,0x53,0x3B,0x1F,0xFA,0x5C,0x8E,0x08,0xCB,0x10,0x7F,0x51,0xF3,0x91,0x4A,0xF4,0xED,0x09,0x5F,
  0x47,0xDB,0xB1,0xCD,0xF1,0xB7,0x40,0xCE,0x5B,0x92,0x0E,0x46,0x07,0x6C,0x01,0x8F,0xCF,0x35,0xD0,0xA7,0x1A,0x25,0x52,0x07,
  0x3A,0x1E,0xA7,0x73,0xB5,0xD2,0xDC,0x2C,0x18,0xDC,0xCE,0xEC,0x00,0xE7,0xA7,0xF2,0xE7,0xF3,0x14,0xA2,0xD2,0xDD,0x26,0xBF,
  0x97,0x33,0x44,0xAC,0x37,0x30,0xEA,0x7D,0x03,0xD2,0xA4,0x7D,0xCF,0x41,0x4F,0x64,0xB1,0xBC,0xB6,0xD0,0xAD,0x34,0xB3,0x77,
  0xBC,0x99,0xA4,0x86,0x62,0x64,0xF4,0xF9,0x67,0x82,0x49,0xCE,0x49,0x1C,0x81,0xF3,0x4E,0x66,0xD2,0x34,0xF4,0x5B,0x51,0x3C,
  0x51,0xA0,0x8C,0x88,0x91,0x40,0xCA,0x71,0x92,0x72,0x3D,0xB2,0x7A,0x7B,0x00,0x3B,0x9A,0x9C,0x9C,0x2A,0xE4,0x8A,0x45,0xC9,
  0x69,0x0A,0x74,0xEB,0x24,0xB5,0x9A,0x23,0x75,0x25,0xBB,0x2B,0x10,0xB0,0xA0,0x94,0x13,0x34,0x98,0xCE,0xD0,0x3B,0x81,0xD4,
  0x9E,0x9D,0xBB,0xD1,0x6F,0x03,0x2A,0x1B,0x8B,0xB7,0x31,0x44,0x72,0xCC,0xC0,0xEF,0xFC,0x86,0x32,0x49,0xFB,0x66,0x95,0xF8,
  0xDE,0x6F,0x0E,0x58,0x47,0x27,0xD7,0x5C,0x45,0x03,0xB4,0x26,0x24,0x5C,0xFE,0xFE,0x4C,0x9C,0xED,0x1C,0x7A,0x54,0xF7,0x03,
  0x1F,0x34,0xB7,0xC3,0x3A,0xB5,0xAD,0xB4,0x32,0xF9,0x5A,0x97,0x9D,0x2E,0xF1,0xE5,0x24,0x8E,0x83,0xCB,0xF8,0xDD,0xD5,0xBE,
  0xC0,0x53,0xC7,0x1C,0x1A,0xE8,0x5F,0xD9,0x34,0x5B,0x2F,0x88,0x74,0xF5,0x96,0xE9,0x6D,0xED,0xF3,0x2A,0x2A,0x90,0xD3,0x8E,
  0x70,0x7A,0x9D,0xBD,0x80,0xCF,0x72,0x69,0xEE,0x97,0x7A,0xD7,0x5A,0x6B,0x17,0x98,0xC8,0xC1,0xC3,0x63,0x38,0x03,0x71,0xE0,
  0x00,0x3D,0x86,0x29,0x46,0xAA,0x1F,0x5B,0xB6,0xB8,0x5B,0xEB,0x65,0x12,0xA8,0x2A,0x2E,0x82,0x04,0x55,0x53,0xFC,0x5E,0xED,
  0xF1,0xDA,0x86,0xD1,0x26,0x16,0xB3,0xC6,0x5D,0x7C,0xD1,0x26,0x66,0x55,0xEC,0x11,0x4E,0xD0,0x78,0xEA,0x58,0x9F,0xD2,0x96,
  0x58,0x97,0x86,0x53,0xBE,0xCD,0x8D,0x93,0x7F,0xF0,0xA3,0x95,0xB8,0x2C,0xAC,0xB9,0xF8,0xDD,0xDB,0xF4,0xA3,0xA4,0x5D,0xB1,
  0x04,0x41,0xEA,0x3C,0x9F,0x8E,0x33,0x4A,0x34,0xDB,0x9B,0x6B,0xAB,0x68,0xD4,0xB1,0x49,0x62,0xC6,0xC8,0xC7,0x39,0x38,0xEB,
  0x8F,0xE7,0xCD,0x15,0x75,0x71,0x73,0x85,0x82,0xD9,0x56,0x49,0x26,0x56,0x3E,0x62,0xB7,0x03,0x1D,0x7F,0xCF,0x8A,0x45,0x8E,
  0x86,0xE7,0x65,0xB1,0x80,0x48,0x25,0x86,0xD0,0x09,0x63,0x42,0x9B,0xB8,0xF7,0xE3,0x76,0x33,0xD2,0x91,0x3F,0x89,0xF4,0xDD,
  0x3A,0xDD,0xA3,0x75,0x92,0xE2,0x38,0x54,0x46,0xCC,0xCD,0xB5,0x48,0xCE,0x19,0xF8,0xEB,0x93,0xC0,0xF7,0xC5,0x0D,0xA7,0xEA,
  0x53,0xDC,0xCF,0xF5,0x16,0x9A,0x74,0x42,0x3F,0xC4,0x64,0x9F,0x24,0x15,0xA1,0x41,0x4C,0xD7,0xC6,0xE4,0xC7,0xBC,0x9F,0x47,
  0x41,0x9E,0x28,0xC8,0x63,0x66,0x5C,0x1C,0x02,0x7E,0x6B,0x3F,0x03,0xEA,0x7A,0x84,0xAC,0xEF,0x02,0x08,0xCF,0x11,0xB7,0x0A,
  0xA0,0x7B,0xF3,0xFE,0x7C,0xD1,0x26,0xDE,0xF2,0xC6,0x60,0xF0,0x4D,0xA5,0xAC,0x47,0x01,0x9A,0x4B,0x83,0x93,0xFA,0xE4,0xE6,
  0x99,0x41,0x34,0x65,0x92,0xBB,0x1F,0x24,0x32,0xA2,0xF0,0x2B,0xC6,0x12,0x01,0x9D,0xB4,0xEF,0x41,0x8C,0x5D,0x5B,0x23,0x16,
  0x0E,0x4F,0x04,0xA8,0x1B,0x4F,0xDA,0x9A,0x4B,0xA5,0x29,0xE9,0x8C,0x54,0xDA,0x68,0xBA,0x49,0x98,0xC9,0x32,0x47,0x20,0x83,
  0x55,0xEC,0x24,0xE4,0x74,0xAD,0x6C,0x9A,0x52,0x9C,0xE5,0x68,0x66,0xD3,0x15,0x7F,0x87,0x8A,0x01,0xE2,0x64,0x91,0x10,0xC8,
  0x19,0x71,0x81,0x47,0x2A,0x2B,0x0C,0x9E,0xB5,0x93,0xD3,0x6F,0xE4,0x6F,0xCE,0x9A,0x9B,0xA9,0x07,0x05,0xB1,0x51,0x53,0x44,
  0x28,0x76,0x00,0xC6,0x00,0xA8,0x94,0x5E,0x94,0x2D,0x95,0xC8,0x7C,0x02,0x68,0xA9,0x1D,0x47,0x24,0xF1,0x54,0xE4,0xA8,0x5A,
  0x24,0xB8,0x1D,0xAA,0x12,0xE3,0x18,0x2D,0x8C,0xD0,0xF3,0x5E,0xA4,0x7C,0x9E,0x94,0x34,0x97,0xA9,0x21,0x3B,0x10,0xCA,0xBD,
  0x4E,0xDE,0x48,0xA2,0xA4,0x99,0x83,0x6D,0xED,0xC3,0xB9,0x57,0x39,0x1E,0xE2,0xA3,0x77,0x68,0x63,0x67,0x8A,0x42,0xAF,0x6C,
  0xC3,0x0C,0x47,0x6F,0x93,0x57,0x69,0xC8,0xB1,0x91,0x33,0x07,0xF2,0xF1,0xF8,0x87,0x5A,0x86,0xA1,0x72,0xED,0x86,0x54,0x56,
  0x43,0xC7,0x98,0x38,0xC0,0xF9,0xAA,0x24,0xA8,0x09,0x98,0xE9,0x3C,0x30,0x46,0xA9,0x9B,0x04,0xF3,0x5D,0x88,0xCA,0x29,0x18,
  0x23,0xA6,0x7F,0x9D,0x3B,0xBB,0xFD,0x9F,0xFD,0x62,0x3A,0x5D,0xBB,0xDA,0xBA,0x15,0x92,0x23,0x14,0x99,0x40,0xE3,0xDE,0x3F,
  0x6C,0xF7,0xF9,0xA3,0xB4,0x0B,0x98,0xDF,0x54,0x8C,0xC2,0xE8,0x71,0xD1,0x8A,0x95,0x01,0xBB,0x0C,0x77,0xAC,0x95,0xDC,0x5E,
  0x3F,0xF1,0x3E,0xB0,0xF2,0x58,0x6B,0x4B,0xA7,0x58,0x3D,0xCC,0x91,0xC3,0x1A,0x42,0xBB,0x8C,0x51,0x9D,0xAD,0x2B,0x12,0x09,
  0xC1,0x6E,0x16,0x86,0x5C,0xD1,0xC3,0x07,0x29,0x69,0x22,0x98,0xF1,0xB9,0xB3,0xA1,0x4D,0x6B,0x75,0x61,0x6B,0x6E,0xEB,0xFB,
  0xD5,0x8C,0xED,0x99,0x13,0x85,0xC7,0xFB,0x94,0x76,0xC7,0xB0,0xAE,0x28,0x9E,0x2A,0xB9,0xD2,0xB5,0xB9,0xAD,0x9E,0x2B,0x88,
  0x7E,0x81,0xA5,0xDC,0x5F,0x39,0x92,0x22,0xC0,0x2F,0xA7,0xF8,0xB1,0xBB,0x3F,0x61,0x9F,0x7A,0xDA,0x78,0x82,0x0F,0x15,0xF8,
  0x12,0xC0,0x6A,0x52,0x78,0xAE,0xC6,0xEA,0xD1,0xD9,0x49,0x83,0x51,0x45,0x54,0x95,0xCE,0x4E,0xC0,0xC0,0x02,0xAD,0xC1,0xF8,
  0xAA,0xE1,0xD5,0xEC,0x7F,0x69,0x7F,0xB3,0xED,0x6E,0xE1,0xB4,0xE1,0xA7,0xEB,0x7A,0x63,0x04,0xC2,0x9E,0x79,0xFC,0x21,0x58,
  0xF5,0x04,0x02,0x31,0x49,0x0C,0x91,0xC9,0x1E,0x48,0x7A,0xAF,0xE7,0xB1,0x57,0x88,0x3C,0x55,0xE5,0x78,0x8A,0x6B,0x13,0x19,
  0x11,0x42,0xA8,0x1E,0xE8,0x46,0x08,0x5C,0x29,0x60,0x3A,0xE0,0xE4,0x60,0x0F,0x9A,0x5F,0xE2,0x1F,0x17,0x8B,0x7D,0x1E,0xC6,
  0xEC,0x48,0xC9,0x3F,0x31,0x08,0x81,0xCA,0x95,0x2B,0x95,0xC8,0xF6,0x0C,0x31,0x5E,0xE9,0x7E,0x17,0xD4,0x75,0x38,0x8C,0x37,
  0x30,0x02,0x62,0x85,0x23,0x79,0x5F,0xD2,0x03,0x85,0x0C,0x09,0xEE,0x0E,0x40,0xE9,0xDC,0x56,0x4F,0xC5,0x3E,0x17,0xF2,0x2E,
  0x55,0x1F,0x52,0x69,0xE0,0x66,0x2C,0x24,0x51,0xBB,0xCB,0x63,0x82,0xCA,0x79,0xC7,0x50,0x7F,0x4A,0x78,0xC9,0x31,0x5C,0x68,
  0x0A,0xC6,0x7F,0x15,0xF8,0x87,0x54,0x8A,0x1B,0x30,0xEE,0x63,0x69,0x0B,0x04,0x3B,0x51,0x49,0x39,0x2C,0x4F,0x6C,0xF3,0xCF,
  0xF7,0xAE,0x9A,0xB6,0x7A,0xD6,0xA5,0xAD,0x59,0x58,0xBA,0x34,0x56,0xB6,0x8B,0xE6,0x4F,0x70,0xCD,0xE9,0xC6,0x78,0x45,0xFE,
  0xBD,0x7D,0xBE,0x2B,0x34,0x9E,0x32,0x4F,0x0F,0xE8,0xAB,0x63,0xA0,0x69,0xF1,0xDB,0xCC,0x47,0x32,0xDD,0x12,0x0B,0x9F,0x92,
  0x07,0xAA,0xB3,0x4D,0x07,0x8C,0xBC,0x61,0x74,0x2D,0xEF,0x75,0x95,0x3A,0x7B,0x3A,0x16,0x8E,0xCE,0x71,0xB0,0x7D,0xC2,0x9C,
  0x9C,0x71,0xEF,0xC9,0xAA,0xF1,0x8B,0x56,0xC4,0x52,0xF1,0x1D,0x2D,0xBC,0x3F,0xE1,0xFB,0x89,0x27,0x5B,0xAB,0xFB,0x3B,0xAD,
  0x55,0xE4,0x62,0x6E,0x67,0x2B,0x23,0x20,0x3D,0x95,0x49,0xC0,0xC0,0x1D,0x4E,0x71,0x8C,0xD5,0x91,0x78,0x0F,0xC3,0x3F,0x4A,
  0xAB,0x6F,0xE4,0x9B,0x9D,0x9B,0x4C,0xA4,0xE5,0x98,0x1E,0xFB,0xBA,0x03,0xF6,0xA1,0x9F,0xF6,0x3B,0xA2,0xD9,0xE9,0xEA,0x2C,
  0x96,0x4B,0xA3,0x28,0xC3,0xE2,0x20,0x5D,0xC8,0xC6,0x41,0x2C,0xC3,0xBF,0x60,0x7B,0x12,0x4F,0x15,0x44,0xDE,0x08,0xB7,0xF0,
  0xDE,0x9F,0xF5,0xDE,0x1A,0x6B,0x38,0xB5,0x56,0xE7,0xFF,0x00,0x91,0x7C,0x8D,0x1C,0x63,0xE7,0x19,0x5C,0xFC,0x2A,0x9C,0x77,
  0x63,0x51,0x86,0x59,0x39,0x52,0x4D,0x05,0xC6,0x35,0xA6,0x15,0xFE,0x9B,0xA8,0x69,0x9B,0xE3,0xB8,0x92,0xE2,0x5D,0x39,0x97,
  0x08,0xD9,0x52,0x78,0xE3,0xBE,0x06,0x3A,0x7A,0x89,0xE3,0xDE,0x97,0x5D,0x5C,0x6C,0x8C,0x47,0x6D,0x72,0xB0,0xCD,0x22,0x80,
  0xA6,0x01,0xBD,0xD4,0x0E,0xAB,0xBC,0xE0,0x1C,0xF7,0xC1,0xC7,0xDE,0x95,0x4D,0xAB,0xEB,0xBA,0x95,0xAC,0x9A,0x76,0xA3,0x76,
  0x97,0x17,0x41,0x77,0x30,0x83,0x71,0x46,0x19,0xEA,0x1B,0x83,0x8F,0xBD,0x29,0xD2,0xCE,0xD9,0xCC,0x7A,0x9C,0xD6,0xD2,0x49,
  0x1E,0x54,0x08,0xD7,0xCF,0x75,0x3F,0x91,0x03,0xB7,0x73,0xFA,0xD7,0x4D,0xB2,0x55,0x47,0x42,0xD1,0x6E,0x6D,0x64,0x86,0x68,
  0x6D,0x99,0xD2,0xE0,0x46,0x20,0xDE,0xB9,0x6C,0x00,0x30,0x59,0x8F,0x03,0x81,0x93,0x8F,0xB5,0x59,0x69,0x77,0x71,0xA7,0xC0,
  0x6C,0x3F,0x79,0x71,0x70,0x50,0x96,0x74,0x19,0x60,0x99,0xC8,0x00,0x7B,0x9C,0x74,0xE0,0x70,0x2B,0x19,0x05,0xFA,0x48,0x62,
  0xFF,0x00,0x4F,0xD2,0xEF,0x66,0xCB,0x16,0x67,0x91,0x59,0xB0,0x7A,0x64,0x85,0x00,0x76,0xF9,0xAD,0x8E,0x87,0x16,0x61,0x90,
  0xCF,0x15,0xC3,0x4F,0x26,0x4A,0xC6,0x14,0xAE,0xF6,0x3D,0x49,0xE9,0xC0,0xF9,0x34,0x8D,0xBE,0x83,0x42,0x6D,0x73,0xF7,0xD7,
  0xD6,0x76,0x42,0xD6,0x38,0x5E,0x42,0xB3,0xCB,0x6C,0xCA,0x0E,0xC1,0x8C,0x2A,0xE7,0xAB,0xB6,0x73,0xE9,0x1F,0x24,0xE0,0x56,
  0x8F,0x46,0x83,0xC9,0x13,0x3D,0xE4,0x6C,0x66,0x27,0x84,0xFE,0x05,0x0B,0xC0,0x3E,0xDF,0xE7,0x4A,0x2E,0x3B,0x14,0x83,0x51,
  0xB8,0x9A,0x33,0x0C,0x97,0x2D,0x85,0x93,0x82,0x3C,0x94,0x03,0x04,0x33,0x01,0x93,0xFF,0x00,0x6A,0xE3,0xAF,0x26,0x8E,0x76,
  0x81,0x2D,0xC9,0x96,0x79,0x61,0xDC,0x3A,0xB2,0xA8,0x91,0xC7,0x60,0xA8,0x33,0xB4,0x7C,0x75,0xA1,0x57,0xD8,0x2F,0x42,0x9B,
  0xFB,0xC8,0xEE,0x9F,0xCB,0xBB,0xBA,0x54,0x40,0x7F,0xFE,0xB9,0x8C,0x7F,0x97,0x00,0xD1,0x56,0x76,0xB6,0x4F,0x22,0x83,0x3D,
  0xCE,0x0F,0xE1,0x2C,0xFB,0x94,0x7B,0x9C,0x91,0x55,0xC9,0x79,0xA6,0x5A,0xAA,0x8F,0x25,0xCA,0xE3,0x27,0x78,0x39,0xFD,0x00,
  0xA9,0x9D,0x5E,0xD6,0x58,0x83,0x43,0xBB,0xCB,0xC7,0xA6,0x38,0xD7,0x8F,0xE6,0x94,0xDC,0x68,0x56,0xCD,0xD7,0x86,0x23,0x58,
  0xC3,0xF9,0x37,0x26,0x68,0x73,0x9C,0xE3,0xA1,0xFC,0xB8,0xAD,0x74,0x2C,0x36,0x60,0xB0,0x63,0xEE,0x2B,0x91,0x43,0xE3,0x21,
  0x62,0xA8,0xB2,0x89,0x56,0x3E,0x81,0x4C,0x78,0x27,0xF9,0x01,0xFC,0xA9,0xDC,0x5F,0xB4,0x1B,0x34,0x88,0x6E,0x05,0x38,0xEF,
  0x4A,0xE2,0xFE,0x1D,0x38,0xDA,0xAA,0xB3,0xA1,0x1C,0x0F,0xC5,0x81,0x41,0xDC,0xBC,0x6A,0x09,0x24,0x0A,0xC1,0x5D,0xF8,0xFA,
  0x26,0x81,0x8C,0x24,0x96,0x3D,0x05,0x64,0xE7,0xF1,0x76,0xA1,0x77,0x33,0x21,0x70,0x83,0xDA,0x8A,0xC5,0x26,0x67,0x96,0x31,
  0xF4,0xBB,0x4D,0xB0,0x3B,0xF3,0xD8,0x51,0x37,0x08,0xC6,0x42,0x81,0x80,0xC7,0x5A,0x30,0xFA,0x07,0xA0,0x72,0x6B,0xE8,0x6D,
  0x99,0xC3,0x16,0xE4,0xD7,0x9B,0x8E,0x16,0x80,0xE5,0x45,0x56,0x50,0x32,0x80,0x41,0x34,0xCD,0x63,0x66,0x1C,0xE4,0xD7,0x96,
  0x51,0xB0,0x04,0x10,0x29,0xDD,0xB5,0xBA,0x08,0xC1,0x61,0xCD,0x51,0x45,0x82,0xCC,0xCD,0xF4,0x19,0x52,0x31,0xC7,0xDA,0x80,
  0xB3,0xBA,0x7B,0x59,0x42,0xA9,0x78,0xCF,0xB9,0x42,0x45,0x6B,0x6F,0xAD,0xD4,0xFF,0x00,0xD3,0x20,0x67,0xA9,0xAF,0xAD,0x2D,
  0xCF,0x0A,0xB9,0x76,0xFF,0x00,0xB4,0x55,0x71,0x43,0x62,0xC9,0xE8,0x8C,0x17,0x25,0xE1,0x53,0x23,0x26,0x0F,0x05,0x80,0x1F,
  0xDE,0x83,0xD5,0xA5,0xB7,0xB2,0x81,0x9A,0x39,0x41,0x66,0x38,0xF5,0xAE,0x41,0xFB,0x53,0x3B,0xB4,0x8A,0x14,0x61,0x32,0x80,
  0x47,0x21,0x72,0x39,0x35,0x94,0xBF,0xB5,0x9F,0x58,0xBB,0xF2,0x44,0x82,0x28,0x86,0x3F,0xE9,0xCA,0x01,0x1F,0x7F,0x7F,0xCA,
  0xBB,0x23,0x12,0x69,0x97,0x78,0x49,0x20,0xD4,0xFC,0x40,0x8F,0x2C,0x40,0x47,0x6A,0x37,0xB9,0xE9,0xCF,0xB0,0x03,0xFA,0x9E,
  0xBE,0xD5,0xA4,0x83,0x47,0xD4,0x34,0xDD,0x43,0xEA,0xF4,0xEF,0xA5,0xBF,0xB3,0x5B,0x73,0x02,0x43,0x31,0x30,0xCA,0x80,0xB9,
  0x7C,0x86,0xE4,0x1E,0xA4,0x73,0x8A,0xE7,0xDA,0x9E,0x93,0xE1,0xCB,0x1B,0x9F,0xF4,0xC8,0x75,0x6D,0x45,0x2E,0xD8,0x19,0x5E,
  0xDD,0x2E,0x4C,0x7E,0x6F,0x6D,0xD9,0x5E,0xA4,0x7B,0x66,0xB1,0x5A,0x88,0x9F,0x41,0xD4,0x9D,0xED,0x35,0xCD,0x7A,0xD6,0x38,
  0x01,0x73,0x19,0x98,0xCF,0x8E,0x7D,0x3C,0x74,0x2A,0x7D,0xCD,0x4F,0x26,0x28,0xE4,0x8B,0x83,0xE9,0x94,0x84,0xDC,0x5D,0xA3,
  0x59,0xE3,0xCF,0x0F,0x78,0xAB,0x5A,0xF1,0x5B,0x5D,0x5A,0xC1,0xF4,0x7A,0x64,0xF6,0xC2,0x0B,0xA4,0xBF,0x96,0x39,0x2D,0xB6,
  0x80,0x46,0xFC,0x03,0x9E,0x03,0x64,0x71,0x9D,0xC0,0x53,0x8F,0x09,0xD8,0xD9,0x68,0x96,0xD7,0xA9,0xA7,0xA4,0xB3,0x5A,0xCE,
  0x91,0xE6,0x67,0x3C,0x37,0x94,0x36,0xAA,0xA0,0x3C,0xE3,0xBE,0x4F,0x52,0x4D,0x21,0xD2,0x6C,0x3C,0x59,0xE2,0xFD,0x22,0xE0,
  0xDE,0x6B,0x32,0x2C,0x8F,0x1E,0xE8,0xE0,0x7C,0x46,0x9F,0x06,0x40,0x9F,0xAE,0x28,0xC6,0xB0,0x6F,0x01,0xF8,0x69,0xD7,0x53,
  0xD4,0xE4,0x9B,0x7E,0xF7,0x9E,0xEB,0x66,0x5D,0x22,0x00,0x0D,0x91,0xA9,0xE3,0x73,0x36,0xEE,0x7B,0x01,0xEF,0x5C,0xF9,0x52,
  0xC5,0x15,0x89,0x32,0xD8,0x31,0x3C,0x92,0x6D,0x2D,0xB2,0x1E,0x28,0xD5,0x75,0xAB,0x9B,0x78,0x6D,0x2C,0x6C,0x25,0x08,0x1B,
  0x13,0xB0,0x90,0x22,0xB9,0x6E,0x4E,0x0B,0x10,0x71,0xCF,0xDF,0x8A,0x5F,0x6D,0xE1,0x38,0x2F,0x35,0x15,0x9B,0xC5,0x27,0xCE,
  0x88,0x11,0xE4,0x5B,0xDB,0x4C,0x3C,0xB8,0xFF,0x00,0xEE,0xC7,0x52,0x49,0xF9,0xCD,0x09,0xA4,0xF8,0x8F,0x44,0x6D,0x3E,0x7B,
  0xFD,0x3E,0xD2,0x2B,0xC4,0xB8,0x8D,0x8A,0xCB,0x74,0xBB,0xDD,0xCF,0x3C,0x31,0x6C,0x91,0xC8,0xE4,0x52,0xFF,0x00,0xD9,0x15,
  0xB3,0x6B,0xF7,0x8A,0xB7,0xB1,0xC9,0x63,0x69,0x6A,0xA7,0xEA,0xF6,0x83,0x1A,0xC9,0x2B,0x93,0xB5,0x06,0x0F,0x38,0xCE,0x46,
  0x3A,0x0C,0x0E,0xF5,0xCF,0xC2,0x55,0x50,0x7B,0x3B,0x72,0x60,0x78,0x92,0x96,0x4A,0xA6,0x6C,0xBC,0x63,0xA5,0x58,0xEB,0x89,
  0x16,0x95,0xA3,0xE9,0x90,0xCF,0x79,0x75,0xE9,0x57,0x8C,0x71,0x08,0xF7,0x62,0x3A,0x00,0x7A,0xD6,0x9B,0x41,0xF0,0x4E,0x81,
  0xE1,0xAD,0x32,0x1B,0x56,0x8C,0xDF,0x5F,0xBE,0x03,0xC9,0x23,0x0F,0x49,0xC6,0x19,0x87,0xFB,0x40,0xE7,0xFC,0x34,0xD2,0xD2,
  0x5B,0x3D,0x0E,0xD0,0x59,0x78,0x7B,0x4F,0x66,0x31,0x91,0xBB,0x0D,0x80,0x09,0xEE,0xCC,0x7F,0xF2,0x78,0xA1,0x6D,0xAE,0x1D,
  0x27,0x91,0xA5,0x64,0xBA,0xBF,0x07,0x6B,0x04,0x8F,0x29,0x17,0xFF,0x00,0x88,0x3D,0xFA,0xF7,0x39,0xAA,0xC6,0x53,0x8A,0xA6,
  0xEC,0xE2,0x71,0x52,0xDA,0x00,0xD7,0x7C,0x0B,0xA7,0x5E,0x69,0x17,0x70,0xD9,0xDF,0xDC,0x5A,0x8B,0x95,0xF2,0x9E,0x79,0x24,
  0x2E,0xDE,0x58,0xEA,0x8B,0x92,0x02,0xAF,0x18,0xC2,0xE3,0x3D,0x09,0xAC,0x0C,0x7E,0x10,0xB4,0xD3,0xEE,0x27,0x36,0xAB,0xA9,
  0xCB,0x63,0x0B,0x79,0x4D,0x2C,0x92,0x34,0x70,0x6F,0xE8,0x17,0x62,0x8C,0xC9,0xD8,0x61,0x41,0x03,0xDB,0xAD,0x74,0xFB,0x8B,
  0x8B,0x94,0x76,0x79,0xF7,0xB3,0xE7,0xD2,0xCF,0x80,0x71,0xF6,0xE8,0x00,0xFB,0x57,0x39,0xF1,0x06,0xA5,0x79,0xA7,0xEB,0x61,
  0xD4,0xDD,0xCF,0x68,0xE3,0xD0,0xC2,0x39,0xAE,0x4F,0x43,0x90,0xB8,0x01,0x14,0x73,0xDF,0x9E,0xB5,0xD9,0x8E,0x7C,0x88,0x4A,
  0x14,0x17,0xA8,0x69,0xDA,0x2D,0xCC,0x91,0x5A,0xDC,0xE9,0xDA,0x85,0xE1,0x38,0x2E,0x92,0x4C,0x6D,0x62,0x8C,0x74,0xC2,0xAB,
  0x0C,0x9C,0xF5,0xEC,0x78,0xE7,0x15,0x66,0xBF,0xA1,0x8D,0x26,0xC5,0x6E,0x7C,0x3F,0x2D,0x8E,0x9E,0x88,0xBE,0xA8,0xA0,0x87,
  0xCF,0x9C,0x8C,0x74,0x0C,0x4E,0x73,0xF7,0x38,0xAC,0xCF,0x8A,0xB4,0x9B,0xF9,0x65,0x8B,0x53,0xB3,0xBA,0xD4,0xBC,0x90,0x06,
  0xE8,0xA2,0xB7,0x45,0x28,0x3A,0xF5,0x20,0xB1,0xEF,0xDB,0x15,0xA9,0xF0,0x8E,0xB9,0x6F,0x7D,0x68,0xB1,0xC9,0x1D,0xE2,0xCE,
  0x8B,0xB4,0x7D,0x52,0xAC,0x65,0xBE,0x40,0x2B,0xCF,0xDE,0xAC,0x98,0x9C,0x4E,0x71,0x65,0xAB,0xDC,0x6A,0xAC,0xCA,0xFA,0xA5,
  0xD9,0x24,0x93,0xBA,0xE2,0x55,0xE3,0x1E,0xF9,0x27,0x02,0x8C,0xB6,0xD7,0xEF,0x20,0x40,0xB0,0xDD,0x19,0x1D,0x49,0x27,0xE9,
  0x76,0x37,0x1F,0x24,0x00,0x47,0xBF,0x5A,0xD3,0x78,0xC7,0xC3,0x91,0xCC,0x0D,0xC5,0xBC,0x77,0x4A,0xCA,0x0B,0x3C,0x31,0x98,
  0xD5,0x58,0x7B,0xFA,0x57,0x03,0xF3,0xCD,0x72,0x81,0xA8,0x45,0x6D,0x7B,0x2A,0x47,0x24,0xF0,0x4B,0xF8,0x5D,0x24,0x21,0xC9,
  0x1D,0xCE,0x46,0x3F,0x90,0xA0,0xD5,0x85,0x1D,0x6F,0x42,0xD7,0x21,0x93,0x4D,0x59,0x6E,0x7C,0xDB,0xB9,0x62,0x3E,0x94,0x66,
  0x67,0x5D,0xDE,0xE7,0x9C,0x1A,0x2F,0xFF,0x00,0xBC,0xE5,0xB5,0x8F,0x33,0x5A,0x45,0x1E,0x32,0xCC,0x10,0x65,0x94,0x7B,0x92,
  0x33,0xCD,0x72,0xED,0x0F,0x5D,0x4B,0x6D,0xC8,0xD2,0x87,0x2C,0xDB,0x51,0x9C,0x15,0x65,0xF9,0x04,0x63,0x3F,0x9D,0x6F,0xB4,
  0xFD,0x5E,0xC6,0xFE,0x14,0xB6,0x9E,0x3B,0x99,0x9C,0x75,0xD8,0x7C,0xA1,0xF0,0x4F,0xBF,0xEB,0x50,0x49,0xA7,0x4C,0x69,0x51,
  0x3B,0x5F,0x1F,0xDA,0x5D,0xDC,0xB7,0x93,0xA4,0xEA,0x58,0x62,0x7F,0x7D,0x22,0x6D,0x51,0xF9,0x81,0x53,0xB8,0xD6,0x85,0xC1,
  0x95,0x2D,0x2E,0xF5,0x04,0xDC,0x32,0xE6,0x1B,0x90,0xCA,0x9F,0x18,0xC6,0x7F,0x5A,0xA7,0x51,0xB6,0x82,0x50,0xB1,0xC1,0x6F,
  0x6F,0x76,0xC0,0xF3,0x1C,0xD7,0xA4,0x60,0xFB,0xED,0x3C,0x7F,0x5A,0x52,0xFB,0x2D,0xDD,0xAD,0x2D,0x6D,0xE2,0x8E,0x47,0x19,
  0x96,0x45,0x3C,0x7F,0xDA,0xBC,0xFF,0x00,0x3A,0xA3,0xD7,0x42,0xA4,0x99,0x1B,0xF8,0x97,0xCF,0x59,0x9A,0x69,0x5A,0x20,0x33,
  0xBA,0x49,0x77,0xB6,0x7E,0x7E,0x69,0x62,0xEA,0xC4,0x4C,0xEA,0xCC,0x58,0x0F,0xC3,0x91,0x40,0x6A,0xE6,0xEA,0xCE,0xE3,0x7F,
  0x9D,0x19,0x0E,0x36,0xED,0x53,0x90,0xBF,0x14,0xBD,0x65,0x42,0x9E,0x63,0x03,0xE6,0xFC,0x53,0x46,0x5A,0x33,0x5B,0x34,0x50,
  0xDE,0x4B,0x34,0xA5,0xE0,0x5C,0x38,0xEA,0x09,0xEB,0x45,0xAD,0xD3,0x22,0xF9,0x84,0xB0,0x94,0xF0,0x45,0x64,0x2D,0xE6,0x91,
  0xE6,0x26,0xDD,0xA4,0x57,0x1C,0x9C,0x9E,0x28,0xE5,0xD6,0x0E,0xCF,0x29,0xDF,0x32,0x9E,0xB9,0xA7,0x53,0x17,0x89,0xFA,0x19,
  0xED,0xD5,0x70,0x4F,0x53,0x52,0x8A,0xDC,0x9E,0xFC,0x51,0x13,0x6D,0x72,0x1B,0xB5,0x78,0xAD,0x8E,0x73,0x8A,0xF2,0xD3,0xAD,
  0x16,0xAB,0x2A,0x5D,0xC8,0xC4,0x0A,0x36,0x3B,0x85,0x11,0x90,0xE7,0x9A,0x06,0x63,0x21,0x42,0xD1,0x29,0x73,0xDF,0x14,0x18,
  0x91,0x4F,0x0F,0x32,0x86,0xEE,0xAB,0xEA,0x3F,0xCB,0x8F,0xE7,0x46,0xC0,0x90,0xD5,0x67,0x49,0x65,0x0B,0xD8,0xD3,0x14,0xBA,
  0x82,0xDA,0x23,0xB5,0x46,0x7D,0xC6,0x69,0x0C,0x53,0x2C,0x7F,0x81,0x1C,0x93,0xDC,0x9D,0xB9,0xFD,0x3F,0xBD,0x79,0x75,0x7C,
  0x2D,0xE3,0x2C,0xDE,0x48,0x38,0xE3,0xD3,0x9F,0xE6,0x73,0x55,0xC5,0x24,0x81,0x22,0x9D,0x4A,0xF3,0xEA,0xAF,0x3C,0x85,0x49,
  0x24,0x91,0x8E,0x42,0xED,0xC9,0xC7,0xBF,0x3C,0x01,0xF7,0xA7,0xDA,0x36,0x97,0x69,0xA6,0x58,0x4B,0x79,0x71,0x1E,0x24,0x2B,
  0xC9,0xC6,0xFC,0xE3,0xFE,0xD1,0x5C,0xE7,0xC3,0xFA,0xE5,0xF6,0xA3,0xAF,0xB2,0xD9,0x5A,0xDD,0x5D,0x46,0x64,0xD8,0xF2,0xA3,
  0xF9,0x31,0x28,0xCF,0xC0,0xC9,0xFB,0x9F,0xE5,0x5B,0x5F,0x1E,0x5D,0xC3,0x2A,0x5A,0xE8,0xE1,0x72,0xB7,0x18,0x47,0x55,0x90,
  0x2B,0x85,0xF7,0x19,0xE6,0xBA,0xDB,0xD0,0xAA,0x22,0xBB,0xFB,0x4D,0x2F,0x5F,0x6F,0x3A,0xEE,0xC4,0x24,0x6A,0x7D,0x05,0xED,
  0x1D,0x09,0x3F,0x0C,0xDB,0x5B,0x3F,0x6A,0xCF,0xFF,0x00,0xFA,0x33,0x0D,0xC0,0x9A,0xFB,0x46,0x96,0x48,0x09,0xC9,0x8E,0x2B,
  0xC6,0x6F,0xC5,0xFE,0xE2,0x41,0x27,0x1E,0xCB,0xD2,0xBA,0x0D,0x9F,0x85,0xB4,0x98,0x6C,0x2D,0x6D,0xE7,0x8C,0x79,0x51,0x11,
  0xB1,0x5C,0x8D,0xC4,0xFE,0x55,0xA7,0xBC,0x88,0x88,0x12,0x28,0x26,0xF2,0xD5,0x47,0x6C,0x0C,0x54,0x25,0x27,0x1B,0x68,0xA4,
  0x55,0xE8,0xE6,0xBE,0x0B,0xF0,0x8E,0xB9,0xE1,0x48,0xDE,0x2B,0x7B,0x4D,0x36,0x4F,0xA9,0x1F,0xBF,0x78,0xDD,0xB7,0x3B,0x93,
  0xF8,0x98,0x9C,0xF0,0x05,0x01,0xFB,0x48,0xFD,0x9C,0x4F,0xAD,0x44,0xD7,0x1F,0xFD,0xC1,0x7B,0x6B,0x3A,0x46,0x40,0x8D,0x5B,
  0x31,0x9F,0x92,0xA4,0x56,0xE6,0x3B,0xBB,0x81,0x29,0x41,0x7C,0x5C,0x16,0x3E,0xA1,0x1F,0x23,0xF3,0xFF,0x00,0xD0,0xA5,0x1E,
  0x23,0xBC,0x30,0xC6,0xC6,0x79,0xDC,0x46,0x88,0x72,0x1B,0xD5,0xD0,0xF2,0x4F,0x3D,0x7E,0x2B,0x92,0x30,0x53,0x97,0x37,0x76,
  0x74,0xF3,0x94,0x15,0x2D,0x1F,0x98,0xFC,0x68,0xCD,0xA1,0xA5,0xB6,0x95,0x60,0xCA,0x96,0xD0,0x8D,0xBB,0xD7,0x21,0xCB,0x63,
  0x92,0x7D,0xF2,0x72,0x49,0x07,0xBD,0x68,0xBC,0x37,0xE2,0x88,0x2C,0xF4,0xCB,0x55,0xB7,0x95,0xA2,0x89,0x86,0x64,0x8C,0x67,
  0x2C,0xE3,0x1C,0x9F,0x9E,0x3E,0xDD,0x3A,0x56,0x3B,0xC7,0x9A,0xE5,0x8E,0xAF,0xAA,0x3C,0x56,0x8E,0xBB,0x21,0x91,0xF7,0xCE,
  0xC7,0x87,0x24,0xFF,0x00,0x08,0x1D,0x87,0xC5,0x24,0xBA,0xD4,0x4D,0x8D,0xCC,0x30,0xE9,0x9A,0x91,0xB9,0x8D,0x82,0x16,0x71,
  0x16,0xDF,0x59,0x18,0x23,0x07,0x9E,0x33,0x8A,0xEF,0x8C,0x14,0x51,0x29,0xE5,0x9E,0x4F,0xFA,0x67,0xE9,0x9F,0x0F,0xF8,0x92,
  0x4B,0x8F,0xDD,0x90,0xDF,0x53,0x2B,0xB3,0x98,0xD7,0xF0,0x8C,0xFB,0x9D,0xC3,0x1D,0xBE,0x79,0xE9,0xC5,0x34,0x7B,0xB3,0x67,
  0x6E,0x2D,0x93,0x81,0x8D,0xE0,0xB4,0x8A,0x4F,0x27,0x90,0x31,0xCF,0x07,0xE3,0xBD,0x72,0xDF,0x1B,0xDA,0xEB,0x1E,0x02,0xD3,
  0x6C,0x66,0x8A,0xEA,0x29,0xE6,0xBD,0x95,0x6D,0xF7,0xCA,0x80,0x31,0x62,0x09,0x18,0x23,0xA6,0x38,0xFE,0x54,0x92,0xCF,0xC5,
  0xFA,0xBE,0x99,0x76,0x96,0x5E,0x27,0x1B,0x43,0xFA,0x96,0x44,0x1C,0x48,0x3D,0x8E,0x3B,0xFC,0xF1,0x5C,0xF5,0xCE,0x37,0x00,
  0xA5,0x4F,0xFA,0x3B,0x7E,0x9C,0x57,0xCC,0x37,0x0A,0xB7,0x03,0x3C,0x96,0x8B,0x69,0x51,0xF7,0x20,0x52,0x5F,0x13,0x59,0x5F,
  0xEA,0xD1,0xB2,0x45,0x75,0x15,0xC4,0x41,0xF7,0x18,0xEE,0x37,0xF3,0xF1,0x90,0x4F,0x15,0x67,0x87,0xAF,0xA2,0xBC,0xB2,0x17,
  0x56,0x97,0x0E,0x80,0x8F,0x4B,0x0E,0xA3,0xDC,0x1C,0x75,0x1F,0xF0,0x45,0x36,0xB6,0x6F,0x36,0x76,0x5B,0x88,0xBD,0x6A,0x72,
  0x24,0x8C,0x8C,0x36,0x7D,0xC5,0x4F,0x0B,0x69,0x53,0x29,0x35,0x6E,0xD0,0x25,0xB5,0xA5,0x8D,0xB5,0x84,0x76,0xFF,0x00,0xB8,
  0x83,0xCC,0x5D,0x8C,0x63,0x2A,0x54,0x7E,0x67,0xFE,0x6B,0x9B,0xEA,0x36,0x57,0xDE,0x16,0xD6,0xDA,0x5B,0x7B,0xA3,0x3D,0xB3,
  0x1D,0xC1,0x04,0x48,0xA8,0x17,0x3D,0xCA,0x91,0xFC,0xEB,0xAD,0x5C,0xE9,0xB6,0xB2,0x48,0xB2,0x4D,0x11,0x91,0xC1,0xF4,0xB0,
  0x38,0x23,0xF4,0xEB,0xF6,0xAC,0x67,0xED,0x33,0xC3,0xD1,0xDF,0x58,0x7D,0x4A,0x46,0xD1,0x88,0xF3,0x8F,0xA6,0x45,0x2E,0x7F,
  0x98,0xFD,0x0F,0xE9,0x5D,0x78,0xDF,0x87,0x3C,0x97,0xA6,0xA3,0x4A,0xBB,0x5B,0xFD,0x32,0x39,0xA2,0x87,0xF7,0x8C,0xB9,0x63,
  0x11,0x03,0x23,0xDF,0xB8,0xFF,0x00,0x9A,0xE6,0x7F,0xB4,0xBD,0x0A,0x25,0xB4,0x92,0xF3,0x4F,0xB1,0xB3,0xCB,0xE7,0x7B,0xC5,
  0x09,0xF3,0x09,0xEB,0x83,0x8C,0x64,0xFE,0xB4,0x37,0xEC,0xBF,0xC4,0x6B,0x6B,0x76,0xDA,0x5D,0xED,0xDB,0x67,0x3E,0x85,0x92,
  0x33,0x19,0xFC,0xFA,0x66,0xBA,0xB5,0xEE,0x9E,0xBA,0x8D,0xA4,0x91,0xDD,0x20,0x75,0x71,0xB5,0x65,0x8D,0x88,0x71,0xED,0x8F,
  0x7F,0xCE,0xA8,0x9D,0x13,0x6A,0x8F,0xCA,0xB1,0xDF,0x33,0x05,0x42,0xC6,0x32,0xA7,0x1B,0x54,0x11,0x83,0xEF,0xCF,0x7A,0x7D,
  0xA2,0xEB,0x7A,0x96,0x9F,0x22,0xAC,0x1B,0x27,0x8D,0x8E,0x76,0xCE,0x80,0x82,0x3D,0xB9,0xA7,0xBE,0x38,0xD1,0x75,0x7B,0x7B,
  0xC6,0x16,0xD7,0x1E,0x67,0x96,0x30,0x16,0x78,0x82,0x3E,0x07,0x70,0xDD,0xFF,0x00,0x3A,0xC1,0x4B,0x75,0x2E,0x51,0x6F,0xE1,
  0x2E,0xD9,0xC8,0x25,0x88,0x27,0xEC,0x47,0x51,0xF6,0xA6,0x66,0x5B,0x47,0x7E,0xD0,0xF5,0xA5,0xBF,0xB0,0x54,0x44,0x8A,0xDA,
  0xE7,0x6F,0x3E,0x52,0x02,0x07,0xE5,0xD4,0x56,0x7F,0xC4,0xD2,0x41,0x63,0x14,0x8E,0xD1,0xCF,0x2C,0xEF,0xFC,0x6D,0x9E,0xBF,
  0x9D,0x73,0x5D,0x02,0xF9,0xBE,0xA3,0x1E,0x63,0xC5,0x9C,0x95,0x7C,0x64,0x63,0xE7,0x3F,0xD6,0x9B,0x6A,0xF7,0x1A,0xBC,0xAA,
  0xB1,0x2C,0xF2,0x5C,0x02,0x32,0x0A,0xB6,0xEE,0x29,0x00,0xA3,0x44,0x2F,0x2F,0x1E,0xF1,0x91,0x23,0xC2,0xAA,0x8C,0xFE,0x20,
  0x31,0x56,0xC5,0xE5,0xA4,0x58,0xF3,0x11,0x8F,0xF1,0x0C,0xD0,0xFA,0x5C,0x72,0x03,0xB5,0x82,0x42,0xE7,0xA9,0xDA,0x39,0xA6,
  0x12,0x59,0xCC,0x81,0x89,0x64,0x97,0xFF,0x00,0xF3,0x50,0x09,0xA0,0x9F,0x81,0x62,0xA7,0x92,0x44,0x19,0x43,0xB5,0x09,0xE0,
  0xE2,0xAB,0x12,0xEF,0x6F,0xDE,0x0F,0x50,0xE8,0xDE,0xF5,0x19,0x05,0xE2,0x46,0xDE,0x6A,0x30,0x5C,0xF1,0xBC,0x8E,0x28,0x64,
  0x86,0x54,0xC1,0x2E,0x18,0x13,0xC8,0xCF,0x4A,0x3D,0x05,0x1F,0xAE,0xAD,0x55,0xE5,0xE4,0x8F,0x4F,0x73,0xD8,0x54,0x6E,0x36,
  0x89,0x02,0xAF,0xAB,0xEF,0xD3,0xF4,0xEF,0x57,0xC4,0x36,0x2E,0x3A,0xFB,0x0A,0xF4,0x22,0xC9,0x81,0xC6,0x6B,0xCF,0x6A,0xC7,
  0x5A,0x07,0x7F,0x31,0xA3,0x2A,0x4E,0x47,0xB7,0x61,0xF9,0x50,0x90,0xD9,0xAA,0xB1,0x70,0x37,0x31,0x3D,0x00,0xA6,0xA2,0x30,
  0x88,0x58,0xB0,0x0A,0x38,0x24,0xF4,0x15,0x49,0xBB,0x48,0xF7,0x2D,0xBA,0x90,0x48,0xE5,0xFF,0x00,0x88,0xFF,0x00,0x6F,0xCA,
  0x8A,0xFF,0x00,0x4C,0xC8,0xFD,0x30,0x24,0x2C,0xCC,0x11,0xBF,0xDA,0x39,0x6F,0xFC,0x7E,0x75,0x46,0xA3,0x1D,0x9D,0xBC,0x0E,
  0xCF,0x0A,0xCA,0xD8,0xE1,0x5F,0xD4,0x4F,0xE5,0xD2,0xA8,0x32,0x1D,0xD9,0x2E,0x10,0x51,0x11,0xBC,0x32,0x82,0x90,0x07,0x96,
  0x43,0xC1,0xC0,0xC6,0xE3,0xED,0x9A,0x7C,0x75,0x62,0xB1,0x7F,0x82,0x96,0x79,0x2E,0x5F,0x51,0x98,0x15,0xB6,0x88,0x10,0x88,
  0x5C,0xA2,0x0F,0xB2,0xF0,0x38,0xA5,0xFE,0x1D,0xD7,0xFF,0x00,0xD5,0xFC,0x65,0x71,0x38,0xB1,0x58,0xA1,0x1E,0x88,0x2E,0x1D,
  0x89,0x79,0x48,0x3C,0x95,0x51,0x9C,0x8F,0x9F,0xE7,0x4E,0xBE,0x9C,0x2E,0x9F,0x7A,0x27,0x91,0x37,0x85,0x22,0x45,0x8B,0xF8,
  0x47,0xB1,0x27,0x20,0x7F,0x33,0xF1,0x55,0x78,0x1D,0x62,0x8D,0x1D,0x2D,0x60,0x5B,0x40,0xC7,0x6F,0xE2,0xCC,0x8E,0x07,0x77,
  0x63,0xC8,0x1E,0xCB,0xD0,0x57,0x4C,0x98,0x22,0xAC,0xD7,0xCD,0x68,0x24,0x9D,0x65,0x95,0xD9,0xE4,0x41,0x90,0xBB,0xF6,0x81,
  0xF7,0xC7,0x6F,0x8A,0x2E,0xE3,0x26,0x01,0x85,0x0C,0xDD,0xD7,0xA6,0x7F,0xCF,0x9A,0x82,0x2A,0x85,0xDB,0x0F,0xA7,0xDD,0x8F,
  0x27,0xEF,0xF7,0xF6,0x1D,0xA8,0x6B,0x9B,0x90,0x19,0x95,0x46,0x51,0x07,0x3C,0xFE,0x23,0xED,0xFE,0x75,0xAE,0x79,0xF4,0xCB,
  0x43,0xB1,0x36,0xAD,0x7C,0xAB,0x1C,0xD2,0x2B,0x08,0x56,0x10,0x59,0xDB,0xB1,0x1D,0xCF,0xF9,0xF7,0xAE,0x23,0xE2,0xAD,0x4F,
  0x53,0xF1,0x8D,0x86,0xAE,0xDE,0x19,0x56,0x36,0x31,0x45,0x20,0x8D,0x98,0x9C,0xCA,0x50,0x7A,0xFC,0xBF,0xCB,0x9F,0x9E,0x6B,
  0x7B,0xFB,0x50,0xB4,0xD4,0xB5,0x4F,0x0E,0xEA,0x71,0x69,0xEA,0x54,0x94,0x08,0xBB,0x3A,0xB1,0xEE,0x3F,0xAD,0x72,0x9F,0xD9,
  0xE6,0xBD,0x2F,0x83,0x25,0xB0,0xB3,0xD5,0x6D,0xA7,0x48,0xC6,0xAE,0xB3,0x34,0x8E,0x85,0x76,0x45,0x24,0x6D,0x1C,0x8A,0x47,
  0x7E,0x4A,0xB0,0xC7,0xB1,0xA5,0xC0,0xDD,0x5B,0xEC,0xA4,0xE3,0xF0,0xE3,0xD3,0x59,0xDC,0x5A,0xAA,0xFD,0x44,0x45,0x41,0x00,
  0xFE,0xB5,0x01,0x13,0xBB,0x22,0xC2,0xA4,0xC8,0xC4,0x04,0x51,0xD4,0xB6,0x78,0x1F,0xAE,0x2B,0xA9,0x7E,0xDA,0x74,0x24,0xB7,
  0xF1,0x55,0xC3,0x68,0xF3,0xC3,0x79,0x69,0x38,0xF3,0x62,0x8A,0xD7,0x0E,0x62,0x39,0xE5,0x4E,0xDE,0x83,0xB8,0x27,0xDF,0x1D,
  0xA9,0x27,0xEC,0xB6,0xCE,0xCC,0x7E,0xD0,0x34,0x79,0x7C,0x43,0x34,0x76,0x3A,0x7D,0xB4,0xC2,0xE2,0x57,0xB8,0xF4,0x86,0x29,
  0xEA,0x55,0xFB,0x96,0x00,0x7E,0xB5,0xDC,0x9D,0xC6,0xC8,0xB8,0xD1,0xD1,0x7F,0xFA,0xA8,0xB8,0x95,0x24,0xF0,0xC5,0xA3,0xE5,
  0x67,0x8A,0x33,0x3C,0x80,0x7F,0x0B,0xED,0x50,0x7F,0x43,0x9A,0xE5,0x09,0x26,0xA3,0xE2,0xFF,0x00,0x10,0x8B,0xA9,0xE3,0x22,
  0x33,0x84,0x63,0x1E,0x42,0x44,0x02,0x9C,0x00,0x4F,0x7E,0x33,0xF3,0x5B,0xCF,0xDB,0x1E,0xB1,0x6D,0xE3,0x6F,0x1C,0xDD,0x35,
  0x84,0xCB,0x2C,0x16,0x70,0xAC,0x50,0xB2,0xB7,0xA6,0x6E,0xAC,0xE4,0x1F,0xB9,0xC7,0xE5,0x5C,0xFD,0x3E,0xAA,0x2B,0x77,0xB3,
  0xB0,0x9A,0xE0,0x5A,0x3B,0xEF,0x68,0x90,0x75,0x3D,0x32,0x71,0xCD,0x2E,0x25,0x51,0x34,0x93,0xB3,0x7B,0xFB,0x28,0xF1,0x1B,
  0x5A,0x4D,0x2D,0x85,0xCC,0xA0,0xC6,0xA3,0x38,0x27,0x2A,0x08,0x3F,0xFB,0xFC,0x8D,0x76,0xDD,0x3E,0xE9,0x65,0x24,0x80,0xB9,
  0x43,0xB5,0xC7,0x52,0xBE,0xC7,0xE5,0x48,0xEF,0xFA,0xF4,0xAF,0xCF,0xFE,0x08,0xF0,0xB6,0xAB,0xF5,0x29,0x79,0x0C,0x6F,0x6C,
  0x91,0x72,0x0B,0x8E,0x79,0xEE,0x6B,0xBB,0x68,0x28,0xF7,0x96,0xCA,0xD3,0x44,0x2D,0xEE,0x61,0xF4,0x3B,0x47,0xD0,0xE3,0xA1,
  0xF8,0x1D,0x3F,0xCE,0x29,0x27,0x15,0x7A,0x0A,0x93,0x35,0x16,0xAE,0x1B,0x2A,0x14,0x6F,0xC6,0x36,0xB1,0xE2,0x86,0xD7,0x34,
  0xFB,0x4B,0xDB,0x59,0x52,0xE9,0x59,0x32,0xB8,0x6E,0xD8,0xFB,0xFB,0x8A,0x2E,0xD6,0x19,0x3C,0xB5,0x62,0xCA,0xD8,0xF7,0xFF,
  0x00,0x8A,0xB2,0x56,0x3E,0x5A,0x09,0xB9,0x4C,0xE0,0x13,0xFD,0xE8,0x20,0x33,0xF3,0x47,0x8D,0xF4,0x4B,0xCD,0x0F,0x53,0x69,
  0x6D,0xE7,0x49,0xA2,0x5E,0x54,0x39,0x66,0x20,0x7B,0xE4,0x7F,0x51,0x83,0xEE,0x3B,0x9D,0x27,0xEC,0xFF,0x00,0xC6,0x0D,0x38,
  0x8E,0xDE,0xF1,0xC4,0xC7,0xF0,0x8C,0x4A,0x18,0x0F,0xCC,0x10,0x7F,0x5A,0xD6,0x7E,0xD2,0xBC,0x21,0x6F,0xAB,0x2B,0x5C,0xD8,
  0x3F,0x95,0x76,0xBE,0xA6,0x0A,0xDB,0x49,0x38,0xE0,0x8E,0xDF,0xF1,0x5C,0x29,0xED,0x6F,0xB4,0x5D,0x75,0x7E,0xA1,0x4A,0x4B,
  0x9C,0x87,0x0B,0xB4,0x9F,0x92,0x2A,0xA9,0xDA,0x12,0x8E,0xFD,0xAE,0x69,0x63,0x53,0xB4,0x38,0xBF,0x96,0x35,0x23,0x2A,0x08,
  0x07,0x1E,0xD8,0x62,0x2B,0x91,0x6B,0xDA,0x1C,0xB1,0xCC,0x60,0x3F,0x48,0xC5,0xF2,0xA0,0x32,0xA8,0x7C,0x8F,0x95,0x38,0x6F,
  0xEB,0x5D,0x3F,0xC1,0xD7,0xC9,0xA9,0xE8,0xEA,0x4D,0xB4,0x4D,0xC1,0x0E,0x3A,0x16,0x23,0xA9,0xC1,0xEF,0xF1,0x58,0x8F,0x1E,
  0x68,0xA8,0x97,0x06,0x5B,0x25,0xB4,0x96,0x09,0x7F,0x12,0x60,0x14,0x39,0xED,0xC7,0xE1,0x3F,0x07,0xF2,0x23,0xA5,0x34,0x5D,
  0x8A,0xD5,0x33,0x07,0x0D,0x85,0xC5,0x96,0xEB,0x5B,0xA9,0x21,0x8C,0x39,0xE3,0x2C,0x06,0x3F,0x5C,0x11,0x4C,0x6C,0x22,0x61,
  0x27,0xD3,0xCF,0x32,0xED,0xEA,0xA4,0x11,0x9F,0xC8,0xD2,0xB7,0x96,0x5B,0x47,0x6B,0x66,0x59,0x54,0x06,0xFF,0x00,0xF8,0xD7,
  0x20,0x4B,0x1B,0x0F,0x8D,0xDC,0xFE,0x87,0x3F,0x26,0x99,0xD8,0x3C,0x71,0x40,0xEB,0x14,0x65,0x37,0xFE,0x2B,0x79,0x7D,0x48,
  0x3D,0x8A,0x9E,0xA0,0x7B,0x7F,0x53,0x41,0x85,0x05,0x5C,0x59,0xFA,0x7D,0x44,0x10,0x0F,0xE2,0x03,0x69,0xFC,0xEB,0xC1,0xF4,
  0xAB,0x10,0x32,0xAC,0xCC,0x41,0xFC,0x42,0x4C,0x83,0x54,0x1B,0x89,0x19,0x36,0xCC,0x24,0x8F,0x1E,0x90,0xFD,0x76,0xFD,0xFE,
  0x2B,0xC9,0x34,0xF9,0x56,0xD9,0xE7,0x1F,0xBC,0x43,0xC8,0x68,0xCF,0xFC,0x57,0x3C,0x9E,0xCA,0x25,0xA2,0x1A,0x98,0xB7,0x90,
  0x05,0x8D,0x42,0xFC,0x8C,0x9A,0x03,0x63,0xA0,0xC4,0x87,0xD2,0x0F,0x5A,0x22,0x77,0x12,0x32,0x26,0xFD,0xA5,0x87,0x05,0xBB,
  0xD7,0x84,0xC8,0xAC,0x16,0x6C,0x31,0xC7,0x5A,0x68,0xB7,0xE8,0x1A,0x47,0xEB,0x37,0x53,0xB5,0x8A,0x9E,0x45,0x46,0x1C,0xAA,
  0x92,0xE7,0x09,0xF1,0xDC,0xFB,0x0A,0x1A,0x39,0xFC,0xD3,0xB7,0x90,0x83,0x96,0x3E,0xD5,0x19,0x2E,0xFD,0x65,0x54,0x05,0x54,
  0xE0,0x0F,0x6A,0xE2,0xD0,0xC5,0xF7,0x32,0xBC,0xE4,0x28,0xF4,0xA8,0xE8,0x07,0x41,0x55,0xDB,0xC1,0x23,0x16,0xC8,0xE8,0x6A,
  0x50,0x4A,0x93,0x2E,0x0E,0x06,0x7B,0xD3,0x5B,0x20,0x8E,0x84,0x82,0x00,0x1D,0x49,0xA0,0xB6,0xCD,0xE0,0x22,0x69,0xBB,0x90,
  0xB3,0x61,0x40,0x19,0x2C,0x7A,0x0A,0x8C,0x48,0xDC,0xA5,0xB6,0x63,0x53,0xC1,0x97,0x1E,0xB6,0xFB,0x7B,0x0A,0x26,0xFE,0xED,
  0x15,0x76,0x13,0xB5,0x47,0x20,0x7F,0xC9,0xA5,0xD6,0x5A,0x8F,0x9B,0x39,0x11,0xA9,0x2A,0xBD,0x30,0x3A,0x9F,0x6A,0xBC,0x2A,
  0xF4,0x23,0x2D,0xB9,0xB0,0x71,0x61,0x73,0x04,0x0B,0xE5,0xC2,0x47,0x27,0x3C,0xBB,0x7D,0xFD,0xA8,0x6F,0x02,0xE8,0xD1,0x69,
  0x56,0x8C,0xF2,0xCA,0x26,0xB8,0x98,0x92,0x08,0xE8,0x14,0x1F,0xFC,0x7F,0x2A,0x6D,0x70,0xE2,0x78,0x8D,0xBB,0xC8,0xA1,0x9C,
  0x61,0x82,0x9F,0xC0,0xBD,0xFF,0x00,0x3C,0x52,0x8D,0x26,0x79,0x22,0xBB,0x75,0x99,0xD6,0x38,0xB7,0x05,0x8E,0x3F,0xF6,0xA8,
  0xE8,0x3F,0x41,0x56,0x69,0x30,0x2D,0x1A,0xB4,0x76,0x92,0x43,0x1A,0x72,0x07,0x00,0x03,0xD7,0xDC,0x93,0x57,0x1B,0x2D,0x99,
  0x39,0xC9,0xEA,0x78,0xA8,0xE9,0x43,0x31,0x99,0x0E,0x42,0xF6,0x3D,0xDB,0xE6,0x98,0x6D,0x0D,0xF8,0x8F,0x07,0xB5,0x2B,0x8D,
  0xA1,0x94,0xA8,0x54,0xD6,0x0A,0xF1,0x2A,0xBA,0xF0,0x3F,0x87,0xAF,0xEB,0x58,0x5F,0xDA,0x4F,0x83,0xC6,0xB9,0x2D,0x85,0xBA,
  0x34,0x90,0x46,0x8D,0xE7,0x48,0xF1,0xA0,0x25,0x80,0xE8,0xB9,0x3C,0x0E,0x6B,0xA8,0x85,0x03,0xED,0xDA,0xAA,0xBE,0x55,0x30,
  0x9C,0x75,0xC7,0x6A,0x10,0x49,0x3D,0x85,0xC9,0xF8,0x7E,0x77,0x4F,0xD9,0xB7,0xAE,0x47,0x86,0x4D,0xAA,0xAA,0x0E,0x63,0x24,
  0x6D,0xC9,0xC6,0x58,0x1E,0x98,0xE7,0x9F,0x8A,0xCE,0x6B,0x3E,0x03,0x92,0xD8,0x11,0x27,0x98,0xE8,0xE4,0xB1,0xC7,0xAB,0x00,
  0x1C,0x13,0x9E,0x83,0xBF,0xFE,0x6B,0xF4,0x55,0xCD,0xA1,0x58,0xE7,0x5D,0xA1,0x43,0x28,0x39,0xC7,0x2C,0x7E,0x69,0x16,0xA9,
  0x6C,0xD7,0x50,0x24,0x44,0x14,0x57,0xF4,0xFE,0xEF,0x81,0x80,0x38,0x1F,0x6E,0xB5,0xD3,0x19,0x21,0x1D,0xB3,0xF3,0xD5,0xCF,
  0x80,0xAF,0x2D,0xAD,0xD2,0xFB,0x48,0xB8,0xDD,0x2E,0x77,0x24,0x6E,0x36,0x80,0x0F,0x4C,0x31,0xEF,0xF0,0x45,0x3E,0xFD,0x98,
  0xF8,0x37,0x52,0xB9,0x79,0x6E,0xA6,0xB7,0x78,0x4C,0x33,0xE1,0xE3,0x90,0x63,0x77,0x19,0xE0,0xF7,0xAE,0xBF,0xA5,0x58,0x7D,
  0x18,0xFA,0x69,0xF6,0x32,0xEE,0x2C,0xBE,0x9E,0x99,0xFC,0x43,0x1D,0x30,0x33,0x4C,0xE1,0x86,0x3D,0x3E,0x12,0xB6,0xEA,0x00,
  0x52,0x4E,0xD5,0xE2,0xB4,0xA4,0x91,0xAD,0xBD,0x0B,0xA1,0xD3,0xFC,0x82,0xA4,0xC0,0x31,0xD0,0xAC,0x6D,0xC7,0xE9,0xD2,0x9B,
  0xDA,0xD9,0xC2,0x92,0x09,0x23,0x1B,0x7E,0x08,0xC7,0xDC,0x50,0x76,0xF3,0xAC,0xC4,0xED,0xDD,0x26,0xEE,0x46,0xE2,0x3F,0x4F,
  0x7A,0x65,0x6C,0x54,0x8C,0x05,0x60,0x71,0xC8,0x3F,0xE7,0x35,0xCD,0xCA,0xDD,0x94,0x4A,0x90,0x7C,0x11,0x81,0x19,0x20,0x85,
  0xF7,0x14,0x15,0xCB,0x6C,0x0C,0xAE,0x83,0x6B,0x8E,0xA7,0xA1,0xFB,0xFF,0x00,0x7A,0x2D,0xB7,0x79,0x27,0x61,0x00,0xF5,0x19,
  0xE9,0x54,0x3E,0x26,0x87,0x64,0x80,0xA9,0xEA,0x08,0x38,0x2A,0x7F,0xCF,0xCA,0xAD,0x14,0x4D,0xB1,0x06,0xA3,0xA7,0xC3,0x77,
  0x0B,0x24,0x44,0x2B,0xAF,0x21,0x49,0xCF,0x03,0xB0,0x35,0xC6,0xBC,0x6F,0xE1,0xCF,0x22,0xF1,0xC1,0xB9,0x21,0xFF,0x00,0x14,
  0x69,0x3F,0xA4,0x1F,0x8C,0xF4,0xCF,0xB3,0x03,0xF0,0x7E,0x3B,0x3E,0xA2,0x86,0xDC,0x3C,0x88,0x09,0x0A,0x32,0x40,0xE8,0x7E,
  0x7E,0x0D,0x73,0x9F,0x19,0x5C,0x41,0xAA,0xD9,0x4F,0x04,0xAF,0x1C,0x88,0x8D,0x94,0x56,0x1B,0x5E,0x23,0xEC,0x47,0x1C,0xFB,
  0x30,0x23,0xD8,0xFB,0xD5,0x22,0x81,0xC8,0x45,0xE0,0xCD,0x62,0x3B,0x66,0xFA,0x3B,0xA8,0xDE,0x39,0xF1,0x85,0x75,0x1B,0x19,
  0xFF,0x00,0x31,0xC1,0xC7,0xEB,0xFD,0x6B,0xCD,0x69,0xA2,0xBB,0xBD,0x96,0x52,0xEF,0x6B,0x72,0xD9,0x06,0x58,0x8E,0x7C,0xC1,
  0xD3,0xD4,0x08,0xC3,0x7C,0xAB,0x0F,0xCC,0x56,0x1E,0xE5,0xEE,0x2D,0x19,0xA4,0x87,0x2F,0x1A,0x36,0xD7,0xF5,0x12,0xB9,0x1D,
  0x37,0x0F,0xC4,0x8D,0xF2,0x46,0x0F,0xBD,0x30,0xF3,0xCD,0xF9,0xF3,0x6E,0x09,0x2A,0xE0,0x8D,0xE7,0xAA,0x1D,0xA7,0x83,0x8E,
  0xE3,0xF2,0x38,0xE4,0x1A,0x29,0x51,0x9E,0xCB,0x0C,0x26,0x55,0x78,0x2E,0x55,0x66,0x8D,0x78,0xE0,0x8C,0xAF,0xC7,0x27,0x91,
  0xEC,0x73,0x91,0xD0,0xD0,0x97,0x56,0xF2,0x5B,0xC7,0xE8,0xDB,0x2A,0x1F,0x4A,0x48,0xCB,0x82,0xA7,0xFD,0xAE,0x3D,0x8F,0xBD,
  0x42,0xD1,0x2F,0x74,0xD9,0x1A,0x5F,0x2C,0xDD,0xDA,0x81,0xEA,0xC1,0xCB,0x2A,0x9E,0xE0,0xF7,0x1F,0x7A,0x35,0x6E,0x62,0x72,
  0x8D,0x1C,0x83,0x0F,0xE9,0xC3,0x71,0xE9,0xF6,0xFF,0x00,0xD5,0x06,0x64,0x07,0x67,0x32,0x39,0x10,0x3B,0x4B,0x0C,0x87,0xA0,
  0x7E,0x47,0xDA,0xAD,0xBC,0xB9,0xB8,0x89,0xCA,0xC6,0x8A,0xB2,0x46,0x79,0xDA,0x71,0x9F,0xCA,0xA1,0x76,0x92,0xED,0x56,0x21,
  0x24,0xDA,0xDC,0x38,0x5E,0x47,0xDC,0x8A,0xA8,0xCE,0x2E,0x98,0xA5,0xCA,0x16,0x70,0x70,0x1D,0x4F,0x35,0x06,0xBD,0x2A,0x8A,
  0xE4,0x95,0x26,0x6C,0xCD,0x02,0xF9,0x8B,0xC9,0xCF,0x1F,0xA5,0x52,0xCA,0x06,0x1C,0x29,0xC1,0x3E,0xF9,0xAF,0x40,0x61,0x31,
  0x00,0x86,0x1D,0x06,0x78,0x3F,0x9D,0x57,0x18,0x9A,0x40,0xCA,0xC3,0x1C,0xFA,0x4A,0xF4,0xFB,0x50,0x89,0x9D,0x1F,0xAC,0xE2,
  0x89,0x7C,0xB2,0x13,0x05,0x07,0x39,0xF7,0x34,0xB2,0xF9,0x41,0x0D,0xBB,0x81,0xF1,0xDE,0xAD,0xB1,0xBA,0xDB,0x68,0x82,0x4C,
  0x8D,0xDD,0x6A,0x37,0x17,0x70,0x3E,0x37,0x0E,0x07,0xF3,0xAE,0x7E,0x49,0x1B,0x8B,0x65,0x10,0x6F,0x55,0x0A,0x73,0x8F,0xE1,
  0xAB,0x1B,0x52,0x65,0x0D,0x12,0x29,0xDA,0x87,0xD6,0x47,0x73,0xFF,0x00,0x8A,0x3E,0x1B,0x50,0xD6,0xFB,0x87,0x0F,0x8C,0xAF,
  0xC5,0x56,0x96,0x05,0x97,0x18,0x18,0xFE,0x21,0x59,0x24,0x0B,0x11,0x6A,0x52,0x5C,0x5D,0x2B,0x34,0x2E,0xCA,0x7A,0x0F,0x6A,
  0x8D,0x84,0x17,0xC1,0x42,0xBC,0xA5,0x50,0x75,0x73,0xD4,0xFD,0xA9,0xC4,0x96,0xC5,0x4E,0x17,0x80,0x7B,0xD3,0x0B,0x00,0xAC,
  0x98,0x54,0x5C,0x8E,0xB9,0xA0,0xAE,0xF4,0xCC,0x51,0xA5,0x5B,0x18,0xE0,0x91,0x94,0x10,0x5B,0x8D,0xC7,0xDB,0xBD,0x07,0x3D,
  0x91,0x1A,0x92,0x5D,0x48,0xCC,0x21,0x8F,0x20,0x01,0xFC,0x46,0xB4,0x08,0x9B,0x8E,0xD6,0x38,0x8C,0x75,0xF9,0xA0,0xF5,0x65,
  0x17,0x30,0xB4,0x76,0xD9,0x44,0xC6,0x0B,0x8E,0xC3,0xBE,0x2B,0xA2,0x1F,0x04,0x63,0xBD,0x36,0xE5,0x2E,0x22,0x00,0x60,0xA2,
  0x74,0x51,0xD2,0x9A,0x19,0x04,0x71,0xEE,0x61,0x96,0x3D,0x05,0x65,0xBC,0x38,0xCB,0x14,0x28,0xAC,0x02,0x20,0xFC,0x09,0xEE,
  0x3D,0xCD,0x3C,0x6B,0x80,0x41,0x24,0x82,0xDD,0x87,0x61,0x54,0xA0,0x20,0xAF,0x37,0x18,0x2D,0xED,0xC9,0xF6,0xAF,0x4B,0x17,
  0x42,0xA3,0x82,0x3F,0x5A,0x5A,0x93,0x99,0x27,0xDA,0x0E,0x55,0x3B,0xFC,0xD5,0x9E,0x69,0xCB,0x05,0x3C,0xFB,0xD2,0x34,0x32,
  0x28,0xBF,0x90,0xAA,0x3E,0x70,0x4F,0x6C,0x7F,0x9F,0x14,0x95,0xA4,0x91,0x18,0xAF,0x18,0x56,0xC7,0x4E,0x3A,0x53,0x4B,0xB9,
  0x00,0x8D,0x98,0xAF,0x7E,0x29,0x6C,0xED,0x10,0x8D,0x9D,0x8E,0x02,0x8C,0x93,0x51,0x93,0x91,0x68,0xA5,0x45,0x0D,0x70,0xB2,
  0xB6,0x40,0xF5,0x06,0xE3,0xE0,0xD5,0x52,0xCD,0xE6,0x64,0x38,0xE7,0x1C,0x1C,0x74,0xA9,0x6C,0x52,0xFC,0x1F,0xC7,0xEA,0x1F,
  0xA5,0x7C,0x23,0xF3,0x14,0x02,0x39,0x3F,0xCA,0xB2,0x94,0xA5,0xA6,0x1E,0x29,0x10,0xB4,0x84,0x6E,0x66,0xC0,0x0C,0x4F,0xE2,
  0xF9,0xA6,0x56,0xF2,0xB0,0x53,0x90,0x72,0x3B,0x13,0xD6,0xA9,0xB6,0x8C,0xA3,0xF9,0x64,0x70,0x79,0x14,0x52,0xC6,0xA2,0x3D,
  0xA7,0x81,0xDA,0xA9,0x8E,0x14,0x24,0xE4,0x5A,0x92,0x8D,0xA1,0xA3,0x38,0x19,0x19,0x07,0xB7,0xDE,0xBC,0xDC,0xB1,0x9C,0x36,
  0x40,0xEC,0x7D,0xBE,0x2A,0x08,0xA3,0x0C,0x73,0xCF,0x1C,0xE3,0x86,0x15,0x09,0x25,0x8D,0x9B,0xCB,0x72,0x57,0x70,0x20,0x6E,
  0xEF,0xF1,0xF7,0x15,0xD2,0x91,0x06,0x2F,0xD6,0xD0,0x25,0xAB,0x2E,0x71,0x8E,0x54,0x03,0xD3,0xE5,0x7F,0xB5,0x70,0xFF,0x00,
  0x18,0xDE,0x1B,0xCB,0xE9,0x6D,0xAE,0x50,0x5B,0x5F,0x47,0x80,0x93,0xA7,0xA4,0xFE,0x78,0xEA,0xA7,0xB1,0xFB,0x83,0x5D,0x97,
  0x5B,0xBB,0x54,0x84,0xF9,0x81,0xA4,0x44,0xE5,0x94,0x0C,0x9C,0x0E,0xE3,0xE4,0x75,0xAE,0x2B,0xFB,0x43,0x89,0x27,0xD5,0xA3,
  0x36,0xF3,0x2C,0x9B,0xCF,0xA5,0xD4,0xF5,0x53,0xC8,0xFF,0x00,0x3E,0x7E,0xD4,0xC8,0xC8,0xC7,0x5D,0xFD,0x54,0x77,0x59,0x39,
  0x8E,0xFC,0x29,0x57,0x53,0xF8,0x66,0x5F,0x6F,0xBF,0xCF,0x7E,0x33,0xD2,0xA7,0x6C,0xD3,0x2C,0xC5,0xED,0x93,0xCA,0x9C,0x2F,
  0x1D,0x92,0x54,0xFF,0x00,0x6B,0x0F,0x71,0xEF,0xDA,0xAC,0x59,0xFC,0xE4,0x7B,0x4D,0x5C,0x6E,0x45,0xE6,0x3B,0x85,0xE1,0xD0,
  0x74,0x07,0xF2,0x3C,0x1F,0xFD,0x51,0x36,0x96,0x53,0x09,0xC5,0xBC,0xF2,0xAB,0x48,0x0E,0xEB,0x6B,0x85,0x38,0xDD,0xF0,0x7B,
  0x1F,0xED,0x9A,0x21,0x19,0xE9,0x13,0xC6,0xF0,0xB7,0x9A,0x86,0x06,0x8C,0xFA,0xA3,0x24,0xFA,0x0F,0xB8,0xF6,0x07,0xF3,0x1D,
  0xF9,0xA1,0x2E,0xED,0x6D,0x2E,0x77,0xA8,0x63,0x05,0xC7,0x24,0x46,0x40,0xC3,0xFD,0x88,0xE3,0x3E,0xD4,0x44,0x2F,0xFB,0xD5,
  0xB5,0xB9,0x4F,0x22,0x55,0x38,0x8A,0x63,0xD8,0xFF,0x00,0xB1,0xBE,0x3D,0xA8,0x6D,0x54,0x46,0x8B,0x2B,0xB7,0xEE,0x65,0x5C,
  0x09,0x23,0x3F,0x84,0xFC,0xAF,0xB5,0x20,0x50,0xB9,0x64,0xFA,0x76,0x3F,0x50,0xFB,0xA3,0x61,0xD0,0xF0,0x4F,0xF6,0x35,0x4F,
  0xD3,0x1C,0x97,0x85,0x98,0x8C,0xE5,0x1B,0xDC,0x7F,0x7A,0xFA,0xF2,0x68,0x80,0x0B,0x20,0xF3,0x21,0x62,0x3D,0x6B,0xD4,0x66,
  0xA4,0xE5,0x99,0x3C,0x9B,0x66,0xFD,0xDB,0x60,0x86,0x1D,0xF1,0x52,0x91,0x44,0x7A,0x08,0x8D,0xF7,0xCA,0xAC,0xD9,0x1D,0x7D,
  0x8D,0x42,0x05,0x75,0x66,0x7D,0xC4,0x23,0x72,0x2A,0x6A,0xCC,0xC4,0x6F,0xC8,0x7C,0x61,0x97,0xDF,0xE6,0x86,0xB7,0x6D,0xD3,
  0x81,0xEB,0x91,0x47,0x54,0xEE,0x2B,0x45,0x68,0x12,0xB3,0xF5,0x14,0x68,0xED,0x33,0x46,0x76,0xF3,0xD4,0x54,0x86,0x8C,0x1A,
  0x75,0x72,0x4E,0x41,0xE0,0x76,0xA9,0x18,0x56,0xDD,0xE7,0x23,0x83,0x9D,0xD9,0xCE,0x76,0xFC,0xD5,0x56,0x17,0xD2,0x2D,0xD3,
  0x46,0xAC,0x24,0x5C,0x64,0xB5,0x73,0x7E,0xB4,0x9E,0xCD,0xCB,0xE0,0xFD,0x14,0x41,0x00,0x0D,0xC9,0x02,0xA9,0x8A,0xED,0x0A,
  0xF6,0x1C,0xE3,0xEF,0x59,0xFD,0x67,0x56,0xBA,0x82,0xD2,0x57,0x8D,0x72,0x46,0x78,0x03,0x93,0x4B,0xFC,0x35,0xA8,0x4F,0x76,
  0xA6,0x49,0x23,0x60,0x0F,0x23,0x3E,0xD4,0xDA,0xF0,0x5D,0xFA,0x6C,0x70,0x24,0x42,0x02,0xD4,0x6D,0x20,0x2A,0xC4,0x00,0x7E,
  0x4D,0x57,0x6F,0x70,0xAD,0x94,0x5C,0xA9,0x1D,0x73,0xEF,0x57,0xC3,0x31,0xF3,0x04,0x7B,0xBA,0xF6,0xA5,0xE2,0xAE,0xC6,0x4F,
  0x54,0x12,0xD6,0xAD,0x3A,0xED,0x0D,0xD7,0xA8,0x15,0x5C,0x96,0x2E,0x10,0x42,0xAA,0x44,0x79,0xC1,0xF7,0x6A,0x36,0x00,0xCA,
  0xFC,0x51,0xC8,0x0F,0xE2,0x73,0xD0,0x71,0x5D,0x50,0x82,0xEC,0x8B,0x6C,0x45,0x35,0x9B,0xA8,0x3B,0x49,0x8F,0x1D,0x4F,0xC0,
  0xED,0x48,0xE5,0xD4,0xE4,0x9B,0x50,0x8E,0xC2,0xD8,0x90,0x59,0xB2,0xCD,0xEC,0x29,0xFE,0xB3,0x74,0x02,0x14,0x2D,0xB4,0x37,
  0x5F,0xB5,0x22,0xD3,0xD2,0x2F,0x36,0x7B,0x93,0xE8,0xCF,0xA5,0x09,0xFE,0xB5,0x47,0x13,0x26,0x3F,0x86,0xE2,0x28,0x70,0x91,
  0x91,0xC0,0xC3,0x35,0x7B,0x1D,0xCA,0xB3,0x67,0xBB,0x1C,0x28,0xAC,0xBB,0xCD,0x23,0xA7,0x93,0x07,0x57,0x66,0x62,0xDE,0xCA,
  0x3B,0xD5,0x2B,0xA9,0x49,0x19,0x59,0x09,0xF4,0x45,0x1B,0x63,0xE5,0x8F,0x1F,0xD2,0x96,0x87,0x46,0x9E,0xEE,0x50,0xC1,0x14,
  0xFF,0x00,0x17,0x02,0x91,0xF8,0x81,0xFC,0xA8,0x24,0xE7,0x86,0xC6,0x7E,0xC2,0x87,0x5D,0x5B,0xEA,0x6F,0x62,0x44,0x38,0xDA,
  0x41,0x23,0xDB,0x9F,0xED,0x42,0xF8,0xAA,0xE8,0x24,0x33,0x21,0x3C,0x80,0x71,0xFA,0x66,0x91,0xC0,0x78,0xBF,0x06,0xDA,0x29,
  0xFA,0x8B,0x2B,0x79,0x5B,0x19,0xD8,0x3F,0xAE,0x28,0x9C,0x85,0x9C,0x2F,0xFB,0xB1,0x8F,0xD6,0x97,0x68,0x12,0x05,0xB1,0x55,
  0x23,0x1B,0x76,0xAE,0x3E,0xC3,0x9F,0xE7,0x46,0x58,0x37,0x9B,0x2B,0x5C,0x49,0xC0,0xCE,0x17,0xF2,0xE3,0xFB,0xD2,0xA8,0xD0,
  0xDC,0x86,0x2C,0x73,0x31,0x00,0x7A,0x81,0xCF,0xDB,0x8A,0xB9,0x59,0x7C,0x89,0x09,0x3F,0x84,0x67,0xF5,0xA5,0x6B,0x74,0xA9,
  0x70,0x4B,0x1E,0x64,0x93,0x1F,0xA9,0xFE,0xD5,0x3B,0xAB,0xD8,0xE3,0x8E,0xEF,0x0D,0xC4,0x68,0xA4,0xE3,0xB0,0xE4,0xD5,0x62,
  0x89,0x30,0x9B,0x99,0x92,0x38,0x83,0xE4,0x61,0x4E,0xD9,0x06,0x7B,0x76,0x34,0x9F,0x51,0xB9,0x54,0x9B,0xC9,0x95,0x88,0x0E,
  0xBB,0x95,0xF3,0xD1,0x87,0x7F,0x8E,0x84,0x1A,0xA6,0xE7,0x54,0x85,0x32,0xD2,0x7F,0xD3,0x95,0x70,0xC3,0xDC,0x11,0xFF,0x00,
  0x22,0xB3,0x07,0x54,0x91,0x9C,0x41,0x3B,0x86,0x96,0x02,0xA5,0x1C,0x8F,0x4B,0x0C,0xF0,0x7E,0xC7,0xFE,0x6A,0xA9,0x08,0x38,
  0xBE,0x94,0xDC,0xDB,0xC6,0xF1,0x31,0x69,0xD7,0x9F,0x4F,0xF1,0x8F,0x71,0xFF,0x00,0xE4,0x0F,0x38,0xFB,0x8A,0xE4,0x7E,0x34,
  0x52,0x65,0x91,0x56,0x24,0x77,0x8D,0x4B,0x30,0x03,0x89,0x62,0x27,0xF1,0x01,0xF0,0x73,0xC7,0x6C,0xD6,0xD7,0xEB,0x0C,0x52,
  0x18,0xE0,0xC8,0xD9,0x34,0x92,0x42,0xA7,0x83,0x8C,0xFA,0x93,0xEE,0xA4,0x67,0xED,0x4B,0xBC,0x67,0x69,0x16,0xA3,0x69,0x6D,
  0xA8,0xDB,0x1F,0x2A,0x52,0x70,0x4A,0xF1,0xB1,0x8F,0xBF,0xC1,0xE9,0x5B,0xA0,0xAE,0xCE,0x7B,0x05,0x93,0x4A,0x50,0x3B,0x31,
  0x04,0x6E,0x8E,0x42,0x37,0x76,0xE7,0x3E,0xE0,0xF4,0x23,0xE3,0x34,0x65,0x9C,0x2F,0x1D,0x9C,0xB6,0xEE,0xB9,0xF2,0xBD,0x68,
  0xA4,0xF4,0x5C,0xF2,0x01,0xF6,0x1C,0x10,0x7E,0x69,0x8D,0xA4,0x2C,0x71,0x1A,0x29,0x8E,0x42,0xBB,0x82,0xFF,0x00,0xB2,0x45,
  0xFE,0xF5,0x34,0x87,0xCB,0x90,0x5C,0xF2,0x20,0x2D,0xB9,0x4E,0x3F,0x01,0x3C,0x32,0x1F,0x8E,0xB4,0xA3,0x0B,0x6E,0xAE,0x96,
  0xE6,0xDD,0x58,0x46,0x67,0x51,0x88,0xE5,0x42,0x71,0x20,0x1D,0x88,0xF7,0x22,0x81,0xBC,0x91,0x9E,0x61,0x1B,0xB7,0x9D,0x1A,
  0xAE,0xD3,0xB8,0x60,0x8F,0xB8,0xAB,0x75,0x05,0x8E,0xDE,0xF7,0x7C,0x64,0xF9,0x0D,0xC3,0xFB,0xA8,0xEC,0x7E,0xD5,0x17,0x66,
  0x92,0x39,0xA5,0xC0,0x79,0x22,0x3E,0xBC,0x0F,0xC4,0x08,0xEB,0x4B,0x2D,0x85,0x20,0x7B,0x7B,0x10,0x93,0x08,0xC7,0x30,0xBA,
  0xE4,0x83,0xFC,0x35,0x5C,0x96,0xA6,0x39,0xF6,0x5B,0xE3,0x18,0x24,0x8C,0xE3,0x3F,0x35,0x74,0x0E,0xFE,0x40,0x75,0xEC,0xDB,
  0x7C,0xBF,0x6C,0xD5,0x72,0x23,0xA5,0xD4,0x12,0x44,0x87,0x39,0xC3,0xE7,0xB7,0xC5,0x22,0x5A,0x19,0xB3,0xCB,0x48,0xDB,0xEA,
  0x77,0xCE,0x18,0x00,0x3D,0x24,0xF7,0xAB,0xE3,0xB7,0x65,0x91,0xE4,0x8B,0xD0,0x51,0x86,0x6A,0xCB,0x8B,0xB5,0x9A,0xDC,0x44,
  0xE0,0x08,0xD7,0x23,0x77,0x70,0x7B,0x66,0x84,0x88,0x9B,0x61,0x1A,0xC9,0x3E,0xE8,0xE4,0xE2,0x8C,0x56,0xAC,0x57,0x2D,0x9D,
  0xFC,0xD8,0x5D,0x7F,0xA6,0x4A,0x2E,0xA6,0x05,0xC3,0x76,0x25,0x6A,0xCB,0x08,0x77,0x30,0xFA,0x4D,0xCD,0x22,0x72,0xC0,0x93,
  0x8F,0xFC,0xD2,0xDD,0x6F,0xC5,0x36,0xF6,0xFA,0x2C,0x72,0xDB,0x91,0x35,0xCB,0xE3,0x85,0x39,0x3F,0x9D,0x2E,0xD3,0x7C,0x66,
  0xF6,0x12,0xDB,0xB5,0xCC,0x5B,0x1A,0x73,0xB4,0x83,0xFE,0x71,0x5C,0xAD,0x39,0xBB,0x4C,0x78,0xD2,0x55,0x46,0xC2,0x0B,0xC5,
  0xFA,0x29,0x5E,0xE8,0x28,0x00,0x90,0xC0,0xF6,0x1F,0x15,0x9F,0xBF,0xF1,0x95,0x86,0x8E,0xD8,0xB5,0x55,0x93,0x66,0x18,0xAA,
  0xF4,0xA6,0xD7,0x77,0xD0,0x5D,0x69,0xF7,0x3F,0x59,0x14,0x91,0x26,0xDD,0xC7,0x81,0x82,0x3F,0xE6,0xB9,0xCE,0x8F,0xA3,0x5A,
  0x6A,0xFA,0x9D,0xCC,0x8A,0xDE,0x62,0x63,0x6A,0x9C,0x7A,0x54,0xFC,0x0F,0x7A,0x30,0xBD,0xA6,0x07,0x55,0xFE,0x9D,0x12,0x3F,
  0x10,0xC7,0x71,0x6C,0xB7,0x89,0x13,0x28,0x2A,0x30,0x08,0xE7,0x27,0x9A,0x2E,0xD3,0x53,0x2F,0x22,0x4B,0xBF,0xD2,0x4E,0x38,
  0x1D,0xEA,0xAF,0x0C,0x78,0x6A,0x78,0xED,0x02,0x4D,0xFF,0x00,0x41,0x7F,0x0E,0x4E,0x78,0xAD,0x1A,0x69,0x30,0xA2,0x85,0x01,
  0x40,0x1D,0xC5,0x1F,0x6D,0x01,0x75,0x41,0xFA,0x74,0xCD,0x29,0x52,0x46,0x78,0xA7,0xB0,0xC6,0x59,0x79,0x1C,0x52,0xAB,0x27,
  0x86,0xDD,0x00,0xC0,0x38,0xA6,0x6B,0x78,0xAE,0xBE,0x92,0x05,0x75,0x45,0xAA,0x25,0x42,0x6D,0x7E,0xCC,0x32,0x96,0x09,0xC8,
  0x15,0x81,0xB9,0x9A,0x61,0x7D,0x23,0xDD,0xB9,0x8E,0xD2,0x1F,0xC0,0x83,0x8D,0xC7,0xDE,0xBA,0x84,0xF2,0xAB,0x29,0xEF,0x59,
  0x1F,0x10,0x68,0xA3,0x51,0x46,0x05,0xCA,0x2F,0x39,0x20,0x53,0x46,0x40,0xA3,0x3B,0x63,0xA9,0x1B,0xA2,0x63,0x81,0x36,0xAB,
  0x9C,0x67,0xFD,0xB1,0xAF,0x53,0xF9,0x9A,0x1A,0xFA,0xE1,0x64,0x30,0xC7,0x11,0xCA,0x05,0x76,0x24,0x74,0x2D,0xDA,0xAE,0xBE,
  0x8E,0x1D,0x13,0x4A,0x6B,0x4B,0x25,0xCC,0xB2,0x2E,0xC2,0xC4,0xF3,0xB7,0xB9,0xA4,0xB6,0xD3,0xC4,0x6E,0x6D,0xB2,0x76,0xC5,
  0x1F,0x6F,0x73,0x8A,0x32,0x8B,0x0C,0x5D,0x0C,0xF4,0x28,0x76,0x6A,0xD3,0x4B,0x21,0xE8,0x3A,0x50,0xBE,0x25,0x95,0xA4,0xBA,
  0x8A,0x36,0x38,0x69,0x5B,0x60,0xA9,0xE8,0x37,0x7F,0x51,0xA8,0x4E,0xC7,0x94,0x8C,0x12,0x48,0xEE,0x73,0x81,0x49,0xAE,0xAE,
  0x1A,0xFB,0xC5,0xF1,0x99,0x0F,0xEE,0x6D,0x14,0x96,0xF6,0xCD,0x22,0x29,0x7B,0x35,0xB6,0x57,0x04,0x44,0xEB,0x1F,0x05,0xBD,
  0x2A,0x3E,0xE7,0x8F,0xE5,0xCD,0x37,0x96,0x55,0x82,0x35,0x44,0xE5,0x62,0x4C,0x9F,0x93,0x59,0xFD,0x1A,0x37,0x94,0x2C,0x8F,
  0x95,0xC9,0x2D,0xF6,0xF6,0xFE,0x54,0xE6,0x59,0x63,0x52,0xC5,0xCF,0x03,0xD4,0xDF,0x61,0xDA,0xB5,0x02,0xC5,0x93,0x3C,0xDB,
  0xED,0x77,0x1C,0xB7,0x9B,0x9E,0xBD,0xFF,0x00,0xCF,0xE9,0x4A,0xEF,0xEF,0xDD,0xA0,0xBF,0x54,0x24,0xB4,0xA8,0xE5,0x7D,0x8A,
  0xC6,0xE4,0x0F,0xD7,0x3F,0xCE,0xBC,0xBF,0xD6,0xA3,0x33,0x2B,0xAF,0x55,0xCA,0xA8,0x1E,0xF9,0xE6,0x83,0x8A,0x55,0x96,0xDC,
  0xA2,0x38,0x25,0x60,0x39,0x6F,0xB9,0x1F,0xDA,0x8A,0x54,0x16,0xEC,0xF2,0xEE,0xF0,0xCF,0xA7,0xDC,0xBA,0x64,0x9B,0x64,0x4C,
  0x8F,0x76,0x42,0x7F,0xA8,0x3F,0xCE,0x95,0x6A,0x97,0x10,0x5C,0x68,0xD7,0x12,0x29,0x3E,0x64,0x08,0xA7,0x8F,0xE2,0x5C,0x0C,
  0xFE,0x99,0x14,0x14,0xFA,0x82,0xAD,0xAF,0xD3,0x2B,0x8F,0x36,0xE0,0x3C,0xE4,0x8E,0x98,0xC8,0x50,0x3F,0x3E,0x6B,0x35,0x7F,
  0xA9,0x85,0xBA,0x96,0xDC,0x92,0x2D,0xE4,0x8F,0x18,0xF6,0xDC,0x87,0xFA,0x60,0x53,0xA1,0x47,0x92,0x6A,0x12,0x5C,0x33,0xA4,
  0x6D,0x9B,0xD8,0xC2,0x4C,0x9F,0x2C,0x3A,0xFE,0xB8,0xFE,0x5F,0x34,0x75,0xBD,0xEA,0x5D,0x41,0x77,0x66,0x38,0x82,0x70,0x5E,
  0x1C,0xF5,0x47,0xEA,0x07,0xDB,0xA8,0x15,0xCE,0x45,0xFC,0x96,0xB7,0xB6,0xF3,0xCA,0x7D,0x40,0x28,0x62,0x3B,0xA9,0x03,0x9F,
  0xE4,0x0D,0x68,0x2C,0xAE,0x7C,0xB9,0x91,0xC3,0x6D,0xFD,0xE6,0x14,0x9E,0x99,0xEB,0x8F,0xB1,0xEA,0x3E,0xF4,0x41,0x43,0xB9,
  0x64,0x56,0x6B,0x4B,0xD1,0x81,0x2E,0x3C,0xB9,0x46,0x78,0x6C,0x7B,0x8F,0x7A,0xAA,0x79,0x48,0x32,0xB4,0x71,0x9D,0x8D,0xC3,
  0xC6,0x4F,0x0D,0xEC,0x7E,0xE3,0xA7,0xF3,0xA1,0xED,0x9F,0xEA,0x22,0xFD,0xEE,0xD2,0x19,0x8E,0xE1,0xD3,0xBF,0xFE,0xAA,0xE9,
  0x64,0x49,0x91,0xAC,0xDC,0x90,0xC0,0x82,0x4F,0x7D,0xBD,0xA9,0x18,0xC8,0x51,0x7C,0x23,0xC8,0x62,0x07,0xE1,0x2A,0xA0,0xF7,
  0xF8,0xFB,0xD0,0x96,0xB1,0x9B,0x79,0x63,0x70,0xA4,0xC1,0x20,0xC7,0x3F,0xD2,0x99,0x7D,0x27,0x94,0x92,0x45,0x79,0xB8,0x86,
  0x21,0x95,0xF1,0xD4,0x7F,0x7A,0xB5,0xA0,0x8E,0x4B,0x67,0x82,0x1B,0x85,0x90,0xA9,0x1B,0x76,0xFB,0x7C,0xD0,0x51,0xB3,0x36,
  0x2C,0x09,0xE4,0x5E,0xBA,0x18,0x8E,0x17,0xF0,0x8C,0x7F,0x2A,0xA8,0xDD,0x66,0x56,0x6D,0x87,0xCB,0x97,0x20,0x1E,0x99,0xFB,
  0xFC,0x8A,0x2F,0xEA,0x5C,0x6A,0x33,0x24,0xA3,0x00,0xE0,0x86,0xED,0xBA,0xAC,0xB8,0xB4,0x96,0x45,0x77,0x85,0x92,0x68,0x5B,
  0x04,0xA3,0x60,0x14,0x6F,0x71,0xF1,0x46,0x30,0x03,0x90,0xBD,0x21,0x56,0x40,0xEC,0x82,0x45,0xFC,0x2E,0x07,0x04,0x8A,0x1E,
  0x4B,0x55,0xB8,0x91,0x2D,0xCB,0x6D,0x97,0xAC,0x4C,0x78,0xCD,0x5F,0x09,0x51,0xBD,0x4C,0x6C,0x43,0x64,0x10,0x0F,0xE1,0x35,
  0x19,0x2C,0x4C,0x76,0xA6,0x4B,0x76,0x12,0x42,0xBF,0x87,0x27,0xD5,0x1B,0x77,0x06,0x92,0x54,0x8C,0xAC,0xE8,0x5E,0x14,0xD2,
  0x2F,0x35,0x4F,0x0E,0x5C,0x5C,0x5A,0x34,0x71,0x14,0x90,0xA9,0x8C,0x8E,0x57,0x15,0x1B,0x08,0x44,0xDA,0x9D,0xBB,0x5F,0xDB,
  0x93,0xE4,0x12,0x55,0x8F,0x2A,0x48,0x1F,0xCC,0xD0,0x6D,0x0E,0xBB,0xE1,0x8B,0x99,0x2D,0x22,0xB4,0x9E,0xDF,0xEA,0x53,0xD5,
  0x19,0xF5,0x06,0x3E,0xF9,0x15,0x4E,0x8F,0xAB,0x4F,0xE6,0x3D,0xA6,0xAB,0x76,0x22,0x95,0x98,0x18,0xD6,0x45,0xC6,0x0E,0x6B,
  0x9E,0x50,0x5C,0xB5,0xD1,0x45,0x26,0xFB,0x36,0xF6,0x7A,0xAC,0x5A,0xCC,0xB7,0x16,0x40,0xAC,0x71,0x81,0x8C,0x01,0xCF,0xEB,
  0xF6,0xAD,0x07,0xEC,0xFB,0xC2,0xB1,0x20,0x2D,0x11,0x1F,0x88,0xEE,0xF4,0xF4,0xE7,0xF9,0xD2,0x8D,0x08,0xD8,0x5F,0xB4,0x97,
  0x37,0x90,0xAC,0x17,0xB0,0x1D,0xA4,0x83,0xC3,0xFC,0x8A,0xE9,0x1E,0x0D,0x91,0x04,0x12,0xEC,0x20,0x8F,0xE9,0x4D,0x8A,0x4A,
  0x4B,0x66,0xC9,0x0E,0x3D,0x0E,0xE5,0xB3,0x41,0x12,0xC7,0x18,0x01,0x17,0xAD,0x20,0xD6,0xB1,0x6F,0x19,0xDC,0x70,0xBE,0xE3,
  0xB5,0x39,0xBF,0xBE,0x30,0x15,0x5D,0xB9,0x0C,0x71,0x58,0xEF,0x18,0x6A,0x0F,0x6D,0x13,0x28,0x50,0xE1,0xB8,0x00,0xD4,0x33,
  0xCE,0x31,0x56,0x1C,0x49,0xB6,0x65,0x6F,0xBC,0x46,0xF1,0x5F,0xA5,0x9D,0xA9,0x77,0x95,0xBA,0x93,0xD3,0x1E,0xE6,0x9B,0xD9,
  0x6B,0xF2,0x5A,0x5B,0x19,0x2F,0x1B,0x27,0xFD,0xA0,0xD6,0x17,0x50,0xD4,0x05,0xAE,0xB5,0x6D,0x22,0x80,0x03,0x7A,0x5F,0x8F,
  0x7A,0x65,0xAF,0xC6,0xD7,0x96,0x6B,0x2C,0x65,0x78,0x18,0x25,0x78,0xA9,0x63,0x9B,0xE1,0x69,0x96,0x94,0x17,0x2A,0x66,0xBE,
  0xD7,0xC7,0x9A,0x64,0xCC,0xD1,0x89,0x97,0xCC,0x1D,0x57,0x3C,0xD4,0xE6,0xF1,0x4C,0x33,0x02,0xB1,0x95,0x20,0xFC,0xD7,0x02,
  0xBE,0xB6,0xF2,0xE7,0x96,0x58,0x58,0x89,0x62,0xE4,0x90,0x7A,0xD4,0xAD,0xF5,0x1B,0x88,0xEE,0x72,0xD2,0x10,0x8E,0x01,0xEB,
  0xC8,0x35,0xD1,0x1C,0x92,0xFA,0x27,0xEB,0x89,0xD5,0x7C,0x41,0xA9,0x42,0x88,0x44,0x10,0x2C,0x97,0x32,0x0C,0x67,0x3D,0x2B,
  0x11,0x77,0x0D,0xCC,0x32,0xF9,0x93,0x92,0x26,0x7E,0x88,0x0E,0x76,0x8F,0x7A,0x8C,0x5A,0x85,0xBA,0xC9,0x1C,0x92,0xCA,0xCE,
  0xE0,0xE7,0x14,0x4D,0xDD,0xDC,0x17,0x96,0xF2,0x32,0x24,0x99,0x1D,0x48,0xEF,0xF1,0x5D,0xB8,0xE5,0xC9,0x1C,0xF3,0x8F,0x10,
  0xAD,0x13,0x50,0x58,0x55,0xA2,0xB7,0xE1,0x42,0xFA,0xDB,0xDE,0x8F,0xD1,0xED,0xD2,0x7B,0x96,0x66,0xFC,0x73,0xB6,0xF6,0x03,
  0xAE,0x3B,0x56,0x01,0x6E,0xE4,0x8E,0xED,0x6D,0x2D,0xD5,0xB0,0x4F,0xA8,0x7F,0x7A,0xE9,0xBE,0x11,0xB4,0x7F,0x25,0xDC,0x21,
  0x12,0xC8,0x00,0x07,0xFD,0xAB,0x4F,0xC4,0x9B,0x74,0x3B,0x0C,0x14,0x24,0x69,0xC2,0x03,0xC9,0x14,0xA7,0x52,0xBE,0xDB,0xF5,
  0x12,0x96,0xF4,0x46,0xA4,0xFD,0xFD,0x85,0x69,0x27,0x81,0x21,0xB4,0x61,0x10,0x62,0x40,0xDB,0xB8,0x0C,0x9F,0xCA,0xB0,0xBA,
  0xFC,0x13,0x45,0x0A,0xC7,0x24,0x6E,0x88,0xE7,0x02,0x35,0xE5,0xDB,0xEF,0x41,0x21,0x93,0x33,0xE4,0x1B,0xA1,0x0E,0xE6,0x2A,
  0x10,0x92,0x48,0xEE,0x5B,0x9C,0x57,0xBA,0x8E,0xB9,0x1E,0x9E,0xCD,0x22,0x11,0xE5,0xBC,0x85,0x70,0x3B,0x80,0x30,0x3F,0x9F,
  0xF4,0xA1,0x24,0xB7,0x9E,0x29,0xD2,0xDD,0x32,0x32,0x99,0x27,0x3C,0x27,0xB9,0x3F,0x38,0xAC,0xAE,0xBD,0x20,0xBD,0xD6,0x23,
  0xB6,0xB3,0x04,0xC7,0x19,0x24,0xFB,0x0E,0x30,0x2B,0x55,0x0D,0xD8,0x49,0x6B,0x8C,0x4B,0x2C,0x67,0xD5,0x11,0x0B,0x1E,0x7F,
  0x89,0x01,0xCF,0x1F,0x9D,0x2E,0xD6,0x6E,0x95,0xA6,0x5B,0x94,0x01,0xA3,0x94,0x2B,0x1F,0xE6,0x0D,0x59,0x0D,0xE8,0x8E,0xEE,
  0xDC,0xC8,0xDB,0xA2,0x88,0x88,0x1C,0x03,0xD8,0xF3,0x9F,0xCA,0x95,0x25,0xB5,0xE2,0x35,0xC2,0x48,0xAC,0xDB,0x59,0xE2,0x51,
  0xEE,0x33,0xFE,0x1A,0xC6,0x3D,0x8A,0x3F,0xA9,0xB8,0x45,0x27,0x85,0x4F,0x4E,0xEE,0x9C,0x1E,0x9F,0xD2,0xB4,0x76,0x70,0xC8,
  0x61,0x9A,0x0C,0x1C,0xA8,0x04,0x8F,0x6F,0xFD,0x50,0x3A,0x2E,0x92,0xE1,0x6E,0x9D,0xD8,0x0F,0x20,0x00,0x73,0xDB,0x34,0xF2,
  0xD1,0xA7,0xB4,0x33,0x23,0x24,0x4F,0x33,0x02,0x11,0x99,0xBF,0x10,0x1C,0xE0,0x9F,0xCF,0xAD,0x6A,0x35,0x9E,0x69,0xFE,0x72,
  0x2B,0x4C,0x22,0xF3,0xAD,0x78,0x2E,0x01,0xE9,0xCE,0x09,0x15,0xF4,0x17,0x31,0xFD,0x60,0x79,0x8E,0xDC,0x21,0x40,0xE4,0x1C,
  0xE3,0xB5,0x10,0x17,0x3A,0x58,0x92,0xD5,0xA6,0x56,0xC8,0x25,0x47,0x23,0x27,0xA8,0xA4,0xF3,0xDE,0xB5,0xBC,0x89,0x05,0xCA,
  0x6C,0x59,0x49,0x08,0xCE,0x30,0xB9,0xF6,0x3F,0xDE,0x87,0x40,0x18,0x6A,0xF7,0x12,0xDD,0x5B,0xA0,0x67,0x21,0x3F,0x0A,0xC8,
  0x54,0x9D,0xAC,0x0F,0x19,0xF8,0xA1,0x63,0x8D,0x6D,0x24,0x89,0xD1,0x26,0x49,0x4F,0x3B,0xB3,0x94,0xDD,0xDD,0x68,0xA3,0x32,
  0x0B,0x60,0x90,0x44,0xE0,0x32,0x64,0x0D,0xF9,0xD8,0x7B,0xAF,0xDA,0xA5,0x1D,0xC0,0xB8,0x48,0xDD,0x55,0xD5,0x91,0x70,0xC8,
  0xBC,0x86,0xFF,0x00,0xC8,0xAC,0x8C,0x09,0x02,0x99,0xAE,0x8A,0xA1,0x46,0x8A,0x4C,0xA0,0xCF,0x51,0xF1,0xF9,0x57,0xC9,0x15,
  0xEE,0x98,0xD3,0xA4,0xE8,0x19,0x07,0x00,0xB0,0xED,0x47,0xDB,0x1F,0xA4,0x83,0xCB,0xD4,0x62,0x0C,0x0B,0x12,0xAC,0xC9,0x8E,
  0xBF,0x23,0x9A,0xA2,0xED,0xDE,0xE1,0x0A,0x19,0x15,0x8B,0x1C,0x29,0x47,0xE4,0x8F,0x63,0xEC,0x69,0xBA,0x00,0xAE,0xCA,0xE2,
  0x51,0x78,0xD7,0x6D,0x18,0x92,0x17,0xF4,0xB4,0x69,0xD4,0x7C,0xE2,0x8C,0xB0,0x79,0xA2,0xBA,0x76,0x68,0xD7,0xCB,0x94,0xE0,
  0x67,0xA3,0x7D,0xE8,0x84,0xB6,0xB6,0xB6,0xB6,0xF3,0xA3,0x94,0x25,0xD4,0x7C,0xED,0x39,0x56,0xEB,0xD0,0xFB,0xD5,0x57,0x0C,
  0xB7,0x36,0xE0,0x6E,0x47,0x0A,0x78,0x18,0xE5,0x49,0xEB,0x8A,0x94,0xC6,0x89,0xD7,0x2D,0x6F,0xAE,0x35,0x0B,0xD0,0x6F,0xDF,
  0xEA,0x0C,0xA3,0x68,0x0E,0x30,0x62,0x3E,0xFF,0x00,0x6A,0x37,0x55,0xF0,0x5E,0x91,0x3D,0xA4,0x97,0xA1,0x8C,0xD7,0xCA,0x30,
  0x55,0x9B,0x20,0x1A,0xFB,0x5D,0x6B,0x7B,0xBD,0x26,0xDE,0x0D,0x21,0xE0,0x49,0x49,0x08,0xB2,0xB3,0x6D,0xC1,0x3D,0xBE,0x0D,
  0x79,0xA3,0xF8,0x7D,0xF4,0xB9,0x4D,0xE6,0xA5,0x7D,0x70,0xF3,0xB4,0x78,0xF2,0xDB,0xFE,0x9A,0xF3,0xD4,0xFB,0xD7,0x1A,0x8F,
  0x14,0xD2,0x2C,0x9D,0xB4,0xC2,0x22,0xF0,0xDD,0xAE,0x87,0xA3,0xDB,0x1D,0x43,0x0F,0x2B,0xFA,0xB7,0xC7,0xD1,0x7B,0xD3,0x9F,
  0x0C,0xEA,0xB6,0xB6,0xC9,0x28,0xF3,0xE3,0x08,0x70,0x01,0xDD,0xD6,0x89,0xB1,0x9A,0x3B,0xCD,0x1E,0xE2,0x4D,0x70,0x41,0x6D,
  0x1C,0x52,0x6C,0x01,0x8E,0x43,0xAF,0x63,0xCF,0xBD,0x2C,0xD5,0xB4,0x7B,0x1D,0x5A,0xCD,0xB4,0xB8,0xE2,0x8E,0x18,0x9B,0x95,
  0x92,0x15,0xEB,0x8E,0x41,0x07,0xB5,0x37,0x4F,0x42,0xDD,0xAB,0x63,0xDB,0xCD,0x46,0xDE,0x65,0x21,0xA5,0x52,0xAC,0x38,0xE7,
  0xA5,0x65,0xB5,0xBD,0x76,0xD7,0xCA,0xF2,0xEE,0x9E,0x30,0xE9,0xC7,0x27,0xA8,0xA0,0x6C,0xBC,0x2B,0x3E,0x9F,0xE1,0x9B,0x89,
  0xEE,0x12,0xFE,0x4B,0x84,0x56,0x78,0xE2,0x95,0xB2,0xC5,0x47,0x71,0x8A,0x87,0x87,0x34,0x2D,0x17,0x5A,0x8A,0x17,0xD5,0xB4,
  0xEB,0xAF,0x32,0x30,0x5F,0x74,0xE8,0xE9,0x19,0xC0,0xEE,0x4D,0x4A,0x58,0xB9,0xBA,0x7E,0x8F,0x17,0x4A,0xD1,0x94,0xD4,0x6E,
  0x6C,0x2E,0x9D,0xE5,0x59,0x10,0xB6,0xDC,0x81,0x9E,0x87,0xB5,0x4F,0xC3,0x1A,0xBD,0xAA,0xD8,0xDC,0x47,0x75,0x32,0xEF,0x0F,
  0xB4,0xA9,0x3D,0x3E,0x6B,0x45,0xAE,0xFE,0xCA,0x92,0xFE,0x68,0x6E,0x59,0x52,0xC0,0x95,0xE5,0x2C,0xE4,0x2C,0x42,0xF6,0xCF,
  0x18,0x35,0xCA,0xB5,0xEF,0x05,0xCF,0x63,0x74,0x44,0x32,0xDE,0xE4,0x12,0x18,0x91,0xC9,0x00,0xF5,0xF6,0xAD,0x8F,0xF1,0x94,
  0x74,0x3C,0xB2,0xF2,0xD8,0x67,0x8B,0xE1,0x58,0x2E,0x44,0xB6,0xB2,0x00,0x24,0xFC,0x40,0x1E,0x2B,0x27,0x71,0x72,0x1E,0x40,
  0x43,0x64,0xAF,0x41,0x5A,0xC8,0x3F,0x67,0x9A,0x96,0xA3,0x64,0x26,0x4D,0x4D,0x4B,0x91,0xF8,0x1C,0x67,0x1E,0xD4,0xB7,0x51,
  0xFD,0x9C,0xEB,0x7A,0x79,0x57,0x8F,0x37,0x0A,0x57,0x96,0x1E,0xF5,0x68,0xC1,0x75,0x62,0xF2,0x7F,0x0A,0xEC,0xAE,0x95,0xF6,
  0xAF,0x00,0xF7,0xA6,0x16,0x17,0x4E,0x6E,0x1E,0x15,0x02,0x34,0x4E,0xAC,0xC7,0xAF,0xD8,0x50,0xDA,0x4F,0x83,0xF5,0x37,0x67,
  0x32,0xC8,0x2D,0x99,0x53,0x3F,0xBC,0x5E,0xA7,0xD8,0x56,0x9B,0xC2,0x9E,0x15,0x71,0x1C,0x93,0xC8,0xA9,0x34,0xC9,0xD5,0x9F,
  0xAF,0xE4,0x2A,0xF8,0x92,0x8F,0x64,0xF2,0x36,0xC6,0xBE,0x1A,0xF0,0xDD,0xB4,0xB3,0x7D,0x4B,0x34,0x8C,0x49,0x0C,0x7D,0x3D,
  0x6B,0xA1,0x84,0x6B,0x6B,0x7D,0xB6,0xF6,0xE0,0x02,0x32,0x4C,0x82,0x8B,0xF0,0x6E,0x8F,0xF4,0xD6,0x6B,0x2D,0xD1,0xDD,0x23,
  0x0F,0x4A,0x9F,0xE1,0xA2,0xF5,0x5B,0x98,0xE3,0x56,0x59,0x0A,0xE4,0x9D,0xAA,0x3D,0xCD,0x73,0xFE,0x4F,0xE4,0xCA,0x9A,0x83,
  0xA3,0x62,0xC6,0xAF,0xFA,0x32,0x1A,0xAE,0xAF,0x71,0x68,0xB2,0x8F,0x3F,0x69,0x55,0xC9,0xC0,0x03,0x15,0x99,0x93,0x56,0xBD,
  0xBB,0x85,0xDD,0x25,0x62,0xB9,0x3B,0x59,0xFD,0x39,0xC5,0x69,0xBC,0x4F,0xA2,0x43,0x73,0x08,0x2E,0xAC,0x65,0x60,0x49,0x39,
  0xE0,0x1A,0xCA,0xEA,0x96,0xB2,0x43,0x60,0x61,0xC1,0x00,0x10,0x01,0x03,0x18,0x1D,0xEB,0x93,0x1E,0x7C,0x89,0x6D,0x9D,0x1F,
  0xAE,0x2F,0xA4,0x2C,0xB9,0x8A,0x6B,0xA9,0x1E,0x4B,0xB8,0xDB,0x6E,0xCC,0x61,0x5B,0x69,0x6C,0xFF,0x00,0xC7,0xDE,0xB3,0xDA,
  0xAC,0x56,0x56,0xB1,0x66,0x17,0x59,0x2E,0x25,0x3B,0x17,0x60,0xFD,0xDA,0x7D,0xB1,0xD4,0xFC,0x9A,0xBE,0xE7,0x52,0xBC,0xB4,
  0xB5,0x85,0x67,0xC6,0xD9,0xA5,0x2A,0xAE,0x48,0x39,0x04,0x9E,0x9F,0x95,0x7B,0x68,0xD1,0xDC,0xCF,0x1E,0x17,0x20,0x11,0xDB,
  0x8C,0x0E,0x9C,0x57,0x6C,0x73,0x49,0x2D,0x93,0xFD,0x6B,0xC1,0x15,0x86,0x9D,0x1A,0x47,0x2D,0xAB,0xB0,0x7B,0xAF,0x54,0xEE,
  0x3F,0xDA,0x31,0xD3,0xFE,0x71,0x4E,0xA5,0x8A,0x21,0xF4,0xE6,0xD8,0x09,0x64,0xF4,0xC7,0x93,0xEE,0x41,0xF5,0x7E,0x82,0x8F,
  0x87,0x4D,0xB7,0xB1,0xBB,0x6B,0xA1,0x1C,0x93,0xB9,0x0C,0x58,0x8C,0x60,0x67,0xFA,0xD0,0xBA,0x9C,0x7F,0x47,0x69,0x3F,0xD0,
  0xBE,0x64,0x57,0xDD,0xBD,0xB8,0x20,0x91,0xDB,0xF5,0xAB,0xC7,0x2A,0x91,0x29,0x63,0x68,0x4C,0xA2,0x64,0xD4,0x4C,0x53,0x02,
  0x9C,0x11,0x23,0x0E,0xFC,0x9C,0x71,0x4C,0x64,0xB7,0x76,0xB6,0x6B,0x66,0xDB,0x97,0x2A,0x23,0x6F,0x73,0x83,0x9C,0x7B,0x73,
  0x40,0x3E,0xF6,0x85,0xEE,0xE4,0x2C,0x1E,0xE0,0x6F,0x38,0x1C,0xF1,0x80,0x71,0xF7,0x3F,0xD6,0x98,0x58,0xC6,0xD7,0x02,0x7F,
  0x33,0xD3,0x1A,0xBF,0xA3,0x07,0x84,0xC0,0xDD,0x8F,0xD7,0x35,0x44,0xC4,0xA2,0x31,0xEE,0x8E,0xDA,0xE1,0x11,0x73,0x0E,0x53,
  0x80,0x70,0x52,0x41,0xED,0xF9,0x8A,0x1A,0xF5,0x65,0xD4,0xE6,0x86,0x68,0xF2,0x65,0x55,0x20,0xC4,0x54,0x10,0x1A,0x98,0x45,
  0x2C,0x73,0x49,0xBE,0x55,0x06,0x09,0x23,0xDE,0x70,0x7B,0xE7,0x82,0x3D,0xC8,0xA1,0x83,0xE6,0xFA,0xEE,0x76,0x93,0x74,0x50,
  0xA6,0x41,0x5F,0xE2,0x00,0xFA,0x4F,0xC1,0xF9,0xA1,0x60,0xA3,0xDD,0x16,0x6B,0x56,0xB5,0x6B,0x87,0x31,0xC7,0x24,0x79,0x47,
  0x2D,0x90,0xAC,0x3E,0x6A,0x52,0x2A,0x47,0x38,0x7B,0x66,0x8B,0x64,0x9C,0xFA,0x24,0xC6,0xD3,0x42,0xBC,0x31,0x19,0x62,0xBA,
  0xB2,0xE6,0xD6,0xE1,0xCA,0xDC,0x41,0x27,0x4D,0xD8,0xE7,0xFB,0xD0,0x97,0xF0,0xB0,0xFA,0x81,0x18,0x6D,0x90,0xB6,0xDF,0x57,
  0x46,0x5E,0xA3,0xFF,0x00,0x74,0x7C,0x32,0x43,0xCB,0x35,0x8B,0x53,0x90,0xC6,0xD7,0x50,0xAC,0xA3,0xFE,0x9A,0xB3,0xF0,0xDF,
  0xD8,0xD5,0x2B,0x60,0x7E,0xAD,0xD2,0xE9,0x7F,0xF9,0x0C,0xB8,0x52,0x5B,0x0C,0x3E,0xDE,0xE2,0x95,0xC1,0x68,0x2D,0x02,0x86,
  0x23,0xCB,0x99,0x78,0x66,0xE0,0xA1,0x3D,0x39,0xA3,0xF4,0xC9,0xD5,0xE4,0x68,0x75,0x17,0x32,0x3C,0x79,0x00,0x9E,0x1B,0x1E,
  0xC0,0xD2,0xEC,0x60,0xA3,0x6A,0xA8,0xB2,0x45,0x39,0x77,0x95,0x86,0x55,0x64,0x4E,0xA3,0xE1,0xAA,0x88,0x2C,0x22,0x91,0x83,
  0x59,0xB3,0x89,0x06,0x32,0x1B,0x8E,0x45,0x4D,0xB5,0x1B,0x78,0xE3,0x4D,0xB7,0x05,0xC0,0xC8,0x07,0x6F,0xF5,0xA0,0x2E,0xDE,
  0x50,0x04,0xA8,0x14,0x8E,0xBB,0xD0,0x70,0x4F,0xB3,0x0E,0xC7,0xE6,0x92,0x49,0xAE,0x8C,0x8E,0xE3,0x65,0x14,0x1A,0xAB,0xA9,
  0x97,0x4C,0x99,0x2E,0x62,0x22,0x59,0x2D,0xD6,0x2D,0xA2,0x46,0xE7,0x8E,0x7F,0x5A,0x62,0x92,0xBD,0xC3,0xC6,0x87,0x4B,0xBD,
  0x85,0x65,0x6F,0x2C,0xC5,0x21,0x51,0xD3,0xB7,0x5A,0x1F,0xC3,0x57,0xB3,0xD9,0xCC,0x96,0x4D,0x05,0xD7,0x9F,0x14,0x4C,0x43,
  0xCB,0x34,0x61,0xE6,0xDB,0xC7,0xAC,0x0C,0x9C,0x67,0xB9,0x20,0x63,0xEE,0x2B,0xDD,0x47,0x4F,0xBB,0xB8,0x89,0xF4,0xDB,0x6D,
  0x56,0x35,0xF3,0x95,0xA6,0x9E,0x5F,0x24,0xC8,0x50,0xFB,0xAB,0x96,0xC2,0xF1,0xD0,0x7D,0xEA,0x2A,0x49,0xDE,0x87,0xAA,0xED,
  0x86,0xDC,0x41,0x1C,0x26,0x59,0x2F,0xB4,0x2B,0xB9,0xF0,0xC1,0x23,0x8A,0x39,0x03,0x29,0x3D,0x37,0x75,0xC7,0xF6,0xA2,0xB4,
  0xD9,0xAC,0xD9,0x84,0x51,0xDD,0xDB,0xCD,0x73,0x6F,0x2E,0xDF,0x2A,0x19,0x15,0xFC,0xB3,0xD7,0x6B,0x11,0xDF,0x1D,0x6B,0x3B,
  0xA4,0x45,0x23,0xE9,0x17,0xDE,0x65,0xFD,0xFE,0xA1,0x3B,0x86,0x89,0xCB,0x40,0xC6,0x32,0x0B,0x60,0x00,0xAA,0x70,0x47,0x3C,
  0x15,0xC9,0xE7,0x1D,0x69,0x92,0x3C,0x7A,0x1E,0x95,0x0E,0xA4,0xDA,0x4D,0xCD,0xBC,0x36,0xF0,0xEC,0x75,0x31,0x24,0x1B,0x7B,
  0x96,0x39,0x25,0xB2,0x4E,0x4E,0x4F,0x27,0x8A,0x15,0x5D,0x1B,0xBD,0x0F,0x7C,0x41,0x3E,0xA7,0x73,0x68,0xD7,0x1A,0x7D,0xF5,
  0xBD,0xAD,0xBC,0xB0,0xEE,0x12,0xBC,0x5B,0x99,0x39,0xF7,0x39,0x18,0xFC,0xA9,0x6E,0x94,0xB7,0xA2,0xCE,0x0B,0x4D,0x6A,0xE9,
  0x2E,0x46,0xC2,0xC1,0xC2,0x92,0x66,0x4E,0xBB,0xD8,0x63,0x03,0x19,0xAB,0x2D,0xF5,0xCD,0x3E,0x58,0xE0,0x68,0xC3,0xDC,0xC1,
  0xA8,0x16,0x7C,0x1C,0x94,0x45,0xC7,0x43,0xCF,0x4F,0xE1,0x3D,0x79,0xC5,0x67,0xAC,0x75,0xBF,0x11,0xA5,0xEE,0xAB,0x15,0xDE,
  0x97,0x0D,0xBE,0x96,0x1D,0x12,0xDA,0x5F,0x34,0x14,0x40,0x32,0x06,0x59,0x7B,0x70,0x38,0xEA,0x33,0x59,0xBF,0x86,0x4B,0xC3,
  0x43,0x7B,0xAA,0x5B,0x5A,0xCF,0x1E,0xF9,0xD6,0x2B,0x09,0x14,0x65,0xCB,0x8E,0x3A,0xE0,0x7B,0xF3,0xF6,0xF6,0xE6,0xBC,0xB4,
  0xBA,0xB6,0xB0,0xB6,0xB8,0xFF,0x00,0xE3,0xEF,0x31,0x28,0x90,0x45,0x31,0x2E,0x17,0x77,0xB1,0xC5,0x65,0x7C,0x6C,0xB7,0x57,
  0x31,0xC2,0xF6,0x30,0xCC,0x6E,0x23,0xB8,0x46,0x32,0x28,0x0C,0x13,0x07,0x20,0x90,0xDC,0x11,0xDC,0xF5,0xA0,0x35,0x4D,0x42,
  0x79,0xB4,0x2B,0x89,0xAE,0x26,0x96,0xCB,0x50,0x52,0x76,0x95,0xC4,0xA2,0x50,0x07,0x0C,0x14,0x31,0x04,0x9E,0x4F,0x18,0xC7,
  0x1D,0x69,0x79,0x7D,0x63,0x55,0x0D,0x35,0xCD,0x5A,0x47,0xD5,0xEC,0xC6,0x98,0xDA,0x5B,0xA4,0xA7,0x74,0xCA,0xD2,0x79,0x6C,
  0x13,0xA7,0xA4,0x01,0xEA,0x39,0xEF,0x56,0x58,0xEA,0xFA,0x75,0xEC,0xB2,0x59,0x5B,0x2B,0xB5,0xFC,0x00,0xAB,0x22,0x82,0x50,
  0x36,0x7F,0xDC,0x78,0xAE,0x77,0xE0,0xC7,0x9E,0xF3,0x4A,0x9D,0xE7,0xB4,0xFF,0x00,0x51,0xBA,0x84,0xF9,0x71,0xC9,0x72,0x0A,
  0xC9,0xB4,0x36,0x5B,0x6F,0x70,0x40,0x27,0xF3,0x34,0xFF,0x00,0x55,0x3A,0x8D,0xF4,0x22,0x4F,0x0F,0x69,0xD6,0xF6,0x56,0x53,
  0x42,0x27,0x99,0x60,0x67,0x91,0xA0,0xD9,0x91,0x99,0x02,0x29,0xDA,0x78,0xE0,0x67,0x3F,0x14,0xE9,0xFA,0xCD,0xED,0x23,0x41,
  0xF5,0x16,0xC2,0x6B,0x87,0xBD,0xBA,0x02,0x18,0xE6,0x58,0x52,0x1F,0x2B,0xD4,0xCC,0x7A,0x9C,0xFF,0x00,0x6A,0xF2,0xC4,0xDA,
  0x5C,0x5D,0x39,0xD3,0xE7,0x12,0x28,0x7C,0x34,0x41,0x78,0x5F,0xBD,0x20,0xD5,0xBC,0x41,0x1E,0x91,0xA5,0x69,0xF0,0x6B,0x73,
  0xDB,0x3C,0xF6,0xD1,0x23,0xCA,0xAC,0x8C,0x65,0xB9,0x46,0x04,0xAC,0x8A,0xC3,0x82,0xA3,0xA7,0xB9,0xA8,0x2E,0xA1,0x1E,0xA5,
  0xA7,0x5B,0x6A,0xBA,0x66,0xA7,0x6A,0x96,0xCC,0x42,0xAC,0x46,0x3F,0x2D,0x9C,0xE7,0xD5,0xBC,0x03,0xD0,0x51,0x4D,0xAE,0xFA,
  0x33,0x57,0xA3,0xB1,0xDC,0x4D,0x1D,0xB5,0x8E,0xE0,0x57,0x21,0x32,0x0D,0x73,0x3F,0x0C,0xCB,0x79,0xAD,0x78,0x9A,0xEE,0xFE,
  0xF6,0x46,0x68,0x2D,0x98,0xAC,0x4B,0x91,0xB4,0x7C,0xFD,0xE8,0xBD,0x53,0xC4,0x2B,0x17,0x87,0xB3,0x25,0xC1,0x67,0xD9,0xE9,
  0x2A,0xBD,0xBE,0x05,0x66,0xBF,0x67,0x5E,0x21,0xB5,0x9A,0xDE,0xF6,0xD9,0xC4,0x91,0xED,0x7D,0xDB,0xE4,0x04,0x6E,0xCD,0x72,
  0x64,0xC4,0xDC,0x9B,0x5D,0x14,0xC7,0x2A,0x5B,0xEC,0xD8,0xF8,0x8E,0xFA,0x48,0xA2,0x6F,0x25,0x52,0x42,0x14,0xF2,0xDC,0x60,
  0x76,0xAC,0x85,0xFB,0xAD,0xC6,0x95,0x23,0xCA,0xCE,0x77,0x44,0x19,0xF0,0xB8,0xC7,0xF8,0x68,0xFD,0x66,0xE2,0x19,0x64,0x66,
  0x79,0x0B,0x24,0x63,0x70,0x40,0x7A,0xFB,0x56,0x3B,0x5A,0xD7,0xD6,0xCA,0xDE,0x74,0x91,0xC1,0x76,0x80,0x9D,0xB9,0xE8,0x4D,
  0x72,0xE3,0x52,0x93,0xA6,0x5D,0xD4,0x55,0xA3,0x3A,0xB6,0xA3,0x54,0xD5,0x63,0x08,0x1F,0xCB,0xB6,0x1B,0x89,0x3F,0x87,0x71,
  0xE0,0x0A,0x71,0x17,0x99,0x0B,0x48,0xB0,0xF0,0x91,0xF0,0xB8,0x5E,0x0F,0xB9,0xA0,0xFC,0x03,0x65,0x7B,0xA8,0x83,0x06,0x9D,
  0x68,0xF2,0xDE,0x36,0x66,0x0A,0xEC,0x10,0x05,0xCF,0x52,0x4F,0x41,0x5B,0xE9,0x7C,0x2F,0x2F,0xFA,0x4D,0xE4,0xF7,0xD7,0xA1,
  0x25,0x81,0x40,0x92,0x0B,0x73,0x1E,0xFD,0xC7,0xA0,0xE4,0xF4,0xE7,0xA9,0xEB,0x5D,0xEA,0x2D,0x3A,0x68,0x8D,0xA7,0xD1,0xCF,
  0x2F,0x7C,0xCB,0x88,0xA2,0x9D,0x2E,0x3C,0xB9,0x71,0xEA,0x2A,0x30,0xD9,0xA1,0x75,0x79,0x9A,0x5B,0x78,0x7C,0xD7,0x55,0x78,
  0xD7,0x05,0xB3,0x82,0xDC,0xE7,0x15,0xAD,0xD3,0x7C,0x35,0xA7,0x38,0xD4,0x62,0xD6,0xAC,0xEF,0x62,0x9A,0xDC,0xEC,0x01,0xAE,
  0x94,0xB1,0x21,0x77,0x1C,0x94,0x18,0x04,0x0C,0x71,0xDE,0xB4,0x9A,0x74,0x1A,0x25,0x97,0x87,0x53,0x4B,0x6D,0x36,0x16,0x17,
  0x11,0x8B,0xA5,0xFA,0x88,0xBC,0xD6,0x62,0x7A,0x03,0x2F,0x63,0x8C,0x71,0xDA,0x9D,0x7C,0x15,0xC8,0xE5,0x96,0xD7,0xA2,0x74,
  0x55,0x32,0x00,0xA8,0xA7,0x77,0xDB,0x8A,0x32,0x0B,0xB1,0x16,0x94,0xE1,0xC1,0x12,0xCE,0xCC,0x07,0xDC,0xE7,0x27,0xF4,0xAD,
  0xDC,0x3A,0x2F,0x86,0x62,0xD2,0x6E,0xB5,0x2B,0x9B,0x48,0x9A,0x1B,0x82,0x23,0x61,0x0B,0x72,0x8F,0x9E,0x98,0xE0,0x2E,0x31,
  0xD7,0xBD,0x01,0x61,0xE1,0x7D,0x1F,0x59,0xBA,0x82,0x7D,0x39,0x96,0x1B,0x68,0xB6,0x80,0x9B,0x98,0xB9,0xE7,0x04,0x9D,0xDC,
  0x7E,0x74,0xEA,0x75,0xB1,0x69,0x33,0x01,0x65,0x70,0x15,0x63,0x8B,0x76,0xD1,0x12,0x48,0xDB,0x9B,0xB1,0xC1,0xDB,0xFA,0x9E,
  0x2B,0xDB,0x2B,0xAF,0x2F,0x4F,0x9F,0x23,0x32,0x39,0xC1,0x53,0xD0,0xE7,0xAF,0xF7,0xAD,0xDE,0xAD,0xE0,0x0B,0x4B,0x61,0x36,
  0x2F,0x59,0x1A,0x37,0x2E,0x63,0xE0,0xE3,0xD9,0x72,0x4E,0x09,0xFC,0xEA,0x8B,0x2F,0xD9,0xF2,0xB5,0xAC,0x71,0xBD,0xE4,0x9F,
  0x53,0x20,0x67,0x31,0x95,0x55,0x20,0x77,0xEA,0x48,0xCF,0x6A,0x75,0x9A,0x35,0xB1,0x1C,0x1F,0x86,0x6A,0xD2,0xE1,0x23,0x68,
  0x93,0xC9,0x25,0x25,0x62,0x06,0x0E,0x0B,0xE7,0xBE,0x3E,0x31,0x50,0x9C,0x79,0x12,0xB5,0x94,0x72,0x24,0xC9,0x2C,0x4D,0xBF,
  0xD9,0x58,0x36,0x40,0xFD,0x2B,0x67,0xA5,0x68,0x5A,0x1C,0xD7,0xC1,0x6D,0xEE,0x25,0xB8,0x8A,0x04,0x10,0x93,0x2F,0x63,0xD7,
  0x90,0x3B,0x1F,0xE7,0x53,0xD6,0x7C,0x3F,0x69,0x6B,0x78,0xB2,0x5A,0xDA,0x40,0x91,0xB4,0x64,0xE2,0x44,0x21,0x4E,0x3A,0xED,
  0x3C,0xF3,0xF1,0x59,0xE6,0x49,0xD0,0x16,0x33,0x9A,0x59,0x5D,0x79,0xCB,0xF4,0x72,0xCE,0xBE,0x4B,0x82,0x23,0x79,0x06,0x76,
  0xF3,0x91,0x9A,0xA0,0x19,0x63,0x95,0x62,0x98,0xFE,0xF3,0x76,0xD6,0x42,0x7A,0x1F,0x70,0x6B,0x5B,0xAE,0x68,0x96,0x0A,0xD2,
  0xFD,0x1D,0xA4,0x0C,0x22,0x0B,0x2C,0xB2,0x89,0x36,0xC6,0x10,0x8E,0xA3,0xBD,0x53,0x27,0x86,0x1E,0xE6,0x68,0xEE,0x21,0x49,
  0x13,0xCB,0xDB,0xB1,0x3C,0xB7,0x20,0x8C,0x76,0x62,0x39,0x18,0xA7,0x8E,0x48,0xB5,0x6C,0x0E,0x2D,0x33,0x33,0xE8,0xB6,0xBC,
  0x68,0x1D,0x8A,0xB3,0x72,0x33,0xD8,0xD4,0x2E,0x2F,0x25,0xB4,0xFD,0xF4,0x39,0x4E,0x76,0xC8,0xA3,0x95,0x3F,0x3F,0x63,0x5A,
  0x49,0x3C,0x09,0x73,0x7A,0x92,0x48,0x93,0xCC,0x2E,0x81,0xDC,0x85,0x93,0x2A,0x17,0x3C,0x92,0x7D,0xFE,0x05,0x31,0x87,0xF6,
  0x53,0x77,0x35,0xAC,0xF1,0x1D,0x66,0xDC,0x4E,0x8B,0xFC,0x43,0x0B,0xC7,0x5C,0xD0,0xFD,0x90,0xB0,0xD3,0xA3,0xA9,0x4F,0xA6,
  0x9B,0x6D,0x68,0x5A,0x87,0xB7,0x48,0xEF,0xE6,0x92,0x5B,0xAB,0xAB,0x96,0x1E,0x6C,0x4F,0xB4,0x05,0xDA,0x14,0x85,0x0D,0x81,
  0xE9,0x19,0x20,0x73,0x9E,0xF4,0xA7,0x5D,0xF1,0x45,0x86,0x91,0x35,0xB4,0x7E,0x22,0x73,0x1C,0x50,0x48,0x27,0x82,0x35,0x81,
  0xD4,0x4C,0x17,0x2B,0xBD,0xF0,0x7A,0x12,0x33,0xB7,0x38,0xC6,0x3A,0xD2,0xE8,0xB4,0x1D,0x46,0x49,0xA6,0x93,0xC6,0xD7,0x37,
  0x1A,0x94,0x5F,0xBB,0xB8,0xF2,0x2C,0x6E,0x00,0x31,0x3B,0x64,0x65,0x78,0x19,0x00,0xA8,0x18,0x5C,0x67,0x23,0x19,0xC7,0x2C,
  0x2D,0x6D,0xE0,0x8B,0x45,0xBA,0xB7,0xB8,0xB3,0xD5,0xED,0xEE,0x2D,0xA6,0xFD,0xFA,0xD9,0xC8,0x2E,0x59,0xA5,0x6C,0x36,0x3C,
  0xC6,0x4C,0x2E,0x08,0x00,0xA8,0x18,0x1C,0x9C,0x77,0xA5,0xD2,0x36,0xC6,0x50,0x78,0x9A,0xF7,0x50,0x83,0x4C,0x6B,0x2B,0x7B,
  0xB7,0x9A,0xE8,0x93,0x2D,0xDC,0xD6,0x46,0x38,0xD0,0x8E,0x70,0x07,0x01,0x02,0xAE,0x7B,0x9C,0x93,0xD7,0x9E,0x5A,0xF8,0xC9,
  0xAE,0x63,0x01,0xA2,0x92,0x4F,0xA2,0x9A,0x0C,0xAD,0xBA,0x31,0x63,0x23,0x16,0xE5,0x98,0x60,0xE5,0x76,0xED,0x3C,0x74,0xCD,
  0x27,0xD3,0xAD,0x2E,0xF5,0x3B,0x49,0x6E,0x57,0x5E,0x90,0x47,0x6F,0x26,0x25,0xBB,0xBC,0x28,0xCC,0x70,0xA0,0xB2,0x9C,0x63,
  0x9C,0x90,0x0B,0x11,0xF9,0x1A,0xB7,0x57,0xB5,0xB7,0x49,0xD3,0x40,0x9F,0x4B,0xBF,0xD4,0x6E,0x52,0x15,0x9C,0x4A,0x66,0x0F,
  0x30,0x5C,0x92,0x59,0xCA,0x90,0x14,0xFA,0x78,0x03,0xA8,0xC0,0xC5,0x0C,0xAB,0x5F,0xC8,0x21,0xED,0x9E,0x68,0xF6,0x72,0x46,
  0xA4,0x23,0x2D,0x8C,0x30,0xCC,0x3C,0xC8,0x64,0x4C,0xAB,0x81,0xCF,0x4E,0xAB,0x92,0x08,0x07,0x14,0xB1,0xDE,0x29,0x6F,0x75,
  0x2B,0x07,0xD5,0xAF,0x5E,0xE0,0xCB,0xE6,0x33,0xC1,0x11,0x01,0x63,0x1C,0xED,0x5C,0x00,0xA4,0xF3,0xB7,0x27,0x38,0xDD,0xCD,
  0x01,0x6B,0x75,0xBB,0x4C,0xD4,0xF5,0x29,0x64,0xD4,0x5B,0x44,0xB9,0xB2,0x31,0x2C,0x6E,0x42,0xBC,0x72,0xE3,0x21,0x84,0x83,
  0xF0,0xA8,0xE7,0xD3,0xFF,0x00,0x15,0x0F,0x0E,0xDA,0xCC,0x6E,0x52,0x1D,0x39,0x35,0x5B,0x1B,0x3B,0x43,0xF5,0x44,0xB1,0x94,
  0xC7,0x71,0x95,0x23,0x0D,0x19,0x6C,0x81,0xB8,0x64,0x63,0x39,0xEE,0x6A,0x7C,0x12,0x8E,0x98,0xFB,0xB0,0xAD,0x73,0xC5,0xB3,
  0x78,0x62,0xE2,0x0B,0x7B,0x85,0xBA,0xBC,0x96,0x76,0xF3,0xB7,0xDC,0x45,0xB1,0x54,0x7F,0xB4,0x14,0x1B,0x4E,0x39,0xEB,0x9E,
  0x9C,0xD0,0x5A,0x06,0xBB,0xA7,0x5C,0xF8,0x8E,0xF2,0x03,0x6B,0x7D,0x14,0x72,0xB2,0xDB,0xC7,0x3E,0x9C,0x85,0x8A,0x1F,0x56,
  0x0A,0xB6,0x38,0x27,0x24,0x92,0x7F,0x2A,0xDC,0x69,0x3A,0xAC,0x1A,0x85,0xC3,0x5D,0x6A,0xF6,0xD7,0x76,0x76,0x16,0x87,0x74,
  0x66,0x4C,0xC7,0xE7,0x1D,0xBC,0xB6,0xDE,0x78,0xCE,0x3B,0x80,0x7E,0x71,0x9A,0xF2,0xCF,0x4D,0xB5,0x5B,0xA9,0xA6,0xD3,0x21,
  0x9E,0xCE,0xE6,0xF8,0x34,0x9E,0x74,0x64,0x14,0x47,0x04,0x8D,0xCD,0x93,0xF7,0x1D,0x3E,0xD4,0x66,0xAA,0xB5,0xB0,0x45,0xDD,
  0x80,0xEA,0x7A,0x5F,0xD7,0x68,0xB3,0x60,0x47,0x75,0x72,0x01,0x88,0x97,0x42,0x93,0x4E,0xA0,0xEE,0xC1,0x07,0x1B,0x72,0x30,
  0x30,0x73,0xDE,0xA3,0xE1,0x38,0x22,0x41,0x1D,0xBB,0x68,0xBA,0x9D,0x9D,0x92,0x38,0x73,0x73,0x05,0xEB,0x61,0x43,0x2B,0x65,
  0x1A,0x32,0x7F,0x08,0x27,0xBE,0x48,0xE2,0x98,0xEB,0x3A,0xD5,0xF7,0xD7,0x43,0x1F,0xD0,0xFF,0x00,0xA9,0x69,0xED,0x19,0x53,
  0x35,0xA9,0xD9,0x30,0x94,0x1C,0x81,0x93,0xDB,0xD2,0xC3,0x3C,0xE6,0xAD,0xD2,0xA6,0xBB,0xB8,0x68,0x6E,0x2D,0x34,0xFB,0xCB,
  0x05,0xCB,0xC5,0xFE,0x9D,0x32,0x1C,0xB0,0x27,0xD3,0x24,0x84,0x8C,0x82,0x47,0x23,0x18,0x1D,0xFA,0xF1,0x4A,0x93,0xA6,0x86,
  0xEB,0xB3,0x0F,0x77,0xE1,0xBB,0x37,0xD4,0x3C,0xA9,0x34,0xFB,0x9D,0x43,0x48,0xB5,0x3C,0x5C,0xB4,0x9B,0x95,0xC0,0x18,0xDC,
  0x55,0xB9,0x19,0xE4,0x0D,0xA7,0x19,0x06,0x89,0xD0,0x3C,0x29,0xA3,0x79,0x77,0x52,0x5A,0x2C,0xF6,0xB6,0xD9,0x78,0xC0,0x97,
  0x0D,0xB7,0x9C,0xF1,0x9C,0xEE,0xFB,0xD6,0xFE,0xE5,0xA0,0xB4,0x82,0xE4,0xDC,0x98,0x34,0xFB,0x48,0x94,0xEF,0x1B,0x93,0x2A,
  0x07,0x24,0xE0,0x70,0xC4,0xE3,0xA7,0x43,0xD6,0x96,0xDC,0x4A,0x35,0xB8,0xB4,0xFB,0x9D,0x2F,0x50,0x09,0xA6,0xDC,0x1D,0xC2,
  0x78,0x24,0xCB,0x2A,0x83,0x93,0x85,0x18,0x2A,0x48,0xEA,0x7A,0x0A,0x67,0x7E,0xA0,0x26,0xBA,0x4C,0x43,0x66,0x25,0x0A,0xD1,
  0x5C,0xC1,0xBA,0x29,0x9C,0xA5,0x90,0xB9,0x1B,0x5A,0x43,0x8C,0xE0,0x02,0x33,0x8E,0x0E,0x05,0x33,0xBA,0xD1,0xAC,0xC5,0xA1,
  0xFA,0x8B,0x7B,0x6B,0x40,0x46,0xD4,0xF3,0x08,0x40,0x58,0x8F,0x93,0xC9,0xA3,0xA6,0xD2,0x34,0xB8,0x60,0x82,0x5D,0x4A,0xFA,
  0x0B,0xA8,0x60,0x75,0x78,0x6E,0x2E,0x64,0xDE,0xE5,0x89,0xFC,0x65,0x8E,0x07,0x5E,0x07,0xBF,0x1C,0xD7,0xD6,0xDA,0x95,0x96,
  0xA3,0xA5,0x6A,0x37,0xD7,0x42,0xD2,0x48,0x23,0xDC,0x1A,0x6C,0xEF,0x40,0x17,0x01,0x46,0x18,0x11,0xBB,0x24,0x60,0x0E,0xE7,
  0xBE,0x33,0x53,0xA5,0x7D,0x8D,0xC9,0xD7,0x46,0x5A,0x0F,0x0A,0xE9,0x02,0x4B,0x87,0xBA,0x8E,0x54,0xB6,0x48,0xD7,0x08,0x65,
  0x63,0xDF,0x04,0xF5,0xE9,0xD2,0x98,0xD9,0xF8,0x4A,0x09,0xB5,0x98,0x55,0x34,0xCD,0x1A,0xDA,0x05,0x88,0x10,0xF2,0xC6,0xCE,
  0x65,0xE0,0x8F,0x4F,0x6C,0x8E,0x09,0x24,0xF2,0x08,0xAF,0x1B,0x5A,0xD3,0xEE,0xA3,0x8E,0x1D,0x61,0xE3,0x31,0x86,0x6B,0x88,
  0x6D,0x00,0x07,0x67,0x07,0x21,0xD5,0x3D,0x3F,0x88,0x0C,0x1C,0x9E,0xBF,0x6C,0x09,0xA5,0xC6,0xC3,0x53,0xB2,0x84,0xDE,0x5E,
  0x59,0x8B,0x3F,0x2C,0xC6,0x9E,0x5A,0x32,0xED,0x0F,0xBA,0x4C,0x12,0x4E,0x4F,0xA8,0x2F,0x52,0x07,0x18,0x07,0x14,0x78,0xAE,
  0xD3,0x03,0x6F,0xA1,0xD0,0x6D,0x2F,0x4A,0x97,0xC9,0x8E,0x4B,0x1B,0xB8,0xEE,0x47,0x97,0x24,0xD6,0xD2,0x6E,0x97,0x20,0xFA,
  0x01,0x04,0x63,0x1D,0xB1,0x91,0xDF,0x8A,0xC5,0x5C,0xAC,0xFF,0x00,0xEB,0x71,0x5F,0xC3,0xA9,0x5C,0x49,0x73,0x14,0x8A,0x1E,
  0xD2,0x44,0x56,0x43,0x19,0xC8,0xED,0xD3,0xD5,0xC0,0xFE,0xBD,0x73,0x66,0xA7,0xE2,0xA5,0x86,0xD6,0x4F,0xF5,0x3B,0x46,0x89,
  0x9A,0x69,0x6E,0x1D,0x6E,0xAD,0x84,0x6F,0x2B,0x9C,0xAE,0xF3,0x8E,0x85,0x8E,0xD0,0xB8,0xC9,0x01,0x46,0x0F,0x39,0xA9,0xF8,
  0x62,0xCD,0xF5,0x7F,0x0B,0x4E,0xF1,0x5D,0xFD,0x35,0xD5,0xF5,0xE2,0xCB,0x3A,0xAE,0x24,0x91,0xA1,0x5C,0x61,0x59,0x8F,0x0A,
  0x32,0x0F,0x40,0x7A,0xE0,0xE7,0x34,0x1C,0x94,0x76,0xC0,0xB9,0x78,0x1D,0xE2,0x19,0xB4,0xB1,0xA3,0x9B,0x85,0x8A,0xCF,0xFF,
  0x00,0xDD,0x6E,0x51,0xD6,0x47,0x93,0x6C,0x6C,0xF8,0xEA,0xC7,0x18,0x27,0x20,0x8E,0xC0,0x10,0x28,0x4B,0xCD,0x0A,0xFE,0x1B,
  0xFB,0x1B,0x6B,0xFB,0xDB,0x79,0x23,0x69,0x48,0x9E,0x34,0x93,0xF7,0x8A,0x00,0xCE,0x14,0xBF,0x07,0x1D,0x48,0xE3,0x8E,0xD5,
  0xA1,0x83,0xC2,0xCB,0x7F,0x6C,0xC3,0x54,0x29,0x77,0x0A,0x9C,0x43,0x0B,0xA8,0x68,0xD0,0x7B,0x1E,0x01,0xCF,0xC8,0x23,0xA7,
  0x4C,0x0A,0xF6,0xDE,0xC5,0xED,0x75,0xAB,0x89,0x95,0x4C,0x20,0xA8,0x8C,0x2A,0x8D,0xC7,0x07,0xF8,0xB2,0x49,0x00,0xF1,0xD7,
  0x00,0x90,0x4F,0x38,0xAD,0xCD,0x47,0x4B,0x66,0xDC,0xBB,0xD1,0xCF,0xE5,0xD2,0x74,0xBB,0x6D,0x7E,0x4B,0x6D,0x46,0x6D,0x5B,
  0x6C,0x93,0x1F,0x25,0x62,0x06,0x10,0x42,0xA9,0x23,0x7E,0x3F,0x11,0x3D,0xBE,0x78,0xAD,0xA6,0x97,0x6D,0xA5,0x5C,0xDA,0x83,
  0x14,0x57,0x53,0x5D,0x34,0x67,0x65,0xDD,0xE4,0x26,0x25,0x8F,0x77,0x41,0xE5,0x91,0xEA,0x23,0x91,0xD7,0x8A,0x6F,0x25,0x84,
  0xD1,0x1B,0x58,0x2D,0x42,0xC7,0x6F,0x1C,0x6C,0xBB,0x9A,0x5F,0x2E,0x5C,0x75,0xC6,0x02,0xE7,0x19,0xF7,0x3D,0x3A,0x0A,0xB2,
  0xCE,0xD2,0x1B,0x28,0x26,0x96,0x21,0x7D,0x74,0xCB,0xB9,0xF7,0x4A,0x4B,0xCA,0x85,0x87,0x54,0x51,0xD7,0x23,0xF9,0x7E,0x74,
  0xBC,0xDB,0xDB,0x61,0xA5,0xF0,0x41,0xA9,0x69,0xF6,0x4B,0x67,0x65,0xA4,0xEB,0x17,0x2A,0xF8,0x62,0x84,0x12,0x82,0x37,0xEE,
  0xA4,0xA6,0x31,0xCF,0x27,0x9F,0x6C,0xE6,0x81,0x96,0x0B,0x4D,0x28,0x59,0x9D,0x3F,0x54,0x7B,0x1D,0x36,0x69,0xDA,0x36,0xB4,
  0x86,0x0F,0x35,0x87,0x2B,0xE9,0x47,0x73,0x90,0x39,0xEA,0x47,0x03,0xA6,0x6B,0x4D,0x35,0xBD,0xB4,0x32,0x32,0xC3,0x14,0x7F,
  0x52,0xD6,0xAA,0x04,0xF2,0x28,0xDC,0x57,0x1E,0x9C,0x64,0x60,0x60,0x01,0x82,0x79,0xEB,0xD7,0x9A,0x10,0x88,0xEF,0x20,0x7B,
  0xAB,0xD7,0x49,0xEE,0x9A,0x41,0x04,0x05,0x54,0x48,0x63,0xE3,0x6B,0x02,0x00,0xC8,0xC8,0xEA,0x46,0x7B,0x1C,0xF6,0xAD,0x19,
  0x2B,0x36,0xE8,0xCE,0x68,0x5A,0x7F,0x84,0xE6,0xD6,0xAE,0xAE,0x6D,0xEF,0x64,0xB7,0xF3,0x43,0x28,0x98,0xB9,0x28,0xD2,0x0F,
  0xE1,0x0C,0x70,0xA7,0x9C,0x1C,0x75,0xA7,0x97,0x16,0x57,0xCE,0xE2,0x45,0xD6,0x5A,0xD5,0xE1,0x62,0xB1,0x5C,0x88,0x55,0x93,
  0x69,0x03,0x2A,0x72,0x71,0xCE,0x33,0xC7,0x34,0x24,0xEB,0x17,0x89,0x2C,0x26,0xD3,0x6D,0x2D,0xAE,0x6D,0x20,0x4C,0xB4,0x4C,
  0x61,0x44,0x0A,0x73,0x86,0xF2,0xC9,0x2C,0x3A,0x64,0x1F,0xD3,0x3D,0xA8,0x76,0xD2,0x63,0xD3,0xB4,0xEB,0x5B,0x79,0x35,0x4F,
  0xA8,0x8A,0xCD,0x77,0x13,0x38,0xC1,0x93,0x24,0x80,0xA1,0x33,0x8E,0x09,0xE9,0xF6,0xCE,0x69,0xB9,0xD3,0xBF,0x40,0x92,0x7D,
  0xB2,0xA8,0x3C,0x3D,0xA7,0xD9,0x34,0x37,0xB2,0x48,0x4B,0xC4,0x1A,0x4F,0x30,0x10,0x77,0x9E,0x49,0x93,0x69,0x38,0xEE,0x06,
  0xDC,0xF1,0x9A,0x77,0x66,0xCD,0x65,0x69,0x3D,0xAF,0xD4,0x13,0x0C,0x69,0xBD,0xA4,0x9A,0x52,0x59,0xE4,0x61,0xC2,0x05,0x1F,
  0x97,0x03,0x8A,0xC6,0xCC,0x97,0x77,0x9E,0x1F,0x36,0x5A,0x45,0xEB,0x4E,0xBE,0x70,0xB6,0x91,0x8B,0x08,0xCE,0x73,0x9F,0xC2,
  0x38,0x00,0x01,0xC7,0xBE,0x2B,0x4C,0xF0,0x69,0x7A,0x1E,0x98,0x02,0x25,0xDD,0xFD,0xC2,0x8D,0x8F,0x0F,0x92,0x64,0x7D,0xC9,
  0xEA,0x66,0x8C,0x91,0x81,0x83,0x91,0xC5,0x67,0x17,0x5B,0xDB,0x37,0x2B,0x2B,0xD2,0xF5,0x7B,0x9B,0x39,0x2E,0x53,0x5A,0xB7,
  0xB7,0xFC,0x22,0x41,0x71,0x1B,0xF1,0x1B,0x1F,0xE1,0x6C,0x9E,0x3A,0x8E,0x7A,0x57,0x9A,0xE4,0x52,0x6A,0xA9,0xA8,0x5C,0xC9,
  0x76,0xD6,0xF2,0x44,0xC9,0xB1,0x22,0x8C,0x7E,0xFE,0x26,0x51,0xC8,0x39,0x01,0x4F,0x5E,0x7A,0x63,0xAD,0x59,0x6B,0xA3,0xDE,
  0x48,0x2E,0x6E,0x9A,0xFE,0x6B,0xBB,0x5B,0xD2,0x36,0xDA,0x6F,0x00,0xEC,0xEE,0x32,0x09,0x07,0x93,0xD4,0x73,0xFA,0x1A,0xCB,
  0xD8,0x6B,0xF6,0xB6,0x9A,0xB3,0x68,0xB6,0x2A,0xF7,0xD7,0x4F,0x29,0x88,0x4C,0x5F,0x11,0xC5,0xFC,0x29,0xEA,0x00,0x64,0x64,
  0x60,0xE4,0x72,0x47,0xDE,0xB4,0x53,0x76,0x67,0x49,0x9F,0xFF,0xD9
};
const uint32_t BOOT_SPLASH_MANUL_JPG_LEN = sizeof(BOOT_SPLASH_MANUL_JPG);

// Render full-screen 240x240 boot splash image (Manul-Picture.png) to LCD
void showBootSplashScreen() {
  canvas.fillScreen(TFT_BLACK);
  canvas.drawJpg(BOOT_SPLASH_MANUL_JPG, BOOT_SPLASH_MANUL_JPG_LEN, 0, 0);
  canvas.pushSprite(0, 0);
}

void setup() {
  // Lower ESP32-S3 CPU clock to 80 MHz in active mode for power savings
  setCpuFrequencyMhz(ACTIVE_CPU_FREQ_MHZ);
  Serial.begin(115200);

  // 0. Load saved watch settings from Flash before initializing AXP2101 (charge current & GPS/LoRa rails)
  loadWatchSettings();

  // 1. Start system I2C0 bus (AXP2101, PCF8563, DRV2605L, BMA423)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
  initAXP2101();

  // 2. Initialize display and show Manul boot splash for 1 second (with 250 ms vibration)
  tft.init();
  tft.setBrightness(getFullBrightnessPwm());
  canvas.setColorDepth(16);
  // Offload 115 KB framebuffer to PSRAM when available so internal DRAM stays free for BLE HID + Wi-Fi
  if (psramFound()) {
    canvas.setPsram(true);
  }
  if (canvas.createSprite(240, 240) == NULL) {
    canvas.setPsram(false);
    canvas.createSprite(240, 240);
  }
  showBootSplashScreen();
  uint32_t bootSplashStartMs = millis();

  // 3. Initialize DRV2605L haptic driver and buzz for 250 ms while boot splash is visible
  initDRV2605L();

  // 4. Configure BMA423 INT1 (IO14) & wake button (IO0) and initialize BMA423 hardware pedometer
  pinMode(BMA423_INT_PIN, INPUT_PULLUP);
  pinMode(WAKE_BUTTON_PIN, INPUT_PULLUP);
  initBMA423();

  // 5. Set safe initial level on IR LED pin (GPIO2 -> Q15 MMBT3904) and configure LEDC PWM after tft.init()
  pinMode(IR_SEND_PIN, OUTPUT);
  digitalWrite(IR_SEND_PIN, LOW);
  initIrLedc(irCarrierKhz);
  tft.setBrightness(getFullBrightnessPwm());

  // 6. Start secondary I2C1 bus and initialize FT6236/FT6336U touchscreen (with saved sensitivity)
  pinMode(TOUCH_INT_PIN, INPUT_PULLUP);
  Wire1.begin(TOUCH_SDA_PIN, TOUCH_SCL_PIN, 400000);
  initTouchScreen();

  // 7. Configure GPS UART (38400 baud when GPS enabled, Hi-Z INPUT when disabled in Settings)
  applyGpsUartState();

  // 8. Initial read of RTC time, battery level/charging, and ESP32/BMA423/AXP2101 temperature sensors
  readRTC(rtcTime);
  calMonth = rtcTime.month;
  calYear  = rtcTime.year;
  batteryPercent  = readBatteryPercent();
  batteryCharging = readBatteryCharging();
  recordEsp32TempSample();

  // 9. Apply global Wi-Fi power setting (STA if enabled, OFF if disabled) and load saved networks
  loadSavedWifiCredentials();
  applyWifiPowerState();

  // Keep Manul boot splash on screen for a full 1 second (1000 ms) total
  while (millis() - bootSplashStartMs < BOOT_SPLASH_MS) {
    delay(10);
  }

  lastTouchMs         = millis();
  lastRtcReadMs       = millis();
  lastWifiAutoCheckMs = millis();
  lastStepSaveMs      = millis();

  // Render initial screen (Main menu)
  renderCurrentScreen();
}

void loop() {
  // Stop short touch vibration pulse on schedule
  updateTouchVibration();

  // Continuously read data stream from GPS receiver
  updateGPS();

  // Background step counting from BMA423 sensor
  updateStepCounter();

  // Process touchscreen input and backlight state machine
  updateTouchAndBacklight();

  // Monitor Wi-Fi connection and background 1-minute auto-reconnect
  updateWifiAutoAndConnect();

  // Background check for 2 alarms, Snooze, and countdown timer (with screen wake)
  updateAlarmAndTimer();

  // Continuous SOS signal scheduler (sound + screen strobe + DRV2605L motor)
  if (currentScreen == SCREEN_SOS && sosActive) {
    updateSosSignal();
  }
  // Active Morse code letter playback scheduler
  if (currentScreen == SCREEN_MORSE && morsePlayingIdx >= 0) {
    updateMorsePlayback();
  }
  // Tetris gravity drop scheduler
  if (currentScreen == SCREEN_TETRIS && !tetrisPaused && !tetrisGameOver && blState != BL_STATE_OFF) {
    updateTetrisGame();
  }
  // Hide Paint "SAVED TO FLASH" banner after 1.6 s
  if (currentScreen == SCREEN_PAINT && paintSaveBannerMs != 0 && (millis() - paintSaveBannerMs >= 1600UL)) {
    paintSaveBannerMs = 0;
    renderCurrentScreen();
  }

  // Poll RTC time/date, battery charge/charging, and ESP32 temperature once per second
  uint32_t now = millis();

  // Smooth Stopwatch refresh (20 Hz) while Stopwatch screen is open
  if (currentScreen == SCREEN_STOPWATCH && swRunning && blState != BL_STATE_OFF && (now - lastSwDrawMs >= 50UL)) {
    lastSwDrawMs = now;
    renderCurrentScreen();
  }
  // Smooth Timer refresh (10 Hz) while Timer screen is open
  if (currentScreen == SCREEN_TIMER && timerRunning && blState != BL_STATE_OFF && (now - lastTimerDrawMs >= 100UL)) {
    lastTimerDrawMs = now;
    renderCurrentScreen();
  }
  // Smooth bubble Inclinometer refresh (~20 Hz) while screen is open
  if (currentScreen == SCREEN_INCLINOMETER && blState != BL_STATE_OFF && (now - lastInclDrawMs >= 50UL)) {
    lastInclDrawMs = now;
    renderCurrentScreen();
  }
  // Smooth accelerometer chart refresh (5 Hz) while Accel. sensor screen is open
  if (currentScreen == SCREEN_ACCEL && blState != BL_STATE_OFF && (now - lastAccelDrawMs >= 200UL)) {
    lastAccelDrawMs = now;
    renderCurrentScreen();
  }
  // Continuous PDM microphone spectral analysis (~18 Hz) while Sound spectrometer is open
  if (currentScreen == SCREEN_SOUND_SPEC && blState != BL_STATE_OFF && (now - lastSndSpecDrawMs >= 55UL)) {
    lastSndSpecDrawMs = now;
    updateSoundSpectrometer();
    renderCurrentScreen();
  }
  // Continuous BMA423 200 Hz vibration sampling & FFT Spectroid render (~12.5 Hz) while Accel. spectrometer is open
  if (currentScreen == SCREEN_ACCEL_SPEC && blState != BL_STATE_OFF) {
    sampleAccelSpectrometerOnce();
    if (now - lastAccelSpecDrawMs >= ACC_SPEC_DRAW_INTERVAL_MS) {
      lastAccelSpecDrawMs = now;
      updateAccelSpectrometer();
      renderCurrentScreen();
    }
  }
  // Non-blocking subnet ARP scan while Hosts scan screen is open
  if (currentScreen == SCREEN_HOSTS_SCAN && hostsScanRunning) {
    updateHostsScan();
  }
  // Deferred non-blocking BLE scan trigger & completion/timeout polling
  if (currentScreen == SCREEN_BT_SCAN && btScanPending) {
    runBluetoothScanNow();
  }
  if (btScanRunning) {
    updateBluetoothScan();
  }
  // Deferred BLE HID Keyboard / Presenter startup (runs outside touch handler after motor stops)
  if ((currentScreen == SCREEN_BT_KEYBOARD && bleHidKbStartPending) ||
      (currentScreen == SCREEN_BT_PRESENTER && bleHidKbStartPending)) {
    startBleHidKeyboard();
  }
  // Core-1 UI refresh when BLE HID Keyboard / Presenter connection or LED status changes on Core-0
  if ((currentScreen == SCREEN_BT_KEYBOARD && bleHidKbUiDirty) ||
      (currentScreen == SCREEN_BT_PRESENTER && bleHidKbUiDirty)) {
    bleHidKbUiDirty = false;
    if (blState != BL_STATE_OFF) {
      renderCurrentScreen();
    }
  }
  // Deferred BLE HID Mouse startup (runs outside touch handler after motor stops)
  if (currentScreen == SCREEN_BT_MOUSE && bleHidMouseStartPending) {
    startBleHidMouse();
  }
  // 40 Hz BMA423 accelerometer cursor streaming & UI refresh while BLE HID Mouse is active
  if (currentScreen == SCREEN_BT_MOUSE && bleHidMouseActive) {
    updateBluetoothMouse();
  }
  // Deferred WeatherAPI HTTPS request after rendering "Loading weather..." splash
  if (currentScreen == SCREEN_WEATHER && weatherFetchPending) {
    fetchWeatherDataNow();
  }
  // Deferred NOAA SWPC HTTPS request after rendering "Loading NOAA..." splash
  if (currentScreen == SCREEN_SUN_ACTIVITY && sunFetchPending) {
    fetchSunActivityNow();
  }
  if (now - lastRtcReadMs >= 1000UL) {
    lastRtcReadMs = now;
    readRTC(rtcTime);
    batteryPercent  = readBatteryPercent();
    batteryCharging = readBatteryCharging();
    recordEsp32TempSample();

    if (now - lastStepSaveMs >= STEP_SAVE_INTERVAL_MS) {
      saveStepCountToFlash(false);
    }

    if (blState != BL_STATE_OFF && !earthDragging && currentScreen != SCREEN_SOS) {
      renderCurrentScreen();
    }
  }
}
