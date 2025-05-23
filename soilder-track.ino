#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// LCD setup (I2C address might be 0x27 or 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// TinyGPS++ instance
TinyGPSPlus gps;

// Use hardware serial port 2 (GPIO16/17 or GPIO18/19)
HardwareSerial gpsSerial(1); // Use UART1

void setup() {
  // Begin Serial for debugging
  Serial.begin(115200);
  while (!Serial);

  // GPS serial: RX = GPIO19, TX = GPIO18
  gpsSerial.begin(9600, SERIAL_8N1, 18, 19);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  delay(1000);
  lcd.clear();
}

void loop() {
  // Read data from GPS module
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);

    // Print raw GPS NMEA sentence for debugging
    Serial.print(c);
  }

  // Check if a valid location was received
  if (gps.location.isUpdated()) {
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();

    Serial.print("Latitude: ");
    Serial.println(latitude, 6);
    Serial.print("Longitude: ");
    Serial.println(longitude, 6);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Lat:");
    lcd.setCursor(4, 0);
    lcd.print(latitude, 6);

    lcd.setCursor(0, 1);
    lcd.print("Lng:");
    lcd.setCursor(4, 1);
    lcd.print(longitude, 6);
  } else {
    Serial.println("Waiting for GPS fix...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Waiting for GPS");
    lcd.setCursor(0, 1);
    lcd.print("Fix...");
  }

  delay(1000); // update every second
}
