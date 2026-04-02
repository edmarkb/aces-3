#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <U8g2lib.h>
#include <Wire.h>

// ----- OLED Object -----
// Using SH1106 128x64 I2C, adjust if using SSD1306
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// ----- Function Prototypes -----
void oledSetup();
void oledBootMessage(const char* msg);
void oledShow(float temperature, float humidity, int gasValue, bool fireDetected);

// ----- OLED SETUP -----
void oledSetup() {
  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);  // Normal readable font
  display.sendBuffer();
}

// ----- BOOT MESSAGE -----
void oledBootMessageACES() {
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr); // choose your font

  // Center "ACES"
  const char* title = "ACES";
  int16_t titleWidth = display.getStrWidth(title);
  int16_t xTitle = (128 - titleWidth) / 2; // 128 = OLED width
  display.setCursor(xTitle, 32);           // y = 32 for vertical center
  display.print(title);

  display.sendBuffer();
  delay(2000); // show for 2 seconds
}


// ----- MAIN DISPLAY FUNCTION -----
void oledShow(float temperature, float humidity, int gasValue, bool fireDetected) {
  display.clearBuffer();
  display.setCursor(5, 15); display.print("TEMPERATURE: "); display.print(temperature); display.print(" C");
  display.setCursor(5, 30); display.print("HUMIDITY   : "); display.print(humidity); display.print(" %");
  display.setCursor(5, 45); display.print("GAS/SMOKE  : "); display.print(gasValue);
  display.setCursor(5, 60); display.print("FIRE/FLAME : "); display.print(fireDetected ? "ALERT!" : "SAFE");
  display.sendBuffer();
}

#endif
