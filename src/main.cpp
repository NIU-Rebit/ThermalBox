/***************************************************************************
  This is a library for the AMG88xx GridEYE 8x8 IR camera

  This sketch makes an inetrpolated pixel thermal camera with the
  GridEYE sensor and a 2.4" tft featherwing:
	 https://www.adafruit.com/product/3315

  Designed specifically to work with the Adafruit AMG8833 Featherwing
          https://www.adafruit.com/product/3622

  These sensors use I2C to communicate. The device's I2C address is 0x69

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Dean Miller, James DeVito & ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

/***************************************************************************
  Revisions
  1.0     Deiss     Modified version from Adafruit Example, added scale, measurement at center, autoscale
  1.1     Deiss     Exchanged TFT driver for better performance regarding framerate
  1.2     Deiss     Changed datatype of MINTEMP and MAXTEMP to fit pixels
  1.3     Deiss     Bugfix MINTEMP and MAXTEMP typecast instead of different datatype

  Housing availablte at Thingiverse

  Used components:
  Thermal sensor            https://learn.adafruit.com/adafruit-amg8833-8x8-thermal-camera-sensor/overview
  Display                   https://de.aliexpress.com/item/240x320-2-8-SPI-TFT-LCD-Touch-Panel-Serial-Port-Module-with-PCB-ILI9341-5V-3/32815224002.html?spm=a2g0s.9042311.0.0.pr9FRa
  MCU                       Wemos D1 Mini clone

  Pinouts
  MCU         Device
  D1          AMG SCL         *might go wrong, please try D2
  D2          AMG SDA         *might go wrong, please try D1
  Gnd         Dispaly GND, AMG Gnd
  3v3         Dispaly Vcc,Display LED,Display RST, AMG Vcc
  D0          Dispaly T_IRQ   (or PEN(Touch Chip Interrupt))
  D3          Display D/C     (or DC(Data/Command Select))
  D8          Display CS      (TFT or CS1(LCD Chip Select))
  D7          Display SDI     (MOSI)
  D6          Dispaly SDO     (MISO)
  D5          Display SCK     (CLK)

 ***************************************************************************/
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include <Adafruit_GFX.h> // Core graphics library
// #include <Adafruit_ILI9341.h>
#include "config.h"
#include <Esp.h>

//Comment this out to remove the text overlay
// #define SHOW_TEMP_TEXT

//low range of the sensor (this will be blue on the screen)
//#define MINTEMP 20

//high range of the sensor (this will be red on the screen)
//#define MAXTEMP 28

Adafruit_AMG88xx amg;
unsigned long delayTime;

#define AMG_COLS 8
#define AMG_ROWS 8
float pixels[AMG_COLS * AMG_ROWS];
uint16_t MINTEMP = 20;
uint16_t MAXTEMP = 28;
char buf[10];
uint16_t centerTemp;
boolean measure = true;
unsigned long tempTime = millis();
unsigned long batteryTime = 1;

#define METRIC ;
#define INTERPOLATED_COLS 24
#define INTERPOLATED_ROWS 24
#define PIN_INT D0

float get_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void set_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y, float f);
void get_adjacents_1d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void get_adjacents_2d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
float cubicInterpolate(float p[], float x);
float bicubicInterpolate(float p[], float x, float y);
void interpolate_image(float *src, uint8_t src_rows, uint8_t src_cols,
                       float *dest, uint8_t dest_rows, uint8_t dest_cols);
void drawScale();
void drawMeasurement();
void getLimits();
void drawBattery();
void drawpixels(float *p, uint8_t rows, uint8_t cols, uint8_t boxWidth, uint8_t boxHeight, boolean showVal);

void setup()
{
  delay(500);
  Serial.begin(115200);
  Serial.println("\n\nAMG88xx Interpolated Thermal Camera!");

  tft.begin();
  tft.setRotation(2);
  tft.fillScreen(ILI9341_BLACK);

  drawScale();

  // default settings
  if (!amg.begin())
  {
    Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
    while (1)
    {
      delay(4000);
      ESP.restart();
    }
  }

  Serial.println("-- Thermal Camera Test --");
}

void loop()
{
  //read all the pixels
  amg.readPixels(pixels);

  float dest_2d[INTERPOLATED_ROWS * INTERPOLATED_COLS];
  interpolate_image(pixels, AMG_ROWS, AMG_COLS, dest_2d, INTERPOLATED_ROWS, INTERPOLATED_COLS);
  uint16_t boxsize = min(tft.width() / INTERPOLATED_COLS, tft.height() / INTERPOLATED_COLS);

  drawpixels(dest_2d, INTERPOLATED_ROWS, INTERPOLATED_COLS, boxsize, boxsize, false);

  // Update battery everx 30s
  if (batteryTime < millis())
  {
    drawBattery();
    batteryTime = millis() + 30000;
  }

  // Check touch screen
  if (digitalRead(PIN_INT) == false)
  {

    getLimits();

    MAXTEMP = MAXTEMP + 3.0;
    MINTEMP = MINTEMP - 3.0;
    Serial.printf("%d touch!\n", millis() / 1000);
    drawScale(); // Redraw scale with new limits

    if (millis() - tempTime > 3000)
    {
      measure = !measure;
      tempTime = millis();
      tft.fillRect(0, 300, 100, 16, ILI9341_BLACK);
    }
  }
  else
  {
    tempTime = millis();
  }
}

void drawpixels(float *p, uint8_t rows, uint8_t cols, uint8_t boxWidth, uint8_t boxHeight, boolean showVal)
{
  int colorTemp;
  for (int y = 0; y < rows; y++)
  {
    for (int x = 0; x < cols; x++)
    {
      float val = get_point(p, rows, cols, x, cols - y); // mirror image by adding "cols - x"
      if (val >= MAXTEMP)
        colorTemp = MAXTEMP;
      else if (val <= MINTEMP)
        colorTemp = MINTEMP;
      else
        colorTemp = val;

      uint8_t colorIndex = map(colorTemp, MINTEMP, MAXTEMP, 0, 255);
      colorIndex = constrain(colorIndex, 0, 255);
      //draw the pixels!
      uint16_t color = val * 2;
      tft.fillRect(boxWidth * x, boxHeight * y, boxWidth, boxHeight, camColors[colorIndex]);

      if (measure == true && y == 12 && x == 12)
      {
        drawMeasurement(); //Draw after center pixels to reduce flickering
      }
    }
  }
}

// Draw a temperature scale with min/max value
void drawScale()
{
  tft.setTextSize(2);

  uint16_t MIN = MINTEMP;
  uint16_t MAX = MAXTEMP;

#ifdef IMPERIAL
  MIN = MIN * 1.8 + 32;
  MAX = MAX * 1.8 + 32;
#endif

  int xpos;
  if (MAX > 99)
  {
    xpos = 188;
  }
  else
  {
    xpos = 204;
  };

  sprintf(buf, "%2d", MIN);
  tft.setCursor(2, 248);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.print(buf);

  sprintf(buf, " %2d", MAX);
  tft.setCursor(xpos, 248);
  tft.print(buf);

  for (int i = 0; i < 240; i++)
  {
    tft.drawLine(i, 270, i, 290, camColors[i + 8]);
  }
}

// Draw a circle + measured value
void drawMeasurement()
{
  // Mark center measurement
  tft.drawCircle(120, 120, 3, ILI9341_WHITE);
  // Measure and print center temperature
  centerTemp = pixels[27];

#ifdef IMPERIAL
  centerTemp = centerTemp * 1.8 + 32;
#endif

  tft.setCursor(0, 300);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  sprintf(buf, "%s:%2d", "Temp", centerTemp);
  tft.print(buf);
}

// Calculate min/max value
void getLimits()
{
  MINTEMP = 255;
  MAXTEMP = 0;

  for (int i = 0; i < AMG_COLS; i++)
  {
    for (int j = 0; j < AMG_ROWS; j++)
    {
      //MINTEMP = min(MINTEMP, (uint16_t)(pixels[i, j]));
      //MAXTEMP = _max(MAXTEMP, (uint16_t)(pixels[i, j]));
      if ((uint16_t)pixels[i, j] < MINTEMP)
        MINTEMP = (uint16_t)pixels[i, j];
      if ((uint16_t)pixels[i, j] > MAXTEMP)
        MAXTEMP = (uint16_t)pixels[i, j];
    }
  }
}

int measureBattery()
{
  uint16_t adcValue = analogRead(A0);
  int volt = adcValue / 102.3 * 4.5; // Using 130kOhm resistor
  return volt;
}

// Draw battery symbol
void drawBattery()
{
  int volt = measureBattery() - 32; // range from 3.2V - 4.2V
  volt = constrain(volt, 1, 10);

  // draw battery
  tft.drawRect(208, 304, 30, 10, ILI9341_WHITE);
  tft.fillRect(237, 306, 3, 6, ILI9341_WHITE);
  tft.fillRect(209, 305, 28, 8, ILI9341_BLACK);
  if (volt > 3)
    tft.fillRect(209, 305, volt * 3 - 2, 8, ILI9341_GREEN);
  else
    tft.fillRect(209, 305, volt * 3 - 2, 8, ILI9341_RED);
}