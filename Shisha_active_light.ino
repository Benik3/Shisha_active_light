// Using HP206C pressure sensor to detect Shisha puffs and activate light
// Benik3

#include <I2C.h>
#include <NeoPixelBus.h>
#include <avr/power.h>
#include <curveFitting.h>

// #define DEBUG // uncomment for Serial debug

#define PixelCount 3
#define PixelPin 2
#define PixelClkPin 3

// HP206C I2C address is 0x76(118)
#define Addr 0x76

#define ORDER 1   // function order of aproximation (1=lienar)
#define COENUM 2  // number of coefficient of the curve (k + q)
#define SAMPLES 4 // number of values stored in array for fitting
#define SLOPE 10.0  // min slope of fitted linear curve needed to detect puffs

//gamma calibration curve
const uint8_t g16[] = {0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 6, 6, 6, 7, 7, 8, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 16, 16, 17, 18, 18, 19, 19, 20, 21, 21, 22, 22, 23, 24, 24, 25, 26, 26, 27, 28, 28, 29, 30, 30, 31, 32, 33, 33, 34, 35, 36, 36, 37, 38, 39, 39, 40, 41, 42, 43, 43, 44, 45, 46, 47, 47, 48, 49, 50, 51, 52, 53, 53, 54, 55, 56, 57, 58, 59, 60, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 99, 100, 101, 102, 103, 104, 105, 106, 107, 109, 110, 111, 112, 113, 114, 115, 117, 118, 119, 120, 121, 122, 124, 125, 126, 127, 128, 130, 131, 132, 133, 135, 136, 137, 138, 139, 141, 142, 143, 144, 146, 147, 148, 150, 151, 152, 153, 155, 156, 157, 159, 160, 161, 163, 164, 165, 166, 168, 169, 170, 172, 173, 175, 176, 177, 179, 180, 181, 183, 184, 185, 187, 188, 190, 191, 192, 194, 195, 197, 198, 199, 201, 202, 204, 205, 207, 208, 209, 211, 212, 214, 215, 217, 218, 220, 221, 223, 224, 226, 227, 229, 230, 232, 233, 235, 236, 238, 239, 241, 242, 244, 245, 247, 248, 250, 251, 253, 255};

double values[SAMPLES];
int32_t value;
uint32_t timer = 0;
uint32_t timer2 = 0;
uint8_t reading = 0;
int8_t light = 0;

// CurveFitting
double coeffs[COENUM];
double t[SAMPLES];

//NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);
NeoPixelBus<DotStarBgrFeature, DotStarMethod> strip(PixelCount, PixelClkPin, PixelPin);
// for hardware SPI (best performance but must use hardware pins)
//NeoPixelBus<DotStarBgrFeature, DotStarSpiMethod> strip(PixelCount); //SCK=13, MOSI=11


//HslColor newColor;
//HslColor black = HslColor(0.0, 0.0, 0.0);
RgbColor newColor;
RgbColor black = RgbColor(0, 0, 0);
float progress = 0.0;

void setup()
{
  power_all_disable();
  power_timer0_enable();
  power_twi_enable();

  // Initializate the random function
  randomSeed(analogRead(A4));

  // Initialise I2C communication as Master
  I2c.begin();

  for (uint8_t i = 0; i < 20; i++) { //make all pins inputs with pullups enabled
    pinMode(i, INPUT_PULLUP);
  }

  pinMode(PixelPin, OUTPUT);  // just WS2812 data and cllock pin is output
  pinMode(PixelClkPin, OUTPUT);

  strip.Begin();
  strip.ClearTo(black);
  strip.Show();

  newColor = RgbColor(HsbColor((float) random(0, 10000) / 10000.0, 1.000, 1.000));

  //CurveFitting y axis
  for (int i = 0; i < SAMPLES; i++)
  {
    t[i] = i;
  }

  // first reading to fill array of values to prevent light up when turning ON arduino
  // Recalibration of HP206c
  I2c.write(Addr, 0x28);
  delay(10);

  // Send OSR and channel setting command
  I2c.write(Addr, 0b01000000); //010_000_00 = 4096 precision, press+temp
  delay(140);

  uint8_t data[3];
  I2c.read(Addr, 0x31, 3, data);

  // Convert the data to 32bits
  value = (int32_t) data[0] << 16 | (int32_t) data[1] << 8 | (int32_t) data[2];
  value = (value ^ 0x800000) - 0x800000;  // signed conversion, 0x800000 = 1u << 23
  value = value / 100;  //convert to meters

  for (int i = 0; i < SAMPLES; i++)
  {
    values[i] = value;
  }

#ifdef DEBUG
  power_usart0_enable();
  Serial.begin(115200);
#endif
}

void loop()
{
  if (millis() - timer2 > 50)
  {
    timer2 = millis();

    if (light == 1 && progress < 1.00) // ramping up
    {
      RgbColor color = RgbColor::LinearBlend(black, newColor, progress);
      color.R = g16[color.R];
      color.G = g16[color.G];
      color.B = g16[color.B];
      color.G = map(color.G, 0, 255, 0, 120); // white balance
      color.B = map(color.B, 0, 255, 0, 100);

      strip.ClearTo(color);
      strip.Show();
      progress = progress + 0.007;
      if (progress >= 1.00)
      {
        progress = 1.00;
      }
    }
    else if (light == -1 && progress > 0.00) // ramping down
    {
      RgbColor color = RgbColor::LinearBlend(black, newColor, progress);
      color.R = g16[color.R];
      color.G = g16[color.G];
      color.B = g16[color.B];
      color.G = map(color.G, 0, 255, 0, 120);
      color.B = map(color.B, 0, 255, 0, 100);

      strip.ClearTo(color);
      strip.Show();
      progress = progress - 0.03;
      if (progress <= 0.00)  // faded fully down to black, change color
      {
        progress = 0.00;
        strip.ClearTo(black);
        strip.Show();
        newColor = RgbColor(HsbColor((float) random(0, 10000) / 10000.0, 1.000, 1.000));
      }
    }
  }

  if (!reading)
  {
    // Recalibration of HP206c
    I2c.write(Addr, 0x28);
    delay(10);

    // Send OSR and channel setting command
    I2c.write(Addr, 0b01000000); //010_000_00 = 4096 precision, press+temp
    timer = millis();
    reading = 1;

  }
  else if (millis() - timer > 140)
  {
    reading = 0;

    // read 3 bytes of altitude
    uint8_t data[3];
    I2c.read(Addr, 0x31, 3, data);

    // Convert the data to 32bits
    value = (int32_t) data[0] << 16 | (int32_t) data[1] << 8 | (int32_t) data[2];
    value = (value ^ 0x800000) - 0x800000;  // signed conversion, 0x800000 = 1u << 23
    value = value / 100;  //convert to meters

    memmove(values, &values[1], sizeof(values[0]) * (SAMPLES - 1));
    values[SAMPLES - 1] = value;

    int ret = fitCurve(ORDER, SAMPLES, t, values, COENUM, coeffs);
    if (ret == 0) // 0 if no error
    {
      if (coeffs[0] > SLOPE)
        light = 1;
      else if (coeffs[0] < (SLOPE * -1.0))
        light = -1;
    }

#ifdef DEBUG
    for (int i = 0; i < COENUM; i++)
    {
      Serial.print(coeffs[i]);
      Serial.print("\t");
    }
    Serial.print(" | ");
    for (int i = 0; i < SAMPLES; i++)
    {
      Serial.print(values[i]);
      Serial.print(", ");
    }
    if (coeffs[0] > SLOPE)
      Serial.println("UP");
    else if (coeffs[0] < (SLOPE * -1.0))
      Serial.println("DOWN");
    else
      Serial.println("");
#endif
  }
}
