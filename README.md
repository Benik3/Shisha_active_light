# Shisha_active_light
Shisha active LED light responsive to puffs

The project use Arduino Nano, APA102 (or WS2812 or similar LEDs) and HP206C waterproof pressure sensor.
By change of the pressure the LEDs light up when you start to smoke and blend down when you stop.
On each end of puff (when light goes back to black) new random color is generated for next puff.

### Libraries:  
[NeoPixelBus](https://github.com/Makuna/NeoPixelBus) - nice library for LED control  
[CurveFitting](https://github.com/Rotario/arduinoCurveFitting) - for linear aproximation to detect changes in pressure  
[I2C by Rambo/Wayne Truchsess](https://github.com/rambo/I2C) - Not available in Arduino library manager. This library works better and has better functions then standard Wire library.
But it doesn't work on Atmega32u4 (Arduino Leonardo/Pro micro). Anyway with default Wire library I had lock-ups on Atmega32u4.
