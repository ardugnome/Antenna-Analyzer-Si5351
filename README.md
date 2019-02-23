# Antenna-Analyzer-Si5351
An arduino (nano) based antenna analyzer based on K6BEZ design with modifications by DG7EAO.

This analyzer is a working model of the DG7EAO antenna analyzer based on the K6BEZ design, 
with additional hardware modification in the form of the original MCP6002 op-amp, 
an ILI9341 TFT 2.8 screen with a working XPT2046 chip touch interface. 
//includes a rudimentary touch interface code and connections

Libraries used are Adafruit_ILI9341.h,Adafruit_GFX.h, URTouch.h.

Unit is powered by 9v battery on Arduino Nano's RAW pin but it can take power off the USB cable as well.
Upon power-up it performs a 1-30 MHz sweep then reads the value of the frequency potentiometer 
at A2 and the value of the sweep width potentiometer at A3.
By depressing SW1 user is presented with a menu that allows frequency and sweep width 
selection via the two potentometers mentioned above. 
Range was extended to 153MHz but it is unusable past 150MHz
Calibration file is included but is based on my own personal build and AA143 diodes used.
Power from Si5351 has been reduced to 2ma or -3dBm for better performance in the VHF range.
TFT screen requires 5v to 3v3 level converters!
Frequency settings depend on span but analyzer covers from 1 to 35 MHz, 43-57MHz, 139-153MHz. 
Frequency setting "31" is a full HF spectrum sweep (1-30MHz) 
VNA software is available from the DG7EAO site or elswere to control the analyzer via the Arduino USB port if desired.

2/20/2019 - A new version starting the sweep at 100Khz will be uploaded shortly, should be helpful for 137KHz and 472Khz experiments. It makes use of the span potentiometer RV2 in conjunction with the frequency potentiometer RV1 to start the scan at the lowest frequency the chip permits. Both RV1 and RV2 should be set for minimum value.

Connections:
   
Power =  9V battery to Arduino RAW pin.
Arduino Nano 5V pin can be used for TFT Backlight through a 40-50 ohm resistor
Arduino Nano 3v3 can be used to power the Si5351 and provide 3v3 to TFT 



Note: Leave ICSP header alone in case you brick the Nano, and connect to the corresponding Arduino pins.
(ex. MOSI=11, MISO=12, SCK=13)


Credits:
http://www.dg7eao.de/arduino/antennen-analysator/
https://sites.google.com/site/k6bezprojects/antenna-analyser
https://groups.io/g/SoftwareControlledHamRadio
