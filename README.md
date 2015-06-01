# TMP007
TI's IR thermopile temperature sensor in a small breakout board

 Demonstrate basic TMP007 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled pyrometry data out. Sketch runs on the 3.3 V Teensy 3.1.
 
 The TMP007 is an infrared (IR) thermopile sensor that measures the temperature of an object without 
 contacting the object. The integrated thermopile absorbs the infrared energy emitted from the object
 in the sensor field of view. The thermopile voltage is digitized and provided as an input to the integrated
 math engine, along with the die temperature (TDIE). The math engine then computes the corresponding object 
 temperature.
