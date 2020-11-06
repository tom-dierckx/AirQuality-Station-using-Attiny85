# Battery powered portable air quality station

As we have all been more indoors during the COVID crisis. Keeping rooms well ventilated is really important. I build this station as a reminder to open my windows when the CO2 levels indoor had risen above a certain level.

What can it measure: CO2, Volatile organic compounds, temperature (degrees Celsius this can be changed in code), humidity and barometric pressure.

I decided to build an AirQ station in march that I have been using to monitor the indoor air levels. The sensors are not calibrated and cannot be relied on for medical uses I just compare the air quality when opening the window and keeping it closed.

The old system was based around the CCS811 and the DHT22 sensor. I wasn't happy about the code so I only published the DHT22 part [here](https://github.com/tom-dierckx/DHT-attiny85-i2c) and the 3d printed case [here](https://www.thingiverse.com/thing:4262574).

We are in a lock-down again so it was time for a revision, a portable version powered by a battery (As my workplace tends to move around a bit). This time I removed some features (previous version could store on an SD card) and switched the Arduino mini with an Attiny85 running on a lower clock (1 Mhz internal).

## Components

### Sensors

I used breakouts of the following sensors that I found on Aliexpress.

CCS811:
- CO2 particles in the air
- Volatile organic compounds

BME280:
- Temperature
- Humidity
- Air pressure

Both of these breakouts expose a i2c interface that we will use.

### Other electric components/modules

Items needed for the electronic circuit:
- LCD display with the i2c to LCD module.
- Attiny85
- Lithium battery charger (based on the TP4056)
- Lithium Ion Battery (Type 18650)
- Red LED (for debugging)
- Resistor 1K Ohm 
- Resistor 10K Ohm
- Button

Printable components:

Battery case: I used this 18650 battery case that I found on thingiverse [here](https://www.thingiverse.com/thing:535688) it worked well even if it is deprecated (used aluminium from a coca cola can as leads).

Arduino UNO for programming the attiny85.

As I'm just doing this in my spare time I just use my glue gun to fix everything in place. So if you do not change the models you are going to need a glue gun and alot of glue :D.

### Connecting everything together.

I just connected all the i2c interfaces together. And attached the button and a red LED to the attiny85. The red led is the `errorPin` in code and will get lit when the attiny cannot setup a connection with a sensor.

The button is used to cycle all the sensor values on the display.

TODO: make a simple diagram

## Code part

Just import this repository in platformIO and you should be good to go, the code isn't that complex. If you have any questions shoot.

## Can we finally build the thing please

I uploaded the STL files to thingiverse. You can find these [here]().
Or in the images directory in this repository. 





