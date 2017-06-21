BTLE_Enviro_Mon
===============
STATUS [Working]

An arduino project which allows serial access to environmental data via bluetooth from a smartphone app.

* Monitors: Temperature, Humidity, Atmospheric pressure
* Tries to calculate altitude if you set the sea level barometric pressure at that particular time. Though may be set to uncalibrated by default if this isn’t implemented yet.
* Calculates the dew point
* Outputs text data via bluetooth module HM-10 (clone)
* AT commands are for the HM-10 clone, not the original HM-10 devices.
* iPhone app written by More by Alex van der Lugt https://itunes.apple.com/us/app/hm10-bluetooth-serial-lite/id1030454675?mt=8
* Hardware uses Arduino mini pro(3.3v), SHT31 and BMP280. A 0.96" SPI OLED display is also implemented in code, though not actually used in the hardware any more. OLED code left active. 
* Minor modifications to the libraries were required. Libraries are not included here.
* See www.guvvy.co.uk for a brief writeup of the project

See issues list for details of latest issues.
