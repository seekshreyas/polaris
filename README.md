## Polaris - inertial navigation system

[Polaris](http://github.com/dronedynamics/polaris/) is open-source inertial nagivation software (INS) for fixed-wing aircraft. It provides realtime estimates to allow for higher-level navigation software to be developed.

For installation instructions (you _will_ need to follow these if you want to get it working), more details and screenshots go to [http://dronedynamics.com/polaris/](http://dronedynamics.com/polaris/).

## Basics

Polaris is a software INS that runs against X-Plane out of the box but can also be used on real hardware.

Polaris is designed to take the following inputs:

- 3-axis accelerometer
- 3-axis gyro
- 3-axis magnetometer (including [magnetic declination](http://en.wikipedia.org/wiki/Magnetic_declination) lookup table)
- Static air sensor (altitude)
- Differential air sensor (indicated airspeed)
- GPS
- Various temperatures (both on the sensors and ambient)

And turn those inputs into a realtime probabilistic model with the following estimates:

- Orientation (pitch, roll, and yaw)
- Position (longitude, latitude, and altitude)
- Speeds (indicated airspeed, true airspeed, speed over ground)
- Wind (x/y/z axis windspeed estimates which help make a more accurate [dead-reckoning](http://en.wikipedia.org/wiki/Dead_reckoning) estimate)

## Features

Polaris is able to build and maintain its realtime probabilistic model primarily using an (extended Kalman filter](http://en.wikipedia.org/wiki/Extended_Kalman_filter) (EKF). There are _two_ major steps to the EKF: **predicting** your current state and **updating** your model based on what actually happened.

Right now Polaris uses [Euler angles](http://en.wikipedia.org/wiki/Euler_angles) but in the future it may use [quaternions](http://en.wikipedia.org/wiki/Quaternion).

More to come in this section…

### Todo

1. Local magnetic declination lookup table
2. Get course over ground working (emulated or hpath)
3. Get cpress working (true airspeed / indicated airspeed ratio)
4. Code up alternate algorithms such as DCM and benchmark it against the EKF
5. Quaternion-based EKF
6. More advanced display/logging
7. Clean up website, installation instructions
8. Make a screencast for setting it up and explaining it
9. Port to other platforms
10. Email Austin Meyer (X-Plane) and see if he's willing to include raw magnetometer values
11. Once all the algorithms are verified (including a write-up on the testing perhaps), introduce sensor noise
12. Your idea here ✈ [tim@dronedynamics.com](mailto:tim@dronedynamics.com)

## Issues

Have a bug or feature request? Please create an issue here on GitHub!

https://github.com/dronedynamics/polaris/issues

## Authors

**Ryan Beall**

**Tim Trueman**

+ http://twitter.com/timtrueman
+ http://timtrueman.com/
+ http://github.com/timtrueman

**Add your name above this line if you send a pull request…**

## License

Copyright 2011 Tim Trueman and Ryan Beall

Licensed under the Apache License, Version 2.0: http://www.apache.org/licenses/LICENSE-2.0