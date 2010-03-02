How do Extended Kalman Filters work?
======
Let's say you've grown bored of re-watching Battlestar Galactica on DVD and you're looking for a challenge. You decide you want to figure out the pitch and roll of your RC aircraft using nothing more than a 3-axis gyroscope, a 3-axis accelerometer and a differential pressure sensor for airspeed.

In a perfect world and with a gyroscope with infinite precision you could simply accumulate the readings from the gyroscope: if you're reading at 50Hz (50 times per second), you would take the reading and multiply it by a fraction to convert from degrees per 1/50th of a second to per second. (Footnote: That fraction is called "dt". Hopefully you've taken calculus and understand integrals and that dt refers to a variable with respect to time.) If you were to rolling to the right at a constant 15 degrees per second, you would know what your roll is given the time. At half a second you'd be at +7.5 degrees and at 3 seconds you'd be at +45 degrees. That's essentially what inertial navigation is doing.

There's a few problems though that complicate things:

 * Gyros aren't infinitely precise nor you can add up every instant in time. This means that somewhat quickly the accumulated readings from the gyro will differ from reality.
 * Sensor noise (readings from a sensor are essentially a normal distribution around the correct value)

In order to make our lives easier when estimating the pitch and roll we're going to model the flight dynamics of the aircraft. While this limits how the aircraft can fly it means the estimation is easier and more accurate when we place limits and rules on how we can fly.





So yeah I didn't get the time to finish this…is this heading the right direction? Help finish this!