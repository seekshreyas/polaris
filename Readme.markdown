Shumai
======

Shumai is _the_ Extended Kalman Filter (EKF) for fixed-wing aircraft. It provides realtime orientation data to allow for higher-level navigation software to be developed.

In this first release only roll and pitch are included but yaw, position and wind estimation will follow shortly.

What you should understand very clearly about this filter is it has gimbal lock / singularity issues. UAVs aren't used for aerobatics (yet) so if you keep things gentle and smooth (±45 degrees from level in pitch and roll) you'll be just fine.

Shumai was developed by [Tim Trueman](http://github.com/timtrueman) and [Ryan Beall](http://diydrones.com/profile/RyanBeall). It's based on a [BYU paper](http://contentdm.lib.byu.edu/ETD/image/etd1527.pdf) [PDF].

Installation
==========

Mac/Linux/Unix
1. Install X-Plane (this takes the longest)
2. Download source code
3. sudo easy_install numpy
4. Start X-Plane and set the checkbox that outputs data
5. run "python shumai.py" in the command line

Windows
1. Install X-Plane (this takes the longest)
2. Download source code
3. Install the 32-bit Python 2.6 package
4. Install the 32-bit numpy package
5. Set the registry thingy so python works in the command line
6. Start X-Plane and set the checkbox that outputs data
7. run "python shumai.py" in the command line

Future
=============
Yaw estimate
Position estimate
Wind vector estimate
Altitude estimate
Airspeed estimate
Tuning / noise / real hardware testing (perhaps a genetic algorithm for tuning)
Porting to other languages/platforms (microcontrollers)

Screenshots
===========
X-Plane + Shumai running
---------
![Running](http://dronedynamics.com/shumai-running.jpg)

Performance
--------
![Performance](http://dronedynamics.com/shumai-performance.jpg)

FAQ
===

Q: Why did you build this and release it as open source?<br/>
A: It's coming. I promise.

Q: What license is it released under?<br/>
A: The MIT license: http://en.wikipedia.org/wiki/MIT_License

Q: Something doesn't work.<br/>
A: Email or IM me: tim@dronedynamics.com
   
Q: Where's the documentation?<br/>
A: It's coming. I promise.

Q: Why does your filter suffer from gimbal lock / singularities? I want to do aerobatics!<br/>
A: The vast majority of UAVs are not going to be doing crazy maneuvers; a few common sense rules keeps things simple and allows for optimized precision. Also if you think you can solve this problem, email me or just write the code! This is open source after all.
