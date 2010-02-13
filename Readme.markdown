** FIRST OF ALL **
This is a WIP!

Shumai
======

Shumai is _the_ Extended Kalman Filter (EKF) for fixed-wing aircraft. It provides realtime orientation data to allow for higher-level navigation software to be developed.

In this first release only roll and pitch are included but yaw, position and wind estimation will follow shortly.

Shumai was developed by [Tim Trueman](http://github.com/timtrueman) and Ryan Beall. It's based on a [BYU paper](http://contentdm.lib.byu.edu/ETD/image/etd1527.pdf) [PDF].

Quickstart
==========
1. Install X-Plane (this takes the longest)
2. Download source code
3. Install Python
4. Install numpy
5. Install curses (if available)
6. Start X-Plane and run "python shumai.py" in the command line

Configuration
=============
Todo.

Screenshots
===========
I'll put up better screenshots later…

X-Plane + Shumai running
---------
![Running](http://dronedynamics.com/s/pretty_matrix_outputs-20100209-014613.jpg)

Performance
--------
![Performance](http://dronedynamics.com/s/EKF_data-20100213-004222.jpg)

FAQ
===
Q: Something doesn't work.<br/>
A: Email or IM me: tim@dronedynamics.com
   
Q: Where's the documenation?<br/>
A: It's coming. I promise.