Shumai
======
Shumai (pronounced "Shoe-my") is _the_ open-source inertial nagivation software for fixed-wing aircraft. It provides realtime estimates to allow for higher-level navigation software to be developed.

For installation instructions (you _will_ need to follow these if you want to get it working), more details and photos go to [http://dronedynamics.com/shumai/](http://dronedynamics.com/shumai/).

Todo
======
1. Pull in local magnetic declination
2. Get course over ground working (emulated or hpath)
3. Get position estimation working
4. Code up DCM and benchmark it against the EKF
5. More advanced display (separate things out into sensors, estimates and truth/performance data)
6. Clean up website, installation instructions
7. Make a screencast for setting it up and explaining it
8. Port to other platforms
9. Evaluate options for avoiding singularities
10. Email Austin Meyer and see if he's willing to include raw magnetometer values
11. Once all the algorithms are verified (including a write-up on the testing perhaps), introduce sensor noise
12. Your idea here ✈ [tim@dronedynamics.com](mailto:tim@dronedynamics.com)