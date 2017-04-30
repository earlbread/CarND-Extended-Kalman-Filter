Extended Kalman Filter
===

This is a implementation of Extended Kalman Filter in C++.

Dependencies
---

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

Basic Build Instructions
---

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/obj_pose-laser-radar-synthetic-input.txt output.txt`
