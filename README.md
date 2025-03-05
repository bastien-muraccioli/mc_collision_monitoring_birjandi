mc_rtc Collision Monitoring Birjandi plugin
==

This project is a [mc_rtc] plugin implementing the **Observer-Extended Direct Method** for collision monitoring in robot manipulators using **proprioception and a single IMU sensor**.

This implementation follows the methodology described in the paper:

Observer-Extended Direct Method for Collision Monitoring in Robot Manipulators Using Proprioception and IMU Sensing.

It has been implemented and tested on the **Kinova robotic arm**, with an **IMU mounted on the last joint** to detect collisions at the **end effector**, just like in the experiment described in the paper.

# References

S. A. B. Birjandi, J. Kühn and S. Haddadin, "Observer-Extended Direct Method for Collision Monitoring in Robot Manipulators Using Proprioception and IMU Sensing," in IEEE Robotics and Automation Letters, vol. 5, no. 2, pp. 954-961, April 2020, doi: 10.1109/LRA.2020.2967287.
keywords: {Collision avoidance;Robot sensing systems;Monitoring;Estimation;Torque;Computational modeling;Collision avoidance;sensor fusion;robot safety},

# Author

This implementation was developed by Bastien Muraccioli and is not affiliated with the original authors of the referenced paper.

[mc_rtc]: https://jrl-umi3218.github.io/mc_rtc/
