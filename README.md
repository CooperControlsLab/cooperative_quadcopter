# cooperative_quadcopter
We fly coopertive teams of quadcopters
Author: Justin Rooney

## Crazyswarm Useful Starting Links

[USC Paper Discussing Crazyswarm Setup](http://act.usc.edu/publications/Hoenig_Springer_ROS2017.pdf)<br/>
[Official Crazyswarm Doucmentation](https://crazyswarm.readthedocs.io/en/latest/index.html)<br/>
[Condensed Crazyswarm Paper](http://act.usc.edu/publications/Preiss_ICRA2017.pdf)

## Useful Tips for 705 setup

mocap server at IP address: 199.98.21.246<br/>
`ping 199.98.21.246`<br/>

install vscode on Linux<br/>
`sudo snap install --classic code`

## Path to acquire needed skills to fly drones with ROS and CF architectures

[Udacity Introductory Python Course](https://www.udacity.com/course/introduction-to-python--ud1110?fbclid=IwAR0sJsyTI8hajwfGY3absE38o74-UE_sEc6gVM15cFBakyygvYMt5I4VR-0)<br/>
[A Fantastic and Detailed C++ Course](https://www.learncpp.com/)<br/>
[Udacity Introductory Bash Course](https://www.udacity.com/course/shell-workshop--ud206?fbclid=IwAR2WmDUQmo6p9Eyq1fQXz1gs7hwWDnXLb0d-27z7NG3vBEe876c1x-pRFFM)<br/>
[Programming Robots with ROS Book](http://marte.aslab.upm.es/redmine/files/dmsf/p_drone-testbed/170324115730_268_Quigley_-_Programming_Robots_with_ROS.pdf)<br/>
[ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)<br/>
[HackerRank (Practice with Unit Testing)](https://www.hackerrank.com/)<br/>
[CodeWars (Practice with Unit Testing)](https://www.codewars.com/)<br/>

## Script credit to the following:

Python Scripts for Crazyflie 2.0<br/>
Cooper Union 2018<br/>
Authors: Andrew Chin and Andrew Mosin<br/>
[Follow Instructions Here](https://www.bitcraze.io/getting-started-with-the-crazyflie-2-0/)

To run cfclient (UI based)

`cd ~/crazyflie/crazyflie-clients-python/bin`
`python3 cfclient`

Labeled drones underneath with small numbers 1-2. These have updated firmware and unique radio URI's.
`radio://0/35/2M/E7E7E7E701`
`radio://0/80/2M/E7E7E7E702`

To run scripts to ensure radio is working:
`cd crazyswarm/crazyflie-lib-python/examples`

`gcc -Wall -Werror -Wextra -pedantic -L. ViconDataStreamSDK_CPPTest.cpp -llibViconDataStreamSDK_CPP.so -o test`

To get Vicon data stream pushed to Linux navigate to: 
`cd ~/crazyswarm/ros_ws/src/externalDependencies/libmotioncapture/externalDependencies/vicon-datastream-sdk/build`

run:
`./ViconDataStreamSDK_CPPTest 199.98.21.246`

