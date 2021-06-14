# HeroSwarm-Localization-Tracker

Run the python code BasicLocOdo.py for single robot tag detection.
Use the other https://github.com/herolab-uga/heroswarm_v1/blob/master/scripts/localizedControl.py code on HeroSwarm_V1 in HeroLab organization on the robot end to receive data.
Note that the robot is the server to which the localization system connects. So, the robot code in the above link should run first followed by the localization BasicLocOdo.py code in this repo. Ensure they both bind to the same IP and port number.

If no server/ robot code is initiated, the tag detection should still run and the position and odo augmentation should be visible. It is recommended to run all the localization code in a high speed Computer. The camera resolution may be changed according to the detection speed and tag size requirements.
