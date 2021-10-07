# teensy_encoder
This is a PlatformIO project that for reading position and velocity feedback
from the Autonics E40 encoder through Teensy 4.1. An easy way to get started
with PlatformIO is to use Visual Studio Code, install the `PlatformIO IDE`
extension, and just simply add this directory as an existing project. 

This project requires `rosserial_arduino` to generate the necessary header files
to be included under the `lib` directory. These files are excluded from the git
repository due to the number of files involved. To install `rosserial_arduino`:
```
sudo apt install ros-$ROS_DISTRO-rosserial-arduino
```
Then, to generate the required header files, first make sure you are in the
`teensy_encoder` directory then execute
```
rosrun rosserial_arduino make_libraries.py lib/
```
