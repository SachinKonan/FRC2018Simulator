# First Robotics Competition Simulator

In order to simulate real-time gameplay for the First Robotics Competition, I made a Python Simulator which features a birds-eye-view of the robot in the FRC game and all the obstacles. Users can define auto-paths and run the robot using the up, down, left, and right arrows, and the program automatically performs collision and perimeter detection. Additionally, a program is provided, called "azizline.py" to generate simulator, and real robot paths in the autonomous period.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

What things you need to install the software and how to install them

```
Python 3.5
```

### Installing

```
pip install pygame
pip install numpy
```

## Generating Auto Paths

There are a list of provided paths in the paths.txt file which can attach the scale or switch from any placement on the field (Left, Middle, Right). To generate custom paths, first edit the side and width lengths of the robot in azizline.py:

```
#these were the dimensions of our robot
real_robot_side = 33.5
real_robot_len = 38.5
```

then:
```
cd into working directory of FRC2018 simulator
python3 azizline.py
```

The 2018 FRC game will popup at a 2-time scale, in otherwords every 1 px increase is 0.5 inches. You first select a starting point and can click points around the field you would like to navigate through. The blue outer lines are your frame perimeter, so pick points where your frame perimeter won't intersect any obstacles.

When finished, press 'y' to print the path or 'q' to completely quit.
The first line of the output is a list of the pixel points of our robot's path. You will need this for the simulator. The second line contains a tuple of list. The first list in the tuple are the robotinstructions for the Simulator. The second list in the tuple are the robotinstructions for your Real Robot.
## Running the Simulator

Either taking in the information from the 'azizline.py' or the 'paths.txt' file you can now run a simulation. In 'simulator_game.py' replace:
```
tasks =
path =
```

Also in 'objects.txt' replace the last two numbers in "Start" with your robot dimensions*2:
```
#So our robot was 38.5 long by 32.5 wide, but since the simulator is 2-scale your multiply these dimensions by 2
Start: 279, 28, 77, 67
```

Now you can run the simulator with:
```
python simulator_game.py
```

After the auto-path has finished running, you can use key commands to go to yellow cubes and deposit them in either the switch or scale stations.
## Deployment
To transfer the auto-path from the simulator to real-life, you must take the outputted robotpath from "azizline.py", which is the second list in the tuple of the second line. Then you can use a method like in "AutoDrive.java", which will go through each of the actions, sequentially.
## Authors

* **Sachin Konan** - Programmer for 5465, Si Se Puede Robotics Team.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details
