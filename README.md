[![Language](https://img.shields.io/github/languages/top/team401/2019-Robot-Code.svg)](https://github.com/team401/2019-Robot-Code) 
[![License](https://img.shields.io/github/license/team401/2019-Robot-Code.svg)](https://github.com/team401/2019-Robot-Code/blob/master/LICENSE)
[![CI](https://api.travis-ci.org/team401/2019-Robot-Code.svg?branch=master)](https://travis-ci.org/team401/2019-Robot-Code)

# 2019-Robot-Code

## Software
* [SnakeSkin](https://github.com/team401/SnakeSkin): Modular domain specific language
* [SnakeSkin-Units](https://github.com/team401/SnakeSkin-Units): Statically compiled units-of-measure library
* [SnakeSkin-Gradle](https://github.com/team401/SnakeSkin-Gradle): Custom Gradle plugin for project configuration
* [Taxis](https://github.com/team401/Taxis): Motion planning and trajectory following
* [LightLink](https://github.com/team401/LightLink): Custom I2C LED control library
* [MP-Generator](https://github.com/team401/MP-Generator): Cross-platform trajectory generation app


## Robot features
* Custom 2 speed 6 Neo drive
* Telescoping "light red" arm for intaking and scoring on both sides of robot
* 3 position fully contained ground hatch intake
* 360 degree range of motion wrist with cargo and hatch tools

## Programming
* [Field relative positioning through nonlinear state estimation](https://github.com/team401/2019-Robot-Code/blob/master/src/main/kotlin/org/team401/robot2019/control/vision/VisionState.kt)
* [Dual Limelights](https://github.com/team401/2019-Robot-Code/blob/master/src/main/kotlin/org/team401/robot2019/control/vision/LimelightCamera.kt) for [pose correction](https://github.com/team401/2019-Robot-Code/blob/master/src/main/kotlin/org/team401/robot2019/control/vision/VisionOdometryUpdater.kt) and dynamic trajectory updating
* [Superstructure motion planner](https://github.com/team401/2019-Robot-Code/blob/master/src/main/kotlin/org/team401/robot2019/control/superstructure/planning/SuperstructureMotionPlanner.kt) for synchronized arm extension, rotation, and wrist movement
* [Plotting and simulation](https://github.com/team401/2019-Robot-Code/blob/master/src/main/kotlin/org/team401/robot2019/control/superstructure/armsim/ArmSim.kt) for arm testing without a robot
* [Motion profiled](https://github.com/team401/2019-Robot-Code/blob/master/src/main/kotlin/org/team401/robot2019/control/climbing/ClimbingController.kt) rack-and-pinion climber

