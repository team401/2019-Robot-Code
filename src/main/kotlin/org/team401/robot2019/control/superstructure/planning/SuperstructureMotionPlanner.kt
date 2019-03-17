package org.team401.robot2019.control.superstructure.planning

import org.snakeskin.measure.*
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.robot2019.subsystems.arm.control.ArmKinematics
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.geometry.*
import org.team401.robot2019.control.superstructure.planning.command.*
import org.team401.robot2019.subsystems.WristSubsystem
import org.team401.robot2019.util.epsGt
import java.util.*

object SuperstructureMotionPlanner {
    val commandQueue = LinkedList<SuperstructureCommand>()

    /**
     * Last observed state of the arm
     */
    private var lastObservedArmState = ArmState(0.0.Inches, 0.0.Radians, 0.0.RadiansPerSecond)

    /**
     * Last observed state of the wrist
     */
    private var lastObservedWristState = WristState(0.0.Radians, false, false)

    /**
     * Active "target" tool for the system.
     */
    var activeTool = WristMotionPlanner.Tool.HatchPanelTool
    @Synchronized set
    @Synchronized get

    /**
     * Active "target" tool angle for the system.
     */
    private var activeToolAngle = 0.0.Radians

    @Synchronized fun notActiveTool(): WristMotionPlanner.Tool {
        return when (activeTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> WristMotionPlanner.Tool.CargoTool
            WristMotionPlanner.Tool.CargoTool -> WristMotionPlanner.Tool.HatchPanelTool
        }
    }

    // Each system has three states: e stopped, coordinated control and holding
    // The buttons set desired position and switch into coordinated control
    // Maybe this should check the motors are listening beforehand
    //TODO make this class manage timestamp vs dt to detect long pauses in execution (i.e. robot disabled)
    @Synchronized
    fun update(time: Double, dt: Double, armState: ArmState, wristState: WristState) {
        lastObservedArmState = armState     //Update state variables
        lastObservedWristState = wristState

        //Pop the first command from the queue
        if (commandQueue.isNotEmpty()) { //If there are commands in the queue
            val currentCommand = commandQueue.pop() //Remove the first element

            try {
                currentCommand.update(dt, armState, wristState) //Update the command
                if (!currentCommand.isDone()) { //If the command isn't done
                    commandQueue.push(currentCommand) //Put it back at the start of the queue
                }
            } catch (e: Exception) {
                //An exception occured during this command.  Print out some debug info and reset the motion planner.
                System.err.println("Exception encountered in Superstructure Motion Planner")
                System.err.println("Active Command:\t${currentCommand.javaClass.simpleName}")
                System.err.println("Description:\t${currentCommand.getDescription()}")
                System.err.println("Stack trace:")
                e.printStackTrace()

                reset() //Reset the planner
            }
        }
    }

    /**
     * Sets up the controller and the planner for starting configuration
     */
    @Synchronized
    fun startUp(
        startArmState: ArmState,
        startWristState: WristState,
        startingTool: WristMotionPlanner.Tool = WristMotionPlanner.Tool.HatchPanelTool
    ){
        lastObservedArmState = startArmState
        lastObservedWristState = startWristState
        activeTool = startingTool
        reset()
    }

    /**
     * Hacky solution to a dumb problem.  It appears that the motion command classes not being initially JIT compiled
     * causes jerky motion on the first command submitted to the queue upon startup.  This function creates a fake
     * profile and executes it 1000 times, which should be enough to cause the JIT compiler to statically compile the
     * class.  It is entirely possible that there is a bug in the motion planner that causes this until a command is run,
     * but this is unlikely as we are perfect and never make mistakes (please ignore 2017 robot).
     */
    @Synchronized fun preCompile() {
        startUp(
            ArmState(
                Geometry.ArmGeometry.minSafeArmLength + 1.0.Inches,
                0.0.Radians,
                0.0.RadiansPerSecond
            ),
            WristState(0.0.Radians, false, false),
            WristMotionPlanner.Tool.HatchPanelTool
        )

        requestMove(SuperstructureSetpoint(ArmKinematics.forward(
            PointPolar(
                Geometry.ArmGeometry.minSafeArmLength + 1.0.Inches,
                Math.PI.Radians
            )), WristMotionPlanner.Tool.HatchPanelTool, 0.0.Radians, WristSubsystem.CargoGrabberStates.Clamped, WristSubsystem.HatchClawStates.Clamped))

        var time = 0.0
        val dt = 0.1

        for (i in 0 until 1000) {
            SuperstructureMotionPlanner.update(time, dt,  ArmState(
                Geometry.ArmGeometry.minSafeArmLength + 1.0.Inches,
                0.0.Radians,
                0.0.RadiansPerSecond
            ), WristState(0.0.Radians, false, false))
            time += dt
        }

        startUp(ArmState(0.0.Inches, 0.0.Radians, 0.0.RadiansPerSecond),
            WristState(0.0.Radians, false, false), WristMotionPlanner.Tool.HatchPanelTool)
    }

    @Synchronized
    private fun reset(){
        commandQueue.clear()

        //Push through a command to the controller so that when the subsystems
        //enter coordinated control, they'll hold at their current states and not
        //jerk around.
        SuperstructureController.update(
            ArmState(
                lastObservedArmState.armRadius,
                lastObservedArmState.armAngle,
                0.0.RadiansPerSecond //Stop arm motion
            ),
            lastObservedWristState,
            activeTool
        )
    }

    /**
     * Returns whether or not the motion planner is done with it's command queue.
     * If the queue is empty, the planner is considered "done" and this will return true
     */
    @Synchronized fun isDone(): Boolean {
        return commandQueue.isEmpty()
    }

    /**
     * Asks the system to change tools, preferably without moving.
     *
     * Returns true if the motion planner will do the action, false if not
     */
    @Synchronized fun requestToolChange(tool: WristMotionPlanner.Tool): Boolean {
        if (tool == activeTool) { //If we're trying to switch to the same tool, ignore it
            println("Ignoring switch to already selected tool!")
            return false
        }

        when (activeTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> {
                if (lastObservedWristState.hasHatchPanel) { //If we're in the hatch panel tool and we have a hatch panel
                    println("You're a tool!  Drop the hatch panel to switch tools.") //Reject the switch
                    return false
                }
            }

            WristMotionPlanner.Tool.CargoTool -> {
                if (lastObservedWristState.hasCargo) { //Same as above for cargo
                    println("You're a tool!  Drop the cargo to switch tools.")
                    return false
                }
            }
        }

        if (!isDone()) return false //Don't do this unless our other steps are done
        reset()

        val startArmPose = ArmKinematics.forward(lastObservedArmState) //Calculate the current arm pose

        /*
        var hadToMove = false

        val safeToolChangePolar =
            if (startArmPose.x >= 0.0.Inches) {
                ArmKinematics.inverse(Point2d(Geometry.ArmGeometry.minToolChangeX, Geometry.ArmGeometry.minToolChangeY))
            } else {
                ArmKinematics.inverse(Point2d((-1.0).Unitless * Geometry.ArmGeometry.minToolChangeX, Geometry.ArmGeometry.minToolChangeY))
            }

        if (startArmPose.x < Geometry.ArmGeometry.minToolChangeX) {
            //Do a rotation only move from current pose to new pose
            commandQueue.add(RotationOnlyCommand(safeToolChangePolar.theta, activeTool))
            hadToMove = true
        }

        if (lastObservedArmState.armRadius <= safeToolChangePolar.r) {
            //Do an extension only move from current pose to new pose
            commandQueue.add(ExtensionOnlyCommand(safeToolChangePolar.r, activeTool))
            hadToMove = true
        }
        */


        val sideOffset = if (startArmPose.x >= 0.0.Inches) {
            WristMotionPlanner.POSITIVE_X_OFFSET
        } else {
            WristMotionPlanner.NEGATIVE_X_OFFSET
        }

        //Calculate the current tool's angle from the floor
        activeTool = notActiveTool()
        commandQueue.add(SetWristAngleCommand(
            activeTool,
            activeToolAngle,
            startArmPose
        ))

        /*
        if (hadToMove) {
            //TODO move back, skipping for now
        }
        */

        return true
    }

    /**
     * Forces the system to stop its current action and go to the safe home position.
     */
    @Synchronized fun goHome() {
        reset()

        //Determine the active tool from the current angle of the system.  This acts as a tool "reset" if something goes wrong
        val currentPose = ArmKinematics.forward(lastObservedArmState)
        //TODO fix this logic

        /*
        activeTool = if (currentPose.x >= 0.0.Inches) {
            //We're on the right, so 0 to 180 is the hatch tool
            if (lastObservedWristState.wristPosition.value in (-Math.PI / 2.0)..(Math.PI / 2.0)) {
                //Active tool is hatch tool
                WristMotionPlanner.Tool.HatchPanelTool
            } else {
                //Active tool is cargo tool
                WristMotionPlanner.Tool.CargoTool
            }
        } else {
            //We're on the left
            if (lastObservedWristState.wristPosition.value in (-Math.PI / 2.0)..(Math.PI / 2.0)) {
                //Active tool is cargo tool
                WristMotionPlanner.Tool.CargoTool
            } else {
                //Active tool is hatch tool
                WristMotionPlanner.Tool.HatchPanelTool
            }
        }
        */

        if (lastObservedArmState.armRadius < Geometry.ArmGeometry.minSafeArmLength) {
            //We need to extend out to the minimum radius first
            commandQueue.add(ExtensionOnlyCommand(Geometry.ArmGeometry.minSafeArmLength, activeTool))
        }
        val safePoint = ArmKinematics.forward(PointPolar(Geometry.ArmGeometry.minSafeArmLength + 0.1.Inches, lastObservedArmState.armAngle))

        activeToolAngle = 0.0.Radians

        //Now we need to move the arm to the safe location
        commandQueue.add(MoveSuperstructureCommandStaticWrist(safePoint, Point2d(0.0.Inches, Geometry.ArmGeometry.minSafeArmLength + 0.1.Inches), activeTool, 0.0.Radians, Geometry.ArmGeometry.minSafeArmLength))
    }

    @Synchronized fun goToClimb() {
        reset()

        commandQueue.add(RotationOnlyCommand((120.0).Degrees.toRadians(), WristMotionPlanner.Tool.HatchPanelTool))
        commandQueue.add(ExtensionOnlyCommand(Geometry.ArmGeometry.minSafeArmLength + 0.1.Inches, WristMotionPlanner.Tool.HatchPanelTool))
        commandQueue.add(SetWristAngleCommand(WristMotionPlanner.Tool.HatchPanelTool, 0.0.Radians, ArmKinematics.forward(PointPolar(Geometry.ArmGeometry.minSafeArmLength + 0.1.Inches, 90.0.Degrees.toRadians()))))
        commandQueue.add(DelayCommand(1.0.Seconds))
        commandQueue.add(ExtensionOnlyCommand(Geometry.ArmGeometry.armBaseLength + Geometry.ArmGeometry.armExtensionStickout + 2.0.Inches, WristMotionPlanner.Tool.HatchPanelTool))
    }

    /**
     * Asks the system to move to a new pose, while following a series of constraints.
     *
     * Returns true if the motion planner will do the action, false if not
     */
    @Synchronized fun requestMove(setpoint: SuperstructureSetpoint): Boolean {
        if (setpoint.tool != activeTool) {
            println("Please switch tools before performing this move")
            return false
        }

        if (!isDone()) return false
        reset()

        val currentPose = ArmKinematics.forward(lastObservedArmState)

        /*
        //First, we'll obtain the potential pose of the wrist given an immediate rotation to the target angle
        val wristInstantRotatedPose = EndpointKinematics.forward(
            lastObservedArmState,
            WristState(
                WristMotionPlanner.calculateFinalFloorAngle(lastObservedArmState, setpoint.toolAngle, setpoint.tool),
                lastObservedWristState.hasCargo,
                lastObservedWristState.hasHatchPanel
            ),
            setpoint.cargoGrabberState,
            setpoint.hatchClawState
        )

        //We now need to evaluate the x coordinate with the greatest magnitude that the profile will traverse
        //while at the minimum radius.  This can be determined by looking at the angular direction and whether
        //or not the trajectory crosses y = 0.  If it does, the x value at y = 0 is the maximum.  Otherwise, it is
        //the endpoint with the greatest x magnitude.
        val startAngle = lastObservedArmState.armAngle
        val endAngle = ArmKinematics.inverse(setpoint.point).theta
        val startX = ArmKinematics.forward(lastObservedArmState).x
        val endX = setpoint.point.x

        var xMaxTheta =
        if (Math.abs(startX.value) epsGt Math.abs(endX.value)) {
            //We're moving to the left.  The angle with the maximum x angle is the starting angle
            xMaxTheta = startAngle
        } else {
            //We're moving to the right.  The angle with the maximum x angle is the ending angle
            xMaxTheta = endAngle
        }

        if (startAngle.value epsGt endAngle.value) {
            //We are moving clockwise.
            if (Math.PI in endAngle.value..startAngle.value || 0.0 in endAngle.value..startAngle.value) {
                //180 degrees is within this trajectory, so y = 0 is the x coordinate with the greatest magnitude
                //in the negative direction
                xMax = Geometry.ArmGeometry.minSafeWristToolChangeRadius
            }
        } else {
            //We are moving counter-clockwise.
            //Same cases above, except accounting for the fact that Kotlin ranges must start with the smaller number
            if (Math.PI in startAngle.value..endAngle.value || 0.0 in startAngle.value..endAngle.value) {
                xMax = Geometry.ArmGeometry.minSafeWristToolChangeRadius
            }
        }
        */

        //TODO there is definitely a case here where we could end up pushing the extension into the floor.
        //TODO if the radius is too short and we are already on the floor, this will happen
        //TODO add checks to prevent this
        val targetPose = setpoint.point
        val minimumRadius = activeTool.minimumRadius + 1.0.Inches //TODO test if this is really necessary
        if (lastObservedArmState.armRadius < minimumRadius) {
            val safePoint = ArmKinematics.forward(PointPolar(minimumRadius + 0.1.Inches, lastObservedArmState.armAngle))
            commandQueue.add(ExtensionOnlyCommand(minimumRadius + 0.1.Inches, activeTool))
            commandQueue.add(
                MoveSuperstructureCommandStaticWrist(
                    safePoint,
                    targetPose,
                    activeTool,
                    setpoint.toolAngle,
                    minimumRadius
                )
            )
        } else {
            commandQueue.add(
                MoveSuperstructureCommandStaticWrist(
                    currentPose,
                    targetPose,
                    activeTool,
                    setpoint.toolAngle,
                    minimumRadius
                )
            )
        }

        activeToolAngle = setpoint.toolAngle

        return true
    }
}