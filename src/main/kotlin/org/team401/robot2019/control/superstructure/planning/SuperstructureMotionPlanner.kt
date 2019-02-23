package org.team401.robot2019.control.superstructure.planning

import org.snakeskin.measure.Inches
import org.snakeskin.measure.Radians
import org.snakeskin.measure.RadiansPerSecond
import org.snakeskin.measure.Seconds
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.team401.robot2019.subsystems.arm.control.ArmKinematics
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.geometry.*
import org.team401.robot2019.control.superstructure.planning.command.*
import java.util.*

object SuperstructureMotionPlanner {
    val commandQueue = LinkedList<SuperstructureCommand>()

    /**
     * Maximum time in seconds that a command queue is allowed to run.
     * If a command queue runs for any longer than this time, it will be cancelled,
     * and each subsystem will be put into their respective "hold" states.
     *
     * If this timeout is violated, it is assumed that something is stuck
     */
    private val queueTimeout = 5.0.Seconds

    /**
     * Represents the last time the queue was reset.  This is used to see if we have timed out
     */
    private var lastQueueScheduleTime = 0.0

    /**
     * If this is true, the superstructure has timed out and listening subsystems should switch to holding
     */
    var hasTimedOut = false
    private set
    @Synchronized get

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
    private var activeTool = WristMotionPlanner.Tool.HatchPanelTool

    /**
     * Active "target" tool angle for the system.
     */
    private var activeToolAngle = 0.0.Radians

    private fun notActiveTool(): WristMotionPlanner.Tool {
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
        try {
            lastObservedArmState = armState     //Update state variables
            lastObservedWristState = wristState

            //println("updated arm state: $armState")

            /*
        if (hasTimedOut) { //If we timed out last loop
            reset() //Reset the motion planner
            return //Break execution here
        }
        */
            //Pop the first command from the queue
            if (commandQueue.isNotEmpty()) { //If there are commands in the queue
                val currentCommand = commandQueue.pop() //Remove the first element
                currentCommand.update(dt, armState, wristState) //Update the command
                if (!currentCommand.isDone()) { //If the command isn't done
                    commandQueue.push(currentCommand) //Put it back at the start of the queue
                }

                /*
            //If we have commands, we need to update the watchdog
            if (time - lastQueueScheduleTime > queueTimeout.toSeconds().value) {
                System.err.println("Superstructure reached watchdog timeout!  All mechanisms entering holding.")
                hasTimedOut = true
            }
            */
            }
        } catch (e: Exception) {
            reset()
            throw e
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

        requestMove(ArmSetpoint(ArmKinematics.forward(
            PointPolar(
                Geometry.ArmGeometry.minSafeArmLength + 1.0.Inches,
                Math.PI.Radians
            )), WristMotionPlanner.Tool.HatchPanelTool, 0.0.Radians))

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
     */
    @Synchronized fun requestToolChange(tool: WristMotionPlanner.Tool) {
        if (tool == activeTool) { //If we're trying to switch to the same tool, ignore it
            println("Ignoring switch to already selected tool!")
            return
        }

        when (activeTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> {
                if (lastObservedWristState.hasHatchPanel) { //If we're in the hatch panel tool and we have a hatch panel
                    println("You're a tool!  Drop the hatch panel to switch tools.") //Reject the switch
                    return
                }
            }

            WristMotionPlanner.Tool.CargoTool -> {
                if (lastObservedWristState.hasCargo) { //Same as above for cargo
                    println("You're a tool!  Drop the cargo to switch tools.")
                    return
                }
            }
        }


        val startArmPose = ArmKinematics.forward(lastObservedArmState) //Calculate the current arm pose
        val adjustedArmPose = if (startArmPose.x < 0.0.Inches) { //TODO measure an actual valid x coordinate for tool change position
            Point2d(
                Math.min(startArmPose.x.toInches().value, -Geometry.ArmGeometry.minSafeWristToolChangeRadius.toInches().value).Inches,
                Geometry.ArmGeometry.minSafeWristRotationHeight.toInches()
            )
        } else {
            Point2d(
                Math.max(startArmPose.x.toInches().value, Geometry.ArmGeometry.minSafeWristToolChangeRadius.toInches().value).Inches,
                Geometry.ArmGeometry.minSafeWristRotationHeight.toInches()
            )
        }
        if (startArmPose.y < Geometry.ArmGeometry.minSafeWristRotationHeight) { //If the arm is below the "virtual floor"
            //Before changing tools we need to raise up to the minimum height.  We'll stay at the same x-coordinate:
            commandQueue.add(MoveSuperstructureCommand(startArmPose, adjustedArmPose, tool, Geometry.ArmGeometry.minSafeArmLength))
        }
        //Schedule the command to rotate the wrist
        commandQueue.add(SetWristAngleCommand(notActiveTool(), lastObservedWristState.wristPosition, adjustedArmPose))
        if (startArmPose.y < Geometry.ArmGeometry.minSafeWristRotationHeight) { //If the arm was below the "virtual floor"
            //Now that we've changed tools, we need to move back down to the original pose
            commandQueue.add(MoveSuperstructureCommand(adjustedArmPose, startArmPose, tool, Geometry.ArmGeometry.minSafeArmLength))
        }
    }

    /**
     * Asks the system to move to a new pose, while following a series of constraints.
     */
    @Synchronized fun requestMove(setpoint: ArmSetpoint) {
        if (setpoint.tool != activeTool) {
            println("Please switch tools before performing this move")
            return
        }

        reset()
        val currentPose = ArmKinematics.forward(lastObservedArmState)
        val targetPose = setpoint.point
        val targetToolAngle = setpoint.toolAngle
        val minimumRadius = activeTool.minimumRadius + 1.0.Inches //TODO test if this is really necessary
        if (lastObservedArmState.armRadius < minimumRadius) {
            val safePoint = ArmKinematics.forward(PointPolar(minimumRadius + 0.1.Inches, lastObservedArmState.armAngle))
            commandQueue.add(ExtensionOnlyCommand(minimumRadius, activeTool))
            commandQueue.add(MoveSuperstructureCommand(safePoint, targetPose, activeTool, minimumRadius))
        } else {
            commandQueue.add(MoveSuperstructureCommand(currentPose, targetPose, activeTool, minimumRadius))
        }
        commandQueue.add(SetWristAngleCommand(activeTool, targetToolAngle, targetPose))
    }

    @Synchronized fun testRotationOnly(angleSetpoint: AngularDistanceMeasureRadians) {
        reset()
        commandQueue.add(RotationOnlyCommand(angleSetpoint, activeTool))
    }
}