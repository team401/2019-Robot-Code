package org.team401.robot2019.control.superstructure.planning

import org.snakeskin.measure.Inches
import org.snakeskin.measure.Radians
import org.snakeskin.measure.RadiansPerSecond
import org.snakeskin.measure.Seconds
import org.team401.robot2019.subsystems.arm.control.ArmKinematics
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.geometry.*
import org.team401.robot2019.control.superstructure.planning.command.MoveSuperstructureCommand
import org.team401.robot2019.control.superstructure.planning.command.SuperstructureCommand
import java.util.*

object SuperstructureMotionPlanner {
    private val commandQueue = LinkedList<SuperstructureCommand>()

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

    // Each system has three states: e stopped, coordinated control and holding
    // The buttons set desired position and switch into coordinated control
    // Maybe this should check the motors are listening beforehand
    //TODO make this class manage timestamp vs dt to detect long pauses in execution (i.e. robot disabled)
    @Synchronized
    fun update(time: Double, dt: Double, armState: ArmState, wristState: WristState) {
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
        val adjustedArmPose = if (startArmPose.x < 0.0.Inches) {
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
            commandQueue.add(MoveSuperstructureCommand(startArmPose, adjustedArmPose, tool))
        }
        //Schedule the command to rotate the wrist
        //TODO add wrist command
        if (startArmPose.y < Geometry.ArmGeometry.minSafeWristRotationHeight) { //If the arm was below the "virtual floor"
            //Now that we've changed tools, we need to move back down to the original pose
            commandQueue.add(MoveSuperstructureCommand(adjustedArmPose, startArmPose, tool))
        }
    }

    /**
     * Asks the system to move to a new pose, while following a series of constraints.
     */
    @Synchronized fun requestMove(endPose: Point2d) {
        reset()
        val currentPose = ArmKinematics.forward(lastObservedArmState)
        if (lastObservedArmState.armRadius < Geometry.ArmGeometry.minSafeArmLength) {
            val safePoint = ArmKinematics.forward(PointPolar(Geometry.ArmGeometry.minSafeArmLength + 0.01.Inches, lastObservedArmState.armAngle))
            commandQueue.add(MoveSuperstructureCommand(currentPose, safePoint, activeTool))
            commandQueue.add(MoveSuperstructureCommand(safePoint, endPose, activeTool))
        } else {
            commandQueue.add(MoveSuperstructureCommand(currentPose, endPose, activeTool))
        }
    }
}