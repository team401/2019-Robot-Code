package org.team401.robot2019.subsystems.arm.planning

import org.snakeskin.measure.Radians
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureMagEncoderTicks
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.robot2019.subsystems.arm.control.ArmKinematics
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.subsystems.arm.control.SuperstructureController
import org.team401.robot2019.subsystems.arm.control.SuperstructureControlOutput
import org.team401.robot2019.subsystems.arm.geometry.*
import kotlin.math.PI

object SuperstructureMotionPlanner{

    private lateinit var currentArmState: ArmState
    private lateinit var currentWristState: WristState
    private lateinit var desiredLocation: PointPolar

    private var done = false
    private var armMotionCommanded = false // Fail safe

    // Each system has three states: e stopped, coordinated control and holding
    // The buttons set desired position and switch into coordinated control
    // Maybe this should check the motors are listening beforehand
    //TODO make this class manage timestamp vs dt to detect long pauses in execution (i.e. robot disabled)
    fun update(dt: Double, armState: ArmState, wristState: WristState): SuperstructureControlOutput {
        currentArmState = armState
        currentWristState = wristState

        // TODO Think through possible logic fails
        if (armMotionCommanded && !done){
            val commandedArmState = ArmMotionPlanner.update(dt) // Returns the arm's angle and position
            // Pass to wrist planner
            val commandedWristState = WristMotionPlanner.update(
                commandedArmState,
                currentWristState
            ) // returns the wrist's angle
            SuperstructureController.update(commandedArmState, commandedWristState)

            if (ArmMotionPlanner.isDone()){
                done = true
            }

            return SuperstructureController.output
        }
        /*
        return SuperstructureControlOutput(
            currentArmState,
            currentWristState,
            0.0
        )
        */
        return SuperstructureController.output //TODO implement the above logic somewhere else
    }

    fun commandMove(desiredLocation: Point2d){
        SuperstructureMotionPlanner.desiredLocation = ArmKinematics.inverse(desiredLocation)

        reset()
        ArmMotionPlanner.setDesiredTrajectory(ArmKinematics.forward(PointPolar(currentArmState.armRadius, currentArmState.armAngle)), desiredLocation)
        armMotionCommanded = true
    }

    fun switchTool(newTool: WristMotionPlanner.Tool): WristState {
        when {
            armMotionCommanded -> return currentWristState
            ArmKinematics.forward(currentArmState).y < Geometry.ArmGeometry.minSafeWristRotationHeight ->{
                //throw InvalidPointException("Wrist is too close to the ground to switch tools!")
            }
            newTool == currentWristState.currentTool -> println("Cannot switch. Eject game piece and try again")
            currentWristState.hasHatchPanel || currentWristState.hasCargo -> println("Cannot switch. Eject game piece and try again")
            else -> return WristState(
                (currentWristState.wristPosition + (PI / 2.0).Radians) as AngularDistanceMeasureRadians,
                newTool,
                false, //TODO update this
            false
            )
        }
        // Default
        return currentWristState
    }

    private fun reset(){
        desiredLocation = PointPolar(currentArmState.armRadius, currentArmState.armAngle)
        ArmMotionPlanner.reset()
        done = false
        armMotionCommanded = false
    }
    private fun convertToEncoderTicks(extension: LinearDistanceMeasureInches): AngularDistanceMeasureMagEncoderTicks{
        return extension.toAngularDistance(Geometry.ArmGeometry.armToInches).toMagEncoderTicks()
    }

    fun isDone(): Boolean{
        return done
    }

}