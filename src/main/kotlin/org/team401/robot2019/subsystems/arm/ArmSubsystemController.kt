package org.team401.robot2019.subsystems.arm

import org.snakeskin.measure.Radians
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureMagEncoderTicks
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.robot2019.subsystems.arm.armsim.ArmKinematics
import org.team401.armsim.InvalidPointException
import org.team401.robot2019.subsystems.arm.armsim.Point2d
import org.team401.armsim.PointPolar
import org.team401.robot2019.config.Geometry
import kotlin.math.PI

object ArmSubsystemController{

    private lateinit var currentArmState: ArmState
    private lateinit var currentWristState: WristState
    private lateinit var desiredLocation: PointPolar

    private var done = false
    private var armMotionCommanded = false // Fail safe

    // Each system has three states: e stopped, coordinated control and holding
    // The buttons set desired position and switch into coordinated control
    // Maybe this should check the motors are listening beforehand
    fun update(armState: ArmState, wristState: WristState): ArmSystemMotionPoint{
        currentArmState = armState
        currentWristState = wristState

        // TODO Think through possible logic fails
        if (armMotionCommanded && !done){
            val commandedArmState = ArmPlanner.update() // Returns the arm's angle and position
            // Pass to wrist planner
            val commandedWristState = WristPlanner.update(commandedArmState, currentWristState) // returns the wrist's angle
            val commandedRotationFF = ArmController.calculateRotationFF(commandedArmState) // returns the FF

            if (ArmPlanner.isDone()){
                done = true
            }

            return ArmSystemMotionPoint(commandedArmState, commandedWristState, commandedRotationFF)
        }
        return ArmSystemMotionPoint(currentArmState, currentWristState, 0.0)
    }

    fun commandMove(desiredLocation: Point2d){
        this.desiredLocation = ArmKinematics.inverse(desiredLocation)

        reset()
        ArmPlanner.setDesiredPath(ArmKinematics.forward(PointPolar(currentArmState.armRadius, currentArmState.armAngle)), desiredLocation)
        armMotionCommanded = true
    }

    fun switchTool(newTool: Tool): WristState{
        when {
            armMotionCommanded -> return currentWristState
            ArmKinematics.forward(currentArmState).y < Geometry.ArmGeometry.minSafeWristRotationHeight ->{
                throw InvalidPointException("Wrist is too close to the ground to switch tools!")
            }
            newTool == currentWristState.currentTool -> println("Cannot switch. Eject game piece and try again")
            currentWristState.hasGamePiece -> println("Cannot switch. Eject game piece and try again")
            else -> return WristState((currentWristState.wristPosition + (PI/2.0).Radians) as AngularDistanceMeasureRadians, newTool, false)
        }
        // Default
        return currentWristState
    }

    private fun reset(){
        desiredLocation = PointPolar(currentArmState.armRadius, currentArmState.armAngle)
        ArmPlanner.reset()
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