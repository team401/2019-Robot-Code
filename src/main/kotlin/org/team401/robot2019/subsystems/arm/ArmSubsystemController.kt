package org.team401.robot2019.subsystems.arm

import org.snakeskin.units.MagEncoderTicks
import org.snakeskin.units.Radians
import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureCTREMagEncoder
import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.units.measure.distance.linear.LinearDistanceMeasureInches
import org.snakeskin.units.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond
import org.team401.armsim.ArmKinematics
import org.team401.armsim.Point2d
import org.team401.armsim.PointPolar
import org.team401.robot2019.config.Geometry

object ArmSubsystemController{

    private lateinit var currentArmAngle: AngularDistanceMeasureRadians
    private lateinit var currentArmExtension: LinearDistanceMeasureInches
    private lateinit var currentArmRadius: LinearDistanceMeasureInches
    private lateinit var currentArmVelocity: AngularVelocityMeasureRadiansPerSecond
    private lateinit var desiredLocation: PointPolar

    private var done = false
    private var motionCommanded = false // Fail safe

    // Each system has three states: e stopped, coordinated control and holding
    // The buttons set desired position and switch into coordinated control
    // Maybe this should check the motors are listening beforehand
    fun update(armState: ArmState): ArmSystemMotionPoint{
        currentArmAngle = armState.position.second
        currentArmExtension = armState.position.first
        currentArmVelocity = armState.armVelocity
        currentArmRadius = (currentArmExtension + Geometry.ArmGeometry.minArmLength) as LinearDistanceMeasureInches


        if (motionCommanded && !done){
            val commandedArmState = ArmPather.update()
            // Pass to wrist pather
            // TODO Double check these values
            val targetExtension = convertToEncoderTicks((commandedArmState.position.first - Geometry.ArmGeometry.minArmLength) as LinearDistanceMeasureInches)
            val targetRotation = commandedArmState.position.second.toUnit(MagEncoderTicks) as AngularDistanceMeasureCTREMagEncoder

            val rotationFeedForward = ArmController.calculateRotationFF(commandedArmState)
            return ArmSystemMotionPoint(targetExtension, targetRotation, rotationFeedForward)
        }
        val extensionEncoderTicks = currentArmExtension.toAngularDistance(Geometry.ArmGeometry.armToInches) as AngularDistanceMeasureCTREMagEncoder
        val angleEncoderTicks = currentArmAngle.toUnit(MagEncoderTicks) as AngularDistanceMeasureCTREMagEncoder
        return ArmSystemMotionPoint(extensionEncoderTicks, angleEncoderTicks, 0.0)
    }

    fun commandMove(desiredLocation: Point2d){
        this.desiredLocation = ArmKinematics.inverse(desiredLocation)

        reset()
        ArmPather.setDesiredPath(ArmKinematics.forward(PointPolar(currentArmRadius, currentArmAngle)), desiredLocation)
        motionCommanded = true
    }

    private fun reset(){
        desiredLocation = PointPolar(currentArmRadius, currentArmAngle.toUnit(Radians) as AngularDistanceMeasureRadians)
        ArmPather.reset()
        done = false
        motionCommanded = false
    }
    private fun convertToEncoderTicks(extension: LinearDistanceMeasureInches): AngularDistanceMeasureCTREMagEncoder{
        return extension.toAngularDistance(Geometry.ArmGeometry.armToInches).toUnit(MagEncoderTicks) as AngularDistanceMeasureCTREMagEncoder
    }

}