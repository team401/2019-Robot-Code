package org.team401.robot2019.vision2

import edu.wpi.first.wpilibj.CounterBase
import edu.wpi.first.wpilibj.Encoder
import org.snakeskin.measure.MagEncoderTicks
import org.snakeskin.rt.RealTimeExecutor
import org.snakeskin.rt.RealTimeTask
import org.team401.robot2019.control.vision.VisionManager
import org.team401.robot2019.subsystems.DrivetrainSubsystem
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.geometry.Twist2d

object RobotStateEstimator: RealTimeTask {
    override val name = "vision2_state_estimator"

    private val driveEncoderLeft = Encoder(0, 1, false, CounterBase.EncodingType.k4X)
    private val driveEncoderRight = Encoder(2, 3, false, CounterBase.EncodingType.k4X)
    private val imu = DrivetrainSubsystem.imu

    private var gyroOffset = Rotation2d.identity()

    fun setHeading(heading: Rotation2d) {
        val currentHeadingRaw = Rotation2d.fromDegrees(imu.fusedHeading)
        gyroOffset = heading.rotateBy(currentHeadingRaw.inverse())
    }

    fun getHeading(): Rotation2d {
        return Rotation2d.fromDegrees(imu.fusedHeading).rotateBy(gyroOffset)
    }

    fun resetEncoders() {
        driveEncoderLeft.reset()
        driveEncoderRight.reset()
    }

    private var leftEncoderPrevDistance: Double
    private var rightEncoderPrevDistance: Double
    private var prevHeading: Rotation2d

    init {
        driveEncoderRight.setReverseDirection(true)
        resetEncoders()
        setHeading(Rotation2d.identity())
        leftEncoderPrevDistance = getLeftEncoderInches()
        rightEncoderPrevDistance = getRightEncoderInches()
        prevHeading = getHeading()
    }


    fun getLeftEncoderInches(): Double {
        val ticks = driveEncoderLeft.raw
        val rads = ticks / 4096.0 * 2.0 * Math.PI
        return rads * VisionConstants.driveWheelRadiusInches
    }

    fun getRightEncoderInches(): Double {
        val ticks = driveEncoderRight.raw
        val rads = ticks / 4096.0 * 2.0 * Math.PI
        return rads * VisionConstants.driveWheelRadiusInches
    }

    fun getLeftEncoderInchesPerSecond(): Double {
        val ticksPerSecond = driveEncoderLeft.rate * 4.0
        val radsPerSecond = ticksPerSecond / 4096.0 * 2.0 * Math.PI
        return radsPerSecond * VisionConstants.driveWheelRadiusInches
    }

    fun getRightEncoderInchesPerSecond(): Double {
        val ticksPerSecond = driveEncoderRight.rate * 4.0
        val radsPerSecond = ticksPerSecond / 4096.0 * 2.0 * Math.PI
        return radsPerSecond * VisionConstants.driveWheelRadiusInches
    }

    override fun action(ctx: RealTimeExecutor.RealTimeContext) {

        /*
        //Drive odometry
        val leftDistance = getLeftEncoderInches()
        val rightDistance = getRightEncoderInches()
        val dLeftDistance = leftDistance - leftEncoderPrevDistance
        val dRightDistance = rightDistance - rightEncoderPrevDistance
        val gyroAngle = getHeading()
        val lastMeasurement = RobotState.getLatestFieldToVehicle().value
        val odometryTwist = DifferentialKinematics.forwardKinematics(
            lastMeasurement.rotation, dLeftDistance, dRightDistance, gyroAngle
        )
        val measuredVelocity = DifferentialKinematics.forwardKinematics(
            dLeftDistance, dRightDistance, prevHeading.inverse().rotateBy(gyroAngle).radians
        ).scaled(1.0 / ctx.dt)
        val predictedVelocity = DifferentialKinematics.forwardKinematics(
            getLeftEncoderInchesPerSecond(),
            getRightEncoderInchesPerSecond()
        ).scaled(ctx.dt)
        RobotState.addObservations(ctx.time, odometryTwist, measuredVelocity, predictedVelocity)

        leftEncoderPrevDistance = leftDistance
        rightEncoderPrevDistance = rightDistance
        prevHeading = gyroAngle

        //Vision updates
        val activeLimelight = VisionManager.frontCamera
        val frameCapTime = ctx.time - activeLimelight.getLatencySeconds()
        val target = activeLimelight.getTarget()
        RobotState.addVisionUpdate(frameCapTime, target, activeLimelight)
        */
        val leftRaw = driveEncoderLeft.raw
        val rightRaw = driveEncoderRight.raw
        //s = r*theta
        //s = 60
        //r = ?
        //theta = measured
        //r = s / theta
        println("Left: $leftRaw\tRight: $rightRaw")

    }
}