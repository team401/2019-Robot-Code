package org.team401.robot2019.subsystems.drivetrain

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.snakeskin.component.Gearbox
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.units.Degrees
import org.snakeskin.utility.CheesyDriveController
import org.team401.robot2019.LeftStick
import org.team401.robot2019.RightStick
import org.team401.robot2019.SketchyPigeonFactory
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.config.HardwareMap
import org.team401.robot2019.config.Physics
import org.team401.taxis.diffdrive.component.PathFollowingDiffDrive
import org.team401.taxis.diffdrive.component.impl.SmartPathFollowingDiffDrive
import org.team401.taxis.diffdrive.control.FeedforwardOnlyPathController
import org.team401.taxis.diffdrive.odometry.OdometryTracker
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.geometry.Translation2d
import org.team401.taxis.trajectory.TimedView
import org.team401.taxis.trajectory.TrajectoryIterator
import java.text.DecimalFormat

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */
object Drivetrain: Subsystem(100L), PathFollowingDiffDrive by SmartPathFollowingDiffDrive(
    Geometry.DrivetrainGeometry,
    Physics.DrivetrainDynamics,
    Gearbox(
        TalonSRX(HardwareMap.Drivetrain.leftFrontTalonId),
        TalonSRX(HardwareMap.Drivetrain.leftMidFTalonId),
        TalonSRX(HardwareMap.Drivetrain.leftMidRTalonId),
        TalonSRX(HardwareMap.Drivetrain.leftRearTalonId)
    ),
    Gearbox(
        TalonSRX(HardwareMap.Drivetrain.rightFrontTalonId),
        TalonSRX(HardwareMap.Drivetrain.rightMidFTalonId),
        TalonSRX(HardwareMap.Drivetrain.rightMidRTalonId),
        TalonSRX(HardwareMap.Drivetrain.rightRearTalonId)
    ),
    SketchyPigeonFactory.getPigeon(HardwareMap.Drivetrain.pigeonImuId),
    FeedforwardOnlyPathController()
) {

    private fun radiansPerSecondToTicksPer100ms(rad_s: Double): Double {
        return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0
    }

    val stateEstimator = OdometryTracker(this)

    enum class DriveStates {
        DriverControl,
        PathFollowing
    }

    private val cheesyController = CheesyDriveController()
    private val velocityScale = 1883.0
    private val pA = 0.0

    val driveMachine: StateMachine<DriveStates> = stateMachine {
        state(DriveStates.DriverControl) {
            entry {
                cheesyController.reset()
            }

            action {
                /*
                val translation = LeftStick.readAxis { PITCH } * 1883.0
                val rotation = RightStick.readAxis { ROLL } * 1883.0

                val leftSet = translation + rotation
                val rightSet = translation - rotation

                tank(ControlMode.Velocity, leftSet, rightSet)

                SmartDashboard.putNumber("left_error", left.getVelocity().value - leftSet)
                SmartDashboard.putNumber("right_error", right.getVelocity().value - rightSet)

*/

                cheesyController.update(
                    LeftStick.readAxis { PITCH },
                    RightStick.readAxis { ROLL },
                    true,
                    RightStick.readButton { TRIGGER }
                ).applyTo(this@Drivetrain, ControlMode.PercentOutput)

            }
        }


        state(DriveStates.PathFollowing) {
            entry {
                pathManager.reset()
                setPose(Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)))
                val trajec = pathManager.generateTrajectory(false, listOf(Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)), Pose2d(8.0 * 12.0, 0.0, Rotation2d.fromDegrees(0.0))), listOf(), 3.0 * 12.0, 3.0 * 12.0, 9.0)
                pathManager.setTrajectory(TrajectoryIterator(TimedView(trajec)))
            }

            rtAction {
                val out = pathManager.update(time, driveState.getFieldToVehicle(time))
                val leftVel =
                    radiansPerSecondToTicksPer100ms(out.left_velocity)
                val rightVel =
                    radiansPerSecondToTicksPer100ms(out.right_velocity)
                val leftAccel = radiansPerSecondToTicksPer100ms(
                    out.left_accel
                ) / 1000.0
                val rightAccel = radiansPerSecondToTicksPer100ms(
                    out.right_accel
                ) / 1000.0
                val leftFf = out.left_feedforward_voltage / 12.0
                val rightFf = out.right_feedforward_voltage / 12.0

                SmartDashboard.putNumber("left_vel_err", left.getVelocity().value - leftVel)
                SmartDashboard.putNumber("right_vel_err", right.getVelocity().value - rightVel)

                left.master.set(ControlMode.Velocity, leftVel, DemandType.ArbitraryFeedForward,
                    leftFf + pA * leftAccel / 1023.0)

                right.master.set(ControlMode.Velocity, rightVel, DemandType.ArbitraryFeedForward,
                    rightFf + pA * rightAccel / 1023.0)

                if (pathManager.isDone) {
                    stop()
                    setState(DriveStates.DriverControl)
                }
            }
        }

        default {
            entry {
                stop()
            }
        }
    }

    override fun setup() {
        left.setInverted(true)
        left.setSensor(FeedbackDevice.CTRE_MagEncoder_Relative)
        right.setSensor(FeedbackDevice.CTRE_MagEncoder_Relative)

        left.setPosition(0.0.Degrees)
        right.setPosition(0.0.Degrees)
        setPose(Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(0.0)))

        setNeutralMode(NeutralMode.Brake)

        on (Events.TELEOP_ENABLED) {

            driveMachine.setState(DriveStates.DriverControl)
        }
    }

    const val ACCEL_TO_MS2 = 0.0005985504150390625
    val xyz = ShortArray(3)
    val xyzDps = DoubleArray(3)

    val df = DecimalFormat("##.###")

    override fun action() {
        imu.getBiasedAccelerometer(xyz)
        imu.getRawGyro(xyzDps)

        val x = xyz[0] * ACCEL_TO_MS2
        val y = xyz[1] * ACCEL_TO_MS2
        val z = xyz[2] * ACCEL_TO_MS2

        val xDps = xyzDps[0]
        val yDps = xyzDps[1]
        val zDps = xyzDps[2]

        println(driveState.getLatestFieldToVehicle().value)

    }
}