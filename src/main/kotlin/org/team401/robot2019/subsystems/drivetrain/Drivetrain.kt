package org.team401.robot2019.subsystems.drivetrain

import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix.sensors.PigeonIMU
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DriverStation
import org.snakeskin.component.impl.SparkMaxCTRESensoredGearbox
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.measure.RadiansPerSecond
import org.snakeskin.measure.RevolutionsPerMinute
import org.snakeskin.utility.CheesyDriveController
import org.team401.robot2019.LeftStick
import org.team401.robot2019.RightStick
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.config.HardwareMap
import org.team401.robot2019.config.Physics
import org.team401.taxis.diffdrive.component.IPathFollowingDiffDrive
import org.team401.taxis.diffdrive.component.impl.PigeonPathFollowingDiffDrive
import org.team401.taxis.diffdrive.control.FeedforwardOnlyPathController
import org.team401.taxis.diffdrive.odometry.OdometryTracker

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */

object Drivetrain: Subsystem(500L), IPathFollowingDiffDrive<SparkMaxCTRESensoredGearbox> by PigeonPathFollowingDiffDrive(
    SparkMaxCTRESensoredGearbox(
        TalonSRX(HardwareMap.Arm.leftIntakeWheelId), //TODO grab these from the Arm subsystem
        CANSparkMax(HardwareMap.Drivetrain.leftFrontSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless),
        CANSparkMax(HardwareMap.Drivetrain.leftMidSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless),
        CANSparkMax(HardwareMap.Drivetrain.leftRearSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless)
    ),
    SparkMaxCTRESensoredGearbox(
        TalonSRX(HardwareMap.Arm.rightIntakeWheelTalonId), //TODO grab these from the arm subsystem
        CANSparkMax(HardwareMap.Drivetrain.rightFrontSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless),
        CANSparkMax(HardwareMap.Drivetrain.rightMidSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless),
        CANSparkMax(HardwareMap.Drivetrain.rightRearSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless)
    ),
    PigeonIMU(HardwareMap.Drivetrain.pigeonImuId),
    Geometry.DrivetrainGeometry,
    Physics.DrivetrainDynamics,
    FeedforwardOnlyPathController()
) {
    enum class DriveStates {
        DisabedForFault,
        OpenLoopOperatorControl
    }

    enum class DriveFaults {
        MotorControllerReset
    }

    private val cheesyController = CheesyDriveController(ControlParameters.DrivetrainCheesyDriveParameters)

    val stateEstimator = OdometryTracker(this)

    val driveMachine: StateMachine<DriveStates> = stateMachine {
        state(DriveStates.DisabedForFault) {
            entry {
                stop() //Send no more commands to the controllers
            }
        }

        state(DriveStates.OpenLoopOperatorControl) {
            entry {
                cheesyController.reset()
            }

            action {
                val output = cheesyController.update(
                    LeftStick.readAxis { PITCH },
                    RightStick.readAxis { ROLL },
                    false,
                    RightStick.readButton { TRIGGER }
                )

                tank(output.left, output.right)

                val leftWheelSpeed = Math.abs(left.getVelocity().value).RadiansPerSecond
                val rightWheelSpeed = Math.abs(right.getVelocity().value).RadiansPerSecond
                val leftMotorSpeed = Math.abs(left.master.encoder.velocity / 7.619048).RevolutionsPerMinute
                val rightMotorSpeed = Math.abs(right.master.encoder.velocity / 7.619048).RevolutionsPerMinute

                val leftDelta = leftMotorSpeed - leftWheelSpeed
                val rightDelta = rightMotorSpeed - rightWheelSpeed

                println("Left: ${leftDelta.toLinearVelocity(wheelRadius).toFeetPerSecond()}  ${leftDelta.toRadiansPerSecond()}\tRight: ${rightDelta.toLinearVelocity(
                    wheelRadius).toFeetPerSecond()}  ${rightDelta.toRadiansPerSecond()}")
            }
        }

        default {
            entry {
                stop()
            }
        }
    }

    private fun configureDriveMotorControllers() {
        both {
            master.idleMode = CANSparkMax.IdleMode.kBrake
            master.setSmartCurrentLimit(40)
            master.rampRate = .25
            master.motorType = CANSparkMaxLowLevel.MotorType.kBrushless
            slaves.forEach {
                it.idleMode = CANSparkMax.IdleMode.kBrake
            }

            left.inverted = true
            right.inverted = false
        }
    }

    override fun action() {
        //Detect faults
        if (left.master.getStickyFault(CANSparkMax.FaultID.kHasReset) || left.slaves.any { it.getStickyFault(CANSparkMax.FaultID.kHasReset) } ||
            right.master.getStickyFault(CANSparkMax.FaultID.kHasReset) || right.slaves.any {
                it.getStickyFault(
                    CANSparkMax.FaultID.kHasReset
                )
            }) {
            fault(DriveFaults.MotorControllerReset)
            DriverStation.reportWarning("[Fault] A drive motor controller has reset!", false)
        }

        //Respond to faults
        if (isFaulted(DriveFaults.MotorControllerReset)) {
            driveMachine.setState(DriveStates.DisabedForFault).waitFor()
            configureDriveMotorControllers() //Reconfigure motor controllers
            clearFault(DriveFaults.MotorControllerReset)
            both {
                master.clearFaults()
                slaves.forEach {
                    it.clearFaults()
                }
            }
            println("[Fault Cleared] Drive motor controllers reconfigured")
            driveMachine.setState(DriveStates.OpenLoopOperatorControl)
        }
    }

    override fun setup() {
        configureDriveMotorControllers()

        on(Events.TELEOP_ENABLED) {
            driveMachine.setState(DriveStates.OpenLoopOperatorControl)
        }
    }
}