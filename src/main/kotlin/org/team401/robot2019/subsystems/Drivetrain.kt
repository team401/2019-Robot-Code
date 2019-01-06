package org.team401.robot2019.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import org.snakeskin.component.Gearbox
import org.snakeskin.component.TalonPigeonIMU
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.units.Degrees
import org.snakeskin.units.Inches
import org.snakeskin.utility.CheesyDriveController
import org.team401.robot2019.LeftStick
import org.team401.robot2019.RightStick
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.config.HardwareMap
import org.team401.robot2019.config.Physics
import org.team401.taxis.diffdrive.component.PathFollowingDiffDrive
import org.team401.taxis.diffdrive.component.impl.SmartPathFollowingDiffDrive
import org.team401.taxis.diffdrive.control.FeedforwardOnlyPathController

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */
object Drivetrain: Subsystem(), PathFollowingDiffDrive by SmartPathFollowingDiffDrive(
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
    TalonPigeonIMU(HardwareMap.Drivetrain.pigeonImuId),
    FeedforwardOnlyPathController(),
    pathGenerationMaxDx = 2.0.Inches,
    pathGenerationMaxDy = 0.25.Inches,
    pathGenerationMaxDTheta = 5.0.Degrees
) {
    enum class DriveStates {
        DriverControl,
        PathFollowing
    }

    private val cheesyController = CheesyDriveController()

    val driveMachine: StateMachine<DriveStates> = stateMachine {
        state(DriveStates.DriverControl) {
            entry {
                cheesyController.reset()
            }

            action {
                cheesyController.update(
                    LeftStick.readAxis { PITCH },
                    RightStick.readAxis { ROLL },
                    true,
                    RightStick.readButton { TRIGGER }
                ).applyTo(this@Drivetrain, ControlMode.PercentOutput)
            }
        }

        default {
            entry {
                stop()
            }
        }
    }

    override fun setup() {
        on (Events.TELEOP_ENABLED) {
            driveMachine.setState(DriveStates.DriverControl)
        }
    }
}