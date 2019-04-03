package org.team401.robot2019.control.drivetrain

import edu.wpi.first.wpilibj.DriverStation
import org.snakeskin.rt.RealTimeExecutor
import org.snakeskin.rt.RealTimeTask
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.subsystems.DrivetrainSubsystem
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d

/**
 * Watches our current odometry pose, as well as a "fed" pose that represents the desired pose from whatever
 * is currently controlling the drive.  If the error between these two poses exceeds a given threshold for n cycles, notify
 * the drivetrain of this and print a warning.
 */
object OdometryWatchdog: RealTimeTask {
    override val name = "OdometryWatchdog"

    private val driveState = DrivetrainSubsystem.driveState
    private var lastFeed = Pose2d.identity()
    private var enabled = false
    private var counter = 0

    private val dtransMax = ControlParameters.DrivetrainParameters.maxOdometryTranslationError.toInches().value
    private val dthetaMax = ControlParameters.DrivetrainParameters.maxOdometryRotationError.toRadians().value

    @Synchronized override fun action(ctx: RealTimeExecutor.RealTimeContext) {
        val currentPose = driveState.getFieldToVehicle(ctx.time)
        val dtrans = Math.abs(currentPose.translation.distance(lastFeed.translation)) //Distances may be directional
        val dtheta = Math.abs(currentPose.rotation.distance(lastFeed.rotation))

        if (dtrans >= dtransMax || dtheta >= dthetaMax) {
            counter ++
        } else {
            counter = 0
        }

        if (isTriggered()) {
            //We've exceeded the limit, spam the conosle.  Outside listeners can use the "isTriggered" function
            //to do their own handling
            DriverStation.reportWarning("[Drivetrain] Odometry error too great!  dtrans: $dtrans in.\tdtheta: ${Math.toDegrees(dtheta)} deg.", false)
        }
    }

    /**
     * Feeds the watchdog with a new setpoint
     */
    @Synchronized fun feed(desiredPose: Pose2d) {
        lastFeed = desiredPose
    }

    /**
     * Resets and enables the watchdog.  Call this when you're about to start something that uses
     * odometry information.
     */
    @Synchronized fun enable(initialState: Pose2d) {
        lastFeed = initialState
        counter = 0
        enabled = true
    }

    /**
     * Disables the watchdog.  Call this when you aren't using odometry information
     */
    @Synchronized fun disable() {
        enabled = false
    }

    /**
     * Returns true if the watchdog is enabled and has exceeded the threshold
     * Any path following controller NEEDS to monitor this as well as feed the watchdog.
     */
    @Synchronized fun isTriggered(): Boolean {
        return enabled && counter >= ControlParameters.DrivetrainParameters.maxOdometryErrorCycles
    }
}