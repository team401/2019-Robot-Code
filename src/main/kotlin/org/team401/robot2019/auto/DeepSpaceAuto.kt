package org.team401.robot2019.auto

import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.DelayStep
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.dsl.auto
import org.snakeskin.measure.Seconds
import org.team401.robot2019.auto.steps.DisableVisionStateEstimator
import org.team401.robot2019.auto.steps.DriveTrajectoryStep
import org.team401.robot2019.auto.steps.EnableVisionStateEstimator
import org.team401.robot2019.auto.steps.OperatorDriveStep
import org.team401.robot2019.control.drivetrain.CriticalPoses
import org.team401.robot2019.control.drivetrain.Trajectories

object DeepSpaceAuto: RobotAuto(20L) {
    override fun assembleAuto(): SequentialSteps {
        return auto {
            /*
            parallel {
                sequential {
                    delay(2.0.Seconds)
                    step(EnableVisionStateEstimator(CriticalPoses.leftRocketFarGoal, true))
                }

                sequential {
                    step(DriveTrajectoryStep(Trajectories.habLeftLowToFarRocketHighLeft, true))
                    step(DisableVisionStateEstimator())
                }
            }
            */

            step(OperatorDriveStep())
        }
    }
}