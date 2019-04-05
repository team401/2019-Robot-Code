package org.team401.robot2019.auto

import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.dsl.auto
import org.team401.robot2019.auto.steps.OperatorDriveStep

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