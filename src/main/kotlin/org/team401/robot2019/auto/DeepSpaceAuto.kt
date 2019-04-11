package org.team401.robot2019.auto

import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.dsl.auto
import org.snakeskin.measure.Inches
import org.snakeskin.measure.Seconds
import org.team401.robot2019.auto.steps.climber.HomeClimberStep
import org.team401.robot2019.auto.steps.drivetrain.*
import org.team401.robot2019.auto.steps.superstructure.*
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.control.drivetrain.CriticalPoses
import org.team401.robot2019.control.drivetrain.Trajectories
import org.team401.robot2019.control.superstructure.SuperstructureRoutines
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner

object DeepSpaceAuto: RobotAuto(20L) {
    const val doesAutoWork = false //oops

    override fun assembleAuto(): SequentialSteps {
        if (!doesAutoWork) {
            return auto {
                parallel {
                    step(PrepareDriverVisionStep())
                    step(OperatorDriveStep())
                    sequential {
                        step(ArmHomeStep()) //The first step in any auto routine is to home the arm.  This can't be interrupted
                        step(SuperstructureGoHomeStep()) //Next, move to the home position
                        step(SuperstructureSwitchToolStep(WristMotionPlanner.Tool.HatchPanelTool)) //Configure for hatch tool
                    }
                    step(HomeClimberStep()) //Homes the climber.
                }
            }
        }

        //val startToRocketTrajectory =

        return auto {
            //Step 1: Arm homing and initial trajectory
            parallel {
                //Prepare cameras
                step(PrepareVisionStep())

                //Arm configuration sequence
                sequential {
                    step(ArmHomeStep()) //The first step in any auto routine is to home the arm.  This can't be interrupted
                    step(SuperstructureGoHomeStep()) //Next, move to the home position
                    step(SuperstructureSwitchToolStep(WristMotionPlanner.Tool.HatchPanelTool)) //Configure for hatch tool
                }

                sequential {
                    step(HomeClimberStep()) //Homes the climber.  This must happen before we can drive.
                    //Drive to the near rocket
                    parallel {
                        step(DriveTrajectoryStep(Trajectories.level1HabToNearRocketLeft, true))

                        //Wait for drive to pass an x value, move to scoring position, enable vision
                        sequential {
                            step(WaitForOdometry(WaitForOdometry.Axis.X, WaitForOdometry.Direction.POSITIVE, 150.0))
                            step(EnableVisionStateEstimator(CriticalPoses.fieldToNearRocketLeft, true)) //Enable vision pose updater
                            step(SuperstructureMoveStep(ControlParameters.SuperstructurePositions.rocketHatchHighFront)) //Move to scoring position
                        }

                    }

                    //Turn off vision
                    step(DisableVisionStateEstimator())

                    //Score the hatch
                    step(SuperstructureScoreStep())

                    //Wait a bit for the claws to retract
                    delay(0.5.Seconds)

                    //Drive from the near rocket to the inbounding station, move to the intake position, enable vision
                    parallel {
                        step(DriveTrajectoryStep(Trajectories.nearRocketLeftToInboundingStationLeft))

                        sequential {
                            step(WaitForOdometry(WaitForOdometry.Axis.X, WaitForOdometry.Direction.NEGATIVE, 160.0))
                            step(EnableVisionStateEstimator(CriticalPoses.fieldToInboundingStationLeft, false))
                            step(SuperstructureIntakeStep(SuperstructureRoutines.Side.BACK))
                        }
                    }

                    //Turn off vision
                    step(DisableVisionStateEstimator())

                    //Grab the hatch
                    step(SuperstructureStopIntakingStep())

                    //Wait a bit for the claws to close
                    delay(0.5.Seconds)

                    //Drive from the inbounding station to the rocket, move to scoring position, enable vision
                    parallel {
                        step(DriveTrajectoryStep(Trajectories.inboundingStationLeftToNearRocketLeft))

                        sequential {
                            step(WaitForOdometry(WaitForOdometry.Axis.X, WaitForOdometry.Direction.POSITIVE, 110.0))
                            step(EnableVisionStateEstimator(CriticalPoses.fieldToNearRocketLeft, true))
                            step(SuperstructureMoveStep(ControlParameters.SuperstructurePositions.rocketHatchMidFront))
                        }
                    }

                    //Turn off vision
                    step(DisableVisionStateEstimator())

                    //Score the hatch
                    step(SuperstructureScoreStep())

                    //Wait a bit for the claws to release
                    delay(0.5.Seconds)
                }
            }
            step(OperatorDriveStep())
        }
    }
}