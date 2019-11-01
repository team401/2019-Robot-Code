package org.team401.robot2019.control.superstructure

import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner

/**
 * Class implementing heuristics for automatically switching the active side of the robot
 * based on specific drive commands, as well as handling prioritization of locking the active
 * side based on different events
 */
class SuperstructureSideManager {
    /**
     * A series of potential actions which can trigger locking and unlocking conditions
     */
    enum class Action {
        SUPERSTRUCTURE_STOWED,
        TOGGLED,
        DRIVER_SET_BACK,
        DRIVER_SET_FRONT,
        SUPERSTRUCTURE_MOVED_TO_SETPOINT,
        SCORE_STARTED,
        SCORE_FINISHED,
        CARGO_INTAKE_STARTED,
        HATCH_INTAKE_STARTED,
        INTAKE_FINISHED,
        VISION_STARTED,
        VISION_FINISHED,
        AUTO_STARTED,
        AUTO_FINISHED
    }

    private var lockedByToggle = false
    private var lockedByScore = false
    private var lockedByIntake = false
    private var lockedByVision = false
    private var lockedBySuperstructureMove = false
    private var lockedByAuto = false

    @Synchronized
    fun reset() {
        lockedByToggle = false
        lockedByScore = false
        lockedByIntake = false
        lockedByVision = false
        lockedBySuperstructureMove = false
        lockedByAuto = false
    }

    /**
     * Returns true if we are locked to the current side
     */
    @Synchronized
    fun isLocked(): Boolean {
        return lockedByToggle || lockedByScore || lockedByIntake || lockedByVision || lockedBySuperstructureMove
    }

    /**
     * Returns the side that should be active currently
     */
    @Synchronized
    fun getSide(): SuperstructureRoutines.Side {
        return currentSide
    }

    //Some state variables:
    private var currentSide = SuperstructureRoutines.Side.FRONT //The currently active side
    private var wasLastMoveActionIntaking = false //Tracks whether or not the last action which required a move was intaking
    private var lastIntakeSide = SuperstructureRoutines.Side.FRONT

    private var lastDriveReportTimestamp = 0.0
    private var driveAccumForwards = 0.0
    private var driveAccumReverse = 0.0

    /**
     * Reports that a specific action has occurred
     */
    @Synchronized
    fun reportAction(action: Action) {
        when (action) {
            Action.SUPERSTRUCTURE_STOWED -> {
                //Stowing the superstructure should clear all locks except auto
                lockedByToggle = false
                lockedByScore = false
                lockedByIntake = false
                lockedByVision = false
                lockedBySuperstructureMove = false
                //We should also clear the intaking flag
                wasLastMoveActionIntaking = false
            }

            Action.SUPERSTRUCTURE_MOVED_TO_SETPOINT -> {
                //Clear the intaking flag
                wasLastMoveActionIntaking = false
                //Set lock flag
                lockedBySuperstructureMove = true
            }

            Action.TOGGLED -> {
                //Toggle occured
                if (!lockedByScore && !lockedByIntake && !lockedByVision && !lockedByAuto) {
                    //If we aren't locked by scoring, intaking, vision, or auto, flip the active side
                    currentSide = when (currentSide) {
                        SuperstructureRoutines.Side.FRONT -> SuperstructureRoutines.Side.BACK
                        SuperstructureRoutines.Side.BACK -> SuperstructureRoutines.Side.FRONT
                    }
                    //Lock the side by toggle
                    lockedByToggle = true
                }
            }

            Action.DRIVER_SET_BACK -> {
                //Driver requested back of robot
                if (!lockedByScore && !lockedByIntake && !lockedByVision && !lockedByAuto) {
                    //If we aren't locked by scoring, intaking, vision, or auto, set the active side to back
                    currentSide = SuperstructureRoutines.Side.BACK

                    //Lock the side by toggle
                    lockedByToggle = true
                }
            }

            Action.DRIVER_SET_FRONT -> {
                //Driver requested back of robot
                if (!lockedByScore && !lockedByIntake && !lockedByVision && !lockedByAuto) {
                    //If we aren't locked by scoring, intaking, vision, or auto, set the active side to front
                    currentSide = SuperstructureRoutines.Side.FRONT

                    //Lock the side by toggle
                    lockedByToggle = true
                }
            }

            Action.SCORE_STARTED -> {
                //Scoring started.  Lock by score
                lockedByScore = true
            }

            Action.SCORE_FINISHED -> {
                //Scoring finished.  Unlock from score, toggle, and intake (if that's stuck somehow)
                lockedByScore = false
                lockedByToggle = false
                lockedByIntake = false
                lockedBySuperstructureMove = false
                wasLastMoveActionIntaking = false
            }

            Action.HATCH_INTAKE_STARTED -> {
                //Intaking started.  Lock by intake
                lockedByIntake = true
                lockedBySuperstructureMove = false
                if (!lockedByVision && !lockedByAuto && !lockedByToggle) {
                    //If we aren't locked by something else already
                    //Check if the last action was intaking
                    if (wasLastMoveActionIntaking) {
                        //If it was, set the current side to the last side we used for intaking
                        currentSide = lastIntakeSide
                    } else {
                        //The last move was not intaking.  We are intaking now, set the side
                        lastIntakeSide = currentSide
                        //And mark the flag
                        wasLastMoveActionIntaking = true
                    }
                }
            }

            Action.CARGO_INTAKE_STARTED -> {
                //Cargo intaking started.  Lock by intake
                lockedByIntake = true

                //Cargo intaking must happen on the back of the robot.  Therefore, disregard any other locks
                currentSide = SuperstructureRoutines.Side.BACK
            }

            Action.INTAKE_FINISHED -> {
                //Unlock from intaking
                lockedByIntake = false
            }

            Action.VISION_STARTED -> {
                //Lock for vision
                lockedByVision = true
            }

            Action.VISION_FINISHED -> {
                //Unlock from vision
                lockedByVision = false
            }

            Action.AUTO_STARTED -> {
                //Lock for auto
                //lockedByAuto = true
            }

            Action.AUTO_FINISHED -> {
                //Unlock from auto
                lockedByAuto = false
            }
        }
    }

    private val driveThreshold = 0.1
    private val driveTimeSeconds = 0.1
    private val driveBackRocketCycleThresholdInches = 219.813 //Measured from field CAD (loading station to rocket edge)

    /**
     * Reports data from the drive stick
     */
    @Synchronized
    fun reportDriveCommand(timestamp: Double, driveThrottle: Double, driveXInches: Double) {
        if (lastDriveReportTimestamp == 0.0) {
            lastDriveReportTimestamp = timestamp
            return
        }

        val dt = timestamp - lastDriveReportTimestamp

        when {
            driveThrottle >= driveThreshold -> {
                //Moving forwards over threshold, accumulate forwards, reset reverse
                driveAccumForwards += dt
                driveAccumReverse = 0.0
            }
            driveThrottle <= -driveThreshold -> {
                //Moving reverse over threshold, accumulate reverse, reset forwards
                driveAccumReverse += dt
                driveAccumForwards = 0.0
            }
            else -> {
                //Not moving in either threshold, reset both accumulators
                driveAccumForwards = 0.0
                driveAccumReverse = 0.0
            }
        }

        val driveSelectedSide: SuperstructureRoutines.Side? = when {
            driveAccumForwards >= driveTimeSeconds -> {
                //Moved forwards over time threshold, forward
                SuperstructureRoutines.Side.FRONT
            }
            driveAccumReverse >= driveTimeSeconds -> {
                //Moved reverse over time threshold, reverse
                SuperstructureRoutines.Side.BACK
            }
            else -> null //No selected side, meaning we just use whatever side we did before
        }

        if (SuperstructureController.output.wristTool == WristMotionPlanner.Tool.HatchPanelTool && !isLocked() && driveSelectedSide != null) {
            //If we're not locked and there was a side selected by the drive, update the current side
            currentSide = if (driveXInches >= driveBackRocketCycleThresholdInches) {
                //We're past the back rocket cycle position threshold, so suggest front side only
                SuperstructureRoutines.Side.FRONT
            } else {
                //We're not past the back cycle threshold, so suggest the drive selected side
                driveSelectedSide
            }
        }
    }
}