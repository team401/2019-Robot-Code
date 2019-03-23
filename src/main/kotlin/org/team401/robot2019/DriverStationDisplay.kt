package org.team401.robot2019

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.WidgetType

/**
 * @author Eli Jelesko
 * @version 3/13/2019
 *
 */

object DriverStationDisplay {
    private val mainTab = Shuffleboard.getTab("Main")

    /**
     * Displays if the arm has been E stopped
     */
    val armStopped = mainTab.add("Arm Enabled", true)
        .withSize(1,1)
        .withPosition(6, 1)
        .entry

    /**
     * Displays if the wrist has been E stopped
     */
    val wristStopped = mainTab.add("Wrist Enabled", true)
        .withSize(1,1)
        .withPosition(7, 1)
        .entry

    /**
     * Displays if the superstructure is ignoring our safety constraints
     */
    val manualOverride = mainTab.add("Manual Override", false)
        .withSize(1,1)
        .withPosition(6, 2)
        .entry

    /**
     * Displays if the robot thinks it has a game piece, either a hatch panel or cargo
     */
    val hasGamePiece = mainTab.add("Have a game piece", false)
        .withSize(1,1)
        .withPosition(6, 3)
        .entry

    /**
     * Displays if the drive is in climb align, where it is slowed down.
     */
    val climbRepositionModeEnabled = mainTab.add("ClimbAlign", false)
        .withSize(1,1)
        .withPosition(7, 3)
        .entry

}