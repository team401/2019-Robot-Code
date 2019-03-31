package org.team401.robot2019

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard

/**
 * @author Eli Jelesko
 * @version 3/13/2019
 *
 */

object DriverStationDisplay {
    private val mainTab = Shuffleboard.getTab("Main")

    /**
     * Displays if the arm pivot has been E stopped
     */
    val pivotStopped = mainTab.add("Pivot Stopped", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withSize(1,1)
        .withPosition(0, 1)
        .entry

    /**
     * Displays if the arm extension has been E stopped
     */
    val extensionStopped = mainTab.add("Extension Stopped", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withSize(1,1)
        .withPosition(0, 2)
        .entry
    /**
     * Displays if the wrist has been E stopped
     */
    val wristStopped = mainTab.add("Wrist Stopped", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withSize(1,1)
        .withPosition(1, 1)
        .entry

    /**
     * Displays if the drivetrain has been E stopped
     */
    val driveStopped = mainTab.add("Drive Enabled", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withSize(1,1)
        .withPosition(2, 1)
        .entry

    /**
     * Displays if the climbing system has been E stopped
     */
    val climbStopped = mainTab.add("Climbing Enabled", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withSize(1,1)
        .withPosition(3, 1)
        .entry

    /**
     * Displays if the floor pickup has been E stopped
     */
    val floorPickupStopped = mainTab.add("Floor Stopped", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withSize(1,1)
        .withPosition(4,1)
        .entry

    /**
     * Displays if the superstructure is ignoring our safety constraints
     */
    val manualOverride = mainTab.add("Manual Override", false)
        .withSize(1,1)
        .withPosition(0, 0)
        .entry

    /**
     * Displays if the robot thinks it has a game piece, either a hatch panel or cargo
     */
    val hasGamePiece = mainTab.add("Have a game piece", false)
        .withSize(1,1)
        .withPosition(1, 0)
        .entry

    /**
     * Displays if the drive is in climb align, where it is slowed down.
     */
    val climbRepositionModeEnabled = mainTab.add("ClimbAlign", false)
        .withSize(1,1)
        .withPosition(2, 0)
        .entry

    /**
     * Shows if the extension is homed
     */
    val extensionHomed = mainTab.add("Homed", false)
        .withSize(1,1)
        .withPosition(3,0)
        .entry


    fun init(){
        pivotStopped.setBoolean(false)
        wristStopped.setBoolean(false)
        driveStopped.setBoolean(false)
        climbStopped.setBoolean(false)
        floorPickupStopped.setBoolean(false)

        manualOverride.setBoolean(false)
        hasGamePiece.setBoolean(false)
        climbRepositionModeEnabled.setBoolean(false)

        extensionHomed.setBoolean(false)
    }
}