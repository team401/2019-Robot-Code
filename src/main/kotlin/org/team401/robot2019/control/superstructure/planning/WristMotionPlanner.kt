package org.team401.robot2019.control.superstructure.planning

import org.jetbrains.annotations.TestOnly
import org.snakeskin.measure.*
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRadiansPerMillisecond
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.control.superstructure.geometry.Point2d
import org.team401.robot2019.control.superstructure.geometry.PointPolar
import org.team401.robot2019.control.superstructure.geometry.WristState
import org.team401.robot2019.subsystems.arm.control.ArmKinematics

object WristMotionPlanner {
    private val PI_2 = (Math.PI / 2.0).Radians
    private val NPI_2 = (-1.0 * (Math.PI / 2.0)).Radians
    val POSITIVE_X_OFFSET = 0.0.Radians
    val NEGATIVE_X_OFFSET = (Math.PI).Radians

    //Tools on the wrist
    enum class Tool(val angularOffset: AngularDistanceMeasureRadians, val minimumRadius: LinearDistanceMeasureInches) {
        CargoTool((-Math.PI).Radians, Geometry.ArmGeometry.cargoToolMinSafeLength),
        HatchPanelTool((0.0).Radians, Geometry.ArmGeometry.hatchPanelToolMinSafeLength)
    }

    //Control modes for the wrist
    enum class ControlMode {
        MaintainAngle,
        AngleCommand
    }

    private var commandedMode = ControlMode.MaintainAngle
    private var commandedAngle = 0.0.Radians
    private var commandedTool = Tool.HatchPanelTool
    private var commandedArmState = PointPolar(0.0.Inches, 0.0.Radians)
    private var commandedArmStateRectangular = Point2d(0.0.Inches, 0.0.Inches)

    /**
     * Configures the wrist planner to keep the wrist at a set angle from the floor,
     * or snap to safe angles during periods where a gamepiece would be crushed between the wrist and the arm
     */
    fun setToMaintainAngleMode(angle: AngularDistanceMeasureRadians, tool: Tool, finalState: Point2d) {
        commandedMode = ControlMode.MaintainAngle
        commandedAngle = angle
        commandedTool = tool
        commandedArmStateRectangular = finalState
        commandedArmState = ArmKinematics.inverse(finalState)
    }

    /**
     * Configures the wrist planner to keep the wrist parallel to the floor
     */
    fun setToParallelMode(tool: Tool, finalState: Point2d) {
        setToMaintainAngleMode(0.0.Radians, tool, finalState)
    }

    /**
     * Configures the wrist planner to target a specific "floor relative" angle.  This requires
     * the desired tool to reference, as well as the floor angle.
     *
     * This configuration depends on the side of the coordinate system that the arm is on, and thus
     * should only be called once the arm is at a desired endpoint that is clearly on one side
     */
    fun setToAngleMode(tool: Tool, angleFromFloor: AngularDistanceMeasureRadians, finalState: Point2d) {
        commandedMode = ControlMode.AngleCommand
        commandedAngle = angleFromFloor
        commandedTool = tool
        commandedArmStateRectangular = finalState
        commandedArmState = ArmKinematics.inverse(finalState)
    }

    /**
     * Calculates the absolute angle given the angle from the floor, tool, and side offset
     */
    fun calculateFloorAngle(armAngle: AngularDistanceMeasureRadians,
                                    wristCommmandAngle: AngularDistanceMeasureRadians,
                                    toolOffset: AngularDistanceMeasureRadians,
                                    sideOffset: AngularDistanceMeasureRadians
                                    ): AngularDistanceMeasureRadians {
        //Equation: talon angle = (pi/2) - arm angle + desired angle + tool modifier (180) + side modifier (180)
        return (PI_2 - armAngle + wristCommmandAngle + toolOffset + sideOffset)
    }

    fun update(armState: ArmState, wristState: WristState): WristState {
        //The talon's "zero" should be the cargo tool pointed straight down when the arm is at 0
        //This means it can be rotated 180 degrees to each side

        val finalCommandAngle = when (commandedMode) {
            ControlMode.AngleCommand -> {
                //In direct angular command, we use the final arm setpoint angle to drive the calculation
                val sideOffset = if (commandedArmStateRectangular.x >= 0.0.Inches) {
                    POSITIVE_X_OFFSET
                } else {
                    NEGATIVE_X_OFFSET
                }

                val commandMultiplier = if (sideOffset == POSITIVE_X_OFFSET) {
                    1.0.Unitless
                } else {
                    (-1.0).Unitless
                }

                calculateFloorAngle(
                    commandedArmState.theta,
                    commandMultiplier * commandedAngle,
                    commandedTool.angularOffset,
                    sideOffset
                )
            }

            ControlMode.MaintainAngle -> {
                val currentArmRectangular = ArmKinematics.forward(armState)
                val sideOffset = when {
                    currentArmRectangular.x >= Geometry.ArmGeometry.wristParallelCollisionAngle -> POSITIVE_X_OFFSET
                    currentArmRectangular.x <= (-1.0).Unitless * Geometry.ArmGeometry.wristParallelCollisionAngle -> NEGATIVE_X_OFFSET
                    else -> {
                        //In this case, we want to identify the direction of movement, and pick a flip this way
                        if (currentArmRectangular.x >= commandedArmStateRectangular.x) {
                            //We're moving negative ("left"), so we want to switch tool offset now to the positive one
                            NEGATIVE_X_OFFSET
                        } else {
                            //We're moving positive ("right"), so we want to switch tool offset now to the negative one
                            POSITIVE_X_OFFSET
                        }
                    }
                }

                val commandMultiplier = if (sideOffset == POSITIVE_X_OFFSET) {
                    1.0.Unitless
                } else {
                    (-1.0).Unitless
                }

                calculateFloorAngle(
                    armState.armAngle,
                    commandMultiplier * commandedAngle,
                    commandedTool.angularOffset,
                    sideOffset
                )
            }
        }

        //println("Current: ${wristState.wristPosition.toDegrees()}  Desired ${finalCommandAngle.toDegrees()}")

        return WristState(finalCommandAngle, wristState.hasCargo, wristState.hasHatchPanel)
    }
}