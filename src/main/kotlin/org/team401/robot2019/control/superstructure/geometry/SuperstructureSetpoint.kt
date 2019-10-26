package org.team401.robot2019.control.superstructure.geometry

import org.snakeskin.measure.Unitless
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner
import org.team401.robot2019.subsystems.WristSubsystem

/**
 * @author Cameron Earle
 * @version 2/18/2019
 *
 */
data class SuperstructureSetpoint(val point: Point2d,
                                  val tool: WristMotionPlanner.Tool,
                                  val toolAngle: AngularDistanceMeasureRadians,
                                  val wristToolState: WristSubsystem.WristToolStates,
                                  val visionHeightMode: VisionHeightMode) {
    companion object {
        fun holdingCargo(point: Point2d, toolAngle: AngularDistanceMeasureRadians, visionHeightMode: VisionHeightMode): SuperstructureSetpoint {
            return SuperstructureSetpoint(
                point,
                WristMotionPlanner.Tool.CargoTool,
                toolAngle,
                WristSubsystem.WristToolStates.Cargo,
                visionHeightMode
            )
        }

        fun holdingHatch(point: Point2d, toolAngle: AngularDistanceMeasureRadians, visionHeightMode: VisionHeightMode): SuperstructureSetpoint {
            return SuperstructureSetpoint(
                point,
                WristMotionPlanner.Tool.HatchPanelTool,
                toolAngle,
                WristSubsystem.WristToolStates.Hatch,
                visionHeightMode
            )
        }

        fun intakingCargo(point: Point2d, toolAngle: AngularDistanceMeasureRadians, visionHeightMode: VisionHeightMode): SuperstructureSetpoint {
            return SuperstructureSetpoint(
                point,
                WristMotionPlanner.Tool.CargoTool,
                toolAngle,
                WristSubsystem.WristToolStates.Cargo,
                visionHeightMode
            )
        }

        fun intakingHatch(point: Point2d, toolAngle: AngularDistanceMeasureRadians, visionHeightMode: VisionHeightMode): SuperstructureSetpoint {
            return SuperstructureSetpoint(
                point,
                WristMotionPlanner.Tool.HatchPanelTool,
                toolAngle,
                WristSubsystem.WristToolStates.Hatch,
                visionHeightMode
            )
        }
    }

    fun fromFloor(): SuperstructureSetpoint {
        return SuperstructureSetpoint(
            Point2d(point.x, point.y + Geometry.ArmGeometry.originToFloor),
            tool,
            toolAngle,
            wristToolState,
            visionHeightMode
        )
    }

    fun upBy(distance: LinearDistanceMeasureInches): SuperstructureSetpoint {
        return SuperstructureSetpoint(
            Point2d(point.x, point.y + distance),
            tool,
            toolAngle,
            wristToolState,
            visionHeightMode
        )
    }

    fun atX(x: LinearDistanceMeasureInches): SuperstructureSetpoint {
        return SuperstructureSetpoint(Point2d(x, point.y),
            tool,
            toolAngle,
            wristToolState,
            visionHeightMode
        )
    }

    fun withAngle(angle: AngularDistanceMeasureRadians): SuperstructureSetpoint {
        return SuperstructureSetpoint(
            point,
            tool,
            angle,
            wristToolState,
            visionHeightMode
        )
    }

    fun flipped(): SuperstructureSetpoint {
        return SuperstructureSetpoint(Point2d(point.x * (-1.0).Unitless, point.y),
            tool,
            toolAngle,
            wristToolState,
            visionHeightMode
        )
    }

    fun withHeightMode(newHeightMode: VisionHeightMode): SuperstructureSetpoint {
        return SuperstructureSetpoint(
            point,
            tool,
            toolAngle,
            wristToolState,
            newHeightMode
        )
    }
}