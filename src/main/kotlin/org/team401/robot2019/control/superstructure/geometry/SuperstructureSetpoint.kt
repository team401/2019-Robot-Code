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
                                  val cargoGrabberState: WristSubsystem.CargoGrabberStates,
                                  val hatchClawState: WristSubsystem.HatchClawStates,
                                  val visionHeightMode: VisionHeightMode) {
    companion object {
        fun holdingCargo(point: Point2d, toolAngle: AngularDistanceMeasureRadians, visionHeightMode: VisionHeightMode): SuperstructureSetpoint {
            return SuperstructureSetpoint(
                point,
                WristMotionPlanner.Tool.CargoTool,
                toolAngle,
                WristSubsystem.CargoGrabberStates.Clamped,
                WristSubsystem.HatchClawStates.Clamped,
                visionHeightMode
            )
        }

        fun holdingHatch(point: Point2d, toolAngle: AngularDistanceMeasureRadians, visionHeightMode: VisionHeightMode): SuperstructureSetpoint {
            return SuperstructureSetpoint(
                point,
                WristMotionPlanner.Tool.HatchPanelTool,
                toolAngle,
                WristSubsystem.CargoGrabberStates.Unclamped,
                WristSubsystem.HatchClawStates.Clamped,
                visionHeightMode
            )
        }

        fun intakingCargo(point: Point2d, toolAngle: AngularDistanceMeasureRadians, visionHeightMode: VisionHeightMode): SuperstructureSetpoint {
            return SuperstructureSetpoint(
                point,
                WristMotionPlanner.Tool.CargoTool,
                toolAngle,
                WristSubsystem.CargoGrabberStates.Unclamped,
                WristSubsystem.HatchClawStates.Clamped,
                visionHeightMode
            )
        }

        fun intakingHatch(point: Point2d, toolAngle: AngularDistanceMeasureRadians, visionHeightMode: VisionHeightMode): SuperstructureSetpoint {
            return SuperstructureSetpoint(
                point,
                WristMotionPlanner.Tool.HatchPanelTool,
                toolAngle,
                WristSubsystem.CargoGrabberStates.Unclamped,
                WristSubsystem.HatchClawStates.Unclamped,
                visionHeightMode
            )
        }
    }

    fun fromFloor(): SuperstructureSetpoint {
        return SuperstructureSetpoint(
            Point2d(point.x, point.y + Geometry.ArmGeometry.originToFloor),
            tool,
            toolAngle,
            cargoGrabberState,
            hatchClawState,
            visionHeightMode
        )
    }

    fun upBy(distance: LinearDistanceMeasureInches): SuperstructureSetpoint {
        return SuperstructureSetpoint(
            Point2d(point.x, point.y + distance),
            tool,
            toolAngle,
            cargoGrabberState,
            hatchClawState,
            visionHeightMode
        )
    }

    fun atX(x: LinearDistanceMeasureInches): SuperstructureSetpoint {
        return SuperstructureSetpoint(Point2d(x, point.y),
            tool,
            toolAngle,
            cargoGrabberState,
            hatchClawState,
            visionHeightMode
        )
    }

    fun withAngle(angle: AngularDistanceMeasureRadians): SuperstructureSetpoint {
        return SuperstructureSetpoint(
            point,
            tool,
            angle,
            cargoGrabberState,
            hatchClawState,
            visionHeightMode
        )
    }

    fun flipped(): SuperstructureSetpoint {
        return SuperstructureSetpoint(Point2d(point.x * (-1.0).Unitless, point.y),
            tool,
            toolAngle,
            cargoGrabberState,
            hatchClawState,
            visionHeightMode
        )
    }

    fun withHeightMode(newHeightMode: VisionHeightMode): SuperstructureSetpoint {
        return SuperstructureSetpoint(
            point,
            tool,
            toolAngle,
            cargoGrabberState,
            hatchClawState,
            newHeightMode
        )
    }
}