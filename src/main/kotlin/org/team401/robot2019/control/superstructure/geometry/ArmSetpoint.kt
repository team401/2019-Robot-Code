package org.team401.robot2019.control.superstructure.geometry

import org.snakeskin.measure.Unitless
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner

/**
 * @author Cameron Earle
 * @version 2/18/2019
 *
 */
data class ArmSetpoint(val point: Point2d,
                       val tool: WristMotionPlanner.Tool,
                       val toolAngle: AngularDistanceMeasureRadians) {
    companion object {
        fun fromFloor(point: Point2d, tool: WristMotionPlanner.Tool, toolAngle: AngularDistanceMeasureRadians): ArmSetpoint {
            return ArmSetpoint(Point2d(point.x, point.y + Geometry.ArmGeometry.floorOffset), tool, toolAngle)
        }
    }


    fun upBy(distance: LinearDistanceMeasureInches): ArmSetpoint {
        return ArmSetpoint(Point2d(point.x, point.y + distance), tool, toolAngle)
    }

    fun withAngle(angle: AngularDistanceMeasureRadians): ArmSetpoint {
        return ArmSetpoint(point, tool, angle)
    }

    fun flipped(): ArmSetpoint {
        return ArmSetpoint(Point2d(point.x * (-1.0).Unitless, point.y), tool, toolAngle)
    }
}