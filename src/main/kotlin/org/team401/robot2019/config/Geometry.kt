package org.team401.robot2019.config

import org.snakeskin.component.template.TankDrivetrainGeometryTemplate
import org.snakeskin.units.Inches
import kotlin.math.atan2

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */
object Geometry {
    /*
    object DrivetrainGeometry: TankDrivetrainGeometryTemplate {
        override val wheelRadius = 3.062954.Inches
        override val wheelbase = 25.625.Inches
    }
    */
    object ArmGeometry{
        //All values with the pivot as the origin
        val maxX = 8.0.Inches
        val maxY = 10.0.Inches
        val minY = (-2.0).Inches
        val maxExtension = 0.0.Inches
        val maxArmLength = 0.0.Inches
        val minExtension = 0.0.Inches
        val minArmLength = 4.0.Inches
        val pivotHeight = 0.0.Inches

        val maxTheta = atan2(maxY.value, maxX.value) // From -Pi to Pi

        val startExtension = 0.0.Inches // Distance from pivot to fully retracted

        val armToInches = 0.0.Inches

    }
}