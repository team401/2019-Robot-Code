package org.team401.robot2019.config

import org.snakeskin.template.TankDrivetrainGeometryTemplate
import org.snakeskin.units.Inches

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */
object Geometry {
    object DrivetrainGeometry: TankDrivetrainGeometryTemplate {
        override val wheelRadius = 8.0.Inches
        override val wheelbase = 24.0.Inches
    }
}