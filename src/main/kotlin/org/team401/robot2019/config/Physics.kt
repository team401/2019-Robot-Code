package org.team401.robot2019.config

import org.snakeskin.utility.Selectable
import org.team401.taxis.template.DriveDynamicsTemplate

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */
object Physics {
    object DrivetrainDynamics: DriveDynamicsTemplate {
        override val angularDrag by Selectable(1.0, 1.0)
        override val inertialMass by Selectable(66.95, 51.6868506)
        override val leftKa by Selectable(0.0142, 0.0192)
        override val leftKs by Selectable(0.1955, 0.1524)
        override val leftKv by Selectable(0.1421, 0.1466)
        override val momentOfInertia by Selectable(1.3, 1.331905811)
        override val rightKa by Selectable(0.0212, 0.0230)
        override val rightKs by Selectable(0.1919, 0.1396)
        override val rightKv by Selectable(0.1406, 0.1462)
        override val trackScrubFactor by Selectable(1.009268402942991, 1.00567713365549)
    }
}