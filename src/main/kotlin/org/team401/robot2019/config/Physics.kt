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
        override val inertialMass by Selectable(69.172836, 51.6868506)
        override val leftKa by Selectable(0.002, 0.0188)
        override val leftKs by Selectable(0.0065, 0.1539)
        override val leftKv by Selectable(0.0133, 0.1480)
        override val momentOfInertia by Selectable(2.0, 1.0)
        override val rightKa by Selectable(0.0037, 0.0204)
        override val rightKs by Selectable(0.0051, 0.1370)
        override val rightKv by Selectable(0.0129, 0.1473)
        override val trackScrubFactor by Selectable(1.0181122803726486, 1.00567713365549)
    }
}