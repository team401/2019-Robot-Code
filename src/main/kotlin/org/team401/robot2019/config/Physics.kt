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
        override val angularDrag = 12.0
        override val inertialMass = 67.154
        override val leftKa = 0.0184
        override val leftKs = 0.1574
        override val leftKv = 0.1440
        override val momentOfInertia = 1.3
        override val rightKa = 0.0218
        override val rightKs = 0.1843
        override val rightKv = 0.1411
        override val trackScrubFactor = 1.009268402942991
    }
}