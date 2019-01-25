package org.team401.robot2019.config

import org.team401.taxis.template.DriveDynamicsTemplate

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */
object Physics {
    object DrivetrainDynamics: DriveDynamicsTemplate {
        override val angularDrag = 1.0
        override val inertialMass = 24.948
        override val leftKa = arrayOf(0.173, 0.0177, 0.0197).average()
        override val leftKs = arrayOf(0.5689, 0.4616, 0.4405).average()
        override val leftKv = arrayOf(0.1630, 0.1650, 0.1645).average()
        override val momentOfInertia = 2.0
        override val rightKa = arrayOf(0.0210, 0.0237, 0.0237).average()
        override val rightKs = arrayOf(0.4749, 0.3437, 0.3313).average()
        override val rightKv = arrayOf(0.1719, 0.1756, 0.1763).average()
        override val trackScrubFactor = arrayOf(1.12995, 1.12880, 1.12774).average()
    }
}