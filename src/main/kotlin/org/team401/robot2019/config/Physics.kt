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
        override val inertialMass = 60.0
        override val leftKa = 1.0
        override val leftKs = 1.0
        override val leftKv = 1.0
        override val momentOfInertia = 1.0
        override val rightKa = 1.0
        override val rightKs = 1.0
        override val rightKv = 1.0
        override val trackScrubFactor = 1.0
    }
}