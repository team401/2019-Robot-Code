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
        override val leftKa = 0.002
        override val leftKs = 0.0065
        override val leftKv = 0.0133
        override val momentOfInertia = 1.0
        override val rightKa = 0.0037
        override val rightKs = 0.0051
        override val rightKv = 0.0129
        override val trackScrubFactor = 1.0181122803726486
    }
}