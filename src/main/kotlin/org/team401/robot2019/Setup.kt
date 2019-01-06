package org.team401.robot2019

import org.snakeskin.controls.ControlPoller
import org.snakeskin.dsl.*

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */
@Setup
fun setup() {
    ControlPoller.pollInAutonomous = true
}