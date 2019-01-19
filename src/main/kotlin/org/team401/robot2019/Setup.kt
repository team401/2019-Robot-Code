package org.team401.robot2019

import org.snakeskin.controls.ControlPoller
import org.snakeskin.dsl.*
import org.snakeskin.registry.Controllers
import org.snakeskin.units.MagEncoderTicks
import org.snakeskin.units.MagEncoderTicksPer100Ms
import org.snakeskin.units.Radians
import org.snakeskin.units.RadiansPerSecond
import org.team401.robot2019.subsystems.*

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */
@Setup
fun setup() {
    ControlPoller.pollInAutonomous = true

    Subsystems.add(PrototypeArm)
    Controllers.add(Gamepad)
}

fun main(args: Array<String>) {
    val accel = 10.0
    val cruise = 100.0
    val target = -5000.0 //pos unit
    val dt = .01
    var vel = 0.0
    var lastPos = 0.0
    var pos = 0.0
    var t = 0.0

    var flatSeconds = 0.0

    val halfway = target / 2.0

    while (Math.abs(pos - target) > 0) { //Run until we're at the target
        if (halfway - pos > 0) { //The profile is less than halfway through
            vel = Math.min(cruise, vel + (accel * dt))
            if (vel == cruise) {
                flatSeconds += dt //If we're at cruise, begin accumulating the number of seconds we are there
            }
        } else { //The profile is more than halfway through
            flatSeconds -= dt //Begin decrementing seconds from flatSeconds.  When this is <= 0, begin decelerating
            if (flatSeconds <= 0) {
                vel = Math.max(0.0, vel - (accel * dt))
            }
        }
        pos = Math.min(target, lastPos + (vel * dt))
        if (vel == 0.0 && pos != target) { //We aren't quite hitting our setpoint, just jump to it.
            pos = target
        }
        t += dt
        println("$t\t$vel\t$pos")
        lastPos = pos
    }
}