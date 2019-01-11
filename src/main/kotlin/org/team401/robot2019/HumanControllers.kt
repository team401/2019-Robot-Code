package org.team401.robot2019

import org.snakeskin.dsl.HumanControls
import org.team401.robot2019.subsystems.PrototypeArm

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */
val LeftStick = HumanControls.t16000m(0) {
    invertAxis(Axes.PITCH)
}

val RightStick = HumanControls.t16000m(1) {

}
val Gamepad = HumanControls.f310(2){
    whenButton(Buttons.B){
        PrototypeArm.armMachine.setState(PrototypeArm.ArmStates.E_STOPPED)
    }
}
