package org.team401.robot2019.auto

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser

enum class AutoMode(val prettyName: String, val side: AutoSide) {
    Manual("Manual", AutoSide.Left),
    TwoHatchLeft("TwoHatchLeft", AutoSide.Left),
    TwoHatchRight("TwoHatchRight", AutoSide.Right);

    companion object {
        fun toSendableChooser(): SendableChooser<AutoMode> {
            val chooser = SendableChooser<AutoMode>()
            values().forEachIndexed {
                    index, target ->
                if (index == 0) {
                    chooser.addDefault(target.prettyName, target)
                } else {
                    chooser.addObject(target.prettyName, target)
                }
            }
            return chooser
        }

    }
}