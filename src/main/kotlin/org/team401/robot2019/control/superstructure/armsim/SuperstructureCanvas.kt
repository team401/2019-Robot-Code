package org.team401.robot2019.control.superstructure.armsim

import org.snakeskin.measure.Inches
import org.snakeskin.measure.Radians
import org.snakeskin.measure.RadiansPerSecond
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.control.superstructure.geometry.WristState
import org.team401.robot2019.subsystems.arm.control.ArmKinematics
import java.awt.Canvas
import java.awt.Color
import java.awt.Graphics
import javax.swing.JFrame
import javax.swing.SwingUtilities
import kotlin.math.roundToInt

/**
 * @author Cameron Earle
 * @version 2/19/2019
 *
 */
class SuperstructureCanvas(val ppi: Double): Canvas() {
    private var armState = ArmState(0.0.Inches, 0.0.Radians, 0.0.RadiansPerSecond)
    private var wristState = WristState(0.0.Radians, false, false)

    private fun getOriginX(): Int {
        return size.width / 2
    }

    private fun getOriginY(): Int {
        return size.height / 2
    }

    fun update(newArmState: ArmState, newWristState: WristState) {
        armState = newArmState
        wristState = newWristState
    }

    private fun xToFrame(x: LinearDistanceMeasureInches): Int {
        return (getOriginX() + x.value).roundToInt()
    }

    private fun yToFrame(y: LinearDistanceMeasureInches): Int {
        return (getOriginY() - y.value).roundToInt()
    }

    private fun drawWcs(g: Graphics) {
        g.color = Color.gray //Set to gray color
        g.drawLine(getOriginX(), 0, getOriginX(), size.height) //Draw y axis
        g.drawLine(0, getOriginY(), size.width, getOriginY()) //Draw y axis
    }

    private fun drawArm(g: Graphics) {
        //Get pose of arm endpoint
        val armEndpoint = ArmKinematics.forward(armState)
        val endpointX = xToFrame(armEndpoint.x)
        val endpointY = yToFrame(armEndpoint.y)

        g.drawLine(getOriginX(), getOriginY(), endpointX, endpointY)
    }

    override fun paint(g: Graphics) {
        drawWcs(g)
        drawArm(g)
    }
}