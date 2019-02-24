package org.team401.robot2019.control.superstructure.armsim

import org.snakeskin.measure.Inches
import org.snakeskin.measure.Radians
import org.snakeskin.measure.RadiansPerSecond
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.control.superstructure.geometry.Point2d
import org.team401.robot2019.control.superstructure.geometry.PointPolar
import org.team401.robot2019.control.superstructure.geometry.WristState
import org.team401.robot2019.subsystems.arm.control.ArmKinematics
import java.awt.*
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
        return (getOriginX() + x.value * ppi).roundToInt()
    }

    private fun yToFrame(y: LinearDistanceMeasureInches): Int {
        return (getOriginY() - y.value * ppi).roundToInt()
    }

    /**
     * Draws the coordinate grid on the graphics canvas
     */
    private fun drawWcs(g: Graphics2D) {
        g.stroke = BasicStroke(1.0f)
        g.color = Color.gray //Set to gray color
        g.drawLine(getOriginX(), 0, getOriginX(), size.height) //Draw y axis
        g.drawLine(0, getOriginY(), size.width, getOriginY()) //Draw y axis
    }

    /**
     * Draws the arm on the grid of the graphics canvas
     */
    private fun drawArm(g: Graphics2D) {
        //Get pose of arm endpoints
        val armBaseEndpoint = ArmKinematics.forward(PointPolar(Geometry.ArmGeometry.armBaseLength, armState.armAngle))
        val armEndEndpoint = ArmKinematics.forward(armState)

        val baseEndpointX = xToFrame(armBaseEndpoint.x)
        val baseEndpointY = yToFrame(armBaseEndpoint.y)

        val endEndpointX = xToFrame(armEndEndpoint.x)
        val endEndpointY = yToFrame(armEndEndpoint.y)

        //Draw arm extension
        g.stroke = BasicStroke((1.5 * ppi).toFloat())
        g.color = Color.red
        g.drawLine(baseEndpointX, baseEndpointY, endEndpointX, endEndpointY)

        //Draw arm base
        g.stroke = BasicStroke((2.0 * ppi).toFloat())
        g.color = Color.blue
        g.drawLine(getOriginX(), getOriginY(), baseEndpointX, baseEndpointY)

        if (armState.armRadius < Geometry.ArmGeometry.armBaseLength) {
            //There is an intersection, we should draw it now
            g.stroke = BasicStroke((1.5 * ppi).toFloat())
            g.color = Color.magenta
            g.drawLine(baseEndpointX, baseEndpointY, endEndpointX, endEndpointY)
        }
    }

    override fun paint(g: Graphics) {
        val g2d = g as Graphics2D
        drawWcs(g2d)
        drawArm(g2d)
    }
}