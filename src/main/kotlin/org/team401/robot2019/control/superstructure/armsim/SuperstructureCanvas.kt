package org.team401.robot2019.control.superstructure.armsim

import org.snakeskin.measure.*
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
    companion object {
        private val cargoToolLength = 14.0.Inches
        private val hatchToolLength = 12.0.Inches
        private val upIndicatorLength = 3.0.Inches

        private val originToFloor = (-23.0).Inches
        private val originToFrame = 14.5.Inches
    }

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
     * Draws the coordinate grid on the graphics canvas, as well as the floor, frame perimeter, and extension limits
     */
    private fun drawWcs(g: Graphics2D) {
        g.stroke = BasicStroke(1.0f)
        g.color = Color.gray //Set to gray color
        g.drawLine(getOriginX(), 0, getOriginX(), size.height) //Draw y axis
        g.drawLine(0, getOriginY(), size.width, getOriginY()) //Draw x axis

        g.stroke = BasicStroke(2.0f)
        g.color = Color.cyan

        val floorY = yToFrame(originToFloor)
        g.drawLine(0, floorY, size.width, floorY)

        val frameXRight = xToFrame(originToFrame)
        val frameXLeft = xToFrame((-1.0).Unitless * originToFrame)

        g.drawLine(frameXLeft, 0, frameXLeft, size.height)
        g.drawLine(frameXRight, 0, frameXRight, size.height)

        g.color = Color.red

        val limitXRight = xToFrame(30.0.Inches + originToFrame)
        val limitXLeft = xToFrame((-1.0).Unitless * (30.0.Inches + originToFrame))

        g.drawLine(limitXLeft, 0, limitXLeft, size.height)
        g.drawLine(limitXRight, 0, limitXRight, size.height)
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

    /**
     * Draws the wrist on the grid of the graphics canvas
     */
    private fun drawWrist(g: Graphics2D) {
        val armEndpoint = ArmKinematics.forward(armState)

        val endpointX = xToFrame(armEndpoint.x)
        val endpointY = yToFrame(armEndpoint.y)

        val wristAngleAdjusted = wristState.wristPosition - (Math.PI / 2.0).Radians + armState.armAngle
        val upIndicatorAngle = wristAngleAdjusted + (Math.PI / 2.0).Radians
        val hatchX = xToFrame((armEndpoint.x.value + (hatchToolLength.value * Math.cos(wristAngleAdjusted.value))).Inches)
        val hatchY = yToFrame((armEndpoint.y.value + (hatchToolLength.value * Math.sin(wristAngleAdjusted.value))).Inches)

        val cargoX = xToFrame((armEndpoint.x.value - (cargoToolLength.value * Math.cos(wristAngleAdjusted.value))).Inches)
        val cargoY = yToFrame((armEndpoint.y.value - (cargoToolLength.value * Math.sin(wristAngleAdjusted.value))).Inches)

        val upX = xToFrame((armEndpoint.x.value + (upIndicatorLength.value * Math.cos(upIndicatorAngle.value))).Inches)
        val upY = yToFrame((armEndpoint.y.value + (upIndicatorLength.value * Math.sin(upIndicatorAngle.value))).Inches)

        g.stroke = BasicStroke((1.0 * ppi).toFloat())
        g.color = Color.yellow
        g.drawLine(endpointX, endpointY, hatchX, hatchY)

        g.stroke = BasicStroke((1.0 * ppi).toFloat())
        g.color = Color.orange
        g.drawLine(endpointX, endpointY, cargoX, cargoY)

        g.stroke = BasicStroke((1.0 * ppi).toFloat())
        g.color = Color.green
        g.drawLine(endpointX, endpointY, upX, upY)
    }

    override fun paint(g: Graphics) {
        val g2d = g as Graphics2D
        drawWcs(g2d)
        drawArm(g2d)
        drawWrist(g2d)
    }
}