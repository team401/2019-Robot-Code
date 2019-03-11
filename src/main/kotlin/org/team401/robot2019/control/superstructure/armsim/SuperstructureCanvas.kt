package org.team401.robot2019.control.superstructure.armsim

import org.snakeskin.measure.*
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.control.superstructure.geometry.*
import org.team401.robot2019.subsystems.WristSubsystem
import org.team401.robot2019.subsystems.arm.control.ArmKinematics
import java.awt.*
import java.awt.geom.AffineTransform
import java.awt.image.BufferedImage
import kotlin.math.roundToInt

/**
 * @author Cameron Earle
 * @version 2/19/2019
 *
 */
class SuperstructureCanvas(val ppi: Double, val cargoToolLength: LinearDistanceMeasureInches, val hatchToolLength: LinearDistanceMeasureInches, sizeIn: Dimension): Canvas() {
    companion object {
        private val upIndicatorLength = 3.0.Inches
        private val cargoPortDiameter = 16.0.Inches
        private val cargoPortOffset = 28.0.Inches
        private val cargoPortHeight = 27.5.Inches - (cargoPortDiameter / 2.0.Unitless)

    }

    private fun dist(a: Point2d, b: Point2d): LinearDistanceMeasureInches {
        val dx2 = ((b.x - a.x) * (b.x - a.x)).value
        val dy2 = ((b.y - a.y) * (b.y - a.y)).value
        return Math.sqrt(dx2 + dy2).Inches
    }

    private var armState = ArmState(0.0.Inches, 0.0.Radians, 0.0.RadiansPerSecond)
    private var wristState = WristState(0.0.Radians, false, false)

    private lateinit var staticComponents: BufferedImage

    private var oldSize = size

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
     * Draws a horizontal line at the given height
     */
    private fun drawHorizontal(y: LinearDistanceMeasureInches, g: Graphics2D) {
        val yPix = yToFrame(y)
        g.drawLine(0, yPix, size.width, yPix)
    }

    /**
     * Draws the coordinate grid on the graphics canvas, as well as the floor, frame perimeter, and extension limits
     */
    private fun drawWcs(g: Graphics2D) {
        g.stroke = BasicStroke(1.0f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER)
        g.color = Color.gray //Set to gray color
        g.drawLine(getOriginX(), 0, getOriginX(), size.height) //Draw y axis
        g.drawLine(0, getOriginY(), size.width, getOriginY()) //Draw x axis

        g.stroke = BasicStroke(2.0f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER)
        g.color = Color.cyan

        val floorY = yToFrame(Geometry.ArmGeometry.originToFloor)
        g.drawLine(0, floorY, size.width, floorY)

        val frameXRight = xToFrame(Geometry.ArmGeometry.originToFrame)
        val frameXLeft = xToFrame((-1.0).Unitless * Geometry.ArmGeometry.originToFrame)

        g.drawLine(frameXLeft, 0, frameXLeft, size.height)
        g.drawLine(frameXRight, 0, frameXRight, size.height)

        g.color = Color.red

        val limitXRight = xToFrame(30.0.Inches + Geometry.ArmGeometry.originToFrame)
        val limitXLeft = xToFrame((-1.0).Unitless * (30.0.Inches + Geometry.ArmGeometry.originToFrame))

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
        g.stroke = BasicStroke((1.5 * ppi).toFloat(), BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER)
        g.color = Color.red
        g.drawLine(baseEndpointX, baseEndpointY, endEndpointX, endEndpointY)

        //Draw arm base
        g.stroke = BasicStroke((2.0 * ppi).toFloat(), BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER)
        g.color = Color.blue
        g.drawLine(getOriginX(), getOriginY(), baseEndpointX, baseEndpointY)

        if (armState.armRadius < Geometry.ArmGeometry.armBaseLength) {
            //There is an intersection, we should draw it now
            g.stroke = BasicStroke((1.5 * ppi).toFloat(), BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER)
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

        val wristPose = EndpointKinematics.forward(armState, wristState, WristSubsystem.CargoGrabberStates.Clamped, WristSubsystem.HatchClawStates.Clamped)
        val hatchX = xToFrame(wristPose.hatchEndpoint.x)
        val hatchY = yToFrame(wristPose.hatchEndpoint.y)
        val hatchAX = xToFrame(wristPose.hatchEndpointA.x)
        val hatchAY = yToFrame(wristPose.hatchEndpointA.y)
        val hatchBX = xToFrame(wristPose.hatchEndpointB.x)
        val hatchBY = yToFrame(wristPose.hatchEndpointB.y)
        val cargoX = xToFrame(wristPose.cargoEndpoint.x)
        val cargoY = yToFrame(wristPose.cargoEndpoint.y)
        val backstopX = xToFrame(wristPose.backstopEndpoint.x)
        val backstopY = yToFrame(wristPose.backstopEndpoint.y)

        val upX = xToFrame((armEndpoint.x.value - (upIndicatorLength.value * Math.cos(upIndicatorAngle.value))).Inches)
        val upY = yToFrame((armEndpoint.y.value - (upIndicatorLength.value * Math.sin(upIndicatorAngle.value))).Inches)

        g.stroke = BasicStroke((1.0 * ppi).toFloat(), BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER)
        g.color = Color.yellow
        g.drawLine(endpointX, endpointY, hatchX, hatchY)
        g.color = Color.white
        g.drawLine(hatchX, hatchY, hatchAX, hatchAY)
        g.drawLine(hatchX, hatchY, hatchBX, hatchBY)

        g.stroke = BasicStroke((1.0 * ppi).toFloat(), BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER)
        g.color = Color.orange
        g.drawLine(endpointX, endpointY, cargoX, cargoY)
        g.color = Color.white
        g.drawLine(endpointX, endpointY, backstopX, backstopY)

        g.stroke = BasicStroke((1.0 * ppi).toFloat(), BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER)
        g.color = Color.green
        g.drawLine(endpointX, endpointY, upX, upY)
    }

    /**
     * Draws lines across the entire screen showing the cargo ports
     */
    private fun drawCargoShipLines(g: Graphics2D) {
        g.stroke = BasicStroke((1.0 * ppi).toFloat(), BasicStroke.CAP_SQUARE, BasicStroke.JOIN_MITER, 10.0f, FloatArray(1) {10.0f}, 0.0f)
        g.color = Color.black

        //First cargo port
        val firstCargoPortBottomHeight = Geometry.ArmGeometry.originToFloor + cargoPortHeight
        val firstCargoPortTopHeight = firstCargoPortBottomHeight + cargoPortDiameter

        val secondCargoPortBottomHeight = firstCargoPortBottomHeight + cargoPortOffset
        val secondCargoPortTopHeight = secondCargoPortBottomHeight + cargoPortDiameter
        
        val thirdCargoPortBottomHeight = secondCargoPortBottomHeight + cargoPortOffset
        val thirdCargoPortTopHeight = thirdCargoPortBottomHeight + cargoPortDiameter

        g.color = Color.green
        drawHorizontal(firstCargoPortBottomHeight, g)
        drawHorizontal(firstCargoPortTopHeight, g)

        g.color = Color.yellow
        drawHorizontal(secondCargoPortBottomHeight, g)
        drawHorizontal(secondCargoPortTopHeight, g)

        g.color = Color.red
        drawHorizontal(thirdCargoPortBottomHeight, g)
        drawHorizontal(thirdCargoPortTopHeight, g)
    }

    private val transform = AffineTransform()

    private fun renderImage() {
        staticComponents = BufferedImage(width, height, BufferedImage.TYPE_INT_RGB)
        val g = staticComponents.createGraphics()
        g.color = Color.lightGray
        g.fill(Rectangle(0, 0, width, height))
        drawWcs(g)
        drawCargoShipLines(g)
    }

    override fun paint(g: Graphics) {
        if (size != oldSize) {
            renderImage() //re-render
            oldSize = size
        }

        val g2d = g as Graphics2D
        g.drawRenderedImage(staticComponents, transform)
        drawArm(g2d)
        drawWrist(g2d)
    }

    init {
        size = sizeIn
        renderImage()

    }
}