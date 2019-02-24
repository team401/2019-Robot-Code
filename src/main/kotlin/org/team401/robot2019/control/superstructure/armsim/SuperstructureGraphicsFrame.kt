package org.team401.robot2019.control.superstructure.armsim

import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.control.superstructure.geometry.WristState
import java.awt.BorderLayout
import java.awt.Color
import java.awt.Dimension
import java.awt.EventQueue
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledFuture
import java.util.concurrent.TimeUnit
import javax.swing.*
import javax.swing.border.BevelBorder
import kotlin.math.roundToLong

/**
 * @author Cameron Earle
 * @version 2/23/2019
 *
 */
class SuperstructureGraphicsFrame(ppi: Double, dt: Double, fps: Double, val data: List<ArmSim.SimFrame>): JFrame("Arm Simulation") {
    private val executor = Executors.newSingleThreadScheduledExecutor()
    private var currentFuture: ScheduledFuture<*>? = null

    private val sPerFrame = (1.0 / fps)

    private val playButton = JButton("▶")
    private val pauseButton = JButton("❚❚")
    private val stepRevButton = JButton("◀❚")
    private val stepFwdButton = JButton("❚▶")
    private val resetButton = JButton("Reset")
    private val speedSpinner = JSpinner(SpinnerNumberModel(1.0, 0.0, Double.POSITIVE_INFINITY, 0.1))

    private val canvas = SuperstructureCanvas(ppi)

    private var activeTime = 0.0

    fun getPlaybackSpeed(): Double {
        return speedSpinner.value as Double
    }

    fun reset() {
        activeTime = 0.0
        pause()
        draw()
    }

    fun draw() {
        val point = data.first { it.time >= activeTime }
        val armState = ArmState(point.command.armRadius, point.command.armAngle, point.command.armVelocity)
        val wristState = WristState(point.command.wristTheta, false, false)
        canvas.update(armState, wristState)
        canvas.repaint()
        validate()
    }

    fun advance(time: Double) {
        if (activeTime < data.last().time) {
            activeTime += time
        } else {
            pause()
        }
    }

    fun decrement(time: Double) {
        if (activeTime > 0.0) {
            activeTime -= time
        }
    }

    @Synchronized fun play() {
        if (currentFuture == null) {
            currentFuture = executor.scheduleAtFixedRate({
                draw()
                advance(sPerFrame * getPlaybackSpeed())
                println("update")
            }, 0L, (sPerFrame * 1000.0).roundToLong(), TimeUnit.MILLISECONDS)
        }
    }

    @Synchronized fun pause() {
        currentFuture?.cancel(false)
        currentFuture = null
    }

    init {
        canvas.setSize(400, 400)
        val canvasPane = JPanel(BorderLayout())
        canvasPane.border = BorderFactory.createLineBorder(Color.black)
        canvasPane.add(canvas, BorderLayout.CENTER)
        add(canvasPane, BorderLayout.CENTER)

        val lowerPane = JPanel()
        lowerPane.border = BorderFactory.createLineBorder(Color.black)
        lowerPane.add(stepRevButton)
        lowerPane.add(playButton)
        lowerPane.add(pauseButton)
        lowerPane.add(stepFwdButton)
        lowerPane.add(resetButton)

        (speedSpinner.editor as JSpinner.NumberEditor).textField.columns = 4

        val speedPane = JPanel()
        speedPane.add(JLabel("Playback Speed:"))
        speedPane.add(speedSpinner)

        lowerPane.add(speedPane)

        add(lowerPane, BorderLayout.PAGE_END)

        playButton.addActionListener { play() }
        pauseButton.addActionListener { pause() }
        resetButton.addActionListener { reset() }
        stepFwdButton.addActionListener { advance(dt); draw() }
        stepRevButton.addActionListener { decrement(dt); draw() }

        reset()
    }
}