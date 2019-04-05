package org.team401.robot2019.control.superstructure.armsim

import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.control.superstructure.geometry.WristState
import java.awt.BorderLayout
import java.awt.Color
import java.awt.Dimension
import java.awt.KeyboardFocusManager
import java.awt.event.KeyEvent
import java.text.DecimalFormat
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledFuture
import java.util.concurrent.TimeUnit
import javax.swing.*
import kotlin.math.roundToLong


/**
 * @author Cameron Earle
 * @version 2/23/2019
 *
 */
class SuperstructureGraphicsFrame(ppi: Double,
                                  val dt: Double,
                                  fps: Double,
                                  val data: List<ArmSim.SimFrame>,
                                  cargoToolLength: LinearDistanceMeasureInches,
                                  hatchToolLength: LinearDistanceMeasureInches
): JFrame("Arm Simulation") {
    private val executor = Executors.newSingleThreadScheduledExecutor()
    private var currentFuture: ScheduledFuture<*>? = null

    private val sPerFrame = (1.0 / fps)

    private val playButton = JButton("▶")
    private val pauseButton = JButton("❚❚")
    private val stepRevButton = JButton("◀❚")
    private val stepFwdButton = JButton("❚▶")
    private val resetButton = JButton("Reset")
    private val speedSpinner = JSpinner(SpinnerNumberModel(1.0, 0.0, Double.POSITIVE_INFINITY, 0.1))
    private val dataLabel = JLabel("T: 000.000 sec  Arm Length: 000.000 in  Arm Angle: 000.000 deg  Wrist Angle: 000.000 deg")

    private val canvas = SuperstructureCanvas(ppi, cargoToolLength, hatchToolLength, Dimension(481, 400))

    private val decFmt = DecimalFormat("###.###")

    private var activeTime = 0.0

    private var keyHeld = false

    fun getPlaybackSpeed(): Double {
        return speedSpinner.value as Double
    }

    fun reset() {
        activeTime = 0.0
        pause()
        draw()
    }

    fun draw() {
        try {
            val point = data.first { it.time >= activeTime }
            val armState = ArmState(point.command.armRadius, point.command.armAngle, point.command.armVelocity)
            val wristState = WristState(point.command.wristTheta, false, false)
            dataLabel.text =
                "T: ${decFmt.format(point.time)}  Arm Length: ${decFmt.format(armState.armRadius.value)} in  Arm Angle: ${decFmt.format(
                    armState.armAngle.toDegrees().value
                )} deg  Wrist Angle: ${decFmt.format(wristState.wristPosition.toDegrees().value)} deg"
            canvas.update(armState, wristState)
            canvas.repaint()
            validate()
        } catch (e: Exception) {}
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
            }, 0L, (sPerFrame * 1000.0).roundToLong(), TimeUnit.MILLISECONDS)
        }
    }

    @Synchronized fun pause() {
        currentFuture?.cancel(false)
        currentFuture = null
    }

    init {
        val canvasPane = JPanel(BorderLayout())
        canvasPane.border = BorderFactory.createLineBorder(Color.black)
        canvasPane.add(canvas, BorderLayout.CENTER)
        add(canvasPane, BorderLayout.CENTER)

        val lowerPane = JPanel(BorderLayout())

        val controlsPane = JPanel()
        lowerPane.border = BorderFactory.createLineBorder(Color.black)
        controlsPane.add(stepRevButton)
        controlsPane.add(playButton)
        controlsPane.add(pauseButton)
        controlsPane.add(stepFwdButton)
        controlsPane.add(resetButton)

        (speedSpinner.editor as JSpinner.NumberEditor).textField.columns = 4

        val speedPane = JPanel()
        speedPane.add(JLabel("Playback Speed:"))
        speedPane.add(speedSpinner)

        controlsPane.add(speedPane)
        lowerPane.add(controlsPane, BorderLayout.PAGE_START)
        lowerPane.add(dataLabel, BorderLayout.PAGE_END)

        add(lowerPane, BorderLayout.PAGE_END)

        playButton.addActionListener { play() }
        pauseButton.addActionListener { pause() }
        resetButton.addActionListener { reset() }
        stepFwdButton.addActionListener { advance(dt); draw() }
        stepRevButton.addActionListener { decrement(dt); draw() }

        KeyboardFocusManager.getCurrentKeyboardFocusManager()
            .addKeyEventDispatcher {
                if (it.id == KeyEvent.KEY_RELEASED) {
                    when (it.keyCode) {
                        39 -> { //Right arrow key
                            advance(dt)
                            draw()
                        }
                        37 -> { //Left arrow key
                            decrement(dt)
                            draw()
                        }
                    }
                }
                false
            }

        reset()
    }
}