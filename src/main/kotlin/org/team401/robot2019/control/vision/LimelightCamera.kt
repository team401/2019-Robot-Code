package org.team401.robot2019.control.vision

import edu.wpi.first.networktables.*
import org.snakeskin.hardware.Hardware
import org.snakeskin.logic.LockingDelegate
import org.snakeskin.measure.Degrees
import org.snakeskin.measure.Inches
import org.snakeskin.measure.Milliseconds
import org.snakeskin.measure.Seconds
import org.snakeskin.measure.time.TimeMeasureMilliseconds
import org.team401.taxis.geometry.Pose2d

/**
 * Class for reading data from and controlling a Limelight camera.
 *
 * Some notes about the pose data returned by the Limelight:
 * 1. The distance away from the target (x in our coordinate system) is z, and gets smaller (negative) as you move away
 * 2. The horizontal displacement from the target (y in our coordinate system) is x, and gets larger (positive) as you move right
 * 3. The yaw decreases (negative) as the camera looks more rightward from the target
 */
class LimelightCamera(val name: String, val robotToCamera: Pose2d, val constantLatency: TimeMeasureMilliseconds = 11.0.Milliseconds) {
    /**
     * Represents the different modes for the limelight camera.
     * @property VisionProcessor Puts the camera in vision mode, which will run the selected pipeline
     * @property DriverCamera Puts the camera in driver mode, which disables the pipeline and raises exposure
     */
    object CameraMode {
        const val VisionProcessor = 0
        const val DriverCamera = 1
    }

    /**
     * Represents the different modes of the LED
     */
    object LedMode {
        const val UsePipeline = 0
        const val Off = 1
        const val Blink = 2
        const val On = 3
    }

    /**
     * Represents the different modes the camera can stream in
     */
    object StreamingMode {
        const val SideBySide = 0
        const val PipMain = 1
        const val PipSecondary = 2
    }

    private val table = NetworkTableInstance.getDefault().getTable(name)

    /**
     * Inner class for holding the TableEntry objects that are used for the limelight
     */
    private inner class TableEntries {
        val camtran = table.getEntry("camtran")
        val tv = table.getEntry("tv")
        val tx = table.getEntry("tx")
        val ty = table.getEntry("ty")
        val tl = table.getEntry("tl")
        val getpipe = table.getEntry("getpipe")
        val ledMode = table.getEntry("ledMode")
        val camMode = table.getEntry("camMode")
        val pipeline = table.getEntry("pipeline")
        val stream = table.getEntry("stream")
    }

    /**
     * Singleton instance of TableEntries
     */
    private val entries = TableEntries()

    /**
     * Inner class responsible for listening for camera data and storing it in the outer field
     */
    private inner class Listener: TableEntryListener {
        private val camtranDefault = DoubleArray(6)

        override fun valueChanged(
            table: NetworkTable,
            key: String,
            entry: NetworkTableEntry,
            value: NetworkTableValue,
            flags: Int
        ) {
            val timestamp = Hardware.getRelativeTime() //Capture the timestamp as soon as possible.
            val camtran = entries.camtran.getDoubleArray(camtranDefault)
            val tv = entries.tv.getDouble(0.0) == 1.0 //Has targets.  If it's 1.0, it's true, otherwise false
            val tx = entries.tx.getDouble(0.0) //Target angle x
            val ty = entries.ty.getDouble(0.0) //Target angle y
            val tl = entries.tl.getDouble(0.0) //Pipeline latency
            val getpipe = entries.getpipe.getDouble(0.0).toInt() //Active pipeline

            //Set active frame
            frame = VisionFrame(
                timestamp.Seconds,
                tv,
                tx.Degrees,
                ty.Degrees,
                tl.Milliseconds + constantLatency,
                getpipe,
                camtran[0].Inches,
                camtran[1].Inches,
                camtran[2].Inches,
                camtran[3].Degrees,
                camtran[4].Degrees,
                camtran[5].Degrees
            )
        }

    }

    /**
     * The latest frame captured by this camera.
     */
    var frame by LockingDelegate(VisionFrame.identity())
    private set

    private var activeListener = -1

    /**
     * Tells the NT API to begin listening for data from this camera.
     */
    @Synchronized fun startListening() {
        //Listen for camtran.  This should update once every frame, and once when we transition to no targets.
        if (activeListener == -1) {
            activeListener = table.addEntryListener("camtran", Listener(), EntryListenerFlags.kUpdate)
        }
        //If activeListener is not -1, we are already listening and should not start a new listener
    }

    /**
     * Tells the NT API to stop listening for data from this camera.
     */
    @Synchronized fun stopListening() {
        if (activeListener != -1) {
            table.removeEntryListener(activeListener)
            activeListener = -1
        }
        //If activeListener is -1, we are not listening and should not try to stop anything
    }

    /**
     * Returns true if a listener is currently active.
     */
    @Synchronized fun isListening(): Boolean {
        return activeListener != -1
    }

    /**
     * Sets the active ledMode of the camera
     */
    @Synchronized fun setLedMode(mode: Int) {
        entries.ledMode.setNumber(mode)
    }

    /**
     * Sets the active cameraMode of the camera
     */
    @Synchronized fun setCameraMode(mode: Int) {
        entries.camMode.setNumber(mode)
    }

    /**
     * Sets the active pipeline of the camera
     */
    @Synchronized fun setPipeline(pipeline: Int) {
        entries.pipeline.setNumber(pipeline)
    }

    /**
     * Sets the active streaming mode of the camera
     */
    @Synchronized fun setStreamingMode(mode: Int) {
        entries.stream.setNumber(mode)
    }

    /**
     * Sets the camera mode to vision mode, the LED mode to follow the pipeline, and the pipeline to the given pipeline
     */
    @Synchronized fun configForVision(pipeline: Int) {
        entries.camMode.setNumber(CameraMode.VisionProcessor)
        entries.ledMode.setNumber(LedMode.UsePipeline)
        entries.pipeline.setNumber(pipeline)
    }

    /**
     * Sets the camera to driver mode, and the LED mode to off
     */
    @Synchronized fun configForDriverView() {
        entries.camMode.setNumber(CameraMode.DriverCamera)
        entries.ledMode.setNumber(LedMode.Off)
    }

    @Synchronized fun resetFrame() {
        frame = VisionFrame.identity()
    }
}