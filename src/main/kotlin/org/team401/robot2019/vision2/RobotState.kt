package org.team401.robot2019.vision2

import org.snakeskin.hardware.Hardware
import org.team401.taxis.geometry.*
import org.team401.taxis.util.InterpolatingDouble
import org.team401.taxis.util.InterpolatingTreeMap
import org.team401.vision2.AimingParameters
import org.team401.vision2.GoalTrack
import org.team401.vision2.GoalTracker
import org.team401.vision2.TargetInfo
import java.util.*
import kotlin.math.abs
import kotlin.math.hypot

object RobotState {
    private const val OBS_BUFFER_SIZE = 100

    private lateinit var fieldToVehicleMap: InterpolatingTreeMap<InterpolatingDouble, Pose2d>

    lateinit var vehicleVelocityPredicted: Twist2d
    private set
    @Synchronized get

    lateinit var vehicleVelocityMeasured: Twist2d
    private set
    @Synchronized get

    lateinit var vehicleVelocityMeasuredFiltered: MovingAverageTwist2d
    private set
    @Synchronized get

    var distanceDriven = 0.0
    private set
    @Synchronized get

    private val targetTrackerLow = GoalTracker()

    private val cameraToTargetLowMap = arrayListOf<Translation2d?>()

    init {
        reset(0.0, Pose2d.identity())
    }

    @Synchronized
    fun reset(startTime: Double, initialFieldToVehicle: Pose2d) {
        fieldToVehicleMap = InterpolatingTreeMap(OBS_BUFFER_SIZE)
        fieldToVehicleMap[InterpolatingDouble(startTime)] = initialFieldToVehicle
        vehicleVelocityPredicted = Twist2d.identity()
        vehicleVelocityMeasured = Twist2d.identity()
        vehicleVelocityMeasuredFiltered = MovingAverageTwist2d(25)
        distanceDriven = 0.0
    }

    @Synchronized
    fun reset() {
        reset(Hardware.getRelativeTime(), Pose2d.identity())
    }

    @Synchronized
    fun getFieldToVehicle(timestamp: Double) = fieldToVehicleMap.getInterpolated(InterpolatingDouble(timestamp))

    @Synchronized
    fun getLatestFieldToVehicle() = fieldToVehicleMap.lastEntry()

    @Synchronized
    fun getPredictedFieldToVehicle(lookaheadTime: Double) = getLatestFieldToVehicle().value
        .transformBy(Pose2d.exp(vehicleVelocityPredicted.scaled(lookaheadTime)))

    @Synchronized
    fun addFieldToVehicleObservation(timestamp: Double, observation: Pose2d) {
        fieldToVehicleMap[InterpolatingDouble(timestamp)] = observation
    }

    @Synchronized
    fun addObservations(timestamp: Double, displacement: Twist2d, measuredVelocity: Twist2d, predictedVelocity: Twist2d) {
        distanceDriven += displacement.dx
        addFieldToVehicleObservation(
            timestamp,
            DifferentialKinematics.integrateForwardKinematics(getLatestFieldToVehicle().value, displacement)
        )
        vehicleVelocityMeasured = measuredVelocity
        if (abs(vehicleVelocityMeasured.dtheta) < 2.0 * Math.PI) {
            //reject high angular velocities
            vehicleVelocityMeasuredFiltered.add(vehicleVelocityMeasured)
        } else {
            vehicleVelocityMeasuredFiltered.add(Twist2d(
                vehicleVelocityMeasured.dx,
                vehicleVelocityMeasured.dy,
                0.0
            ))
        }
        vehicleVelocityPredicted = predictedVelocity
    }

    @Synchronized
    fun resetDistanceDriven() {
        distanceDriven = 0.0
    }

    @Synchronized
    fun resetVision() {
        targetTrackerLow.reset()
    }

    private fun calculateCameraToTarget(target: TargetInfo, camera: LimelightCameraEnhanced): Translation2d? {
        val xzPlaneTranslation = Translation2d(target.x, target.z).rotateBy(camera.horizontalPlaneToCamera)
        val x = xzPlaneTranslation.x()
        val y = target.y
        val z = xzPlaneTranslation.y() //this is really z

        //Find the intersection with the goal
        val differentialHeight = camera.height - VisionConstants.lowTargetHeight
        if ((z < 0.0) == (differentialHeight > 0.0)) {
            val scaling = differentialHeight / -z
            val distance = hypot(x, y) * scaling
            val angle = Rotation2d(x, y, true)
            return Translation2d(distance * angle.cos(), distance * angle.sin())
        }
        return null
    }

    private fun updateTargetTracker(timestamp: Double, cameraToTargetList: List<Translation2d?>, tracker: GoalTracker, camera: LimelightCameraEnhanced) {
        if (cameraToTargetList.size != 2 ||
                    cameraToTargetList[0] == null ||
                    cameraToTargetList[1] == null) return

        val cameraToTarget = Pose2d.fromTranslation(cameraToTargetList[0]!!.interpolate(
            cameraToTargetList[1], 0.5))

        val fieldToTarget = getFieldToVehicle(timestamp).transformBy(camera.robotToCamera).transformBy(cameraToTarget)
        tracker.update(timestamp, listOf(Pose2d(fieldToTarget.translation, Rotation2d.identity())))
    }

    @Synchronized
    fun addVisionUpdate(timestamp: Double, observations: List<TargetInfo>?, camera: LimelightCameraEnhanced) {
        cameraToTargetLowMap.clear()

        if (observations == null || observations.isEmpty()) {
            targetTrackerLow.update(timestamp, arrayListOf())
            return
        }

        observations.forEach {
            cameraToTargetLowMap.add(calculateCameraToTarget(it, camera))
        }

        updateTargetTracker(timestamp, cameraToTargetLowMap, targetTrackerLow, camera)
    }

    private val possibleTargetNormals = doubleArrayOf(0.0, 90.0, 180.0, 270.0, 30.0, 150.0, 210.0, 330.0)

    @Synchronized
    fun getFieldToVisionTarget(): Pose2d? {
        if (!targetTrackerLow.hasTracks()) {
            return null
        }

        val fieldToTarget = targetTrackerLow.tracks[0].field_to_target
        val normalPositive = (fieldToTarget.rotation.degrees + 360) % 360
        var normalClamped = possibleTargetNormals[0]
        possibleTargetNormals.forEach {
            if (abs(normalPositive - it) < abs(normalPositive - normalClamped)) {
                normalClamped = it
            }
        }

        return Pose2d(fieldToTarget.translation, Rotation2d.fromDegrees(normalClamped))
    }

    @Synchronized
    fun getVehicleToVisionTarget(timestamp: Double): Pose2d? {
        val fieldToTarget = getFieldToVisionTarget() ?: return null

        return getFieldToVehicle(timestamp).inverse().transformBy(fieldToTarget)
    }

    @Synchronized
    fun getAimingParameters(prevTrackId: Int, maxTrackAge: Double): Optional<AimingParameters> {
        val reports = targetTrackerLow.tracks

        if (reports.isEmpty()) {
            return Optional.empty()
        }

        val timestamp = Hardware.getRelativeTime()

        val comparator = GoalTracker.TrackReportComparator(
            VisionConstants.trackStabilityWeight,
            VisionConstants.trackAgeWeight,
            VisionConstants.trackSwitchingWeight,
            prevTrackId, timestamp
        )
        reports.sortWith(comparator)

        var report: GoalTracker.TrackReport? = null
        for (track: GoalTracker.TrackReport in reports) {
            if (track.latest_timestamp > timestamp - maxTrackAge) {
                report = track
                break
            }
        }

        if (report == null) {
            return Optional.empty()
        }

        val vehicleToGoal = getFieldToVehicle(timestamp).inverse().transformBy(report.field_to_target)

        val params = AimingParameters(
            vehicleToGoal,
            report.field_to_target,
            report.field_to_target.rotation,
            report.latest_timestamp,
            report.stability,
            report.id
        )
        return Optional.of(params)
    }
}