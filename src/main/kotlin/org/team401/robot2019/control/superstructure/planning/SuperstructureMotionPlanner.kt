package org.team401.robot2019.control.superstructure.planning

import org.snakeskin.measure.*
import org.snakeskin.measure.acceleration.angular.AngularAccelerationMeasureRadiansPerSecondPerSecond
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.geometry.*
import org.team401.robot2019.control.superstructure.planning.command.*
import org.team401.robot2019.subsystems.WristSubsystem
import org.team401.robot2019.subsystems.arm.control.ArmKinematics
import java.util.*

object SuperstructureMotionPlanner {
    enum class ControlMode {
        Planning, //Coordinated, motion planned control
        ArmJog, //Manual jogging
        WristJog // Manual wrist jogging
    }

    enum class SpeedMode(
        val acceleration: AngularAccelerationMeasureRadiansPerSecondPerSecond,
        val velocity: AngularVelocityMeasureRadiansPerSecond
    ) {
        Slow(ControlParameters.ArmParameters.rotationSlowAcceleration, ControlParameters.ArmParameters.rotationSlowVelocity),
        Normal(ControlParameters.ArmParameters.rotationAcceleration, ControlParameters.ArmParameters.rotationVelocity)
    }

    /**
     * The control mode that the system is currently in
     */
    var activeControlMode = ControlMode.Planning
    private set
    @Synchronized get

    var activeSpeedMode = SpeedMode.Normal
    private set
    @Synchronized get

    private var jogArmPose = Point2d(0.0.Inches, 0.0.Inches)
    private var jogWristState = WristState(0.0.Radians, false, false)
    private var jogXRate = 0.0
    private var jogYRate = 0.0
    private var jogWristRate = 0.0

    @Synchronized fun setToSlowSpeedMode() {
        reset()
        activeSpeedMode = SpeedMode.Slow
        ArmMotionPlanner.setSpeed(SpeedMode.Slow)
    }

    @Synchronized fun setToNormalSpeedMode() {
        reset()
        activeSpeedMode = SpeedMode.Normal
        ArmMotionPlanner.setSpeed(SpeedMode.Normal)
    }

    @Synchronized fun setToPlanningMode() {
        reset()
        activeControlMode = ControlMode.Planning
    }

    @Synchronized fun setToArmJogMode() {
        reset()
        jogArmPose = ArmKinematics.forward(lastObservedArmState)
        jogWristState = lastObservedWristState
        activeControlMode = ControlMode.ArmJog
    }

    @Synchronized fun setToWristJogMode() {
        reset()
        jogArmPose = ArmKinematics.forward(lastObservedArmState)
        jogWristState = lastObservedWristState
        activeControlMode = ControlMode.WristJog
    }

    /**
     * Updates the jog values
     */
    @Synchronized fun updateArmJog(x: Double, y: Double) {
        jogXRate = x
        jogYRate = y
    }

    @Synchronized fun updateWristJog(angle: Double) {
        jogWristRate = angle
    }

    val commandQueue = LinkedList<SuperstructureCommand>()

    /**
     * Last observed state of the arm
     */
    private var lastObservedArmState = ArmState(0.0.Inches, 0.0.Radians, 0.0.RadiansPerSecond)

    /**
     * Last observed state of the wrist
     */
    private var lastObservedWristState = WristState(0.0.Radians, false, false)

    /**
     * Active "target" tool for the system.
     */
    var activeTool = WristMotionPlanner.Tool.HatchPanelTool
    @Synchronized set
    @Synchronized get

    /**
     * Active "target" tool angle for the system.
     */
    private var activeToolAngle = 0.0.Radians

    @Synchronized fun notActiveTool(): WristMotionPlanner.Tool {
        return when (activeTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> WristMotionPlanner.Tool.CargoTool
            WristMotionPlanner.Tool.CargoTool -> WristMotionPlanner.Tool.HatchPanelTool
        }
    }

    // Each system has three states: e stopped, coordinated control and holding
    // The buttons set desired position and switch into coordinated control
    @Synchronized
    fun update(time: Double, dt: Double, armState: ArmState, wristState: WristState) {
        lastObservedArmState = armState     //Update state variables
        lastObservedWristState = wristState

        when (activeControlMode) {
            ControlMode.Planning -> {
                //Pop the first command from the queue
                if (commandQueue.isNotEmpty()) { //If there are commands in the queue
                    val currentCommand = commandQueue.pop() //Remove the first element

                    try {
                        currentCommand.update(dt, armState, wristState) //Update the command
                        if (!currentCommand.isDone()) { //If the command isn't done
                            commandQueue.push(currentCommand) //Put it back at the start of the queue
                        }
                    } catch (e: Exception) {
                        //An exception occured during this command.  Print out some debug info and reset the motion planner.
                        System.err.println("Exception encountered in Superstructure Motion Planner")
                        System.err.println("Active Command:\t${currentCommand.javaClass.simpleName}")
                        System.err.println("Description:\t${currentCommand.getDescription()}")
                        System.err.println("Stack trace:")
                        e.printStackTrace()

                        reset() //Reset the planner
                    }
                }
            }

            ControlMode.ArmJog -> {
                var newX = jogArmPose.x.value + (jogXRate * 0.1)
                var newY = jogArmPose.y.value + (jogYRate * 0.1)

                val checkPose = ArmKinematics.inverse(Point2d(newX.Inches, newY.Inches))

                // Check limits
                if (newX !in -Geometry.ArmGeometry.maxX.value .. Geometry.ArmGeometry.maxX.value) {
                    newX = if (newX < -Geometry.ArmGeometry.maxX.value) {
                        -Geometry.ArmGeometry.maxX.value
                    }else{
                        Geometry.ArmGeometry.maxX.value
                    }
                }
                if (newY !in Geometry.ArmGeometry.minY.value .. Geometry.ArmGeometry.maxY.value) {
                    newY = if (newY > Geometry.ArmGeometry.maxY.value){
                        Geometry.ArmGeometry.maxY.value
                    }else{
                        Geometry.ArmGeometry.minY.value
                    }
                }

                if (checkPose.r < Geometry.ArmGeometry.minSafeArmLength) {
                    val temp = ArmKinematics.forward(PointPolar(Geometry.ArmGeometry.minSafeArmLength, checkPose.theta))
                    newX = temp.x.value
                    newY = temp.y.value
                }

                val newPose = Point2d(newX.Inches, newY.Inches)
                val newState = ArmKinematics.inverse(newPose)

                jogArmPose = newPose

                SuperstructureController.update(
                    ArmState(newState.r, newState.theta, 0.0.RadiansPerSecond),
                    jogWristState,
                    activeTool,
                    VisionHeightMode.NONE
                )
            }
            ControlMode.WristJog -> {

                var newWristAngle = jogWristState.wristPosition.value + (jogWristRate * 0.025)

                if (newWristAngle > 3.0) {
                    newWristAngle = 3.0
                }
                if (newWristAngle < -3.0) {
                    newWristAngle = -3.0
                }

                //println("Wrist jog. Angle: $newWristAngle")

                val newWristState = WristState(newWristAngle.Radians, jogWristState.hasCargo, jogWristState.hasHatchPanel)

                jogWristState = newWristState

                val armState = ArmKinematics.inverse(jogArmPose)

                SuperstructureController.update(
                    ArmState(armState.r, armState.theta, 0.0.RadiansPerSecond),
                    newWristState,
                    activeTool,
                    VisionHeightMode.NONE
                )
            }
        }
    }

    /**
     * Sets up the controller and the planner for starting configuration
     */
    @Synchronized
    fun startUp(
        startArmState: ArmState,
        startWristState: WristState,
        startingTool: WristMotionPlanner.Tool = WristMotionPlanner.Tool.HatchPanelTool
    ){
        lastObservedArmState = startArmState
        lastObservedWristState = startWristState
        activeTool = startingTool
        reset()
    }

    /**
     * Hacky solution to a dumb problem.  It appears that the motion command classes not being initially JIT compiled
     * causes jerky motion on the first command submitted to the queue upon startup.  This function creates a fake
     * profile and executes it 1000 times, which should be enough to cause the JIT compiler to statically compile the
     * class.  It is entirely possible that there is a bug in the motion planner that causes this until a command is run,
     * but this is unlikely as we are perfect and never make mistakes (please ignore 2017 robot).
     */
    @Synchronized fun preCompile() {
        startUp(
            ArmState(
                Geometry.ArmGeometry.minSafeArmLength + 1.0.Inches,
                0.0.Radians,
                0.0.RadiansPerSecond
            ),
            WristState(0.0.Radians, false, false),
            WristMotionPlanner.Tool.HatchPanelTool
        )

        requestMove(SuperstructureSetpoint(ArmKinematics.forward(
            PointPolar(
                Geometry.ArmGeometry.minSafeArmLength + 1.0.Inches,
                Math.PI.Radians
            )), WristMotionPlanner.Tool.HatchPanelTool, 0.0.Radians, WristSubsystem.WristToolStates.Cargo, VisionHeightMode.NONE))

        var time = 0.0
        val dt = 0.1

        for (i in 0 until 1000) {
            SuperstructureMotionPlanner.update(time, dt,  ArmState(
                Geometry.ArmGeometry.minSafeArmLength + 1.0.Inches,
                0.0.Radians,
                0.0.RadiansPerSecond
            ), WristState(0.0.Radians, false, false))
            time += dt
        }

        startUp(ArmState(0.0.Inches, 0.0.Radians, 0.0.RadiansPerSecond),
            WristState(0.0.Radians, false, false), WristMotionPlanner.Tool.HatchPanelTool)
    }

    @Synchronized
    private fun reset(){
        commandQueue.clear()

        //Push through a command to the controller so that when the subsystems
        //enter coordinated control, they'll hold at their current states and not
        //jerk around.
        SuperstructureController.update(
            ArmState(
                lastObservedArmState.armRadius,
                lastObservedArmState.armAngle,
                0.0.RadiansPerSecond //Stop arm motion
            ),
            lastObservedWristState,
            activeTool,
            VisionHeightMode.NONE
        )
    }

    /**
     * Returns whether or not the motion planner is done with it's command queue.
     * If the queue is empty, the planner is considered "done" and this will return true
     */
    @Synchronized fun isDone(): Boolean {
        return commandQueue.isEmpty()
    }

    /**
     * Asks the system to change tools, preferably without moving.
     *
     * Returns true if the motion planner will do the action, false if not
     */
    @Synchronized fun requestToolChange(tool: WristMotionPlanner.Tool): Boolean {
        if (tool == activeTool) { //If we're trying to switch to the same tool, ignore it
            println("Ignoring switch to already selected tool!")
            return false
        }

        when (activeTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> {
                if (lastObservedWristState.hasHatchPanel) { //If we're in the hatch panel tool and we have a hatch panel
                    println("You're a tool!  Drop the hatch panel to switch tools.") //Reject the switch
                    return false
                }
            }

            WristMotionPlanner.Tool.CargoTool -> {
                if (lastObservedWristState.hasCargo) { //Same as above for cargo
                    println("You're a tool!  Drop the cargo to switch tools.")
                    return false
                }
            }
        }

        if (!isDone()) return false //Don't do this unless our other steps are done
        reset()

        val startArmPose = ArmKinematics.forward(lastObservedArmState) //Calculate the current arm pose

        /*
        var hadToMove = false

        val safeToolChangePolar =
            if (startArmPose.x >= 0.0.Inches) {
                ArmKinematics.inverse(Point2d(Geometry.ArmGeometry.minToolChangeX, Geometry.ArmGeometry.minToolChangeY))
            } else {
                ArmKinematics.inverse(Point2d((-1.0).Unitless * Geometry.ArmGeometry.minToolChangeX, Geometry.ArmGeometry.minToolChangeY))
            }

        if (startArmPose.x < Geometry.ArmGeometry.minToolChangeX) {
            //Do a rotation only move from current pose to new pose
            commandQueue.add(RotationOnlyCommand(safeToolChangePolar.theta, activeTool))
            hadToMove = true
        }

        if (lastObservedArmState.armRadius <= safeToolChangePolar.r) {
            //Do an extension only move from current pose to new pose
            commandQueue.add(ExtensionOnlyCommand(safeToolChangePolar.r, activeTool))
            hadToMove = true
        }
        */


        val sideOffset = if (startArmPose.x >= 0.0.Inches) {
            WristMotionPlanner.POSITIVE_X_OFFSET
        } else {
            WristMotionPlanner.NEGATIVE_X_OFFSET
        }

        //Calculate the current tool's angle from the floor
        activeTool = notActiveTool()
        commandQueue.add(SetWristAngleCommand(
            activeTool,
            activeToolAngle,
            startArmPose
        ))

        /*
        if (hadToMove) {
            //TODO move back, skipping for now
        }
        */

        return true
    }

    /**
     * Forces the system to stop its current action and go to the safe home position.
     */
    @Synchronized fun goHome() {
        reset()

        if (lastObservedArmState.armRadius < Geometry.ArmGeometry.minSafeArmLength) {
            //We need to extend out to the minimum radius first
            commandQueue.add(ExtensionOnlyCommand(Geometry.ArmGeometry.minSafeArmLength, activeTool))
        }
        val safePoint = ArmKinematics.forward(PointPolar(Geometry.ArmGeometry.minSafeArmLength + 0.1.Inches, lastObservedArmState.armAngle))

        activeToolAngle = 0.0.Radians

        //Now we need to move the arm to the safe location
        commandQueue.add(MoveSuperstructureCommandStaticWrist(safePoint, Point2d(0.0.Inches, Geometry.ArmGeometry.minSafeArmLength + 0.1.Inches), activeTool, 0.0.Radians, Geometry.ArmGeometry.minSafeArmLength, VisionHeightMode.NONE))
    }

    /**
     * Stows the system to prepare to climb.  This shortens the system below the minimum length
     * in order to keep us from going over the driver station wall.  Thus, to ensure the safety
     * of the mechanism, the movement is done sequentially rather than in parallel
     */
    @Synchronized fun goToClimb() {
        reset()

        commandQueue.add(ExtensionOnlyCommand(Geometry.ArmGeometry.minSafeArmLength + 0.1.Inches, WristMotionPlanner.Tool.HatchPanelTool))
        commandQueue.add(SetWristAngleCommand(WristMotionPlanner.Tool.HatchPanelTool, 0.0.Radians, ArmKinematics.forward(PointPolar(Geometry.ArmGeometry.minSafeArmLength + 0.1.Inches, 90.0.Degrees.toRadians()))))
        commandQueue.add(DelayCommand(1.0.Seconds))
        commandQueue.add(ExtensionOnlyCommand(Geometry.ArmGeometry.armBaseLength + Geometry.ArmGeometry.armExtensionStickout + 2.0.Inches, WristMotionPlanner.Tool.HatchPanelTool))
    }

    /**
     * "Thrusts" the arm backwards (climbing forwards) to shift our CG over the platform as the front (climbing backwards)
     * legs are being retracted.  This helps to keep us from falling backwards off of the platform when we climb.
     */
    @Synchronized fun climbThrust() {
        reset()

        commandQueue.add(RotationOnlyCommand((120.0).Degrees.toRadians(), WristMotionPlanner.Tool.HatchPanelTool))
    }

    /**
     * Asks the system to move to a new pose, while following a series of constraints.
     *
     * Returns true if the motion planner will do the action, false if not
     */
    @Synchronized fun requestMove(setpoint: SuperstructureSetpoint): Boolean {
        if (setpoint.tool != activeTool) {
            println("Please switch tools before performing this move")
            return false
        }

        if (!isDone()) return false
        reset()

        val currentPose = ArmKinematics.forward(lastObservedArmState)

        /*
        //First, we'll obtain the potential pose of the wrist given an immediate rotation to the target angle
        val wristInstantRotatedPose = EndpointKinematics.forward(
            lastObservedArmState,
            WristState(
                WristMotionPlanner.calculateFinalFloorAngle(lastObservedArmState, setpoint.toolAngle, setpoint.tool),
                lastObservedWristState.hasCargo,
                lastObservedWristState.hasHatchPanel
            ),
            setpoint.cargoGrabberState,
            setpoint.hatchClawState
        )

        //We now need to evaluate the x coordinate with the greatest magnitude that the profile will traverse
        //while at the minimum radius.  This can be determined by looking at the angular direction and whether
        //or not the trajectory crosses y = 0.  If it does, the x value at y = 0 is the maximum.  Otherwise, it is
        //the endpoint with the greatest x magnitude.
        val startAngle = lastObservedArmState.armAngle
        val endAngle = ArmKinematics.inverse(setpoint.point).theta
        val startX = ArmKinematics.forward(lastObservedArmState).x
        val endX = setpoint.point.x

        var xMaxTheta =
        if (Math.abs(startX.value) epsGt Math.abs(endX.value)) {
            //We're moving to the left.  The angle with the maximum x angle is the starting angle
            xMaxTheta = startAngle
        } else {
            //We're moving to the right.  The angle with the maximum x angle is the ending angle
            xMaxTheta = endAngle
        }

        if (startAngle.value epsGt endAngle.value) {
            //We are moving clockwise.
            if (Math.PI in endAngle.value..startAngle.value || 0.0 in endAngle.value..startAngle.value) {
                //180 degrees is within this trajectory, so y = 0 is the x coordinate with the greatest magnitude
                //in the negative direction
                xMax = Geometry.ArmGeometry.minSafeWristToolChangeRadius
            }
        } else {
            //We are moving counter-clockwise.
            //Same cases above, except accounting for the fact that Kotlin ranges must start with the smaller number
            if (Math.PI in startAngle.value..endAngle.value || 0.0 in startAngle.value..endAngle.value) {
                xMax = Geometry.ArmGeometry.minSafeWristToolChangeRadius
            }
        }
        */

        //TODO there is definitely a case here where we could end up pushing the extension into the floor.
        //TODO if the radius is too short and we are already on the floor, this will happen
        //TODO add checks to prevent this
        val targetPose = setpoint.point
        val minimumRadius = activeTool.minimumRadius + 1.0.Inches //TODO test if this is really necessary
        if (lastObservedArmState.armRadius < minimumRadius) {
            val safePoint = ArmKinematics.forward(PointPolar(minimumRadius + 0.1.Inches, lastObservedArmState.armAngle))
            commandQueue.add(ExtensionOnlyCommand(minimumRadius + 0.1.Inches, activeTool))
            commandQueue.add(
                MoveSuperstructureCommandStaticWrist(
                    safePoint,
                    targetPose,
                    activeTool,
                    setpoint.toolAngle,
                    minimumRadius,
                    setpoint.visionHeightMode
                )
            )
        } else {
            commandQueue.add(
                MoveSuperstructureCommandStaticWrist(
                    currentPose,
                    targetPose,
                    activeTool,
                    setpoint.toolAngle,
                    minimumRadius,
                    setpoint.visionHeightMode
                )
            )
        }

        activeToolAngle = setpoint.toolAngle

        return true
    }

    @Synchronized fun goToFloorPickup(){
        reset()

        commandQueue.add(ExtensionOnlyCommand(Geometry.ArmGeometry.minSafeArmLength, activeTool))
        commandQueue.add(SetWristAngleAbsoluteCommand(activeTool, ControlParameters.FloorPickupParameters.floorPickupAngle))
        commandQueue.add(DelayCommand(0.5.Seconds))
        commandQueue.add(ExtensionOnlyCommand(ControlParameters.FloorPickupParameters.floorPickupPoint.r, activeTool))
        commandQueue.add(RotationOnlyCommand(ControlParameters.FloorPickupParameters.floorPickupPoint.theta, activeTool))
    }

    @Synchronized fun returnFromFloorPickup() {
        reset()

        commandQueue.add(SetWristAngleAbsoluteCommand(activeTool, ControlParameters.FloorPickupParameters.floorPickupAngle - 0.0.Degrees.toRadians()))
        commandQueue.add(ExtensionOnlyCommand(ControlParameters.FloorPickupParameters.floorPickupPoint.r + 5.0.Inches, activeTool))
        commandQueue.add(RotationOnlyCommand(90.0.Degrees.toRadians(), activeTool))
    }
}