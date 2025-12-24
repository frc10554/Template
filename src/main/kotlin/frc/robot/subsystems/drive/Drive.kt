package frc.robot.subsystems.drive

import choreo.trajectory.SwerveSample
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.robot.Constants
import frc.robot.generated.TunerConstants
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs
import java.util.concurrent.locks.Lock
import java.util.concurrent.locks.ReentrantLock
import kotlin.math.hypot
import kotlin.math.max

class Drive(
    private val gyroIO: GyroIO,
    flModuleIO: ModuleIO?,
    frModuleIO: ModuleIO?,
    blModuleIO: ModuleIO?,
    brModuleIO: ModuleIO?
) : SubsystemBase() {
    private val gyroInputs = GyroIOInputsAutoLogged()
    private val modules: Array<Module?> = arrayOfNulls<Module>(4) // FL, FR, BL, BR
    private val sysId: SysIdRoutine
    private val gyroDisconnectedAlert = Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError)

    private val kinematics = SwerveDriveKinematics(*moduleTranslations)
    private var rawGyroRotation = Rotation2d()
    private val lastModulePositions: Array<SwerveModulePosition> =  // For delta tracking
        arrayOf(
            SwerveModulePosition(),
            SwerveModulePosition(),
            SwerveModulePosition(),
            SwerveModulePosition()
        )
    private val poseEstimator = SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d())

    // Choreo path following controllers
    private val xController = PIDController(12.0, 0.0, 0.0)
    private val yController = PIDController(12.0, 0.0, 0.0)
    private val thetaController = PIDController(5.0, 0.0, 0.0)

    init {
        // Enable continuous input for theta controller
        thetaController.enableContinuousInput(-Math.PI, Math.PI)

        // Initialize modules - validate inputs first
        val fl = flModuleIO ?: throw IllegalStateException("Front left module IO is null")
        val fr = frModuleIO ?: throw IllegalStateException("Front right module IO is null")
        val bl = blModuleIO ?: throw IllegalStateException("Back left module IO is null")
        val br = brModuleIO ?: throw IllegalStateException("Back right module IO is null")
        
        val flConstants = TunerConstants.FrontLeft
        val frConstants = TunerConstants.FrontRight
        val blConstants = TunerConstants.BackLeft
        val brConstants = TunerConstants.BackRight
        
        modules[0] = Module(fl, 0, flConstants)
        modules[1] = Module(fr, 1, frConstants)
        modules[2] = Module(bl, 2, blConstants)
        modules[3] = Module(br, 3, brConstants)

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit)

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start()

        // Configure SysId
        sysId =
            SysIdRoutine(
                SysIdRoutine.Config(
                    null,
                    null,
                    null
                ) { state: SysIdRoutineLog.State? ->
                    Logger.recordOutput(
                        "Drive/SysIdState",
                        state.toString()
                    )
                },
                Mechanism(
                    { voltage: Voltage? -> voltage?.let { runCharacterization(it.`in`(Units.Volts)) } }, null, this
                )
            )
    }

    override fun periodic() {
        odometryLock.lock() // Prevents odometry updates while reading data
        try {
            gyroIO.updateInputs(gyroInputs)
            (gyroInputs as? LoggableInputs)?.let {
                Logger.processInputs("Drive/Gyro", it)
            }
            modules.forEach { it?.periodic() }
        } finally {
            odometryLock.unlock()
        }

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            modules.forEach { it?.stop() }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput<SwerveModuleState?>("SwerveStates/Setpoints", *arrayOf<SwerveModuleState?>())
            Logger.recordOutput<SwerveModuleState?>("SwerveStates/SetpointsOptimized", *arrayOf<SwerveModuleState?>())
        }

        // Update odometry
        val sampleTimestamps = modules[0]?.odometryTimestamps ?: return
        val sampleCount = sampleTimestamps.size
        for (i in 0 until sampleCount) {
            // Read wheel positions and deltas from each module
            val modulePositions = arrayOfNulls<SwerveModulePosition>(4)
            val moduleDeltas = arrayOfNulls<SwerveModulePosition>(4)
            var hasValidData = false
            
            for (moduleIndex in 0..3) {
                val module = modules[moduleIndex] ?: continue
                val odometryPositions = module.odometryPositions
                if (i >= odometryPositions.size) continue
                
                val currentPosition = odometryPositions[i] ?: continue
                modulePositions[moduleIndex] = currentPosition
                moduleDeltas[moduleIndex] = SwerveModulePosition(
                    currentPosition.distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                    currentPosition.angle
                )
                lastModulePositions[moduleIndex] = currentPosition
                hasValidData = true
            }
            
            // Skip update if no valid data
            if (!hasValidData) continue

            // Update gyro angle
            if (gyroInputs.connected && i < gyroInputs.odometryYawPositions.size) {
                // Use the real gyro angle
                gyroInputs.odometryYawPositions[i]?.let {
                    rawGyroRotation = it
                }
            } else {
                // Use the angle delta from the kinematics and module deltas
                // Only calculate if we have valid deltas
                val validDeltas = moduleDeltas.filterNotNull()
                if (validDeltas.size >= 2) {
                    // Fill array with valid deltas and zero deltas for missing modules
                    val deltasForTwist = Array(4) { index ->
                        moduleDeltas[index] ?: SwerveModulePosition(0.0, Rotation2d())
                    }
                    val twist = kinematics.toTwist2d(*deltasForTwist)
                    rawGyroRotation = rawGyroRotation.plus(Rotation2d(twist.dtheta))
                }
            }

            // Apply update - ensure we have at least some valid positions
            val validPositions = modulePositions.filterNotNull()
            if (validPositions.size >= 2) {
                // Fill array with valid positions and zero positions for missing modules
                val positionsForUpdate = Array(4) { index ->
                    modulePositions[index] ?: SwerveModulePosition(0.0, Rotation2d())
                }
                poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, positionsForUpdate)
            }
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Constants.Mode.SIM)
    }

    /**
     * Follows a Choreo SwerveSample.
     */
    fun followTrajectory(sample: SwerveSample) {
        val pose = this.pose ?: return
        
        // Calculate feedback
        val xFF = sample.vx
        val yFF = sample.vy
        val thetaFF = sample.omega

        val xFeedback = xController.calculate(pose.x, sample.x)
        val yFeedback = yController.calculate(pose.y, sample.y)
        val thetaFeedback = thetaController.calculate(pose.rotation.radians, sample.heading)

        val outputSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback,
            yFF + yFeedback,
            thetaFF + thetaFeedback,
            pose.rotation
        )
        
        runVelocity(outputSpeeds)
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    fun runVelocity(speeds: ChassisSpeeds) {
        // Calculate module setpoints
        val discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02)
        val setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts)

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", *setpointStates)
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds)

        // Send setpoints to modules
        modules.forEachIndexed { index, module ->
            module?.runSetpoint(setpointStates[index])
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", *setpointStates)
    }

    /** Runs the drive in a straight line with the specified drive output.  */
    fun runCharacterization(output: Double) {
        modules.forEach { it?.runCharacterization(output) }
    }

    /** Stops the drive.  */
    fun stop() {
        runVelocity(ChassisSpeeds())
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    fun stopWithX() {
        val headings = moduleTranslations.map { it.angle }.toTypedArray()
        kinematics.resetHeadings(*headings)
        stop()
    }

    /** Returns a command to run a quasistatic test in the specified direction.  */
    fun sysIdQuasistatic(direction: SysIdRoutine.Direction?): Command? {
        return run(Runnable { runCharacterization(0.0) })
            .withTimeout(1.0)
            .andThen(sysId.quasistatic(direction))
    }

    /** Returns a command to run a dynamic test in the specified direction.  */
    fun sysIdDynamic(direction: SysIdRoutine.Direction?): Command? {
        return run(Runnable { runCharacterization(0.0) }).withTimeout(1.0).andThen(sysId.dynamic(direction))
    }

    @get:AutoLogOutput(key = "SwerveStates/Measured")
    private val moduleStates: Array<SwerveModuleState?>
        /** Returns the module states (turn angles and drive velocities) for all of the modules.  */
        get() {
            val states = arrayOfNulls<SwerveModuleState>(4)
            modules.forEachIndexed { index, module ->
                states[index] = module?.state
            }
            return states
        }

    private val modulePositions: Array<SwerveModulePosition?>
        /** Returns the module positions (turn angles and drive positions) for all of the modules.  */
        get() {
            val positions = arrayOfNulls<SwerveModulePosition>(4)
            modules.forEachIndexed { index, module ->
                positions[index] = module?.position
            }
            return positions
        }

    @get:AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private val chassisSpeeds: ChassisSpeeds?
        /** Returns the measured chassis speeds of the robot.  */
        get() {
            // Ensure we have exactly 4 module states for kinematics
            // Use zero state if module is null (shouldn't happen, but defensive programming)
            val states = Array(4) { index ->
                modules[index]?.state ?: SwerveModuleState(0.0, Rotation2d())
            }
            return kinematics.toChassisSpeeds(*states)
        }

    val wheelRadiusCharacterizationPositions: DoubleArray
        /** Returns the position of each module in radians.  */
        get() = modules.mapNotNull { it?.wheelRadiusCharacterizationPosition }.toDoubleArray()

    val fFCharacterizationVelocity: Double
        /** Returns the average velocity of the modules in rotations/sec (Phoenix native units).  */
        get() = modules.mapNotNull { it?.fFCharacterizationVelocity }.average()

    @get:AutoLogOutput(key = "Odometry/Robot")
    var pose: Pose2d?
        /** Returns the current odometry pose.  */
        get() = poseEstimator.estimatedPosition
        /** Resets the current odometry pose.  */
        set(pose) {
            poseEstimator.resetPosition(rawGyroRotation, this.modulePositions, pose)
        }

    val rotation: Rotation2d?
        /** Returns the current odometry rotation.  */
        get() = this.pose?.rotation

    /** Adds a new timestamped vision measurement.  */
    fun addVisionMeasurement(
        visionRobotPoseMeters: Pose2d?,
        timestampSeconds: Double,
        visionMeasurementStdDevs: Matrix<N3, N1>?
    ) {
        poseEstimator.addVisionMeasurement(
            visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs
        )
    }

    val maxLinearSpeedMetersPerSec: Double
        /** Returns the maximum linear speed in meters per sec.  */
        get() = TunerConstants.kSpeedAt12Volts.`in`(Units.MetersPerSecond)

    val maxAngularSpeedRadPerSec: Double
        /** Returns the maximum angular speed in radians per sec.  */
        get() = this.maxLinearSpeedMetersPerSec / DRIVE_BASE_RADIUS

    companion object {
        // TunerConstants doesn't include these constants, so they are declared locally
        val ODOMETRY_FREQUENCY: Double =
            if (TunerConstants.kCANBus.isNetworkFD) 250.0 else 100.0
        
        val DRIVE_BASE_RADIUS: Double = run {
            val frontLeft = TunerConstants.FrontLeft
            val frontRight = TunerConstants.FrontRight
            val backLeft = TunerConstants.BackLeft
            val backRight = TunerConstants.BackRight
            
            max(
                max(
                    hypot(frontLeft.LocationX, frontLeft.LocationY),
                    hypot(frontRight.LocationX, frontRight.LocationY)
                ),
                max(
                    hypot(backLeft.LocationX, backLeft.LocationY),
                    hypot(backRight.LocationX, backRight.LocationY)
                )
            )
        }

        val odometryLock: Lock = ReentrantLock()
        val moduleTranslations: Array<Translation2d>
            /** Returns an array of module translations.  */
            get() = run {
                val fl = TunerConstants.FrontLeft
                val fr = TunerConstants.FrontRight
                val bl = TunerConstants.BackLeft
                val br = TunerConstants.BackRight
                
                arrayOf(
                    Translation2d(fl.LocationX, fl.LocationY),
                    Translation2d(fr.LocationX, fr.LocationY),
                    Translation2d(bl.LocationX, bl.LocationY),
                    Translation2d(br.LocationX, br.LocationY)
                )
            }
    }
}
