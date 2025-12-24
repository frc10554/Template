package frc.robot.subsystems.drive

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.*
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import frc.robot.generated.TunerConstants
import frc.robot.util.PhoenixUtil
import java.util.*

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 *
 * Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
class ModuleIOTalonFX(private val constants: SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>) :
    ModuleIO {
    // Hardware objects
    private val driveTalon: TalonFX = TalonFX(constants.DriveMotorId, TunerConstants.kCANBus)
    private val turnTalon: TalonFX = TalonFX(constants.SteerMotorId, TunerConstants.kCANBus)
    private val cancoder: CANcoder = CANcoder(constants.EncoderId, TunerConstants.kCANBus)

    // Voltage control requests
    private val voltageRequest = VoltageOut(0.0)
    private val positionVoltageRequest = MotionMagicVoltage(0.0)
    private val velocityVoltageRequest = VelocityVoltage(0.0)

    // Torque-current control requests
    private val torqueCurrentRequest = TorqueCurrentFOC(0.0)
    private val positionTorqueCurrentRequest = MotionMagicTorqueCurrentFOC(0.0)
    private val velocityTorqueCurrentRequest = VelocityTorqueCurrentFOC(0.0)

    // Timestamp inputs from Phoenix thread
    private val timestampQueue: Queue<Double?>

    // Inputs from drive motor
    private val drivePosition: StatusSignal<Angle?>
    private val drivePositionQueue: Queue<Double?>
    private val driveVelocity: StatusSignal<AngularVelocity?>
    private val driveAppliedVolts: StatusSignal<Voltage?>
    private val driveCurrent: StatusSignal<Current?>

    // Inputs from turn motor
    private val turnAbsolutePosition: StatusSignal<Angle?>
    private val turnPosition: StatusSignal<Angle?>
    private val turnPositionQueue: Queue<Double?>
    private val turnVelocity: StatusSignal<AngularVelocity?>
    private val turnAppliedVolts: StatusSignal<Voltage?>
    private val turnCurrent: StatusSignal<Current?>

    // Connection debouncers
    private val driveConnectedDebounce = Debouncer(0.5)
    private val turnConnectedDebounce = Debouncer(0.5)
    private val turnEncoderConnectedDebounce = Debouncer(0.5)

    init {

        // Configure drive motor
        val driveConfig = constants.DriveMotorInitialConfigs
            ?: throw IllegalStateException("Drive motor initial configs are null")
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake
        driveConfig.Slot0 = constants.DriveMotorGains
        driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent
        driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true
        driveConfig.MotorOutput.Inverted =
            if (constants.DriveMotorInverted)
                InvertedValue.Clockwise_Positive
            else
                InvertedValue.CounterClockwise_Positive
        PhoenixUtil.tryUntilOk(5) { driveTalon.configurator.apply(driveConfig, 0.25) }
        PhoenixUtil.tryUntilOk(5) { driveTalon.setPosition(0.0, 0.25) }

        // Configure turn motor
        val turnConfig = TalonFXConfiguration()
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake
        turnConfig.Slot0 = constants.SteerMotorGains
        turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId
        turnConfig.Feedback.FeedbackSensorSource =
            when (constants.FeedbackSource) {
                SwerveModuleConstants.SteerFeedbackType.RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder
                SwerveModuleConstants.SteerFeedbackType.FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder
                SwerveModuleConstants.SteerFeedbackType.SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder
                else -> throw RuntimeException(
                    "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers."
                )
            }
        turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio
        turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio
        turnConfig.MotionMagic.MotionMagicAcceleration =
            turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100
        turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio
        turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true
        turnConfig.MotorOutput.Inverted =
            if (constants.SteerMotorInverted)
                InvertedValue.Clockwise_Positive
            else
                InvertedValue.CounterClockwise_Positive
        PhoenixUtil.tryUntilOk(5) { turnTalon.configurator.apply(turnConfig, 0.25) }

        // Configure CANCoder
        val cancoderConfig = constants.EncoderInitialConfigs
            ?: throw IllegalStateException("Encoder initial configs are null")
        cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset
        cancoderConfig.MagnetSensor.SensorDirection =
            if (constants.EncoderInverted)
                SensorDirectionValue.Clockwise_Positive
            else
                SensorDirectionValue.CounterClockwise_Positive
        cancoder.configurator.apply(cancoderConfig)

        // Create timestamp queue
        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue()

        // Create drive status signals
        drivePosition = driveTalon.position
        drivePositionQueue =
            PhoenixOdometryThread.getInstance().registerSignal(driveTalon.position)
        driveVelocity = driveTalon.velocity
        driveAppliedVolts = driveTalon.motorVoltage
        driveCurrent = driveTalon.statorCurrent

        // Create turn status signals
        turnAbsolutePosition = cancoder.absolutePosition
        turnPosition = turnTalon.position
        turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnTalon.position)
        turnVelocity = turnTalon.velocity
        turnAppliedVolts = turnTalon.motorVoltage
        turnCurrent = turnTalon.statorCurrent

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(
            Drive.ODOMETRY_FREQUENCY, drivePosition, turnPosition
        )
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent,
            turnAbsolutePosition,
            turnVelocity,
            turnAppliedVolts,
            turnCurrent
        )
        ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon)
    }

    override fun updateInputs(inputs: ModuleIOInputs?) {
        val inputsNonNull = inputs ?: return
        // Refresh all signals
        val driveStatus =
            BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent)
        val turnStatus =
            BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent)
        val turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition)

        // Update drive inputs
        inputsNonNull.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK)
        inputsNonNull.drivePositionRad = Units.rotationsToRadians(drivePosition.valueAsDouble)
        inputsNonNull.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.valueAsDouble)
        inputsNonNull.driveAppliedVolts = driveAppliedVolts.valueAsDouble
        inputsNonNull.driveCurrentAmps = driveCurrent.valueAsDouble

        // Update turn inputs
        inputsNonNull.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK)
        inputsNonNull.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK)
        inputsNonNull.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.valueAsDouble)
        inputsNonNull.turnPosition = Rotation2d.fromRotations(turnPosition.valueAsDouble)
        inputsNonNull.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.valueAsDouble)
        inputsNonNull.turnAppliedVolts = turnAppliedVolts.valueAsDouble
        inputsNonNull.turnCurrentAmps = turnCurrent.valueAsDouble

        // Update odometry inputs
        inputsNonNull.odometryTimestamps =
            timestampQueue.stream().mapToDouble { value: Double? -> value ?: 0.0 }.toArray()
        inputsNonNull.odometryDrivePositionsRad =
            drivePositionQueue.stream()
                .mapToDouble { value: Double? -> Units.rotationsToRadians(value ?: 0.0) }
                .toArray()
        inputsNonNull.odometryTurnPositions =
            turnPositionQueue.stream()
                .map { value: Double? ->
                    value?.let { Rotation2d.fromRotations(it) }
                }
                .toList()
                .filterNotNull()
                .toTypedArray()
        timestampQueue.clear()
        drivePositionQueue.clear()
        turnPositionQueue.clear()
    }

    override fun setDriveOpenLoop(output: Double) {
        driveTalon.setControl(
            when (constants.DriveMotorClosedLoopOutput) {
                SwerveModuleConstants.ClosedLoopOutputType.Voltage -> voltageRequest.withOutput(output)
                SwerveModuleConstants.ClosedLoopOutputType.TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output)
                else -> voltageRequest.withOutput(output)
            }
        )
    }

    override fun setTurnOpenLoop(output: Double) {
        turnTalon.setControl(
            when (constants.SteerMotorClosedLoopOutput) {
                SwerveModuleConstants.ClosedLoopOutputType.Voltage -> voltageRequest.withOutput(output)
                SwerveModuleConstants.ClosedLoopOutputType.TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output)
                else -> voltageRequest.withOutput(output)
            }
        )
    }

    override fun setDriveVelocity(velocityRadPerSec: Double) {
        val velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec)
        driveTalon.setControl(
            when (constants.DriveMotorClosedLoopOutput) {
                SwerveModuleConstants.ClosedLoopOutputType.Voltage -> velocityVoltageRequest.withVelocity(
                    velocityRotPerSec
                )

                SwerveModuleConstants.ClosedLoopOutputType.TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(
                    velocityRotPerSec
                )
                else -> velocityVoltageRequest.withVelocity(velocityRotPerSec)
            }
        )
    }

    override fun setTurnPosition(rotation: Rotation2d?) {
        rotation?.let {
            turnTalon.setControl(
                when (constants.SteerMotorClosedLoopOutput) {
                    SwerveModuleConstants.ClosedLoopOutputType.Voltage -> positionVoltageRequest.withPosition(it.rotations)
                    SwerveModuleConstants.ClosedLoopOutputType.TorqueCurrentFOC -> positionTorqueCurrentRequest.withPosition(
                        it.rotations
                    )
                    else -> positionVoltageRequest.withPosition(it.rotations)
                }
            )
        }
    }
}
