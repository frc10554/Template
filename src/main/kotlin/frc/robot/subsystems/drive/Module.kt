package frc.robot.subsystems.drive

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs

class Module(
    private val io: ModuleIO,
    private val index: Int,
    private val constants: SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
) {
    private val inputs = ModuleIOInputsAutoLogged()

    private val driveDisconnectedAlert: Alert = Alert(
        "Disconnected drive motor on module $index.",
        AlertType.kError
    )
    private val turnDisconnectedAlert: Alert = Alert(
        "Disconnected turn motor on module $index.",
        AlertType.kError
    )
    private val turnEncoderDisconnectedAlert: Alert = Alert(
        "Disconnected turn encoder on module $index.",
        AlertType.kError
    )

    /** Returns the module positions received this cycle.  */
    var odometryPositions: Array<SwerveModulePosition?> = arrayOf()
        private set

    fun periodic() {
        io.updateInputs(inputs)
        (inputs as? LoggableInputs)?.let {
            Logger.processInputs("Drive/Module$index", it)
        }

        // Calculate positions for odometry
        val sampleCount = inputs.odometryTimestamps.size // All signals are sampled together
        odometryPositions = arrayOfNulls(sampleCount)
        for (i in 0 until sampleCount) {
            if (i >= inputs.odometryDrivePositionsRad.size) break
            val positionMeters: Double = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius
            val angle: Rotation2d? = inputs.odometryTurnPositions.getOrNull(i)
            if (angle != null) {
                odometryPositions[i] = SwerveModulePosition(positionMeters, angle)
            }
        }

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected)
        turnDisconnectedAlert.set(!inputs.turnConnected)
        turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected)
    }

    /** Runs the module with the specified setpoint state. Mutates the state to optimize it.  */
    fun runSetpoint(state: SwerveModuleState) {
        // Optimize velocity setpoint
        angle.let { state.optimize(it) }
        state.cosineScale(inputs.turnPosition)

        // Apply setpoints
        io.setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius)
        io.setTurnPosition(state.angle)
    }

    /** Runs the module with the specified output while controlling to zero degrees.  */
    fun runCharacterization(output: Double) {
        io.setDriveOpenLoop(output)
        io.setTurnPosition(Rotation2d())
    }

    /** Disables all outputs to motors.  */
    fun stop() {
        io.setDriveOpenLoop(0.0)
        io.setTurnOpenLoop(0.0)
    }

    val angle: Rotation2d
        /** Returns the current turn angle of the module.  */
        get() = inputs.turnPosition

    val positionMeters: Double
        /** Returns the current drive position of the module in meters.  */
        get() = inputs.drivePositionRad * constants.WheelRadius

    val velocityMetersPerSec: Double
        /** Returns the current drive velocity of the module in meters per second.  */
        get() = inputs.driveVelocityRadPerSec * constants.WheelRadius

    val position: SwerveModulePosition
        /** Returns the module position (turn angle and drive position).  */
        get() = SwerveModulePosition(this.positionMeters, this.angle)

    val state: SwerveModuleState
        /** Returns the module state (turn angle and drive velocity).  */
        get() = SwerveModuleState(this.velocityMetersPerSec, this.angle)

    val odometryTimestamps: DoubleArray
        /** Returns the timestamps of the samples received this cycle.  */
        get() = inputs.odometryTimestamps

    val wheelRadiusCharacterizationPosition: Double
        /** Returns the module position in radians.  */
        get() = inputs.drivePositionRad

    val fFCharacterizationVelocity: Double
        /** Returns the module velocity in rotations/sec (Phoenix native units).  */
        get() = Units.radiansToRotations(inputs.driveVelocityRadPerSec)
}
