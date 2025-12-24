package frc.robot.subsystems.drive

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog

@AutoLog
open class ModuleIOInputs {
    @JvmField
    var driveConnected: Boolean = false
    @JvmField
    var drivePositionRad: Double = 0.0
    @JvmField
    var driveVelocityRadPerSec: Double = 0.0
    @JvmField
    var driveAppliedVolts: Double = 0.0
    @JvmField
    var driveCurrentAmps: Double = 0.0

    @JvmField
    var turnConnected: Boolean = false
    @JvmField
    var turnEncoderConnected: Boolean = false
    @JvmField
    var turnAbsolutePosition: Rotation2d = Rotation2d()
    @JvmField
    var turnPosition: Rotation2d = Rotation2d()
    @JvmField
    var turnVelocityRadPerSec: Double = 0.0
    @JvmField
    var turnAppliedVolts: Double = 0.0
    @JvmField
    var turnCurrentAmps: Double = 0.0

    @JvmField
    var odometryTimestamps: DoubleArray = doubleArrayOf()
    @JvmField
    var odometryDrivePositionsRad: DoubleArray = doubleArrayOf()
    @JvmField
    var odometryTurnPositions: Array<Rotation2d?> = arrayOf()
}

interface ModuleIO {
    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: ModuleIOInputs?) {}

    /** Run the drive motor at the specified open loop value.  */
    fun setDriveOpenLoop(output: Double) {}

    /** Run the turn motor at the specified open loop value.  */
    fun setTurnOpenLoop(output: Double) {}

    /** Run the drive motor at the specified velocity.  */
    fun setDriveVelocity(velocityRadPerSec: Double) {}

    /** Run the turn motor to the specified rotation.  */
    fun setTurnPosition(rotation: Rotation2d?) {}
}
