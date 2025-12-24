package frc.robot.subsystems.drive

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog

@AutoLog
open class GyroIOInputs {
    @JvmField
    var connected: Boolean = false
    @JvmField
    var yawPosition: Rotation2d = Rotation2d()
    @JvmField
    var yawVelocityRadPerSec: Double = 0.0
    @JvmField
    var odometryYawTimestamps: DoubleArray = doubleArrayOf()
    @JvmField
    var odometryYawPositions: Array<Rotation2d?> = arrayOf()
}

interface GyroIO {
    fun updateInputs(inputs: GyroIOInputs?) {}
}
