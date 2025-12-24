package frc.robot.subsystems.drive

import com.studica.frc.AHRS
import com.studica.frc.AHRS.NavXComType
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import java.util.*

/** IO implementation for NavX.  */
class GyroIONavX : GyroIO {
    private val navX: AHRS = AHRS(NavXComType.kMXP_SPI, Drive.ODOMETRY_FREQUENCY.toInt().toByte().toInt())
    private val yawPositionQueue: Queue<Double?> = PhoenixOdometryThread.getInstance().registerSignal { navX.yaw.toDouble() }
    private val yawTimestampQueue: Queue<Double?> = PhoenixOdometryThread.getInstance().makeTimestampQueue()

    override fun updateInputs(inputs: GyroIOInputs?) {
        val inputsNonNull = inputs ?: return
        inputsNonNull.connected = navX.isConnected
        inputsNonNull.yawPosition = Rotation2d.fromDegrees(-navX.yaw.toDouble())
        inputsNonNull.yawVelocityRadPerSec = Units.degreesToRadians(-navX.rawGyroZ.toDouble())

        inputsNonNull.odometryYawTimestamps =
            yawTimestampQueue.stream().mapToDouble { value: Double? -> value ?: 0.0 }.toArray()
        inputsNonNull.odometryYawPositions =
            yawPositionQueue.stream()
                .map { value: Double? ->
                    value?.let { Rotation2d.fromDegrees(-it) }
                }
                .toList()
                .filterNotNull()
                .toTypedArray()
        yawTimestampQueue.clear()
        yawPositionQueue.clear()
    }
}
