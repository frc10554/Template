package frc.robot.subsystems.drive

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import frc.robot.generated.TunerConstants
import java.util.*

/** IO implementation for Pigeon 2.  */
class GyroIOPigeon2 : GyroIO {
    private val pigeon = Pigeon2(
        TunerConstants.DrivetrainConstants.Pigeon2Id,
        TunerConstants.kCANBus
    )
    private val yaw: StatusSignal<Angle?> = pigeon.yaw
    private val yawPositionQueue: Queue<Double?>
    private val yawTimestampQueue: Queue<Double?>
    private val yawVelocity: StatusSignal<AngularVelocity?> = pigeon.angularVelocityZWorld

    init {
        pigeon.configurator.apply(Pigeon2Configuration())
        pigeon.configurator.setYaw(0.0)
        yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY)
        yawVelocity.setUpdateFrequency(50.0)
        pigeon.optimizeBusUtilization()
        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue()
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.yaw)
    }

    override fun updateInputs(inputs: GyroIOInputs?) {
        val inputsNonNull = inputs ?: return
        inputsNonNull.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity) == StatusCode.OK
        inputsNonNull.yawPosition = Rotation2d.fromDegrees(yaw.valueAsDouble)
        inputsNonNull.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.valueAsDouble)

        inputsNonNull.odometryYawTimestamps =
            yawTimestampQueue.stream().mapToDouble { value: Double? -> value ?: 0.0 }.toArray()
        inputsNonNull.odometryYawPositions =
            yawPositionQueue.stream()
                .map { value: Double? ->
                    value?.let { Rotation2d.fromDegrees(it) }
                }
                .toList()
                .filterNotNull()
                .toTypedArray()
        yawTimestampQueue.clear()
        yawPositionQueue.clear()
    }
}
