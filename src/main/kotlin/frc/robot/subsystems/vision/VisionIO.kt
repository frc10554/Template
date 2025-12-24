package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog

interface VisionIO {
    fun updateInputs(inputs: VisionIOInputs) {}
}

@AutoLog
open class VisionIOInputs {
    @JvmField var connected: Boolean = false
    @JvmField var latestTargetObservation: TargetObservation = TargetObservation(Rotation2d(), Rotation2d())
    @JvmField var poseObservations: Array<PoseObservation> = emptyArray()
    @JvmField var tagIds: IntArray = IntArray(0)
}
