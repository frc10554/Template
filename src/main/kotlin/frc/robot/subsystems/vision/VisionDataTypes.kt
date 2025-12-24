package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import org.littletonrobotics.junction.AutoLog
import java.nio.ByteBuffer

@AutoLog
open class VisionSummary {
    @JvmField var tagPoses: Array<Pose3d> = emptyArray()
    @JvmField var robotPoses: Array<Pose3d> = emptyArray()
    @JvmField var robotPosesAccepted: Array<Pose3d> = emptyArray()
    @JvmField var robotPosesRejected: Array<Pose3d> = emptyArray()
}

/** Represents the angle to a simple target, not used for pose estimation.  */
data class TargetObservation(
    val tx: Rotation2d,
    val ty: Rotation2d
) : StructSerializable {
    companion object {
        @JvmField
        val struct = TargetObservationStruct()
    }
}

class TargetObservationStruct : Struct<TargetObservation> {
    override fun getTypeClass(): Class<TargetObservation> = TargetObservation::class.java
    override fun getTypeName(): String = "TargetObservation"
    override fun getSize(): Int = Rotation2d.struct.size * 2
    override fun getSchema(): String = "Rotation2d tx;Rotation2d ty"
    override fun unpack(bb: ByteBuffer): TargetObservation {
        val tx = Rotation2d.struct.unpack(bb)
        val ty = Rotation2d.struct.unpack(bb)
        return TargetObservation(tx, ty)
    }
    override fun pack(bb: ByteBuffer, value: TargetObservation) {
        Rotation2d.struct.pack(bb, value.tx)
        Rotation2d.struct.pack(bb, value.ty)
    }
}

/** Represents a robot pose sample used for pose estimation.  */
data class PoseObservation(
    val timestamp: Double,
    val pose: Pose3d,
    val ambiguity: Double,
    val tagCount: Int,
    val averageTagDistance: Double,
    val type: PoseObservationType
) : StructSerializable {
    companion object {
        @JvmField
        val struct = PoseObservationStruct()
    }
}

class PoseObservationStruct : Struct<PoseObservation> {
    override fun getTypeClass(): Class<PoseObservation> = PoseObservation::class.java
    override fun getTypeName(): String = "PoseObservation"
    override fun getSize(): Int = 8 + Pose3d.struct.size + 8 + 4 + 8 + 1 // timestamp(8) + pose + ambiguity(8) + tagCount(4) + dist(8) + type(1)
    override fun getSchema(): String = "double timestamp;Pose3d pose;double ambiguity;int32 tagCount;double averageTagDistance;uint8 type"
    
    override fun unpack(bb: ByteBuffer): PoseObservation {
        val timestamp = bb.double
        val pose = Pose3d.struct.unpack(bb)
        val ambiguity = bb.double
        val tagCount = bb.int
        val averageTagDistance = bb.double
        val typeIdx = bb.get().toInt()
        val type = PoseObservationType.values().getOrElse(typeIdx) { PoseObservationType.PHOTON_SINGLE_TAG }
        return PoseObservation(timestamp, pose, ambiguity, tagCount, averageTagDistance, type)
    }

    override fun pack(bb: ByteBuffer, value: PoseObservation) {
        bb.putDouble(value.timestamp)
        Pose3d.struct.pack(bb, value.pose)
        bb.putDouble(value.ambiguity)
        bb.putInt(value.tagCount)
        bb.putDouble(value.averageTagDistance)
        bb.put(value.type.ordinal.toByte())
    }
}

enum class PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTON_SINGLE_TAG,
    PHOTON_MULTI_TAG,
}
