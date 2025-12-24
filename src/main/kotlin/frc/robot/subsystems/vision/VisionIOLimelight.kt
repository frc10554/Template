package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.DoubleArrayPublisher
import edu.wpi.first.networktables.DoubleArraySubscriber
import edu.wpi.first.networktables.DoubleSubscriber
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.RobotController
import java.util.LinkedList
import java.util.function.Supplier

/** IO implementation for real Limelight hardware. */
class VisionIOLimelight(
    name: String,
    private val rotationSupplier: Supplier<Rotation2d>
) : VisionIO {
    private val orientationPublisher: DoubleArrayPublisher
    private val latencySubscriber: DoubleSubscriber
    private val txSubscriber: DoubleSubscriber
    private val tySubscriber: DoubleSubscriber
    private val megatag1Subscriber: DoubleArraySubscriber
    private val megatag2Subscriber: DoubleArraySubscriber

    init {
        val table = NetworkTableInstance.getDefault().getTable(name)
        orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish()
        latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0)
        txSubscriber = table.getDoubleTopic("tx").subscribe(0.0)
        tySubscriber = table.getDoubleTopic("ty").subscribe(0.0)
        megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(doubleArrayOf())
        megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(doubleArrayOf())
    }

    override fun updateInputs(inputs: VisionIOInputs) {
        // Update connection status based on whether an update has been seen in the last 250ms
        inputs.connected = ((RobotController.getFPGATime() - latencySubscriber.lastChange) / 1000) < 250

        // Update target observation
        inputs.latestTargetObservation = TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()),
            Rotation2d.fromDegrees(tySubscriber.get())
        )

        // Update orientation for MegaTag 2
        orientationPublisher.accept(
            doubleArrayOf(rotationSupplier.get().degrees, 0.0, 0.0, 0.0, 0.0, 0.0)
        )
        NetworkTableInstance.getDefault().flush() // Increases network traffic but recommended by Limelight

        // Read new pose observations from NetworkTables
        val tagIds = HashSet<Int>()
        val poseObservations = LinkedList<PoseObservation>()

        for (rawSample in megatag1Subscriber.readQueue()) {
            if (rawSample.value.isEmpty()) continue
            for (i in 11 until rawSample.value.size step 7) {
                tagIds.add(rawSample.value[i].toInt())
            }
            poseObservations.add(
                PoseObservation(
                    // Timestamp, based on server timestamp of publish and latency
                    rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
                    // 3D pose estimate
                    parsePose(rawSample.value),
                    // Ambiguity, using only the first tag because ambiguity isn't applicable for multitag
                    if (rawSample.value.size >= 18) rawSample.value[17] else 0.0,
                    // Tag count
                    rawSample.value[7].toInt(),
                    // Average tag distance
                    rawSample.value[9],
                    // Observation type
                    PoseObservationType.MEGATAG_1
                )
            )
        }

        for (rawSample in megatag2Subscriber.readQueue()) {
            if (rawSample.value.isEmpty()) continue
            for (i in 11 until rawSample.value.size step 7) {
                tagIds.add(rawSample.value[i].toInt())
            }
            poseObservations.add(
                PoseObservation(
                    // Timestamp, based on server timestamp of publish and latency
                    rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
                    // 3D pose estimate
                    parsePose(rawSample.value),
                    // Ambiguity, zeroed because the pose is already disambiguated
                    0.0,
                    // Tag count
                    rawSample.value[7].toInt(),
                    // Average tag distance
                    rawSample.value[9],
                    // Observation type
                    PoseObservationType.MEGATAG_2
                )
            )
        }

        // Save pose observations to inputs object
        inputs.poseObservations = poseObservations.toTypedArray()

        // Save tag IDs to inputs objects
        inputs.tagIds = tagIds.toIntArray()
    }

    /** Parses the 3D pose from a Limelight botpose array. */
    private fun parsePose(rawLLArray: DoubleArray): Pose3d {
        return Pose3d(
            rawLLArray[0],
            rawLLArray[1],
            rawLLArray[2],
            Rotation3d(
                Units.degreesToRadians(rawLLArray[3]),
                Units.degreesToRadians(rawLLArray[4]),
                Units.degreesToRadians(rawLLArray[5])
            )
        )
    }
}
