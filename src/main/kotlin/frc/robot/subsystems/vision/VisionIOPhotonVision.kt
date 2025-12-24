package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import frc.robot.VisionConstants
import org.photonvision.PhotonCamera
import java.util.LinkedList

/** IO implementation for real PhotonVision hardware. */
open class VisionIOPhotonVision(
    name: String,
    protected val robotToCamera: Transform3d
) : VisionIO {
    protected val camera: PhotonCamera = PhotonCamera(name)

    override fun updateInputs(inputs: VisionIOInputs) {
        inputs.connected = camera.isConnected

        // Read new camera observations
        val tagIds = HashSet<Int>()
        val poseObservations = LinkedList<PoseObservation>()
        
        for (result in camera.allUnreadResults) {
            // Update latest target observation
            if (result.hasTargets()) {
                inputs.latestTargetObservation = TargetObservation(
                    Rotation2d.fromDegrees(result.bestTarget.yaw),
                    Rotation2d.fromDegrees(result.bestTarget.pitch)
                )
            } else {
                inputs.latestTargetObservation = TargetObservation(Rotation2d(), Rotation2d())
            }

            // Add pose observation
            if (result.multitagResult.isPresent) { // Multitag result
                val multitagResult = result.multitagResult.get()

                // Calculate robot pose
                val fieldToCamera = multitagResult.estimatedPose.best
                val fieldToRobot = fieldToCamera.plus(robotToCamera.inverse())
                val robotPose = Pose3d(fieldToRobot.translation, fieldToRobot.rotation)

                // Calculate average tag distance
                var totalTagDistance = 0.0
                for (target in result.targets) {
                    totalTagDistance += target.bestCameraToTarget.translation.norm
                }

                // Add tag IDs
                for (id in multitagResult.fiducialIDsUsed) {
                    tagIds.add(id.toInt())
                }

                // Add observation
                poseObservations.add(
                    PoseObservation(
                        result.timestampSeconds, // Timestamp
                        robotPose, // 3D pose estimate
                        multitagResult.estimatedPose.ambiguity, // Ambiguity
                        multitagResult.fiducialIDsUsed.size, // Tag count
                        totalTagDistance / result.targets.size, // Average tag distance
                        PoseObservationType.PHOTON_MULTI_TAG
                    )
                )
            } else if (result.targets.isNotEmpty()) { // Single tag result
                val target = result.targets[0]

                // Calculate robot pose
                val tagPose = VisionConstants.aprilTagLayout.getTagPose(target.fiducialId)
                if (tagPose.isPresent) {
                    val fieldToTarget = Transform3d(tagPose.get().translation, tagPose.get().rotation)
                    val cameraToTarget = target.bestCameraToTarget
                    val fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse())
                    val fieldToRobot = fieldToCamera.plus(robotToCamera.inverse())
                    val robotPose = Pose3d(fieldToRobot.translation, fieldToRobot.rotation)

                    // Add tag ID
                    tagIds.add(target.fiducialId)

                    // Add observation
                    poseObservations.add(
                        PoseObservation(
                            result.timestampSeconds, // Timestamp
                            robotPose, // 3D pose estimate
                            target.poseAmbiguity, // Ambiguity
                            1, // Tag count
                            cameraToTarget.translation.norm, // Average tag distance
                            PoseObservationType.PHOTON_SINGLE_TAG
                        )
                    )
                }
            }
        }

        // Save pose observations to inputs object
        inputs.poseObservations = poseObservations.toTypedArray()

        // Save tag IDs to inputs objects
        inputs.tagIds = tagIds.toIntArray()
    }
}
