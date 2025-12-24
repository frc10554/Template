package frc.robot

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d

object VisionConstants {
    // AprilTag layout
    val aprilTagLayout: AprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)

    // Camera names, must match names configured on coprocessor
    const val camera0Name = "camera_0"
    const val camera1Name = "camera_1"

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    val robotToCamera0 = Transform3d(0.2, 0.0, 0.2, Rotation3d(0.0, -0.4, 0.0))
    val robotToCamera1 = Transform3d(-0.2, 0.0, 0.2, Rotation3d(0.0, -0.4, Math.PI))

    // Basic filtering thresholds
    const val maxAmbiguity = 0.3
    const val maxZError = 0.75

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    const val linearStdDevBaseline = 0.02 // Meters
    const val angularStdDevBaseline = 0.06 // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    val cameraStdDevFactors = doubleArrayOf(
        1.0, // Camera 0
        1.0 // Camera 1
    )

    // Multipliers to apply for MegaTag 2 observations
    const val linearStdDevMegatag2Factor = 0.5 // More stable than full 3D solve
    const val angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY // No rotation data available
}
