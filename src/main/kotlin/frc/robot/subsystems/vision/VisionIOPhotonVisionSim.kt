package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Transform3d
import frc.robot.VisionConstants
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim
import java.util.function.Supplier

/** IO implementation for physics sim using PhotonVision simulator. */
class VisionIOPhotonVisionSim(
    name: String,
    robotToCamera: Transform3d,
    private val poseSupplier: Supplier<Pose2d>
) : VisionIOPhotonVision(name, robotToCamera) {
    
    private val cameraSim: PhotonCameraSim

    companion object {
        private var visionSim: VisionSystemSim? = null
    }

    init {
        // Initialize vision sim
        if (visionSim == null) {
            visionSim = VisionSystemSim("main")
            visionSim!!.addAprilTags(VisionConstants.aprilTagLayout)
        }

        // Add sim camera
        val cameraProperties = SimCameraProperties()
        cameraSim = PhotonCameraSim(camera, cameraProperties)
        
        visionSim!!.addCamera(cameraSim, robotToCamera)
    }

    override fun updateInputs(inputs: VisionIOInputs) {
        visionSim!!.update(poseSupplier.get())
        super.updateInputs(inputs)
    }
}
