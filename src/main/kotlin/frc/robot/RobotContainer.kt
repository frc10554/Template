package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.DriveCommands
import frc.robot.generated.TunerConstants
import frc.robot.subsystems.drive.*
import frc.robot.subsystems.vision.*
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    // Subsystems
    private val drive: Drive
    private val vision: Vision

    // Controller
    private val controller = CommandXboxController(0)

    // Dashboard inputs
    private val sysIdChooser: LoggedDashboardChooser<String>
    private val autos: Autos

    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        when (Constants.currentMode) {
            Constants.Mode.REAL -> {
                // Real robot, instantiate hardware IO implementations
                val fl = TunerConstants.FrontLeft
                val fr = TunerConstants.FrontRight
                val bl = TunerConstants.BackLeft
                val br = TunerConstants.BackRight
                
                drive = Drive(
                    GyroIOPigeon2(),
                    ModuleIOTalonFX(fl),
                    ModuleIOTalonFX(fr),
                    ModuleIOTalonFX(bl),
                    ModuleIOTalonFX(br)
                )
                vision = Vision(
                    { pose, timestamp, stdDevs -> drive.addVisionMeasurement(pose, timestamp, stdDevs) },
                    VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0)
                )
            }

            Constants.Mode.SIM -> {
                // Sim robot, instantiate physics sim IO implementations
                val fl = TunerConstants.FrontLeft
                val fr = TunerConstants.FrontRight
                val bl = TunerConstants.BackLeft
                val br = TunerConstants.BackRight
                
                drive = Drive(
                    object : GyroIO {},
                    ModuleIOSim(fl),
                    ModuleIOSim(fr),
                    ModuleIOSim(bl),
                    ModuleIOSim(br)
                )
                vision = Vision(
                    { pose, timestamp, stdDevs -> drive.addVisionMeasurement(pose, timestamp, stdDevs) },
                    VisionIOPhotonVisionSim(
                        VisionConstants.camera0Name,
                        VisionConstants.robotToCamera0
                    ) { drive.pose ?: Pose2d() }
                )
            }

            else -> {
                // Replayed robot, disable IO implementations
                drive = Drive(
                    object : GyroIO {},
                    object : ModuleIO {},
                    object : ModuleIO {},
                    object : ModuleIO {},
                    object : ModuleIO {}
                )
                vision = Vision(
                    { pose, timestamp, stdDevs -> drive.addVisionMeasurement(pose, timestamp, stdDevs) },
                    object : VisionIO {
                        override fun updateInputs(inputs: VisionIOInputs) {
                            // No-op for replay
                        }
                    }
                )
            }
        }

        // Register Named Commands
        registerNamedCommands()

        // Set up SysId routines
        sysIdChooser = LoggedDashboardChooser<String>("SysId Routines")
        sysIdChooser.addDefaultOption(
            "Drive Wheel Radius Characterization", "Drive Wheel Radius Characterization"
        )
        sysIdChooser.addOption(
            "Drive Simple FF Characterization", "Drive Simple FF Characterization"
        )
        sysIdChooser.addOption(
            "Drive SysId (Quasistatic Forward)", "Drive SysId (Quasistatic Forward)"
        )
        sysIdChooser.addOption(
            "Drive SysId (Quasistatic Reverse)", "Drive SysId (Quasistatic Reverse)"
        )
        sysIdChooser.addOption(
            "Drive SysId (Dynamic Forward)", "Drive SysId (Dynamic Forward)"
        )
        sysIdChooser.addOption(
            "Drive SysId (Dynamic Reverse)", "Drive SysId (Dynamic Reverse)"
        )

        // Set up auto routines
        autos = Autos(drive, sysIdChooser)

        // Put Choreo's auto chooser on the dashboard
        SmartDashboard.putData("Choreo Autos", autos.autoChooser)
        
        // Configure the button bindings
        configureButtonBindings()
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a [edu.wpi.first.wpilibj.GenericHID] or one of its subclasses ([ ] or [edu.wpi.first.wpilibj.XboxController]), and then passing it to a [ ].
     */
    private fun configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.defaultCommand = DriveCommands.joystickDrive(
            drive,
            { -controller.leftY },
            { -controller.leftX },
            { -controller.rightX })

        // Lock to 0° when A button is held
        controller
            .a()
            .whileTrue(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    { -controller.leftY },
                    { -controller.leftX },
                    { Rotation2d() })
            )

        // Switch to X pattern when X button is pressed
        controller.x().onTrue(Commands.runOnce({ drive.stopWithX() }, drive))

        // Reset gyro to 0° when B button is pressed
        controller
            .b()
            .onTrue(
                Commands.runOnce(
                    {
                        drive.pose?.let {
                            drive.pose = Pose2d(it.translation, Rotation2d())
                        }
                    },
                    drive
                )
                    .ignoringDisable(true)
            )
    }
    
    private fun registerNamedCommands() {
    }

    val autonomousCommand: Command?
        /**
         * Use this to pass the autonomous command to the main [Robot] class.
         *
         * @return the command to run in autonomous
         */
        get() = autos.autoChooser.selectedCommandScheduler()
}
