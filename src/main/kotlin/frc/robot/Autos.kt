package frc.robot

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.trajectory.SwerveSample
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.commands.DriveCommands
import frc.robot.subsystems.drive.Drive
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

class Autos(private val drive: Drive, private val sysIdChooser: LoggedDashboardChooser<String>) {
    private val autoFactory: AutoFactory = AutoFactory(
        { drive.pose ?: Pose2d() },
        { pose -> drive.pose = pose },
        { sample ->
            if (sample is SwerveSample) {
                drive.followTrajectory(sample)

                Logger.recordOutput("Odometry/Target Pose",
                    Pose2d(sample.x, sample.y, Rotation2d(sample.heading)))

                Logger.recordOutput("Odometry/Target Speeds", sample.chassisSpeeds)
            }
        },
        true,
        drive
    )
    val autoChooser: AutoChooser = AutoChooser()

    init {
        // Register autos
        autoChooser.addRoutine("Test Auto", ::testAuto)
        autoChooser.addRoutine("System Identification", ::sysIdAuto)
    }

    fun testAuto(): AutoRoutine {
        val routine = autoFactory.newRoutine("Test Auto")
        
        // Example of simple trajectory
        val traj = routine.trajectory("New Path")
        routine.active().onTrue(traj.resetOdometry().andThen(traj.cmd()).andThen(DriveCommands.joystickDriveAtAngle(drive, { 0.0 }, { 0.0 }, { Rotation2d() })))
        
        return routine
    }

    private fun sysIdAuto(): AutoRoutine {
        val routine = autoFactory.newRoutine("System Identification")
        routine.active().onTrue(
            Commands.defer({
                when (sysIdChooser.get()) {
                    "Drive Wheel Radius Characterization" -> DriveCommands.wheelRadiusCharacterization(drive)
                    "Drive Simple FF Characterization" -> DriveCommands.feedforwardCharacterization(drive)
                    "Drive SysId (Quasistatic Forward)" -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
                    "Drive SysId (Quasistatic Reverse)" -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
                    "Drive SysId (Dynamic Forward)" -> drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
                    "Drive SysId (Dynamic Reverse)" -> drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
                    else -> Commands.none()
                }
            }, setOf(drive))
        )
        return routine
    }
}
