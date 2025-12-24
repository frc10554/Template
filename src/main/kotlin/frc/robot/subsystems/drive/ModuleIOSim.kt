package frc.robot.subsystems.drive

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import kotlin.math.abs
import kotlin.math.sign

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
class ModuleIOSim(constants: SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>) :
    ModuleIO {
    // Create drive and turn sim models
    private val driveSim: DCMotorSim = DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DRIVE_GEARBOX, constants.DriveInertia, constants.DriveMotorGearRatio
        ),
        DRIVE_GEARBOX
    )
    private val turnSim: DCMotorSim = DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            TURN_GEARBOX, constants.SteerInertia, constants.SteerMotorGearRatio
        ),
        TURN_GEARBOX
    )

    private var driveClosedLoop = false
    private var turnClosedLoop = false
    private val driveController = PIDController(DRIVE_KP, 0.0, DRIVE_KD)
    private val turnController = with(PIDController(TURN_KP, 0.0, TURN_KD)){
        enableContinuousInput(-Math.PI, Math.PI)
        this
    }
    private var driveFFVolts = 0.0
    private var driveAppliedVolts = 0.0
    private var turnAppliedVolts = 0.0

    override fun updateInputs(inputs: ModuleIOInputs?) {
        val inputsNonNull = inputs!!
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts =
                driveFFVolts + driveController.calculate(driveSim.angularVelocityRadPerSec)
        } else {
            driveController.reset()
        }
        if (turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(turnSim.angularPositionRad)
        } else {
            turnController.reset()
        }

        // Update simulation state
        driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0))
        turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0))
        driveSim.update(0.02)
        turnSim.update(0.02)

        // Update drive inputs
        inputsNonNull.driveConnected = true
        inputsNonNull.drivePositionRad = driveSim.angularPositionRad
        inputsNonNull.driveVelocityRadPerSec = driveSim.angularVelocityRadPerSec
        inputsNonNull.driveAppliedVolts = driveAppliedVolts
        inputsNonNull.driveCurrentAmps = abs(driveSim.currentDrawAmps)

        // Update turn inputs
        inputsNonNull.turnConnected = true
        inputsNonNull.turnEncoderConnected = true
        inputsNonNull.turnAbsolutePosition = Rotation2d(turnSim.angularPositionRad)
        inputsNonNull.turnPosition = Rotation2d(turnSim.angularPositionRad)
        inputsNonNull.turnVelocityRadPerSec = turnSim.angularVelocityRadPerSec
        inputsNonNull.turnAppliedVolts = turnAppliedVolts
        inputsNonNull.turnCurrentAmps = abs(turnSim.currentDrawAmps)

        // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
        inputsNonNull.odometryTimestamps = doubleArrayOf(Timer.getTimestamp())
        inputsNonNull.odometryDrivePositionsRad = doubleArrayOf(inputsNonNull.drivePositionRad)
        inputsNonNull.odometryTurnPositions = arrayOf(inputsNonNull.turnPosition)
    }

    override fun setDriveOpenLoop(output: Double) {
        driveClosedLoop = false
        driveAppliedVolts = output
    }

    override fun setTurnOpenLoop(output: Double) {
        turnClosedLoop = false
        turnAppliedVolts = output
    }

    override fun setDriveVelocity(velocityRadPerSec: Double) {
        driveClosedLoop = true
        driveFFVolts = DRIVE_KS * sign(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec
        driveController.setSetpoint(velocityRadPerSec)
    }

    override fun setTurnPosition(rotation: Rotation2d?) {
        rotation?.let {
            turnClosedLoop = true
            turnController.setSetpoint(it.radians)
        }
    }

    companion object {
        // TunerConstants doesn't support separate sim constants, so they are declared locally
        private const val DRIVE_KP = 0.05
        private const val DRIVE_KD = 0.0
        private const val DRIVE_KS = 0.0
        private const val DRIVE_KV_ROT = 0.91035 // Same units as TunerConstants: (volt * secs) / rotation
        private val DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT)
        private const val TURN_KP = 8.0
        private const val TURN_KD = 0.0
        private val DRIVE_GEARBOX: DCMotor = DCMotor.getKrakenX60Foc(1)
        private val TURN_GEARBOX: DCMotor = DCMotor.getKrakenX60Foc(1)
    }
}
