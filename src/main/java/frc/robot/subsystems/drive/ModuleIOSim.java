package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of module IO. The sim models are configured using
 * a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOSim implements ModuleIO {
    // TunerConstants doesn't support separate sim constants, so they are declared
    // locally
    private static final double DRIVE_KP = 0.05;
    private static final double DRIVE_KD = 0.0;
    private static final double DRIVE_KS = 0.0;
    private static final double DRIVE_KV_ROT = 0.91035; // Same units as TunerConstants: (volt * secs) / rotation
    private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
    private static final double TURN_KP = 8.0;
    private static final double TURN_KD = 0.0;
    private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;

    private PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
    private PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;
    private OpenLoopMode driveOpenLoopMode = OpenLoopMode.CLOSED_LOOP;
    private OpenLoopMode turnOpenLoopMode = OpenLoopMode.CLOSED_LOOP;
    private double driveOpenLoopOutput = 0.0;
    private double turnOpenLoopOutput = 0.0;

    public ModuleIOSim(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        // Create drive and turn sim models
        driveSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DRIVE_GEARBOX, constants.DriveInertia, constants.DriveMotorGearRatio),
                DRIVE_GEARBOX);
        turnSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        TURN_GEARBOX, constants.SteerInertia, constants.SteerMotorGearRatio),
                TURN_GEARBOX);

        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void updateInputs(ModuleIOInputs inputs) {
        // Run closed-loop control
        if (driveOpenLoopMode == OpenLoopMode.CLOSED_LOOP) {
            driveAppliedVolts = driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
        } else {
            driveController.reset();
        }
        if (turnOpenLoopMode == OpenLoopMode.CLOSED_LOOP) {
            turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
        } else {
            turnController.reset();
        }

        // Update simulation state
        driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
        turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
        driveSim.update(0.02);
        turnSim.update(0.02);

        // Update drive inputs
        inputs.driveConnected = true;
        inputs.drivePosition = Radians.of(driveSim.getAngularPositionRad());
        inputs.driveVelocity = RadiansPerSecond.of(driveSim.getAngularVelocityRadPerSec());
        inputs.driveAppliedVoltage = Volts.of(driveAppliedVolts);
        inputs.driveCurrent = Amps.of(Math.abs(driveSim.getCurrentDrawAmps()));
        inputs.driveOpenLoopMode = driveOpenLoopMode;
        inputs.driveOpenLoopOutput = driveOpenLoopOutput;

        // Update turn inputs
        inputs.turnConnected = true;
        inputs.turnEncoderConnected = true;
        inputs.turnPosition = Radians.of(turnSim.getAngularPositionRad());
        inputs.turnAbsolutePosition = Rotation2d.fromRadians(inputs.turnPosition.in(Radians));
        inputs.turnVelocity = RadiansPerSecond.of(turnSim.getAngularVelocityRadPerSec());
        inputs.turnAppliedVoltage = Volts.of(turnAppliedVolts);
        inputs.turnCurrent = Amps.of(Math.abs(turnSim.getCurrentDrawAmps()));
        inputs.turnOpenLoopMode = turnOpenLoopMode;
        inputs.turnOpenLoopOutput = turnOpenLoopOutput;

        // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
        // matter)
        inputs.odometryTimestamps = new double[] { Timer.getFPGATimestamp() };
        inputs.odometryDrivePositionsRad = new double[] { inputs.drivePosition.in(Radians) };
        inputs.odometryTurnPositions = new Rotation2d[] { Rotation2d.fromRadians(inputs.turnPosition.in(Radians)) };
    }

    public void setDriveOpenLoop(double output) {
        driveOpenLoopMode = OpenLoopMode.VOLTAGE;
        driveOpenLoopOutput = output;
        driveAppliedVolts = output;
    }

    public void setTurnOpenLoop(double output) {
        turnOpenLoopMode = OpenLoopMode.VOLTAGE;
        turnOpenLoopOutput = output;
        turnAppliedVolts = output;
    }

    public void setDriveVelocity(edu.wpi.first.units.measure.AngularVelocity velocityRadPerSec) {
        driveOpenLoopMode = OpenLoopMode.CLOSED_LOOP;
        driveOpenLoopOutput = 0.0;
        double velocityRadPerSecValue = velocityRadPerSec.in(RadiansPerSecond);
        driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSecValue) + DRIVE_KV * velocityRadPerSecValue;
        driveController.setSetpoint(velocityRadPerSecValue);
    }

    public void setTurnPosition(Rotation2d rotation) {
        turnOpenLoopMode = OpenLoopMode.CLOSED_LOOP;
        turnOpenLoopOutput = 0.0;
        turnController.setSetpoint(rotation.getRadians());
    }
}
