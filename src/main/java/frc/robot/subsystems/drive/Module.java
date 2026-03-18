package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private final Alert turnEncoderDisconnectedAlert;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    public Module(
            ModuleIO io,
            int index,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.io = io;
        this.index = index;
        this.constants = constants;
        driveDisconnectedAlert = new Alert(
                "Disconnected drive motor on module " + Integer.toString(index) + ".",
                AlertType.kError);
        turnDisconnectedAlert = new Alert(
                "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
        turnEncoderDisconnectedAlert = new Alert(
                "Disconnected turn encoder on module " + Integer.toString(index) + ".",
                AlertType.kError);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);
        turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
    }

    /**
     * Runs the module with the specified setpoint state. Mutates the state to
     * optimize it.
     */
    public void runSetpoint(SwerveModuleState state) {
        // Optimize velocity setpoint
        state.optimize(getAngle());
        state.cosineScale(getAngle());

        // Apply setpoints
        io.setDriveVelocity(RadiansPerSecond.of(state.speedMetersPerSecond / constants.WheelRadius));
        io.setTurnPosition(state.angle);
    }

    /**
     * Runs the module with the specified output while controlling to zero degrees.
     */
    public void runCharacterization(double output) {
        io.setDriveCharacterizationVoltage(Volts.of(output));
        io.setTurnPosition(Rotation2d.kZero);
    }

    public void runCharacterization(double driveOutput, Rotation2d turnAngle) {
        io.setDriveCharacterizationVoltage(Volts.of(driveOutput));
        io.setTurnPosition(turnAngle);
    }

    public void runSteerCharacterization(double output) {
        io.setDriveOpenLoop(0.0);
        io.setTurnCharacterizationVoltage(Volts.of(output));
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setDriveOpenLoop(0.0);
        io.setTurnOpenLoop(0.0);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(inputs.turnPosition.in(Radians));
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters() {
        return inputs.drivePosition.in(Radians) * constants.WheelRadius;
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocity.in(RadiansPerSecond) * constants.WheelRadius;
    }

    /** Returns the drive motor voltage. */
    public double getDriveAppliedVolts() {
        return inputs.driveAppliedVoltage.in(Volts);
    }

    /** Returns the drive motor current in amps. */
    public double getDriveCurrentAmps() {
        return inputs.driveCurrent.in(Amps);
    }

    /** Returns the current turn position in radians without wrapping. */
    public double getTurnPositionRad() {
        return inputs.turnPosition.in(Radians);
    }

    /** Returns the turn motor velocity in radians per second. */
    public double getTurnVelocityRadPerSec() {
        return inputs.turnVelocity.in(RadiansPerSecond);
    }

    /** Returns the turn motor voltage. */
    public double getTurnAppliedVolts() {
        return inputs.turnAppliedVoltage.in(Volts);
    }

    /** Returns the turn motor current in amps. */
    public double getTurnCurrentAmps() {
        return inputs.turnCurrent.in(Amps);
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /** Returns the timestamps of the samples received this cycle. */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }
}
