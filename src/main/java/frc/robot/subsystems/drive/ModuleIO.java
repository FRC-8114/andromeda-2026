package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  public static enum OpenLoopMode {
    CLOSED_LOOP,
    VOLTAGE,
    TORQUE_CURRENT
  }

  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;

    public Angle drivePosition = Radians.of(0.0);
    public AngularVelocity driveVelocity = RadiansPerSecond.of(0.0);
    public Voltage driveAppliedVoltage = Volts.of(0.0);
    public Current driveCurrent = Amps.of(0.0);
    public OpenLoopMode driveOpenLoopMode = OpenLoopMode.CLOSED_LOOP;
    public double driveOpenLoopOutput = 0.0;

    public boolean turnConnected = false;
    public boolean turnEncoderConnected = false;

    public Rotation2d turnAbsolutePosition = Rotation2d.kZero;
    public Angle turnPosition = Radians.of(0.0);
    public AngularVelocity turnVelocity = RadiansPerSecond.of(0.0);
    public Voltage turnAppliedVoltage = Volts.of(0.0);
    public Current turnCurrent = Amps.of(0.0);
    public OpenLoopMode turnOpenLoopMode = OpenLoopMode.CLOSED_LOOP;
    public double turnOpenLoopOutput = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(ModuleIOInputs inputs);

  /** Run the drive motor at the specified open loop value. */
  public void setDriveOpenLoop(double output);

  /** Run the drive motor at the specified voltage for characterization. */
  public default void setDriveCharacterizationVoltage(Voltage volts) {
    setDriveOpenLoop(volts.in(Volts));
  }

  /** Run the turn motor at the specified open loop value. */
  public void setTurnOpenLoop(double output);

  /** Run the turn motor at the specified voltage for characterization. */
  public default void setTurnCharacterizationVoltage(Voltage volts) {
    setTurnOpenLoop(volts.in(Volts));
  }

  /** Run the drive motor at the specified velocity. */
  public void setDriveVelocity(AngularVelocity velocityRadPerSec);

  /** Run the turn motor to the specified rotation. */
  public void setTurnPosition(Rotation2d rotation);
}
