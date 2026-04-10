package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public interface IntakeRollersIO {
    @AutoLog
    public static class IntakeRollersInputs {
        public MutAngularVelocity targetVelocityRadPerSec = RadiansPerSecond.mutable(0);

        public MutAngularVelocity velocityRadPerSec = RadiansPerSecond.mutable(0);
        public MutVoltage appliedVoltageVolts = Volts.mutable(0);
        public MutAngle positionRotations = Rotations.mutable(0);
        public MutCurrent currentAmps = Amps.mutable(0);
    }

    void setTargetVelocity(AngularVelocity velocity);

    void runVolts(Voltage voltage);
    void runCurrent(Current current);

    void stop();

    void updateInputs(IntakeRollersInputs inputs);
}
