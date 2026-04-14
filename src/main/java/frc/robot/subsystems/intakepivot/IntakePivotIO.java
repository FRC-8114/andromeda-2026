package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public interface IntakePivotIO {
    @AutoLog
    public static class IntakePivotInputs {
        public MutAngle targetAngleRadians = Rotations.mutable(0);
        public MutAngle positionRadians = Rotations.mutable(0);
        public MutAngularVelocity velocityRadPerSec = RPM.mutable(0);
        public MutVoltage voltageVolts = Volts.mutable(0);
        public MutCurrent currentAmps = Amps.mutable(0);
    }

    void setTarget(Angle angle);

    void setTargetWithFeedForward(Angle angle, Voltage feedforward);

    void runVolts(Voltage volts);
    void runCurrent(Current current);

    void stop();

    void updateInputs(IntakePivotInputs inputs);
}
