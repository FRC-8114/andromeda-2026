package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface IntakePivotIO {
    @AutoLog
    public static class IntakePivotInputs {
        public Angle position = Rotations.of(0);
        public AngularVelocity velocity = RPM.of(0);
        public Voltage appliedVoltage = Volts.of(0);
        public Current appliedCurrent = Amps.of(0);
    }

    void setTarget(Angle angle);
    void setTargetWithFeedForward(Angle angle, Current feedforward);
    
    void runVolts(Voltage volts);

    void updateInputs(IntakePivotInputs inputs);
}
