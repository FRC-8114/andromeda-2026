package frc.robot.subsystems.intakepivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public interface IntakePivotIO {
    @AutoLog
    public static class IntakePivotInputs {
        public double positionRads = 0;
        public double velocityRPM = 0;
        public double appliedVoltageVolts = 0;
    }

    void setTarget(Angle angle);
    void holdTargetDown(Angle angle);
    
    void runVolts(Voltage volts);

    void updateInputs(IntakePivotInputs inputs);
}
