package frc.robot.subsystems.hopperlanes;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface HopperLanesIO {
    @AutoLog
    public static class HopperLanesInputs {
        public double velocityRPM = 0;
        public double motorPositionRads = 0;
        public double appliedVoltageVolts = 0;
    }

    void runVolts(Voltage volts);

    void setVelocity(AngularVelocity velocity);

    void stopMotor();

    void updateInputs(HopperLanesInputs inputs);
}
