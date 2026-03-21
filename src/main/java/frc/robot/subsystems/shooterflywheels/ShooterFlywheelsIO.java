package frc.robot.subsystems.shooterflywheels;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface ShooterFlywheelsIO {
    @AutoLog
    public static class ShooterInputs {
        public double leftFlywheelRPMs = 0;
        public double rightFlywheelRPMs = 0;
        public double leftCurrentAmps = 0;
        public double rightCurrentAmps = 0;
        public double leftAppliedVoltage = 0;
        public double rightAppliedVoltage = 0;
        public double leftPositionRads = 0;
        public double rightPositionRads = 0;
    }

    void setFlywheelVelocity(AngularVelocity velocity);

    void runVolts(Voltage volts);

    void stopFlywheels();

    void updateInputs(ShooterInputs inputs);
}
