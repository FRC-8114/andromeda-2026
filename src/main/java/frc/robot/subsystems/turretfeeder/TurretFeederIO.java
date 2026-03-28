package frc.robot.subsystems.turretfeeder;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface TurretFeederIO {
    @AutoLog
    public static class TurretFeederInputs {
        public double beltPositionDeg = 0;
        public double beltVelocityRPM = 0;
        public double appliedVoltage = 0;
        public double appliedTorqueCurrent = 0;
    }

    void runVolts(Voltage volts);
    void runTorqueCurrent(Current torque);
    void setVelocity(AngularVelocity velocity);
    void runDutyCycle();

    void stopMotor();

    void updateInputs(TurretFeederInputs inputs);
}
