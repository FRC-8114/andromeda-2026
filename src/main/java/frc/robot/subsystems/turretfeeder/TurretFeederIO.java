package frc.robot.subsystems.turretfeeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface TurretFeederIO {
    @AutoLog
    public static class TurretFeederInputs {
        public Angle feederPosition = Degrees.of(0);
        public AngularVelocity feederVelocity = RPM.of(0);
        public Voltage appliedVoltage = Volts.of(0);
        public Current appliedTorqueCurrent = Amps.of(0);
    }

    void runVolts(Voltage volts);
    void runTorqueCurrent(Current torque);
    void setVelocity(AngularVelocity velocity);
    void runDutyCycle();

    void stopMotor();

    void updateInputs(TurretFeederInputs inputs);
}
