package frc.robot.subsystems.shooterflywheels;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface ShooterFlywheelsIO {
    @AutoLog
    public static class ShooterInputs {
        public AngularVelocity leftFlywheelVelocity = RPM.of(0);
        public AngularVelocity rightFlywheelVelocity = RPM.of(0);

        public Current leftCurrent = Amps.of(0);
        public Current rightCurrent = Amps.of(0);

        public Voltage leftAppliedVoltage = Volts.of(0);
        public Voltage rightAppliedVoltage = Volts.of(0);

        public Angle leftPosition = Radians.of(0);
        public Angle rightPosition = Radians.of(0);
    }

    void setFlywheelVelocity(AngularVelocity velocity);

    void runVolts(Voltage volts);
    void runCurrent(double current);

    void stopFlywheels();

    void updateInputs(ShooterInputs inputs);
}
