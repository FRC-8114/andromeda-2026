package frc.robot.subsystems.shooterflywheels;

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

public interface ShooterFlywheelsIO {
    @AutoLog
    public static class ShooterFlywheelsInputs {
        public MutAngularVelocity targetVelocity      = RadiansPerSecond.mutable(0);

        public MutAngularVelocity leftVelocity        = RadiansPerSecond.mutable(0);
        public MutCurrent leftCurrent                 = Amps.mutable(0);
        public MutVoltage leftAppliedVoltage          = Volts.mutable(0);
        public MutAngle leftPosition                  = Rotations.mutable(0);

        public MutAngularVelocity rightVelocity       = RadiansPerSecond.mutable(0);
        public MutCurrent rightCurrent                = Amps.mutable(0);
        public MutVoltage rightAppliedVoltage         = Volts.mutable(0);
        public MutAngle rightPosition                 = Rotations.mutable(0);
    }

    void setFlywheelVelocity(AngularVelocity velocity);
    void runVolts(Voltage volts);
    void runCurrent(Current current);
    void stop();

    void updateInputs(ShooterFlywheelsInputs inputs);
}
