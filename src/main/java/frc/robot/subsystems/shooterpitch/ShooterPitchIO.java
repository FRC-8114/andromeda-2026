package frc.robot.subsystems.shooterpitch;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
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

public interface ShooterPitchIO {
    @AutoLog
    public static class ShooterPitchInputs {
        public MutAngle targetAngle            = Rotations.mutable(0);
        public MutAngle position               = Rotations.mutable(0);
        public MutAngularVelocity velocity     = RadiansPerSecond.mutable(0);
        public MutVoltage appliedVoltage       = Volts.mutable(0);
        public MutCurrent appliedCurrent       = Amps.mutable(0);
    }

    void setTarget(Angle angle);
    void setVoltage(Voltage volts);
    void runCurrent(Current current);

    default void setHomingVoltage(Voltage volts) {
        setVoltage(volts);
    }

    default void stop() {
        setVoltage(Volts.of(0));
    }

    default boolean supportsHomingReseed() {
        return false;
    }

    default void reseedPosition(Angle angle) {
    }

    void updateInputs(ShooterPitchInputs inputs);
}
