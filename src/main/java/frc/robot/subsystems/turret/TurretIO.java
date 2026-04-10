package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public boolean hasValidCRT = false;

        public MutAngle goalPositionRadians = Degrees.mutable(0);
        public MutAngle positionRadians = Degrees.mutable(0);
        public MutAngle crtPositionRadians = Degrees.mutable(0);
        public MutAngularVelocity velocityRadPerSec = RPM.mutable(0);
        public MutVoltage voltageVolts = Volts.mutable(0);
        public MutCurrent currentAmps = Amps.mutable(0);

        public double crtPositionErrorRadians = 0.0;
        public long reseedSampleCount = 0;
    }

    void updateInputs(TurretIOInputs inputs);

    void setTarget(Angle angle);

    void setVoltage(double volts);
    void setCurrent(double amps);
}
