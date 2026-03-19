package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public boolean hasValidCRT = false;

        public Angle goalPosition = Degrees.of(0); 
        public Angle currentTurretPosition = Degrees.of(0);
        public Angle crtTurretPosition = Degrees.of(0);
        public AngularVelocity turretVelocity = RPM.of(0);
        public Voltage appliedVoltage = Volts.of(0);
        public Current appliedCurrent = Amps.of(0);

        public long motorPositionErrorCounter = 0;
    }

    void updateInputs(TurretIOInputs inputs);

    void setTarget(Angle angle);

    void setVoltage(double volts);
    void setCurrent(double amps);
}
