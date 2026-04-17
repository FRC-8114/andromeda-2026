package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public MutAngle drumPosition = Rotations.mutable(0);
        public MutAngularVelocity drumVelocity = RPM.mutable(0);
        public MutCurrent appliedCurrent = Amps.mutable(0);
        public MutVoltage appliedVoltage = Volts.mutable(0);
    }

    void runVolts(Voltage volts);
    void setPosition(Angle position);

    void updateInputs(ClimberIOInputs inputs);
}
