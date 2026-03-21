package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double rotations = 0.0;
        public double velocityRPM = 0.0;
        public double currentAmps = 0.0;
        public double appliedVoltageVolts = 0.0;
    }

    void runVolts(Voltage volts);

    void setPosition(double absoluteDrumRotations);

    void updateInputs(ClimberIOInputs inputs);
}
