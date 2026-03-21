package frc.robot.subsystems.intakerollers;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;

public interface IntakeRollersIO {
    @AutoLog
    public static class IntakeRollersInputs {
        public double velocityRPM = 0;
        public double appliedVoltageVolts = 0;
        public double rollerPositionRads = 0;
    }

    void runVolts(Voltage volts);

    void stopRollers();

    void updateInputs(IntakeRollersInputs inputs);
}
