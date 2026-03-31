package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollers extends SubsystemBase {
    private static class Constants {
        static final Voltage intakeVoltage = Volts.of(7.3);
    }

    private final IntakeRollersIO io;
    private final IntakeRollersInputsAutoLogged inputs = new IntakeRollersInputsAutoLogged();

    public IntakeRollers(IntakeRollersIO io) {
        this.io = io;
    }

    public Command intakeForever() {
        return run(() -> io.runVolts(Constants.intakeVoltage));
    }
    public Command stopIntake() {
        return runOnce(() -> io.stopRollers());
    }

    public Command intake() {
        return runEnd(
            () -> io.runVolts(Constants.intakeVoltage),
            () -> io.runVolts(Volts.of(0.0))
        )
            .withName("Intake Rollers");
    }

    public AngularVelocity getVelocity() {
        return RPM.of(inputs.velocityRPM);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakeRollers", inputs);
    }
}
