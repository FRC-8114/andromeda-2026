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
        private static final Voltage INTAKE_VOLTAGE = Volts.of(7.5);
        private static final AngularVelocity INTAKE_VELOCITY = RPM.of(1500);
    }

    private final IntakeRollersIO io;
    private final IntakeRollersInputsAutoLogged inputs = new IntakeRollersInputsAutoLogged();

    public IntakeRollers(IntakeRollersIO io) {
        this.io = io;
    }

    public Command intake() {
        return startEnd(() -> io.setTargetVelocity(Constants.INTAKE_VELOCITY), () -> io.stop());
    }

    public Command stop() {
        return runOnce(() -> io.stop());
    }

    public AngularVelocity getVelocity() {
        return inputs.velocityRadPerSec.copy();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        
        Logger.processInputs("IntakeRollers", inputs);
    }
}
