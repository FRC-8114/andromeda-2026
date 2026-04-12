package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.SysIDMechanism;

public class IntakeRollers extends SubsystemBase implements SysIDMechanism {
    private static class Constants {
        static final Voltage INTAKE_VOLTAGE = Volts.of(11);
        // static final AngularVelocity INTAKE_VELOCITY = RPM.of(1700);
    }

    private final IntakeRollersIO io;
    private final IntakeRollersInputsAutoLogged inputs = new IntakeRollersInputsAutoLogged();
    private final SysIdRoutine sysId;

    public IntakeRollers(IntakeRollersIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> Logger.recordOutput("IntakeRollers/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (volts) -> io.runVolts(volts),
                        null, this));
    }

    public Command intake() {
        return startEnd(() -> io.runVolts(Constants.INTAKE_VOLTAGE), io::stop);
    }

    public Command intakeVolts() {
        return startEnd(
            () -> io.runVolts(Constants.INTAKE_VOLTAGE),
            io::stop
        );
    }

    public Command stop() {
        return runOnce(io::stop);
    }

    public AngularVelocity getVelocity() {
        return inputs.velocityRadPerSec;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    @Override
    public List<SysIDMechanism.NamedMechanism> sysIdMechanisms() {
        return List.of(SysIDMechanism.named("Intake Rollers", this::sysIdDynamic, this::sysIdQuasistatic));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakeRollers", inputs);
    }
}
