package frc.robot.supersystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakerollers.IntakeRollers;

public class Intake extends SubsystemBase {
    private final IntakePivot pivot;
    private final IntakeRollers rollers;

    private boolean isDefaultStow = false;

    public Intake(IntakePivot pivot, IntakeRollers rollers) {
        this.pivot = pivot;
        this.rollers = rollers;

        setDefaultCommand(stowSwitchCommand());
    }

    private Command stowSwitchCommand() {
        return Commands.either(stow(), hold(), () -> isDefaultStow);
    }

    public Command intake() {
        return Commands.parallel(
                pivot.deploy(),
                rollers.intake());
    }

    public Command hold() {
        return Commands.parallel(
                pivot.hold(),
                rollers.stop());
    }

    public Command deploy() {
        return Commands.parallel(
                pivot.deploy(),
                rollers.stop());
    }

    public Command stow() {
        return Commands.parallel(
                pivot.stow(),
                rollers.stop());
    }

    public Command pump() {
        return Commands.parallel(
                pivot.pump(),
                rollers.stop());
    }
}
