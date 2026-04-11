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

    private Command withThis(Command command) {
        command.addRequirements(this);

        return command;
    }

    public void toggleStow() {
        isDefaultStow = !isDefaultStow;
    }

    public Command intake() {
        return withThis(Commands.parallel(
                pivot.deploy(),
                rollers.intake()));
    }

    public Command hold() {
        return withThis(Commands.parallel(
                pivot.hold(),
                rollers.stop()));
    }

    public Command deploy() {
        return withThis(Commands.parallel(
                pivot.deploy(),
                rollers.stop()));
    }

    public Command stow() {
        return withThis(Commands.parallel(
                pivot.stow(),
                rollers.stop()));
    }

    public Command pump() {
        return withThis(Commands.parallel(
                pivot.pump(),
                rollers.stop()));
    }
}
