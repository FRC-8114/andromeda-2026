package frc.robot.supersystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakerollers.IntakeRollers;

public class Intake extends SubsystemBase {
    private final IntakePivot pivot;
    private final IntakeRollers rollers;

    public Intake(IntakePivot pivot, IntakeRollers rollers) {
        this.pivot = pivot;
        this.rollers = rollers;
    }

    public Command intake() {
        return Commands.parallel(
            pivot.deploy(),
            rollers.intake()
        )
            .withName("Intake");
    }

    public Command hold() {
        return Commands.parallel(
            pivot.hold(),
            rollers.stopIntake()
        )
            .withName("IntakeHold");
    }

    public Command deploy() {
        return Commands.parallel(
            pivot.deploy(),
            rollers.stopIntake()
        )
            .withName("IntakeDeploy");
    }

    public Command stow() {
        return Commands.parallel(
            pivot.stow(),
            rollers.stopIntake()
        )
            .withName("IntakeStow");
    }

    public Command pump() {
        return Commands.parallel(
            pivot.pump(),
            rollers.stopIntake()
        )
            .withName("IntakePump");
    }
}
