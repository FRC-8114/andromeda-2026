package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IntakePivot extends SubsystemBase {
    public static class Constants {
        public static final Angle DEPLOYED_ANGLE = Degrees.of(0);
        public static final Angle HOLD_ANGLE = Rotations.of(0.141);
        public static final Angle STOWED_ANGLE = Rotations.of(0.37);

        private static final Current HOLD_DOWN_FEEDFORWARD = Amps.of(20);
        private static final Angle ANGLE_TOLERANCE = Degrees.of(1);
    }

    private IntakePivotIO io;
    private final IntakePivotInputsAutoLogged inputs = new IntakePivotInputsAutoLogged();

    public IntakePivot(IntakePivotIO io) {
        this.io = io;

        setDefaultCommand(hold());
    }

    public Trigger isAtAngle(Angle ang) {
        return new Trigger(
                () -> inputs.position.isNear(ang, Constants.ANGLE_TOLERANCE));
    }
    public Trigger isDeployed = isAtAngle(Constants.DEPLOYED_ANGLE);
    public Trigger isStowed = isAtAngle(Constants.STOWED_ANGLE);
    public Trigger isAtHold = isAtAngle(Constants.HOLD_ANGLE);

    public Command deploy() {
        return runEnd(
            () -> io.setTargetWithFeedForward(Constants.DEPLOYED_ANGLE,
                Constants.HOLD_DOWN_FEEDFORWARD),
            () -> io.setTarget(Constants.HOLD_ANGLE));
    }

    public Command hold() {
        return run(() -> io.setTarget(Constants.HOLD_ANGLE));
    }

    public Command pump() {
        return Commands.repeatingSequence(
            run(() -> io.setTarget(Rotations.of(0.08))) // low
                .withTimeout(Seconds.of(0.25)),
            run(() -> io.setTarget(Rotations.of(0.20)))
                .withTimeout(Seconds.of(0.25))
        )
            .finallyDo(() -> io.setTarget(Constants.HOLD_ANGLE));
    }

    public Command stow() {
        return run(() -> io.setTarget(Constants.STOWED_ANGLE)).until(isStowed);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakePivot", inputs);
    }
}
