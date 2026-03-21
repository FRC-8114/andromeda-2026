package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    public static class Constants {
        // drum diameter: 0.787in
        public static final double STOW_ROTATIONS = 0;
        public static final double DEPLOY_ROTATIONS = 3.023;
        public static final double CLIMB_ROTATIONS = 1.511;

        static final double rotationTolerance = 0.03;
        static final double moveTimeoutSecs = 4.0;
    }

    private enum ClimbState {
        STOW, DEPLOY, CLIMB, UNCLIMB
    }
    private ClimbState state = ClimbState.STOW;

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO io) {
        this.io = io;
    }

    private Command goToRotations(double goalRotations) {
        return run(() -> io.setPosition(goalRotations))
            .until(() -> MathUtil.isNear(goalRotations, inputs.rotations, Constants.rotationTolerance))
            .withTimeout(Constants.moveTimeoutSecs);
    }

    public Command deploy() {
        return goToRotations(Constants.DEPLOY_ROTATIONS)
            .andThen(runOnce(() -> {
                this.state = ClimbState.DEPLOY;
            }));
    }

    public Command climb() {
        return goToRotations(Constants.CLIMB_ROTATIONS)
            .andThen(runOnce(() -> {
                this.state = ClimbState.CLIMB;
            }));
    }

    public Command unclimb() {
        return goToRotations(Constants.DEPLOY_ROTATIONS)
            .andThen(runOnce(() -> {
                this.state = ClimbState.UNCLIMB;
            }));
    }

    public Command stow() {
        return goToRotations(Constants.STOW_ROTATIONS)
            .andThen(runOnce(() -> {
                this.state = ClimbState.STOW;
            }));
    }

    public Command move(boolean up) {
        return runEnd(() -> io.runVolts(Volts.of(3.92).times(up ? -1 : 1)), () -> io.runVolts(Volts.of(0)));
    }

    public Command doNext() {
        return Commands.defer(() -> {
            return switch (state) {
                case DEPLOY -> climb();
                case CLIMB -> unclimb();
                case UNCLIMB -> stow();
                case STOW -> deploy();
                default -> Commands.none();
            };
        }, Set.of(this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        Logger.recordOutput("Climber/ClimbState", state);
    }
}
