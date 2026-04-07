package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Climber extends SubsystemBase {
    // drum diameter: 0.787in
    static final double rotationTolerance = 0.03;
    static final double moveTimeoutSecs = 4.0;

    private enum ClimbState {
        STOW, DEPLOY, CLIMB, UNCLIMB
    }
    private ClimbState state = ClimbState.STOW;

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO io) {
        this.io = io;
    }

    public final Trigger isStowed = new Trigger(() -> MathUtil.isNear(ClimberConstants.stowRotations, inputs.positionRot, rotationTolerance));
    public final Trigger isDeployed = new Trigger(() -> MathUtil.isNear(ClimberConstants.deployRotations, inputs.positionRot, rotationTolerance));
    public final Trigger isClimbed = new Trigger(() -> MathUtil.isNear(ClimberConstants.climbRotations, inputs.positionRot, rotationTolerance));

    public Command deploy() {
        return run(() -> io.setPosition(ClimberConstants.deployRotations))
            .until(isDeployed)
            .andThen(runOnce(() -> {this.state = ClimbState.DEPLOY;}));
    }

    public Command climb() {
        return run(() -> io.setPosition(ClimberConstants.climbRotations))
            .until(isClimbed)
            .andThen(runOnce(() -> {this.state = ClimbState.CLIMB;}));
    }

    public Command unclimb() {
        return run(() -> io.setPosition(ClimberConstants.deployRotations))
            .until(isDeployed)
            .andThen(runOnce(() -> {this.state = ClimbState.UNCLIMB;}));
    }

    public Command stow() {
        return run(() -> io.setPosition(ClimberConstants.stowRotations))
            .until(isStowed)
            .andThen(runOnce(() -> {this.state = ClimbState.STOW;}));
    }

    public Command move(boolean up) {
        return runEnd(() -> io.runVolts(Volts.of(5).times(up ? -1 : 1)), () -> io.runVolts(Volts.of(0)));
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
