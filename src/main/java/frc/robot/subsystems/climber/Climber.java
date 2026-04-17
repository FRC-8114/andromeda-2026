package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import java.util.Set;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
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

    @AutoLogOutput
    private ClimbState climbState = ClimbState.STOW;

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO io) {
        this.io = io;
    }

    public final Trigger stowCurrentSpike = new Trigger(() -> inputs.torqueCurrent.gte(Amps.of(30)));

    public final Trigger isStowed = new Trigger(() -> inputs.drumPosition.isNear(ClimberConstants.stowRotations, rotationTolerance));
    public final Trigger isDeployed = new Trigger(() -> inputs.drumPosition.isNear(ClimberConstants.deployRotations, rotationTolerance));
    public final Trigger isClimbed = new Trigger(() -> inputs.drumPosition.isNear(ClimberConstants.climbRotations, rotationTolerance));

    public Command deploy() {
        return run(() -> io.setPosition(ClimberConstants.deployRotations))
            .until(isDeployed)
            .andThen(runOnce(() -> {this.climbState = ClimbState.DEPLOY;}));
    }

    public Command climb() {
        return run(() -> io.setPosition(ClimberConstants.climbRotations))
            .until(isClimbed)
            .andThen(runOnce(() -> {this.climbState = ClimbState.CLIMB;}));
    }

    public Command unclimb() {
        return run(() -> io.setPosition(ClimberConstants.deployRotations))
            .until(isDeployed)
            .andThen(runOnce(() -> {this.climbState = ClimbState.UNCLIMB;}));
    }

    public Command stow() {
        double startStow = Timer.getFPGATimestamp();
        return run(() -> io.setPosition(ClimberConstants.stowRotations))
            .until(isStowed.or(stowCurrentSpike.and(() -> (Timer.getFPGATimestamp() - startStow) > 1 )))
            .andThen(runOnce(() -> {this.climbState = ClimbState.STOW;}));
    }

    public Command move(boolean down) {
        return runEnd(() -> io.runVolts(Volts.of(5).times(down ? -1 : 1)), () -> io.runVolts(Volts.of(0)));
    }

    public Command doNext() {
        return Commands.defer(() -> {
            return switch (climbState) {
                case DEPLOY -> climb();
                case CLIMB -> unclimb();
                case UNCLIMB -> stow();
                case STOW -> deploy();
            };
        }, Set.of(this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }
}
