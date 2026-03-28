package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IntakePivot extends SubsystemBase {
    private static final Angle angleTolerance = Degrees.of(0.5);

    // private static final Angle kickReleaseAngle = Degrees.of(18);
    // private static final double kickVoltage = -2.5;
    // private static final double holdDeployVoltage = 0.2;

    private IntakePivotIO io;
    private final IntakePivotInputsAutoLogged inputs = new IntakePivotInputsAutoLogged();

    public IntakePivot(IntakePivotIO io) {
        this.io = io;

        setDefaultCommand(run(() -> {
            if (isDeployed.getAsBoolean()) {
                io.holdTargetDown(IntakePivotConstants.deployAngle);
            }
        }));
    }

    public Trigger isDeployed = new Trigger(() -> Degrees.of(inputs.positionDeg).isNear(IntakePivotConstants.deployAngle, angleTolerance));
    public Trigger isStowed = new Trigger(() -> Radians.of(inputs.positionDeg).isNear(IntakePivotConstants.stowAngle, angleTolerance));

    // public Command periodicPulse() {
    // return Commands.repeatingSequence(
    // run(() -> io.runVolts(Volts.of(0.1))).withTimeout(0.25),
    // deploy().withTimeout(1.0),
    // Commands.waitTime(Seconds.of(1.0)))
    // .finallyDo(() -> io.setTarget(deployAngle));
    // }
    // public Command deployWithKick() {
    // return Commands.sequence(
    // run(() -> io.runVolts(Volts.of(kickVoltage))).until(() ->
    // Radians.of(inputs.positionRads).lte(kickReleaseAngle)),
    // run(() -> io.runVolts(Volts.of(holdDeployVoltage))).withTimeout(0.2),
    // deploy());
    // }

    public Command deploy() {
        return run(() -> io.setTarget(IntakePivotConstants.deployAngle)).until(isDeployed);
    }

    public Command stow() {
        return run(() -> io.setTarget(IntakePivotConstants.stowAngle)).until(isStowed);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakePivot", inputs);
    }
}
