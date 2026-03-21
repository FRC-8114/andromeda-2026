package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IntakePivot extends SubsystemBase {
    public static final Angle stowAngle = Radians.of(1.98);
    public static final Angle deployAngle = Rotations.of(0.05);

    private static final Angle angleTolerance = Degrees.of(5);
    private static final Angle kickReleaseAngle = Degrees.of(18);
    private static final double kickVoltage = -2.5;
    private static final double holdDeployVoltage = 0.2;
    private IntakePivotIO io;
    private final IntakePivotInputsAutoLogged inputs = new IntakePivotInputsAutoLogged();

    private SysIdRoutine sysId;

    public IntakePivot(IntakePivotIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null,
                        (state) -> Logger.recordOutput("IntakePivot/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> io.runVolts(voltage), null, this));
    }

    public Trigger isDeployed = new Trigger(() -> Radians.of(inputs.positionRads).isNear(deployAngle, angleTolerance));
    public Trigger isStowed = new Trigger(() -> Radians.of(inputs.positionRads).isNear(stowAngle, angleTolerance));

    public Command periodicPulse() {
        return Commands.repeatingSequence(
                run(() -> io.runVolts(Volts.of(0.1))).withTimeout(0.25),
                deploy().withTimeout(1.0),
                Commands.waitTime(Seconds.of(1.0)))
                .finallyDo(() -> io.setTarget(deployAngle));
    }

    public Command deploy() {
        return run(() -> io.setTarget(deployAngle));
    }

    public Command deployWithKick() {
        return Commands.sequence(
                run(() -> io.runVolts(Volts.of(kickVoltage))).until(() -> Radians.of(inputs.positionRads).lte(kickReleaseAngle)),
                run(() -> io.runVolts(Volts.of(holdDeployVoltage))).withTimeout(0.2),
                deploy());
    }

    public Command stow() {
        return run(() -> io.setTarget(stowAngle))
                .until(isStowed);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction)
                .until(isDeployed.or(isStowed));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction)
                .until(isDeployed.or(isStowed));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakePivot", inputs);
        Logger.recordOutput("IntakePivot/IsDeployed", isDeployed.getAsBoolean());
        Logger.recordOutput("IntakePivot/IsStowed", isStowed.getAsBoolean());
    }
}
