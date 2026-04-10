package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.SysIDMechanism;

public class IntakePivot extends SubsystemBase implements SysIDMechanism {
    public static class Constants {
        public static final Angle DEPLOYED_ANGLE = Degrees.of(0);
        public static final Angle HOLD_ANGLE = Radians.of(1);
        public static final Angle STOWED_ANGLE = Rotations.of(0.4);

        static final Angle PUMP_LOW_ANGLE = Rotations.of(0.08);
        static final Angle PUMP_HIGH_ANGLE = Rotations.of(0.20);
        static final double PUMP_DURATION_SECS = 0.25;

        static final Current HOLD_DOWN_FEEDFORWARD = Amps.of(20);
        static final Angle ANGLE_TOLERANCE = Degrees.of(1);
    }

    private final IntakePivotIO io;
    private final IntakePivotInputsAutoLogged inputs = new IntakePivotInputsAutoLogged();
    private final SysIdRoutine sysId;

    @AutoLogOutput
    public final Trigger isDeployed;

    @AutoLogOutput
    public final Trigger isStowed;

    @AutoLogOutput
    public final Trigger isAtHold;

    public IntakePivot(IntakePivotIO io) {
        this.io = io;

        isDeployed = isAtAngle(Constants.DEPLOYED_ANGLE);
        isStowed = isAtAngle(Constants.STOWED_ANGLE);
        isAtHold = isAtAngle(Constants.HOLD_ANGLE);

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        // amps
                        Volts.of(2.0).per(Second),
                        Volts.of(20.0),
                        Seconds.of(10.0),
                        (state) -> Logger.recordOutput("IntakePivot/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (fakeVolts) -> io.runCurrent(Amps.of(fakeVolts.in(Volts))),
                        null, this));
    }

    public Trigger isAtAngle(Angle angle) {
        return new Trigger(() -> inputs.positionRadians.isNear(angle, Constants.ANGLE_TOLERANCE));
    }

    public Command deploy() {
        return runEnd(
                () -> io.setTargetWithFeedForward(Constants.DEPLOYED_ANGLE, Constants.HOLD_DOWN_FEEDFORWARD),
                () -> io.setTarget(Constants.HOLD_ANGLE));
    }

    public Command hold() {
        return runEnd(() -> io.setTarget(Constants.HOLD_ANGLE), io::stop);
    }

    public Command stow() {
        return runEnd(() -> io.setTarget(Constants.STOWED_ANGLE), io::stop);
    }

    public Command pump() {
        return Commands.repeatingSequence(
                run(() -> io.setTarget(Constants.PUMP_LOW_ANGLE))
                        .withTimeout(Seconds.of(Constants.PUMP_DURATION_SECS)),
                run(() -> io.setTarget(Constants.PUMP_HIGH_ANGLE))
                        .withTimeout(Seconds.of(Constants.PUMP_DURATION_SECS)))
                .finallyDo(() -> io.setTarget(Constants.HOLD_ANGLE));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    @Override
    public List<SysIDMechanism.NamedMechanism> sysIdMechanisms() {
        return List.of(SysIDMechanism.named("Intake Pivot", this::sysIdDynamic, this::sysIdQuasistatic));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakePivot", inputs);
    }
}
