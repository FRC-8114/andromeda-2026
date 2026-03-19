package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.SysIDMechanism;

public class Turret extends SubsystemBase implements SysIDMechanism {
    public static class Constants {
        private static final Angle ANGLE_TOLERANCE = Degrees.of(1);

        // The reachable travel range wraps around +X, so robot-relative zero lies in the deadzone.
        public static final Angle MIN_ANGLE = Degrees.of(40);
        public static final Angle MAX_ANGLE = Radians.of(4.85);
    }

    private final TurretIO pivotMotor;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    public Turret(TurretIO pivotMotor) {
        this.pivotMotor = pivotMotor;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.5).per(Second), Volts.of(2.5), null,
                        (state) -> Logger.recordOutput("Turret/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> pivotMotor.setVoltage(voltage.in(Volts)), null, this));

    }

    public static Angle normalizeAngle(Angle angle) {
        double fullRotationRadians = 2.0 * Math.PI;
        double normalizedAngle = angle.in(Radians) % fullRotationRadians;

        if (normalizedAngle < 0.0) {
            normalizedAngle += fullRotationRadians;
        }

        return Radians.of(normalizedAngle);
    }

    public static boolean isWithinTravelRange(Angle angle) {
        double normalizedAngle = normalizeAngle(angle).in(Radians);
        return normalizedAngle >= Constants.MIN_ANGLE.in(Radians)
                && normalizedAngle <= Constants.MAX_ANGLE.in(Radians);
    }

    private static Angle getClosestBoundary(Angle angle) {
        Angle normalizedAngle = normalizeAngle(angle);
        double minDistance = Math.abs(MathUtil.inputModulus(
                normalizedAngle.in(Radians) - Constants.MIN_ANGLE.in(Radians),
                -Math.PI,
                Math.PI));
        double maxDistance = Math.abs(MathUtil.inputModulus(
                normalizedAngle.in(Radians) - Constants.MAX_ANGLE.in(Radians),
                -Math.PI,
                Math.PI));
        return minDistance <= maxDistance ? Constants.MIN_ANGLE : Constants.MAX_ANGLE;
    }

    public static Angle clampAngle(Angle angle) {
        Angle normalizedAngle = normalizeAngle(angle);
        if (isWithinTravelRange(normalizedAngle)) {
            return normalizedAngle;
        }
        return getClosestBoundary(normalizedAngle);
    }

    public static Angle resolveTargetAngle(Angle currentAngle, Angle requestedAngle) {
        Angle normalizedCurrentAngle = normalizeAngle(currentAngle);
        Angle normalizedRequestedAngle = normalizeAngle(requestedAngle);

        if (isWithinTravelRange(normalizedRequestedAngle)) {
            return normalizedRequestedAngle;
        }

        double shortestError = MathUtil.inputModulus(
                normalizedRequestedAngle.in(Radians) - normalizedCurrentAngle.in(Radians),
                -Math.PI,
                Math.PI);

        if (shortestError > 0.0) {
            return Constants.MAX_ANGLE;
        }
        if (shortestError < 0.0) {
            return Constants.MIN_ANGLE;
        }
        return getClosestBoundary(normalizedRequestedAngle);
    }

    static double calculateTravelErrorRadians(Angle currentAngle, Angle targetAngle) {
        return normalizeAngle(targetAngle).in(Radians) - normalizeAngle(currentAngle).in(Radians);
    }

    private Angle getTurretPosition() {
        return normalizeAngle(inputs.currentTurretPosition);
    }

    public double getTurretPositionRads() {
        return getTurretPosition().in(Radians);
    }

    private Angle getResolvedTargetAngle(Angle target) {
        return resolveTargetAngle(getTurretPosition(), target);
    }

    private void commandTarget(Angle target) {
        Angle normalizedTarget = normalizeAngle(target);
        Angle resolvedTarget = getResolvedTargetAngle(normalizedTarget);
        Logger.recordOutput("Turret/RequestedTargetRad", normalizedTarget.in(Radians));
        Logger.recordOutput("Turret/ResolvedTargetRad", resolvedTarget.in(Radians));
        Logger.recordOutput("Turret/RequestedTargetInDeadzone", !isWithinTravelRange(normalizedTarget));
        pivotMotor.setTarget(resolvedTarget);
    }

    @Override
    public void periodic() {
        pivotMotor.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        Logger.recordOutput("Turret/NormalizedPositionRad", getTurretPositionRads());
    }

    public boolean isAtAngle(Angle target) {
        double error = calculateTravelErrorRadians(getTurretPosition(), getResolvedTargetAngle(target));
        return Math.abs(error) <= Constants.ANGLE_TOLERANCE.in(Radians);
    }

    public Command setAngle(Angle angle) {
        return run(() -> commandTarget(angle));
    }

    public Command followAngle(Supplier<Angle> angle) {
        return run(() -> commandTarget(angle.get()));
    }

    private boolean isOutOfBounds() {
        return !isWithinTravelRange(getTurretPosition());
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> pivotMotor.setVoltage(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction)
                        .until(this::isOutOfBounds)
                        .finallyDo(() -> pivotMotor.setVoltage(0.0)));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> pivotMotor.setVoltage(0.0))
                .withTimeout(1.0)
                .andThen(sysId.dynamic(direction)
                        .until(this::isOutOfBounds)
                        .finallyDo(() -> pivotMotor.setVoltage(0.0)));
    }

    @Override
    public List<SysIDMechanism.NamedMechanism> sysIdMechanisms() {
        return List.of(SysIDMechanism.named("Turret", this::sysIdDynamic, this::sysIdQuasistatic));
    }
}
