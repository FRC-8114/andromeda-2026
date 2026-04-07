package frc.robot.subsystems.shooterpitch;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterPitch extends SubsystemBase {
    public static class Constants {
        public static final Angle MAX_ANGLE = Degree.of(34.5);
        public static final Angle MIN_ANGLE = Degree.of(5.361328);

        private static final Angle ANGLE_TOLERANCE = Degrees.of(1);
        private static final double READY_VELOCITY_TOLERANCE_RAD_PER_SEC = Math.toRadians(5.0);

        private static final Angle ZERO_ANGLE = Degrees.of(0);
        private static final double DEFAULT_HOMING_VOLTAGE = -1.5;
        private static final double DEFAULT_HOMING_CURRENT_SPIKE_AMPS = 30.0;
    }

    private final ShooterPitchIO pitchMotor;
    private final ShooterPitchInputsAutoLogged inputs = new ShooterPitchInputsAutoLogged();
    private final SysIdRoutine sysId;

    private final LoggedNetworkNumber angle = new LoggedNetworkNumber("Tuning/ShooterPitch",
            Constants.MIN_ANGLE.in(Degrees));
    private final LoggedNetworkNumber homingVoltage = new LoggedNetworkNumber(
            "Tuning/ShooterPitchHomingVoltage", Constants.DEFAULT_HOMING_VOLTAGE);
    private final LoggedNetworkNumber homingCurrentSpikeAmps = new LoggedNetworkNumber(
            "Tuning/ShooterPitchHomingCurrentSpikeAmps", Constants.DEFAULT_HOMING_CURRENT_SPIKE_AMPS);

    public ShooterPitch(ShooterPitchIO pitchMotor) {
        this.pitchMotor = pitchMotor;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null,
                        (state) -> Logger.recordOutput("ShooterPitch/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> pitchMotor.setVoltage(voltage.in(Volts)), null, this));

        setDefaultCommand(Commands.either(homeAndReseed(), holdMinAngle(), pitchMotor::supportsHomingReseed));
    }

    public double getPitchPositionRads() {
        return inputs.pitchPosition;
    }

    private Angle getPitchPosition() {
        return Radians.of(inputs.pitchPosition);
    }

    @Override
    public void periodic() {
        pitchMotor.updateInputs(inputs);
        Logger.processInputs("ShooterPitch", inputs);
    }

    public boolean isAtAngle(Angle target) {
        return target.isNear(getPitchPosition(), Constants.ANGLE_TOLERANCE)
                && Math.abs(inputs.velocityRadsPerSec) <= Constants.READY_VELOCITY_TOLERANCE_RAD_PER_SEC;
    }

    private boolean hasHomingCurrentSpike() {
        return Math.abs(inputs.appliedCurrentAmps) >= homingCurrentSpikeAmps.get();
    }

    private Command holdMinAngle() {
        return run(() -> pitchMotor.setTarget(Constants.MIN_ANGLE));
    }

    public Command homeAndReseed() {
        return Commands.sequence(
                runEnd(() -> pitchMotor.setHomingVoltage(homingVoltage.get()), () -> pitchMotor.setVoltage(0.0))
                        .until(this::hasHomingCurrentSpike),
                runOnce(() -> {
                    pitchMotor.setVoltage(0.0);
                    pitchMotor.reseedPosition(Constants.MIN_ANGLE);
                }),
                holdMinAngle());
    }

    public Command setAngle(Angle pitchAngle) {
        return runEnd(() -> pitchMotor.setTarget(pitchAngle), () -> pitchMotor.setTarget(Constants.ZERO_ANGLE));
    }

    public Command tuneAngle() {
        return followAngle(() -> Degrees.of(angle.get()));
    }

    public Command followAngle(Supplier<Angle> pitchAngle) {
        return runEnd(() -> {
            Angle target = pitchAngle.get();
            pitchMotor.setTarget(target);
        }, () -> pitchMotor.setTarget(Constants.ZERO_ANGLE));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> pitchMotor.setVoltage(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> pitchMotor.setVoltage(0.0))
                .withTimeout(1.0)
                .andThen(sysId.dynamic(direction));
    }
}
