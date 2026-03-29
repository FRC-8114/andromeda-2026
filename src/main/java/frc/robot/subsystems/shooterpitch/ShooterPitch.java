package frc.robot.subsystems.shooterpitch;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterPitch extends SubsystemBase {
    public static class Constants {
        public static final Angle MAX_ANGLE = Degree.of(34.5);
        public static final Angle MIN_ANGLE = Degree.of(5.361328);

        private static final Angle ANGLE_TOLERANCE = Degrees.of(1);
        private static final double READY_VELOCITY_TOLERANCE_RAD_PER_SEC = Math.toRadians(5.0);

        private static final Angle ZERO_ANGLE = Degrees.of(0);
    }

    private final ShooterPitchIO pitchMotor;
    private final ShooterPitchInputsAutoLogged inputs = new ShooterPitchInputsAutoLogged();
    private final SysIdRoutine sysId;

    // private final LoggedNetworkNumber angle = new
    // LoggedNetworkNumber("Tuning/ShooterPitch", Constants.MIN_ANGLE.in(Degrees));

    public ShooterPitch(ShooterPitchIO pitchMotor) {
        this.pitchMotor = pitchMotor;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null,
                        (state) -> Logger.recordOutput("ShooterPitch/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> pitchMotor.setVoltage(voltage.in(Volts)), null, this));

        setDefaultCommand(setAngle(Degrees.of(0)));
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

    public Command setAngle(Angle pitchAngle) {
        return runEnd(() -> pitchMotor.setTarget(pitchAngle), () -> pitchMotor.setTarget(Constants.ZERO_ANGLE));
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
