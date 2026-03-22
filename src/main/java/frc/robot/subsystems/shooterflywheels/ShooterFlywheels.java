package frc.robot.subsystems.shooterflywheels;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.SysIDMechanism;

public class ShooterFlywheels extends SubsystemBase implements SysIDMechanism {
    private static class Constants {
        static final double FLYWHEEL_TOLERANCE_RPM = 50.0;
    }

    private final ShooterFlywheelsIO io;
    private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    private AngularVelocity targetVelocity = RPM.of(0.0);
    
    private static final LoggedNetworkNumber tuneVelocity = new LoggedNetworkNumber("Tuning/TuneShooterVelocityRPM", 2000);
    
    private SysIdRoutine sysId;

    public ShooterFlywheels(ShooterFlywheelsIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(3).per(Second), Volts.of(50), Seconds.of(20),
                        (state) -> Logger.recordOutput("ShooterFlywheels/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> io.runCurrent(voltage.in(Volts)),
                        null, this));
    }

    private boolean isWheelAtSpeed(AngularVelocity velocity) {
        return velocity.isNear(targetVelocity, Constants.FLYWHEEL_TOLERANCE_RPM);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ShooterFlywheels", inputs);
    }

    public final Trigger atSpeed = new Trigger(
            () -> isWheelAtSpeed(inputs.leftFlywheelVelocity) && isWheelAtSpeed(inputs.rightFlywheelVelocity));

    public Command runFlywheelsTunableVelocity() {
        return runEnd(
            () -> {
                targetVelocity = RPM.of(tuneVelocity.get());
                io.setFlywheelVelocity(targetVelocity);
            },
            () -> {
                targetVelocity = RPM.of(0.0);
                io.stopFlywheels();
            });
    }

    public Command runFlywheels(AngularVelocity target) {
        return runEnd(
                () -> {
                    targetVelocity = target;
                    io.setFlywheelVelocity(target);
                },
                () -> {
                    targetVelocity = RPM.of(0.0);
                    io.stopFlywheels();
                });
    }
    public Command runFlywheels(Supplier<AngularVelocity> target) {
        return runEnd(
                () -> {
                    targetVelocity = target.get();
                    io.setFlywheelVelocity(targetVelocity);
                },
                () -> {
                    targetVelocity = RPM.of(0.0);
                    io.stopFlywheels();
                });
    }

    public Command stopFlywheels() {
        return runOnce(() -> {
            targetVelocity = RPM.of(0.0);
            io.stopFlywheels();
        });
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    @Override
    public List<SysIDMechanism.NamedMechanism> sysIdMechanisms() {
        return List.of(SysIDMechanism.named("Shooter Flywheels", this::sysIdDynamic, this::sysIdQuasistatic));
    }
}
