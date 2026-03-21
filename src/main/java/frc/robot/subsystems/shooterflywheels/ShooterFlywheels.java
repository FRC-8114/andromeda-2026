package frc.robot.subsystems.shooterflywheels;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterFlywheels extends SubsystemBase {
    private static class Constants {
        static final double flywheelToleranceRPM = 50;
    }

    private final ShooterFlywheelsIO io;
    private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    private AngularVelocity targetVelocity;
    
    private static final LoggedNetworkNumber tuneVelocity = new LoggedNetworkNumber("Tuning/TuneShooterVelocityRPM", 2000);
    
    private SysIdRoutine sysId;

    public ShooterFlywheels(ShooterFlywheelsIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null,
                        (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> io.runVolts(voltage),
                        null, this));
    }

    public double getAverageFlywheelRPMs() {
        return (inputs.leftFlywheelRPMs + inputs.rightFlywheelRPMs) / 2.0;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public Trigger atSpeed = new Trigger(() -> RPM.of(inputs.leftFlywheelRPMs).isNear(targetVelocity, Constants.flywheelToleranceRPM));

    public Command runFlywheelsTunableVelocity() {
        return runEnd(
            () -> io.setFlywheelVelocity(RPM.of(tuneVelocity.get())),
            () -> io.stopFlywheels());
    }

    public Command runFlywheels(AngularVelocity target) {
        targetVelocity = target;

        return runEnd(
                () -> io.setFlywheelVelocity(target),
                () -> io.stopFlywheels());
    }
    public Command runFlywheels(Supplier<AngularVelocity> target) {
        return runEnd(
                () -> {
                    targetVelocity = target.get();
                    io.setFlywheelVelocity(targetVelocity);
                },
                () -> io.stopFlywheels());
    }

    public Command stopFlywheels() {
        return runOnce(() -> io.stopFlywheels());
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }
}
