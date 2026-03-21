package frc.robot.subsystems.turretfeeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.SysIDMechanism;

public class TurretFeeder extends SubsystemBase implements SysIDMechanism {
    private static final AngularVelocity velocityTolerance = RPM.of(100);
    private static final AngularVelocity turretLoaderVelocity = RPM.of(1300);
    private static final Current turretLoaderTorqueCurrent = Amps.of(90);
    private static final Voltage turretLoaderVoltage = Volts.of(7);

    private final TurretFeederIO io;
    private final TurretFeederInputsAutoLogged inputs = new TurretFeederInputsAutoLogged();

    private SysIdRoutine sysId;

    private final LoggedNetworkNumber tuneTurretLoaderVelocity = new LoggedNetworkNumber("Tuning/TurretLoaderVelocityRPM", turretLoaderVelocity.in(RPM));

    public TurretFeeder(TurretFeederIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null,
                        (state) -> Logger.recordOutput("TurretFeeder/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> io.runVolts(voltage), null, this));
    }

    public final Trigger atSpeed = new Trigger(() -> RPM.of(inputs.velocityRPM).isNear(turretLoaderVelocity, velocityTolerance));

    public Command feed() {
        return runEnd(
            () -> io.setVelocity(turretLoaderVelocity),
            () -> io.stopMotor()
        );
    }

    public Command feedVoltage() {
        return runEnd(
            () -> io.runVolts(turretLoaderVoltage),
            () -> io.stopMotor()
        );
    }

    public Command feedTorqueCurrent() {
        return runEnd(
            () -> io.runTorqueCurrent(turretLoaderTorqueCurrent),
            () -> io.stopMotor()
        );
    }

    public Command feedDutyCycle() {
        return runEnd(
            () -> io.runDutyCycle(),
            () -> io.stopMotor()
        );
    }

    public Command feedTunable() {
        return runEnd(
            () -> io.setVelocity(RPM.of(tuneTurretLoaderVelocity.get())),
            () -> io.stopMotor()
        );
    }

    public AngularVelocity getVelocity() {
        return RPM.of(inputs.velocityRPM);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    @Override
    public List<SysIDMechanism.NamedMechanism> sysIdMechanisms() {
        return List.of(SysIDMechanism.named("Turret Feeder", this::sysIdDynamic, this::sysIdQuasistatic));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("TurretFeeder", inputs);
        Logger.recordOutput("TurretFeeder/AtSpeed", atSpeed.getAsBoolean());
    }
}
