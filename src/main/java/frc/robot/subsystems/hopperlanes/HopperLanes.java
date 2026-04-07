package frc.robot.subsystems.hopperlanes;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.SysIDMechanism;

public class HopperLanes extends SubsystemBase implements SysIDMechanism {
    private static final AngularVelocity indexerVelocityTolerance = RPM.of(20);

    private static final AngularVelocity indexerVelocity = RPM.of(480);

    private final HopperLanesIO io;
    private final HopperLanesInputsAutoLogged inputs = new HopperLanesInputsAutoLogged();

    private SysIdRoutine sysId;

    private final LoggedNetworkNumber tuneIndexerVelocity = new LoggedNetworkNumber("Tuning/IndexerVelocityRPM", indexerVelocity.in(RPM));

    public HopperLanes(HopperLanesIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(3).per(Second), Volts.of(50), null,
                        (state) -> Logger.recordOutput("HopperLanes/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> io.runCurrent(voltage.in(Volts)), null, this));
    }

    public Command feed() {
        return runEnd(
                () -> io.setVelocity(indexerVelocity),
                io::stopMotor);
    }

    public final Trigger atSpeed = new Trigger(
            () -> RPM.of(inputs.velocityRPM).isNear(indexerVelocity, indexerVelocityTolerance));

    public Command feedTunable() {
        return runEnd(
                () -> io.setVelocity(RPM.of(tuneIndexerVelocity.get())),
                io::stopMotor);
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
        return List.of(SysIDMechanism.named("Hopper Lanes", this::sysIdDynamic, this::sysIdQuasistatic));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("HopperLanes", inputs);
    }
}
