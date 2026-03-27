package frc.robot.subsystems.intakepivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IntakePivotTest extends IntakePivot {
    private SysIdRoutine sysId;

    public IntakePivotTest(IntakePivotIO io) {
        super(io);

        sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
                        null, null, null,
                        (state) -> Logger.recordOutput("IntakePivot/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> io.runVolts(voltage), null, this));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction)
                .until(isDeployed.or(isStowed));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction)
                .until(isDeployed.or(isStowed));
    }
}
