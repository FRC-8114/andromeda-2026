package frc.robot.supersystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.hopperlanes.HopperLanes;
import frc.robot.subsystems.shooterflywheels.ShooterFlywheels;
import frc.robot.subsystems.shooterpitch.ShooterPitch;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turretfeeder.TurretFeeder;
import frc.robot.supersystems.shooter.ShotSolverUtil.ShotSolution;

public class Shooter extends SubsystemBase {
    private static final double READY_TIMEOUT_SECONDS = 0.8;

    private final Turret turret;
    private final ShooterFlywheels flywheels;
    private final ShooterPitch shooterPitch;
    private final TurretFeeder turretFeeder;
    private final HopperLanes hopperLanes;
    private final Supplier<ShotSolution> shotSolver;

    public Shooter(
            Turret turret,
            ShooterFlywheels flywheels,
            ShooterPitch shooterPitch,
            TurretFeeder turretFeeder,
            HopperLanes hopperLanes,
            Supplier<ShotSolution> shotSolver) {
        this.turret = turret;
        this.flywheels = flywheels;
        this.shooterPitch = shooterPitch;
        this.turretFeeder = turretFeeder;
        this.hopperLanes = hopperLanes;
        this.shotSolver = shotSolver;
    }

    public ShotSolution getShotSolution() {
        return shotSolver.get();
    }

    public Command aimAtGoal() {
        return Commands.parallel(
                turret.followAngle(() -> getShotSolution().turretYaw()),
                shooterPitch.followAngle(() -> getShotSolution().pitch()));
    }

    public Command autoAimTurret() {
        return turret.followAngle(() -> getShotSolution().turretYaw());
    }

    public boolean isReadyToShoot() {
        ShotSolution shotSolution = getShotSolution();
        return flywheels.atSpeed(shotSolution.rpm()).getAsBoolean()
                && turretFeeder.atSpeed.getAsBoolean()
                && hopperLanes.atSpeed.getAsBoolean()
                && shooterPitch.isAtAngle(shotSolution.pitch()).getAsBoolean()
                && turret.isAtAngle(shotSolution.turretYaw()).getAsBoolean();
    }

    public Trigger isReadyToShootAt(Angle yaw, Angle pitch, AngularVelocity velocity) {
        return flywheels.atSpeed(velocity)
                .and(turretFeeder.atSpeed)
                .and(hopperLanes.atSpeed)
                .and(shooterPitch.isAtAngle(pitch))
                .and(turret.isAtAngle(yaw));
    }

    public Command spinUp() {
        return flywheels.runFlywheels(() -> getShotSolution().rpm());
    }

    public Command prepareToShoot() {
        return Commands.parallel(aimAtGoal(), spinUp(), turretFeeder.feed(), hopperLanes.reverse().withTimeout(1));
    }

    public Command shootAt(Angle yaw, Angle pitch, AngularVelocity velocity) {
        return Commands.parallel(
                turret.setAngle(yaw),
                flywheels.runFlywheels(velocity),
                shooterPitch.setAngle(pitch),
                turretFeeder.feed(),
                Commands.waitUntil(isReadyToShootAt(yaw, pitch, velocity))
                        .withTimeout(READY_TIMEOUT_SECONDS)
                        .andThen(hopperLanes.feed()));
    }

    public Command shoot() {
        return Commands.sequence(
                Commands.deadline(
                        Commands.waitUntil(this::isReadyToShoot).withTimeout(READY_TIMEOUT_SECONDS),
                        prepareToShoot()),
                Commands.parallel(
                        aimAtGoal(),
                        spinUp(),
                        turretFeeder.feed(),
                        hopperLanes.feed()));
    }

    public Command cleanUpShoot() {
        return Commands.parallel(

        );
    }
}
