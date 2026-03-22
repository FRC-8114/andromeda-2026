package frc.robot.supersystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopperlanes.HopperLanes;
import frc.robot.subsystems.shooterflywheels.ShooterFlywheels;
import frc.robot.subsystems.shooterpitch.ShooterPitch;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turretfeeder.TurretFeeder;
import frc.robot.util.AllianceFlipUtil;

public class Shooter extends SubsystemBase {
    private static final double READY_TIMEOUT_SECONDS = 2.0;

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
            Drive drive) {
        this(
                turret,
                flywheels,
                shooterPitch,
                turretFeeder,
                hopperLanes,
                new TurretShotSolver(Shooter::getDefaultTargetPose, new TurretShotSolver.DriveKinematicsSupplier(drive)));
    }

    public Shooter(
            Turret turret,
            frc.robot.subsystems.shooterflywheels.ShooterFlywheels flywheels,
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

    private static Pose3d getDefaultTargetPose() {
        return new Pose3d(AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint), Rotation3d.kZero);
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
        return flywheels.atSpeed.getAsBoolean()
                && turretFeeder.atSpeed.getAsBoolean()
                && hopperLanes.atSpeed.getAsBoolean()
                && shooterPitch.isAtAngle(shotSolution.pitch())
                && turret.isAtAngle(shotSolution.turretYaw());
    }

    public Command spinUp() {
        return flywheels.runFlywheels(() -> getShotSolution().rpm());
    }

    public Command spinUpTurretLanes() {
        return turretFeeder.feed();
    }

    public Command runIndexerLanes() {
        return hopperLanes.feed();
    }

    public Command prepareToShoot() {
        return Commands.parallel(aimAtGoal(), spinUp(), spinUpTurretLanes());
    }

    public Command shoot() {
        return Commands.sequence(
                Commands.deadline(
                        Commands.waitUntil(this::isReadyToShoot).withTimeout(READY_TIMEOUT_SECONDS),
                        prepareToShoot()),
                        Commands.parallel(
                                aimAtGoal(),
                                spinUp(),
                                spinUpTurretLanes(),
                                runIndexerLanes()));
    }

}
