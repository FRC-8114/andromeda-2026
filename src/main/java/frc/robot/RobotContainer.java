// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hopperlanes.HopperLanes;
import frc.robot.subsystems.hopperlanes.HopperLanesIOReal;
import frc.robot.subsystems.hopperlanes.HopperLanesIOSim;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakepivot.IntakePivotIOReal;
import frc.robot.subsystems.intakepivot.IntakePivotIOSim;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.intakerollers.IntakeRollersIOReal;
import frc.robot.subsystems.intakerollers.IntakeRollersIOSim;
import frc.robot.subsystems.shooterflywheels.ShooterFlywheels;
import frc.robot.subsystems.shooterflywheels.ShooterFlywheelsIOReal;
import frc.robot.subsystems.shooterflywheels.ShooterFlywheelsIOSim;
import frc.robot.subsystems.shooterpitch.ShooterPitch;
import frc.robot.subsystems.shooterpitch.ShooterPitchIOReal;
import frc.robot.subsystems.shooterpitch.ShooterPitchIOSim;
import frc.robot.supersystems.intake.Intake;
import frc.robot.supersystems.shooter.Shooter;
import frc.robot.supersystems.shooter.ShotSolverUtil;
import frc.robot.supersystems.shooter.TurretShotSolverAnglemap;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOReal;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turretfeeder.TurretFeeder;
import frc.robot.subsystems.turretfeeder.TurretFeederIOReal;
import frc.robot.subsystems.turretfeeder.TurretFeederIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.SubsystemRegistry;
import java.util.Optional;

public class RobotContainer {
    private final SubsystemRegistry subsystemRegistry = new SubsystemRegistry();
    private final Drive drive;
    private final AutoChooser autoChooser;

    @SuppressWarnings("unused")
    private final Turret turret;

    @SuppressWarnings("unused")
    private final ShooterFlywheels flywheels;

    @SuppressWarnings("unused")
    private final ShooterPitch shooterPitch;

    @SuppressWarnings("unused")
    private final TurretFeeder turretFeeder;

    @SuppressWarnings("unused")
    private final HopperLanes hopperLanes;

    @SuppressWarnings("unused")
    private final IntakePivot intakePivot;

    @SuppressWarnings("unused")
    private final IntakeRollers intakeRollers;

    @SuppressWarnings("unused")
    private final Intake intake;

    @SuppressWarnings("unused")
    private final Climber climber;

    @SuppressWarnings("unused")
    private final Shooter shooter;

    @SuppressWarnings("unused")
    private final Vision vision;

    public RobotContainer() {
        switch (RobotConstants.getRobotMode()) {
            case REAL: {
                turret = subsystemRegistry.register(new Turret(new TurretIOReal()));
                flywheels = subsystemRegistry.register(new ShooterFlywheels(new ShooterFlywheelsIOReal()));
                shooterPitch = subsystemRegistry.register(new ShooterPitch(new ShooterPitchIOReal()));
                turretFeeder = subsystemRegistry.register(new TurretFeeder(new TurretFeederIOReal()));
                hopperLanes = subsystemRegistry.register(new HopperLanes(new HopperLanesIOReal()));
                intakePivot = subsystemRegistry.register(new IntakePivot(new IntakePivotIOReal()));
                intakeRollers = subsystemRegistry.register(new IntakeRollers(new IntakeRollersIOReal()));
                intake = subsystemRegistry.register(new Intake(intakePivot, intakeRollers));
                climber = subsystemRegistry.register(new Climber(new ClimberIOReal()));
                break;
            }
            case SIMULATION, REPLAY: {
                turret = subsystemRegistry.register(new Turret(new TurretIOSim()));
                flywheels = subsystemRegistry.register(new ShooterFlywheels(new ShooterFlywheelsIOSim()));
                shooterPitch = subsystemRegistry.register(new ShooterPitch(new ShooterPitchIOSim()));
                turretFeeder = subsystemRegistry.register(new TurretFeeder(new TurretFeederIOSim()));
                hopperLanes = subsystemRegistry.register(new HopperLanes(new HopperLanesIOSim()));
                intakePivot = subsystemRegistry.register(new IntakePivot(new IntakePivotIOSim()));
                intakeRollers = subsystemRegistry.register(new IntakeRollers(new IntakeRollersIOSim()));
                intake = subsystemRegistry.register(new Intake(intakePivot, intakeRollers));
                climber = subsystemRegistry.register(new Climber(new ClimberIOSim()));
                break;
            }
            default:
                throw new IllegalStateException("robot somehow not in state");
        }

        drive = subsystemRegistry.register(createDrive());

        var kinematicsSupplier = new ShotSolverUtil.DriveKinematicsSupplier(drive);
        shooter = subsystemRegistry.register(
                new Shooter(turret, flywheels, shooterPitch, turretFeeder, hopperLanes,
                        new TurretShotSolverAnglemap(new ShotSolverUtil.TargetSupplier(kinematicsSupplier),
                                kinematicsSupplier)));

        vision = subsystemRegistry.register(Vision.fromCameraConstants(
                () -> new Vision.RobotStateSample(
                        Timer.getFPGATimestamp(),
                        drive.getFieldGyroRotation3d(),
                        drive.getRawGyroVelocityRadPerSec()),
                drive::addVisionMeasurement,
                drive::setPose));

        autoChooser = new Autos(subsystemRegistry).createChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();
    }

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its
        // likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }

    private Drive createDrive() {
        return switch (RobotConstants.getRobotMode()) {
            case REAL -> new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOTalonFX(TunerConstants.FrontLeft),
                    new ModuleIOTalonFX(TunerConstants.FrontRight),
                    new ModuleIOTalonFX(TunerConstants.BackLeft),
                    new ModuleIOTalonFX(TunerConstants.BackRight));
            case SIMULATION, REPLAY -> new Drive(
                    new GyroIOSim(),
                    new ModuleIOSim(TunerConstants.FrontLeft),
                    new ModuleIOSim(TunerConstants.FrontRight),
                    new ModuleIOSim(TunerConstants.BackLeft),
                    new ModuleIOSim(TunerConstants.BackRight));
        };
    }

    private final CommandXboxController driverController = new CommandXboxController(0);

    boolean isSOTMSpeedEnabled = false;

    private void configureBindings() {
        drive.setDefaultCommand(
                drive.joystickDrive(
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX(),
                        () -> drive.getMaxLinearSpeedMetersPerSec()
                                * (driverController.rightTrigger().getAsBoolean() ? 0.2 : 1),
                        () -> drive.getMaxAngularSpeedRadPerSec()
                                * (driverController.rightTrigger().getAsBoolean() ? 0.2 : 1)));

        driverController.rightTrigger().whileTrue(shooter.shoot());
        driverController.b().onTrue(intake.deploy());
        driverController.leftTrigger().whileTrue(intake.intake());

        // driverController.povUp().whileTrue(hopperLanes.feed());
        // driverController.povUp().whileTrue(flywheels.runFlywheels(RPM.of(1800)));

        driverController.povUp().whileTrue(climber.move(false));
        // driverController.povDown().whileTrue(climber.move(true));

        driverController.x().onTrue(climber.deploy());
        driverController.y().onTrue(climber.climb());
        driverController.a().onTrue(climber.stow());

        driverController.rightBumper().whileTrue(hopperLanes.reverse());

        driverController.povDown().whileTrue(
                Commands.parallel(flywheels.runFlywheelsTunableVelocity(),
                        turret.aimTunable(),
                        shooterPitch.tuneAngle()).alongWith(
                                Commands.waitSeconds(1)
                                        .andThen(turretFeeder.feed().until(turretFeeder.atSpeed)
                                                .andThen(hopperLanes.feed().alongWith(turretFeeder.feed())))));

        driverController.b().onTrue(Commands.runOnce(intake::toggleStow));

        driverController.povRight().whileTrue(intake.pump());
        driverController.povLeft().onTrue(climber.doNext());
    }

    public void updateDashboard() {
        SmartDashboard.putBoolean("HubShiftTeam", isHubActive());
    }

    public void enabledInit() {
    }

    public void teleopInit() {
        vision.setIMUMode(VisionConstants.LIMELIGHT_IMU_MODE);
    }

    public void disabledInit() {
        vision.setIMUMode(VisionConstants.LIMELIGHT_SEED_IMU_MODE);

        driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    }

    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();
    }

    public <T extends Subsystem> Optional<T> getSubsystem(Class<T> type) {
        return subsystemRegistry.get(type);
    }

    public <T extends Subsystem> T requireSubsystem(Class<T> type) {
        return subsystemRegistry.require(type);
    }
}
