// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import frc.robot.subsystems.vision.VisionConstants.LimelightPoseMode;
import frc.robot.subsystems.vision.VisionIO.PoseEstimation;
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
                this::acceptVisionMeasurement,
                this::seedPoseFromVision,
                drive::getFieldGyroRotation3d,
                drive::getRawGyroVelocityRadPerSec,
                drive::getPose));
        // vision = null;

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

    private void acceptVisionMeasurement(PoseEstimation observation) {
        drive.addVisionMeasurement(observation.pose().toPose2d(), observation.timestamp().in(Seconds),
                observation.stddev());
    }

    private void seedPoseFromVision(PoseEstimation observation) {
        drive.setPose(observation.pose().toPose2d());
    }

    private final CommandXboxController driverController = new CommandXboxController(0);

    private void configureBindings() {
        drive.setDefaultCommand(
                drive.joystickDrive(
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX()));

        driverController.rightTrigger().whileTrue(shooter.shoot());
        driverController.b().onTrue(intakePivot.deploy());
        driverController.leftTrigger().whileTrue(Commands.parallel(intakeRollers.intake(), intakePivot.deploy()));

        // driverController.povUp().whileTrue(hopperLanes.feed());
        // driverController.povUp().whileTrue(flywheels.runFlywheels(RPM.of(1800)));

        driverController.start()
                .onTrue(Commands.runOnce(() -> VisionConstants.LIMELIGHT_ESTIMATION_MODE = LimelightPoseMode.MEGATAG1));

        driverController.povUp().whileTrue(climber.move(true));
        driverController.povDown().whileTrue(climber.move(false));

        // driverController.povDown().whileTrue(
        // Commands.parallel(flywheels.runFlywheelsTunableVelocity(),
        // turret.aimTunable(),
        // shooterPitch.tuneAngle()).alongWith(Commands.waitSeconds(1)
        // .andThen(turretFeeder.feed().until(turretFeeder.atSpeed).andThen(hopperLanes.feed().alongWith(turretFeeder.feed())))));

        driverController.povRight().whileTrue(intakePivot.pump());

        driverController.povLeft().whileTrue(shooter.shootAt(
                Degrees.of(180),
                Degrees.of(25),
                RPM.of(2200)));
    }

    public void updateDashboard() {
        SmartDashboard.putBoolean("HubShiftTeam", isHubActive());
    }

    public void enabledInit() {
    }

    public void teleopInit() {
        vision.setIMUMode(4 /* INTERNAL_EXTERNAL_ASSIST */);
    }

    public void disabledInit() {
        vision.setIMUMode(1 /* EXTERNAL_SEED */);

        driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    }

    public Command getAutonomousCommand() {
        return Commands.parallel(autoChooser.selectedCommand(),
                Commands.waitSeconds(1).andThen(() -> vision.setIMUMode(4))); // INTERNAL_EXTERNAL_ASSIST
    }

    public <T extends Subsystem> Optional<T> getSubsystem(Class<T> type) {
        return subsystemRegistry.get(type);
    }

    public <T extends Subsystem> T requireSubsystem(Class<T> type) {
        return subsystemRegistry.require(type);
    }
}
