// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO.PoseEstimation;

public class RobotContainer {
  private final Drive drive;
  private final AutoChooser autoChooser;
  
  @SuppressWarnings("unused")
  private final Vision vision;

  public RobotContainer() {
    drive = createDrive();
    autoChooser = new Autos(drive).createChooser();
    vision = Vision.fromCameraConstants(
        this::acceptVisionMeasurement,
        this::seedPoseFromVision,
        drive::getFieldGyroRotation3d,
        drive::getRawGyroVelocityRadPerSec,
        drive::getPose);
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
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
    drive.addVisionMeasurement(observation.pose().toPose2d(), observation.timestamp().in(Seconds), observation.stddev());
  }

  private void seedPoseFromVision(PoseEstimation observation) {
    drive.setPose(observation.pose().toPose2d());
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
