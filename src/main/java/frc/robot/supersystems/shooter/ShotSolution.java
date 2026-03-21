package frc.robot.supersystems.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public record ShotSolution(AngularVelocity rpm, Angle pitch, Angle turretYaw) {};
