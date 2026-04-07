package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = false;
        inputs.yawPosition = Rotation2d.kZero;
        inputs.pitchPosition = Rotation2d.kZero;
        inputs.rollPosition = Rotation2d.kZero;
        inputs.yawVelocity = RadiansPerSecond.of(0.0);
        inputs.pitchVelocity = RadiansPerSecond.of(0.0);
        inputs.rollVelocity = RadiansPerSecond.of(0.0);
        inputs.odometryYawTimestamps = new double[] {};
        inputs.odometryYawPositions = new Rotation2d[] {};
    }
}
