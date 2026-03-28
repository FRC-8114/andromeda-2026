package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakePivotIOSim implements IntakePivotIO {
    private static final MomentOfInertia pivotMoi = KilogramSquareMeters.of(0.02);
    private static final Distance armLength = Inches.of(11.5);

    private final SingleJointedArmSim pivotSimModel = new SingleJointedArmSim(
        DCMotor.getKrakenX60Foc(1),
        IntakePivotConstants.gearRatio,
        pivotMoi.in(KilogramSquareMeters),
        armLength.in(Meters),
        IntakePivotConstants.deployAngle.in(Radians),
        IntakePivotConstants.stowAngle.in(Radians),
        true,
        IntakePivotConstants.stowAngle.in(Radians)
    );
    private final PIDController pivotSimController = new PIDController(6.0, 0.0, 0.2);

    private boolean isClosedLoop = false;
    private MutVoltage pivotAppliedVoltage = Volts.mutable(0);

    public void setTarget(Angle angle) {
        isClosedLoop = true;
        pivotSimController.setSetpoint(angle.in(Radians));
    }
    public void holdTargetDown(Angle angle) {
        isClosedLoop = true;
        pivotSimController.setSetpoint(angle.in(Radians));
    }

    public void runVolts(Voltage volts) {
        isClosedLoop = false;
        pivotAppliedVoltage.mut_replace(volts);
    }

    public void updateInputs(IntakePivotInputs inputs) {
        if (isClosedLoop) {
            pivotAppliedVoltage.mut_replace(pivotSimController.calculate(pivotSimModel.getAngleRads()), Volts);
        } else {
            pivotSimController.reset();
        }

        pivotAppliedVoltage.mut_replace(MathUtil.clamp(pivotAppliedVoltage.in(Volts), -12.0, 12.0), Volts);
        pivotSimModel.setInputVoltage(pivotAppliedVoltage.in(Volts));
        pivotSimModel.update(0.02);

        inputs.positionDeg = Units.degreesToRadians(pivotSimModel.getAngleRads());
        inputs.velocityRPM = Units.radiansPerSecondToRotationsPerMinute(pivotSimModel.getVelocityRadPerSec());
        inputs.appliedVoltageVolts = pivotAppliedVoltage.in(Volts);
    }
}