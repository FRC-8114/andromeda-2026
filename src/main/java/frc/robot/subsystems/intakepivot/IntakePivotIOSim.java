package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakePivotIOSim implements IntakePivotIO {
    private static final double gearRatio = 11.8125;
    private static final MomentOfInertia pivotMoi = KilogramSquareMeters.of(0.02);
    private static final Distance armLength = Inches.of(11.5);

    private final SingleJointedArmSim pivotSimModel = new SingleJointedArmSim(
        DCMotor.getKrakenX60Foc(1),
        gearRatio,
        pivotMoi.in(KilogramSquareMeters),
        armLength.in(Meters),
        IntakePivot.deployAngle.in(Radians),
        IntakePivot.stowAngle.in(Radians),
        true,
        IntakePivot.stowAngle.in(Radians)
    );
    private final PIDController pivotSimController = new PIDController(6.0, 0.0, 0.2);

    private boolean isClosedLoop = false;
    private MutVoltage pivotAppliedVoltage = Volts.mutable(0);

    public void setTarget(Angle angle) {
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

        inputs.positionRads = pivotSimModel.getAngleRads();
        inputs.velocityRPM = pivotSimModel.getVelocityRadPerSec() * 30 / Math.PI;
        inputs.appliedVoltageVolts = pivotAppliedVoltage.in(Volts);
    }

    @Override
    public void holdDown(Angle angle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'holdDown'");
    }

}