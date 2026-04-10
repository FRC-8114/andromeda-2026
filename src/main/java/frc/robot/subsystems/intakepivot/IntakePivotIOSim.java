package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakePivotIOSim implements IntakePivotIO {
    private static final DCMotor PIVOT_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final double PIVOT_MOI_KG_M2 = 0.02;
    private static final double ARM_LENGTH_METERS = Inches.of(11.5).in(Meters);
    private static final double GEAR_RATIO = 11.625;

    private final SingleJointedArmSim pivotSimModel = new SingleJointedArmSim(
        PIVOT_GEARBOX,
        GEAR_RATIO,
        PIVOT_MOI_KG_M2,
        ARM_LENGTH_METERS,
        IntakePivot.Constants.DEPLOYED_ANGLE.in(Radians),
        IntakePivot.Constants.STOWED_ANGLE.in(Radians),
        true,
        IntakePivot.Constants.STOWED_ANGLE.in(Radians)
    );
    private final PIDController pivotSimController = new PIDController(6.0, 0.0, 0.2);

    private boolean isClosedLoop = false;
    private final MutVoltage pivotAppliedVoltage = Volts.mutable(0);
    private Angle targetAngle = IntakePivot.Constants.STOWED_ANGLE;

    @Override
    public void setTarget(Angle angle) {
        targetAngle = angle;
        pivotSimController.setSetpoint(angle.in(Radians));
        isClosedLoop = true;
    }

    @Override
    public void setTargetWithFeedForward(Angle angle, Current feedforward) {
        setTarget(angle); // feedforward not modeled in sim
    }

    @Override
    public void runVolts(Voltage volts) {
        isClosedLoop = false;
        pivotAppliedVoltage.mut_replace(volts);
    }

    @Override
    public void runCurrent(Current current) {
        // Approximate: V = I * R
        isClosedLoop = false;
        pivotAppliedVoltage.mut_replace(
            MathUtil.clamp(current.in(Amps) * PIVOT_GEARBOX.rOhms, -12.0, 12.0), Volts);
    }

    @Override
    public void stop() {
        isClosedLoop = false;
        pivotAppliedVoltage.mut_replace(0, Volts);
    }

    @Override
    public void updateInputs(IntakePivotInputs inputs) {
        if (isClosedLoop) {
            pivotAppliedVoltage.mut_replace(
                pivotSimController.calculate(pivotSimModel.getAngleRads()), Volts);
        } else {
            pivotSimController.reset();
        }

        pivotAppliedVoltage.mut_replace(
            MathUtil.clamp(pivotAppliedVoltage.in(Volts), -12.0, 12.0), Volts);
        pivotSimModel.setInputVoltage(pivotAppliedVoltage.in(Volts));
        pivotSimModel.update(0.02);

        inputs.targetAngleRadians.mut_replace(targetAngle);
        inputs.positionRadians.mut_replace(pivotSimModel.getAngleRads(), Radians);
        inputs.velocityRadPerSec.mut_replace(pivotSimModel.getVelocityRadPerSec(), RadiansPerSecond);
        inputs.voltageVolts.mut_replace(pivotAppliedVoltage);
        inputs.currentAmps.mut_replace(pivotSimModel.getCurrentDrawAmps(), Amps);
    }
}
