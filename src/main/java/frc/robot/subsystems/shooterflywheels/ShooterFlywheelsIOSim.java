package frc.robot.subsystems.shooterflywheels;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterFlywheelsIOSim implements ShooterFlywheelsIO {
    private static final DCMotor FLYWHEEL_GEARBOX = DCMotor.getKrakenX44Foc(1);
    private static final double GEAR_RATIO = 1.0;
    private static final double MOI = 0.001;
    private static final double KP = 3.0;

    private final DCMotorSim leftSim  = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(FLYWHEEL_GEARBOX, MOI, GEAR_RATIO), FLYWHEEL_GEARBOX);
    private final DCMotorSim rightSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(FLYWHEEL_GEARBOX, MOI, GEAR_RATIO), FLYWHEEL_GEARBOX);

    private final PIDController leftController  = new PIDController(KP, 0, 0);
    private final PIDController rightController = new PIDController(KP, 0, 0);

    private boolean closedLoop = false;
    private double leftVolts  = 0.0;
    private double rightVolts = 0.0;
    private AngularVelocity targetVelocity = RadiansPerSecond.of(0);

    public void setFlywheelVelocity(AngularVelocity velocity) {
        targetVelocity = velocity;
        leftController.setSetpoint(velocity.in(RadiansPerSecond));
        rightController.setSetpoint(velocity.in(RadiansPerSecond));
        closedLoop = true;
    }

    public void runVolts(Voltage volts) {
        closedLoop = false;
        leftVolts  = volts.in(Volts);
        rightVolts = volts.in(Volts);
    }

    public void runCurrent(Current current) {
        // Approximate: V = I * R
        closedLoop = false;
        leftVolts  = MathUtil.clamp(current.in(Amps) * FLYWHEEL_GEARBOX.rOhms, -12.0, 12.0);
        rightVolts = leftVolts;
    }

    public void stop() {
        closedLoop = false;
        leftVolts  = 0.0;
        rightVolts = 0.0;
    }

    public void updateInputs(ShooterFlywheelsInputs inputs) {
        if (closedLoop) {
            leftVolts  = leftController.calculate(leftSim.getAngularVelocityRadPerSec());
            rightVolts = rightController.calculate(rightSim.getAngularVelocityRadPerSec());
        } else {
            leftController.reset();
            rightController.reset();
        }

        leftSim.setInputVoltage(MathUtil.clamp(leftVolts, -12.0, 12.0));
        rightSim.setInputVoltage(MathUtil.clamp(rightVolts, -12.0, 12.0));
        leftSim.update(0.02);
        rightSim.update(0.02);

        inputs.targetVelocity.mut_replace(targetVelocity);

        inputs.leftVelocity.mut_replace(leftSim.getAngularVelocity());
        inputs.leftCurrent.mut_replace(Math.abs(leftSim.getCurrentDrawAmps()), Amps);
        inputs.leftAppliedVoltage.mut_replace(leftVolts, Volts);
        inputs.leftPosition.mut_replace(leftSim.getAngularPosition());

        inputs.rightVelocity.mut_replace(rightSim.getAngularVelocity());
        inputs.rightCurrent.mut_replace(Math.abs(rightSim.getCurrentDrawAmps()), Amps);
        inputs.rightAppliedVoltage.mut_replace(rightVolts, Volts);
        inputs.rightPosition.mut_replace(rightSim.getAngularPosition());
    }
}
