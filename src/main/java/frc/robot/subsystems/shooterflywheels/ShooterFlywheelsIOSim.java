package frc.robot.subsystems.shooterflywheels;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterFlywheelsIOSim implements ShooterFlywheelsIO {
    private static final DCMotor MOTOR_GEARBOX = DCMotor.getKrakenX44Foc(1);
    private static final double GEAR_RATIO = 1.0;
    private static final double MOI = 0.001;

    private static final double KP = 3;

    private final DCMotorSim leftSim;
    private final DCMotorSim rightSim;

    private final PIDController leftController = new PIDController(KP, 0, 0);
    private final PIDController rightController = new PIDController(KP, 0, 0);

    private boolean closedLoop = false;
    private double leftVolts = 0.0;
    private double rightVolts = 0.0;

    public ShooterFlywheelsIOSim() {
        leftSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(MOTOR_GEARBOX, MOI, GEAR_RATIO),
                MOTOR_GEARBOX);
        rightSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(MOTOR_GEARBOX, MOI, GEAR_RATIO),
                MOTOR_GEARBOX);
    }

    public void setFlywheelVelocity(AngularVelocity velocity) {
        closedLoop = true;
        double setpointRadPerSec = velocity.in(RPM) / 60.0 * 2.0 * Math.PI;
        leftController.setSetpoint(setpointRadPerSec);
        rightController.setSetpoint(setpointRadPerSec);
    }

    public void runVolts(Voltage volts) {
        closedLoop = false;
        leftVolts = volts.in(Volts);
        rightVolts = volts.in(Volts);
    }

    public void stopFlywheels() {
        closedLoop = false;
        leftVolts = 0.0;
        rightVolts = 0.0;
    }

    public void updateInputs(ShooterInputs inputs) {
        if (closedLoop) {
            leftVolts = leftController.calculate(leftSim.getAngularVelocityRadPerSec());
            rightVolts = rightController.calculate(rightSim.getAngularVelocityRadPerSec());
        } else {
            leftController.reset();
            rightController.reset();
        }

        leftSim.setInputVoltage(MathUtil.clamp(leftVolts, -12.0, 12.0));
        rightSim.setInputVoltage(MathUtil.clamp(rightVolts, -12.0, 12.0));
        leftSim.update(0.02);
        rightSim.update(0.02);

        inputs.leftFlywheelRPMs = leftSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI) * 60.0;
        inputs.leftAppliedVoltage = leftVolts;
        inputs.leftCurrentAmps = Math.abs(leftSim.getCurrentDrawAmps());

        inputs.rightFlywheelRPMs = rightSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI) * 60.0;
        inputs.rightAppliedVoltage = rightVolts;
        inputs.rightCurrentAmps = Math.abs(rightSim.getCurrentDrawAmps());
    }
}
