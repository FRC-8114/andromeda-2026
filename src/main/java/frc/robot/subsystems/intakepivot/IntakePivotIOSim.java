package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakePivotIOSim implements IntakePivotIO {
    private static final DCMotor DEPLOY_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final double DEPLOY_GEAR_RATIO = 25.0;
    private static final double DEPLOY_MOI = 0.01;

    private static final double DEPLOY_KP = 5.0;

    private final DCMotorSim deploySim;

    private final PIDController deployController = new PIDController(DEPLOY_KP, 0, 0);
    
    private boolean deployClosedLoop = false;
    private double deployAppliedVolts = 0.0;

    public IntakePivotIOSim() {
        deploySim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DEPLOY_GEARBOX, DEPLOY_MOI, DEPLOY_GEAR_RATIO),
            DEPLOY_GEARBOX);   
    }

    public void setTarget(Angle angle) {
        deployClosedLoop = true;
        deployController.setSetpoint(angle.in(Radians));
    }

    public void runVolts(Voltage volts) {
        deployClosedLoop = false;
        deployAppliedVolts = volts.in(Volts);
    }

        public void updateInputs(IntakePivotInputs inputs) {
        if (deployClosedLoop) {
            deployAppliedVolts = deployController.calculate(deploySim.getAngularPositionRad());
        } else {
            deployController.reset();
        }

        deploySim.setInputVoltage(MathUtil.clamp(deployAppliedVolts, -12.0, 12.0));
        deploySim.update(0.02);

        inputs.positionRads = deploySim.getAngularPositionRad();
        inputs.velocityRPM = deploySim.getAngularVelocityRPM();
        inputs.appliedVoltageVolts = deployAppliedVolts;
    }
}