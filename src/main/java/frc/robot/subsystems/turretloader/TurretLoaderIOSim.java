package frc.robot.subsystems.turretloader;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretLoaderIOSim implements TurretLoaderIO {
    private static final DCMotor MOTOR_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final double GEAR_RATIO = 1.0;
    private static final double MOI = 0.001;

    private static final double KP = 0.1;

    private final DCMotorSim motorSim;
    private final PIDController motorController = new PIDController(KP, 0, 0);

    private boolean closedLoop = false;
    private double appliedVolts = 0.0;

    public TurretLoaderIOSim() {
        motorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(MOTOR_GEARBOX, MOI, GEAR_RATIO),
                MOTOR_GEARBOX);
    }

    public void runVolts(Voltage volts) {
        closedLoop = false;
        appliedVolts = volts.in(Volts);
    }

    public void setVelocity(AngularVelocity velocity) {
        closedLoop = true;
        double setpointRadPerSec = velocity.in(RPM) / 60.0 * 2.0 * Math.PI;
        motorController.setSetpoint(setpointRadPerSec);
    }

    public void stopMotor() {
        closedLoop = false;
        appliedVolts = 0.0;
    }

    public void updateInputs(TurretLoaderInputs inputs) {
        if (closedLoop) {
            appliedVolts = motorController.calculate(motorSim.getAngularVelocityRadPerSec());
        } else {
            motorController.reset();
        }

        motorSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
        motorSim.update(0.02);

        inputs.motorPositionRads = motorSim.getAngularPositionRad();
        inputs.velocityRPM = motorSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI) * 60.0;
        inputs.appliedVoltageVolts = appliedVolts;
    }

    @Override
    public void runTorqueCurrent(Current torque) {
        closedLoop = false;
        appliedVolts = MathUtil.clamp(torque.baseUnitMagnitude() * 0.1, -12.0, 12.0);
    }

    @Override
    public void runDutyCycle() {
        closedLoop = false;
        appliedVolts = 12.0;
    }
}
