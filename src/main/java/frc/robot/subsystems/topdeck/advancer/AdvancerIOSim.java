package frc.robot.subsystems.topdeck.advancer;

import static frc.robot.Constants.AdvancerConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics sim implementation of advancer IO. */
public class AdvancerIOSim implements AdvancerIO {
  private static final DCMotor advancerGearbox = DCMotor.getNEO(1);
  private static final double advancerMOI = 0.004;
  private static final double advancerReduction = 1.0;
  private static final double advancerSimP = 0.01;
  private static final double advancerSimD = 0.0;

  private final DCMotorSim advancerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(advancerGearbox, advancerMOI, advancerReduction),
          advancerGearbox);
  private final PIDController advancerController =
      new PIDController(advancerSimP, 0.0, advancerSimD);

  private boolean advancerClosedLoop = false;
  private double advancerFFVolts = 0.0;
  private double advancerAppliedVolts = 0.0;

  @Override
  public void updateInputs(AdvancerIOInputs inputs) {
    double velocityRPM =
        Units.radiansPerSecondToRotationsPerMinute(advancerSim.getAngularVelocityRadPerSec());

    if (advancerClosedLoop) {
      advancerAppliedVolts = advancerFFVolts + advancerController.calculate(velocityRPM);
    } else {
      advancerController.reset();
    }

    advancerAppliedVolts = MathUtil.clamp(advancerAppliedVolts, -12.0, 12.0);
    advancerSim.setInputVoltage(advancerAppliedVolts);
    advancerSim.update(0.02);

    velocityRPM =
        Units.radiansPerSecondToRotationsPerMinute(advancerSim.getAngularVelocityRadPerSec());

    inputs.advancerConnected = true;
    inputs.advancerPositionRot = Units.radiansToRotations(advancerSim.getAngularPositionRad());
    inputs.advancerVelocityRPM = velocityRPM;
    inputs.advancerAppliedVolts = advancerAppliedVolts;
    inputs.advancerSupplyCurrentAmps = Math.abs(advancerSim.getCurrentDrawAmps());
    inputs.advancerStatorCurrentAmps = inputs.advancerSupplyCurrentAmps;

    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryAdvancerPositionsRot = new double[] {inputs.advancerPositionRot};
    inputs.odometryAdvancerVelocityRPM = new double[] {inputs.advancerVelocityRPM};
  }

  @Override
  public void setAdvancerOpenLoop(double output) {
    advancerClosedLoop = false;
    advancerAppliedVolts = output * 12.0;
  }

  @Override
  public void setAdvancerVoltage(double volts) {
    advancerClosedLoop = false;
    advancerAppliedVolts = volts;
  }

  @Override
  public void setAdvancerVelocity(double velocityRotationsPerMin) {
    advancerClosedLoop = true;
    advancerFFVolts =
        ADVANCER_KS * Math.signum(velocityRotationsPerMin) + ADVANCER_KV * velocityRotationsPerMin;
    advancerController.setSetpoint(velocityRotationsPerMin);
  }
}
