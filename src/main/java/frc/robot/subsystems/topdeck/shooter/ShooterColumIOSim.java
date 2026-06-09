package frc.robot.subsystems.topdeck.shooter;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics sim implementation of shooter column IO. */
public class ShooterColumIOSim implements ShooterColumIO {
  private static final DCMotor shooterGearbox = DCMotor.getNEO(1);
  private static final double shooterMOI = 0.004;
  private static final double shooterReduction = 1.0;
  private static final double shooterSimP = SHOOTER_KP;
  private static final double shooterSimD = SHOOTER_KD;

  private final DCMotorSim shooterSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(shooterGearbox, shooterMOI, shooterReduction),
          shooterGearbox);
  private final PIDController shooterController = new PIDController(shooterSimP, 0.0, shooterSimD);

  private final double ks;
  private final double kv;

  private boolean shooterClosedLoop = false;
  private double shooterFFVolts = 0.0;
  private double shooterAppliedVolts = 0.0;

  public ShooterColumIOSim(int shooter) {
    ks =
        switch (shooter) {
          case 0 -> KS0;
          case 1 -> KS1;
          case 2 -> KS2;
          case 3 -> KS3;
          default -> throw new IllegalArgumentException("Invalid shooter number: " + shooter);
        };
    kv =
        switch (shooter) {
          case 0 -> KV0;
          case 1 -> KV1;
          case 2 -> KV2;
          case 3 -> KV3;
          default -> throw new IllegalArgumentException("Invalid shooter number: " + shooter);
        };
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double velocityRPM =
        Units.radiansPerSecondToRotationsPerMinute(shooterSim.getAngularVelocityRadPerSec());

    if (shooterClosedLoop) {
      shooterAppliedVolts = shooterFFVolts + shooterController.calculate(velocityRPM);
    } else {
      shooterController.reset();
    }

    shooterSim.setInputVoltage(MathUtil.clamp(shooterAppliedVolts, -12.0, 12.0));
    shooterSim.update(0.02);

    velocityRPM =
        Units.radiansPerSecondToRotationsPerMinute(shooterSim.getAngularVelocityRadPerSec());

    inputs.shooterConnected = true;
    inputs.shooterPositionRad = shooterSim.getAngularPositionRad();
    inputs.shooterVelocityRotPerSec = velocityRPM;
    inputs.shooterAppliedVolts = shooterAppliedVolts;
    inputs.shooterCurrentAmps = Math.abs(shooterSim.getCurrentDrawAmps());

    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryShooterPositionsRad = new double[] {inputs.shooterPositionRad};
    inputs.odometryShooterVelocityRot = new double[] {inputs.shooterVelocityRotPerSec};
  }

  @Override
  public void setShooterOpenLoop(double output) {
    shooterClosedLoop = false;
    shooterAppliedVolts = output;
  }

  @Override
  public void setShooterVelocity(double velocityRotationsPerMin) {
    shooterClosedLoop = true;
    shooterFFVolts =
        ks * Math.signum(velocityRotationsPerMin) + kv * (velocityRotationsPerMin / 60.0);
    shooterController.setSetpoint(velocityRotationsPerMin);
  }
}
