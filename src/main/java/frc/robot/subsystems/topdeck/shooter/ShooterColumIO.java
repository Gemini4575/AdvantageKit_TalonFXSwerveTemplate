package frc.robot.subsystems.topdeck.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterColumIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean shooterConnected = false;
    public double shooterVelocityRotPerSec = 0.0;
    public double shooterPositionRad = 0.0;
    public double shooterAppliedVolts = 0.0;
    public double shooterCurrentAmps = 0.0;
    public double shooterFeedForwardVoltage = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryShooterPositionsRad = new double[] {};
    public double[] odometryShooterVelocityRot = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run the shooter motor at the specified open loop value. */
  public default void setShooterOpenLoop(double output) {}

  /** Run the shooter motor at the specified velocity. */
  public default void setShooterVelocity(double velocityRotationsPerMin) {}
}
