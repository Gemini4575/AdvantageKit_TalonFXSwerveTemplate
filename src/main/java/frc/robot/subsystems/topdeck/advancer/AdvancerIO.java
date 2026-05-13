package frc.robot.subsystems.topdeck.advancer;

import org.littletonrobotics.junction.AutoLog;

public interface AdvancerIO {
  @AutoLog
  public static class AdvancerIOInputs {
    public boolean advancerConnected = false;
    public double advancerPositionRot = 0.0;
    public double advancerVelocityRPM = 0.0;
    public double advancerSupplyCurrentAmps = 0.0;
    public double advancerStatorCurrentAmps = 0.0;
    public double advancerAppliedVolts = 0.0;
    public double advancerTemperatureCelsius = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryAdvancerPositionsRot = new double[] {};
    public double[] odometryAdvancerVelocityRPM = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(AdvancerIOInputs inputs) {}

  /** Run the advancer motor at the specified open loop percent output. */
  public default void setAdvancerOpenLoop(double output) {}

  /** Run the advancer motor at the specified voltage. */
  public default void setAdvancerVoltage(double volts) {}

  /** Run the advancer motor at the specified velocity. */
  public default void setAdvancerVelocity(double velocityRotationsPerMin) {}
}
