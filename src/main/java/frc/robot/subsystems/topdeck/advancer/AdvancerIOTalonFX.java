package frc.robot.subsystems.topdeck.advancer;

import static frc.robot.Constants.AdvancerConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class AdvancerIOTalonFX implements AdvancerIO {
  private final TalonFX advancerMotor = new TalonFX(ADVANCER_MOTOR_ID);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  private final StatusSignal<Angle> position = advancerMotor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = advancerMotor.getVelocity();
  private final StatusSignal<Voltage> appliedVoltage = advancerMotor.getMotorVoltage();
  private final StatusSignal<Current> supplyCurrent = advancerMotor.getSupplyCurrent();
  private final StatusSignal<Current> statorCurrent = advancerMotor.getStatorCurrent();
  private final StatusSignal<Temperature> temperature = advancerMotor.getDeviceTemp();

  public AdvancerIOTalonFX() {
    var advancerConfig = new TalonFXConfiguration();
    advancerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    advancerConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    advancerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    advancerConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    advancerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    advancerConfig.Slot0.kS = 0.11525;
    advancerConfig.Slot0.kV = 0.00196;
    advancerConfig.MotorOutput.Inverted =
        ADVANCER_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    advancerMotor.getConfigurator().apply(advancerConfig);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVoltage, supplyCurrent, statorCurrent, temperature);
    advancerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(AdvancerIOInputs inputs) {
    inputs.advancerConnected =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVoltage, supplyCurrent, statorCurrent, temperature)
            .equals(StatusCode.OK);
    inputs.advancerPositionRot = position.getValueAsDouble();
    inputs.advancerVelocityRPM = velocity.getValueAsDouble() * 60.0;
    inputs.advancerAppliedVolts = appliedVoltage.getValueAsDouble();
    inputs.advancerSupplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.advancerStatorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.advancerTemperatureCelsius = temperature.getValueAsDouble();
  }

  @Override
  public void setAdvancerOpenLoop(double output) {
    advancerMotor.set(output);
  }

  @Override
  public void setAdvancerVoltage(double volts) {
    advancerMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setAdvancerVelocity(double velocityRotationsPerMin) {
    advancerMotor.setControl(velocityRequest.withVelocity(velocityRotationsPerMin / 60.0));
  }

  @Override
  public void setMusicTone(double frequencyHz) {
    advancerMotor.setControl(new MusicTone(frequencyHz));
  }
}
