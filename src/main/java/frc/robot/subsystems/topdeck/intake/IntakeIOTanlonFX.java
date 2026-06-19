package frc.robot.subsystems.topdeck.intake;

import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;

public class IntakeIOTanlonFX implements intakeIO {
  private final TalonFX intakeMotor1;
  private final TalonFX intakeMotor2;

  public IntakeIOTanlonFX() {
    intakeMotor1 = new TalonFX(TOP_INTAKE_MOTOR_ID);
    intakeMotor2 = new TalonFX(BOTTOM_INTAKE_MOTOR_ID);

    TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
    intakeMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    intakeMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeMotorConfig.CurrentLimits.StatorCurrentLimit = 80.0;
    intakeMotorConfig.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast;
    intakeMotorConfig.MotorOutput.Inverted =
        com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    intakeMotor1.getConfigurator().apply(intakeMotorConfig);
    intakeMotor2.getConfigurator().apply(intakeMotorConfig);
  }

  @Override
  public void updateInputs(intakeIOInputs inputs) {
    inputs.intakeConnected = true;
    inputs.intakeKrakenPositionRot =
        (intakeMotor1.getPosition().getValueAsDouble()
                + intakeMotor2.getPosition().getValueAsDouble())
            / 2.0;
    inputs.intakeKrakenVelocityRPM =
        (intakeMotor1.getVelocity().getValueAsDouble()
                + intakeMotor2.getVelocity().getValueAsDouble())
            * 30.0;
    inputs.intakeKrakenSupplyCurrentAmps =
        (intakeMotor1.getSupplyCurrent().getValueAsDouble()
                + intakeMotor2.getSupplyCurrent().getValueAsDouble())
            / 2.0;
    inputs.intakeKrakenStatorCurrentAmps =
        (intakeMotor1.getStatorCurrent().getValueAsDouble()
                + intakeMotor2.getStatorCurrent().getValueAsDouble())
            / 2.0;
    inputs.intakeKrakenAppliedVolts =
        (intakeMotor1.getMotorVoltage().getValueAsDouble()
                + intakeMotor2.getMotorVoltage().getValueAsDouble())
            / 2.0;
    inputs.odometryKrakenTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryKrakenPositionsRot = new double[] {inputs.intakeKrakenPositionRot};
    inputs.odometryKrakenVelocityRPM = new double[] {inputs.intakeKrakenVelocityRPM};
  }

  /**
   * Run the intake Krakens at a voltage
   *
   * @param voltage The voltage to run the intake motors at
   */
  @Override
  public void setKrakenVoltage(double voltage) {
    intakeMotor1.setVoltage(voltage);
    intakeMotor2.setVoltage(voltage);
  }

  /**
   * * Run the intake Krakens at a percetage
   *
   * @param output The percent that the motors run at 1.0 to -1.0
   */
  @Override
  public void setKrakenOpenLoop(double output) {
    intakeMotor1.set(output);
    intakeMotor2.set(output);
  }

  @Override
  public void setMusicTone(double frequencyHz) {
    MusicTone tone = new MusicTone(frequencyHz);
    intakeMotor1.setControl(tone);
    intakeMotor2.setControl(tone);
  }
}
