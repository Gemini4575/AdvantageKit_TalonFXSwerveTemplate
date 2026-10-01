package frc.robot.subsystems.topdeck.intake;

import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.Timer;

public class IntakeIOTanlonFX implements intakeIO {
  private final TalonFX intakeLeader;
  private final TalonFX intakeFollower;

  public IntakeIOTanlonFX() {
    intakeLeader = new TalonFX(INTAKE_LEADER_MOTOR_ID);
    intakeFollower = new TalonFX(INTAKE_FOLLOWER_MOTOR_ID);

    TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
    intakeMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    intakeMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeMotorConfig.CurrentLimits.StatorCurrentLimit = 80.0;
    intakeMotorConfig.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast;
    intakeMotorConfig.MotorOutput.Inverted =
        com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    intakeLeader.getConfigurator().apply(intakeMotorConfig);
    intakeFollower.getConfigurator().apply(intakeMotorConfig);

    // Both Krakens drive the same shaft, so the second motor mirrors the leader in hardware.
    intakeFollower.setControl(new Follower(INTAKE_LEADER_MOTOR_ID, MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(intakeIOInputs inputs) {
    inputs.intakeConnected = true;
    inputs.intakeKrakenPositionRot =
        (intakeLeader.getPosition().getValueAsDouble()
                + intakeFollower.getPosition().getValueAsDouble())
            / 2.0;
    inputs.intakeKrakenVelocityRPM =
        (intakeLeader.getVelocity().getValueAsDouble()
                + intakeFollower.getVelocity().getValueAsDouble())
            * 30.0;
    inputs.intakeKrakenSupplyCurrentAmps =
        (intakeLeader.getSupplyCurrent().getValueAsDouble()
                + intakeFollower.getSupplyCurrent().getValueAsDouble())
            / 2.0;
    inputs.intakeKrakenStatorCurrentAmps =
        (intakeLeader.getStatorCurrent().getValueAsDouble()
                + intakeFollower.getStatorCurrent().getValueAsDouble())
            / 2.0;
    inputs.intakeKrakenAppliedVolts =
        (intakeLeader.getMotorVoltage().getValueAsDouble()
                + intakeFollower.getMotorVoltage().getValueAsDouble())
            / 2.0;
    inputs.odometryKrakenTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryKrakenPositionsRot = new double[] {inputs.intakeKrakenPositionRot};
    inputs.odometryKrakenVelocityRPM = new double[] {inputs.intakeKrakenVelocityRPM};
  }

  /**
   * Run the primary intake Kraken at a voltage. The second Kraken follows it in hardware.
   *
   * @param voltage The voltage to run the intake motors at
   */
  @Override
  public void setKrakenVoltage(double voltage) {
    intakeLeader.setVoltage(voltage);
  }

  /**
   * Run the primary intake Kraken at a percentage. The second Kraken follows it in hardware.
   *
   * @param output The percent that the motors run at 1.0 to -1.0
   */
  @Override
  public void setKrakenOpenLoop(double output) {
    intakeLeader.set(output);
  }

  @Override
  public void setMusicTone(double frequencyHz) {
    MusicTone tone = new MusicTone(frequencyHz);
    intakeLeader.setControl(tone);
  }
}
