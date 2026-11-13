package frc.robot.subsystems.topdeck.intake;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.subsystems.drive.DriveConstants.odometryFrequency;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Timer;

/** Spark MAX implementation for the two NEOs mechanically coupled to the intake shaft. */
public class IntakeIONeo implements intakeIO {
  private final SparkMax intakeLeader = new SparkMax(INTAKE_LEADER_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax intakeFollower =
      new SparkMax(INTAKE_FOLLOWER_MOTOR_ID, MotorType.kBrushless);

  public IntakeIONeo() {
    var leaderConfig = new SparkMaxConfig();
    leaderConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(30)
        .voltageCompensation(12.0)
        .inverted(false);
    leaderConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        intakeLeader,
        5,
        () ->
            intakeLeader.configure(
                leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    var followerConfig = new SparkMaxConfig();
    // The motors face opposite directions on the shared shaft.
    followerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(30)
        .voltageCompensation(12.0)
        .follow(intakeLeader, true);
    tryUntilOk(
        intakeFollower,
        5,
        () ->
            intakeFollower.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(intakeIOInputs inputs) {
    inputs.intakeConnected =
        intakeLeader.getLastError() == REVLibError.kOk
            && intakeFollower.getLastError() == REVLibError.kOk;
    inputs.intakeKrakenPositionRot =
        (intakeLeader.getEncoder().getPosition() + intakeFollower.getEncoder().getPosition()) / 2.0;
    inputs.intakeKrakenVelocityRPM =
        (intakeLeader.getEncoder().getVelocity() + intakeFollower.getEncoder().getVelocity()) / 2.0;
    inputs.intakeKrakenSupplyCurrentAmps =
        (intakeLeader.getOutputCurrent() + intakeFollower.getOutputCurrent()) / 2.0;
    inputs.intakeKrakenStatorCurrentAmps = inputs.intakeKrakenSupplyCurrentAmps;
    inputs.intakeKrakenAppliedVolts =
        (intakeLeader.getAppliedOutput() * intakeLeader.getBusVoltage()
                + intakeFollower.getAppliedOutput() * intakeFollower.getBusVoltage())
            / 2.0;
    inputs.odometryKrakenTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryKrakenPositionsRot = new double[] {inputs.intakeKrakenPositionRot};
    inputs.odometryKrakenVelocityRPM = new double[] {inputs.intakeKrakenVelocityRPM};
  }

  @Override
  public void setKrakenVoltage(double voltage) {
    intakeLeader.setVoltage(voltage);
  }

  @Override
  public void setKrakenOpenLoop(double output) {
    intakeLeader.set(output);
  }
}
