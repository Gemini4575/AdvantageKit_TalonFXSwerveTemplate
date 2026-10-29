package frc.robot.subsystems.topdeck.elevator;

import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleSupplier;

/** Spark MAX implementation for the two motors mechanically coupled to the elevator gearbox. */
public class ElevatorIOSpark implements ElevatorIO {
  private final SparkMax leader = new SparkMax(leaderCanId, MotorType.kBrushless);
  private final SparkMax follower = new SparkMax(followerCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();
  private final SparkClosedLoopController controller = leader.getClosedLoopController();

  public ElevatorIOSpark() {
    var leaderConfig = new SparkMaxConfig();
    leaderConfig
        .idleMode(IdleMode.kBrake)
        .inverted(leaderInverted)
        .smartCurrentLimit(currentLimitAmps)
        .voltageCompensation(12.0);
    leaderConfig
        .encoder
        .positionConversionFactor(metersPerMotorRotation)
        .velocityConversionFactor(metersPerMotorRotation / 60.0);
    leaderConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kP, kI, kD);
    leaderConfig
        .softLimit
        .forwardSoftLimit(maxHeightMeters)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(minHeightMeters)
        .reverseSoftLimitEnabled(true);

    tryUntilOk(
        leader,
        5,
        () ->
            leader.configure(
                leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    var followerConfig = new SparkMaxConfig();
    followerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimitAmps)
        .voltageCompensation(12.0)
        .follow(leader, followerInverted);
    tryUntilOk(
        follower,
        5,
        () ->
            follower.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    ifOk(leader, encoder::getPosition, value -> inputs.positionMeters = value);
    ifOk(leader, encoder::getVelocity, value -> inputs.velocityMetersPerSec = value);
    ifOk(
        leader,
        new DoubleSupplier[] {leader::getAppliedOutput, leader::getBusVoltage},
        values -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(
        leader,
        new DoubleSupplier[] {leader::getOutputCurrent, follower::getOutputCurrent},
        values -> inputs.currentAmps = values);
    ifOk(
        leader,
        new DoubleSupplier[] {leader::getMotorTemperature, follower::getMotorTemperature},
        values -> inputs.tempCelsius = values);
    inputs.leaderConnected = leader.getLastError() == REVLibError.kOk;
    inputs.followerConnected = follower.getLastError() == REVLibError.kOk;
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setPosition(double positionMeters, double feedforwardVolts) {
    controller.setSetpoint(
        positionMeters, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforwardVolts);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }
}
