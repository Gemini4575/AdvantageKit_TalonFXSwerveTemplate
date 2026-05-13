package frc.robot.subsystems.topdeck.advancer;

import static frc.robot.Constants.AdvancerConstants.*;
import static frc.robot.subsystems.drive.DriveConstants.odometryFrequency;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.drive.SparkOdometryThread;
import java.util.Queue;
import java.util.function.DoubleSupplier;

public class AdvancerIOSparkFlex implements AdvancerIO {
  private final SparkFlex rollerMotor;
  private final RelativeEncoder rollerEncoder;
  private final SparkClosedLoopController rollerController;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> rollerPositionQueue;
  private final Queue<Double> rollerVelocityQueue;

  private final Debouncer rollerConnectedDebounce = new Debouncer(0.5);

  public AdvancerIOSparkFlex() {
    rollerMotor = new SparkFlex(ROLLER_MOTOR_ID, MotorType.kBrushless);
    rollerController = rollerMotor.getClosedLoopController();
    rollerEncoder = rollerMotor.getEncoder();

    var rollerConfig = new SparkFlexConfig();
    rollerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30, 30)
        .voltageCompensation(12)
        .inverted(ROLLER_MOTOR_INVERTED)
        .disableFollowerMode();
    rollerConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    rollerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    rollerConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        rollerMotor,
        5,
        () ->
            rollerMotor.configure(
                rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    rollerPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(rollerMotor, rollerEncoder::getPosition);
    rollerVelocityQueue =
        SparkOdometryThread.getInstance().registerSignal(rollerMotor, rollerEncoder::getVelocity);
  }

  @Override
  public void updateInputs(AdvancerIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(rollerMotor, rollerEncoder::getPosition, (value) -> inputs.advancerPositionRot = value);
    ifOk(rollerMotor, rollerEncoder::getVelocity, (value) -> inputs.advancerVelocityRPM = value);
    ifOk(
        rollerMotor,
        new DoubleSupplier[] {rollerMotor::getAppliedOutput, rollerMotor::getBusVoltage},
        (values) -> inputs.advancerAppliedVolts = values[0] * values[1]);
    ifOk(
        rollerMotor,
        rollerMotor::getOutputCurrent,
        (value) -> inputs.advancerSupplyCurrentAmps = value);
    inputs.advancerConnected = rollerConnectedDebounce.calculate(!sparkStickyFault);

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryAdvancerPositionsRot =
        rollerPositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryAdvancerVelocityRPM =
        rollerVelocityQueue.stream().mapToDouble((Double value) -> value).toArray();
    timestampQueue.clear();
    rollerPositionQueue.clear();
    rollerVelocityQueue.clear();
  }

  @Override
  public void setAdvancerOpenLoop(double output) {
    rollerMotor.set(output);
  }

  @Override
  public void setAdvancerVoltage(double volts) {
    rollerMotor.setVoltage(volts);
  }

  @Override
  public void setAdvancerVelocity(double velocityRotationsPerMin) {
    rollerController.setSetpoint(velocityRotationsPerMin, ControlType.kVelocity);
  }
}
