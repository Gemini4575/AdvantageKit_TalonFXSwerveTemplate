package frc.robot.subsystems.topdeck.advancer;

import static frc.robot.Constants.NeoAdvancerConstants.*;
import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.drive.SparkOdometryThread;
import java.util.Queue;
import java.util.function.DoubleSupplier;

public class AdvancerIOSpark implements AdvancerIO {
  private final SparkMax advancerMotor;
  private final RelativeEncoder advancerEncoder;
  private final SparkClosedLoopController advancerController;
  private final boolean advancerInverted;
  private final double ka;
  private final double kv;
  private final double ks;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> advancerPositionQueue;
  private final Queue<Double> advancerVelocityQueue;

  private final Debouncer advancerConnectedDebounce = new Debouncer(0.5);

  public AdvancerIOSpark() {
    advancerInverted = NEO_ADVANCER_MOTOR_INVERTED;
    advancerMotor =
        new SparkMax(
            NEO_ADVANCER_MOTOR_ID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    ka = KA;
    kv = KV;
    ks = KS;

    advancerController = advancerMotor.getClosedLoopController();
    advancerEncoder = advancerMotor.getEncoder();
    // Configure advancer motor
    var advancerConfig = new SparkMaxConfig();
    advancerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(30, 30)
        .voltageCompensation(12)
        .inverted(advancerInverted);
    advancerConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    advancerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    advancerConfig.closedLoop.feedForward.kV(kv);
    advancerConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        advancerMotor,
        5,
        () ->
            advancerMotor.configure(
                advancerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Create odometry queues
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    advancerPositionQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(advancerMotor, advancerEncoder::getPosition);
    advancerVelocityQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(advancerMotor, advancerEncoder::getVelocity);
  }

  @Override
  public void updateInputs(AdvancerIOInputs inputs) {
    sparkStickyFault = false;
    // Update advancer inputs
    sparkStickyFault = false;
    ifOk(
        advancerMotor, advancerEncoder::getPosition, (value) -> inputs.advancerPositionRot = value);
    ifOk(
        advancerMotor, advancerEncoder::getVelocity, (value) -> inputs.advancerVelocityRPM = value);
    ifOk(
        advancerMotor,
        new DoubleSupplier[] {advancerMotor::getAppliedOutput, advancerMotor::getBusVoltage},
        (values) -> inputs.advancerAppliedVolts = values[0] * values[1]);
    ifOk(
        advancerMotor,
        advancerMotor::getOutputCurrent,
        (value) -> inputs.advancerStatorCurrentAmps = value);
    inputs.advancerConnected = advancerConnectedDebounce.calculate(!sparkStickyFault);
    // Update odometry inputs
    inputs.odometryAdvancerPositionsRot =
        advancerPositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryAdvancerVelocityRPM =
        advancerVelocityQueue.stream().mapToDouble((Double value) -> value).toArray();
    timestampQueue.clear();
    advancerPositionQueue.clear();
    advancerVelocityQueue.clear();
  }

  @Override
  public void setAdvancerOpenLoop(double output) {
    advancerMotor.set(output);
  }

  @Override
  public void setAdvancerVoltage(double volts) {
    advancerMotor.setVoltage(volts);
  }

  @Override
  public void setAdvancerVelocity(double velocityRotationsPerMin) {
    advancerController.setSetpoint(velocityRotationsPerMin, ControlType.kVelocity);
  }
}
