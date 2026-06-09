package frc.robot.subsystems.topdeck.shooter;

import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.subsystems.drive.DriveConstants.odometryFrequency;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.drive.SparkOdometryThread;
import java.util.Queue;
import java.util.function.DoubleSupplier;

public class ShooterColumIOSpark implements ShooterColumIO {
  private final SparkFlex shooterMotor;
  private final RelativeEncoder shooterEncoder;
  private final SimpleMotorFeedforward shooterFeedforward;
  private final PIDController shooterController;
  private final boolean shooterInverted;
  private final double ka;
  private final double kv;
  private final double ks;

  private boolean shooterClosedLoop = false;
  private double shooterFeedforwardVoltage = 0.0;
  private double shooterAppliedVolts = 0.0;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> shooterPositionQueue;
  private final Queue<Double> shooterVelocityQueue;

  private final Debouncer shooterConnectedDebounce = new Debouncer(0.5);

  public ShooterColumIOSpark(int shooter) {
    shooterInverted =
        switch (shooter) {
          case 0 -> SHOOTER_MOTOR_0_INVERTED;
          case 1 -> SHOOTER_MOTOR_1_INVERTED;
          case 2 -> SHOOTER_MOTOR_2_INVERTED;
          case 3 -> SHOOTER_MOTOR_3_INVERTED;
          default -> throw new IllegalArgumentException("Invalid shooter number: " + shooter);
        };
    shooterMotor =
        switch (shooter) {
          case 0 -> new SparkFlex(
              SHOOTER_MOTOR_ID_0, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
          case 1 -> new SparkFlex(
              SHOOTER_MOTOR_ID_1, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
          case 2 -> new SparkFlex(
              SHOOTER_MOTOR_ID_2, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
          case 3 -> new SparkFlex(
              SHOOTER_MOTOR_ID_3, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
          default -> throw new IllegalArgumentException("Invalid shooter number: " + shooter);
        };
    ka =
        switch (shooter) {
          case 0 -> KA0;
          case 1 -> KA1;
          case 2 -> KA2;
          case 3 -> KA3;
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
    ks =
        switch (shooter) {
          case 0 -> KS0;
          case 1 -> KS1;
          case 2 -> KS2;
          case 3 -> KS3;
          default -> throw new IllegalArgumentException("Invalid shooter number: " + shooter);
        };
    shooterEncoder = shooterMotor.getEncoder();
    shooterFeedforward = new SimpleMotorFeedforward(ks, kv, ka);
    shooterController = new PIDController(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD);
    // Configure shooter motor
    var shooterConfig = new SparkFlexConfig();
    shooterConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(30, 30)
        .voltageCompensation(12)
        .inverted(shooterInverted);
    shooterConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    shooterConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    shooterConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    // tryUntilOk(
    // shooterMotor,
    // 5,
    // () ->
    shooterMotor.configure(
        shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // );

    // Create odometry queues
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    shooterPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(shooterMotor, shooterEncoder::getPosition);
    shooterVelocityQueue =
        SparkOdometryThread.getInstance().registerSignal(shooterMotor, shooterEncoder::getVelocity);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    sparkStickyFault = false;
    // Update shooter inputs
    sparkStickyFault = false;
    ifOk(shooterMotor, shooterEncoder::getPosition, (value) -> inputs.shooterPositionRad = value);
    ifOk(
        shooterMotor,
        shooterEncoder::getVelocity,
        (value) -> {
          inputs.shooterVelocityRotPerSec = value;
          if (shooterClosedLoop) {
            shooterAppliedVolts =
                MathUtil.clamp(
                    shooterFeedforwardVoltage + shooterController.calculate(value), -12.0, 12.0);
            shooterMotor.setVoltage(shooterAppliedVolts);
          }
        });
    ifOk(
        shooterMotor,
        new DoubleSupplier[] {shooterMotor::getAppliedOutput, shooterMotor::getBusVoltage},
        (values) -> inputs.shooterAppliedVolts = values[0] * values[1]);
    ifOk(
        shooterMotor, shooterMotor::getOutputCurrent, (value) -> inputs.shooterCurrentAmps = value);
    inputs.shooterConnected = shooterConnectedDebounce.calculate(!sparkStickyFault);

    inputs.shooterFeedForwardVoltage = shooterFeedforwardVoltage;
    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryShooterPositionsRad =
        shooterPositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryShooterVelocityRot =
        shooterVelocityQueue.stream().mapToDouble((Double value) -> value).toArray();
    timestampQueue.clear();
    shooterPositionQueue.clear();
    shooterVelocityQueue.clear();
  }

  @Override
  public void setShooterOpenLoop(double output) {
    shooterClosedLoop = false;
    shooterController.reset();
    shooterMotor.setVoltage(output);
  }

  @Override
  public void setShooterVelocity(double velocityRotationsPerMin) {
    shooterClosedLoop = true;
    shooterController.reset();
    shooterFeedforwardVoltage = shooterFeedforward.calculate(velocityRotationsPerMin / 60.0);
    shooterController.setSetpoint(velocityRotationsPerMin);
  }
}
