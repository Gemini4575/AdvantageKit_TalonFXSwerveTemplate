package frc.robot.subsystems.topdeck.intake;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.subsystems.drive.DriveConstants.odometryFrequency;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Timer;

public class IntakeIOSpark implements intakeIO {
  private final SparkMax rotatorMotor;

  public IntakeIOSpark() {
    rotatorMotor = new SparkMax(INTAKE_ROTATOR_CAN_ID, MotorType.kBrushless);
    SparkMaxConfig rotatorConfig = new SparkMaxConfig();
    rotatorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30, 30)
        .voltageCompensation(12)
        .inverted(true);
    rotatorConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    rotatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    rotatorConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        rotatorMotor,
        5,
        () ->
            rotatorMotor.configure(
                rotatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(intakeIOInputs inputs) {
    inputs.intakeConnected = true;
    inputs.intakePositionRot = rotatorMotor.getEncoder().getPosition();
    inputs.intakeVelocityRPM = rotatorMotor.getEncoder().getVelocity();
    inputs.intakeAppliedVolts = rotatorMotor.getAppliedOutput() * rotatorMotor.getBusVoltage();
    inputs.intakeSupplyCurrentAmps = rotatorMotor.getOutputCurrent();
    inputs.intakeStatorCurrentAmps = inputs.intakeSupplyCurrentAmps;
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryIntakePositionsRot = new double[] {inputs.intakePositionRot};
    inputs.odometryIntakeVelocityRPM = new double[] {inputs.intakeVelocityRPM};
  }

  @Override
  public void setIntakeOpenLoop(double output) {
    rotatorMotor.set(output);
  }

  @Override
  public boolean setIntakeOpenLoopUntilSetpointDown() {
    rotatorMotor.set(1.0);

    return rotatorMotor.getEncoder().getPosition() >= Intake_Down_SetPoint;
  }

  @Override
  public boolean setIntakeOpenLoopUntilSetpointUp() {
    rotatorMotor.set(-1.0);

    return rotatorMotor.getEncoder().getPosition() <= Intake_Up_SetPoint;
  }
}
