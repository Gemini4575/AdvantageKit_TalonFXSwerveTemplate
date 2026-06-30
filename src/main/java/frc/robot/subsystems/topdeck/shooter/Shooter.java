package frc.robot.subsystems.topdeck.shooter;

import static frc.robot.Constants.ShooterRPMConstants.HUB_SHOT;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class Shooter extends SubsystemBase {
  private static final double TEST_TARGET_RPM = HUB_SHOT;
  private static final double TEST_RPM_TOLERANCE = 50.0;

  static final Lock odometryLock = new ReentrantLock();
  private final ShooterColum[] shooterColums = new ShooterColum[4];

  public Shooter(
      ShooterColumIO OPshooterColumIO,
      ShooterColumIO IPshooterColimIO,
      ShooterColumIO IGShooterColumIO,
      ShooterColumIO OGShooterColumIO) {
    shooterColums[0] = new ShooterColum(OPshooterColumIO, 0);
    shooterColums[1] = new ShooterColum(IPshooterColimIO, 1);
    shooterColums[2] = new ShooterColum(IGShooterColumIO, 2);
    shooterColums[3] = new ShooterColum(OGShooterColumIO, 3);
  }

  @Override
  public void periodic() {
    odometryLock.lock();
    try {
      for (var colum : shooterColums) {
        colum.periodic();
      }
    } finally {
      odometryLock.unlock();
    }

    // Stop the shooter if the robots disabled duh
    if (DriverStation.isDisabled()) {
      for (var colum : shooterColums) {
        colum.stop();
      }
    }
  }

  /**
   * Sets the shooter velocity in RPM.
   *
   * @param velocity Velocity in RPM
   */
  public void runVelocity(double velocity) {
    Constants.States.SHOOTER_ON = Math.abs(velocity) > 0.0;
    for (int i = 0; i < 4; i++) {
      shooterColums[i].setVelocity(velocity);
    }
  }

  /**
   * Sets the shooter voltage in volts.
   *
   * @param output Voltage in volts
   */
  public void setOpenLoop(double output) {
    Constants.States.SHOOTER_ON = Math.abs(output) > 0.0;
    for (int i = 0; i < 4; i++) {
      shooterColums[i].setOpenLoop(output);
    }
  }

  /** Stops the shooter. */
  public void stop() {
    Constants.States.SHOOTER_ON = false;
    for (int i = 0; i < 4; i++) {
      shooterColums[i].stop();
    }
  }

  public double getAverageRPM() {
    double totalRPM = 0.0;
    int count = 0;
    for (int i = 0; i < shooterColums.length; i++) {
      if (shooterColums[i].isConnected()) {
        totalRPM += shooterColums[i].getVelocityRPM();
        count++;
      }
    }
    return count > 0 ? totalRPM / count : 0.0;
  }

  public boolean printHealth() {
    boolean allGood = true;

    System.out.println("Shooter:");
    for (int i = 0; i < shooterColums.length; i++) {
      boolean columnOk = shooterColums[i].isConnected();
      System.out.println("  Column " + i + ": " + (columnOk ? "GOOD" : "BAD"));
      allGood = allGood && columnOk;
    }

    return allGood;
  }

  public Command motorResponseTest() {
    return Commands.sequence(
            Commands.runOnce(
                () ->
                    System.out.printf(
                        "Shooter velocity check: target=%.0f RPM, tolerance=+/-%.0f RPM%n",
                        TEST_TARGET_RPM, TEST_RPM_TOLERANCE)),
            Commands.run(() -> runVelocity(TEST_TARGET_RPM), this).withTimeout(2.0),
            Commands.runOnce(this::printMotorResponseHealth))
        .finallyDo(this::stop);
  }

  private void printMotorResponseHealth() {
    boolean allGood = true;

    for (int i = 0; i < shooterColums.length; i++) {
      double appliedVolts = Math.abs(shooterColums[i].getAppliedVolts());
      double velocityRPM = Math.abs(shooterColums[i].getVelocityRPM());
      double errorRPM = Math.abs(velocityRPM - TEST_TARGET_RPM);
      boolean columnOk = shooterColums[i].isConnected() && errorRPM <= TEST_RPM_TOLERANCE;

      System.out.printf(
          "  Shooter column %d velocity: %s (target=%.0f RPM, measured=%.0f RPM, error=%.0f RPM,"
              + " applied=%.2f V)%n",
          i, columnOk ? "GOOD" : "BAD", TEST_TARGET_RPM, velocityRPM, errorRPM, appliedVolts);
      allGood = allGood && columnOk;
    }

    System.out.println("Shooter velocity overall: " + (allGood ? "GOOD" : "BAD"));
  }
}
