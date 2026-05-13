package frc.robot.subsystems.topdeck.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class Shooter extends SubsystemBase {
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
   * Sets the shooter velocity in RPM based off of voltage.
   *
   * @param velocity Velocity in RPM
   */
  public void runVelocity(double velocity) {
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
    for (int i = 0; i < 4; i++) {
      shooterColums[i].setOpenLoop(output);
    }
  }

  /** Stops the shooter. */
  public void stop() {
    for (int i = 0; i < 4; i++) {
      shooterColums[i].stop();
    }
  }
}
