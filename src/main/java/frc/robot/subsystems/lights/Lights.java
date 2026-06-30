package frc.robot.subsystems.lights;

import static frc.robot.Constants.LightConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Lights extends SubsystemBase {
  private final Spark blinkin = new Spark(BLINKIN_PWM_PORT);
  private double currentPattern = Double.NaN;

  public Lights() {
    setPattern(LOST_DS_PATTERN);
  }

  @Override
  public void periodic() {
    setPattern(getRobotPattern());
    Logger.recordOutput("Lights/BlinkinPattern", currentPattern);
  }

  private double getRobotPattern() {
    if (!DriverStation.isDSAttached()) {
      return LOST_DS_PATTERN;
    }

    if (Constants.States.SHOOTER_ON) {
      return SHOOTER_PATTERN;
    }

    if (Constants.States.ADVANCER_ON) {
      return ADVANCER_PATTERN;
    }

    if (Constants.States.INTAKE_ON) {
      return INTAKE_PATTERN;
    }

    if (DriverStation.isAutonomous() && DriverStation.isEnabled()) {
      return AUTONOMOUS_PATTERN;
    }

    return IDLE_PATTERN;
  }

  public void setPattern(double pattern) {
    if (pattern == currentPattern) {
      return;
    }

    currentPattern = pattern;
    blinkin.set(pattern);
  }

  public boolean printHealth() {
    System.out.println("Lights:");
    System.out.printf("  Blinkin PWM %d: configured%n", BLINKIN_PWM_PORT);
    return true;
  }
}
