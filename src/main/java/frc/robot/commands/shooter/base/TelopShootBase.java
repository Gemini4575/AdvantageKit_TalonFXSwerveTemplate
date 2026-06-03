package frc.robot.commands.shooter.base;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.advancer.Advancer;
import frc.robot.subsystems.topdeck.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class TelopShootBase extends Command {
  private final Shooter s;
  private final DoubleSupplier velocitySupplier;
  private final Advancer sadfd;
  private Timer timer = new Timer();
  private boolean firstTime = false;

  public TelopShootBase(Shooter a, Advancer ds, DoubleSupplier velocitySupplier) {
    s = a;
    sadfd = ds;
    this.velocitySupplier = velocitySupplier;
    addRequirements(a, ds);
  }

  @Override
  public void initialize() {
    firstTime = false;
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    double velocity = velocitySupplier.getAsDouble();
    s.runVelocity(velocity);
    if (s.getAverageRPM() > velocity - 100 || firstTime) {
      firstTime = true;
      sadfd.advance();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    firstTime = false;
    s.stop();
    sadfd.stopAdvancer();
  }
}
