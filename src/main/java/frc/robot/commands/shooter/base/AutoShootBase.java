package frc.robot.commands.shooter.base;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.advancer.Advancer;
import frc.robot.subsystems.topdeck.shooter.Shooter;

public class AutoShootBase extends Command {
  private final Shooter s;
  public int Velocity = 6784;
  private final Advancer sadfd;
  private Timer timer = new Timer();

  public AutoShootBase(Shooter a, Advancer ds, int velocity) {
    s = a;
    sadfd = ds;
    Velocity = velocity;
    addRequirements(a, ds);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    s.runVelocity(Velocity);
    sadfd.advance();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(5);
  }

  @Override
  public void end(boolean interrupted) {
    s.stop();
    sadfd.stopAdvancer();
  }
}
