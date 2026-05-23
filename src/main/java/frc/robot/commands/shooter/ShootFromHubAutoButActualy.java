package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterRPMConstants;
import frc.robot.subsystems.topdeck.advancer.Advancer;
import frc.robot.subsystems.topdeck.shooter.Shooter;

public class ShootFromHubAutoButActualy extends Command {

  private final Shooter shooter;
  private final Advancer advancer;
  private Timer timer = new Timer();

  public ShootFromHubAutoButActualy(Shooter s, Advancer a) {
    this.shooter = s;
    this.advancer = a;
    addRequirements(a, s);
  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() {
    shooter.runVelocity(ShooterRPMConstants.HUB_SHOT);
    advancer.advance();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    advancer.stopAdvancer();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(5);
  }
}
