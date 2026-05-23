package frc.robot.commands.shooter;

import frc.robot.Constants.ShooterRPMConstants;
import frc.robot.commands.shooter.base.AutoShootBase;
import frc.robot.subsystems.topdeck.advancer.Advancer;
import frc.robot.subsystems.topdeck.shooter.Shooter;

public class ShootFromHubAuton extends AutoShootBase {
  public ShootFromHubAuton(Shooter s, Advancer advancer) {
    super(s, advancer, ShooterRPMConstants.HUB_SHOT);
    System.out.println("Shooting from hub auton");
  }
}
