package frc.robot.commands.shooter;

import frc.robot.Constants.ShooterRPMConstants;
import frc.robot.commands.shooter.base.TelopShootBase;
import frc.robot.subsystems.topdeck.advancer.Advancer;
import frc.robot.subsystems.topdeck.shooter.Shooter;

public class ShootFromHubTele extends TelopShootBase {
  public ShootFromHubTele(Shooter s, Advancer ds) {
    super(s, ds, ShooterRPMConstants.HUB_SHOT);
  }
}
