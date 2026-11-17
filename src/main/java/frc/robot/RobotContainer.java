// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.topdeck.intake.Intake;
import frc.robot.subsystems.topdeck.intake.IntakeIOSim;
import frc.robot.subsystems.topdeck.intake.IntakeIOSpark;
import frc.robot.subsystems.topdeck.intake.IntakeIOTanlonFX;
import frc.robot.subsystems.topdeck.intake.intakeIO;
import frc.robot.util.AutoPreview;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Declares the robot's active subsystems, controls, and autonomous chooser. */
public class RobotContainer {
  private static final double SLOW_DRIVE_SCALE = 0.5;

  private final Drive drive;
  private final Intake intake;
  private final Joystick driver = new Joystick(0);
  private final LoggedDashboardChooser<Command> autoChooser;
  private final AutoPreview autoPreview;

  /** The container for the robot. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        intake = new Intake(new IntakeIOSpark(), new IntakeIOTanlonFX());
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        intake = new Intake(new IntakeIOSim(), new IntakeIOSim());
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        intake = new Intake(new intakeIO() {}, new intakeIO() {});
        break;
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoPreview = new AutoPreview(autoChooser);
    SmartDashboard.putData("Auto Choices", autoChooser.getSendableChooser());
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getY(), () -> -driver.getX(), () -> -driver.getTwist()));

    new Trigger(() -> driver.getPOV() == 0)
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> driver.getY() * SLOW_DRIVE_SCALE,
                () -> driver.getX() * SLOW_DRIVE_SCALE,
                () -> -driver.getTwist() * SLOW_DRIVE_SCALE));

    new JoystickButton(driver, 8).onTrue(Commands.runOnce(drive::stopWithX, drive));
    new JoystickButton(driver, 6)
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive));
    new JoystickButton(driver, 7)
        .whileTrue(
            DriveCommands.joystickDriveAtRelativeAngle(
                drive, () -> driver.getY(), () -> driver.getX(), Rotation2d.fromDegrees(90.0)));
    new JoystickButton(driver, 12)
        .whileTrue(
            DriveCommands.joystickDriveAtRelativeAngle(
                drive, () -> driver.getY(), () -> driver.getX(), Rotation2d.fromDegrees(180.0)));
  }

  /**
   * @return the autonomous command selected on the dashboard.
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Updates the dashboard autonomous preview. Call this while disabled. */
  public void periodic() {
    autoPreview.periodic();
  }

  public void printSubsystemHealth() {
    System.out.println("========== Subsystem Health Check ==========");
    boolean driveOk = drive.printHealth();
    boolean intakeOk = intake.printHealth();
    System.out.println("Overall robot health: " + (driveOk && intakeOk ? "GOOD" : "BAD"));
    System.out.println("============================================");
  }

  public Command getSubsystemTestCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> System.out.println("Keep the robot safely lifted: running subsystem checks.")),
        drive.motorResponseTest(),
        intake.motorResponseTest(),
        Commands.runOnce(() -> System.out.println("Subsystem checks complete.")));
  }
}
