// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.intake.ExtendOrRectactIntake;
import frc.robot.commands.intake.IntakeDown;
import frc.robot.commands.intake.IntakeOff;
import frc.robot.commands.intake.IntakeOffAuton;
import frc.robot.commands.intake.IntakeOn;
import frc.robot.commands.intake.IntakeOnAuto;
import frc.robot.commands.intake.IntakeUp;
import frc.robot.commands.shooter.ShootFromHubAutoButActualy;
import frc.robot.commands.shooter.ShootFromHubAuton;
import frc.robot.commands.shooter.ShootFromHubTele;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.topdeck.advancer.Advancer;
import frc.robot.subsystems.topdeck.advancer.AdvancerIO;
import frc.robot.subsystems.topdeck.advancer.AdvancerIOSim;
import frc.robot.subsystems.topdeck.advancer.AdvancerIOSpark;
import frc.robot.subsystems.topdeck.advancer.AdvancerIOSparkFlex;
import frc.robot.subsystems.topdeck.advancer.AdvancerIOTalonFX;
import frc.robot.subsystems.topdeck.intake.Intake;
import frc.robot.subsystems.topdeck.intake.IntakeIOSim;
import frc.robot.subsystems.topdeck.intake.IntakeIOSpark;
import frc.robot.subsystems.topdeck.intake.IntakeIOTanlonFX;
import frc.robot.subsystems.topdeck.intake.intakeIO;
import frc.robot.subsystems.topdeck.shooter.Shooter;
import frc.robot.subsystems.topdeck.shooter.ShooterColumIO;
import frc.robot.subsystems.topdeck.shooter.ShooterColumIOSim;
import frc.robot.subsystems.topdeck.shooter.ShooterColumIOSpark;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Shooter shooter;
  private final Advancer advancer;
  private final Intake intake;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(2);
  private final Joystick operator = new Joystick(1);
  private final Joystick driver = new Joystick(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        shooter =
            new Shooter(
                new ShooterColumIOSpark(0),
                new ShooterColumIOSpark(1),
                new ShooterColumIOSpark(2),
                new ShooterColumIOSpark(3));
        advancer =
            new Advancer(new AdvancerIOTalonFX(), new AdvancerIOSpark(), new AdvancerIOSparkFlex());
        intake = new Intake(new IntakeIOSpark(), new IntakeIOTanlonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        shooter =
            new Shooter(
                new ShooterColumIOSim(0),
                new ShooterColumIOSim(1),
                new ShooterColumIOSim(2),
                new ShooterColumIOSim(3));
        advancer = new Advancer(new AdvancerIOSim(), new AdvancerIOSim(), new AdvancerIOSim());
        intake = new Intake(new IntakeIOSim(), new IntakeIOSim());

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        shooter =
            new Shooter(
                new ShooterColumIO() {},
                new ShooterColumIO() {},
                new ShooterColumIO() {},
                new ShooterColumIO() {});
        advancer = new Advancer(new AdvancerIO() {}, new AdvancerIO() {}, new AdvancerIO() {});
        intake = new Intake(new intakeIO() {}, new intakeIO() {});
        break;
    }
    // Set up auto routines after named commands are registered.
    NamedCommands.registerCommand(
        "Shoot From The Hub Auto", new ShootFromHubAuton(shooter, advancer).withTimeout(5));
    NamedCommands.registerCommand(
        "Shoot From The Hub", new ShootFromHubAutoButActualy(shooter, advancer).withTimeout(5));
    new EventTrigger("Off The Bump").onTrue(new IntakeDown(intake));
    new EventTrigger("Intake On").onTrue(new IntakeOn(intake).alongWith(new IntakeOnAuto()));
    new EventTrigger("Intake Off").onTrue(new IntakeOffAuton().alongWith(new IntakeOff(intake)));
    new EventTrigger("Shoot").whileTrue(new ShootFromHubTele(shooter, advancer));
    new EventTrigger("Intake Up").onTrue(new IntakeUp(intake));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> driver.getY(), () -> driver.getX(), () -> -driver.getTwist()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    new JoystickButton(driver, 6)
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive));

    new JoystickButton(operator, 11).whileTrue(new ShootFromHubTele(shooter, advancer));

    intake.setDefaultCommand(
        new ExtendOrRectactIntake(
            intake,
            () -> operator.getRawButton(12),
            () -> operator.getRawButton(13),
            () -> operator.getRawButton(20)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void printSubsystemHealth() {
    System.out.println("========== Subsystem Health Check   `=====");
    boolean driveOk = drive.printHealth();
    boolean shooterOk = shooter.printHealth();
    boolean advancerOk = advancer.printHealth();
    boolean intakeOk = intake.printHealth();
    boolean allGood = driveOk && shooterOk && advancerOk && intakeOk;
    System.out.println("Overall robot health: " + (allGood ? "GOOD" : "BAD"));
    System.out.println("============================================");
  }

  public Command getSubsystemTestCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                System.out.println(
                    "Keep the robot safely lifted/clear: running subsystem target checks.")),
        drive.motorResponseTest(),
        shooter.motorResponseTest(),
        advancer.motorResponseTest(),
        intake.motorResponseTest(),
        Commands.runOnce(() -> System.out.println("Subsystem motor response checks complete.")));
  }
}
