// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.autos.SimpleCoralAuton;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.RollerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final RollerSubsystem rollerSubsystem = new RollerSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.DRIVER_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  // Initializing autons
  private final SimpleCoralAuton m_simpleCoralAuton = new SimpleCoralAuton(driveSubsystem, rollerSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up command bindings
    configureBindings();

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption

    // This auton hasn NOT been tested!
    autoChooser.setDefaultOption("Simple Coral Auton", m_simpleCoralAuton);
    SmartDashboard.putData(autoChooser);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // before
    driverController.rightBumper()
        .whileTrue(new RollerCommand(() -> 0, () -> RollerConstants.ROLLER_INTAKE_VALUE, rollerSubsystem));
    driverController.rightTrigger(0.2)
        .whileTrue(new RollerCommand(() -> RollerConstants.ROLLER_EJECT_VALUE, () -> 0, rollerSubsystem));

    driverController.leftBumper()
        .whileTrue(new ArmCommand(() -> ArmConstants.ARM_UP_VALUE, () -> 0, armSubsystem));
    driverController.leftTrigger(0.2)
        .whileTrue(new ArmCommand(() -> 0, () -> ArmConstants.ARM_DOWN_VALUE, armSubsystem));

    driverController.povDown()
        .whileTrue(new ClimbCommand(() -> ClimbConstants.CLIMB_VALUE, () -> 0, climbSubsystem));
    driverController.povUp()
        .whileTrue(new ClimbCommand(() -> 0, () -> ClimbConstants.CLIMB_VALUE, climbSubsystem));

    // Set the default command for the drive subsystem to an instance of the
    // DriveCommand with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value). Similarly for the X axis where we need to flip the value so the
    // joystick matches the WPILib convention of counter-clockwise positive.
    // Pressing right bumper allows for full speed motion. Otherwise, the drivetrain
    // operates at half speed.
    driveSubsystem.setDefaultCommand(new DriveCommand(
        () -> -driverController.getLeftY() *
            (driverController.getHID().getRightBumperButton() ? -0.5 : -1),
        () -> -driverController.getRightX() *
            (driverController.getHID().getRightBumperButton() ? -0.6 : -1),
        driveSubsystem));


    // Set default commands of the subsystems to do nothing while not called
    rollerSubsystem.setDefaultCommand(new RollerCommand(
        () -> 0,
        () -> 0,
        rollerSubsystem));

    armSubsystem.setDefaultCommand(new ArmCommand(
        () -> 0.08,
        () -> 0,
        armSubsystem));

    climbSubsystem.setDefaultCommand(new ClimbCommand(
        () -> 0,
        () -> 0,
        climbSubsystem));
    
    }

    

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
