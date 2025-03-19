// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.L1;
import frc.robot.commands.L2;
import frc.robot.commands.L3;
import frc.robot.commands.L4;
import frc.robot.commands.*;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final Elevator elevator = new Elevator();
  private final Manipulator manipulator = new Manipulator();
  private final Pivot pivot = new Pivot();
  // private final SendableChooser<Command> autoChooser;
  // private final PhotonCamera photonCamera = new PhotonCamera("marlin");
  //private final Cameras camera = new Cameras();

  /*
   * new Rotation3d(0, Units.degreesToRadians(0), 0),
        new Translation3d(Units.inchesToMeters(16),
        Units.inchesToMeters(0),
        Units.inchesToMeters(8)),
        VecBuilder.fill(0,0, 0), VecBuilder.fill(0, 0, 0));
   */


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

    // autoChooser = AutoBuilder.buildAutoChooser("New Auto");

    // SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(), 
  () -> m_driverController.getLeftY() * -1, 
  () -> m_driverController.getLeftX() * -1)
  .withControllerRotationAxis(m_driverController::getRightX)
  .deadband(OperatorConstants.DEADBAND)
  .scaleTranslation(0.8)
  .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
   m_driverController::getRightY)
   .headingWhile(true);

    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), 0.1),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), 0.1),
        () -> -MathUtil.applyDeadband(m_driverController.getRightY(), 0.0),
        () -> -MathUtil.applyDeadband(m_driverController.getRightX(), 0.0));

  

  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    //Configure two bindings, call the new method to change the maximum speed in SwerveSubsystem in both. For the "turbo" one set the maximum speed to 1.0 and "slow" to 0.1
    m_driverController.rightTrigger()
    .whileTrue(new InstantCommand(() -> drivebase.changeSpeed(SpeedConstants.speedMax)))
    .whileFalse(new InstantCommand(() -> drivebase.changeSpeed(SpeedConstants.speedDefault)));

    m_driverController.leftTrigger()
    .whileTrue(new InstantCommand(() -> drivebase.changeSpeed(SpeedConstants.speedMin)))    
    .whileFalse(new InstantCommand(() -> drivebase.changeSpeed(SpeedConstants.speedDefault)));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.y().whileTrue(new AimAtTarget(m_driverController, photonCamera, drivebase));
    m_driverController.a().whileTrue(new RunCommand(() -> {System.out.println("demoooo");}));

    m_operatorController.leftBumper().whileTrue(new RunCommand(() -> elevator.runUp(), elevator)).whileFalse(new RunCommand(() -> elevator.stop(), elevator));
    m_operatorController.rightBumper().whileTrue(new RunCommand(() -> elevator.runDown(), elevator)).whileFalse(new RunCommand(() -> elevator.stop(), elevator));


    m_operatorController.a().whileTrue(new L1(elevator, manipulator, pivot));
    m_operatorController.b().whileTrue(new L2(elevator, manipulator, pivot));
    m_operatorController.x().whileTrue(new L3(elevator, manipulator, pivot));
    m_operatorController.y().whileTrue(new L4(elevator, manipulator, pivot));

    m_operatorController.leftTrigger().whileTrue(new RunCommand(() -> pivot.runUp(), pivot)).whileFalse(new RunCommand(() -> pivot.stop(), pivot));
    m_operatorController.rightTrigger().whileTrue(new RunCommand(() -> pivot.runDown(), pivot)).whileFalse(new RunCommand(() -> pivot.stop(), pivot));
  
    m_operatorController.back().whileTrue(new RunCommand(() -> manipulator.runForward(-0.1), manipulator)).whileFalse(new RunCommand(() -> manipulator.stop(), manipulator));
    m_operatorController.start().whileTrue(new IntakeCoral(elevator, manipulator, pivot));
    
  }

  // /**
  //  * Use this to pass the autonomous command to the main {@link Robot} class.
  //  *
  //  * @return the command to run in autonomous
  //  */
  // public Command getAutomousCommand() {
  //   // An example command will be run in autonomous
  //   return autoChooser.getSelected();
  // }
}
