// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.DriverVision;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

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
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final Elevator elevator = new Elevator();
  private final Manipulator manipulator = new Manipulator();
  private final Pivot pivot = new Pivot();
  private final DriverVision visionSubsystem = new DriverVision();
  private final SendableChooser<Command> autoChooser;
  private final PhotonCamera photonCamera = new PhotonCamera("marlin");
  // private final Cameras camera = new Cameras();

  /*
   * new Rotation3d(0, Units.degreesToRadians(0), 0),
   * new Translation3d(Units.inchesToMeters(16),
   * Units.inchesToMeters(0),
   * Units.inchesToMeters(8)),
   * VecBuilder.fill(0,0, 0), VecBuilder.fill(0, 0, 0));
   */

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OperatorConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    //pivot.setDefaultCommand(new PivotPositionCommand(pivot, pivot.targetPos));

    autoChooser = AutoBuilder.buildAutoChooser("Leave Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> m_driverController.getLeftY() * -1,
      () -> m_driverController.getLeftX() * -1)
      .withControllerRotationAxis(m_driverController::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(m_driverController::getRightX,
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
    m_driverController.rightBumper()
    .whileTrue(new InstantCommand(() -> drivebase.changeSpeed(SpeedConstants.speedMax)))
    .whileFalse(new InstantCommand(() -> drivebase.changeSpeed(SpeedConstants.speedDefault)));

    m_driverController.leftBumper()
    .whileTrue(new InstantCommand(() -> drivebase.changeSpeed(SpeedConstants.speedMin)))    
    .whileFalse(new InstantCommand(() -> drivebase.changeSpeed(SpeedConstants.speedDefault)));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
     //m_driverController.y().whileTrue(new AimAtTarget(m_driverController, photonCamera, drivebase));

    m_operatorController.leftBumper().whileTrue(new RunCommand(() -> elevator.runUp(), elevator)).whileFalse(new RunCommand(() -> elevator.stop(), elevator));
    m_operatorController.leftTrigger().whileTrue(new RunCommand(() -> elevator.runDown(), elevator)).whileFalse(new RunCommand(() -> elevator.stop(), elevator));

     m_operatorController.povUp().whileTrue(
      new SequentialCommandGroup(
          new PivotPositionCommand(pivot, PivotConstants.L1_POSITION),
          new ParallelCommandGroup(
            new ElevatorPositionCommand(
              elevator, 
              ElevatorConstants.L1_ENCODER
            ),
            new PivotPositionCommand(pivot, PivotConstants.L1_POSITION, false)
          )
        )
      );
     m_operatorController.povRight().whileTrue(
        new SequentialCommandGroup(
          new PivotPositionCommand(pivot, PivotConstants.L2_POSITION),
          new ParallelCommandGroup(
            new ElevatorPositionCommand(
              elevator, 
              ElevatorConstants.L2_ENCODER
            ),
            new PivotPositionCommand(pivot, PivotConstants.L2_POSITION, false)
          )
        )
      );
     m_operatorController.povDown().whileTrue(
          new SequentialCommandGroup(
            new PivotPositionCommand(pivot, PivotConstants.L3_POSITION),
            new ParallelCommandGroup(
              new ElevatorPositionCommand(
                elevator, 
                ElevatorConstants.L3_ENCODER
              ),
              new PivotPositionCommand(pivot, PivotConstants.L3_POSITION, false)
            )
          )
        );
     m_operatorController.povLeft().whileTrue(
      new SequentialCommandGroup(
          new PivotPositionCommand(pivot, PivotConstants.L4_POSITION),
          new ParallelCommandGroup(
            new ElevatorPositionCommand(
              elevator, 
              ElevatorConstants.L4_ENCODER
            ),
            new PivotPositionCommand(pivot, PivotConstants.L4_POSITION, false)
          )
        )
      );

    m_operatorController.rightTrigger().whileTrue(new RunCommand(() -> pivot.runUp(), pivot)).whileFalse(new RunCommand(() -> pivot.stop(), pivot)); // Run Pivot Up
    m_operatorController.rightBumper().whileTrue(new RunCommand(() -> pivot.runDown(), pivot)).whileFalse(new RunCommand(() -> pivot.stop(), pivot)); // Run Pivot Down
  
m_operatorController.a().whileTrue(new RunCommand(() -> manipulator.runForward(0.5), manipulator)).whileFalse(new RunCommand(() -> manipulator.stop(), manipulator));
m_operatorController.b().whileTrue(new RunCommand(() -> manipulator.runBack(0.5), manipulator)).whileFalse(new RunCommand(() -> manipulator.stop(), manipulator));  
//m_operatorController.x().whileTrue(
//True(new UpperAlgae(elevator, manipulator));
m_operatorController.back().whileTrue(
  new SequentialCommandGroup(
          new PivotPositionCommand(pivot, PivotConstants.L2_POSITION), // change pivot position to the one that doesnt clip the elevator
          new ParallelCommandGroup(
            new ElevatorPositionCommand(
              elevator, 
              ElevatorConstants.LOWER_ALGAE_ENCODER
            ),
            new PivotPositionCommand(pivot, PivotConstants.L2_POSITION, false) // change pivot position to the one that doesnt clip the elevator
          )
        )
      );
m_operatorController.start().whileTrue(
  new SequentialCommandGroup(
    new PivotPositionCommand(pivot, PivotConstants.PIVOT_NOCLIP), // change pivot position to the one that doesnt clip the elevator
    new ParallelCommandGroup(
      new ElevatorPositionCommand(
        elevator, 
        ElevatorConstants.UPPER_ALGAE_ENCODER
      ),
      new PivotPositionCommand(pivot, PivotConstants.PIVOT_NOCLIP, false) // change pivot position to the one that doesnt clip the elevator
    )
  )
);
  
  
//m_operatorController.x().whileTrue(new IntakeCoral(elevator, manipulator, pivot)).whileFalse(new RunCommand(() -> manipulator.stop(), manipulator));
  m_operatorController.x().whileTrue(new SequentialCommandGroup(
    new PivotPositionCommand(pivot, PivotConstants.CORAL_STATION_POSITION),
    new ParallelCommandGroup(
      new ElevatorPositionCommand(
        elevator, 
        0.0
      ),
      new PivotPositionCommand(pivot, PivotConstants.CORAL_STATION_POSITION, false
      ), 
      new ManipulatorDirectionCommand(manipulator, pivot, -0.1)
    )
  )
); 

  }

  // /**
  // * Use this to pass the autonomous command to the main {@link Robot} class.
  // *
  // * @return the command to run in autonomous
  // */
  public Command getAutomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
