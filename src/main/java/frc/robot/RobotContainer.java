package frc.robot;

import java.rmi.dgc.VMID;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriverVision;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ElevatorCommands.ElevatorManualControl;
import frc.robot.commands.ElevatorCommands.ElevatorSetpointCommand;
import frc.robot.commands.ManipulatorCommands.ManipulatorCommand;
import frc.robot.commands.PivotCommands.PivotManualControl;

public class RobotContainer {
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final PivotSubsystem pivot = new PivotSubsystem();
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final Manipulator manipulator = new Manipulator();
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final DriverVision vision = new DriverVision(); 
  

  public RobotContainer() {
    configureBindings();
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
      () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), 0.1),
      () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), 0.1),
      () -> MathUtil.applyDeadband(m_driverController.getRightY(), 0.0),
      () -> MathUtil.applyDeadband(m_driverController.getRightX(), 0.0));

  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  private void configureBindings() {
    // Example: Manual control; back runs up.
    m_operatorController.back().whileTrue(new ElevatorManualControl(elevator, () -> 0.1));
    
    // Example: Manual control; runs down
    m_operatorController.start().whileTrue(new ElevatorManualControl(elevator, () -> -0.1));
    
    // Example: Move to a specific setpoint
    m_operatorController.a().onTrue(
        new ElevatorSetpointCommand(elevator, 20));

    // m_operatorController.b().onTrue(new PivotManualControl(pivot, m_operatorController::getLeftX));

    //Manipulator
    m_operatorController.x().whileTrue(new ManipulatorCommand(manipulator, ManipulatorConstants.CORAL_INTAKE_SPEED));

    // m_operatorController.x() 
    // TODO: PivotSetpointControl
  }

  // // /**
  // // * Use this to pass the autonomous command to the main {@link Robot} class.
  // // *
  // // * @return the command to run in autonomous
  // // */
  // public Command getAutomousCommand() {
  //   // An example command will be run in autonomous
  //   return autoChooser.getSelected();
  // }
}