package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ElevatorCommands.ElevatorManualControl;
import frc.robot.commands.ElevatorCommands.ElevatorSetpointCommand;
import frc.robot.commands.PivotCommands.PivotManualControl;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class RobotContainer {
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final PivotSubsystem pivot = new PivotSubsystem();
  private final CommandXboxController m_operatorController = new CommandXboxController(0);


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Example: Manual control using joystick input
    m_operatorController.y().whileTrue(new ParallelCommandGroup(
        new ElevatorManualControl(elevator, m_operatorController::getLeftY),
        new RunCommand(() -> SmartDashboard.putNumber("Left y: ", m_operatorController.getLeftY()))));
    // Example: Move to a specific setpoint
    m_operatorController.a().onTrue(
        new ElevatorSetpointCommand(elevator, ElevatorConstants.UPPER_ALGAE_ENCODER));

    m_operatorController.b().onTrue(new PivotManualControl(pivot, m_operatorController::getLeftX));

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