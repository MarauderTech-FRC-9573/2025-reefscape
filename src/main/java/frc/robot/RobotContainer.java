package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ElevatorCommands.ElevatorManualControl;
import frc.robot.commands.ElevatorCommands.ElevatorSetpointCommand;

public class RobotContainer {
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final CommandXboxController m_operatorController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Example: Manual control using joystick input
    m_operatorController.y().whileTrue(
        new ElevatorManualControl(elevator, m_operatorController::getLeftY));

    // Example: Move to a specific setpoint
    m_operatorController.a().onTrue(
        new ElevatorSetpointCommand(elevator, ElevatorConstants.UPPER_ALGAE_ENCODER));
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