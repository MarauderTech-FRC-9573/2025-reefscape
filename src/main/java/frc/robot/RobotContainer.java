package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.ElevatorCommands.ElevatorManualControl;
import frc.robot.commands.ElevatorCommands.ElevatorSetpointCommand;
import frc.robot.commands.ManipulatorCommands.ManipulatorCommand;
import frc.robot.commands.PivotCommands.PivotManualControl;
import frc.robot.commands.PivotCommands.PivotSetpointCommand;

public class RobotContainer {
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final PivotSubsystem pivot = new PivotSubsystem();
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_operatorController = new CommandXboxController(
            OperatorConstants.kOperatorControllerPort);
    private final Manipulator manipulator = new Manipulator();
    private final SwerveSubsystem drivebase = new SwerveSubsystem();
    private final DriverVision vision = new DriverVision();
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
        configureBindings();
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
            () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), 0.1),
            () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), 0.1),
            () -> MathUtil.applyDeadband(m_driverController.getRightY(), 0.0),
            () -> MathUtil.applyDeadband(m_driverController.getRightX(), 0.0));

    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    private void configureBindings() {
        // Configure two bindings, call the new method to change the maximum speed in
        // SwerveSubsystem in both. For the "turbo" one set the maximum speed to 1.0 and
        // "slow" to 0.1
        m_driverController.leftBumper()
                .whileTrue(new InstantCommand(() -> drivebase.changeSpeed(SpeedConstants.speedMax)))
                .whileFalse(new InstantCommand(() -> drivebase.changeSpeed(SpeedConstants.speedDefault)));

        m_driverController.rightBumper()
                .whileTrue(new InstantCommand(() -> drivebase.changeSpeed(SpeedConstants.speedMin)))
                .whileFalse(new InstantCommand(() -> drivebase.changeSpeed(SpeedConstants.speedDefault)));

        // Example: Manual control; back runs up.
        m_operatorController.back().whileTrue(new ElevatorManualControl(elevator, () -> 0.1));

        // Example: Manual control; runs down
        m_operatorController.start().whileTrue(new ElevatorManualControl(elevator, () -> -0.1));

        // Example: Move to a specific setpoint
        m_operatorController.a().onTrue(
                new ElevatorSetpointCommand(elevator, 20));

        // m_operatorController.b().onTrue(new PivotManualControl(pivot,
        // m_operatorController::getLeftX));

        m_operatorController.leftTrigger().whileTrue(new PivotManualControl(pivot, PivotConstants.PIVOT_SPEED_DOWN));

        m_operatorController.rightTrigger().whileTrue(new PivotManualControl(pivot, PivotConstants.PIVOT_SPEED_UP));

        // Manipulator
        m_operatorController.y()
                .whileTrue(new ManipulatorCommand(manipulator, ManipulatorConstants.CORAL_INTAKE_SPEED));

        m_operatorController.x().whileTrue(new ManipulatorCommand(manipulator, ManipulatorConstants.ALGAE_INTAKE_SPEED));

        m_operatorController.a().whileTrue(new ManipulatorCommand(manipulator, ManipulatorConstants.CORAL_SCORE_SPEED));

        m_operatorController.b()
                .whileTrue(new ManipulatorCommand(manipulator, ManipulatorConstants.ALGAE_SCORE_SPEED));

        // m_operatorController.x()
        // TODO: PivotSetpointControl

        // Need new tunings

        // L1
        m_operatorController.povUp()
                .whileTrue(new PivotSetpointCommand(pivot, PivotConstants.L1_POSITION)
                        .andThen(new ElevatorSetpointCommand(elevator, ElevatorConstants.L1_ENCODER)
                                .andThen(new ManipulatorCommand(manipulator, ManipulatorConstants.CORAL_SCORE_SPEED))));

        // L2
        m_operatorController.povRight()
                .whileTrue(new PivotSetpointCommand(pivot, PivotConstants.L2_POSITION)
                        .andThen(new ElevatorSetpointCommand(elevator, ElevatorConstants.L2_ENCODER)
                                .andThen(new ManipulatorCommand(manipulator, ManipulatorConstants.CORAL_SCORE_SPEED))));

        // L3
        m_operatorController.povDown()
                .whileTrue(new PivotSetpointCommand(pivot, PivotConstants.L3_POSITION)
                        .andThen(new ElevatorSetpointCommand(elevator, ElevatorConstants.L3_ENCODER)
                                .andThen(new ManipulatorCommand(manipulator, ManipulatorConstants.CORAL_SCORE_SPEED))));

        // L4
        m_operatorController.povLeft()
                .whileTrue(new PivotSetpointCommand(pivot, PivotConstants.L4_POSITION)
                        .andThen(new ElevatorSetpointCommand(elevator, ElevatorConstants.L4_ENCODER)
                                .andThen(new ManipulatorCommand(manipulator, ManipulatorConstants.CORAL_SCORE_SPEED))));
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