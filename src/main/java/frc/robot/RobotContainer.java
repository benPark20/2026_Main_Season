// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Indexer.SpinStageOne;
import frc.robot.commands.Indexer.SpinStageOneManual;
import frc.robot.commands.Indexer.SpinStageTwo;
import frc.robot.commands.Indexer.SpinStageTwoManual;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  Trigger m_ResetEncoderFieldRelative = new JoystickButton(m_driverController, 7);


  private final Indexer m_Indexer = new Indexer();
  
  // TODO : POSITIVE IS CORRECT WAY
  private final SpinStageOneManual m_IndexerStageOneManual = new SpinStageOneManual(m_Indexer, m_driverController);
  private final SpinStageTwoManual m_IndexerStageTwoManual = new SpinStageTwoManual(m_Indexer, m_driverController); // TODO change axis

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
      // Configure the button bindings
    configureBindings();

    // Configure default commands
   m_robotDrive.setDefaultCommand(
    //     // The left stick controls translation of the robot.
    //     // Turning is controlled by the X axis of the right stick.
          new RunCommand(  
            () -> m_robotDrive.drive(
              MathUtil.applyDeadband(m_driverController.getRawAxis(1), OIConstants.kDriveDeadband), //drive
              MathUtil.applyDeadband(m_driverController.getRawAxis(0), OIConstants.kDriveDeadband),
              MathUtil.applyDeadband(m_driverController.getRawAxis(4), OIConstants.kDriveDeadband), //rotation
                true),
            m_robotDrive));


            

  //new RunCommand( () -> m_robotDrive.setDutyCycle(0,.1),
  //    m_robotDrive));
  }

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
    m_Indexer.setDefaultCommand(m_IndexerStageOneManual); //TODO change to stage two for manual testing
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
//  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
  //  return Autos.exampleAuto(m_exampleSubsystem);
 // }
}
