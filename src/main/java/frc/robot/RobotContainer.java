// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.shooter.ManualShoot;
import frc.robot.commands.shooter.ReadyShooter;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.StadiaController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  // The robot's subsystems and commands are defined here...

  public PS4Controller m_driverController = new PS4Controller(0);

  public Shooter m_Shooter = new Shooter();
  public Trigger preFire = new Trigger(() -> m_Shooter.isHubAlmostActive());
  
  public ManualShoot m_ManualShoot = new ManualShoot(m_Shooter, m_driverController);
  public ReadyShooter m_ReadyShooter = new ReadyShooter(m_Shooter);
  public Shoot m_Shoot = new Shoot(m_Shooter);
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    preFire.onTrue(m_ReadyShooter);
    // In RobotContainer.java (in the constructor or a setup method)
    m_chooser.setDefaultOption("Simple Auto", m_Shoot);

    // Configure the trigger bindings
    configureBindings();
    // In RobotContainer.java


    //m_Shooter.setDefaultCommand(m_Shoot);
   // NamedCommands.registerCommand("Shooter", m_Shoot);
   // chooser.addOption("Auto Name", new PathPlannerAuto("Auto Name"));
   // SmartDashboard.putData("Auto Mode", chooser);

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
    frc.robot.Button.leftTrigger1.whileTrue(m_Shoot);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // TODO: get actual automomous command to return
    //     return null;
      return m_chooser.getSelected();
  }
}
