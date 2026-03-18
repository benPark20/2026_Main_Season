// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Indexer.SpinStageOne;
import frc.robot.commands.Indexer.SpinStageTwo;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// This method most likely needs to be a ParallelCommandGroup that runs the shooter and drivetrain commands

public class ShootAtHub extends ParallelCommandGroup {

  private Shooter m_Shooter;
  private Indexer m_Indexer;
  private Drivetrain m_Drivetrain;

  private enum State {
    PRESHOOT, SHOOTING
  }

  private State state;

  /** Creates a new ShootAtHub. */
  public ShootAtHub(Shooter shooter, Indexer indexer) {
    m_Shooter = shooter;
    m_Indexer = indexer;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //addCommands(new SpinStageOne(indexer, 0.3), new SpinStageTwo(m_Indexer, 0.3), new BasicShoot(m_Shooter));

    addCommands(new BasicShoot(m_Shooter), new SequentialCommandGroup(new WaitCommand(1), new SpinStageTwo(indexer, 0.8), new SpinStageOne(indexer, 0.8)));
  }
}
/*
  @Override
  public void initialize() {
    m_Shooter.spinFlywheels(m_Shooter.getTargetPosition());
  }

  @Override
  public void execute() {
    
    if (m_Shooter.atTargetSpeed() && m_Shooter.atTargetPosition() && m_Drivetrain.atTargetPose()) {
      state = State.SHOOTING;
    } else {
      state = State.PRESHOOT;
    }

    switch (state) {
      case PRESHOOT:
        m_Shooter.spinFlywheels(m_Shooter.getHorizontalVelocity(0.5)); //TEMPORARY JUST TO GET IT TO BUILD!!!
        m_Shooter.pivotShooter(m_Shooter.getTargetPosition());
        // Drivetrain method to aim towards hub
        
        break;
      case SHOOTING:
        m_Indexer.runStageOneMotor(Constants.IndexerConstants.STAGE_ONE_INTAKE_SPEED);
        m_Indexer.runStageTwoMotor(Constants.IndexerConstants.STAGE_TWO_INTAKE_SPEED);
        break;
    }
  }

  @Override
    public void end(boolean interrupted) {

    }
*/