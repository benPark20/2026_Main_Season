// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Indexer.SpinStageOne;
import frc.robot.commands.Indexer.SpinStageTwo;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootAtHub extends ParallelCommandGroup {

  private Shooter m_Shooter;
  private Indexer m_Indexer;


  /** Creates a new ShootAtHub. */
  public AutoShootAtHub(Shooter shooter, Indexer indexer) {
    m_Shooter = shooter;
    m_Indexer = indexer;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //addCommands(new SpinStageOne(indexer, 0.3), new SpinStageTwo(m_Indexer, 0.3), new BasicShoot(m_Shooter));

    addCommands(new BasicShoot(m_Shooter), new SequentialCommandGroup(new WaitCommand(1), new SpinStageTwo(indexer, 0.5)), new SpinStageOne(indexer, 0.35));
  }
}
