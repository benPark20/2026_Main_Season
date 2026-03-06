// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveIntake extends Command {
  private Intake m_Intake;
  private double targetPosition;

  /** Creates a new MoveIntake. */
  public MoveIntake(Intake intake, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Intake = intake;
    targetPosition = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Intake.setActive(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Intake.moveIntakeToPosition(targetPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.setActive(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}
