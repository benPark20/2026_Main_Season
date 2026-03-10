// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drivetrain.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {

  private final DriveSubsystem m_drivesubsystem;

  private boolean outOfFrame;

  private boolean aligned;
  

  public AutoAlign(DriveSubsystem drivetrain) {

    m_drivesubsystem = drivetrain;

    //create a PositionVoltage 

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    outOfFrame = false;
    aligned = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rotation;
    boolean n = LimelightHelpers.getTV("limelight");
    
   // System.out.println("TV: " + n);

    if (n) {
      double XOffsetRotation = LimelightHelpers.getTX("limelight");

    //  rotation = rotationController.calculate(XOffsetRotation);

    //  System.out.println("Rotation:" + rotation);
    // System.out.println("Rotation XOffset: " + XOffsetRotation);

      if (Math.abs(XOffsetRotation) < 5) {
        outOfFrame = true;
        m_drivesubsystem.drive(0, 0, Constants.DriveConstants.kMaxAngularSpeed/2, false);
      } else if (outOfFrame == false) {
        m_drivesubsystem.drive(0, 0, 0, true);
      }
    }
    else if (outOfFrame == false) {
      m_drivesubsystem.drive(0, 0, 0, true);

      //rotation = 15 * (timer.get() + 3.5) * Math.sin(5 * (timer.get() + 3.5));
    } else {
      aligned = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    m_drivesubsystem.drive(0,0,0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   if (aligned) {
      return true;
    } else {
      return false;
    }  }

}
