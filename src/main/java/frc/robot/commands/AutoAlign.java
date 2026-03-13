// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import java.util.concurrent.BlockingDeque;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drivetrain.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {

  private final DriveSubsystem m_drivesubsystem;
  
  private PIDController rotationController;

  public AutoAlign(DriveSubsystem drivetrain) {

    m_drivesubsystem = drivetrain;

    rotationController = new PIDController(Constants.AutoConstants.AUTO_ALIGN_KP, 
      Constants.AutoConstants.AUTO_ALIGN_KI, Constants.AutoConstants.AUTO_ALIGN_KD);
  }

  private boolean isInFrame() {
//    LimelightHelpers.setAlignIDs(10,25);
    return LimelightHelpers.getTV("limelight-shooter");
  }

  private double getError() {
    System.out.println(LimelightHelpers.getTX("limelight-shooter"));
    return LimelightHelpers.getTX("limelight-shooter");
  }

  public double 
  getOutput() {
    return rotationController.calculate(getError(),0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  public boolean isOfftarget() {
    return Math.abs(getError()) > .7;
  }

  public boolean isOntarget() {
    return Math.abs(getError()) < .7;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    SmartDashboard.putBoolean("Target", isInFrame()); 
    SmartDashboard.putNumber("TX", getError());
   // System.out.println("TV: " + n);

    if (isInFrame()) {
      if (isOfftarget()) {
        m_drivesubsystem.drive(0, 0,getOutput(),true);
      } else {
        m_drivesubsystem.drive(0, 0, 0, true);
      }
    }
    else {
      m_drivesubsystem.drive(0, 0, Constants.DriveConstants.kMaxAngularSpeed/32, true);
    }

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    m_drivesubsystem.drive(0,0,0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
 // LimelightHelpers.resetLimelightIDs();
  if (isInFrame()) {
    if (isOntarget()) {
      return true;
    }
    else {
      return false;
    }
  }
  else {
    return false;
  }
  }


  
}
