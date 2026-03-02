// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class DriveSubsystem extends SubsystemBase {

  //TODO Find Chassis Offsets and Angular Offsets(offset)
  private final SwerveModule m_frontLeftModule = new SwerveModule(
   0,
   Constants.DriveConstants.FRONT_LEFT_DRIVE_ID, 
   Constants.DriveConstants.FRONT_LEFT_TURN_ID, 
   0, 
   null);

  //TODO Find Chassis Offsets and Angular Offsets(offset)
  private final SwerveModule m_frontRightModule = new SwerveModule(
    1,
    Constants.DriveConstants.FRONT_RIGHT_DRIVE_ID,
    Constants.DriveConstants.FRONT_RIGHT_TURN_ID,
    0, 
    null);

  //TODO Find Chassis Offsets and Angular Offsets(offset)
  private final SwerveModule m_backLeftModule = new SwerveModule(
    2, 
    Constants.DriveConstants.BACK_LEFT_DRIVE_ID, 
    Constants.DriveConstants.BACK_LEFT_TURN_ID, 
    0, 
    null);

  //TODO Find Chassis Offsets and Angular Offsets(offset)
  private final SwerveModule m_backRightModule = new SwerveModule(3, 
  Constants.DriveConstants.BACK_RIGHT_DRIVE_ID, 
  Constants.DriveConstants.BACK_RIGHT_TURN_ID, 
  0, 
  null);

  //TODO add/fix Gyro
  private Pigeon2 m_gyro = new Pigeon2(0); //TODO find gyro Device ID

  //TODO: add drivetrain kinematics to the Constants file
  
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeftModule.getSwerveModulePosition(),
          m_frontRightModule.getSwerveModulePosition(),
          m_backLeftModule.getSwerveModulePosition(),
          m_backRightModule.getSwerveModulePosition()
      });

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public Pose2d resetPose() {
  }

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

      /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */


    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
