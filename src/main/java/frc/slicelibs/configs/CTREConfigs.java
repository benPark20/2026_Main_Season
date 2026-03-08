// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.slicelibs.configs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants;


/** Add your docs here. */
public class CTREConfigs {

    /* Initiating the different TalonFX configurators */
    public final TalonFXConfiguration m_swerveDriveConfigs = new TalonFXConfiguration();
    public final TalonFXConfiguration m_swerveTurnConfigs = new TalonFXConfiguration();

    public final TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();
    public final TalonFXConfiguration shooterRightConfigs = new TalonFXConfiguration();

    public final TalonFXConfiguration m_intakeConfigs = new TalonFXConfiguration();


    public CTREConfigs(){


        ////////////////////////////
        /// Swerve Drive Configs ///
        ////////////////////////////
        
        var swerveDrivePIDs = m_swerveDriveConfigs.Slot0;
        swerveDrivePIDs.kP = Constants.DriveConstants.DRIVE_KP;
        swerveDrivePIDs.kI = Constants.DriveConstants.DRIVE_KI;
        swerveDrivePIDs.kD = Constants.DriveConstants.DRIVE_KD;

        
        var swerveDriveCurrent = m_swerveDriveConfigs.CurrentLimits;
        swerveDriveCurrent.StatorCurrentLimit = 60;
        swerveDriveCurrent.SupplyCurrentLimit = 40;
        


        //////////////////////////
        /// Swerve Turn Configs //
        //////////////////////////
        
        var swerveTurnPIDs = m_swerveTurnConfigs.Slot0;
        swerveTurnPIDs.kP = Constants.DriveConstants.TURN_KP;
        swerveTurnPIDs.kI = Constants.DriveConstants.TURN_KI;
        swerveTurnPIDs.kD = Constants.DriveConstants.TURN_KD;

        
        var swerveTurnCurrent = m_swerveTurnConfigs.CurrentLimits;
        swerveTurnCurrent.StatorCurrentLimit = 40;
        swerveTurnCurrent.SupplyCurrentLimit = 30;


        m_swerveTurnConfigs.Feedback.SensorToMechanismRatio = Constants.DriveConstants.ANGLE_GEAR_RATIO;
        m_swerveTurnConfigs.ClosedLoopGeneral.ContinuousWrap = true;

        //////////////////////////
        /// Shooter Constants ////
        //////////////////////////

        /* Shooter PIDs */
        var shooterPID = shooterConfigs.Slot0;
        shooterPID.kP = Constants.ShooterConstants.FLYWHEEL_KP;
        shooterPID.kI = Constants.ShooterConstants.FLYWHEEL_KI;
        shooterPID.kD = Constants.ShooterConstants.FLYWHEEL_KD;
        //shooterPID.kS =
        //shooterPID.kV = 510; //TODO tune


        var shooterLimits = shooterConfigs.CurrentLimits;


        var shooterRightPID = shooterRightConfigs.Slot0;
        shooterRightPID.kP = Constants.ShooterConstants.FLYWHEEL_KP;
        shooterRightPID.kI = Constants.ShooterConstants.FLYWHEEL_KI;
        shooterRightPID.kD = Constants.ShooterConstants.FLYWHEEL_KD;

        var shooterRight = shooterRightConfigs.MotorOutput;
        shooterRight.Inverted = InvertedValue.Clockwise_Positive;

        //////////////////////
        /// Intake Configs ///
        //////////////////////
        
        var intakePID = m_intakeConfigs.Slot0;
        // TODO change PIDs
        intakePID.kP = 0.2;
        intakePID.kI = 0;
        intakePID.kD = 0;

        var intakeOutput = m_intakeConfigs.MotorOutput;
        intakeOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    }
    

}
