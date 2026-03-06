package frc.slicelibs.configs;

import java.lang.invoke.VarHandle;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants;

public class CTREConfigs {


    public final TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();
    public final TalonFXConfiguration shooterRightConfigs = new TalonFXConfiguration();

    public CTREConfigs(){

        //////////////////////////
        /// Shooter Constants ////
        //////////////////////////

        /* Shooter PIDs */
        var shooterPID = shooterConfigs.Slot0;
        shooterPID.kP = Constants.ShooterConstants.FLYWHEEL_KP;
        shooterPID.kI = Constants.ShooterConstants.FLYWHEEL_KI;
        shooterPID.kD = Constants.ShooterConstants.FLYWHEEL_KD;


        var shooterLimits = shooterConfigs.CurrentLimits;


        var shooterRightPID = shooterRightConfigs.Slot0;
        shooterPID.kP = Constants.ShooterConstants.FLYWHEEL_KP;
        shooterPID.kI = Constants.ShooterConstants.FLYWHEEL_KI;
        shooterPID.kD = Constants.ShooterConstants.FLYWHEEL_KD;

        var shooterRight = shooterRightConfigs.MotorOutput;
        shooterRight.Inverted = InvertedValue.Clockwise_Positive;
        
    }


    

    

    
}
