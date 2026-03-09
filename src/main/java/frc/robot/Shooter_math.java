package frc.robot;

public class Shooter_math {
    double velocity;
    double distance;
    double angle;
    double gravity;
    double height_of_target;
    Constants conts;
    double radius_of_wheels;


    public Shooter_math(){
        velocity = 0; // the velocity of the shooter in m/s

        distance = 0; // convert inches to meters
        
        angle = 0; // the angle of the shooter in degrees
        
        gravity = 9.81; // gravity in m/s^2
        
        height_of_target = 1.6002; // the distance from the top of the robot to the target in the y direction in meters

        radius_of_wheels = 0.5;
    }

    public double distanceFromGoal(){ 
       if (!LimelightHelpers.getTV("limelight")) {
        return distance; // Return the last calculated distance if target is lost
    }

        //Get the vertical offset (ty) from LimelightHelpers
        double targetOffsetAngle_Vertical = LimelightHelpers.getTY("limelight");

        //Get your constants from the Constants file
        double limelightMountAngleDegrees = Constants.ShooterConstants.AngleOfLimeLight; 
        double limelightLensHeightInches = Constants.ShooterConstants.heightOfLimeLight; 
        double goalHeightInches = Constants.ShooterConstants.heightofAprilTag * 39.37; // convert meters to inches

        //Calculate the total angle to the target
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        //Calculate horizontal distance (X)
        // Formula: (TargetHeight - CameraHeight) / tan(TotalAngle)
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        //Update the global distance variable (converted to meters)
        distance = distanceFromLimelightToGoalInches / 39.37; 
        
        return distance;
    }




    /** @return the minimum velocity needed to shoot the ball into the target
     */
    public double min_velocity(){
        distance = distanceFromGoal(); // Need to account for difference in the limelight's height and the shooter's height
        // distance  =  LimelightHelpers.getBotPose_TargetSpace("limelight")[0]; (can be used but is less reliable)
        velocity = Math.sqrt(gravity*(height_of_target+Math.sqrt(Math.pow(height_of_target, 2)+Math.pow(distance, 2)))); // The above formula is used to calculate the minimum velocity need to shoot the projectile. It is from a physics textbook.
        
        
        return velocity;
    }

    public double min_angle(){
        distance = distanceFromGoal();

        targetangle = min_velocity()/distance;

        angle = (targetangle + 90)/2
        
        // Math.atan((height_of_target+Math.sqrt(Math.pow(height_of_target, 2)+Math.pow(distance, 2)))/distance); // min angle required to get to the distance
        
        return angle*(180/Math.PI); // convert radians to degrees
    }

    
}
