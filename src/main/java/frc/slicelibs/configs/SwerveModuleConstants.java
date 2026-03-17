package frc.slicelibs.configs;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int absoluteEncoderID;
    public final Rotation2d angleOffset;

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int absoluteEncoderID, Rotation2d angleOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.absoluteEncoderID = absoluteEncoderID;
        this.angleOffset = angleOffset;
    }
}