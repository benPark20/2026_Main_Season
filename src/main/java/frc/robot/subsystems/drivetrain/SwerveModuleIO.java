package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {

    @AutoLog
    public static class SwerveModuleIOInputs {
        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSec = 0.0;
        public double driveAccelerationMetersPerSecSquared = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public Rotation2d absoluteAnglePosition = new Rotation2d();
        public Rotation2d integratedAnglePosition = new Rotation2d();
        public double angleVelocityDegreesPerSec = 0.0;
        public double angleAppliedVolts = 0.0;
        public double angleCurrentAmps = 0.0;
    }

    public default void updateInputs(SwerveModuleIOInputs inputs) {}
    public default void runDriveDutyCycle(double percentOutput) {}
    public default void setDriveVoltage(double volts) {}
    public default void setDriveVelocity(double velocity) {}
    public default void runAngleDutyCycle(double percentOutput) {}
    public default void setAngleVoltage(double volts) {}
    public default void setAnglePosition(double degrees) {}
    public default void resetToAbsolute() {}
}