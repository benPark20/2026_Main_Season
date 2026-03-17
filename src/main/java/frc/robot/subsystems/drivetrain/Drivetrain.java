/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import frc.robot.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.util.PathPlannerLogging;

public class Drivetrain extends SubsystemBase {

  private final SwerveModule[] swerveMods;

  private SwerveModulePosition[] lastModulePositions = // For delta tracking
    new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
    };
  private final SwerveDrivePoseEstimator m_odometry;
  private final Pigeon2 m_gyro;
  private final StatusSignal<Angle> gyroYawSignal;
  private final StatusSignal<AngularVelocity> gyroYawVelocitySignal;
  public final Field2d m_field2d;

  private Rotation2d fieldOrientedOffset;
  private Rotation2d simHeading = new Rotation2d();

  private final SysIdRoutine sysIDDriveRoutine;
  public final SendableChooser<Command> sysIDChooser;

  private boolean aligningWithReef = true;

  /** Creates a new Drivetrain. */
  public Drivetrain(SwerveModuleIO mod0IO, SwerveModuleIO mod1IO, SwerveModuleIO mod2IO, SwerveModuleIO mod3IO) {

    swerveMods = new SwerveModule[] {
      new SwerveModule(mod0IO, 0),
      new SwerveModule(mod1IO, 1),
      new SwerveModule(mod2IO, 2),
      new SwerveModule(mod3IO, 3)
    };

    m_gyro = new Pigeon2(Constants.DriveConstants.GYRO_ID);
    gyroYawSignal = m_gyro.getYaw();
    gyroYawVelocitySignal = m_gyro.getAngularVelocityZWorld();

    resetModulesToAbsolute();
    while (!m_gyro.isConnected()) {}
    m_gyro.reset();

    m_field2d = new Field2d();

    // Creates and pushes Field2d to SmartDashboard.
    SmartDashboard.putData(m_field2d);

    DriverStation.waitForDsConnection(60);

    resetHeading(Rotation2d.fromDegrees(DriverStation.getAlliance().get() == Alliance.Blue ? 0 : 180));

    m_odometry = new SwerveDrivePoseEstimator(
      Constants.DriveConstants.kSwerveKinematics, 
      getHeading(), 
      getModulePositions(), 
      new Pose2d(0, 0, Rotation2d.fromDegrees(DriverStation.getAlliance().get() == Alliance.Blue ? 180 : 0)),
      VecBuilder.fill(0.1, 0.1, 0.1),
      VecBuilder.fill(0.3, 0.3, 0.3));

    fieldOrientedOffset = Rotation2d.fromDegrees(DriverStation.getAlliance().get() == Alliance.Blue ? 0 : 180);

    PathPlannerLogging.setLogActivePathCallback(
      (path) -> {
        Logger.recordOutput("Odometry/Trajectory", path.toArray(new Pose2d[path.size()]));
        addField2dTrajectory(path, "Trajectory");
      }
    );
    PathPlannerLogging.setLogTargetPoseCallback(
      (pose) -> {
        Logger.recordOutput("Odometry/Trajectory Setpoint", pose);
      }
    );

    sysIDDriveRoutine = new SysIdRoutine(
      new Config(), 
      new Mechanism(
        voltage -> {
          for (SwerveModule mod : swerveMods) {
            mod.runCharacterization(voltage.in(Units.Volts));
          }
        },
        log -> { 
          for (SwerveModule mod : swerveMods) {
            log.motor("Drive Motor " + mod.moduleNumber)
              .voltage(Units.Volts.of(mod.getDriveVoltage())).linearPosition(Units.Meters.of(mod.getPosition().distanceMeters))
              .linearVelocity(Units.MetersPerSecond.of(mod.getState().speedMetersPerSecond))
              .linearAcceleration(Units.MetersPerSecondPerSecond.of(mod.getDriveAcceleration()));
          }
        },
        this));

    sysIDChooser = new SendableChooser<Command>();

    sysIDChooser.setDefaultOption("Quasistatic Forward", sysIDDriveRoutine.quasistatic(Direction.kForward));
    sysIDChooser.addOption("Quasistatic Reverse", sysIDDriveRoutine.quasistatic(Direction.kReverse));
    sysIDChooser.addOption("Dynamic Forward", sysIDDriveRoutine.dynamic(Direction.kForward));
    sysIDChooser.addOption("Dynamic Reverse", sysIDDriveRoutine.dynamic(Direction.kReverse));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    for (SwerveModule mod : swerveMods) {

      mod.updateInputs();

    }

    updateOdometry();
    m_field2d.setRobotPose(getPose());

    BaseStatusSignal.refreshAll(
      gyroYawSignal,
      gyroYawVelocitySignal
    );

    Logger.recordOutput("Drivetrain/Current Command", getCurrentCommand() == null ? "Nothing" : getCurrentCommand().getName());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Drives the robot at either given field-relative X, Y, and rotational
   * velocities or given robot-relative forward, sideways, and rotational velocities.
   * 
   * 
   * @param transform       A Transform2d object representing either the desired
   *                        field-relative velocities in meters/second for the
   *                        robot to move at along the X and Y axes of the
   *                        field, or the desired robot-relative forward(X)
   *                        and sideways(Y) velocities in meters/second, as well as the 
   *                        desired rotational velocity in radians/second.
   * @param isOpenLoop      Whether the accordingly generated states for the given
   *                        velocities should be set using open loop control for
   *                        the drive motors
   *                        of the swerve modules.
   * @param isFieldRelative Whether the given velocities are relative to the field
   *                        or not.
   */
  public void drive(Transform2d transform, boolean isOpenLoop, boolean isFieldRelative) {

    Rotation2d rotationWithOffset = getHeading().minus(fieldOrientedOffset);

    SwerveModuleState[] states = Constants.kDrivetrain.kSwerveKinematics.toSwerveModuleStates(
      ChassisSpeeds.discretize(
        isFieldRelative ?
          ChassisSpeeds.fromFieldRelativeSpeeds(
            transform.getX(),
            transform.getY(),
            transform.getRotation().getRadians(),
            rotationWithOffset)
          : new ChassisSpeeds(
            transform.getX(), 
            transform.getY(),
            transform.getRotation().getRadians()),
      0.02));      

    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kDrivetrain.MAX_LINEAR_VELOCITY);

    runSetpoints(states, isOpenLoop);

  }

  /**
   * Updates the drivetrain odometry object to the robot's current position on the
   * field.
   */
  public void updateOdometry() {

    m_odometry.update(getHeading(), getModulePositions());

    for (String side : new String[] {"left", "right", "back"}) {

      LimelightHelpers.SetRobotOrientation("limelight-" + side, getHeading().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-" + side);

      if (estimate.tagCount > 0) {
        
        double tagID = LimelightHelpers.getFiducialID("limelight-" + side);

        for (int desiredID : aligningWithReef ? Constants.kField.REEF_APRILTAG_IDS : Constants.kField.CORAL_STATION_APRILTAG_IDS) {

          if (tagID == desiredID) {

            Translation3d aprilTagPosition = LimelightHelpers.getTargetPose3d_RobotSpace("limelight-" + side).getTranslation();

            if (Math.hypot(aprilTagPosition.getX(), aprilTagPosition.getZ()) <= 3.5) {
                
              m_odometry.addVisionMeasurement(new Pose2d(estimate.pose.getX(), estimate.pose.getY(), getHeading()), estimate.timestampSeconds);
                
            }

            break;

          }

        }

      }

    }

  }

  @AutoLogOutput(key = "Odometry/Field Position")
  /**
   * Returns the current pose of the robot without updating
   * the odometry.
   * 
   * @return The current estimated pose of the robot.
   */
  public Pose2d getPose() {

    return m_odometry.getEstimatedPosition();

  }

  public void addField2dTrajectory(List<Pose2d> poses, String trajectoryName) {

    m_field2d.getObject(trajectoryName).setPoses(poses);

  }

  public void addField2dPose(Pose2d pose, String poseName) {

    m_field2d.getObject(poseName).setPose(pose);

  }

  /**
   * Obtains and returns the current positions of all drivetrain swerve modules.
   * 
   * @return The current positions of all drivetrain swerve modules.
   */
  public SwerveModulePosition[] getModulePositions() {

    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for(SwerveModule mod : swerveMods) {

      positions[mod.moduleNumber] = mod.getPosition();

    }

    return positions;

  }

  @AutoLogOutput(key = "Drivetrain/Actual Module States")
  /**
   * Obtains and returns the current states of all drivetrain swerve modules.
   * 
   * @return The current states of all drivetrain swerve modules.
   */
  public SwerveModuleState[] getModuleStates() {

    SwerveModuleState[] states = new SwerveModuleState[4];

    for(SwerveModule mod : swerveMods) {

      states[mod.moduleNumber] = mod.getState();

    }

    return states;

  }

  @AutoLogOutput(key = "Drivetrain/Target Module States")
  /**
   * Obtains and returns the target states that the drivetrain swerve modules have
   * been set to.
   * 
   * @return The target states that the drivetrain swerve modules have been set
   *         to.
   */
  public SwerveModuleState[] getTargetStates() {

    SwerveModuleState[] targetStates = new SwerveModuleState[4];

    for(SwerveModule mod : swerveMods) {

      targetStates[mod.moduleNumber] = mod.getTargetState();

    }

    return targetStates;

  }

  /**
   * Obtains and returns the current absolute angle readings
   * in degrees from the absolute encoders of all swerve modules 
   * without offsets.
   * 
   * @return The current absolute angle readings in degrees from the
   *         absolute encoders of all swerve modules without offsets.
   */
  public double[] getAbsoluteAngles() {

    double[] angles = new double[4];

    for(SwerveModule mod : swerveMods) {

      angles[mod.moduleNumber] = mod.getAbsoluteAngle().getDegrees();

    }

    return angles;

  }

  /**
   * Sets the positions of the integrated angle motor
   * encoders of all swerve modules to the absolute position
   * readings of the absolute encoders with their offsets being 
   * taken into account.
   */
  public void resetModulesToAbsolute() {

    for(SwerveModule mod : swerveMods) {

      mod.resetToAbsolute();

    }

  }

  /**
   * Resets the position of the odometry object using a specified position.
   * 
   * @param position The desired position to reset the odometry of the robot to.
   */
  public void resetOdometry(Pose2d position) {

    m_odometry.resetPosition(getHeading(), getModulePositions(), position);

  }

  /** 
   * Resets the position of the odometry object using a specified rotation
   * while keeping the translation the same.
   * 
   * @param rotation The desired rotation to reset the odometry of the robot to.
   */
  public void resetRotation(Rotation2d rotation) {

    resetOdometry(new Pose2d(getPose().getX(), getPose().getY(), rotation));

  }

  /**
   * Resets the rotation of the odometry object to the rotation received from
   * the limelight.
   */
  public void resetToAprilTagRotation() {

    resetRotation(LimelightHelpers.getBotPose2d_wpiBlue("limelight").getRotation());

  }

  public void resetFieldOrientedHeading() {

    fieldOrientedOffset = getHeading().minus(Rotation2d.fromDegrees(180));
    resetHeading(Rotation2d.fromDegrees(DriverStation.getAlliance().get() == Alliance.Blue? 0 : 180));

  }

  public void reverseFieldOrientedHeading() {

    fieldOrientedOffset = getHeading();
    resetHeading(Rotation2d.fromDegrees(DriverStation.getAlliance().get() == Alliance.Blue? 180 : 0));

  }

  @AutoLogOutput(key = "Drivetrain/Heading")
  /**
   * Obtains and returns the current heading of the robot as a Rotation2d from the
   * gyro object.
   * 
   * @return The current heading of the robot as a Rotation2d.
   */
  public Rotation2d getHeading() {

    if (RobotBase.isReal()) {
      /*return Rotation2d.fromDegrees(Constants.kDrivetrain.INVERT_GYRO? 
        MathUtil.inputModulus(-(gyroYawSignal.getValue().in(Units.Degrees) - 180), 0 , 360) 
        : MathUtil.inputModulus(gyroYawSignal.getValue().in(Units.Degrees) - 180, 0, 360));*/
        return Rotation2d.fromDegrees(
          MathUtil.inputModulus(gyroYawSignal.getValue().in(Units.Degrees) - 180, 0, 360));
    }
    else {
      SwerveModulePosition[] modulePositions = getModulePositions();
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

      for (SwerveModule mod : swerveMods) {
        
        moduleDeltas[mod.moduleNumber] = 
          new SwerveModulePosition(
            modulePositions[mod.moduleNumber].distanceMeters - lastModulePositions[mod.moduleNumber].distanceMeters,
            modulePositions[mod.moduleNumber].angle
          );
        lastModulePositions[mod.moduleNumber] = modulePositions[mod.moduleNumber];

      }

      simHeading = simHeading.plus(new Rotation2d(Constants.kDrivetrain.kSwerveKinematics.toTwist2d(moduleDeltas).dtheta));
      return simHeading;
    }

  }

  /**
   * @return The current rotational velocity of the robot as a Rotation2d
   */
  public Rotation2d getRotationalVelocity() {

    /*return Rotation2d.fromDegrees(
      Constants.kDrivetrain.INVERT_GYRO? 
        -gyroYawVelocitySignal.getValue().in(Units.DegreesPerSecond) 
        : gyroYawVelocitySignal.getValue().in(Units.DegreesPerSecond));*/
      
      return Rotation2d.fromDegrees(gyroYawVelocitySignal.getValue().in(Units.DegreesPerSecond));

  }


  /** 
   * @return Returns the current pitch of the robot as detected by the Gyro
   */
  public double getPitch() {
    return m_gyro.getPitch().getValueAsDouble();
  }

  /**  
   * @return The current roll of the robot as detected by the Gyro 
   */
  public double getRoll() {
    return m_gyro.getPitch().getValueAsDouble();
  }

  /**
   * Resets the gyro yaw axis to the given angle.
   */
  public void resetHeading(Rotation2d angle) {

    m_gyro.setYaw(angle.getMeasure());

  }

  /**
   * Calculates and returns the current chassis speeds of the drivetrain using
   * the average forward and sideways velocities of the individual swerve modules
   * and the rotational velocity measured by the gyro.
   * 
   * @return The current chassis speeds of the drivetrain.
   */
  public ChassisSpeeds getChassisSpeeds() {

    return Constants.kDrivetrain.kSwerveKinematics.toChassisSpeeds(getModuleStates());

  }

  @AutoLogOutput(key = "Drivetrain/Speed")
  /**
   * Calculates and returns the speed of the drivetrain.
   * 
   * @return The current speed of the drivetrain.
   */
  public double getSpeed() {

    return Math.hypot(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond);

  }

  /**
   * Runs the drivetrain at the given robot-relative chassis 
   * speeds after they are converted to swerve module states.
   * 
   * @param speeds The desired chassis speeds to run the drivetrain at.
   * 
   */
  public void runChassisSpeeds(ChassisSpeeds speeds) {

    runSetpoints(
      Constants.kDrivetrain.kSwerveKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(speeds, 0.02)), 
      false);

  }

  /**
   * Runs the drivetrain swerve modules at the given setpoint states.
   * 
   * @param states The desired states for all drivetrain swerve modules to run at.
   * @param isOpenLoop Whether the accordingly generated states for the given
   *                   velocities should be set using open loop control for
   *                   the drive motors of the swerve modules.
   */
  public void runSetpoints(SwerveModuleState[] states, boolean isOpenLoop) {

    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kDrivetrain.MAX_LINEAR_VELOCITY);

    for(SwerveModule mod : swerveMods) {

      mod.runSetpoint(states[mod.moduleNumber], isOpenLoop);

    }

  }

  /**
   * Sets the drive and angle motors of all swerve modules to given drive and
   * angle motor
   * percent outputs.
   * 
   * @param drivePercentOutput The percent output between -1 and 1 to set all
   *                           drive motors to.
   * @param anglePercentOutput The percent output between -1 and 1 to set all
   *                           angle motors to.
   */
  public void runDutyCycle(double drivePercentOutput, double anglePercentOutput) {

    for(SwerveModule mod : swerveMods) {

      mod.runDutyCycle(drivePercentOutput, anglePercentOutput);

    }

  }

  public double[] driveOutputCurents(){

    double[] currents = new double[4];

    for (SwerveModule mod : swerveMods) {

      currents[mod.moduleNumber] = mod.getDriveOutputCurrent();

    }

    return currents;
  }

  public Command getSysIDDriveRoutine() {

    return sysIDChooser.getSelected();

  }

  public CoralStationPosition getClosestCoralStationPosition() {

    if (DriverStation.getAlliance().get() == Alliance.Blue) {

      return getPose().getY() >= 4.025 ? CoralStationPosition.LEFT_CORAL_STATION_RIGHT : CoralStationPosition.RIGHT_CORAL_STATION_LEFT;

    }
    else {

      return getPose().getY() <= 4.025 ? CoralStationPosition.LEFT_CORAL_STATION_RIGHT : CoralStationPosition.RIGHT_CORAL_STATION_LEFT;

    }

  }

  /**
   * Tells this subsystem whether the robot is aligning
   * with the reef or the coral station in order to only
   * use the respective AprilTags for each for vision
   * odometry.
   * 
   * @param aligningWithReef True if aligning with reef,
   *                         false if aligning with coral station
   */
  public void setAligningWithReef(boolean aligningWithReef) {

    this.aligningWithReef = aligningWithReef;

  }

} 