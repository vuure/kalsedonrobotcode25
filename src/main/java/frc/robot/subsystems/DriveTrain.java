package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.AnalogGyro;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.pathplanner.lib.config.RobotConfig;


public class DriveTrain extends SubsystemBase {

  double speed_frontLeft;
  double speed_frontRight;
  double speed_backLeft;
  double speed_backRight;

  WPI_VictorSPX backright;
  WPI_VictorSPX frontright;
  WPI_TalonSRX frontleft;
  WPI_VictorSPX backleft;

  MecanumDrive mDrive;
  Translation2d m_frontLeftLocation;
  Translation2d m_frontRightLocation;
  Translation2d m_backLeftLocation;
  Translation2d m_backRightLocation;

  Encoder m_frontLeftEncoder;
  Encoder m_frontRightEncoder;
  Encoder m_backLeftEncoder;
  Encoder m_backRightEncoder;

  MecanumDriveKinematics kinematics;
  MecanumDriveOdometry odometry;
  MecanumDriveWheelPositions wheel_positions;

  public DriveTrain(){
    
    backright = new WPI_VictorSPX(Constants.BackRightCAN_Num);
    frontright = new WPI_VictorSPX(Constants.FrontRightCAN_Num);
    frontleft = new WPI_TalonSRX(Constants.FrontLeftCAN_Num);
    backleft = new WPI_VictorSPX(Constants.BackLeftCAN_Num);

    m_frontLeftLocation = new Translation2d(0.381, 0.381);
    m_frontRightLocation = new Translation2d(0.381, -0.381);
    m_backLeftLocation = new Translation2d(-0.381, 0.381);
    m_backRightLocation = new Translation2d(-0.381, -0.381);

    mDrive = new MecanumDrive(frontleft, backleft, frontright, backright);
    
    kinematics = new MecanumDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    wheel_positions = new MecanumDriveWheelPositions(
      m_frontLeftEncoder.getDistance(), m_frontRightEncoder.getDistance(),
      m_backLeftEncoder.getDistance(), m_backRightEncoder.getDistance());

    odometry = new MecanumDriveOdometry(kinematics,RobotContainer.m_gyro.getRotation2d(), wheel_positions, new Pose2d(7.582, 7.094, new Rotation2d()));
    //BAŞLAMA POZİSYONLARINI DEĞİŞTİR

    m_frontLeftEncoder.setDistancePerPulse(47.877/Constants.Encoder_1turunpulsesayisi);
    m_frontRightEncoder.setDistancePerPulse(47.877/Constants.Encoder_1turunpulsesayisi);
    m_backLeftEncoder.setDistancePerPulse(47.877/Constants.Encoder_1turunpulsesayisi);
    m_backRightEncoder.setDistancePerPulse(47.877/Constants.Encoder_1turunpulsesayisi);

  }

  @Override
  public void periodic() {
  }

  public void driveWithJoystick(Joystick controller, AnalogGyro gyro, double speed){
    mDrive.driveCartesian(-controller.getRawAxis(Constants.LeftY_Axis) * speed, -controller.getRawAxis(Constants.LeftX_Axis) * speed, controller.getRawAxis(Constants.RightX_Axis) * speed, Rotation2d.fromDegrees(gyro.getAngle()) );
  }

  public void driveForward(double speed){
    mDrive.driveCartesian(speed, 0, 0);
  }

  public void stop() {
    mDrive.stopMotor();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    System.out.println(pose);
    odometry.resetPosition(RobotContainer.m_gyro.getRotation2d(), wheel_positions, pose);
  }

  public ChassisSpeeds wheelSpeeds(){
    return kinematics.toChassisSpeeds(new MecanumDriveWheelSpeeds(m_frontLeftEncoder.getRate(),m_frontRightEncoder.getRate(),m_backLeftEncoder.getRate(),m_backRightEncoder.getRate()));
  }

  public void DriveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(robotRelativeSpeeds);

    frontleft.set(wheelSpeeds.frontLeftMetersPerSecond);
    frontright.set(wheelSpeeds.frontRightMetersPerSecond);
    backleft.set(wheelSpeeds.rearLeftMetersPerSecond);
    backright.set(wheelSpeeds.rearRightMetersPerSecond);
  }
}

