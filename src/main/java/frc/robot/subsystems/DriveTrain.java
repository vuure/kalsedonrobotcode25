package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class DriveTrain extends SubsystemBase {


  WPI_VictorSPX backright;
  WPI_VictorSPX frontright;
  WPI_TalonSRX frontleft;
  WPI_VictorSPX backleft;

  AnalogGyro gyro;

  MecanumDrive mDrive;

  Encoder leftEncoder = new Encoder(Constants.leftEncoder_A, Constants.leftEncoder_B);
  Encoder rightEncoder = new Encoder(Constants.rightEncoder_A, Constants.rightEncoder_B);
  
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
    gyro.getRotation2d(),
    leftEncoder.getDistance(), rightEncoder.getDistance(),
    new Pose2d(5.0, 13.5, new Rotation2d()));

  //Mecanumlar arası mesafe GÜNCELLE
  DifferentialDriveKinematics kinematics =
  new DifferentialDriveKinematics(Units.inchesToMeters(27.0));
  
  RobotConfig config;
  
  public DriveTrain(){

    backright = new WPI_VictorSPX(Constants.BackRightCAN_Num);
    frontright = new WPI_VictorSPX(Constants.FrontRightCAN_Num);
    frontleft = new WPI_TalonSRX(Constants.FrontLeftCAN_Num);
    backleft = new WPI_VictorSPX(Constants.BackLeftCAN_Num);

    gyro = new AnalogGyro(Constants.GyroPWM_Num);


    mDrive = new MecanumDrive(frontleft, backleft, frontright, backright);
    
    //HESAPLAMALARI YENİDEN YAP!
    leftEncoder.setDistancePerPulse(0.005844360);//0,0058443603515625
    rightEncoder.setDistancePerPulse(0.005844360);

    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
            this::getPose, 
            this::resetPose,
            this::wheelSpeeds,
            (speeds, feedforwards) -> DriveRobotRelative(speeds),
            new PPLTVController(0.02), 
            config, 
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
    );
  }

  @Override
  public void periodic() {
  }

  //BURADAKİ SIKINTIYI DÜZELT
  public void driveWithJoystick(Joystick controller, double speed){
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
    //POZİSYON AYARLARINI DEĞİŞTİR
    odometry.resetPosition(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance(),new Pose2d(5.0, 13.5, new Rotation2d()) );
  }

  public ChassisSpeeds wheelSpeeds(){
    return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(leftEncoder.getRate(),rightEncoder.getRate()));
  }

  public void DriveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(robotRelativeSpeeds);

    frontleft.set(wheelSpeeds.leftMetersPerSecond);
    frontright.set(wheelSpeeds.leftMetersPerSecond);
    backleft.set(wheelSpeeds.rightMetersPerSecond);
    backright.set(wheelSpeeds.rightMetersPerSecond);
  }
}

