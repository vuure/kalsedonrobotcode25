package frc.robot;

import frc.robot.commands.ControlArmWithJoystick;
import frc.robot.commands.DriveForwardTimed;
import frc.robot.commands.DriveWithJoysticks;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  private final DriveTrain driveTrain;
  private final Arm arm;

  private final DriveWithJoysticks driveWithJoysticks;
  private final DriveForwardTimed driveForwardTimed;
  private final ControlArmWithJoystick controlArmWithJoystick;

  public static Joystick driverJoystick;
  public static AnalogGyro m_gyro;

  public RobotContainer() {

    driveTrain = new DriveTrain();
    arm = new Arm();

    driveWithJoysticks = new DriveWithJoysticks(driveTrain);
    driveWithJoysticks.addRequirements(driveTrain);
    driveTrain.setDefaultCommand(driveWithJoysticks);

    driveForwardTimed = new DriveForwardTimed(driveTrain);
    driveForwardTimed.addRequirements(driveTrain);

    controlArmWithJoystick = new ControlArmWithJoystick(arm);
    controlArmWithJoystick.addRequirements(arm);
    arm.setDefaultCommand(controlArmWithJoystick);

    driverJoystick = new Joystick(Constants.DriveJoystickPort_Num);

    m_gyro = new AnalogGyro(Constants.GyroPWM_Num);

    configureBindings();
  }

  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return driveForwardTimed;
  }
}
