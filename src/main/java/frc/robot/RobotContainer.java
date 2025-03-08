package frc.robot;

import frc.robot.commands.ControlArmWithJoystick;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.CoralRestCommand;
import frc.robot.commands.LeaveCoral;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CoralIntake;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {

  private final DriveTrain driveTrain = new DriveTrain();
  private final Arm arm = new Arm();
  private final CoralIntake coralIntake = new CoralIntake();

  private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(driveTrain);
  private final ControlArmWithJoystick controlArmWithJoystick = new ControlArmWithJoystick(arm);
  private final CoralRestCommand coralRest = new CoralRestCommand(coralIntake);
  private final LeaveCoral leaveCoral = new LeaveCoral(coralIntake);

  public static Joystick mainJoystick = new Joystick(Constants.DriveJoystickPort_Num);

  private final PathPlannerAuto mainAuto =  new PathPlannerAuto("auto1");

  public RobotContainer() {
    
    driveTrain.setDefaultCommand(driveWithJoysticks);
    arm.setDefaultCommand(controlArmWithJoystick);
    coralIntake.setDefaultCommand(coralRest);

    driveWithJoysticks.addRequirements(driveTrain);

    controlArmWithJoystick.addRequirements(arm);
    
    coralRest.addRequirements(coralIntake);
    leaveCoral.addRequirements(coralIntake);

    configureBindings();
  }

  private void configureBindings() {
    JoystickButton CoralIntakeLeaveButton = new JoystickButton(mainJoystick, Constants.CoralIntakeLeaveButton_Num);

    CoralIntakeLeaveButton.whileTrue(leaveCoral);
    
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return mainAuto;
  }
}
