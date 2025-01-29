package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ControlArmWithPID extends Command {

  private final Arm Arm;
  private final PIDController pidArm;
  private double armSetpoint;


  public ControlArmWithPID(Arm arm, double setpoint) {
    Arm = arm;
    addRequirements(Arm);
    pidArm = new PIDController(Constants.arm_P, Constants.arm_I, Constants.arm_D);

    armSetpoint = setpoint;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Arm.setPower(pidArm.calculate(Arm.encoderReading(), armSetpoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
