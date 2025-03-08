package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;

public class CoralRestCommand extends Command {

  private final CoralIntake Intake;

  public CoralRestCommand(CoralIntake intake) {
    Intake = intake;
    addRequirements(Intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Intake.setPower(0.06);
  }

  @Override
  public void end(boolean interrupted) {
    Intake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
