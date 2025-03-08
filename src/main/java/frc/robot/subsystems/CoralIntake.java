// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {

  PWMSparkMax IntakeMotor1;
  PWMSparkMax IntakeMotor2;
  /*MOTORLARIN SMART CURRENT LIMITINI 50 YAP*/
  public CoralIntake() {
    IntakeMotor1 = new PWMSparkMax(Constants.Intake1PWM_Num);
    IntakeMotor2 = new PWMSparkMax(Constants.Intake2PWM_Num);

    IntakeMotor1.addFollower(IntakeMotor2);
    IntakeMotor1.setInverted(false);
    IntakeMotor2.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setPower(double power){
    IntakeMotor1.set(power);
  }
  
  public void stop(){
    IntakeMotor1.stopMotor();
  }
}
