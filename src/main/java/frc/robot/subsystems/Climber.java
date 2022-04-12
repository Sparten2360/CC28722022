// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  //private DoubleSolenoid climbSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.solenoid5ID, Constants.solenoid6ID);
  //private DoubleSolenoid climbSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,Constants.solenoid7ID,Constants.Solenoid8ID);
  private CANSparkMax climberMotor = new CANSparkMax(Constants.ClimbMotor,MotorType.kBrushless);
  private CANSparkMax windmillMotor = new CANSparkMax(Constants.windmillMotor,MotorType.kBrushless);
  private CANSparkMax windmillMotor2 = new CANSparkMax(Constants.windmillMotor2,MotorType.kBrushless);

  public Climber() {
    windmillMotor.setIdleMode(IdleMode.kBrake);
    windmillMotor2.setIdleMode(IdleMode.kBrake);
    climberMotor.setIdleMode(IdleMode.kBrake);
    //climbSolenoid.set(Value.kReverse);
    //climbSolenoid2.set(Value.kReverse);
  }
  public void setClimbMotor(double speed){
    climberMotor.set(speed);
  }
  public void setWindmillMotor(double speed){
    windmillMotor.set(speed);
    windmillMotor2.set(speed);
  }
  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
  public void stopClimbMotor(){
    climberMotor.set(0);
    windmillMotor.set(0);
    windmillMotor2.set(0);  
  }
}
