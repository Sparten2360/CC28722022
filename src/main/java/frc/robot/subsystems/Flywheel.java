// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
  private final CANSparkMax topFlywheelmotor = new CANSparkMax(Constants.topFlywheelMotor, MotorType.kBrushless);
  private final CANSparkMax bottomFlywheelmotor = new CANSparkMax(Constants.bottomFlywheelMotor, MotorType.kBrushless);
  private final RelativeEncoder topflywheelEncoder = topFlywheelmotor.getEncoder();
  private final RelativeEncoder bottomflywheelEncoder = bottomFlywheelmotor.getEncoder();
  /** Creates a new Flywheel. */
  private final BangBangController topflywheelBangBangController = new BangBangController();
  private final BangBangController bottomflywheelBangBangController = new BangBangController();
  private final PIDController topFlywheelPIDController = new PIDController(Constants.topflywheelKp,Constants.topflywheelKi,Constants.topflywheelKd);
  private final PIDController bottomrFlywheelPIDController = new PIDController(Constants.bottomFlywheelKp, Constants.bottomFlywheelKi, Constants.bottomFlywheelKd);
  private final SimpleMotorFeedforward topflywheelFeedForward = new SimpleMotorFeedforward(Constants.topFlywheelkS, Constants.topFlywheelkV);
  private final SimpleMotorFeedforward bottomflywheelFeedForward = new SimpleMotorFeedforward(Constants.bottomFlywheelKs, Constants.bottomFlywheelkV);
  public Flywheel() {
    topFlywheelmotor.setIdleMode(IdleMode.kCoast);
    bottomFlywheelmotor.setIdleMode(IdleMode.kCoast);
    topFlywheelmotor.setInverted(true);
    bottomFlywheelmotor.setInverted(false);
    topflywheelEncoder.setVelocityConversionFactor(Constants.flywheelVelocityEncoderConversion);
    bottomflywheelEncoder.setVelocityConversionFactor(Constants.flywheelVelocityEncoderConversion);
    topFlywheelmotor.setSmartCurrentLimit(Constants.topFlywheelMotorStallCurrentLimit, Constants.topFlywheelMotorFreeCurrentLimit);
    topflywheelEncoder.setPosition(0);
    bottomflywheelEncoder.setPosition(0);
    topFlywheelPIDController.setSetpoint(0);
    bottomrFlywheelPIDController.setSetpoint(0);
    topflywheelBangBangController.setSetpoint(0);
    bottomflywheelBangBangController.setSetpoint(0);
    topflywheelBangBangController.setTolerance(25);
    bottomflywheelBangBangController.setTolerance(25);
    topFlywheelPIDController.setTolerance(300);
    bottomrFlywheelPIDController.setTolerance(300);
  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("RPM", topflywheelEncoder.getVelocity());
    SmartDashboard.putNumber("PID1Error", topFlywheelPIDController.getPositionError());
    SmartDashboard.putNumber("RPM2", bottomflywheelEncoder.getVelocity());
    SmartDashboard.putNumber("PID2Eror", bottomrFlywheelPIDController.getPositionError());
    SmartDashboard.putBoolean("AtSetpoint", topFlywheelPIDController.atSetpoint());
    SmartDashboard.putNumber("Setpoint",topflywheelBangBangController.getSetpoint());
    // This method will be called once per scheduler run
  }

  public void resetPID(){
    topFlywheelPIDController.setSetpoint(0);
    bottomrFlywheelPIDController.setSetpoint(0);
    topflywheelBangBangController.setSetpoint(0);
    bottomflywheelBangBangController.setSetpoint(0);
  }
  public void setTopFlywheelRPM(double rpm){
    //topFlywheelmotor.set(rpm);//)
    topFlywheelmotor.setVoltage(topFlywheelPIDController.calculate(topflywheelEncoder.getVelocity(),rpm)+topflywheelFeedForward.calculate(rpm));
  }
  public void setBottomFlywheelRPM(double rpm){
    //bottomFlywheelmotor.set(rpm);//
    bottomFlywheelmotor.setVoltage(bottomrFlywheelPIDController.calculate(bottomflywheelEncoder.getVelocity(),rpm)+bottomflywheelFeedForward.calculate(rpm));
  }
  public boolean topflywheelAtSpeed(){
    return (topFlywheelPIDController.atSetpoint());
    //(topflywheelEncoder.getVelocity()>rpm);
  }
  public boolean bottomrflywheelAtSpeed(){
    return(bottomrFlywheelPIDController.atSetpoint());
    //(bottomflywheelEncoder.getVelocity()>rpm);
  }
  public void setMotors(double speed){
    topFlywheelmotor.set(speed);
    bottomFlywheelmotor.set(speed);
  }
  
  public double getBottomMotorRpm(){
    return bottomflywheelEncoder.getVelocity();
  }
  public double getTopMotorRpm(){
    return topflywheelEncoder.getVelocity();
  }
  public void setVoltTopMotor(double volts){
    topFlywheelmotor.setVoltage(volts);
  }
  public void setVoltBottomMotor(double volts){
    bottomFlywheelmotor.setVoltage(volts);
  }
  public void setBottomMotor(double speed){
    bottomFlywheelmotor.set(speed);
  }
  public void brakeMotor(){
    bottomFlywheelmotor.setIdleMode(IdleMode.kBrake);
  }
  public void stop(){
    topFlywheelmotor.stopMotor();
    bottomFlywheelmotor.stopMotor();
  }
}
