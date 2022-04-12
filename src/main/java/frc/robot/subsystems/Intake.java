// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax spinTake;
  private VictorSPX intakeRoller;
  private CameraServer cameraServer;
  private DoubleSolenoid intakeSolenoid;
  private boolean postion;
  private RelativeEncoder spintakeEncoder;
  private AnalogPotentiometer pressureSensor = new AnalogPotentiometer(2,250,-25);
  public Intake() {
  UsbCamera camera = cameraServer.startAutomaticCapture(0);
  camera.setResolution(640, 480);
  CvSource outputStream = CameraServer.putVideo("rectangle", 640, 480);
  spinTake = new CANSparkMax(Constants.spinTake,MotorType.kBrushless);
  spinTake.setIdleMode(IdleMode.kCoast);
  intakeRoller = new VictorSPX(Constants.intakeRoller);
  intakeRoller.setNeutralMode(NeutralMode.Coast);
  intakeRoller.setInverted(false);
  intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.solenoidID, Constants.solenoid2ID); 
  intakeSolenoid.set(Value.kForward);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("IntakeDown", postion);
    SmartDashboard.putNumber("Pressure", pressureSensor.get());
    // This method will be called once per scheduler run
  }
  public void setIntakeRoller(double speed){
    intakeRoller.set(ControlMode.PercentOutput,speed);
  }
  public void setSpinTake(double speed){
    spinTake.set(speed);
  }
  public void setVariableIntake(Joystick controller){
    spinTake.set(controller.getRawAxis(4));
    intakeRoller.set(ControlMode.PercentOutput, -controller.getRawAxis(XboxController.Axis.kRightTrigger.value));
  }
  public boolean getPostion(){
    return postion;
  }
  public void bringIntakeDown(){
    intakeSolenoid.set(Value.kReverse);
  }
  /*public void positionflip(){
      if(!postion){
        postion = true;
      }
      else{
        postion = false;
      }
  }*/
  public void bringIntakeUp(){
      intakeSolenoid.set(Value.kForward);
  }
  public void stop(){
    intakeRoller.set(ControlMode.PercentOutput,0);
    spinTake.set(0);
  }
}
