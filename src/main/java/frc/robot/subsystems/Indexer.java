// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private final VictorSPX indexMotor = new VictorSPX(Constants.indexMotorID);
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  //private final ColorSensorV3 colorSensor2 = new ColorSensorV3(I2C.Port.kMXP);
  public boolean ballinindexer = false;
  public boolean redInIntake = false;
  public boolean redInIndexer = false;
  public boolean blueInIntake = false;
  public boolean blueInIndexer = false;
  /** Creates a new Indexer. */
  public Indexer() {
    indexMotor.setNeutralMode(NeutralMode.Brake);
  }
  public void setIndex(double speed){
    indexMotor.set(ControlMode.PercentOutput,speed);
  }
  /*public void getBallIntaked(){
    if(colorSensor2.getProximity()>Constants.Ballchecktolerance){
      ballinindexer = true;
    }
  }*/
  public void removedBalls(){
       ballinindexer = false;
  }
  public boolean getBallReady(){
    return(colorSensor.getProximity()>Constants.Ballchecktolerance);
  }
  /*public double[] getBallColor(){
    double colorarray[]= new double[]{colorSensor.getBlue(),colorSensor.getRed(),colorSensor2.getBlue(),colorSensor2.getRed()};
    return colorarray;
  }*/
  public int getblue(){
    return colorSensor.getBlue();
  }
  /*public int getBlue2(){
    return colorSensor2.getBlue();
  }*/
  public int getred(){
    return colorSensor.getRed();
  }
  public boolean ballRightColor(){
   if(DriverStation.getAlliance().equals(Alliance.Red)&&(getred()>getblue())){
    return true;
   }
   else if(DriverStation.getAlliance().equals(Alliance.Blue)&&(getblue()>getred())){
    return true;
   }
   else{
    return false;
   }
  }
  /*public int getRed2(){
    return colorSensor2.getRed();
  }*/
  /*public boolean getFull(){
    return(getBallReady()&&(colorSensor2.getProximity()>Constants.Ballchecktolerance));
  }*/
  public boolean getballinItake(){
    return ballinindexer;
  }
  @Override
  public void periodic() {
    /*if(colorSensor2.getBlue()>colorSensor2.getRed()){
      blueInIntake = true;
      redInIntake = false;
    }
    else{
      redInIntake = true;
      blueInIntake = false;
    }
    if(colorSensor.getBlue()>colorSensor.getRed()){
      blueInIndexer = true;
      redInIndexer = false;
    }
    else{
      redInIndexer = true;
      blueInIndexer = false;
    }*/
    SmartDashboard.putNumber("proximity", colorSensor.getProximity());
    /*SmartDashboard.putNumber("Proximity 2", colorSensor2.getProximity());
    SmartDashboard.putBoolean("Full",(colorSensor2.getProximity() > 400) && getBallReady());    
    SmartDashboard.putBoolean("Intake Red", redInIntake);    
    SmartDashboard.putBoolean("Indexer Red", redInIndexer);
    SmartDashboard.putBoolean("Intake Blue", blueInIntake);    
    SmartDashboard.putBoolean("Indexer Blue", blueInIndexer);
    */

    // This method will be called once per scheduler run
  }
  public void stop(){
    indexMotor.set(ControlMode.PercentOutput, 0);
  }
}
