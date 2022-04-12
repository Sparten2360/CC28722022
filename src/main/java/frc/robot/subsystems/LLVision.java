// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LLVision extends SubsystemBase {
  public NetworkTable table;
  public NetworkTableEntry tx,ty,ta,tv,gp,cm,led;
  public double x,y,area,v,cam,Led;
  /** Creates a new LLVision. */
  public LLVision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
    gp = table.getEntry("pipeline");
    cm = table.getEntry("camMode");
    led = table.getEntry("ledMode");
    cm.setNumber(1);
    led.setNumber(1);
  }

  @Override
  public void periodic() {
    x =tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    v = tv.getDouble(0.0);
    cam = cm.getDouble(0.0);
    Led = led.getDouble(0.0);

    // This method will be called once per scheduler run
  }
  public double getX(){
    return x;
  }
  public double getV(){
    return v;
  }
  public double getY(){
    return y;
  }
  public double getArea(){
    return area;
  }
  public double getXY(){
    return x+y;
  }
  public void setPipe(int pipe){
    gp.setNumber(pipe);
  }
  public void setCam(int cam){
    cm.setNumber(cam);
  }
  public void setLed(int l){
    led.setNumber(l);
  }
  public double getLed(){
    return Led;
  }
  public double getCam(){
    return cam;
  }
}
