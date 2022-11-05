// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import javax.print.attribute.standard.JobHoldUntil;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.GroupMotorControllers;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX leftLeader = new WPI_TalonFX(3);
  private WPI_TalonFX rightLeader = new WPI_TalonFX(1);
  private WPI_TalonFX leftFollower = new WPI_TalonFX(4);
  private WPI_TalonFX rightFollower = new WPI_TalonFX(2);
  private MotorControllerGroup leftGroup = new MotorControllerGroup(leftLeader,leftFollower);
  private MotorControllerGroup rightGroup = new MotorControllerGroup(rightLeader,rightFollower);
  private final DifferentialDrive Ddrive = new DifferentialDrive(leftGroup,rightGroup); 
  private final AHRS NavX = new AHRS(SPI.Port.kMXP);
  private final Gyro gyro = new ADXRS450_Gyro();
  private final DifferentialDriveOdometry ddOdometry;
  private final PIDController gyroController = new PIDController(Constants.gyroKp, Constants.gyroKi, Constants.gyroKd);
  private final PIDController distanController = new PIDController(Constants.distanceKp, Constants.distanceKi, Constants.distanceKd);
  //private SlewRateLimiter inputlimiter = new SlewRateLimiter(2);
  private boolean fastTurn = false;
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftFollower.set(ControlMode.Follower, leftLeader.getDeviceID());
    rightFollower.set(ControlMode.Follower, rightLeader.getDeviceID());
    leftLeader.configFactoryDefault();
    leftFollower.configFactoryDefault();
    rightLeader.configFactoryDefault();
    rightFollower.configFactoryDefault();
    leftLeader.setInverted(true);
    leftFollower.setInverted(true);
    rightLeader.setInverted(false);
    rightFollower.setInverted(false);
    leftLeader.setNeutralMode(NeutralMode.Brake);
    rightLeader.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);
    leftLeader.configClosedloopRamp(3);
    rightLeader.configClosedloopRamp(3);
    leftLeader.configNeutralDeadband(.01);
    rightLeader.configNeutralDeadband(.01);
    leftFollower.configNeutralDeadband(.01);
    rightFollower.configNeutralDeadband(.01);
    leftLeader.configOpenloopRamp(.5);
    leftFollower.configOpenloopRamp(.5);
    rightLeader.configOpenloopRamp(.5);
    rightFollower.configOpenloopRamp(.5); 
    /*StatorCurrentLimitConfiguration leftCurrentLimit = new StatorCurrentLimitConfiguration(true, 120,100, 1.0);
    SupplyCurrentLimitConfiguration leftSupplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 90, 100, .5);
    StatorCurrentLimitConfiguration rightCurrentLimit = new StatorCurrentLimitConfiguration(true, 90, 100, 1.0);
    SupplyCurrentLimitConfiguration rightSupplyCurrentLimitConfiguration = new SupplyCurrentLimitConfiguration(true, 90, 100, .5);
    leftLeader.configStatorCurrentLimit(leftCurrentLimit);
    leftLeader.configSupplyCurrentLimit(leftSupplyCurrentLimit);
    rightLeader.configStatorCurrentLimit(rightCurrentLimit);
    rightLeader.configSupplyCurrentLimit(rightSupplyCurrentLimitConfiguration);*/
    leftLeader.setSelectedSensorPosition(0);
    leftFollower.setSelectedSensorPosition(0);
    rightLeader.setSelectedSensorPosition(0);
    rightFollower.setSelectedSensorPosition(0);
    gyro.reset();
    NavX.reset();
    gyroController.setSetpoint(0);
    distanController.setSetpoint(0);
    distanController.setTolerance(.1);
    gyroController.setTolerance(10);

    ddOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }
  public double getHeading(){
    return NavX.getAngle()%360 ;
    }
  public double falconUnitsToMeters(double SensorCounts){
    double motorRoatations = (double)SensorCounts/Constants.CountsPerRev;
    double wheelRotations = motorRoatations/Constants.GearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(Constants.WheelRadiusIn));
    return positionMeters; 
  }
  public void distanceDrive(double meters){
    leftGroup.setVoltage(distanController.calculate(getLeftDistance(),meters));
    rightGroup.setVoltage(distanController.calculate(getRightDistance(),meters));
  }
  public double getLeftDistance(){
    return (leftLeader.getSelectedSensorPosition()* Constants.driveTrainEncoderConversion);
  }
  public double getRightDistance(){
    return (rightLeader.getSelectedSensorPosition()* Constants.driveTrainEncoderConversion);

  }
  public double getLeftVelocity(){
    return (leftLeader.getSelectedSensorVelocity()*Constants.driveTrainVelEncoderConversion);  
  }
  public double getRightVelocity(){
    return (rightLeader.getSelectedSensorVelocity()*Constants.driveTrainVelEncoderConversion);  
  }
  public void setGyroDrive(double Heading){
    leftGroup.setVoltage(-gyroController.calculate(NavX.getAngle(),Heading));
    rightGroup.setVoltage(gyroController.calculate(NavX.getAngle(),Heading));
  }
  @Override
  public void periodic() {
    ddOdometry.update(Rotation2d.fromDegrees(getHeading()), getLeftDistance(), getRightDistance());
    SmartDashboard.putNumber("Left Distance", getLeftDistance());
    SmartDashboard.putNumber("right Distance", getRightDistance());
    SmartDashboard.putNumber("Left velocity", getLeftVelocity());
    SmartDashboard.putNumber("right velocity", getRightVelocity());
    SmartDashboard.putNumber("Time Left", DriverStation.getMatchTime());
    SmartDashboard.putNumber("heading", getHeading());  
  }
  public Pose2d getPose(){
    return ddOdometry.getPoseMeters();
  }
  public DifferentialDriveWheelSpeeds getWheelSpeds(){
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(),getRightVelocity());
  }
  public void drive(Joystick controller,Double Speed){
    Ddrive.arcadeDrive( controller.getRawAxis(Constants.XboxLeftYaxis)*.9, controller.getRawAxis(Constants.XboxRightXaxis)*.6);
  }
  public void ChessyDrive(Joystick controller, double Speed){
    Ddrive.curvatureDrive(controller.getRawAxis(1)*.9, controller.getRawAxis(2)*.9, fastTurn);
  }
  public void tankDrive(double lspeed, double rspeed){
    Ddrive.tankDrive(lspeed, rspeed);
  }
  public void fastTurnflip(){
    fastTurn = true;
  }
  public void TriggerDrive(){
    double moveSpeed = (RobotContainer.driverController.getRawAxis(4))-RobotContainer.driverController.getRawAxis(3);
    double turnSpeed = RobotContainer.driverController.getRawAxis(0);
    
    Ddrive.arcadeDrive(moveSpeed, turnSpeed);
  }
  public void lowlatencyDrive(){
    leftGroup.set(RobotContainer.driverController.getRawAxis(1)+RobotContainer.driverController.getRawAxis(2));
    rightGroup.set(RobotContainer.driverController.getRawAxis(1)-RobotContainer.driverController.getRawAxis(2));
  }
  public void drive(double lspeed,double rspeed){
    leftGroup.set(lspeed);
    rightGroup.set(rspeed);
  }
  public void setMaxOutput(double maxOutput){
    Ddrive.setMaxOutput(maxOutput);
  }
  public void voltDrive(double leftVolts,double rightVolts){
    leftGroup.setVoltage(leftVolts);
    rightGroup.setVoltage(rightVolts);
    Ddrive.feed();
  }
  public double getTurnRate(){
    return NavX.getRate() * (false ? -1.0 :1.0);
  }
  public void resetEncoders(){
    leftLeader.setSelectedSensorPosition(0);
    leftFollower.setSelectedSensorPosition(0);
    rightFollower.setSelectedSensorPosition(0);
    rightLeader.setSelectedSensorPosition(0);
    NavX.reset();
    gyro.reset();
  }
  public void resetOdometry(Pose2d pose){
    resetEncoders();
    ddOdometry.resetPosition(pose, gyro.getRotation2d());
  }
  public void stop(){
    leftLeader.stopMotor();
    rightLeader.stopMotor();
  }
}
