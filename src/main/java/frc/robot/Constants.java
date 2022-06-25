// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    public static final int MotorLeft1ID =1;
    public static final int MotorLeft2ID =2;
    public static final int MotorRight1ID=3;
    public static final int MotorRight2ID=4;
    public static final double ksVolts = 1.25;
    public static final double kvVoltSecondsPerMeter=3.9;
    public static final double kaVoltSecondsSqaredPerMeter=0.4;
    public static final double kPDriveVel= .10694;
    public static final double kTrackwidthMeter=0.7112;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeter);
    public static final double kMaxSpeedMetersPerSecond =3;
    public static final double kMaxAccelerationMetersPerSecondSquared=3;
    public static final double kRamseteB=2;
    public static final double kRamseteZeta=0.7;
    public static final boolean kgyroReversed = false;
    public static final int XboxRightXaxis = 4;
    public static final int XboxLeftYaxis = 1;
    public static final double dtSpeed = .6;
    public static final int DriveJoystick = 0;
    public static final double CountsPerRev = 2048;
    public static final double GearRatio = 7.75;
    public static final double WheelRadiusIn = 2;
    public static final double kFlywheelKV = 0;
    public static final double kFlywheelKA = 0;
    public static final int  FlywheelMotor = 0;
    public static final double IntakeSpeed = .7;
    public static final double OuttakeSpeed = -1;
    public static final double WheelCircumfernce = Units.inchesToMeters(4)*Math.PI;
	public static final int ClimbMotor = 5;
    public static final double climbSpeed = 1;
    public static final int climbDownSpeed = -1;
	public static final int spinTake = 6;
    public static final int intakeRoller = 9;
	public static final int bottomFlywheelMotor = 7;
	public static final int topFlywheelMotor = 8;
    public static final double topFlywheelkS = 0.002125;
	public static final double topFlywheelkV = 0.002125;
	public static final double topFlywheelkA = 0.001;
    public static final double topflywheelKp = 0.002;
    public static final double topflywheelKd = .0002; 
	public static final double bottomFlywheelkA = 0.014274;
	public static final double bottomFlywheelkV = 0.002225;
	public static final double bottomFlywheelKs = 0.002225;
	public static final double flywheelVelocityEncoderConversion = 1/(3*42);
    public static final double driveTrainEncoderConversion = (9*WheelCircumfernce)/(62*2048);
    public static final double driveTrainVelEncoderConversion = (9*WheelCircumfernce)/(62*2048*10);
    public static final double bottomFlywheelKp = 0.0028704;
    public static final int indexMotorID = 10;
    public static final double IndexSpeed = .15;
    public static final double topFlywheelHighShotRPM = .75;
	public static final double bottomFlywheelHighsShotRPM =.75;
    public static final double topFlywheelLowShotRPM = 1; //best 5500
	public static final double bottomFlywheelLowShotRPM = 0; // best 500
    public static final int compressorID = 11;
    public static final int solenoidID = 0;
    public static final int solenoid2ID = 1;
    public static final int solenoid3ID = 7;
    public static final int solenoid4ID = 0;    
    public static final double spinTakePostionConversion = 1/3*42;
    public static final double spinTakeKD = 0;
    public static final double spinTakeKI = 0;
    public static final double spinTakeKP = 0;
    public static final double spinTakeKS = 0;
    public static final double spinTakeKV = 0;
    public static final double spinTakeKA = 0;
    public static final double spinTakeKCOS = 0;
    public static final double RadianTravel = 0;
    public static final double spinTakeVelocity = 0;
    public static final int solenoid5ID = 6;
    public static final int solenoid6ID = 5;
    public static final int solenoid7ID = 3;
    public static final int Solenoid8ID = 4;
    public static final int Ballchecktolerance = 200;
    public static final double gyroKp = 0.01;
    public static final double gyroKi = 0;
    public static final double gyroKd = 0;
    public static final double IdleFlywheelSpeed = 000;
    public static final double shootingSpeed = .1;
    public static final int spinTakeStallLimit = 0;
    public static final int spinTakeFreeLimit = 0;
    public static final int topFlywheelMotorFreeCurrentLimit = 0;
    public static final int topFlywheelMotorStallCurrentLimit = 0;
    public static final int OperatorController = 1;
    public static final double distanceKp = 1;
    public static final double distanceKi = 0;
    public static final double distanceKd = 0;
    public static final double AutoDistanceMeters = 1;
    public static final double slowSpeed = .5;
    public static final int windmillMotor = 11;
    public static final int windmillMotor2 = 12;
    public static final double autoSpeed = 0;
    public static final double MeasurmentSD = 0;
    public static final double stateSD = 0;
    public static final double relms = 0;
    public static final double qelms = 0;
    public static final double topflywheelKi = 0;
    public static final double bottomFlywheelKi = 0;
    public static final double bottomFlywheelKd = 0;
    public static final double drivetrainKv = 0.002;
    public static final double drivetrainKa = 0.002;
    public static final double drivetrainKs = 0.002;
    public static final double LLKd = 0;
    public static final double LLKi = 0.009;
    public static final double LLKp = .2;
    public static final double LLDKp = 0.25;
    public static final double LLDKi = 0.0095;
    public static final double LLDKd = 0;
    public static double msPerSecond = 10;
    public static double k100msPerSecond = 10;

}
