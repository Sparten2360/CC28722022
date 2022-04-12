// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;
import java.util.List;

import javax.swing.plaf.metal.MetalBorders.Flush3DBorder;

import com.fasterxml.jackson.databind.cfg.ContextAttributes;
import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoDistance;
import frc.robot.commands.BringIntakeDown;
import frc.robot.commands.BringIntakeUp;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.CurvatureDrive;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.HighShot;
import frc.robot.commands.HighShotAuto;
import frc.robot.commands.HurricaneDrive;
import frc.robot.commands.IdleFlywheelSpeed;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.LimeLightAim;
import frc.robot.commands.LimeLightDistance;
import frc.robot.commands.LowShot;
import frc.robot.commands.LowShotAuto;
import frc.robot.commands.OuttakeBall;
import frc.robot.commands.PIDBottomFlywheel;
import frc.robot.commands.PIDDistanceDrive;
import frc.robot.commands.PIDTopFlywheel;
import frc.robot.commands.Shoot;
import frc.robot.commands.TimedDrive;
import frc.robot.commands.TriggerDrive;
import frc.robot.commands.TurnToHeading;
import frc.robot.commands.VariableIntake;
import frc.robot.commands.index;
import frc.robot.commands.turnDegree;
import frc.robot.commands.windmillClimb;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LLVision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain dt = new DriveTrain();
  private final LLVision ll = new LLVision();
  private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(dt);
  private final TriggerDrive triggerDrive = new TriggerDrive(dt);
  private final boolean fastTurn = false;
  private final CurvatureDrive curvatureDrive = new CurvatureDrive(dt, fastTurn);
  private final TurnToHeading turnToHeading = new TurnToHeading(dt,2);
  private final AutoAim autoAim = new AutoAim(dt, ll);
  public static Joystick driverController = new Joystick(Constants.DriveJoystick); 
  DifferentialDriveVoltageConstraint autoVoltageConstraint;
  public static Joystick operatorController = new Joystick(Constants.OperatorController);
  TrajectoryConfig config;
  String trajectoryJSON="paths/.json";
  Path trajectoryPath;
  Trajectory trajectory;
  private final Intake intake = new Intake();  
  private final IntakeBall intakeBall;
  private final OuttakeBall outtakeBall;
  private final Climber climber;
  private final ClimbUp climbUp;
  private final ClimbDown climbDown;
  private final HighShot highShot;
  private final LowShot lowShot;
  private final Indexer indexer = new Indexer();
  private final Flywheel flywheel = new Flywheel();
  private final BringIntakeDown bringIntakeDown;
  private final BringIntakeUp bringIntakeUp;
  private final windmillClimb windMillClimb;
  private final VariableIntake variableIntake = new VariableIntake(intake);
  private final index index = new index(indexer);
  private final IdleFlywheelSpeed idleFlywheelSpeed = new IdleFlywheelSpeed(flywheel);
  private final SendableChooser<Command> drivetrainChooser = new SendableChooser<>();
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser.setDefaultOption("1 Ball Auto no lineup", new SequentialCommandGroup(new HighShotAuto(indexer, flywheel),new TimedDrive(dt,2,.5)));
    autoChooser.addOption("1 Ball Auto", new SequentialCommandGroup(new AutoAim(dt,ll), new AutoDistance(ll, dt),new AutoAim(dt, ll),new HighShotAuto(indexer, flywheel),new PIDDistanceDrive(dt, -2,new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter,Constants.kaVoltSecondsSqaredPerMeter))));
    autoChooser.addOption("2 Ball Auto", new SequentialCommandGroup(new LowShotAuto(flywheel, indexer), new TurnToHeading(dt, 180),new ParallelRaceGroup(new IntakeBall(intake),new TimedDrive(dt, 2.25, -.5)),new BringIntakeUp(intake),new TurnToHeading(dt, 180),new ParallelRaceGroup(new TimedDrive(dt, 1, -.5),new index(indexer).withTimeout(3)),new LimeLightAim(ll, dt), new LimeLightDistance(dt, ll), new LimeLightAim(ll,dt),new LowShotAuto(flywheel, indexer)));
    autoChooser.addOption("3 Ball Auto", new SequentialCommandGroup(new HighShot(flywheel, indexer),new TurnToHeading(dt, 180),new ParallelRaceGroup(new IntakeBall(intake),new PIDDistanceDrive(dt,1,new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter,Constants.kaVoltSecondsSqaredPerMeter))),new TurnToHeading(dt, 90), new ParallelRaceGroup(new IntakeBall(intake),new PIDDistanceDrive(dt, 2,new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter,Constants.kaVoltSecondsSqaredPerMeter))),new TurnToHeading(dt, 90),new AutoDistance(ll, dt),new AutoAim(dt,ll), new HighShot(flywheel, indexer)));
    autoChooser.addOption("test", new AutoDistance(ll, dt));
    drivetrainChooser.setDefaultOption("Joystick Drive", driveWithJoysticks);
    drivetrainChooser.addOption("TriggerDrive", triggerDrive); 
    drivetrainChooser.addOption("CurvatureDrive", curvatureDrive);
    drivetrainChooser.addOption("HurricaneDrive", new HurricaneDrive(dt));
    SmartDashboard.putData(drivetrainChooser);
    SmartDashboard.putData(autoChooser);
    CommandScheduler.getInstance().setDefaultCommand(dt, drivetrainChooser.getSelected());
    CommandScheduler.getInstance().setDefaultCommand(indexer, index);
    turnToHeading.addRequirements(dt);
    index.addRequirements(indexer);
    climber = new Climber();
    climbUp = new ClimbUp(climber);
    climbDown = new ClimbDown(climber);
    windMillClimb = new windmillClimb(climber); 
    CommandScheduler.getInstance().setDefaultCommand(climber, windMillClimb);
    intakeBall = new IntakeBall(intake);
    intakeBall.addRequirements(intake);
    outtakeBall = new OuttakeBall(intake,indexer);
    outtakeBall.addRequirements(intake,indexer);
    variableIntake.addRequirements(intake);
    highShot = new HighShot(flywheel,indexer);//indexer);
    highShot.addRequirements(flywheel,indexer);
    lowShot = new LowShot(flywheel,indexer);
    lowShot.addRequirements(flywheel,indexer);
    bringIntakeDown =new BringIntakeDown(intake);
    bringIntakeDown.addRequirements(intake);
    bringIntakeUp = new BringIntakeUp(intake);
    bringIntakeUp.addRequirements(intake);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton intakeButton = new JoystickButton(driverController,5);
    intakeButton.whileHeld(new IntakeBall(intake));
    intakeButton.whenReleased(new BringIntakeUp(intake));
    JoystickButton outtakeButton = new JoystickButton(driverController,6);
    outtakeButton.whileHeld(outtakeBall);
    JoystickButton climbButton = new JoystickButton(operatorController,1);
    climbButton.whileHeld(climbUp);
    JoystickButton climbDownButton = new JoystickButton(operatorController, 2);
    climbDownButton.whileHeld(climbDown);
    JoystickButton shootHighButton = new JoystickButton(operatorController, 3);
    shootHighButton.whileHeld(highShot);
    JoystickButton shootLowButton = new JoystickButton(operatorController, 4);
    shootLowButton.whileHeld(lowShot);
    /*JoystickButton intakeUpButton = new JoystickButton(operatorController, 6);
    intakeUpButton.whenPressed(bringIntakeDown);*/
    JoystickButton autoAimButton = new JoystickButton(operatorController, 12);
    autoAimButton.whileHeld(autoAim.andThen(new HighShot(flywheel,indexer)));
    JoystickButton autoDistanceButton = new JoystickButton(driverController, 8);
    autoDistanceButton.whileHeld(new AutoDistance(ll, dt));
    JoystickButton indexButton = new JoystickButton(operatorController, 14);
    indexButton.whileHeld(new index(indexer));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    /*autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter),
       Constants.kDriveKinematics,
        10);
    TrajectoryConfig config =
    new TrajectoryConfig(
    Constants.kMaxSpeedMetersPerSecond, 
    Constants.kMaxAccelerationMetersPerSecondSquared)
    .setKinematics(Constants.kDriveKinematics)
    .addConstraint(autoVoltageConstraint);
    Trajectory exampleTrajectory = 
    TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0,new Rotation2d(0)),
    List.of(
      new Translation2d(1,1),
      new Translation2d(2,-1)),
      new Pose2d(3,0,new Rotation2d(0)),
      config);
  dt.resetOdometry(exampleTrajectory.getInitialPose());
   RamseteCommand ramseteCommand = new RamseteCommand(
      exampleTrajectory,
      dt::getPose,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter,Constants.kaVoltSecondsSqaredPerMeter),
      Constants.kDriveKinematics,
      dt::getWheelSpeds,
      new PIDController(Constants.kPDriveVel,0,0),
      new PIDController(Constants.kPDriveVel, 0, 0),
      dt::voltDrive,
      dt
    );*/
    
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}