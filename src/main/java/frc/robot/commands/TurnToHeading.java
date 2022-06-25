// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToHeading extends PIDCommand {
  private static final Pose2d Pose2d= new Pose2d();
  public static final SimpleMotorFeedforward dtff = new SimpleMotorFeedforward(Constants.drivetrainKs,Constants.drivetrainKa,Constants.drivetrainKv);
  DriveTrain drivetrain;
  /** Creates a new TurnToHeading. */
  public TurnToHeading(DriveTrain dt, double setpoint) {
    super(
        // The controller that the command will use
        new PIDController(.0175 , 0, 0),
        // This should return the measurement
        dt::getHeading,
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        output -> {
          dt.voltDrive((output+dtff.calculate(setpoint)), -(output+dtff.calculate(setpoint)));
          // Use the output here)
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here
    addRequirements(dt);
    getController().setTolerance(3,.15);
    getController().enableContinuousInput(-360, 360);
    drivetrain = dt;
    SmartDashboard.putNumber("gyro error", getController().getPositionError());
  }

  @Override
  public void initialize(){
    drivetrain.resetEncoders();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
