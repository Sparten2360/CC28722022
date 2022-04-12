// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDDistanceDrive extends PIDCommand {
  DriveTrain drivetrain;
  SimpleMotorFeedforward dtfeedforward = new SimpleMotorFeedforward(Constants.drivetrainKs,Constants.drivetrainKa ,Constants.drivetrainKv);
  /** Creates a new PIDDistanceDrive. */
  public PIDDistanceDrive(DriveTrain dt, double setpoint,SimpleMotorFeedforward dtfeedforward) {
    super(
        // The controller that the command will use
        new PIDController(1, 0, 0),
        // This should return the measurement
        ()-> ((dt.getLeftDistance())+(dt.getRightDistance()/2)),
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        output -> {
          dt.voltDrive(output + dtfeedforward.calculate(setpoint),output+dtfeedforward.calculate(setpoint));
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(.1);
    drivetrain = dt;
  }
  @Override
  public void initialize(){
    drivetrain.resetEncoders();
    getController().reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
