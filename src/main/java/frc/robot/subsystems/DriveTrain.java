// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

/* Luke Removed
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
*/

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  // --- Luke Mode --- \\
  public static boolean LukeMode = false;
  
  // ---------- Motor Output Percents ---------- \\
  public static double xPercent = 0d;
  public static double yPercent = 0d;
  public static double zPercent = 0d;
  public static double aPercent = 0d;

  public static enum ControlMode {
    Driver,
    Auto,
    Vision,
    Stop
  }

  private static ControlMode driveControlMode = ControlMode.Stop;

  // ---------- Motors & Encoders ---------- \\
  public final WPI_VictorSPX FRONT_RIGHT_MOTOR = new WPI_VictorSPX(Motors.FRONT_RIGHT_PORT);
  public final WPI_VictorSPX REAR_RIGHT_MOTOR = new WPI_VictorSPX(Motors.REAR_RIGHT_PORT);
  public final WPI_VictorSPX FRONT_LEFT_MOTOR = new WPI_VictorSPX(Motors.FRONT_LEFT_PORT);
  public final WPI_VictorSPX REAR_LEFT_MOTOR = new WPI_VictorSPX(Motors.REAR_LEFT_PORT);

  private MotorControllerGroup rightMotors = new MotorControllerGroup(FRONT_RIGHT_MOTOR, REAR_RIGHT_MOTOR);
  private MotorControllerGroup leftMotors = new MotorControllerGroup(FRONT_LEFT_MOTOR, REAR_LEFT_MOTOR);

  private final static Encoder leftEncoder = new Encoder(Motors.LEFT_ENCODER_PORT, Motors.LEFT_ENCODER_PORT + 1);
  private final static Encoder rightEncoder = new Encoder(Motors.RIGHT_ENCODER_PORT, Motors.RIGHT_ENCODER_PORT + 1);

  public static DifferentialDrive driveOutputs;


  /** Creates a new DriveTrain. */
  public DriveTrain() {
    // Invert the left side motors as they will be backwards
    leftMotors.setInverted(false);
    rightMotors.setInverted(true);

    // Create a drivetrain
    driveOutputs = new DifferentialDrive(leftMotors, rightMotors);

    // Set Distance per pulse on encoders
    leftEncoder.setDistancePerPulse(Math.PI * Drive.WHEEL_DIAMETER / Motors.ENCODER_RESOLUTION);
    rightEncoder.setDistancePerPulse(Math.PI * Drive.WHEEL_DIAMETER / Motors.ENCODER_RESOLUTION);

  }

  @Override
  public void periodic() {

    // ? Change Drive Style based on current drive mode
    switch (driveControlMode) {
      case Auto:
        break;
      case Driver:
        driverControl();
        break;
      case Stop:
        driveOutputs.arcadeDrive(0.0, 0.0);
        break;
      case Vision:
        break;
    }

  }

  /**
   * Runs the Robot in DriverControl Mode
   */
  private void driverControl() {
    //Update controler inputs
    updateStickValues();
    
    if (Robot.isReal()) {
      if (LukeMode)
      {
        driveOutputs.arcadeDrive(
          -yPercent * Constants.Drive.MAX_SPEED + -yPercent * aPercent * (1 - Constants.Drive.MAX_SPEED), 
          -zPercent * Constants.Drive.MAX_SPEED + -yPercent * aPercent * (1 - Constants.Drive.MAX_SPEED)
          );
      }
      else
      {
        driveOutputs.arcadeDrive(
        -yPercent * 0.65,
        -zPercent * 0.75
        );
      }
    } 
    
    else {
      // Robot turns quickly, so we tone it down in simulation
      driveOutputs.arcadeDrive(yPercent * 0.75, zPercent * 0.4);
    }
  }

  private void updateStickValues() {
    xPercent = RobotContainer.XBOX_CONTROLLER.getLeftX();
    yPercent = RobotContainer.XBOX_CONTROLLER.getLeftY();
    zPercent = RobotContainer.XBOX_CONTROLLER.getRightX();
    aPercent = RobotContainer.XBOX_CONTROLLER.getRightTriggerAxis();

    if (RobotContainer.XBOX_CONTROLLER.getStartButtonReleased() && RobotContainer.XBOX_CONTROLLER.getAButton())
    {
      LukeMode = true;
      System.out.println("Luke Mode Engaged");
    }

    if (RobotContainer.XBOX_CONTROLLER.getStartButtonPressed())
    {
      LukeMode = false;
      System.out.println("Luke Mode Disengaged");
    }
  }

  public static void changeDriveMode(ControlMode c) {
    driveControlMode = c;
  }

}
