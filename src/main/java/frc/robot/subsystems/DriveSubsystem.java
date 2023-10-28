// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  private final WPI_VictorSPX driveFrontLeft = new WPI_VictorSPX(DriveConstants.DRIVE_FRONT_LEFT);
  private final WPI_TalonSRX driveFrontRight = new WPI_TalonSRX(DriveConstants.DRIVE_FRONT_RIGHT);
  private final CANSparkMax driveBackLeft = new CANSparkMax(DriveConstants.DRIVE_BACK_LEFT, MotorType.kBrushed);
  private final WPI_VictorSPX driveBackRight = new WPI_VictorSPX(DriveConstants.DRIVE_BACK_RIGHT);

  private final MotorControllerGroup driveLeft = new MotorControllerGroup(driveFrontLeft, driveBackLeft);
  private final MotorControllerGroup driveRight = new MotorControllerGroup(driveFrontRight, driveBackRight);
  private final DifferentialDrive driveRobot = new DifferentialDrive(driveLeft, driveRight);

  private final PIDController turnController = new PIDController(DriveConstants.TURN_KP, DriveConstants.TURN_KI,
      DriveConstants.TURN_KD);

  private final PIDController leftPID = new PIDController(DriveConstants.leftKP, DriveConstants.leftKI, DriveConstants.leftKD);
  private final PIDController rightPID = new PIDController(DriveConstants.rightKP, DriveConstants.rightKI, DriveConstants.rightKD);

  private final DifferentialDriveKinematics differentialDriveKinematics = new DifferentialDriveKinematics(DriveConstants.kTrackWidth); // added DifferentialDriveKinematics
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);

  private final Encoder encoderLeftDrive = new Encoder(DriveConstants.LEFT_ENCODER_A, DriveConstants.LEFT_ENCODER_B);
  private final Encoder encoderRightDrive = new Encoder(DriveConstants.RIGHT_ENCODER_A, DriveConstants.RIGHT_ENCODER_B); // need to add the physical encoder

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  SlewRateLimiter filter = new SlewRateLimiter(0.38);

  
  public double getEncoderDistance() {
    return (encoderLeftDrive.getDistance());
  }

  public double getRightEncoderDistance(){
    return(encoderRightDrive.getDistance());
  }

  public double getGyroYaw() {
    return (gyro.getYaw());
  }

  public double getGyroPitch() {
    return (gyro.getPitch());
  }

  public void resetGyro() {
    gyro.reset();
  }

  public double getGyroRoll() {
    return (gyro.getRoll());
  }

  public double getTurnControllerSpeedRight() {
    return (turnController.calculate(getGyroYaw(), 90));
  }

  public double getTurnControllerSpeedLeft() {
    return (turnController.calculate(getGyroYaw(), -90));
  }

  public boolean atSetpoint() {
    return (turnController.atSetpoint());
  } 


  public DriveSubsystem() {
    driveFrontLeft.setNeutralMode(NeutralMode.Brake);
    driveFrontRight.setNeutralMode(NeutralMode.Brake);
    driveBackRight.setNeutralMode(NeutralMode.Brake);
    driveBackLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
    driveRight.setInverted(true);
    turnController.setTolerance(DriveConstants.TURN_CONTROLLER_POSITION_TOLERANCE,
        DriveConstants.TURN_CONTROLLER_VELOCITY_TOLERANCE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Roll", getGyroRoll());
    SmartDashboard.putNumber("Pitch", getGyroPitch());
    SmartDashboard.putNumber("Yaw", getGyroYaw());
    SmartDashboard.putNumber("Encoder Left Distance", encoderLeftDrive.getDistance());
    SmartDashboard.putNumber("Encoder Right Distance", encoderRightDrive.getDistance());
    SmartDashboard.putNumber("PID Right Turn Speed", turnController.calculate(getGyroYaw(), 90));
    SmartDashboard.putNumber("PID Left Turn Speed", turnController.calculate(getGyroYaw(), -90));

  }

  public void setMotor(double forwardSpeed, double turnSpeed) {
    driveRobot.arcadeDrive(filter.calculate(forwardSpeed), turnSpeed);
  }

  

}
