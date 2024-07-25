// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.intake;
import frc.robot.Constants.pivot;
import frc.robot.Constants.shutter;
import frc.robot.subsystems.AlignAmp;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Suspension;
import frc.robot.subsystems.TiltShutter;

import java.io.File;
import java.io.IOException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import swervelib.parser.SwerveParser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{
  public static Pose2d pose;
  public static double poseX = 0;
  public static double poseY = 0;
  public static boolean encerraInverte = false;
  public static Command autonomousCommand;
  private XboxController controle = new XboxController(0);
  private AlignAmp alignamp = new AlignAmp();
  private static Robot   instance;
  private RobotContainer m_robotContainer;
  private Suspension oscila = new Suspension();

  private SparkPIDController pidShutter = shutter.m_shut_up.getPIDController();

  private Timer disabledTimer;

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  @Override
  public void robotInit()
  {
    SmartDashboard.putString("Limelight", "http://10.91.69.11:5801/stream.mjpg");
    SmartDashboard.putString("Limelight Shutter", "http://10.91.69.23:5801/stream.mjpg");

    m_robotContainer = new RobotContainer();
    disabledTimer = new Timer();
    pivot.m_pvL.setIdleMode(IdleMode.kBrake);
    pivot.m_pvR.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void robotPeriodic()
  {
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("Sensor Shutter: ", shutter.sensor_shu.get());
    if (autonomousCommand != null && autonomousCommand.isScheduled()) {
      encerraInverte = true;
    }

    m_robotContainer.setPipeAlliance();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    pivot.m_pvL.setIdleMode(IdleMode.kCoast);
    pivot.m_pvR.setIdleMode(IdleMode.kCoast);
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    autonomousCommand = m_robotContainer.getAutonomousCommand();

      if (autonomousCommand != null) {
          autonomousCommand.schedule();
      }

    m_robotContainer.setMotorBrake(true);

    pivot.m_pvL.setIdleMode(IdleMode.kBrake);
    pivot.m_pvR.setIdleMode(IdleMode.kBrake);

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    pidShutter.setP(1);
    pidShutter.setI(0);
    pidShutter.setD(0);

    pivot.m_pvL.setIdleMode(IdleMode.kBrake);
    pivot.m_pvR.setIdleMode(IdleMode.kBrake);
    oscila.start();
    m_robotContainer.zeroShutter();
    if (shutter.sensor_shu.get()) {
      alignamp.camGoToAMP();
    }
    else{
      alignamp.camGoToGround();
    }
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null)
    {
      autonomousCommand.cancel();
    }
    m_robotContainer.setMotorBrake(true);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
    //if (controle.getPOV() == 90) {
    //  System.out.println("entrou");
    //  pidShutter.setReference(5000, ControlType.kVelocity);
    //}
    //else{
    //  pidShutter.setReference(0, ControlType.kDutyCycle);
    //}
    oscila.pisca();
    SmartDashboard.putNumber("Velocidade shutter:", shutter.m_shut_up.getEncoder().getVelocity());
  }
     

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    try
    {
      new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    } catch (IOException e)
    {
      throw new RuntimeException(e);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
}
