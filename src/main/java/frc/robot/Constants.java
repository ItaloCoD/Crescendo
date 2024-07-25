// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = 52; // 32lbs * kg per pound 103.617 * 0.453592
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(4)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(1.5, 0, 0.04);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6; //5
  }
  
  public static class intake {
    public static final CANSparkMax m_itk = new CANSparkMax(50, MotorType.kBrushless);
    public static final CANSparkMax m_itk_left = new CANSparkMax(57, MotorType.kBrushless);
    public static final CANSparkMax m_itk_right = new CANSparkMax(58, MotorType.kBrushless);
    public static final Servo servoCam = new Servo(2);
  }

  public static class pivot {
    
    
    public static final CANSparkMax m_pvL = new CANSparkMax(51, MotorType.kBrushless);
    public static final CANSparkMax m_pvR = new CANSparkMax(52, MotorType.kBrushless);
    public static double setpointEncoder;
  }

  public static class shutter {
    public static final CANSparkMax m_shut_in = new CANSparkMax(53, MotorType.kBrushless);
    public static final CANSparkMax m_shut_up = new CANSparkMax(54, MotorType.kBrushless);
    public static final CANSparkMax m_shut_dw = new CANSparkMax(55, MotorType.kBrushless);
    public static final CANSparkMax m_shut_in2 = new CANSparkMax(56, MotorType.kBrushless);
    public static final DigitalInput sensor_shu = new DigitalInput(0);
  }
}
