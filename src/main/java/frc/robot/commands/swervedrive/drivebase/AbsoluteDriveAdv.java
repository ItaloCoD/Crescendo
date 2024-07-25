// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.Constants.shutter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * A more advanced Swerve Control System that has 4 buttons for which direction to face
 */
public class AbsoluteDriveAdv extends Command
{
  private final SwerveSubsystem swerve;
  private final DoubleSupplier  vX, vY;
  private DoubleSupplier  headingAdjust;
  private final BooleanSupplier lookAway, lookTowards, lookLeft, lookRight, lookbet, lookfeed, looksource;
  private       boolean         resetHeading = false;
  private       double          invert;
  private       double          headingX = 0;
  private       double          headingY = 0;
  private       double          aux = 0;
  /**
   * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
   * torwards/away from alliance wall and y is left/right. Heading Adjust changes the current heading after being
   * multipied by a constant. The look booleans are shortcuts to get the robot to face a certian direction. Based off of
   * ideas in https://www.chiefdelphi.com/t/experiments-with-a-swerve-steering-knob/446172
   *
   * @param swerve        The swerve drivebase subsystem.
   * @param vX            DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1 to 1
   *                      with deadband already accounted for.  Positive X is away from the alliance wall.
   * @param vY            DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1 to 1
   *                      with deadband already accounted for.  Positive Y is towards the left wall when looking through
   *                      the driver station glass.
   * @param headingAdjust DoubleSupplier that supplies the component of the robot's heading angle that should be
   *                      adjusted. Should range from -1 to 1 with deadband already accounted for.
   * @param lookAway      Face the robot towards the opposing alliance's wall in the same direction the driver is
   *                      facing
   * @param lookTowards   Face the robot towards the driver
   * @param lookLeft      Face the robot left
   * @param lookRight     Face the robot right
   * @param lookbet
   * @param lookfeed
   * @param looksource
   */
  public AbsoluteDriveAdv(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingAdjust,
                          BooleanSupplier lookAway, BooleanSupplier lookTowards, BooleanSupplier lookLeft,
                          BooleanSupplier lookRight, BooleanSupplier lookbet, BooleanSupplier lookfeed, BooleanSupplier looksource)
  {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.headingAdjust = headingAdjust;
    this.lookAway = lookAway;
    this.lookTowards = lookTowards;
    this.lookLeft = lookLeft;
    this.lookRight = lookRight;
    this.lookbet = lookbet;
    this.lookfeed = lookfeed;
    this.looksource = looksource;

    addRequirements(swerve);
  }

  @Override
  public void initialize()
  {
    if (Robot.encerraInverte == true) {
      swerve.setupGyroToAlliance();
    }
    Timer.delay(0.05);
    System.out.println("Entrou");
    Robot.encerraInverte = false;

    Rotation2d currentHeading = swerve.getHeading();
    headingX = currentHeading.getSin();
    headingY = currentHeading.getCos();
    resetHeading = true;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if (shutter.sensor_shu.get()) {
      invert = 1;
    }
    else{
      invert = -1;
    }

     headingX = 0;
     headingY = 0;

    // These are written to allow combinations for 45 angles
    // Face Away from Drivers
    if (lookAway.getAsBoolean())
    {
      headingY = -1*invert;
    }
    // Face Right
    if (lookRight.getAsBoolean())
    {
      headingX = 1*invert;
    }
    // Face Left
    if (lookLeft.getAsBoolean())
    {
      headingX = -1*invert;
    }
    // Face Towards the Drivers
    if (lookTowards.getAsBoolean())
    {
      headingY = 1*invert;
    }

    if (lookbet.getAsBoolean())
    {
      if (swerve.redSide() == true) {
        headingY = Math.cos(Math.toRadians(-142));
        headingX = Math.sin(Math.toRadians(-142));
         
      }
      else{
        headingY = Math.cos(Math.toRadians(126));
        headingX = Math.sin(Math.toRadians(126));
      }
    }
    if (lookfeed.getAsBoolean())
    {
      if (swerve.redSide() == true) {
        headingY = Math.cos(Math.toRadians(36));
        headingX = Math.sin(Math.toRadians(36));
         
      }
      else{
        headingY = Math.cos(Math.toRadians(-40));
        headingX = Math.sin(Math.toRadians(-40));
      }
    }

    if (looksource.getAsBoolean())
    {
      if (swerve.redSide() == true) {
        headingY = Math.cos(Math.toRadians(40));
        headingX = Math.sin(Math.toRadians(40));
         
      }
      else{
        headingY = Math.cos(Math.toRadians(-40));
        headingX = Math.sin(Math.toRadians(-40));
      }
    }

    if (resetHeading)
    {
      if (headingX == 0 && headingY == 0 && Math.abs(headingAdjust.getAsDouble()) > 0)
      {
        // Get the curret Heading
        Rotation2d currentHeading = swerve.getHeading();

        // Set the Current Heading to the desired Heading
        headingX = currentHeading.getSin();
        headingY = currentHeading.getCos();
      }
      //Dont reset Heading Again
      resetHeading = false;
    }

    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), headingX, headingY);

    //desiredSpeeds = swerve.discretize(desiredSpeeds);

    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                                           Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                                           swerve.getSwerveDriveConfiguration());
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putNumber("Velocidade X: ", desiredSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Velocidade Y: ", desiredSpeeds.vyMetersPerSecond);
    SmartDashboard.putString("Translation", translation.toString());

    // Make the robot move
    if (Math.abs(headingAdjust.getAsDouble()) > 0)
    {
      resetHeading = true;
      swerve.drive(translation, (Constants.OperatorConstants.TURN_CONSTANT * -headingAdjust.getAsDouble()), true);
    } else
    {
      swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }


}
