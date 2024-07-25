// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.intake;
import frc.robot.Constants.shutter;
import frc.robot.commands.swervedrive.AlignAmpCMD;
import frc.robot.commands.swervedrive.AlignShutterCMD;
import frc.robot.commands.swervedrive.DisablePivot;
import frc.robot.commands.swervedrive.PivotCMD;
import frc.robot.commands.swervedrive.ShutterAMPCMD;
import frc.robot.commands.swervedrive.ShutterSourceCMD;
import frc.robot.commands.swervedrive.ShutterSpeakerCMD;
import frc.robot.commands.swervedrive.TiltCMD;
import frc.robot.commands.swervedrive.TroubleIntakeCMD;
import frc.robot.commands.swervedrive.limeGoAmpCMD;
import frc.robot.commands.swervedrive.limeGoGroundCMD;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.AlignAmp;
import frc.robot.subsystems.AlignShutter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shutter;
import frc.robot.subsystems.TiltShutter;
import frc.robot.subsystems.TroubleIntake;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  public static Object pipeBlue;
  //private Command autonomousCommand;
  private Intake pegaNota = new Intake();
  private Shutter lanca = new Shutter();
  private Pivot pivot = new Pivot();
  private TiltShutter tilt = new TiltShutter();
  private TroubleIntake troubleItk = new TroubleIntake();

  private AlignShutter alignshutter = new AlignShutter();
  private final AlignShutterCMD alignShutterCMD  = new AlignShutterCMD(alignshutter);
  private final TiltCMD tiltCMD = new TiltCMD(tilt, pivot);

  private AlignAmp alignamp = new AlignAmp();
  private final AlignAmpCMD alignAmpCMD = new AlignAmpCMD(alignamp);
  private Trigger sensorShutter = new Trigger(() -> shutter.sensor_shu.get());
  private Trigger nota = new Trigger(()-> alignamp.getYAMP());



  private Command alinhador;
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  // Replace with CommandPS4Controller or CommandJoystick if needed
  static final CommandXboxController driverXbox = new CommandXboxController(0);
  static final CommandXboxController driverXbox2 = new CommandXboxController(1);
   SendableChooser<Command> autoChooser;
  


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    NamedCommands.registerCommand("check1", new InstantCommand());
    NamedCommands.registerCommand("check2", new InstantCommand());
    NamedCommands.registerCommand("check3", new InstantCommand());
    NamedCommands.registerCommand("check4", tiltCMD);
    NamedCommands.registerCommand("check5", new InstantCommand());
    NamedCommands.registerCommand("check6", new InstantCommand());
    initializeAutonomousCommands();
    alinhador = Commands.parallel(alignShutterCMD, tiltCMD);
    configureBindings();
    
  //autoChooser = AutoBuilder.buildAutoChooser();
  SmartDashboard.putData("Auto Chooser", autoChooser);

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband((driverXbox.getLeftY()*alignamp.getAtiva())+alignamp.getOutAmp(),
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(((driverXbox.getRightX()*-1)*alignshutter.getAtiva())+alignshutter.getOutShutter(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   driverXbox.getHID()::getYButtonPressed,
                                                                   driverXbox.getHID()::getAButtonPressed,
                                                                   driverXbox.getHID()::getXButtonPressed,
                                                                   driverXbox.getHID()::getBButtonPressed,
                                                                   driverXbox.getHID()::getLeftBumperPressed,
                                                                   driverXbox.getHID()::getRightBumperPressed,
                                                                   driverXbox.getHID()::getBackButtonPressed
                                                                   );

    drivebase.setDefaultCommand(closedAbsoluteDriveAdv);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    /* CONTROLE CO-DRIVER */
    driverXbox2.axisGreaterThan(3, 0.1).onTrue(alignAmpCMD);
    driverXbox2.axisGreaterThan(3, 0.1).onFalse(new InstantCommand(alignAmpCMD::cancel));
    driverXbox2.axisGreaterThan(3, 0.1).and(driverXbox2.rightBumper()).onTrue(
    Commands.sequence(new PivotCMD(pivot, -20), new ShutterAMPCMD(lanca), new DisablePivot(pivot, 0)));

    driverXbox2.axisGreaterThan(2, 0.1).onTrue(alinhador);
    driverXbox2.axisGreaterThan(2, 0.1).onFalse(new InstantCommand(alinhador::cancel).andThen(new DisablePivot(pivot, 0)));
    driverXbox2.axisGreaterThan(2, 0.1).and(driverXbox2.leftBumper()).onTrue(new ShutterSpeakerCMD(lanca));

    driverXbox2.y().onTrue(new ShutterSourceCMD(lanca));

    driverXbox2.povDown().toggleOnTrue(new TroubleIntakeCMD(troubleItk));
    driverXbox2.povUp().onTrue(new InstantCommand(pegaNota::getNotaSolo));
    /* CONTROLE CO-DRIVER */

    /* CONTROLES GERAIS */
    nota.onTrue(new InstantCommand(pegaNota::getNotaSolo));
    sensorShutter.onTrue(new limeGoAmpCMD(alignamp));
    sensorShutter.onFalse(new limeGoGroundCMD(alignamp));
    /* CONTROLES GERAIS */

    /* CONTROLE DRIVER */
    driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    /* CONTROLE DRIVER */


    

    
    //driverXbox.leftBumper().onTrue(new ShutterSpeakerCMD(lanca));

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    //driverXbox.axisGreaterThan(2, 0.1).toggleOnTrue(new AlignAmpCMD(alignamp));
    //driverXbox.back().toggleOnTrue(Commands.parallel(new AlignShutterCMD(alignshutter), new TiltCMD(tilt, pivot)));
    
    //driverXbox.axisGreaterThan(3, 0.1).onTrue(Commands.sequence(new PivotCMD(pivot, -14), new ShutterAMPCMD(lanca), new DisablePivot(pivot, 2)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand()
  //{
  //  //return autoChooser.getSelected();
  //  // An example command will be run in autonomous
  //}

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
  private void initializeAutonomousCommands() {
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Auto01", drivebase.getAutonomousCommand("Auto01"));
    autoChooser.addOption("Auto02", drivebase.getAutonomousCommand("Auto02"));
    autoChooser.addOption("Auto03", drivebase.getAutonomousCommand("Auto03"));

    SmartDashboard.putData("Auto Chooser", autoChooser);
   }

public Command getAutonomousCommand() {
  return autoChooser.getSelected();
}

public Command zeroShutter(){
  return new DisablePivot(pivot, 2);
}
public void setupGyro(){
  drivebase.setupGyroToAlliance();
}

public void setPipeAlliance()
  {
    if (drivebase.isRedAlliance())
    {
      tilt.pipeRed();
    } else
    {
      tilt.pipeBlue();
    }
  }

}
