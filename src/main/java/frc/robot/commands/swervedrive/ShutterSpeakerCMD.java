package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlignShutter;
import frc.robot.subsystems.Shutter;

public class ShutterSpeakerCMD extends Command {
    
    private final Shutter shutterSubsystem;

    public ShutterSpeakerCMD(Shutter shutterSubsystem){
        this.shutterSubsystem = shutterSubsystem;
        addRequirements(shutterSubsystem);
    }

    @Override
    public void initialize(){
        shutterSubsystem.zeroFinishSpeaker();
        shutterSubsystem.setNotaSperker();
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted){
        //pivotSubsystem.disablePivot();
        AlignShutter.resetFinish();
        System.out.println("terminou Shooter Speaker");
    }

    @Override
    public boolean isFinished(){
        boolean isFinish = shutterSubsystem.speakerIsFinish();
        return isFinish;
    }
}
