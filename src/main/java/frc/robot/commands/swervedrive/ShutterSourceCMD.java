package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shutter;

public class ShutterSourceCMD extends Command {
    
    private final Shutter shutterSubsystem;

    public ShutterSourceCMD(Shutter shutterSubsystem){
        this.shutterSubsystem = shutterSubsystem;
        addRequirements(shutterSubsystem);
    }

    @Override
    public void initialize(){
        shutterSubsystem.zeroFinishSource();
        shutterSubsystem.setNotaSource();
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted){
        //pivotSubsystem.disablePivot();
        System.out.println("terminou Shooter Source");
    }

    @Override
    public boolean isFinished(){
        boolean isFinish = shutterSubsystem.sourceIsFinish();
        return isFinish;
    }
}
