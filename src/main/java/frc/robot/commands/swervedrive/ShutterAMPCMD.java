package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shutter;

public class ShutterAMPCMD extends Command {
    
    private final Shutter shutterSubsystem;

    public ShutterAMPCMD(Shutter shutterSubsystem){
        this.shutterSubsystem = shutterSubsystem;
        addRequirements(shutterSubsystem);
    }

    @Override
    public void initialize(){
        shutterSubsystem.zeroFinishAmp();
        shutterSubsystem.setNotaAmp();
        System.out.println("Entrou Shooter Amp");
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted){
        //pivotSubsystem.disablePivot();
        System.out.println("terminou Shooter Amp");
    }

    @Override
    public boolean isFinished(){
        boolean isFinish = shutterSubsystem.ampIsFinish();
        return isFinish;
    }
}
