package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlignAmp;

public class limeGoAmpCMD extends Command {
    
    private final AlignAmp limeGoAmpSubsystem;

    public limeGoAmpCMD(AlignAmp limeGoAmpSubsystem){
        this.limeGoAmpSubsystem = limeGoAmpSubsystem;
        addRequirements(limeGoAmpSubsystem);
    }

    @Override
    public void initialize(){
        limeGoAmpSubsystem.resetFinish();
        limeGoAmpSubsystem.camGoToAMP();
    }

    @Override
    public void execute(){
        limeGoAmpSubsystem.compareAmp();
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("terminou limeGoAMP");
    }

    @Override
    public boolean isFinished(){
        boolean isFinish = limeGoAmpSubsystem.systemFinish();
        return isFinish;
    }
}
