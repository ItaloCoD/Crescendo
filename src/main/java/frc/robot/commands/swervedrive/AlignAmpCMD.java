package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlignAmp;

public class AlignAmpCMD extends Command {
    
    private final AlignAmp alignAmpSub;

    public AlignAmpCMD(AlignAmp alignAmpSub){
        this.alignAmpSub = alignAmpSub;
        addRequirements(alignAmpSub);
    }

    @Override
    public void initialize(){
        System.out.println("entrou");
    }

    @Override
    public void execute(){
        alignAmpSub.enableAling();
        //System.out.println("executando");
    }

    @Override
    public void end(boolean interrupted){
        alignAmpSub.disableAling();
        alignAmpSub.setFinish();
        System.out.println("terminou");
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
