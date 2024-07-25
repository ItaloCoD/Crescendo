package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlignShutter;

public class AlignShutterCMD extends Command {
    
    private final AlignShutter alignShutterSub;

    public AlignShutterCMD(AlignShutter alignShutterSub){
        this.alignShutterSub = alignShutterSub;
        addRequirements(alignShutterSub);
    }

    @Override
    public void initialize(){
        System.out.println("entrou");
    }

    @Override
    public void execute(){
        alignShutterSub.enableAling();
        //System.out.println("executando");
    }

    @Override
    public void end(boolean interrupted){
        alignShutterSub.setFinish();
        alignShutterSub.disableAling();
        System.out.println("terminou Align Shutter");
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
