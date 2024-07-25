package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlignAmp;

public class limeGoGroundCMD extends Command {
    
    private final AlignAmp limeGoGroundSubsystem;

    public limeGoGroundCMD(AlignAmp limeGoGroundSubsystem){
        this.limeGoGroundSubsystem = limeGoGroundSubsystem;
        addRequirements(limeGoGroundSubsystem);
    }

    @Override
    public void initialize(){
        limeGoGroundSubsystem.resetFinish();
        limeGoGroundSubsystem.camGoToGround();
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("terminou limeGOGround");
    }

    @Override
    public boolean isFinished(){
        boolean isFinish = limeGoGroundSubsystem.systemFinish();
        return isFinish;
    }
}
