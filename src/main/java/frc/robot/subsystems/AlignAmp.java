package frc.robot.subsystems;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intake;


public class AlignAmp extends SubsystemBase{
    private NetworkTable tableAmp = NetworkTableInstance.getDefault().getTable("limelight");
    private final PIDController pidAmp = new PIDController(0.03, 0, 0); // Speaker: (0.03, 0.05, 0.001)
    private         double  outputAmp;
    private         double  ativaAMP;
    private         double  txAmp;
    private         double  tyAmp;
    private         boolean isFinish;

public AlignAmp(){
    pidAmp.setIZone(ativaAMP);
    pidAmp.setTolerance(1);
    ativaAMP = 1;
    outputAmp = 0;
    isFinish = false;
}

@Override
public void periodic(){
    if (Math.abs(txAmp) <=13 && Math.abs(txAmp) > 6) {
        pidAmp.setP(0.05);
    }
    else if (Math.abs(txAmp) > 21.5) {
        pidAmp.setP(0.02);
    }
    else if (Math.abs(txAmp) <= 6) {
        pidAmp.setP(0.1);
    }
    else{
        pidAmp.setP(0.03);
    }

    txAmp = tableAmp.getEntry("tx").getDouble(0.0);
    tyAmp = tableAmp.getEntry("ty").getDouble(0.0);
    SmartDashboard.putNumber("txAmp: ", txAmp);
    SmartDashboard.putNumber("tyAmp: ", tyAmp);
    SmartDashboard.putNumber("outAmp: ", outputAmp);
    SmartDashboard.putNumber("Parametro P: ", pidAmp.getP());
    
}

public void enableAling(){
        ativaAMP = 0;
        outputAmp = -1*pidAmp.calculate(tableAmp.getEntry("tx").getDouble(0.0), 0);
}

public void disableAling(){
        ativaAMP = 1;
        outputAmp = 0;
}

public double getOutAmp(){
    return outputAmp;
}

public double getAtiva(){
    return ativaAMP;
}

public void setFinish(){
    isFinish = true;
}


public void resetFinish(){
    isFinish = false;
}

public BooleanSupplier saida(){
    return () -> isFinish;
}

public boolean systemFinish(){
    return isFinish;
}

public void camGoToAMP(){
        tableAmp.getEntry("pipeline").setNumber(0);
        intake.servoCam.setAngle(130);
}

public void camGoToGround(){
        tableAmp.getEntry("pipeline").setNumber(1);
        intake.servoCam.setAngle(0);
}

public void compareAmp(){
        isFinish = true;
}
public boolean getYAMP(){
    boolean notaIsPresent;
    if (tyAmp < -10) {
        notaIsPresent = true;
    }
    else{
        notaIsPresent = false;
    }
    return notaIsPresent;
}

}
