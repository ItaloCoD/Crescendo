package frc.robot.subsystems;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AlignShutter extends SubsystemBase{
    private NetworkTable tableShutter = NetworkTableInstance.getDefault().getTable("limelight-shutter");
    private final PIDController pidShutter = new PIDController(0.0145, 0.01, 0.0); // Speaker: 0.013 Amp:
    private  double  outputShutter;
    private  double  ativaShutter;
    private static  boolean isFinish2;

public AlignShutter(){
    pidShutter.setIZone(Math.abs(13));
    pidShutter.setTolerance(1);
    ativaShutter = 1;
    outputShutter = 0;
    isFinish2 = false;
}

@Override
public void periodic(){
    SmartDashboard.putNumber("outShutter: ", outputShutter);
}

public void enableAling(){
        ativaShutter = 0;
        outputShutter = pidShutter.calculate(tableShutter.getEntry("tx").getDouble(0.0), 0);
}

public void disableAling(){
        ativaShutter = 1;
        outputShutter = 0;
}

public double getOutShutter(){
    return outputShutter;
}

public double getAtiva(){
    return ativaShutter;
}

public void setFinish(){
    isFinish2 = true;
}


public static void resetFinish(){
    isFinish2 = false;
}

public BooleanSupplier saida(){
    return () -> isFinish2;
}

}
