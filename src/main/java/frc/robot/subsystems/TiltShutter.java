package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TiltShutter extends SubsystemBase {
    private double offsetAngleLime;
    private double anguloMontagemLime = 60.0;
    private double alturaLimeSolo = 23.2283;
    private double alturaTagsSolo = 56.6929;
    private double anguloTotal;
    private double anguloToRadiano;
    private double distancia;
    private double txPivot;
    private double tyPivot;
    private double output;
    private double outputLastValue;
    private double tagIsDetected;
    private  double  pathX;
    private  double  pathY;
    private  double  d;
    public   static double shutterVel;

    private NetworkTable tablePivot = NetworkTableInstance.getDefault().getTable("limelight-shutter");
 
@Override
public void periodic(){
    tagIsDetected = tablePivot.getEntry("tv").getDouble(0);
    txPivot = tablePivot.getEntry("tx").getDouble(0.0);
    tyPivot = tablePivot.getEntry("ty").getDouble(0.0);
    SmartDashboard.putNumber("txShutter: ", txPivot);
    SmartDashboard.putNumber("tyShutter: ", tyPivot);

    pathX = ((0.5*txPivot)+43.02)/23.9;
    SmartDashboard.putNumber("Path X: ", pathX);
    pathY = ((0.06*tyPivot)-1.59)/-0.6;
    SmartDashboard.putNumber("Path Y: ", pathY);
    calculate();
    SmartDashboard.putNumber("Valor Distâcia", distancia);
    SmartDashboard.putNumber("Inclinação", outputLastValue);
}

public void calculate(){

    offsetAngleLime = Math.abs(tyPivot);
    anguloTotal = anguloMontagemLime + offsetAngleLime;
    anguloToRadiano = anguloTotal*(Math.PI/180);
    distancia = (alturaTagsSolo-alturaLimeSolo)/Math.tan(anguloToRadiano);
    output = (-5.7*distancia+(5.7*16))/(2.46-16);

    if (tagIsDetected != 0) {
        outputLastValue = output;
    }
    if (distancia >= 12) {
        shutterVel = 0.8;
    }
    if (distancia <= 12) {
        shutterVel = 1;
    }
}

public double getTilt(){
    return outputLastValue;
}
public double pathX(){
    return pathX;
}
public double pathY(){
    return pathY;
}

public void pipeRed(){
    tablePivot.getEntry("pipeline").setNumber(0);
}

public void pipeBlue(){
    tablePivot.getEntry("pipeline").setNumber(1);
}

}
