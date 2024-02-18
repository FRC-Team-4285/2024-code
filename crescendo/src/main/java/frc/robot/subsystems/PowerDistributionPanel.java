package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PowerDistributionPanel {

    public PowerDistributionPanel(int i) {
        //TODO Auto-generated constructor stub


        try (PowerDistribution examplePD = new PowerDistribution(50, ModuleType.kRev)) {
            
            double current1 = examplePD.getCurrent(1);
            SmartDashboard.putNumber("Current Channel 1", current1);

            double current2 = examplePD.getCurrent(2);
            SmartDashboard.putNumber("Current Channel 1", current2);

            double current3 = examplePD.getCurrent(3);
            SmartDashboard.putNumber("Current Channel 1", current3);

            double current4 = examplePD.getCurrent(4);
            SmartDashboard.putNumber("Current Channel 1", current4);

            double current5 = examplePD.getCurrent(5);
            SmartDashboard.putNumber("Current Channel 5", current5);
        }
        
    }

}
