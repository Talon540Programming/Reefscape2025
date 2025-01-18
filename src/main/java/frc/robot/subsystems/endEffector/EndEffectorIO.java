
package frc.robot.subsystems.endEffector;

public interface EndEffectorIO {

    public class EndEffectorIO {
        double endEffectorappledVolts = 0.0;
        double endEffectorappliedAmps = 0.0;
        double endEffectorRPM = 0.0;
        double endEffectorAngleDeg = 0.0;
        
    }

    public void setEndEffectorVolts(double volts); {
        double endEffectorappliedVolts = volts;
    }
    
    public void setEndEffectorAmps(double amps); {
        double endEffectorappliedAmps = amps;
    }

    public void setEndEffectorRPM(double RPM); {
        double endEffectorRPM = RPM;
    }

    public void setEndEffectorAngleDeg(double angleDeg); {
        double endEffectorAngleDeg = angleDeg;
    }
}