package frc.robot.subsystems.endEffector;

public interface EndEffectorIO {

    public class EndEffectorIO {
        double endEffectorAppledVolts = 0.0;
        double endEffectorAppliedAmps = 0.0;
        double endEffectorRPM = 0.0;
        double endEffectorAngleDeg = 0.0;
        
    }

    public void setEndEffectorVolts(double volts); {
        double endEffectorAppliedVolts = volts;
    }
    
    public void setEndEffectorAmps(double amps); {
        double endEffectorAppliedAmps = amps;
    }

    public void setEndEffectorRPM(double RPM); {
        double endEffectorRPM = RPM;
    }

    public void setEndEffectorAngleDeg(double angleDeg); {
        double endEffectorAngleDeg = angleDeg;
    }
}