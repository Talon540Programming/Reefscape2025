package frc.robot.subsystems.endEffector

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode.RobotType;



public class EndEffector extends SubsystemBase {

    private static final LoggerEntry.Decimal logInputs_endEffectorRPM =
        logInputs.buildDecimal("endEffectorRPM");

    private static final LoggerEntry.Decimal logInputs_endEffectorAppliedVolts =
        logInputs.buildDecimal("endEffectorAppliedVolts");

    private static final LoggerEntry.Decimal logInputs_endEffectorAppliedAmps =
        logInputs.buildDecimal("endEffectorAppliedAmps");

    private static final LoggerEntry.Decimal logInputs_endEffectorAngleDeg =
        logInputs.buildDecimal("endEffectorAngleDeg");



    private static final LoggedTunableNumber kS = group.build("kS");
    private static final LoggedTunableNumber kP = group.build("kP");
    private static final LoggedTunableNumber kV = group.build("kV");
    private static final LoggedTunableNumber ClosedLoopMaxAccelerationConstraint =
        group.build("ClosedLoopMaxAccelerationConstraint");

    while (true) {
        if (RobotType == Constants.RobotMode.RobotType.COMPETITION) {
            kS = 0.0;
            kP = 0.0;
            kV = 0.0;
            ClosedLoopMaxAccelerationConstraint = 0.0;
        } else if (RobotType == Constants.RobotMode.RobotType.PRACTICE) {
            kS = 0.0;
            kP = 0.0;
            kV = 0.0;
            ClosedLoopMaxAccelerationConstraint = 0.0;
        }
    }

    
}
