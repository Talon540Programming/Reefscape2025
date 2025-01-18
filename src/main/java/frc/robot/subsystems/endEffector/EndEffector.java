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

    private static final LoggerEntry.Decimal logInputs_intakeSideTOFDistanceInches =
        logInputs.buildDecimal("IntakeSideTOFDistanceInches");

    private static final LoggerEntry.Decimal logInputs_shooterSideTOFDistanceInches =
        logInputs.buildDecimal("ShooterSideTOFDistanceInches");
}
