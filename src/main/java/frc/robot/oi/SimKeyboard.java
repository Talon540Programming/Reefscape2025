// package frc.robot.oi;

// import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;

// public class SimKeyboard implements ControlsInterface {
//     private final GenericHID hid = new GenericHID(0);

//     @Override
//     public double getDriveX() {
//         return -controller.getLeftY();
//     }

//     @Override
//     public double getDriveY() {
//         return -controller.getLeftX();
//     }

//     @Override
//     public double getDriveTheta() {
//         return -controller.getRightX();
//     }

//     @Override
//     public Trigger robotRelativeOverride() {
//         return controller.leftBumper();
//     }

//     @Override
//     public Trigger depositL1() {
//         return controller.povDown();
//     };


//     @Override
//     public Trigger depositL2left() {
//         return controller.povDownLeft();
//     };

//     @Override
//     public Trigger depositL2right() {
//         return controller.povDownRight();
//     };

//     @Override
//     public Trigger removeL2Algae() {
//         return controller.povLeft();
//     };


//     @Override
//     public Trigger depositL3left() {
//         return controller.povUpLeft();
//     };

//     @Override
//     public Trigger depositL3right() {
//         return controller.povUpRight();
//     };

//     @Override
//     public Trigger removeL3Algae() {
//         return controller.povRight();
//     };
    

//     @Override
//     public Trigger intake() {
//         return controller.rightBumper();
//     };
// }
