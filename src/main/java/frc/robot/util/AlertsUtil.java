// package frc.robot.util;
//
// import edu.wpi.first.math.filter.Debouncer;
// import edu.wpi.first.wpilibj.*;
// import edu.wpi.first.wpilibj.Alert.AlertType;
// import java.util.ArrayList;
// import java.util.List;
// import java.util.function.BooleanSupplier;
//
// public class AlertsUtil {
//   private static final double jitTimeout = 45.0;
//
//   private static final List<Runnable> debouncedAlertRunnables = new ArrayList<>();
//
//   public static void createDebouncedAlert(
//       String text, AlertType type, double timeout, boolean respectJIT, BooleanSupplier condition)
// {
//     var baseAlert = new Alert(text, type);
//     var alertDebouncer = new Debouncer(timeout);
//
//     debouncedAlertRunnables.add(
//         () ->
//             baseAlert.set(
//                 alertDebouncer.calculate(condition.getAsBoolean())
//                     && (!respectJIT || hasJITCompiled())));
//   }
//
//   public static void createDebouncedAlert(
//       String text, AlertType type, double timeout, BooleanSupplier condition) {
//     createDebouncedAlert(text, type, timeout, false, condition);
//   }
//
//   public static void createOneShotAlert(String text, AlertType type) {
//     new Alert(text, type).set(true);
//   }
//
//   public static void updateAlerts() {
//     for (var alert : debouncedAlertRunnables) {
//       alert.run();
//     }
//   }
//
//   private static boolean hasJITCompiled() {
//     return Timer.getTimestamp() < jitTimeout;
//   }
// }
