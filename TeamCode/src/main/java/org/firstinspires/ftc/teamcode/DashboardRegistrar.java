package org.firstinspires.ftc.teamcode; // Make sure this package name matches yours

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

/**
 * This class is used to initialize the FTC Dashboard
 * as soon as the robot controller app starts.
 */
public class DashboardRegistrar {
    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        // This is the important line!
        // It gets the instance of the dashboard, which in turn causes it to
        // register itself with the web server.
        // We don't need to do anything with the instance, just calling the method is enough.
        FtcDashboard.getInstance();

        // Optional: If you want to see a confirmation on the RC screen, you can uncomment this.
        // It will create a dummy OpMode that appears in the list.
        /*
        manager.register("Dashboard Initialized", new OpMode() {
            @Override
            public void init() {
                telemetry.addLine("FTC Dashboard is running.");
                telemetry.update();
            }

            @Override
            public void loop() {}
        });
        */
    }
}
