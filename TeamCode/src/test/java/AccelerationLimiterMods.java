import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities.AccelerationLimiter;
import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.Mecanum;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;
import org.firstinspires.ftc.teamcode.Utilities.Waypoints;
import org.firstinspires.ftc.teamcode.Utilities.Waypoints.LabeledWaypoint;
import org.junit.Before;
import org.junit.Test;

import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import static com.google.common.truth.Truth.assertThat;
import static org.firstinspires.ftc.teamcode.RobotHardware.StartPosition.FIELD_LOADING;
import static org.firstinspires.ftc.teamcode.Utilities.Waypoints.LocationLoading.ALIGNMENT_POSITION_A;
import static org.firstinspires.ftc.teamcode.Utilities.Waypoints.LocationLoading.GRAB_SKYSTONE_A;

public class AccelerationLimiterMods {

    DecimalFormat df;

    private void assertMecanumWheelEqual(Mecanum.Wheels wheels1, Mecanum.Wheels wheels2) {
        assertThat(wheels1.backLeft).isEqualTo(wheels2.backLeft);
        assertThat(wheels1.backRight).isEqualTo(wheels2.backRight);
        assertThat(wheels1.frontLeft).isEqualTo(wheels2.frontLeft);
        assertThat(wheels1.frontRight).isEqualTo(wheels2.frontRight);
    }

    private void printWheels(Mecanum.Wheels wheels) {
        System.out.println( wheels.backLeft + " , " + wheels.backRight + " , " + wheels.frontLeft + " , " + wheels.frontRight);
    }

    @Test
    public void TestJoystickInputModifications() {
        // The way Joystick inputs are calculated has been modified, but should be equivalent.
        ArrayList<Double> joystickInputs = new ArrayList();
        joystickInputs.add(-1.0);
        joystickInputs.add(0.0);
        joystickInputs.add(0.5);
        joystickInputs.add(1.0);

        Mecanum.Wheels oldWheels;
        Mecanum.Wheels newWheels;

//        double leftStickX = 0;
//        double leftStickY = 0;
        double rightStickX = 0;
        double rightStickY = 0;
        for(Double leftStickX: joystickInputs) {
            for(Double leftStickY: joystickInputs) {
                oldWheels = Mecanum.old_simpleJoystickToWheels(leftStickX,leftStickY,rightStickX,rightStickY);
                newWheels = Mecanum.simpleJoystickToWheels(leftStickX,leftStickY,rightStickX,rightStickY);
                assertMecanumWheelEqual(oldWheels,newWheels);

//                // Print values
//                System.out.println(leftStickX + "  " +  leftStickY);
//                printWheels(oldWheels);
//                printWheels(newWheels);
            }
        }
    }


    AccelerationLimiter accelerationLimiter = new AccelerationLimiter(1,1);
    double simTime = 0.0;

    private void printCommand(Mecanum.Command command) {
        System.out.println(df.format(command.vx) + ", " + df.format(command.vy) + ", " + df.format(command.av));
    }

    @Before
    public void initializeTest() {
        df = new DecimalFormat("0.00");
    }

    @Test
    public void TestAccelerationLimit() {

        Mecanum.Command inputCommand = new Mecanum.Command(0,0,0);
        while(simTime < 10.0) {

            if(simTime < 2.0) {
                inputCommand = new Mecanum.Command(0.0, 0.0, 1);
            } else if (simTime < 4.0){
                inputCommand = new Mecanum.Command(0.0, 0.0, -1);
            } else if (simTime < 6.0){
                inputCommand = new Mecanum.Command(-1.0, 0.0, 0);
            } else if (simTime < 8.0) {
                inputCommand = new Mecanum.Command(.1, -1.0, .1);
            }



            Mecanum.Command limitedCommand = accelerationLimiter.updateAndReturnMecanumCommand(simTime,inputCommand);



            // Print results
            System.out.print("Time: " + df.format(simTime) + "   ");
            System.out.print("Command Input: ");
            printCommand(inputCommand);
            System.out.print("Command Limited Output:     ");
            printCommand(limitedCommand);
            System.out.println("");

            simTime += 0.1;
        }






    }

}
