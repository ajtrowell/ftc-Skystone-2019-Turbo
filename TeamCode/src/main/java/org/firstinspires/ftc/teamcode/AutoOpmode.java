package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utilities.CSV;
import org.firstinspires.ftc.teamcode.Utilities.Mutable;
import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.RobotStateContext;
import org.firstinspires.ftc.teamcode.Utilities.TimingMonitor;

public class AutoOpmode extends RobotHardware {

    public TimingMonitor timingMonitor;
    protected Color.Ftc robotColor;
    protected StartPosition robotStartPos;
    public RobotStateContext robotStateContext;
//    public SimpleVision simpleVision;
    public Thread thread;

    // Telemetry Recorder
    private CSV csvWriter;
    private CSV controlWriter;
    private boolean writeControls = false;

    //Interactive Init menu
    private Mutable<Boolean> Simple = new Mutable<>(false);
    private Mutable<Double> AutoDriveSpeed = new Mutable<>(0.5);
    private Mutable<Boolean> RecordTelemetry = new Mutable<>(false);
    private Mutable<Boolean> earlyFlagDrop = new Mutable<>(false);

    @Autonomous(name="auto.Red.Pickup", group="Auto")
    public static class AutoRedPickup extends AutoOpmode {
        @Override public void init() {
            robotColor = Color.Ftc.RED;
            robotStartPos = StartPosition.FIELD_PICKUP;
            super.init();
        }
    }

    @Autonomous(name="auto.Red.Build", group="Auto")
    public static class AutoRedBuild extends AutoOpmode {
        @Override public void init() {
            robotColor = Color.Ftc.RED;
            robotStartPos = StartPosition.FIELD_BUILD;
            super.init();
        }
    }

    @Autonomous(name="auto.Blue.Pickup", group="Auto")
    public static class AutoBluePickup extends AutoOpmode {
        @Override public void init() {
            robotColor = Color.Ftc.BLUE;
            robotStartPos = StartPosition.FIELD_PICKUP;
            super.init();
        }
    }

    @Autonomous(name="auto.Blue.Build", group="Auto")
    public static class AutoBlueBuild extends AutoOpmode {
        @Override public void init() {
            robotColor = Color.Ftc.BLUE;
            robotStartPos = StartPosition.FIELD_BUILD;
            super.init();
        }
    }

    @Override
    public void init() {
        super.init();
        timingMonitor = new TimingMonitor(AutoOpmode.this);
        thread = new Thread(new VisionLoader());
        thread.start();
        robotStateContext = new RobotStateContext(AutoOpmode.this, robotColor, robotStartPos);
        robotStateContext.init();
        telemetry.addData("Initialization:", "Successful!");

        // Initialization Menu
        interactiveInit.addDouble(AutoDriveSpeed, "DriveSpeed",0.8,1.0,.1,.3,.5);
        interactiveInit.addBoolean(RecordTelemetry,"Record Telemetry", true, false);
        interactiveInit.addBoolean(Simple, "Simple Mode", true, false);
        interactiveInit.addBoolean(earlyFlagDrop, "Drop flag early", false, true);
    }

    @Override
    public void init_loop() {
        super.init_loop();

//        if (simpleVision == null) {
//            telemetry.addData("Vision:", "LOADING...");
//        } else {
//            telemetry.addData("Vision:", "INITIALIZED");
//        }
    }

    @Override
    public void start() {
        super.start();

        if(RecordTelemetry.get()) {
            csvWriter = new CSV(this);
            csvWriter.open("telemetry.csv");

            if (writeControls) {
                controlWriter = new CSV(this);
                controlWriter.open("controls.csv");
            }
            recordConstantsToFile();
        }
    }

    @Override
    public void loop() {
        timingMonitor.loopStart();
        if(controller1.start()) { timingMonitor.reset();} // Clear with start button
        super.loop();
        timingMonitor.checkpoint("POST super.loop()");
        robotStateContext.update();
        timingMonitor.checkpoint("POST robotStateMachine.update()");
        if ( useIMU.get() ) {
            imuUtilities.update();
            imuUtilities.getCompensatedHeading();
            timingMonitor.checkpoint("POST imuUtilities.update()");
        }

        // Conditional Telemetry Recording
        if(RecordTelemetry.get()) {
            if(writeControls) {
                writeControlsToFile();
            }
            writeTelemetryToFile();
            timingMonitor.checkpoint("POST telemetry recorder");
        }

        // Telemetry
        mecanumNavigation.displayPosition();
        telemetry.addLine();
        telemetry.addData("State:", robotStateContext.getCurrentState());
        telemetry.addLine();
        telemetry.addData("Period Average (sec)", df_prec.format(getAveragePeriodSec()));
        telemetry.addData("Period Max (sec)", df_prec.format(getMaxPeriodSec()));
        timingMonitor.displayMaxTimes();
        timingMonitor.checkpoint("POST TELEMETRY");

        try {
//            simpleVision.updateTensorFlow(true);
//            simpleVision.displayTensorFlowDetections();
        } catch(Exception e) {
            telemetry.addData("Vision Not Loaded", "");
        }
        timingMonitor.checkpoint("POST Vision");
        telemetry.addData("Lift Ticks",getEncoderValue(MotorName.LEFT_LIFT_WINCH));
    }

    @Override
    public void stop() {
        super.stop();
        closeCSV();
    }




    // Initialize vuforia in a separate thread to avoid init() hangups.
    class VisionLoader implements Runnable {
        public void run() {
//            simpleVision = new SimpleVision(getVuforiaLicenseKey(), AutoOpmode.this,
//                    false, true,false,
//                    false, false);
        }
    }



    private void recordConstantsToFile() {
        CSV constantsWriter = new CSV(this);
        constantsWriter.open("constants.csv");
        constantsWriter.addFieldToRecord("drive_wheel_diameter", Constants.DRIVE_WHEEL_DIAMETER_INCHES);
        constantsWriter.addFieldToRecord("wheelbase_width_in", Constants.WHEELBASE_WIDTH_IN);
        constantsWriter.addFieldToRecord("wheelbase_length_in", Constants.WHEELBASE_LENGTH_IN);
        constantsWriter.addFieldToRecord("wheelbase_k", Math.abs(Constants.WHEELBASE_LENGTH_IN/2.0)
                + Math.abs(Constants.WHEELBASE_WIDTH_IN/2.0));
        constantsWriter.addFieldToRecord("drive_wheel_steps_per_rotation", (double)Constants.DRIVE_WHEEL_STEPS_PER_ROT);
        constantsWriter.completeRecord();
        constantsWriter.close();
    }


    private void writeControlsToFile() {
        controlWriter.addFieldToRecord("time",time);

        controlWriter.addFieldToRecord("left_stick_x", controller1.left_stick_x);
        controlWriter.addFieldToRecord("left_stick_y", controller1.left_stick_y);
        controlWriter.addFieldToRecord("right_stick_x", controller1.right_stick_x);
        controlWriter.addFieldToRecord("right_stick_y", controller1.right_stick_y);
        controlWriter.addFieldToRecord("left_trigger", controller1.left_trigger);
        controlWriter.addFieldToRecord("right_trigger", controller1.right_trigger);

        controlWriter.addFieldToRecord("right_stick_button", controller1.rightStickButton() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("left_stick_button", controller1.leftStickButton() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("right_bumper", controller1.rightBumper() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("left_bumper", controller1.leftBumper() ? 1.0 : 0.0);

        controlWriter.addFieldToRecord("a_button", controller1.A() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("b_button", controller1.B() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("x_button", controller1.X() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("y_button", controller1.Y() ? 1.0 : 0.0);

        controlWriter.completeRecord();
    }


    private void writeTelemetryToFile() {
        // setFieldData sets both titles and recordData.
        csvWriter.addFieldToRecord("time",time);
        // Capture all servo positions:
        for (ServoName s : ServoName.values()) {
            csvWriter.addFieldToRecord(s.name(), getAngle(s));
        }
        // Capture all motor encoder values:
        for (MotorName m : MotorName.values()) {
            csvWriter.addFieldToRecord(m.name()+"_ticks", (double)getEncoderValue(m));
        }
        // Capture all motor power levels:
        for (MotorName m : MotorName.values()) {
            csvWriter.addFieldToRecord(m.name()+"_power", getPower(m));
        }
        // Capture mecanumNavigation current position
        csvWriter.addFieldToRecord("x_in",mecanumNavigation.currentPosition.x);
        csvWriter.addFieldToRecord("y_in",mecanumNavigation.currentPosition.y);
        csvWriter.addFieldToRecord("theta_rad",mecanumNavigation.currentPosition.theta);
        if(useIMU.get()) {
            csvWriter.addFieldToRecord("IMU_heading",imuUtilities.getCompensatedHeading());
        }



        // Add IMU data to current csvWriter record
        //addIMUToRecord(csvWriter);

        // Writes record to file if writer is open.
        csvWriter.completeRecord();

        telemetry.addData("WRITE CONTROLS",writeControls);
        if(writeControls) {
            writeControlsToFile();
        }
    }

    void closeCSV() {
        if(RecordTelemetry.get()) {
            csvWriter.close();
            if (writeControls) {
                controlWriter.close();
            }
        }

    }
}
