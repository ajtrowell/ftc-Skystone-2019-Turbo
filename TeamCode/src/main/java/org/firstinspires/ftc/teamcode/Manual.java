package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utilities.AutoDrive;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Executive;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utilities.Mutable;

@TeleOp (name="Manual",group="Competition")
public class Manual extends RobotHardware {

    //Adding interactive init variables
    private Mutable<Double> LiftSpeed = new Mutable<>(1.0);
    private Mutable<Boolean> CoPilot = new Mutable<>(false);
    private Mutable<Double> Exponential = new Mutable<>(1.0);
    private Mutable<Double> DriveSpeed = new Mutable<>(1.0);

    private double lifterSpeed;
    private boolean copilotEnabled;
    private double exponential;
    private double driveSpeed;

    private Executive.StateMachine manualState;

    @Override
    public void init() {
        super.init();
        interactiveInit.addDouble(LiftSpeed, "Lifter speed", 0.1, 0.2, .3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0);
        interactiveInit.addDouble(DriveSpeed, "Slow Mode Multiplier",  0.25, 0.5, 0.75, 1.0);
        interactiveInit.addDouble(Exponential, "Exponential", 3.0, 1.0);
        interactiveInit.addBoolean(CoPilot, "Copilot Enable", false, true);

        manualState = new Executive.StateMachine(this);
        manualState.changeState(Executive.StateMachine.StateType.DRIVE, new ManualState());
        manualState.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        lifterSpeed = LiftSpeed.get();
        exponential = Exponential.get();
        driveSpeed = DriveSpeed.get();
        copilotEnabled = CoPilot.get();
    }

    @Override
    public void loop() {
        super.loop();

        // Telemetry
        mecanumNavigation.displayPosition();
        manualState.update();
    }


    public class ManualState extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            // Mecanum Drive Control
            setDriveForSimpleMecanum(Math.pow(controller1.left_stick_x, exponential) * driveSpeed,
                    Math.pow(controller1.left_stick_y, exponential) * driveSpeed,
                    Math.pow(controller1.right_stick_x, exponential) * driveSpeed,
                    Math.pow(controller1.right_stick_y, exponential) * driveSpeed);
            nonDriveControls();
        }
    }

    private void nonDriveControls() {
        // Based on copilotEnabled, sets controls for
        // Arm, Wrist, Feeder
        // Feeder Lift, and Lift Winch
        if (copilotEnabled) {
            // Copilot Controls

            if(controller1.YOnce()) {
                mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0, 0, 0));
            }

            if (controller2.leftBumper()) {
                setAngle(ServoName.FOUNDATION, 0.5);
                telemetry.addData("SERVO: ", "UP");
            } else if (controller2.rightBumper()) {
                setAngle(ServoName.FOUNDATION, 0.8);
                telemetry.addData("SERVO: ", "DOWN");
            }

            // Lift Control
            setPower(MotorName.LEFT_LIFT_WINCH, Math.pow(controller2.left_stick_y, exponential) * lifterSpeed);
        } else {
            // Pilot Controls

            if (controller2.leftBumper()) {
                setAngle(ServoName.FOUNDATION, 0.5);
                telemetry.addData("SERVO: ", "UP");
            } else if (controller2.rightBumper()) {
                setAngle(ServoName.FOUNDATION, 0.8);
                telemetry.addData("SERVO: ", "DOWN");
            }

            // Lift Control
            if (controller1.dpadUp()) {
                setPower(MotorName.LEFT_LIFT_WINCH, lifterSpeed);
            } else if (controller1.dpadDown()) {
                setPower(MotorName.LEFT_LIFT_WINCH, -lifterSpeed);
            } else {
                setPower(MotorName.LEFT_LIFT_WINCH, 0);
            }
        }
    }


    public boolean driveMotorToPos (RobotHardware.MotorName motorName, int targetTicks, double power, double rampThreshold) {
        power = Range.clip(Math.abs(power), 0, 1);
        int poweredDistance = 0;
        int arrivedDistance = 50;
        double maxRampPower = 1.0;
        double minRampPower = 0.0;
        int errorSignal = getEncoderValue(motorName) - targetTicks;
        double direction = errorSignal > 0 ? -1.0: 1.0;
        double rampDownRatio = AutoDrive.rampDown(Math.abs(errorSignal), rampThreshold, maxRampPower, minRampPower);

        if (Math.abs(errorSignal) >= poweredDistance) {
            setPower(motorName, direction * power * rampDownRatio);
        } else {
            setPower(motorName, 0);
        }

        return Math.abs(errorSignal) <= arrivedDistance;
    }


    public boolean driveMotorToPos (RobotHardware.MotorName motorName, int targetTicks, double power) {
        int rampDistanceTicks = 400;
        return driveMotorToPos (motorName,targetTicks,power,rampDistanceTicks);
    }
}
