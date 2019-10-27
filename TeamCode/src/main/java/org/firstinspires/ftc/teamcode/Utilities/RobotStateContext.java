package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.AutoOpmode;
import org.firstinspires.ftc.teamcode.Manual;
import org.firstinspires.ftc.teamcode.RobotHardware;

import static org.firstinspires.ftc.teamcode.Utilities.Executive.StateMachine.StateType.DRIVE;

public class RobotStateContext implements Executive.RobotStateMachineContextInterface {

    AutoOpmode opMode;
    Executive.StateMachine stateMachine;
    Color.Ftc teamColor;
    RobotHardware.StartPosition startPosition;
    Waypoints waypoints;
    double driveSpeed = 0.8;


    public RobotStateContext(AutoOpmode opMode, Color.Ftc teamColor, RobotHardware.StartPosition startPosition) {
        this.opMode = opMode;
        this.teamColor = teamColor;
        this.startPosition = startPosition;
        stateMachine = new Executive.StateMachine(opMode);
        waypoints = new Waypoints(teamColor, startPosition, true);
    }

    public void init() {
        stateMachine.changeState(DRIVE, new Start_State());
//        stateMachine.changeState(Executive.StateMachine.StateType.ARM, new ArmLevelState());
        stateMachine.init();
        driveSpeed = opMode.AutoDriveSpeed.get();
    }

    public void update() {
        stateMachine.update();
    }

    public String getCurrentState() {
        return stateMachine.getCurrentState();
    }

    /**
     * Define Concrete State Classes
     */


    class Start_State extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            if (stateTimer.seconds() > 1) {
                opMode.mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0, 0, 0));
                opMode.imuUtilities.updateNow();
                stateMachine.changeState(DRIVE, new Drive_somewhere());
            }
        }
    }

    class Drive_somewhere extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            if (stateTimer.seconds() > 1) {
                arrived = opMode.autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(12, 12, opMode.degreesToRadians(180)), driveSpeed);
                if (arrived) {
                    stateMachine.changeState(DRIVE, new Manual_Control());
                }
            }
        }
    }

    class Stop_State extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            opMode.stopAllMotors();
        }
    }

    class Manual_Control extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            double exponential = 3;
            double driveSpeed = 0.5;
            opMode.setDriveForSimpleMecanum(Math.pow(opMode.controller1.left_stick_x, exponential) * driveSpeed,
                    Math.pow(opMode.controller1.left_stick_y, exponential) * driveSpeed,
                    Math.pow(opMode.controller1.right_stick_x, exponential) * driveSpeed,
                    Math.pow(opMode.controller1.right_stick_y, exponential) * driveSpeed);

            if (opMode.controller1.startOnce()) {
                stateMachine.changeState(DRIVE, new Start_State());
            } else if(opMode.controller1.B()) {
                stateMachine.changeState(DRIVE, new Stop_State());
            } else if(opMode.controller1.A()) {
                arrived = opMode.autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(0,0,0), driveSpeed);
            }
        }
    }


}