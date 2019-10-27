package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.AutoOpmode;
import org.firstinspires.ftc.teamcode.RobotHardware;


import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.Utilities.Executive.StateMachine.StateType.DRIVE;
import static org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;

public class BehaviorSandbox implements Executive.RobotStateMachineContextInterface {

    AutoOpmode opMode;
    Executive.StateMachine stateMachine;
    Color.Ftc teamColor;
    RobotHardware.StartPosition startPosition;
    Waypoints waypoints;
    double driveSpeed = 0.8;


    public BehaviorSandbox(AutoOpmode opMode, Color.Ftc teamColor, RobotHardware.StartPosition startPosition) {
        this.opMode = opMode;
        this.teamColor = teamColor;
        this.startPosition = startPosition;
        stateMachine = new Executive.StateMachine(opMode);
        waypoints = new Waypoints(teamColor, startPosition, true);
    }

    public void init() {
        stateMachine.changeState(DRIVE, new MenuState());
        stateMachine.init();
        driveSpeed = opMode.AutoDriveSpeed.get();
    }

    public void update() {
        stateMachine.update();
        stateMachine.getCurrentState();
    }

    public String getCurrentState() {
        return stateMachine.getCurrentState();
    }

    /**
     * Define Concrete State Classes
     */

    class StartState extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            opMode.mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0, 0, 0));
            opMode.imuUtilities.updateNow();
            stateMachine.changeState(DRIVE, new MenuState());
        }
    }

    class MenuState extends Executive.StateBase {
        @Override
        public void update() {
            super.update();

            opMode.telemetry.addData("A:","Manual Control");
            if (opMode.controller1.AOnce()) {
                stateMachine.changeState(DRIVE, new ManualControl());
            }
            opMode.telemetry.addData("B:","Stop Routine");
            if (opMode.controller1.BOnce()) {
                stateMachine.changeState(DRIVE, new Stop_State());
            }

            opMode.telemetry.addData("X:","BluePickup Drivethrough");
            if (opMode.controller1.XOnce()) {
                stateMachine.changeState(DRIVE, new BB_DT_StartToSkystone_1());
            }

        }
    }

    class ManualControl extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            double exponential = 3;
            double driveSpeed = 0.5;
            opMode.setDriveForSimpleMecanum(Math.pow(opMode.controller1.left_stick_x, exponential) * driveSpeed,
                    Math.pow(opMode.controller1.left_stick_y, exponential) * driveSpeed,
                    Math.pow(opMode.controller1.right_stick_x, exponential) * driveSpeed,
                    Math.pow(opMode.controller1.right_stick_y, exponential) * driveSpeed);

            if(opMode.controller1.A()) {
                arrived = opMode.autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(0,0,0), driveSpeed);
            }

            if(opMode.controller1.Y()) { // Reset navigation position
                opMode.mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0,0,0));
            }

            // Next State Logic
            if (opMode.controller1.startOnce()) {
                stateMachine.changeState(DRIVE, new MenuState());
            }
        }
    }

    /**
     * Blue team, Build side, Drive through
     */

    // Blue Build Drive Through, from Start to Skystone 1
    class BB_DT_StartToSkystone_1 extends Executive.StateBase {
        @Override
        public void init(Executive.StateMachine stateMachine) {
            super.init(stateMachine);
            Navigation2D startWaypoint = new Navigation2D(-24,65.5,opMode.degreesToRadians(-90));
            opMode.mecanumNavigation.setCurrentPosition(new Navigation2D(startWaypoint));
            opMode.imuUtilities.setCompensatedHeading(opMode.radiansToDegrees(startWaypoint.theta));
        }

        @Override
        public void update() {
            super.update();
            if (stateTimer.seconds() > 1) {
                Navigation2D skystone1 = new Navigation2D(-36,48,opMode.degreesToRadians(-90));
                arrived = opMode.autoDrive.rotateThenDriveToPosition(skystone1, driveSpeed);
                if (arrived) {
                    stateMachine.changeState(DRIVE, new BB_DT_Skystone1_and_following());
                }
            }
        }
    }

    /**
     * Should drive through a list of waypoints, and should update mecanum navigation heading from
     * gyro upon each waypoint switch.
     */
    class BB_DT_Skystone1_and_following extends Executive.StateBase {

        ArrayList<Navigation2D> driveThroughList = new ArrayList<>();
        boolean pauseWaypointDrive = false;
        int lastWaypoint = -1;

        @Override
        public void init(Executive.StateMachine stateMachine) {
            super.init(stateMachine);
            driveThroughList.add(new MecanumNavigation.Navigation2D(-36,48,opMode.degreesToRadians(-90))); //SkystoneLook1
            driveThroughList.add(new MecanumNavigation.Navigation2D(-36,36,opMode.degreesToRadians(-90))); //SkystonePickup1
            driveThroughList.add(new MecanumNavigation.Navigation2D(-36,48,opMode.degreesToRadians(-90))); //SkystoneLook1
            driveThroughList.add(new MecanumNavigation.Navigation2D(-24,48,opMode.degreesToRadians(-90))); //Build Side
            driveThroughList.add(new MecanumNavigation.Navigation2D(-24,24,opMode.degreesToRadians(-90))); //Build Dropoff
            driveThroughList.add(new MecanumNavigation.Navigation2D(-24,48,opMode.degreesToRadians(-90))); //Build Side
            driveThroughList.add(new MecanumNavigation.Navigation2D(-60,48,opMode.degreesToRadians(-90))); //SkystoneLook2
            driveThroughList.add(new MecanumNavigation.Navigation2D(-60,36,opMode.degreesToRadians(-90))); //SkystonePickup2
            driveThroughList.add(new MecanumNavigation.Navigation2D(-60,48,opMode.degreesToRadians(-90))); //SkystoneLook2
            driveThroughList.add(new MecanumNavigation.Navigation2D(-24,48,opMode.degreesToRadians(-90))); //Build Side
            driveThroughList.add(new MecanumNavigation.Navigation2D(-24,36,opMode.degreesToRadians(-90))); //Build Dropoff2
        }

        @Override
        public void update() {
            super.update();
            if(opMode.controller1.startOnce()) {
                pauseWaypointDrive = !pauseWaypointDrive;
            }

            if(pauseWaypointDrive) {
                opMode.telemetry.addData("Paused","Multi Waypoint Drive");
                opMode.telemetry.addData("Start","To Resume");
                opMode.telemetry.addData("X:","To Return to Menu");
            } else {
                arrived = opMode.autoDrive.multiWaypointState("Skystone and following", driveSpeed, driveThroughList);
            }

            // Gyro update ONLY on waypoint switch
            if (opMode.autoDrive.currentDriveWaypoint > lastWaypoint) {
                lastWaypoint = opMode.autoDrive.currentDriveWaypoint;
                opMode.updateMecanumHeadingFromGyroNow();
            }

            // Next State Logic
            if (arrived) {
                stateMachine.changeState(DRIVE, new MenuState());
            }
            if (opMode.controller1.XOnce()) {
                stateMachine.changeState(DRIVE, new MenuState());
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


}