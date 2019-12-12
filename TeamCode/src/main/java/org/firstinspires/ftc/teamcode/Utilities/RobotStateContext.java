package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoOpmode;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;

import static org.firstinspires.ftc.teamcode.Utilities.Executive.StateMachine.StateType.*;
import static org.firstinspires.ftc.teamcode.Utilities.Waypoints.LocationLoading.*;

public class RobotStateContext implements Executive.RobotStateMachineContextInterface {

    AutoOpmode opMode;
    Executive.StateMachine<AutoOpmode> stateMachine;
    Color.Ftc teamColor;
    private RobotHardware.StartPosition startPosition;
    private Waypoints waypoints;
    private double driveSpeed;
    private boolean simple;
    private boolean parkInner;
    private boolean dropStones;

    private double scanDelay = 0.5;
    private double liftSpeed = 1;
    private int liftRaised = 1500;

    private Controller controller1;


    public RobotStateContext(AutoOpmode opMode, Color.Ftc teamColor, AutoOpmode.StartPosition startPosition) {
        this.opMode = opMode;
        this.teamColor = teamColor;
        this.startPosition = startPosition;
        this.stateMachine = new Executive.StateMachine(opMode);
        this.waypoints = new Waypoints(teamColor);
        stateMachine.update();

        controller1 = opMode.controller1;
    }

    public void init() {
        stateMachine.changeState(DRIVE, new Start_State());

        stateMachine.init();

        driveSpeed = opMode.AutoDriveSpeed.get();
        simple = opMode.SimpleAuto.get();
        parkInner = opMode.ParkInner.get();
        dropStones = opMode.DropStones.get();
    }



    public void update() {
        stateMachine.update();

        opMode.updateMecanumHeadingFromGyroNow();
    }

    public String getCurrentState() {
        return stateMachine.getCurrentStates();
    }
    
    

    /**
     * Define Concrete State Classes
     */

    // Fork between Build and Load side starting positions.
    class Start_State extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            switch (startPosition) {
                case FIELD_LOADING:
                    setupInitialPosition(waypoints.loading.get(Waypoints.LocationLoading.INITIAL_POSITION));
                    stateMachine.changeState(DRIVE, new Start_Loading_Side());
                    break;
                case FIELD_BUILD:
                    setupInitialPosition(waypoints.building.get(Waypoints.LocationBuild.INITIAL_POSITION));
                    stateMachine.changeState(DRIVE, new Start_Building_Side());
                    break;
                default:
                   throw new IllegalStateException("Field Position must be either FIELD_LOADING or FIELD_BUILD");
            }
        }

        private void setupInitialPosition(Navigation2D initialPosition) {
            opMode.mecanumNavigation.setCurrentPosition(initialPosition);
            opMode.imuUtilities.updateNow();
            opMode.imuUtilities.setCompensatedHeading(radiansToDegrees(initialPosition.theta));
        }
    }

    /**
     * Loading Drive State
     * The initial start state
     */
    class Start_Loading_Side extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            if(stateTimer.seconds() > 1 && simple) stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Simple_Start());
            else if(stateTimer.time() > 1) stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Scan_Position_A_0());
        }
    }
    /**
     * Loading Drive State
     * The state for scanning the first 3 stones to identify the skystone
     */
    class Scan_Position_A_0 extends Executive.StateBase<AutoOpmode> {
        boolean timerReset = false;
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            stateMachine.changeState(ARM, new Raise_Open_Claw());
        }

        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(SCAN_POSITION_A_0), getDriveScale(stateTimer) * driveSpeed);
            if(arrived) {
                if (!timerReset) {
                    stateTimer.reset();
                    timerReset = true;
                }

                if(stateTimer.seconds() > scanDelay) {
                    if (opMode.simpleVision.isSkystoneVisible()) {
                        waypoints.setSkystoneDetectionPosition(2);
                        stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Align_Skystone_A());
                    }
                    stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Scan_Position_A_1());
                }
            }
        }
    }
    /**
     * Loading Drive State
     * The state for scanning the first 3 stones to identify the skystone
     */
    class Scan_Position_A_1 extends Executive.StateBase<AutoOpmode> {
        boolean timerReset = false;
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            stateMachine.changeState(ARM, new Raise_Open_Claw());
        }

        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(SCAN_POSITION_A_1), getDriveScale(stateTimer) * driveSpeed);
            if(arrived) {
                if (!timerReset) {
                    stateTimer.reset();
                    timerReset = true;
                }

                if(stateTimer.seconds() > scanDelay) {
                    if(opMode.simpleVision.isSkystoneVisible()) {
                        waypoints.setSkystoneDetectionPosition(1);
                    } else {
                        waypoints.setSkystoneDetectionPosition(0);
                    }
                    stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Align_Skystone_A());
                }
            }
        }
    }
    /**
     * Loading Drive State
     * The state for aligning in-front of the detected skystone
     */
    class Align_Skystone_A extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.updateMecanumHeadingFromGyroNow();
            stateMachine.changeState(ARM, new Lower_Open_Claw());
        }

        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(ALIGNMENT_POSITION_A), getDriveScale(stateTimer) * driveSpeed);
            if(arrived) {
                stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Grab_Skystone_A());
            }
        }
    }
    /**
     * Loading Drive State
     * The state for grabbing the detected skystone
     */
    class Grab_Skystone_A extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(GRAB_SKYSTONE_A), getDriveScale(stateTimer) * driveSpeed);
            if(arrived) {
                if(!stateMachine.getCurrentStates(ARM).equals("Lower_Close_Claw")) {
                    stateMachine.changeState(ARM, new Lower_Close_Claw());
                }
                if(stateMachine.getStateReference(ARM).arrived) {
                    stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Backup_Skystone_A());
                }
            }
        }
    }
    /**
     * Loading Drive State
     * The state for backing up from the detected skystone to avoid knocking other stones over
     */
    class Backup_Skystone_A extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            if (!stateMachine.getCurrentStates(ARM).equals("Raise_Close_Claw")) {
                stateMachine.changeState(ARM, new Raise_Close_Claw());
                stateTimer.reset();
            }
            if (stateTimer.seconds() > scanDelay) {
                if (stateMachine.getStateReference(ARM).arrived) {
                    arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(ALIGNMENT_POSITION_A), getDriveScale(stateTimer) * driveSpeed);
                    if (arrived) {
                        stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Build_Zone_A());
                    }
                }
            }
        }
    }
    /**
     * Loading Drive State
     * The state for driving to the build zone
     */
    class Build_Zone_A extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(BUILD_ZONE), getDriveScale(stateTimer) * driveSpeed);

            if(arrived) {
                if(dropStones) stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Drop_Skystone_A());
                else stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Align_Foundation_A());
            }
        }
    }

    class Drop_Skystone_A extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            stateMachine.changeState(ARM, new Drop_Off_Skystone_A());
        }

        @Override
        public void update() {
            super.update();
            if(stateMachine.getStateReference(ARM).arrived) {
                stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Align_Skystone_B());
            }
        }
    }

    class Align_Foundation_A extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(FOUNDATION_ALIGNMENT), getDriveScale(stateTimer) * driveSpeed);
            if(arrived) {
                stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Place_Foundation_A());
            }
        }
    }

    class Place_Foundation_A extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(FOUNDATION_DROP_OFF), getDriveScale(stateTimer) * driveSpeed);
            if(arrived) {
                if(!stateMachine.getCurrentStates(ARM).equals("Place_On_Foundation_A")) {
                    stateMachine.changeState(ARM, new Place_On_Foundation_A());
                }
                if(stateMachine.getStateReference(ARM).arrived) {
                    stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Backup_Foundation_A());
                }
            }
        }
    }

    class Backup_Foundation_A extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(FOUNDATION_ALIGNMENT), getDriveScale(stateTimer) * driveSpeed);
            if(arrived) {
                stateMachine.changeState(ARM, new Lower_Close_Claw());
                stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Align_Inner());
            }
        }
    }

    /**
     * Loading Drive State
     * The state for aligning in-front of the detected skystone
     */
    class Align_Skystone_B extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(ALIGNMENT_POSITION_B), getDriveScale(stateTimer) * driveSpeed);
            if(arrived) {
                stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Grab_Skystone_B());
            }
        }
    }
    /**
     * Loading Drive State
     * The state for grabbing the detected skystone
     */
    class Grab_Skystone_B extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(GRAB_SKYSTONE_B), getDriveScale(stateTimer) * driveSpeed);
            if(arrived) {
                if(!stateMachine.getCurrentStates(ARM).equals("Lower_Close_Claw")) {
                    stateMachine.changeState(ARM, new Lower_Close_Claw());
                }
                if(stateMachine.getStateReference(ARM).arrived) {
                    stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Backup_Skystone_B());
                }
            }
        }
    }
    /**
     * Loading Drive State
     * The state for backing up from the detected skystone to avoid knocking other stones over
     */
    class Backup_Skystone_B extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            if(!stateMachine.getCurrentStates(ARM).equals("Raise_Close_Claw")) {
                stateMachine.changeState(ARM, new Raise_Close_Claw());
            }
            if (stateTimer.seconds() > 0.25) {
                if (stateMachine.getStateReference(ARM).arrived) {
                    arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(ALIGNMENT_POSITION_B), getDriveScale(stateTimer) * driveSpeed);
                    if (arrived) {
                        stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Build_Zone_B());
                    }
                }
            }
        }
    }

    class Build_Zone_B extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(BUILD_ZONE), getDriveScale(stateTimer) * driveSpeed);

            if(arrived) {
                if(!dropStones) stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Align_Foundation_B());
                else stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Drop_Skystone_B());
            }
        }
    }

    class Drop_Skystone_B extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            stateMachine.changeState(ARM, new Drop_Off_Skystone_B());
        }

        @Override
        public void update() {
            super.update();

            if(stateMachine.getStateReference(ARM).arrived) {
                stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Align_Inner());
            }
        }
    }

    class Align_Foundation_B extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            if(!stateMachine.getCurrentStates(ARM).equals("Place_On_Foundation_B")) {
                stateMachine.changeState(ARM, new Place_On_Foundation_B());
            }
            if(stateMachine.getStateReference(ARM).arrived) {
                arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(FOUNDATION_ALIGNMENT), getDriveScale(stateTimer) * driveSpeed);
                if (arrived) {
                    stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Place_Foundation_B());
                }
            }
        }
    }

    class Place_Foundation_B extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            // Need to change
            arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(FOUNDATION_DROP_OFF), getDriveScale(stateTimer) * driveSpeed);
            if(arrived) {
                if(!stateMachine.getCurrentStates(ARM).equals("openClaw")) {
                    stateMachine.changeState(ARM, new openClaw());
                    stateTimer.reset();
                }
                if(stateTimer.seconds() > .5) {
                    stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Backup_Foundation_B());
                }
            }
        }
    }

    class Backup_Foundation_B extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.updateMecanumHeadingFromGyroNow();
        }

        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(FOUNDATION_ALIGNMENT), getDriveScale(stateTimer) * driveSpeed);
            if(arrived) {
                stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Align_Inner());
            }
        }
    }

    class Align_Outer extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.updateMecanumHeadingFromGyroNow();
        }

        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(BRIDGE_ALIGNMENT_OUTER), getDriveScale(stateTimer) * driveSpeed);
            if(arrived) {
                stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Park_Inner());
            }
        }
    }

    class Park_Outer extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.updateMecanumHeadingFromGyroNow();
            stateMachine.changeState(ARM, new Lower_Open_Claw());
        }

        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(PARK_OUTER), getDriveScale(stateTimer) * driveSpeed);
            if(arrived) {
                stateMachine.changeState(opMode.shouldContinue(), DRIVE, new A_Manual());
            }
        }
    }

    class Align_Inner extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(BRIDGE_ALIGNMENT_INNER), getDriveScale(stateTimer) * driveSpeed);
            if(arrived) {
                stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Park_Inner());
            }
        }
    }

    class Park_Inner extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            stateMachine.changeState(ARM, new verticalClaw());
        }

        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(PARK_INNER), getDriveScale(stateTimer) * driveSpeed);
            if(arrived) {
                stateMachine.changeState(opMode.shouldContinue(), DRIVE, new A_Manual());
            }
        }
    }

    class Raise_Open_Claw extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, liftRaised,liftSpeed);
            if(arrived) {
                opMode.openClaw();
            }
        }
    }

    class Raise_Close_Claw extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, liftRaised,liftSpeed);
            if(arrived) {
                opMode.closeClaw();
            }
        }
    }

    class Lower_Close_Claw extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, 0,liftSpeed);
            if(arrived) {
                opMode.closeClaw();
            }
        }
    }

    class Place_On_Foundation_A extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, opMode.liftArmTicksForLevelFoundationKnob(1, true, true),liftSpeed);
            if(arrived) {
                opMode.openClaw();
            }
        }
    }

    class Place_On_Foundation_B extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, opMode.liftArmTicksForLevelFoundationKnob(2, true, true),liftSpeed);
        }
    }

    class Drop_Off_Skystone_A extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, opMode.liftArmTicksForLevelFoundationKnob(1, true, true),liftSpeed);
            if(arrived) {
                opMode.openClaw();
            }
        }
    }

    class Drop_Off_Skystone_B extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, opMode.liftArmTicksForLevelFoundationKnob(2, false, false),liftSpeed);
            if(arrived) {
                opMode.openClaw();
            }
        }
    }

    class openClaw extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            opMode.openClaw();
        }
    }

    class verticalClaw extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            opMode.verticalClaw();
        }
    }

    class Lower_Open_Claw extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            opMode.openClaw();
            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, 0,liftSpeed);
        }
    }

    class Simple_Start extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            if(opMode.shouldContinue()) {
                stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Simple_Align());
            }
        }
    }

    class Simple_Align extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            if(parkInner) arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(SIMPLE_ALIGNMENT_INNER), getDriveScale(stateTimer) * driveSpeed);
            else arrived = true;
            if(arrived) {
                if(opMode.shouldContinue()) {
                    stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Simple_Park());
                }
            }
        }
    }

    class Simple_Park extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            if(parkInner) arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(PARK_INNER), getDriveScale(stateTimer) * driveSpeed);
            else arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(PARK_OUTER), getDriveScale(stateTimer) * driveSpeed);
            if(arrived) {
                if(opMode.shouldContinue()) {
                    stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Stop_State());
                }
            }
        }
    }

    class A_Manual extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.updateMecanumHeadingFromGyroNow();
            opMode.telemetry.clear();
            stateMachine.removeStateType(ARM);
        }

        @Override
        public void update() {
            super.update();

            opMode.setDriveForSimpleMecanum(controller1.left_stick_x * 0.2, controller1.left_stick_y * 0.2, controller1.right_stick_x * 0.2, controller1.right_stick_y * 0.2);
            opMode.setPower(RobotHardware.MotorName.LIFT_WINCH, controller1.right_stick_y);
            if(controller1.rightBumper()) {
                opMode.openClaw();
            } else if(controller1.leftBumper()) {
                opMode.closeClaw();
            }

            for (RobotHardware.MotorName m : RobotHardware.MotorName.values()) {
                opMode.telemetry.addData(m.name() + ": ", opMode.getEncoderValue(m));
            }
        }
    }

    class Stop_State extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            opMode.stopAllMotors();
            stateMachine.removeStateType(ARM);
        }
    }


    class Start_Building_Side extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            // TODO: Provide real building side routine. Move the foundation?
            nextState(DRIVE, new Simple_Park());
        }
    }

    public double getDriveScale(ElapsedTime stateTimer) {
        double driveScale;
        double speedDivider = 2.0; // No idea what a good name is
        driveScale = stateTimer.seconds() / speedDivider;
        driveScale = driveScale < 1.0 ? driveScale : 1.0;
        return driveScale;
    }


    private double degreesToRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    private double radiansToDegrees(double radians) {
        return radians * 180 / Math.PI;
    }
}
