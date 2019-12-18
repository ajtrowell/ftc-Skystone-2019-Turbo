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
    private int blocksToDrop;

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
        blocksToDrop = (int) Math.round(opMode.BlocksToDrop.get());
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
            if(stateTimer.seconds() > .5 && simple) stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Simple_Start());
            else if(stateTimer.time() > .5) stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Scan_Skystones());
        }
    }
    /**
     * Loading Drive State
     * The state for scanning the first 3 stones to identify the skystone
     */
    class Scan_Skystones extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            stateMachine.changeState(ARM, new Raise_Open_Claw());
        }

        @Override
        public void update() {
            super.update();
            if (stateMachine.getStateReference(ARM).arrived) {
                if (opMode.skystoneDetector != null) {
                    waypoints.setSkystoneDetectionPosition(opMode.skystoneDetector.getSkystoneIndex());
                } else {
                    waypoints.setSkystoneDetectionPosition(0);
                }
                if (stateTimer.seconds() > scanDelay) {
                    if (blocksToDrop > 0) {
                        stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Align_Skystone_i(1));
                    } else {
                        if (parkInner) {
                            stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Align_Inner());
                        } else {
                            stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Align_Outer());
                        }
                    }
                }
            }
        }
    }
    /**
     * Loading Drive State
     * The state for aligning in-front of the detected skystone
     */
    class Align_Skystone_i extends Executive.StateBase<AutoOpmode> {
        public Align_Skystone_i(int iteration) {
            super(iteration);
        }

        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.updateMecanumHeadingFromGyroNow();
            stateMachine.changeState(ARM, new Lower_Open_Claw());
        }

        @Override
        public void update() {
            super.update();
            switch (getIteration()){
                case 1: arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(ALIGNMENT_POSITION_A), getDriveScale(stateTimer) * driveSpeed); break;
                case 2: arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(ALIGNMENT_POSITION_B), getDriveScale(stateTimer) * driveSpeed); break;
                case 3: arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(ALIGNMENT_POSITION_C), getDriveScale(stateTimer) * driveSpeed); break;
                default: throw new RuntimeException(this.getClass().toString() + ":  Invalid iteration.");
            }
            if(arrived) {
                stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Grab_Skystone_i(getIteration()));
            }
        }
    }
    /**
     * Loading Drive State
     * The state for grabbing the detected skystone
     */
    class Grab_Skystone_i extends Executive.StateBase<AutoOpmode> {
        public Grab_Skystone_i(int iteration) {
            super(iteration);
        }

        @Override
        public void update() {
            super.update();
            switch(getIteration()) {
                case 1: arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(GRAB_SKYSTONE_A), getDriveScale(stateTimer) * driveSpeed); break;
                case 2: arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(GRAB_SKYSTONE_B), getDriveScale(stateTimer) * driveSpeed); break;
                case 3: arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(GRAB_SKYSTONE_C), getDriveScale(stateTimer) * driveSpeed); break;
                default: throw new RuntimeException(this.getClass().toString() + ":  Invalid iteration.");
            }

            if(arrived) {
                if(!stateMachine.getCurrentStates(ARM).equals("Lower_Close_Claw")) {
                    stateMachine.changeState(ARM, new Lower_Close_Claw());
                }
                if(stateMachine.getStateReference(ARM).arrived) {
                    stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Backup_Skystone_i(getIteration()));
                }
            }
        }
    }
    /**
     * Loading Drive State
     * The state for backing up from the detected skystone to avoid knocking other stones over
     */
    class Backup_Skystone_i extends Executive.StateBase<AutoOpmode> {
        public Backup_Skystone_i(int iteration) {
            super(iteration);
        }

        @Override
        public void update() {
            super.update();
            if (!stateMachine.getCurrentStates(ARM).equals("Raise_Close_Claw")) {
                stateMachine.changeState(ARM, new Raise_Close_Claw());
                stateTimer.reset();
            }
            if (stateTimer.seconds() > scanDelay) {
                if (stateMachine.getStateReference(ARM).arrived) {
                    switch (getIteration()) {
                        case 1: arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(ALIGNMENT_POSITION_A), getDriveScale(stateTimer) * driveSpeed); break;
                        case 2: arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(ALIGNMENT_POSITION_B), getDriveScale(stateTimer) * driveSpeed); break;
                        case 3: arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(ALIGNMENT_POSITION_C), getDriveScale(stateTimer) * driveSpeed); break;
                        default: throw new RuntimeException(this.getClass().toString() + ":  Invalid iteration.");
                    }
                    if (arrived) {
                        if(dropStones) stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Skystone_Drop_Zone_i(getIteration()));
                        else stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Align_Foundation_i(getIteration()));
                    }
                }
            }
        }
    }
    /**
     * Loading Drive State
     * The state for driving to the build zone
     */
    class Skystone_Drop_Zone_i extends Executive.StateBase<AutoOpmode> {
        public Skystone_Drop_Zone_i(int iteration) {
            super(iteration);
        }

        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(SKYSTONE_DROP_ZONE), getDriveScale(stateTimer) * driveSpeed);

            if(arrived) {
                stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Drop_Skystone_i(getIteration()));
            }
        }
    }

    class Drop_Skystone_i extends Executive.StateBase<AutoOpmode> {
        public Drop_Skystone_i(int iteration) {
            super(iteration);
        }

        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            switch( getIteration()) {
                case 1: stateMachine.changeState(ARM, new Drop_Off_Skystone_A()); break;
                case 2:  // TODO: Adjust ARM behavior.
                case 3: stateMachine.changeState(ARM, new Drop_Off_Skystone_B()); break;
                default: throw new RuntimeException(this.getClass().toString() + ":  Invalid iteration.");
            }
        }

        @Override
        public void update() {
            super.update();
            if(stateMachine.getStateReference(ARM).arrived) {
                if(blocksToDrop > getIteration()) {
                    stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Align_Skystone_i(getIteration() + 1));
                } else {
                    if (parkInner) {
                        stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Align_Inner());
                    } else {
                        stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Align_Outer());
                    }
                }
            }
        }
    }

    class Align_Foundation_i extends Executive.StateBase<AutoOpmode> {
        public Align_Foundation_i(int iteration) {
            super(iteration);
        }

        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(FOUNDATION_ALIGNMENT), getDriveScale(stateTimer) * driveSpeed);
            if(arrived) {
                stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Place_Foundation_i(getIteration()));
            }
        }
    }

    class Place_Foundation_i extends Executive.StateBase<AutoOpmode> {
        public Place_Foundation_i(int iteration) {
            super(iteration);
        }
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(FOUNDATION_DROP_OFF), getDriveScale(stateTimer) * driveSpeed);
            if(arrived) {
                switch (getIteration()) {
                    case 1:
                        if(!stateMachine.getCurrentStates(ARM).equals("Place_On_Foundation_A")) {
                            stateMachine.changeState(ARM, new Place_On_Foundation_A());
                        }
                        break;
                    case 2:
                    case 3: // TODO: Adjust behavior
                        if(!stateMachine.getCurrentStates(ARM).equals("Place_On_Foundation_B")) {
                            stateMachine.changeState(ARM, new Place_On_Foundation_B());
                        }
                        break;
                    default: throw new RuntimeException(this.getClass().toString() + ":  Invalid iteration.");
                }
                if(stateMachine.getStateReference(ARM).arrived) {
                    stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Backup_Foundation_i(getIteration()));
                }
            }
        }
    }

    class Backup_Foundation_i extends Executive.StateBase<AutoOpmode> {
        public Backup_Foundation_i(int iteration) {
            super(iteration);
        }
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(FOUNDATION_ALIGNMENT), getDriveScale(stateTimer) * driveSpeed);
            if(arrived) {
                stateMachine.changeState(ARM, new Vertical_Claw());

                if(blocksToDrop > getIteration()) {
                    stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Align_Skystone_i(getIteration() + 1));
                } else {
                    if (parkInner) {
                        stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Align_Inner());
                    } else {
                        stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Align_Outer());
                    }
                }
            }
        }
    }




    class Align_Outer extends Executive.StateBase<AutoOpmode> {
        boolean isPrealignmentComplete = false; // Get on the same y and theta as the destination.
        Navigation2D preAlignmentWaypoint;
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            preAlignmentWaypoint = waypoints.loading.get(BRIDGE_ALIGNMENT_OUTER).copyAndLabel("PreAlign Inner");
            preAlignmentWaypoint.x = opMode.mecanumNavigation.currentPosition.x;
            stateMachine.changeState(ARM, new Vertical_Claw());
            opMode.updateMecanumHeadingFromGyroNow();
        }

        @Override
        public void update() {
            super.update();
            if(!isPrealignmentComplete) {
                isPrealignmentComplete = opMode.autoDrive.driveToPositionTranslateOnly(preAlignmentWaypoint, getDriveScale(stateTimer) * driveSpeed);
            } else {
                arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(BRIDGE_ALIGNMENT_OUTER), getDriveScale(stateTimer) * driveSpeed);
                if (arrived) {
                    stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Park_Outer());
                }
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
        boolean isPrealignmentComplete = false; // Get on the same y and theta as the destination.
        Navigation2D preAlignmentWaypoint;
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            preAlignmentWaypoint = waypoints.loading.get(BRIDGE_ALIGNMENT_INNER).copyAndLabel("PreAlign Inner");
            preAlignmentWaypoint.x = opMode.mecanumNavigation.currentPosition.x;
            stateMachine.changeState(ARM, new Vertical_Claw());
            opMode.updateMecanumHeadingFromGyroNow();
        }

        @Override
        public void update() {
            super.update();
            if(!isPrealignmentComplete) {
                isPrealignmentComplete = opMode.autoDrive.driveToPositionTranslateOnly(preAlignmentWaypoint, getDriveScale(stateTimer) * driveSpeed);
            } else {
                arrived = opMode.autoDrive.driveToPositionTranslateOnly(waypoints.loading.get(BRIDGE_ALIGNMENT_INNER), getDriveScale(stateTimer) * driveSpeed);
                if (arrived) {
                    stateMachine.changeState(opMode.shouldContinue(), DRIVE, new Park_Inner());
                }
            }
        }
    }

    class Park_Inner extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            stateMachine.changeState(ARM, new Vertical_Claw());
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

    class Vertical_Claw extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, 0, liftSpeed);
            if(arrived) {
                opMode.verticalClaw();
            }
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
