package club.towr5291.opmodes;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.R;

import club.towr5291.functions.Constants;
import club.towr5291.functions.FileLogger;
import club.towr5291.functions.TOWR5291Tick;
import club.towr5291.functions.TOWR5291Toggle;
import club.towr5291.libraries.LibraryMotorType;
import club.towr5291.libraries.TOWRDashBoard;
import club.towr5291.libraries.robotConfig;
import club.towr5291.libraries.robotConfigSettings;
import club.towr5291.robotconfig.HardwareArmMotorsFreightFrenzy;
import club.towr5291.robotconfig.HardwareArmMotorsSkyStone;
import club.towr5291.robotconfig.HardwareArmMotorsUltimateGoal;
import club.towr5291.robotconfig.HardwareDriveMotors;
// ********************************************************
// *** Created by AKR 10/09/2021                        ***
// *** TOWR 5291 Freight Frenzy 2021                    ***
// ********************************************************

@TeleOp(name = "BaseDrive_FreightFrenzy", group = "5291")
//@Disabled
public class BaseDrive_FreightFrenzy extends OpModeMasterLinear{
    private Constants.stepState stepState = Constants.stepState.STATE_COMPLETE;

    /* Hardware Set Up */
    private HardwareDriveMotors robotDrive = new HardwareDriveMotors();
    private HardwareArmMotorsFreightFrenzy robotArms = new HardwareArmMotorsFreightFrenzy();

    //Settings from the sharepreferences
    private SharedPreferences sharedPreferences;

    // Settings for File Logging
    private FileLogger fileLogger;
    final String TAG = "TeleOp";
    private ElapsedTime runtime = new ElapsedTime();
    private robotConfig ourRobotConfig;

    private int debug;

    private static TOWRDashBoard dashboard = null;
    public static TOWRDashBoard getDashboard() {return dashboard;}




    @Override
    public void runOpMode() throws InterruptedException {

        dashboard = TOWRDashBoard.createInstance(telemetry);
        FtcRobotControllerActivity act = (FtcRobotControllerActivity) (hardwareMap.appContext);

        dashboard.setTextView((TextView) act.findViewById(R.id.textOpMode));
        dashboard.displayPrintf(0, "Starting Menu System");

        ourRobotConfig = new robotConfig();
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);

        ourRobotConfig.setAllianceColor(sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red"));// Using a Function to Store The Robot Specification
        ourRobotConfig.setTeamNumber(sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000"));
        ourRobotConfig.setAllianceStartPosition(sharedPreferences.getString("club.towr5291.Autonomous.Position", "Left"));
        ourRobotConfig.setDelay(Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0")));
        ourRobotConfig.setRobotMotorType(sharedPreferences.getString("club.towr5291.Autonomous.RobotMotorType", "REV20ORBIT"));
        ourRobotConfig.setRobotConfigBase(sharedPreferences.getString("club.towr5291.Autonomous.RobotConfigBase", "TileRunner2x40"));
        debug = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1"));

        //now we have loaded the config from sharedpreferences we can setup the robot
        ourRobotConfig.initConfig();
        dashboard.displayPrintf(0, "Robot Config Loaded");

        fileLogger = new FileLogger(runtime, Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1")), true);// initializing FileLogger
        fileLogger.open();// Opening FileLogger
        fileLogger.writeEvent(TAG, "Log Started");// First Line Add To Log

        // All The Specification of the robot and controller
        fileLogger.writeEvent(1, "Alliance Color", ourRobotConfig.getAllianceColor());
        fileLogger.writeEvent(1, "Alliance Start Position", ourRobotConfig.getAllianceStartPosition());
        fileLogger.writeEvent(1, "Delay", String.valueOf(ourRobotConfig.getDelay()));
        fileLogger.writeEvent(1, "Robot Base Config", ourRobotConfig.getRobotConfigBase());
        fileLogger.writeEvent(1, "Robot Motor Type", ourRobotConfig.getRobotMotorType());
        fileLogger.writeEvent(1, "Team Number", ourRobotConfig.getTeamNumber());

        robotDrive.init(fileLogger, hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(ourRobotConfig.getRobotConfigBase()), LibraryMotorType.MotorTypes.valueOf(ourRobotConfig.getRobotMotorType()));// Starting robot Hardware map
        dashboard.displayPrintf(0, "Robot Base Loaded");

        robotDrive.allMotorsStop();
        robotDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robotArms.init(hardwareMap, dashboard);

        fileLogger.writeEvent(1, "", "Wait For Start ");

        dashboard.displayPrintf(1, "Waiting for Start");

        // setup drive motors
        float LStickX_Ltd;
        float LStickX_RL;
        float LStickX_prev;
        float LStickX_inc = 0.075f;     //  Increase rate of LStickX
        float LStickX_dec = 0.1f;       //  Decrease rate of LStickX
        float LStickX_Coeff = 2.0f;     //  LStickX Shaping Coefficient

        float LStickY_Ltd;
        float LStickY_RL;
        float LStickY_prev;
        float LStickY_inc = 0.075f;     //  Increase rate of LStickY
        float LStickY_dec = 0.1f;       //  Decrease rate of LStickY
        float LStickY_Coeff = 2.0f;     //  LStickY Shaping Coefficient

        float RStickX_Ltd;
        float RStickX_RL;
        float RStickX_prev;
        float RStickX_inc = 0.1f;       //  Increase rate of RStickX
        float RStickX_dec = 0.1f;       //  Decrease rate of RStickX
        float RStickX_Coeff = 3.0f;     //  RStickX Shaping Coefficient

        // Setup Lift Motor
        robotArms.liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotArms.liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int TrgtPos;
        boolean hold;
        hold = false;
        TrgtPos = robotArms.liftMotor1.getCurrentPosition();
        int lvl0_height = 0;        // Ground height
        int lvl1_height = 200;      // Level 1 height
        int lvl2_height = 600;      // Level 2 height
        int lvl3_height = 1000;     // Level 3 height

        // Setup Flywheel Motor
        robotArms.flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        dashboard.clearDisplay();
        fileLogger.writeEvent("Starting Loop");

        dashboard.clearDisplay();
        dashboard.displayPrintf(3, "Controller A Options");
        dashboard.displayPrintf(4, "--------------------");
        dashboard.displayPrintf(8, "Controller B Options");
        dashboard.displayPrintf(9, "--------------------");

        // ********************************************************
        // *** The main loop.  This is where the action happens ***
        // ********************************************************
        telemetry.clearAll();

        boolean prev_LB2 = false;
        double TrgtPow = 0;
        LStickX_prev = 0;
        RStickX_prev = 0;
        LStickY_prev = 0;

        while (opModeIsActive()) {
            fileLogger.writeEvent(1, "In Main Loop");
            dashboard.displayPrintf(5, "Controller Mode -- ", "Mecanum Drive Freight Frenzy");
            fileLogger.writeEvent(debug, "Controller Mode", "Mecanum Drive Freight Frenzy");

            // Rate Limit Stick Measurements
            // this treats an movement away from 0 as an increase, and movement towards zero as a decrease
            if (gamepad1.left_stick_x >= 0) {
                if (LStickX_prev >= 0) { // both positive
                    if (gamepad1.left_stick_x >= LStickX_prev) {
                        LStickX_RL = Math.min(LStickX_prev + LStickX_inc, gamepad1.left_stick_x);
                    } else { // LStickX < LStickX_prev
                        LStickX_RL = Math.max(LStickX_prev - LStickX_dec, gamepad1.left_stick_x);
                    }
                } else { //  LStick > 0 && LStickX_prev < 0
                    LStickX_RL = Math.max(LStickX_prev + LStickX_dec, gamepad1.left_stick_x);
                }
            } else { // gamepad1.left_stick_x < 0
                if (LStickX_prev <= 0) { // both negative
                    if (gamepad1.left_stick_x >= LStickX_prev) {
                        LStickX_RL = Math.min(LStickX_prev + LStickX_dec, gamepad1.left_stick_x);
                    } else { // LStickX < LSTickX_prev
                        LStickX_RL = Math.max(LStickX_prev - LStickX_inc, gamepad1.left_stick_x);
                    }
                } else { // LStickX_prev >= 0
                    LStickX_RL = Math.max(LStickX_prev - LStickX_dec, gamepad1.left_stick_x);
                }
            }

            if (gamepad1.left_stick_y >= 0) {
                if (LStickY_prev >= 0) { // both positive
                    if (gamepad1.left_stick_y >= LStickY_prev) {
                        LStickY_RL = Math.min(LStickY_prev + LStickY_inc, gamepad1.left_stick_y);
                    } else { // LStickX < LStickX_prev
                        LStickY_RL = Math.max(LStickY_prev - LStickY_dec, gamepad1.left_stick_y);
                    }
                } else { //  LStick > 0 && LStickX_prev < 0
                    LStickY_RL = Math.max(LStickY_prev + LStickY_dec, gamepad1.left_stick_y);
                }
            } else { // gamepad1.left_stick_y < 0
                if (LStickY_prev <= 0) { // both negative
                    if (gamepad1.left_stick_y >= LStickY_prev) {
                        LStickY_RL = Math.min(LStickY_prev + LStickY_dec, gamepad1.left_stick_y);
                    } else { // LStickX < LSTickX_prev
                        LStickY_RL = Math.max(LStickY_prev - LStickY_inc, gamepad1.left_stick_y);
                    }
                } else { // LStickX_prev >= 0
                    LStickY_RL = Math.max(LStickY_prev - LStickY_dec, gamepad1.left_stick_y);
                }
            }

            if (gamepad1.right_stick_x >= 0) {
                if (RStickX_prev >= 0) { // both positive
                    if (gamepad1.right_stick_x >= RStickX_prev) {
                        RStickX_RL = Math.min(RStickX_prev + RStickX_inc, gamepad1.right_stick_x);
                    } else { // RStickX < RStickX_prev
                        RStickX_RL = Math.max(RStickX_prev - RStickX_dec, gamepad1.right_stick_x);
                    }
                } else { //  LStick > 0 && RStickX_prev < 0
                    RStickX_RL = Math.max(RStickX_prev + RStickX_dec, gamepad1.right_stick_x);
                }
            } else { // gamepad1.right_stick_x < 0
                if (RStickX_prev <= 0) { // both negative
                    if (gamepad1.right_stick_x >= RStickX_prev) {
                        RStickX_RL = Math.min(RStickX_prev + RStickX_dec, gamepad1.right_stick_x);
                    } else { // RStickX < RStickX_prev
                        RStickX_RL = Math.max(RStickX_prev - RStickX_inc, gamepad1.right_stick_x);
                    }
                } else { // RStickX_prev >= 0
                    RStickX_RL = Math.max(RStickX_prev - RStickX_dec, gamepad1.right_stick_x);
                }
            }

            // Shape Stick Measurments
            LStickX_Ltd = (float) (Math.pow(Math.abs(LStickX_RL),LStickX_Coeff) * Math.signum(LStickX_RL));
            LStickY_Ltd = (float) (Math.pow(Math.abs(LStickY_RL),LStickY_Coeff) * Math.signum(LStickY_RL));
            RStickX_Ltd = (float) (Math.pow(Math.abs(RStickX_RL),RStickX_Coeff) * Math.signum(RStickX_RL));

            // *** Call liftMotorPower and execute arm movement         ***
            DriveMotorControl(LStickX_Ltd,LStickY_Ltd, RStickX_Ltd );        // *** Call DriveMotorControl and execute movement          ***

            LStickX_prev = LStickX_RL;
            LStickY_prev = LStickY_RL;
            RStickX_prev = RStickX_RL;

            // ****************************************************************************************
            // ***                      Robot Arm                                                   ***
            // ****************************************************************************************
            if (TrgtPos < robotArms.liftMotor1.getCurrentPosition()) {
                TrgtPow = 0.33;   // ***  Down Power Level ***
            } else {
                TrgtPow = 0.66; // *** Up Power Level ***
            }

            if (gamepad2.x||gamepad1.x) {             // ***              Move ARM DOWN             ***
                robotArms.liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robotArms.liftMotor1.setPower(-TrgtPow);
                TrgtPos = robotArms.liftMotor1.getCurrentPosition();
                hold = true;
            } else if (gamepad2.y||gamepad1.y) {      // *** Move ARM Up                            ***
                robotArms.liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robotArms.liftMotor1.setPower(TrgtPow);
                TrgtPos = robotArms.liftMotor1.getCurrentPosition();
                hold = true;
            } else if ((gamepad2.back)) {      // *** Learn zero position with drive controller     ***
                TrgtPos = robotArms.liftMotor1.getCurrentPosition();
                robotArms.liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                TrgtPos = robotArms.liftMotor1.getCurrentPosition();
                hold = false;
            } else if (gamepad2.dpad_down){     // *** move to zero                                 ***
                robotArms.liftMotor1.setPower(TrgtPow);
                TrgtPos = lvl0_height;
                hold = true;
            } else if (gamepad2.dpad_left){ // *** move to first tray height                        ***
                robotArms.liftMotor1.setPower(TrgtPow);
                TrgtPos = lvl1_height;
                hold = true;
            } else if (gamepad2.dpad_right){ // *** move to second tray height                      ***
                robotArms.liftMotor1.setPower(TrgtPow);
                TrgtPos = lvl2_height;
                hold = true;
            } else if (gamepad2.dpad_up){ // *** move to third tray height                          ***
                robotArms.liftMotor1.setPower(TrgtPow);
                TrgtPos = lvl3_height;
                hold = true;
            } else {
                if (hold) {
                    robotArms.liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robotArms.liftMotor1.setTargetPosition(TrgtPos);
                    robotArms.liftMotor1.setPower(TrgtPow);
                } else {
                    robotArms.liftMotor1.setPower(0);
                }
            } // End of Robot Arm if statment

            // ****************************************************************************************
            // ***                     gripper claw                                                 ***
            // ****************************************************************************************

            // *** This logic looks for the button press event and acts on the press event ***
            boolean clawIsOpen = robotArms.ClawServo.getPosition() < 0.30;
            boolean curr_LB2 = gamepad2.left_bumper;
            if (curr_LB2 && (curr_LB2 != prev_LB2)){ // *** This logic looks for the button press event
                if (clawIsOpen) {
                    robotArms.ClawServo.setPosition(0.0);
                } else {
                    robotArms.ClawServo.setPosition(0.25); // *** Open Claw  ***
                } // ClawIsOpen
            } else if (gamepad2.right_bumper) {
                robotArms.ClawServo.setPosition(1); // *** Close Claw ***
            }

            prev_LB2 = curr_LB2;

            // ****************************************************************************************
            // ***               Carousel Spinner                                                   ***
            // ****************************************************************************************

            if (gamepad2.left_trigger > 0.1) {
                robotArms.flywheelMotor.setVelocity(-1600 * gamepad2.left_trigger);
            } else if  (gamepad2.right_trigger > 0.1) {
                robotArms.flywheelMotor.setVelocity(1600 * gamepad2.right_trigger);
            } else {
                robotArms.flywheelMotor.setVelocity(0);
            } // if Carousel Spinner

            // ****************************************************************************************
            // ***               Telemetry                                                          ***
            // ****************************************************************************************

            telemetry.clearAll();
            telemetry.addLine("Claw is Open: " + clawIsOpen + " @ " + robotArms.ClawServo.getPosition());
            telemetry.addLine("Flywheel Speed: " + robotArms.flywheelMotor.getVelocity());
            telemetry.addLine("Lift Motor Pos: " + robotArms.liftMotor1.getCurrentPosition());
            telemetry.addLine("          Trgt: " + TrgtPos);
            telemetry.addLine();
            telemetry.addLine("Rx:      " + gamepad1.right_stick_x);
            telemetry.addLine("Rx RL:   " + (double) ((int) (RStickX_RL*100+.5))/100f);
            telemetry.addLine("Rx Lim:  " + (double) ((int) (RStickX_Ltd*100+.5))/100f);

            telemetry.update();

        }// while (opModeIsActive)

        // *** stop the logging ***
        if (fileLogger != null) {
            fileLogger.writeEvent(1, "TeleOP FINISHED - FINISHED");
            fileLogger.writeEvent(1, "Stopped");
            fileLogger.close();
            fileLogger = null;
        } // if fileLogger
    } // runOpMode

    public void DriveMotorControl(float LStickX_Lim, float LStickY_Lim, float RStickX_Lim){

        // ********************************************************
        // ***                  Drive Control                   ***
        // ********************************************************

        double mtr_pwrmax;
        double mtr1_pwrreq;
        double mtr1_pwrcmd;
        double mtr2_pwrreq;
        double mtr2_pwrcmd;
        double mtr3_pwrreq;
        double mtr3_pwrcmd;
        double mtr4_pwrreq;
        double mtr4_pwrcmd;

        // *** Enhanced Proportional Motor Command   Calculation            ***
        // *** Clip max motor power req geometricaly                        ***
        mtr1_pwrreq = 0 - LStickY_Lim + LStickX_Lim + RStickX_Lim;
        mtr2_pwrreq = 0 - LStickY_Lim - LStickX_Lim + RStickX_Lim;
        mtr3_pwrreq = 0 - LStickY_Lim - LStickX_Lim - RStickX_Lim;
        mtr4_pwrreq = 0 - LStickY_Lim + LStickX_Lim - RStickX_Lim;
        mtr_pwrmax = Math.max(1,Math.max(Math.max(mtr1_pwrreq,mtr2_pwrreq),Math.max(mtr3_pwrreq,mtr4_pwrreq)));

        // *** Standard Motor Command Calculation                           ***
        mtr1_pwrcmd = Range.clip(mtr1_pwrreq/mtr_pwrmax, -1, 1);
        mtr2_pwrcmd = Range.clip(mtr2_pwrreq/mtr_pwrmax, -1, 1);
        mtr3_pwrcmd = Range.clip(mtr3_pwrreq/mtr_pwrmax, -1, 1);
        mtr4_pwrcmd = Range.clip(mtr4_pwrreq/mtr_pwrmax, -1, 1);

        robotDrive.baseMotor1.setPower(mtr1_pwrcmd);
        robotDrive.baseMotor2.setPower(mtr2_pwrcmd);
        robotDrive.baseMotor3.setPower(mtr3_pwrcmd);
        robotDrive.baseMotor4.setPower(mtr4_pwrcmd);

            /*robotDrive.baseMotor1.setPower(Math.pow((Range.clip((0 - gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)/mtr_pwrmax, -1, 1)),3));
            robotDrive.baseMotor2.setPower(Math.pow((Range.clip((0 - gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x)/mtr_pwrmax, -1, 1)),3));
            robotDrive.baseMotor3.setPower(Math.pow((Range.clip((0 - gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)/mtr_pwrmax, -1, 1)),3));
            robotDrive.baseMotor4.setPower(Math.pow((Range.clip((0 - gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x)/mtr_pwrmax, -1, 1)),3));*/

    } // DriveMotorControl

} // public class
