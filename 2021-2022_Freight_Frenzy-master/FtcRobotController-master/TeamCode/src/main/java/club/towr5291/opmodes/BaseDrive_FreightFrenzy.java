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
    private boolean hold = false;
    private boolean ClawIsOpen = false;
    private boolean FlywheelOn = false;

    /* Hardware Set Up */
    private HardwareDriveMotors robotDrive = new HardwareDriveMotors();
    private HardwareArmMotorsFreightFrenzy robotArms = new HardwareArmMotorsFreightFrenzy();

    //Settings from the sharepreferences
    private SharedPreferences sharedPreferences;

    private FileLogger fileLogger;
    final String TAG = "TeleOp";
    private ElapsedTime runtime = new ElapsedTime();
    private robotConfig ourRobotConfig;

    private int debug;
    public int originalliftposition;
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

        robotArms.liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //new stuff added for liftmotor
        robotArms.liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robotArms.flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotArms.flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        originalliftposition = robotArms.liftMotor1.getCurrentPosition();

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

        while (opModeIsActive()) {
            fileLogger.writeEvent(1, "In Main Loop");
            dashboard.displayPrintf(5, "Controller Mode -- ", "Mecanum Drive Relic Recovery (BAD)");
            fileLogger.writeEvent(debug, "Controller Mode", "Mecanum Drive Relic Recovery");

            liftMotorPower();           // *** Call liftMotorPower and execute arm movement         ***
            DriveMotorControl();        // *** Call DriveMotorControl and execute movement          ***

            // ********************************************************
            // ***                     gripper claw                 ***
            // ********************************************************
            if (gamepad2.left_bumper) {
                robotArms.ClawServo.setPosition(0); // *** Open Claw ***
                ClawIsOpen = true;
            } else if (gamepad2.right_bumper) {
                robotArms.ClawServo.setPosition(1); // *** Close Claw ***
                ClawIsOpen = false;
            } // if gripper claw

            telemetry.addLine("Claw is Open: " + String.valueOf(ClawIsOpen));
            telemetry.update();

            // ********************************************************
            // ***               Carousel Spinner                   ***
            // ********************************************************

            if (gamepad2.left_trigger > 0.1) {
                robotArms.flywheelMotor.setVelocity(1600 * gamepad2.left_trigger);
                FlywheelOn = true;
            } else {
                robotArms.flywheelMotor.setVelocity(0);
                FlywheelOn = false;
            } // if Carousel Spinner

            telemetry.addLine("Flywheel Speed: " + String.valueOf(robotArms.flywheelMotor.getVelocity()));
            //telemetry.addLine("Flywheel On: " + String.valueOf(FlywheelOn));
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

    public void DriveMotorControl(){

        // ********************************************************
        // ***                  Drive Control                   ***
        // ********************************************************

        double mtr_pwrcurvecoeff = 2.0;  // ***   Adjust this for motor control shaping ***
        double mtr_pwrmax = 0;
        double mtr1_pwrreq = 0;
        double mtr1_pwrcmd = 0;

        double mtr2_pwrreq = 0;
        double mtr2_pwrcmd = 0;

        double mtr3_pwrreq = 0;
        double mtr3_pwrcmd = 0;
        
        double mtr4_pwrreq = 0;
        double mtr4_pwrcmd = 0;

        // *** Enhanced Proportional Motor Command   Calculation            ***
        // *** Clip max motor power req geometricaly                        ***
        mtr1_pwrreq = 0 - gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
        mtr2_pwrreq = 0 - gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
        mtr3_pwrreq = 0 - gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
        mtr4_pwrreq = 0 - gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
        mtr_pwrmax = Math.max(1,Math.max(Math.max(mtr1_pwrreq,mtr2_pwrreq),Math.max(mtr3_pwrreq,mtr4_pwrreq)));

        // *** Standard Motor Command Calculation                           ***
        // mtr_pwrmax = 1;

        mtr1_pwrcmd = Math.pow(Math.abs(Range.clip(mtr1_pwrreq/mtr_pwrmax, -1, 1)),mtr_pwrcurvecoeff) * Math.signum(mtr1_pwrreq);
        mtr2_pwrcmd = Math.pow(Math.abs(Range.clip(mtr2_pwrreq/mtr_pwrmax, -1, 1)),mtr_pwrcurvecoeff) * Math.signum(mtr2_pwrreq);
        mtr3_pwrcmd = Math.pow(Math.abs(Range.clip(mtr3_pwrreq/mtr_pwrmax, -1, 1)),mtr_pwrcurvecoeff) * Math.signum(mtr3_pwrreq);
        mtr4_pwrcmd = Math.pow(Math.abs(Range.clip(mtr4_pwrreq/mtr_pwrmax, -1, 1)),mtr_pwrcurvecoeff) * Math.signum(mtr4_pwrreq);

        robotDrive.baseMotor1.setPower(mtr1_pwrcmd);
        robotDrive.baseMotor2.setPower(mtr2_pwrcmd);
        robotDrive.baseMotor3.setPower(mtr3_pwrcmd);
        robotDrive.baseMotor4.setPower(mtr4_pwrcmd);

            /*robotDrive.baseMotor1.setPower(Math.pow((Range.clip((0 - gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)/mtr_pwrmax, -1, 1)),3));
            robotDrive.baseMotor2.setPower(Math.pow((Range.clip((0 - gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x)/mtr_pwrmax, -1, 1)),3));
            robotDrive.baseMotor3.setPower(Math.pow((Range.clip((0 - gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)/mtr_pwrmax, -1, 1)),3));
            robotDrive.baseMotor4.setPower(Math.pow((Range.clip((0 - gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x)/mtr_pwrmax, -1, 1)),3));*/

    } // DriveMotorControl

    public void liftMotorPower() {

        // ********************************************************************************************
        // ***                      Robot Arm                                                       ***
        // ********************************************************************************************
        if (gamepad2.x||gamepad1.x) {             // ***              Move ARM DOWN                 ***
            robotArms.liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robotArms.liftMotor1.setPower(-0.5);
            hold = true;
        } else if (gamepad2.y||gamepad1.y) {      // *** Move ARM Up                                ***
            robotArms.liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robotArms.liftMotor1.setPower(0.75);
            hold = true;
/*        } else if ((gamepad1.x)) {      // ***  Move arm down to learn zero with drive controller ***
            robotArms.liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robotArms.liftMotor1.setPower(-0.5);
            hold = false;*/
        } else if ((gamepad2.back)) {      // *** Learn zero position with drive controller         ***
            robotArms.liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            originalliftposition = robotArms.liftMotor1.getCurrentPosition();
            hold = false;
        } else if (gamepad2.dpad_down){     // *** move to zero                                     ***
            robotArms.liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotArms.liftMotor1.setTargetPosition(0);
            robotArms.liftMotor1.setPower(+0.75);
            hold = false;
        } else if (gamepad2.dpad_left){ // *** move to first tray height                            ***
            robotArms.liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotArms.liftMotor1.setTargetPosition(250);
            robotArms.liftMotor1.setPower(+0.75);
            hold = true;
        } else if (gamepad2.dpad_right){ // *** move to second tray height                          ***
            robotArms.liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotArms.liftMotor1.setTargetPosition(500);
            robotArms.liftMotor1.setPower(+0.75);
            hold = false;
        } else if (gamepad2.dpad_up){ // *** move to third tray height                              ***
            robotArms.liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotArms.liftMotor1.setTargetPosition(1000);
            robotArms.liftMotor1.setPower(+0.75);
            hold = false;
        } else {
            robotArms.liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (hold=true) {
                robotArms.liftMotor1.setPower(0);
            } // if
        } // if

        telemetry.addLine("Lift Motor " + robotArms.liftMotor1.getCurrentPosition());
        telemetry.update();

    } // liftMotorPower
} // public class
