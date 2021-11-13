/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package club.towr5291.opmodes;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import java.util.HashMap;

import club.towr5291.functions.Constants;
import club.towr5291.functions.FileLogger;
import club.towr5291.functions.ReadStepFileXML;
import club.towr5291.functions.TOWR5291PID;
import club.towr5291.functions.TOWR5291TextToSpeech;
import club.towr5291.functions.UltimateGoalOCV;
import club.towr5291.libraries.ImageCaptureOCV;
import club.towr5291.libraries.LibraryMotorType;
import club.towr5291.libraries.TOWRDashBoard;
import club.towr5291.libraries.robotConfig;
import club.towr5291.libraries.robotConfigSettings;
import club.towr5291.robotconfig.HardwareArmMotorsFreightFrenzy;
import club.towr5291.robotconfig.HardwareDriveMotors;

/*
TOWR 5291 Autonomous

*/

@Autonomous(name = "Auton Rustoni", group = "5291")
@Disabled
public class Autodrive_Rustoni extends OpModeMasterLinear {

    private OpMode onStop = this;
    private OpModeManagerImpl opModeManager;
    private String TeleOpMode = "Base Drive 2021";

    final int LABEL_WIDTH = 200;

    //The autonomous menu settings from the sharepreferences
    private SharedPreferences sharedPreferences;
    private robotConfig ourRobotConfig;

    private ElapsedTime runtime = new ElapsedTime();

    //set up the variables for file logger and what level of debug we will log info at
    public FileLogger fileLogger;
    private int debug = 3;

    private WebcamName robotWebcam;

    //vuforia localisation variables
    private OpenGLMatrix lastLocation = null;
    private double localisedRobotX;
    private double localisedRobotY;
    private double localisedRobotBearing;
    private boolean localiseRobotPos;
    private static final int TARGET_WIDTH = 254;
    private static final int TARGET_HEIGHT = 184;

    //define each state for the step.  Each step should go through some of the states below
    // set up the variables for the state engine
    private int mintCurrentStep = 1;                                            // Current Step in State Machine.
    private Constants.stepState mintCurrentStateStep;                           // Current State Machine State.
    private Constants.stepState mintCurrentStateDrive;                          // Current State of Drive.
    private Constants.stepState mintCurrentStateDriveHeading;                   // Current State of Drive Heading.
    private Constants.stepState mintCurrentStateTankTurn;                       // Current State of Tank Turn.
    private Constants.stepState mintCurrentStatePivotTurn;                      // Current State of Pivot Turn.
    private Constants.stepState mintCurrentStateRadiusTurn;                     // Current State of Radius Turn.
    private Constants.stepState mintCurStVuforiaMove5291;                       // Current State of Vuforia Move
    private Constants.stepState mintCurStVuforiaTurn5291;                       // Current State of Vuforia Turn
    private Constants.stepState mintCurrentStateGyroTurnEncoder5291;            // Current State of the Turn function that take the Gyro as an initial heading
    private Constants.stepState mintCurrentStateEyes5291;                       // Current State of the Eyelids
    private Constants.stepState mintCurrentStateTankTurnGyroHeading;            // Current State of Tank Turn using Gyro
    private Constants.stepState mintCurrentStateMecanumStrafe;                  // Current State of mecanum strafe
    private Constants.stepState mintCurrentStepDelay;                           // Current State of Delay (robot doing nothing)
    private Constants.stepState mintCurrentStateMoveLift;                       // Current State of the Move lift
    private Constants.stepState mintCurrentStateInTake;                         // Current State of the Move lift
    private Constants.stepState mintCurrentStateNextStone;                       // Current State of Finding Gold
    private Constants.stepState mintCurrentStateWyattsGyroDrive;                // Wyatt Gyro Function
    private Constants.stepState mintCurrentStateTapeMeasure;                    // Control Tape Measure
    private Constants.stepState mintCurrentStateFlywheel;                       // Control flywheel
    private Constants.stepState mintCurrentStateClawMovement;                           // Control Servo to open close claw
    private Constants.stepState getMintCurrentStateEjector;                     // Control ejector
    private Constants.stepState mintCurrentStateGrabBlock;                      // Control arm to grab block
    private Constants.stepState mintCurrentStepFindGoldSS;                      // test
    private Constants.stepState mintCurrentStateFlywheelPower;

    private boolean mboolFoundSkyStone = false;

    private double mdblTeamMarkerDrop = .8;
    private double mdblTeamMarkerHome = 0;

    private HashMap<String, Integer> mintActiveSteps = new HashMap<>();
    private HashMap<String, Integer> mintActiveStepsCopy = new HashMap<>();

    //motors
    // load all the robot configurations for this season
    private HardwareDriveMotors robotDrive          = new HardwareDriveMotors();   // Use 5291's hardware
    private HardwareArmMotorsFreightFrenzy robotArms     = new HardwareArmMotorsFreightFrenzy();   // Use 5291's hardware
    //private HardwareSensorsSkyStone sensors         = new HardwareSensorsSkyStone();

    PIDFCoefficients getMotorPIDFMotor1;
    PIDFCoefficients getMotorPIDFMotor2;
    PIDFCoefficients getMotorPIDFMotor3;
    PIDFCoefficients getMotorPIDFMotor4;
    PIDFCoefficients newMotorPIDFMotor1;
    PIDFCoefficients newMotorPIDFMotor2;
    PIDFCoefficients newMotorPIDFMotor3;
    PIDFCoefficients newMotorPIDFMotor4;
    //private HardwareSensorsRoverRuckus sensor       = new HardwareSensorsRoverRuckus();

    private boolean vuforiaWebcam = true;

    //variable for the state engine, declared here so they are accessible throughout the entire opmode with having to pass them through each function
    private double mdblStep;                                 //Step from the step file, probably not needed
    private double mdblStepTimeout;                          //Timeout value ofthe step, the step will abort if the timeout is reached
    private String mstrRobotCommand;                         //The command the robot will execute, such as move forward, turn right etc
    private double mdblStepDistance;                         //used when decoding the step, this will indicate how far the robot is to move in inches
    private double mdblStepSpeed;                            //When a move command is executed this is the speed the motors will run at
    private boolean mblnParallel;                            //used to determine if next step will run in parallel - at same time
    private boolean mblnRobotLastPos;                        //used to determine if next step will run from end of last step or from encoder position
    private double mdblRobotParm1;                           //First Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm2;                           //Second Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm3;                           //Third Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm4;                           //Fourth Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm5;                           //Fifth Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm6;                           //Sixth Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not

    private int mintStartPositionLeft1;                      //Left Motor 1  - start position of the robot in inches, starts from 0 to the end
    private int mintStartPositionLeft2;                      //Left Motor 2  - start position of the robot in inches, starts from 0 to the end
    private int mintStartPositionRight1;                     //Right Motor 1 - start position of the robot in inches, starts from 0 to the end
    private int mintStartPositionRight2;                     //Right Motor 2 - start position of the robot in inches, starts from 0 to the end
    private int mintStepLeftTarget1;                         //Left Motor 1   - encoder target position
    private int mintStepLeftTarget2;                         //Left Motor 2   - encoder target position
    private int mintStepRightTarget1;                        //Right Motor 1  - encoder target position
    private int mintStepRightTarget2;                        //Right Motor 2  - encoder target position
    private double mdblDistanceToMoveTilt1;                  //Tilt Motor 1 - encoder target position
    private double mdblDistanceToMoveTilt2;                  //Tilt Motor 2 - encoder target position
    private double mdblTargetPositionTop1;                   //Main Lift Motor 1 - target Position
    private double mdblTargetPositionTop2;                   //Main Lift Motor 1 - target Position
    private double dblStepSpeedTempLeft;
    private double dblStepSpeedTempRight;
    private double mdblStepTurnL;                            //used when decoding the step, this will indicate if the robot is turning left
    private double mdblStepTurnR;                            //used when decoding the step, this will indicate if the robot is turning right
    private double mdblRobotTurnAngle;                       //used to determine angle the robot will turn
    private int mintLastEncoderDestinationLeft1;             //used to store the encoder destination from current Step
    private int mintLastEncoderDestinationLeft2;             //used to store the encoder destination from current Step
    private int mintLastEncoderDestinationRight1;            //used to store the encoder destination from current Step
    private int mintLastEncoderDestinationRight2;            //used to store the encoder destination from current Step
    private int mintLastPositionLeft1_1;
    private int mintLastPositionLeft2_1;
    private int mintLastPositionLeft1_2;
    private int mintLastPositionLeft2_2;
    private int mintLastPositionRight1_1;
    private int mintLastPositionRight2_1;
    private int mintLastPositionRight1_2;
    private int mintLastPositionRight2_2;
    private boolean blnMotor1Stall1 = false;
    private boolean blnStallTimerStarted = false;
    private boolean blnMotorStall1 = false;
    private boolean blnMotorStall2 = false;
    private boolean blnMotorStall3 = false;
    private boolean blnMotorStall4 = false;
    private Boolean stones[] = {true,true,true,true,true,true};

    private boolean mblnNextStepLastPos;                     //used to detect using encoders or previous calc'd position
    private int mintStepDelay;                               //used when decoding the step, this will indicate how long the delay is on ms.
    private boolean mblnDisableVisionProcessing = false;     //used when moving to disable vision to allow faster speed reading encoders.
    private int mintStepRetries = 0;                         //used to count retries on a step
    private ElapsedTime mStateTime = new ElapsedTime();      // Time into current state, used for the timeout
    private ElapsedTime mStateStalTimee = new ElapsedTime(); // Time into current state, used for the timeout
    private int mintStepNumber;
    private boolean flipit = false;
    private int quadrant;
    private int imuStartCorrectionVar = 0;
    private int imuMountCorrectionVar = 90;
    private boolean blnCrossZeroPositive = false;
    private boolean blnCrossZeroNegative = false;
    private boolean blnReverseDir = false;
    /**
     * Variables for the lift and remembering the current position
     */
    private int mintCurrentLiftCountMotor1          = 0;
    private int mintCurrentLiftCountMotor2          = 0;
    private int mintLiftStartCountMotor1            = 0;
    private int mintLiftStartCountMotor2            = 0;

    //hashmap for the steps to be stored in.  A Hashmap is like a fancy array
    //private HashMap<String, LibraryStateSegAutoRoverRuckus> autonomousSteps = new HashMap<String, LibraryStateSegAutoRoverRuckus>();
    private HashMap<String, String> powerTable = new HashMap<String, String>();
    private ReadStepFileXML autonomousStepsFile = new ReadStepFileXML();

    private UltimateGoalOCV elementColour = new UltimateGoalOCV();

    private ImageCaptureOCV imageCaptureOCV = new ImageCaptureOCV();
    //private LibraryTensorFlowRoverRuckus tensorFlowRoverRuckus = new LibraryTensorFlowRoverRuckus();

    private int mintNumberColourTries = 0;
    private Constants.ObjectColours mColour;

    private Constants.ObjectColours numberOfRings;
    private Constants.ObjectColours mLocation;

    private TOWR5291TextToSpeech towr5291TextToSpeech = new TOWR5291TextToSpeech(false);

    private TOWR5291PID PID1 = new TOWR5291PID();
    private TOWR5291PID PIDLEFT1 = new TOWR5291PID();
    private TOWR5291PID PIDLEFT2 = new TOWR5291PID();
    private TOWR5291PID PIDRIGHT1 = new TOWR5291PID();
    private TOWR5291PID PIDRIGHT2 = new TOWR5291PID();
    private int intdirection;
    private double dblStartVoltage = 0;

    private static TOWRDashBoard dashboard = null;

    public static TOWRDashBoard getDashboard() {
        return dashboard;
    }

    //each robot speeds up and slows down at different rates
    //helps reduce over runs and
    //table for the tilerunner from AndyMark.  These values are for the twin 20 motors which makes the robot fast
    private void loadPowerTableTileRunner() {
        powerTable.put(String.valueOf(0.5), ".1");
        powerTable.put(String.valueOf(1), ".2");
        powerTable.put(String.valueOf(2), ".3");
        powerTable.put(String.valueOf(4), ".4");
        powerTable.put(String.valueOf(6), ".5");
        powerTable.put(String.valueOf(8), ".6");
        powerTable.put(String.valueOf(10), ".7");
        powerTable.put(String.valueOf(12), ".8");
    }

    //table for the custom tanktread robot.  These values are for the twin 40 motors
    private void loadPowerTableTankTread() {
        powerTable.put(String.valueOf(0.5), ".3");
        powerTable.put(String.valueOf(1), ".3");
        powerTable.put(String.valueOf(2), ".4");
        powerTable.put(String.valueOf(4), ".5");
        powerTable.put(String.valueOf(6), ".5");
        powerTable.put(String.valueOf(8), ".6");
        powerTable.put(String.valueOf(10), ".6");
        powerTable.put(String.valueOf(12), ".6");
        powerTable.put(String.valueOf(15), ".8");
    }

    //private boolean vuforiaWebcam = true;
    private double TrackWidth = 14.25;
    private int TrgtPos;
    private int FlywheelSpd = 1200;

    @Override
    public void runOpMode() throws InterruptedException {

        //FtcRobotControllerActivity act = (FtcRobotControllerActivity) (hardwareMap.appContext);

//        ourRobotConfig = new robotConfig();
//        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
//
//        ourRobotConfig.setAllianceColor(sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red"));// Using a Function to Store The Robot Specification
//        ourRobotConfig.setTeamNumber(sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000"));
//        ourRobotConfig.setAllianceStartPosition(sharedPreferences.getString("club.towr5291.Autonomous.Position", "Left"));
//        ourRobotConfig.setDelay(Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0")));
//        ourRobotConfig.setRobotMotorType(sharedPreferences.getString("club.towr5291.Autonomous.RobotMotorType", "REV20ORBIT"));
//        ourRobotConfig.setRobotConfigBase(sharedPreferences.getString("club.towr5291.Autonomous.RobotConfigBase", "TileRunner2x40"));
//        debug = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1"));

        //now we have loaded the config from sharedpreferences we can setup the robot
        //ourRobotConfig.initConfig();
        robotDrive.allMotorsStop();
        robotDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Grip block
        robotArms.ClawServo.setPosition(0.0);

        // Try Moving
        Move(1.0, 1000, 0, 0); // Move Forward
        Move(0.5, 1000, 0, -1); // Move Forward
        Move(0.5, 1000, 0, 1); // Move Forward
        StopDrive(); // Stop Drive Motors
        Move(0.5, 1000, 12, 1);  //Turn Left
        Move(1.0, 1000, 12, -1); // Turn Right
        StopDrive();

        // Spin the carosel
        robotArms.flywheelMotor.setVelocity(1200);

        // Find the floor and reset encoder
        robotArms.liftMotor1.setPower(0.5);
        TrgtPos = robotArms.liftMotor1.getCurrentPosition() + 1000;
        robotArms.liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);
        robotArms.liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(2000); // Wait for claw to rest on floor
        robotArms.liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);
        robotArms.liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TrgtPos = 1000; // Set this to desired level;

        robotArms.flywheelMotor.setVelocity(0);

    } // end of runOpMode


    public void Move(double PwrTotal, int StepTime, double TurnRadius, int TurnDir) {

        double mtr1_pwrcmd; // Left Front Motor
        double mtr2_pwrcmd; // Left Rear Motor
        double mtr3_pwrcmd; // Right Front Motor
        double mtr4_pwrcmd; // Right Rear Motor
        double Pwr_Max, Pwr_Outside, Pwr_Inside;

        // *** Enhanced Proportional Motor Command   Calculation            ***
        // *** Vector Clip max motor power req                              ***
        if (TurnDir == 0.0) { // straight drive
            Pwr_Outside = PwrTotal;
            Pwr_Inside = PwrTotal;
        } else if (Math.abs(TurnRadius) > TrackWidth) { // Large Radius Turn
            Pwr_Outside = PwrTotal * (TurnRadius - 0.5 * TrackWidth) / TurnRadius;
            Pwr_Inside = PwrTotal * (TurnRadius + 0.5 * TrackWidth) / TurnRadius;
        } else { // Tight Radius Turn
            Pwr_Outside = PwrTotal;
            Pwr_Inside = -1 * (TurnRadius / TrackWidth + PwrTotal);
        }
        Pwr_Max = Math.max(1, Math.max(Pwr_Outside, Pwr_Inside));

        if (Math.signum(TurnDir) > 0) { // LH turn
            mtr1_pwrcmd = Range.clip(Pwr_Outside / Pwr_Max, -1, 1);
            mtr2_pwrcmd = Range.clip(Pwr_Outside / Pwr_Max, -1, 1);
            mtr3_pwrcmd = Range.clip(Pwr_Inside / Pwr_Max, -1, 1);
            mtr4_pwrcmd = Range.clip(Pwr_Inside / Pwr_Max, -1, 1);
        } else if (Math.signum(TurnDir) < 0) { // RH turn
            mtr1_pwrcmd = Range.clip(Pwr_Inside / Pwr_Max, -1, 1);
            mtr2_pwrcmd = Range.clip(Pwr_Inside / Pwr_Max, -1, 1);
            mtr3_pwrcmd = Range.clip(Pwr_Outside / Pwr_Max, -1, 1);
            mtr4_pwrcmd = Range.clip(Pwr_Outside / Pwr_Max, -1, 1);
        } else {
            mtr1_pwrcmd = Range.clip(Pwr_Outside / Pwr_Max, -1, 1);
            mtr2_pwrcmd = Range.clip(Pwr_Outside / Pwr_Max, -1, 1);
            mtr3_pwrcmd = Range.clip(Pwr_Outside / Pwr_Max, -1, 1);
            mtr4_pwrcmd = Range.clip(Pwr_Outside / Pwr_Max, -1, 1);
        }

        robotDrive.baseMotor1.setPower(mtr1_pwrcmd);
        robotDrive.baseMotor2.setPower(mtr2_pwrcmd);
        robotDrive.baseMotor3.setPower(mtr3_pwrcmd);
        robotDrive.baseMotor4.setPower(mtr4_pwrcmd);

        sleep(StepTime);

    }

    public void StopDrive() {
        robotDrive.baseMotor1.setPower(0);
        robotDrive.baseMotor2.setPower(0);
        robotDrive.baseMotor3.setPower(0);
        robotDrive.baseMotor4.setPower(0);
    }

}


