package club.towr5291.Concepts;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

import club.towr5291.functions.FileLogger;
import club.towr5291.functions.ReadStepFileRoverRuckus;
import club.towr5291.functions.ReadStepFileXML;
import club.towr5291.libraries.LibraryStateSegAutoRoverRuckus;
import club.towr5291.libraries.robotConfig;


/**
 * Created by Ian Haden on 19/10/2019
 * TOWR 5291 ReadStepsFile
 Copyright (c) 2019 TOWR5291
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 2019-10-19 - Ian Haden - Initial creation
 ***/
@TeleOp(name = "Concept Read Steps SkyStone", group = "5291Concept")
@Disabled
public class ConceptReadStepFileSkyStone extends LinearOpMode {

    private String teamNumber;
    private String allianceParkPosition;
    private String allianceStartPosition;
    private int delay;
    private String numBeacons;
    private String robotConfig;
    private String allianceColor;
    private int debug = 3;

    //set up the variables for the logger
    final String TAG = "Concept Read File";
    private ElapsedTime runtime = new ElapsedTime();
    private FileLogger fileLogger;

    private ReadStepFileXML autonomousStepsTest = new ReadStepFileXML();

    private HashMap<String,LibraryStateSegAutoRoverRuckus> autonomousStepsMap = new HashMap<String,LibraryStateSegAutoRoverRuckus>();
    private club.towr5291.libraries.robotConfig robotconfig = new robotConfig();

    @Override
    public void runOpMode() throws InterruptedException {

        LibraryStateSegAutoRoverRuckus mStateSegAuto;

        if (debug >= 1) {
            fileLogger = new FileLogger(runtime);
            fileLogger.open();
            fileLogger.write("Time,SysMS,Thread,Event,Desc");
            fileLogger.writeEvent(TAG, "Log Started");
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        robotconfig.setTeamNumber("5291");
        robotconfig.setAllianceColor("Blue");
        robotconfig.setAllianceStartPosition("Left");
        Log.d("Steplist", "Number of loaded steps " + autonomousStepsTest.getNumberLoadedSteps());
        fileLogger.writeEvent(debug,"Status", "Reading Steps : ");

        autonomousStepsMap = autonomousStepsTest.ReadStepFile(robotconfig);

        Log.d("Steplist", "Number of loaded steps " + autonomousStepsTest.getNumberLoadedSteps());
        for (int loop = 1; loop <= autonomousStepsTest.getNumberLoadedSteps(); loop++) {
            mStateSegAuto = autonomousStepsMap.get(String.valueOf(loop));
            Log.d("Steplist", "Reading  " + loop + " timeout " + mStateSegAuto.getmRobotTimeOut() + " command " + mStateSegAuto.getmRobotCommand());
        }

        autonomousStepsTest.insertSteps(5, "Delay",  0,    0, false, false,   0,    0,    0,    0,  0, 0,1);

        //autonomousStepsTest.insertSteps(2, "IAN1", false, false, 0, 0, 0, 0, 0, 0, 0, 6);

        //autonomousStepsTest.insertSteps(2, "IAN2", false, false, 0, 0, 0, 0, 0, 0, 0, 8);

        //autonomousStepsTest.insertSteps(2, "IAN3", false, false, 0, 0, 0, 0, 0, 0, 0, 10);

        for (int loop = 1; loop <= autonomousStepsTest.getNumberLoadedSteps(); loop++)
        {
            mStateSegAuto = autonomousStepsMap.get(String.valueOf(loop));
            Log.d("Steplist", "Reading  " + loop + " timeout " + mStateSegAuto.getmRobotTimeOut() + " command " + mStateSegAuto.getmRobotCommand());
        }

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            idle();
        }

        if (debug >= 1) {
            if (fileLogger != null) {
                fileLogger.writeEvent("Status", "Run Time: " + runtime.toString());
                Log.d("Status", "Run Time: " + runtime.toString());
                fileLogger.writeEvent(TAG, "Stopped");
                fileLogger.close();
                fileLogger = null;
            }
        }
    }




}


