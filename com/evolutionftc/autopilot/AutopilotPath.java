package com.evolutionftc.autopilot;


import android.content.Context;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

// this is a snazzy autopilot for ftc made by aedan.
// it was made somewhat for team #9867, 'evolution'
// it uses lots of maths.
// copyright 2017 aedan cullen.


public class AutopilotPath {

    private Telemetry telemetry;
    private Context appContext;

    String pathName;

    private List<AutopilotSegment> pathSegments = new ArrayList<AutopilotSegment>();
    private String currentSegmentId = "NOTHINGNOTHINGNOTHING";

    private String successSegmentId = "NOTHINGNOTHINGNOTHING";
    private String failSegmentId = "NOTHINGNOTHINGNOTHING";

    public AutopilotPath(String pathName, Telemetry telemetry, Context appContext) {
        this.telemetry = telemetry;
        this.appContext = appContext;
        this.pathName = pathName;

        InputStream ins = appContext.getResources().openRawResource(
                appContext.getResources().getIdentifier(pathName, "raw", appContext.getPackageName()));

        try (BufferedReader pathReader = new BufferedReader(new InputStreamReader(ins))) {
            String header = pathReader.readLine();
            String line = pathReader.readLine();
			if (!header.toLowerCase().equals(
"id,success,fail,targetx,targety,targetz,targetorientation,steeringgain,accuracyx,accuracyy,accuracyz,accuracyorientation,basepower,lowestpower,powergain,rampup,rampdown,useorientation")){
				throw new UnsupportedOperationException("Header line in CSV indicates file unparseable, is it of the correct format?");
			}
            while (line != null) {
                String[] lineSegments = line.split(",");
                AutopilotSegment newSegment = new AutopilotSegment();
                newSegment.id = lineSegments[0];
                newSegment.success = lineSegments[1];
                newSegment.fail = lineSegments[2];
                newSegment.navigationTarget = new double[] {
                                Double.valueOf(lineSegments[3]),
                                Double.valueOf(lineSegments[4]),
                                Double.valueOf(lineSegments[5])
                        };
                newSegment.orientationTarget = Double.valueOf(lineSegments[6]);
                newSegment.steeringGain = Double.valueOf(lineSegments[7]);
                newSegment.accuracyThreshold = new double[] {
                                Double.valueOf(lineSegments[8]),
                                Double.valueOf(lineSegments[9]),
                                Double.valueOf(lineSegments[10])
                        };
                newSegment.orientationThreshold = Double.valueOf(lineSegments[11]);
                newSegment.basePower = Double.valueOf(lineSegments[12]);
                newSegment.lowestPower = Double.valueOf(lineSegments[13]);
                newSegment.powerGain = Double.valueOf(lineSegments[14]);
                newSegment.rampUp = Boolean.valueOf(lineSegments[15]);
                newSegment.rampDown = Boolean.valueOf(lineSegments[16]);
                newSegment.useOrientation = Boolean.valueOf(lineSegments[17]);
                pathSegments.add(newSegment);
                line = pathReader.readLine();
            }
        }
        catch (IOException e) {
            throw new IllegalStateException("Error loading path file: " + e.getMessage());
        }
        catch (ArrayIndexOutOfBoundsException e) {
           throw new UnsupportedOperationException("Encountered unparseable line in path file, is it of the correct format?");
        }
        this.pathName = this.pathName + " - loaded " + pathSegments.size() + " segments";

    }

    public void telemetryUpdate() {
        telemetry.addData("* AutopilotPath (ftc-autopilot by Aedan Cullen)", "\n" +
                "\t file:  " + pathName + "\n" +
                "\t current:  " + currentSegmentId + "\n" +
                "\t next:  " + successSegmentId + "\n" +
                "\t fallback:  " + failSegmentId);
        //telemetry.update();
    }

    public AutopilotSegment getSegment(String id) {
        for (AutopilotSegment segment : pathSegments) {
            if (segment.id.equals(id)) {
                return segment;
            }
        }
        return null;
    }

    public AutopilotSegment moveOnSuccess() {

        if (currentSegmentId.equals("NOTHINGNOTHINGNOTHING")) {
            currentSegmentId = "__start__";
            AutopilotSegment newCurrent = getSegment(currentSegmentId);
            successSegmentId = newCurrent.success;
            failSegmentId = newCurrent.fail;
            return newCurrent;
        }
        AutopilotSegment newCurrent = getSegment(successSegmentId);
        if (newCurrent != null) {
            currentSegmentId = successSegmentId;
            successSegmentId = newCurrent.success;
            failSegmentId = newCurrent.fail;
            return newCurrent;
        }
        
        return null;
    }

    public AutopilotSegment moveOnFailure() {

        if (currentSegmentId.equals("NOTHINGNOTHINGNOTHING")) {
            currentSegmentId = "__start__";
            AutopilotSegment newCurrent = getSegment(currentSegmentId);
            successSegmentId = newCurrent.success;
            failSegmentId = newCurrent.fail;
            return newCurrent;
        }
        AutopilotSegment newCurrent = getSegment(failSegmentId);
        if (newCurrent != null) {
            currentSegmentId = failSegmentId;
            successSegmentId = newCurrent.success;
            failSegmentId = newCurrent.fail;
            return newCurrent;
        }

        return null;
    }

}
