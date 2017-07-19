package com.evolutionftc.autopilot;

import com.qualcomm.robotcore.robocol.Telemetry;

import android.os.Environment;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

// this is a snazzy autopilot for ftc made by aedan.
// it was made somewhat for team #9867, 'evolution'
// it uses lots of maths.
// it is gpl licensed.
// copyright 2017 aedan cullen.


public class AutopilotPath {

    private Telemetry telemetry;

    String pathName;

    private List<AutopilotSegment> pathSegments = new ArrayList<AutopilotSegment>();
    private String currentSegmentId = "NOTHINGNOTHINGNOTHING";

    private String successSegmentId = "NOTHINGNOTHINGNOTHING";
    private String failSegmentId = "NOTHINGNOTHINGNOTHING";

    public AutopilotPath(String pathName, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.pathName = pathName;
		String state = Environment.getExternalStorageState();
        if (!(Environment.MEDIA_MOUNTED.equals(state) || Environment.MEDIA_MOUNTED_READ_ONLY.equals(state))) {
            throw new IllegalStateException("Android external storage is not readable");
        }
        String storagePath = Environment.getExternalStorageDirectory().getAbsolutePath();
        File pathsLocation = new File(storagePath + "/AutopilotPaths");
        File pathFile = new File(pathsLocation, pathName);
        try (BufferedReader pathReader = new BufferedReader(new FileReader(pathFile))) {
            String header = pathReader.readLine();
            String line = pathReader.readLine();
			if (!header.toLowerCase().equals(
"id,success,fail,targetx,targety,targetz,steeringgain,accuracyx,accuracyy,accuracyz,basepower,lowestpower,powergain,rampup,rampdown")){
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
                newSegment.steeringGain = Double.valueOf(lineSegments[6]);
                newSegment.accuracyThreshold = new double[] {
                                Double.valueOf(lineSegments[7]),
                                Double.valueOf(lineSegments[8]),
                                Double.valueOf(lineSegments[9])
                        };
                newSegment.basePower = Double.valueOf(lineSegments[10]);
                newSegment.lowestPower = Double.valueOf(lineSegments[11]);
                newSegment.powerGain = Double.valueOf(lineSegments[12]);
                newSegment.rampUp = Boolean.valueOf(lineSegments[13]);
                newSegment.rampDown = Boolean.valueOf(lineSegments[14]);
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
        telemetryUpdate();
    }

    private void telemetryUpdate() {
        telemetry.addData("* AutopilotPath", "\n" +
                "\t csv: " + pathName + "\n" +
                "\t cur: " + currentSegmentId + "\n" +
                "\t suc: " + successSegmentId + "\n" +
                "\t fai: " + failSegmentId);
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
        telemetryUpdate();
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
        telemetryUpdate();
        return null;
    }

}
