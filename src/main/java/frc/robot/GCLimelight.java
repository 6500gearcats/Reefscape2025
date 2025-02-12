
package frc.robot;

import frc.robot.LimelightHelpers.LimelightResults;


public class GCLimelight {
    // Limelight name
    private String name;

    public GCLimelight(String theName)
    {
        name = theName;
    }

    public String getName()
    {
        return name;
    }

    public double getYawDegrees(){
        if(LimelightHelpers.getTV(name))
        {
            return LimelightHelpers.getTX(name);
        }
        return 0;
    }

    public double getPitchDegrees(){
        if(LimelightHelpers.getTV(name))
        {
            return LimelightHelpers.getTY(name);
        }
        return 0;
    }

    public double getTargetAreaPercent()
    {
        if(LimelightHelpers.getTV(name))
        {
            return LimelightHelpers.getTA(name);
        }
        return 0;
    }

    public double getTargetDistanceX()
    {
        if(LimelightHelpers.getTV(name))
        {
            double x = (6.5/(2*getTargetAreaPercent()))/Math.tan(90*Math.PI/180);
            double distance = x/Math.cos(getYawDegrees()*Math.PI/180);
            return distance;
        }
        return 0; // It's Tyler's fault
    }

    public double getChosenTargetYawDegrees(int fiducialID){
        LimelightResults results = LimelightHelpers.getLatestResults(name);
        if(LimelightHelpers.getTV(name)){
            for(int i = 0; i < results.targets_Fiducials.length; i++)
            {
                if(results.targets_Fiducials[i].fiducialID == fiducialID)
                {
                    return results.targets_Fiducials[i].tx;
                }
            }
        }
        return 0.0;
    }

    public double getChosenTargetPitchDegrees(int fiducialID){
        LimelightResults results = LimelightHelpers.getLatestResults(name);
        if(LimelightHelpers.getTV(name)){
            for(int i = 0; i < results.targets_Fiducials.length; i++)
            {
                if(results.targets_Fiducials[i].fiducialID == fiducialID)
                {
                    return results.targets_Fiducials[i].ty;
                }
            }
        }
        return 0.0;
    }

    public double getChosenTargetSkewDegrees(int fiducialID){
        LimelightResults results = LimelightHelpers.getLatestResults(name);
        if(LimelightHelpers.getTV(name)){
            for(int i = 0; i < results.targets_Fiducials.length; i++)
            {
                if(results.targets_Fiducials[i].fiducialID == fiducialID)
                {
                    return results.targets_Fiducials[i].ts;
                }
            }
        }
        return 0.0;
    }

    public double getChosenTargetDistanceX(int fiducialID){
        LimelightResults results = LimelightHelpers.getLatestResults(name);
        if(LimelightHelpers.getTV(name)){
            for(int i = 0; i < results.targets_Fiducials.length; i++)
            {
                if(results.targets_Fiducials[i].fiducialID == fiducialID)
                {
                    double x = (6.5/(2*getTargetAreaPercent()))/Math.tan(90*Math.PI/180);
                    double distance = x/Math.cos(getChosenTargetYawDegrees(fiducialID)*Math.PI/180);
                    return distance;
                }
            }
        }
        return 0.0;        
    }
}
