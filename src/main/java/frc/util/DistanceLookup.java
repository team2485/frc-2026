package frc.util;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
public final class DistanceLookup { // Singleton class
    
    private static InterpolatingDoubleTreeMap intTable = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap timeTable = new InterpolatingDoubleTreeMap();
    private static final DistanceLookup inst = new DistanceLookup();
    private static double distancesOffset = -0.08; // This number can be used to easily change shot distance :)
    private DistanceLookup(){
        // intTable.put(4.468,1.0);
        // intTable.put(0.0,0.05);
        // Key is distance in metres, value is hood angle

        // intTable.put(1.48,0.3);
        // intTable.put(2.5,0.5); // maybe
        // intTable.put(2.38, 0.55);
        // intTable.put(4.15, 0.7);
        // intTable.put(3.5, 0.6); // maybe
        // intTable.put(4.95, 0.9);
        // intTable.put(3.2, 0.57);
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            // distancesOffset *= -1;
        }
        intTable.put(1.12, 0.25+distancesOffset);
        intTable.put(1.48,0.35+distancesOffset);
        intTable.put(1.98, 0.42+distancesOffset);
        intTable.put(2.49, 0.45+distancesOffset);
        intTable.put(2.90, .54+distancesOffset);
        intTable.put(3.252, .59+distancesOffset);
        intTable.put(3.55,0.61+distancesOffset);
        intTable.put(4.0, 0.65+distancesOffset);
        intTable.put(4.45, .747);
        intTable.put(4.95, 0.9);

        // Time Lookup
        // Use a Slow-Mo video to find the time from shooting until hitting the target
        timeTable.put(2.13, 1.5);
        timeTable.put(4.34, 1.4);
        timeTable.put(1.59, 1.4);
    }
    public static DistanceLookup getSelf(){
        return inst;
    }
    public static double getValue(double ind){
        return intTable.get(ind);
    }
    public static double getTime(double ind){
        return timeTable.get(ind);
    }
}
