package frc.util;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
public final class DistanceLookup { // Singleton class
    
    private static InterpolatingDoubleTreeMap intTable = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap timeTable = new InterpolatingDoubleTreeMap();
    private static final DistanceLookup inst = new DistanceLookup();

    private DistanceLookup(){
        // intTable.put(4.468,1.0);
        // intTable.put(0.0,0.05);
        // Key is distance in metres, value is hood angle

        intTable.put(1.48,0.2-0.020);
        intTable.put(2.5,0.4-0.020);
        intTable.put(2.38, 0.395-0.020);
        intTable.put(4.15, 0.6);
        intTable.put(3.5, 0.5-0.020); // maybe
        intTable.put(4.95, 0.8);

        // Time Lookup TODO
        // Use a Slow-Mo video to find the time from shooting until hitting the target
        
        

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
