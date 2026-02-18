package frc.util;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
public class DistanceLookup {
    
    private InterpolatingDoubleTreeMap intTable = new InterpolatingDoubleTreeMap();
    public DistanceLookup(){
        // intTable.put(4.468,1.0);
        // intTable.put(0.0,0.05);

        intTable.put(1.48,0.2);
        intTable.put(2.5,0.4);
        intTable.put(2.38, 0.395);
        intTable.put(4.15, 0.6);
        intTable.put(3.5, 0.5); // maybe

    }
    public double getValue(double ind){
        return intTable.get(ind);
    }
    
}
