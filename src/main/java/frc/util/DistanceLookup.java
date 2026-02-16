package frc.util;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
public class DistanceLookup {
    
    private InterpolatingDoubleTreeMap intTable = new InterpolatingDoubleTreeMap();
    public DistanceLookup(){
        intTable.put(4.468,1.0);
        intTable.put(1.27,.2);
    }
    public double getValue(double ind){
        return intTable.get(ind);
    }
    
}
