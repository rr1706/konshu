package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class VisionConstants {

        public static InterpolatingDoubleTreeMap Tag_N1_2 = new InterpolatingDoubleTreeMap();
        public static InterpolatingDoubleTreeMap Tag_N3 = new InterpolatingDoubleTreeMap();

        public static InterpolatingDoubleTreeMap MultTag_N1_2 = new InterpolatingDoubleTreeMap();
        public static InterpolatingDoubleTreeMap MultTag_N3 = new InterpolatingDoubleTreeMap();

        public static void SetUpTrustConstants(){
            Tag_N1_2.put(.25,40.0);
            Tag_N1_2.put(1.0,20.0);
            Tag_N1_2.put(2.0,2.0);

            Tag_N3.put(.25,4000.0);
            Tag_N3.put(.25,2000.0);
            Tag_N3.put(.25,200.0);
                        
            MultTag_N1_2.put(0.1, 40.0);
            MultTag_N1_2.put(0.25, 20.0);
            MultTag_N1_2.put(1.0, 2.0);

            MultTag_N3.put(0.1, 1000.0);
            MultTag_N3.put(0.25, 200.0);
            MultTag_N3.put(1.0, 50.0);

        }
        
    }    
