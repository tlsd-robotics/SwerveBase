package Common;

/** 
 * Creates a new Limelight Pipeline using the pipelines ID and LED state
 * 
*/
public class LimelightPipeline {
    private double id;
    private boolean ledState;
  
    
    public LimelightPipeline(double id, boolean ledState) {
      this.id = id;
      this.ledState = ledState;
    }

    public double getId() {
      return id;
    }

    public boolean getLedState() {
      return ledState;
    }

}