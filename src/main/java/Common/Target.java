package Common;

/** Create a target with the height and detection type */
public class Target {

    public enum DetectionType {
        Reflective,
        AprilTag,
        GamePiece;
    }

    private double heightFromGroundsInInches;
    private DetectionType detectionType;
    private double alignmentTolerance;


    public Target(double heightFromGroundsInInches, DetectionType detectionType, double alignmentTolerance) {
        this.heightFromGroundsInInches = heightFromGroundsInInches;
        this.detectionType = detectionType;
    }

    
   public double getHeightFromGroundsInInches() {
        return heightFromGroundsInInches;
    }

    public DetectionType getDetectionType() {
        return detectionType;
    }

    public double getAlignmentTolerance() {
        return alignmentTolerance;
    }
} 
