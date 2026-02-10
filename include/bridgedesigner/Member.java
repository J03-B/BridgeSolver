package bridgedesigner;

/**
 * Physics-only member (bar) used by BridgeModel/Analysis.
 * Removes all UI, Swing painting, and OpenGL.
 */
public final class Member {

    private int index = -1;
    private final Joint jointA;
    private final Joint jointB;
    private Material material;
    private Shape shape;

    // These are written by Analysis.initialize(...)
    private double compressionForceStrengthRatio = -1;
    private double tensionForceStrengthRatio = -1;

    public Member(int index, Joint a, Joint b, Material material, Shape shape) {
        this.index = index;
        this.jointA = a;
        this.jointB = b;
        this.material = material;
        this.shape = shape;
    }

    public Member(Joint a, Joint b, Material material, Shape shape) {
        this(-1, a, b, material, shape);
    }

    public int getIndex() {
        return index;
    }

    public void setIndex(int index) {
        this.index = index;
    }

    /** 1-based label convention used by original code. */
    public int getNumber() {
        return index + 1;
    }

    public Joint getJointA() {
        return jointA;
    }

    public Joint getJointB() {
        return jointB;
    }

    public Material getMaterial() {
        return material;
    }

    public void setMaterial(Material material) {
        this.material = material;
    }

    public Shape getShape() {
        return shape;
    }

    public void setShape(Shape shape) {
        this.shape = shape;
    }

    /** Euclidean member length in meters. */
    public double getLength() {
        return jointA.getPointWorld().distance(jointB.getPointWorld());
    }

    /** Slenderness = L / r_g, where r_g is radius of gyration. */
    public double getSlenderness() {
        return getLength() * shape.getInverseRadiusOfGyration();
    }

    public double getCompressionForceStrengthRatio() {
        return compressionForceStrengthRatio;
    }

    public void setCompressionForceStrengthRatio(double ratio) {
        this.compressionForceStrengthRatio = ratio;
    }

    public double getTensionForceStrengthRatio() {
        return tensionForceStrengthRatio;
    }

    public void setTensionForceStrengthRatio(double ratio) {
        this.tensionForceStrengthRatio = ratio;
    }
}
