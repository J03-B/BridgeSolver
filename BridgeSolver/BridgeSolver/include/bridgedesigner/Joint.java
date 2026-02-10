package bridgedesigner;

/**
 * Physics-only joint (node) used by BridgeModel/Analysis.
 * Keeps only what the solver needs: index, fixed flag, and world position.
 */
public final class Joint {

    /** World coordinate of this joint (meters). */
    private final Affine.Point ptWorld = new Affine.Point();

    /** Whether the joint is fixed (support / prescribed joint). */
    private final boolean fixed;

    /** Index of this joint in the bridge joint list (0-based). */
    private int index = -1;

    public Joint(int index, Affine.Point ptWorld, boolean fixed) {
        this.index = index;
        this.fixed = fixed;
        this.ptWorld.setLocation(ptWorld);
    }

    public Joint(int index, Affine.Point ptWorld) {
        this(index, ptWorld, false);
    }

    public Joint(Affine.Point ptWorld) {
        this(-1, ptWorld, false);
    }

    public boolean isFixed() {
        return fixed;
    }

    public Affine.Point getPointWorld() {
        return ptWorld;
    }

    public void setIndex(int index) {
        this.index = index;
    }

    /** 0-based index used by Analysis stiffness assembly. */
    public int getIndex() {
        return index;
    }

    /** 1-based number used for file formats / UI. */
    public int getNumber() {
        return index + 1;
    }

    /** Used by BridgeModel.findJointAt(...) */
    boolean isAt(Affine.Point ptWorld) {
        // You can keep Utility.smallSq if you still have Utility;
        // otherwise use a literal tolerance.
        final double dx = this.ptWorld.x - ptWorld.x;
        final double dy = this.ptWorld.y - ptWorld.y;
        return (dx * dx + dy * dy) < 1e-12;
    }
}
