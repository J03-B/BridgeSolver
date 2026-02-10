/*
 * Analysis.java  
 *   
 * Copyright (C) 2009 Eugene K. Ressler
 *   
 * This program is distributed in the hope that it will be useful,  
 * but WITHOUT ANY WARRANTY; without even the implied warranty of  
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the  
 * GNU General Public License for more details.  
 *   
 * You should have received a copy of the GNU General Public License  
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.  
 */
package bridgedesigner;

import java.io.File;
import java.io.IOException;
import java.util.Iterator;

/**
 * Perform an analysis of forces acting on a BridgeModel.
 * Includes both correct analysis and an intentionally perturbed one
 * for the failure animation.
 * 
 * @author Eugene K. Ressler
 */
public class Analysis {

    /**
     * Steel code factors.
     */
    private static final double deadLoadFactor = 1.35;
    private static final double liveLoadFactor = 1.75 * 1.33;
    
    /**
     * Multiplier for degrading the strength of members to animate the bridge failure.
     */
    private static final double failedMemberDegradation = 1.0 / 50.0;
    
    /**
     * Special values so that a double can be used to encode a flag for a 
     * failed member, an intact member, or (for the case where we are computing
     * a failure animation), the base getLength of a failed member so that it
     * can be drawn with a parabola or broken section.
     */
    private static final double NOT_FAILED = -1;
    private static final double FAILED = 1e6;
    
    /**
     * Analysis has not been initialized yet.
     */
    public static final int NO_STATUS = 0;
    /**
     * Analysis was completed, but the final slenderness check failed.
     * Even though the analysis was completed, it's bogus. We use value 1
     * because the load test report button should be disabled when the check
     * fails. The UNSTABLE case is the dividing line.
     */
    public static final int FAILS_SLENDERNESS = 1;
    /**
     * Analysis could not be completed because bridge is unstable.
     */
    public static final int UNSTABLE = 2;
    /**
     * Analysis was completed, but the load test failed.
     */
    public static final int FAILS_LOAD_TEST = 3;
    /**
     * Analysis was completed, and the bridge passed.
     */
    public static final int PASSES = 4;

    /**
     * Bridge for which analysis was last initialized.
     */
    private BridgeModel bridge;
    
    /**
     * Results matrices:
     * For member forces, indexes are [load index] [member index]
     * For joint displacements, indexes are [load index] [joint index]
     * 
     * In turn, there is one load index for each loaded (deck) joint.
     */
    private double[][] memberForce;
    private double[][] jointDisplacement;
    private boolean[][] memberFails;
    private double[] memberCompressiveStrength;
    private double[] memberTensileStrength;
    private double[] maxMemberCompressiveForces;
    private double[] maxMemberTensileForces;
    private int status = NO_STATUS;

    /**
     * Return the analysis status.
     * <pre>
     * NO_STATUS = 0;          analysis has not been performed
     * UNSTABLE = 1;           analysis could not complete because bridge is unstable
     * FAILS_SLENDERNESS = 2;  analysis completed, but at least one member is too slender
     * FAILS_LOAD_TEST = 3;    analysis completed, but bridge could not carry the load
     * PASSES = 4;             analysis completed, and bridge carries load
     * </pre>
     * @return analysis status indicator
     */
    public int getStatus() {
        return status;
    }
    
    /**
     * Return the member force of a given member and load case.  It is the caller's responsibility
     * to ensure the analysis is valid and indices are in range.
     * 
     * @param ilc load case index
     * @param im member index
     * @return member force
     */
    public double getMemberForce(int ilc, int im) {
        return memberForce[ilc][im];
    }

    /**
     * Return the x-component of the displacement of a given joint and load case.  It is the caller's responsibility
     * to ensure the analysis is valid and indices are in range.
     * 
     * @param ilc load case index
     * @param ij joint index
     * @return x-component of displacement
     */
    public double getXJointDisplacement(int ilc, int ij) {
        return jointDisplacement[ilc][2 * ij];
    }

    /**
     * Return the y-component of the displacement of a given joint and load case. It is the caller's responsibility
     * to ensure the analysis is valid and indices are in range.
     * 
     * @param ilc load case index
     * @param ij joint index
     * @return y-component of displacement
     */
    public double getYJointDisplacement(int ilc, int ij) {
        return jointDisplacement[ilc][2 * ij + 1];
    }

    /**
     * Return the maximum compressive force acting on a given member among all load cases.  It is the caller's 
     * responsibility to ensure the analysis is valid and indices are in range.
     * 
     * @param i member index
     * @return maximum compressive force
     */
    public double getMemberCompressiveForce(int i) {
        return maxMemberCompressiveForces[i];
    }
    
    /**
     * Return the maximum tensile force acting on a given member among all load cases.  It is the caller's 
     * responsibility to ensure the analysis is valid and indices are in range.
     * 
     * @param i member index
     * @return maximum tensile force
     */
    public double getMemberTensileForce(int i) {
        return maxMemberTensileForces[i];
    }
    
    /**
     * Return the max allowable compressive force that may act on a given member before it fails.
     * 
     * This computation ignores slenderness.  Slenderness failures are considered separately.
     * 
     * @param i member index
     * @return compressive strength
     */
    public double getMemberCompressiveStrength(int i) {
        return memberCompressiveStrength[i];
    }
    
    /**
     * Return the max allowable tensile force that may act on a given member before it fails.
     * 
     * @param i member index
     * @return tensile strength
     */
    public double getMemberTensileStrength(int i) {
        return memberTensileStrength[i];
    }
    
    /**
     * Analyze the given bridge and store the results internally for future queries.
     * This mimics the WPBD code exactly. There exists a more precise and efficient algorithm.
     * 
     * @param bridge bridge to analyze
     */
    public void initialize(BridgeModel bridge) {
        initialize(bridge, null);
    }

    /**
     * Analyze the given bridge and store the results internally for future queries.
     * Artificially decrease the strength of failed members to support the failure animation.
     * This mimics the original WPBD code exactly. There exists a more precise and efficient algorithm.
     * 
     * @param bridge bridge to analyze
     * @param failureStatus status of failed members: FAILED, NOT_FAILED, base member getLength, which implies FAILED.
     */
    public void initialize(BridgeModel bridge, double [] failureStatus) {
        this.bridge = bridge;
        DesignConditions conditions = bridge.getDesignConditions();
        status = NO_STATUS;
        int nJoints = bridge.getJoints().size();
        int nEquations = 2 * nJoints;
        int nMembers = bridge.getMembers().size();
        Member[] members = bridge.getMembers().toArray(new Member[nMembers]);
        double[] length = new double[members.length];
        double[] cosX = new double[members.length];
        double[] cosY = new double[members.length];
        for (int i = 0; i < members.length; i++) {
            Affine.Point a = members[i].getJointA().getPointWorld();
            Affine.Point b = members[i].getJointB().getPointWorld();
            double dx = b.x - a.x;
            double dy = b.y - a.y;
            length[i] = hypot(dx, dy);
            cosX[i] = dx / length[i];
            cosY[i] = dy / length[i];
        }
        final int nLoadInstances = conditions.getNLoadedJoints();
        final double pointLoads[][] = new double[nLoadInstances][nEquations];
        for (int im = 0; im < nMembers; im++) {
            double deadLoad =
                    deadLoadFactor *
                    members[im].getShape().getArea() *
                    length[im] *
                    members[im].getMaterial().getDensity() * 9.8066 / 2.0 / 1000.0;
            int dof1 = 2 * members[im].getJointA().getIndex() + 1;
            int dof2 = 2 * members[im].getJointB().getIndex() + 1;
            for (int ilc = 0; ilc < nLoadInstances; ilc++) {
                pointLoads[ilc][dof1] -= deadLoad;
                pointLoads[ilc][dof2] -= deadLoad;
            }
        }
        final double pointDeadLoad = (conditions.getDeckType() == DesignConditions.MEDIUM_STRENGTH_DECK) ? 
            deadLoadFactor * 120.265 + 33.097 : 
            deadLoadFactor * 82.608 + 33.097;
        for (int ij = 0; ij < conditions.getNLoadedJoints(); ij++) {
            int dof = 2 * ij + 1;
            for (int ilc = 0; ilc < nLoadInstances; ilc++) {
                double load = pointDeadLoad;
                if (ij == 0 || ij == conditions.getNLoadedJoints() - 1) {
                    load /= 2;
                }
                pointLoads[ilc][dof] -= load;
            }
        }
        // Standard (light) truck.
        double frontAxleLoad = 44;
        double rearAxleLoad = 181;
        if (conditions.getLoadType() != DesignConditions.STANDARD_TRUCK) {
            // Heavy truck.
            frontAxleLoad = 124;
            rearAxleLoad = 124;
        }
        for (int ilc = 1; ilc < nLoadInstances; ilc++) {
            int iFront = 2 * ilc + 1;
            int iRear = iFront - 2;
            pointLoads[ilc][iFront] -= liveLoadFactor * frontAxleLoad;
            pointLoads[ilc][iRear] -= liveLoadFactor * rearAxleLoad;
        }
        boolean xRestraint[] = new boolean[nJoints];
        boolean yRestraint[] = new boolean[nJoints];
        xRestraint[0] = yRestraint[0] = yRestraint[conditions.getNLoadedJoints() - 1] = true;
        if (conditions.isPier()) {
            int i = conditions.getPierJointIndex();
            xRestraint[i] = yRestraint[i] = true;
            if (conditions.isHiPier()) {
                xRestraint[0] = false;
            }
        }
        if (conditions.isArch()) {
            int i = conditions.getArchJointIndex();
            xRestraint[0] = yRestraint[0] = yRestraint[conditions.getNLoadedJoints() - 1] = false;
            xRestraint[i] = yRestraint[i] = true;
            xRestraint[i + 1] = yRestraint[i + 1] = true;
        }
        if (conditions.isLeftAnchorage()) {
            int i = conditions.getLeftAnchorageJointIndex();
            xRestraint[i] = yRestraint[i] = true;
        }
        if (conditions.isRightAnchorage()) {
            int i = conditions.getRightAnchorageJointIndex();
            xRestraint[i] = yRestraint[i] = true;
        }
        double stiffness[][] = new double[nEquations][nEquations];
        for (int im = 0; im < nMembers; im++) {
            double e = members[im].getMaterial().getE();
            if (failureStatus != null && failureStatus[im] != NOT_FAILED) {
                e *= failedMemberDegradation;
            }
            double aEOverL = members[im].getShape().getArea() * e / length[im];
            double xx = aEOverL * sqr(cosX[im]);
            double yy = aEOverL * sqr(cosY[im]);
            double xy = aEOverL * cosX[im] * cosY[im];
            int j1 = members[im].getJointA().getIndex();
            int j2 = members[im].getJointB().getIndex();
            int j1x = 2 * j1;
            int j1y = 2 * j1 + 1;
            int j2x = 2 * j2;
            int j2y = 2 * j2 + 1;
            stiffness[j1x][j1x] += xx;
            stiffness[j1x][j1y] += xy;
            stiffness[j1x][j2x] -= xx;
            stiffness[j1x][j2y] -= xy;
            stiffness[j1y][j1x] += xy;
            stiffness[j1y][j1y] += yy;
            stiffness[j1y][j2x] -= xy;
            stiffness[j1y][j2y] -= yy;
            stiffness[j2x][j1x] -= xx;
            stiffness[j2x][j1y] -= xy;
            stiffness[j2x][j2x] += xx;
            stiffness[j2x][j2y] += xy;
            stiffness[j2y][j1x] -= xy;
            stiffness[j2y][j1y] -= yy;
            stiffness[j2y][j2x] += xy;
            stiffness[j2y][j2y] += yy;
        }
        for (int ilc = 0; ilc < nLoadInstances; ilc++) {
            for (int ij = 0; ij < nJoints; ij++) {
                if (xRestraint[ij]) {
                    int ix = 2 * ij;
                    for (int ie = 0; ie < nEquations; ie++) {
                        stiffness[ix][ie] = stiffness[ie][ix] = 0;
                    }
                    stiffness[ix][ix] = 1;
                    pointLoads[ilc][ix] = 0;
                }
                if (yRestraint[ij]) {
                    int iy = 2 * ij + 1;
                    for (int ie = 0; ie < nEquations; ie++) {
                        stiffness[iy][ie] = stiffness[ie][iy] = 0;
                    }
                    stiffness[iy][iy] = 1;
                    pointLoads[ilc][iy] = 0;
                }
            }
        }
        for (int ie = 0; ie < nEquations; ie++) {
            double pivot = stiffness[ie][ie];
            if (Math.abs(pivot) < 0.99) {
                status = UNSTABLE;
                return;
            }
            double pivr = 1.0 / pivot;
            for (int k = 0; k < nEquations; k++) {
                stiffness[ie][k] /= pivot;
            }
            for (int k = 0; k < nEquations; k++) {
                if (k != ie) {
                    pivot = stiffness[k][ie];
                    for (int j = 0; j < nEquations; j++) {
                        stiffness[k][j] -= stiffness[ie][j] * pivot;
                    }
                    stiffness[k][ie] = -pivot * pivr;
                }
            }
            stiffness[ie][ie] = pivr;
        }
        memberForce = new double[nLoadInstances][nMembers];
        memberFails = new boolean[nLoadInstances][nMembers];
        jointDisplacement = new double[nLoadInstances][nEquations];
        for (int ilc = 0; ilc < nLoadInstances; ilc++) {
            for (int ie = 0; ie < nEquations; ie++) {
                double tmp = 0;
                for (int je = 0; je < nEquations; je++) {
                    tmp += stiffness[ie][je] * pointLoads[ilc][je];
                }
                jointDisplacement[ilc][ie] = tmp;
            }
            // Compute member forces.
            for (int im = 0; im < nMembers; im++) {
                double e = members[im].getMaterial().getE();
                if (failureStatus != null && failureStatus[im] != NOT_FAILED) {
                    e *= failedMemberDegradation;
                }
                double aeOverL = members[im].getShape().getArea() * e / length[im];
                int ija = members[im].getJointA().getIndex();
                int ijb = members[im].getJointB().getIndex();
                memberForce[ilc][im] = aeOverL *
                        ((cosX[im] * (getXJointDisplacement(ilc, ijb) - getXJointDisplacement(ilc, ija))) +
                        (cosY[im] * (getYJointDisplacement(ilc, ijb) - getYJointDisplacement(ilc, ija))));
            }
        }
        
        memberCompressiveStrength = new double[nMembers];
        memberTensileStrength = new double[nMembers];
        maxMemberCompressiveForces = new double[nMembers];
        maxMemberTensileForces = new double[nMembers];
        
        for (int im = 0; im < nMembers; im++) {
            final Material material = members[im].getMaterial();
            final Shape shape = members[im].getShape();
            memberCompressiveStrength[im] = Inventory.compressiveStrength(material, shape, length[im]);
            memberTensileStrength[im] = Inventory.tensileStrength(material, shape);
        }
        status = PASSES;
        for (int im = 0; im < nMembers; im++) {
            double maxCompression = 0;
            double maxTension = 0;
            for (int ilc = 0; ilc < nLoadInstances; ilc++) {
                double force = memberForce[ilc][im];
                if (force < 0) {
                    force = -force;
                    if (force > maxCompression) {
                        maxCompression = force;
                    }
                    memberFails[ilc][im] = (force / memberCompressiveStrength[im] > 1.0);
                } else {
                    if (force > maxTension) {
                        maxTension = force;
                    }
                    memberFails[ilc][im] = (force / memberTensileStrength[im] > 1.0);
                }
            }
            double cRatio = maxCompression / memberCompressiveStrength[im];
            double tRatio = maxTension / memberTensileStrength[im];
            // A fail for any member of any kind is a fail overall.
            if (cRatio > 1 || tRatio > 1) {
                status = FAILS_LOAD_TEST;
            }
            // Copy ratio information back to the bridge unless we're computing the intentionally distorted 
            // failure bridge.
            if (failureStatus == null) {
                members[im].setCompressionForceStrengthRatio(cRatio);
                members[im].setTensionForceStrengthRatio(tRatio);
            }
            maxMemberCompressiveForces[im] = maxCompression;
            maxMemberTensileForces[im] = maxTension;
        }
        if (!bridge.isPassingSlendernessCheck()) {
            status = FAILS_SLENDERNESS;
        }
    }

    // This is about 50 times faster than Math.hypot() !.
    private static double hypot(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }
        
    public static boolean isStatusBaseLength(double status) {
        return status != FAILED && status != NOT_FAILED;
    }
            
    /**
     * Square the parameter.
     * 
     * @param x quantity to be squared
     * @return the square
     */
    private static double sqr(double x) {
        return x * x;
    }
    
    // Simple command line utility for checking a bridge's pass-fail status.
    private static class Runnable {

        private BridgeModel bridge = new BridgeModel();
        private Analysis analysis = new Analysis();

        private void run(String fileName) {
            try {
                bridge.read(new File(fileName));
                analysis.initialize(bridge);
                if (analysis.getStatus() >= FAILS_SLENDERNESS) {
                    System.out.print(fileName + ": ");
                    System.out.println(analysis.getStatus() == PASSES ? "passes." : "fails.");
                }
            } catch (IOException ex) {
                System.err.println("could not open '" + fileName + "' as a bridge file.");
            }
        }
    }
   
    public static void main(String [] args) {
        if (args.length != 1) {
            System.err.println("usage: java Analysis FileName");
        }
        else {
            new Runnable().run(args[0]);
        }
    }
}
