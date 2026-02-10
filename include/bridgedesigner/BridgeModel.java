/*
 * BridgeModelPhysicsOnly.java
 *
 * Minimal BridgeModel needed for 1:1 physics replication.
 *
 * What was removed:
 * - All UI/reporting (ResourceMap, BDApp, CostReportTableModel, EditableBridgeModel)
 * - All costs/report helpers (Costs inner class, getCosts*, getNotes, toTabDelimitedText, etc.)
 * - All encrypted read/write (.bdc) support (RC4/RC4Key usage)
 * - All sample/template writer helpers
 *
 * What remains:
 * - Bridge structure (joints, members, inventory, designConditions)
 * - Slenderness check helper
 * - Parsing of the CLEAR-TEXT bridge format used by BridgeModel.parseBytes()
 *   (If you only have encrypted .bdc, you cannot use this without the key.)
 */

package bridgedesigner;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Iterator;

public class BridgeModel {

    // ===== Bridge file format constants (unchanged) =====
    public static final int version = 2016;
    protected static final char DELIM = '|';
    protected static final int JOINT_COORD_LEN = 3;
    protected static final int MEMBER_JOINT_LEN = 2;
    protected static final int MEMBER_MATERIAL_LEN = 1;
    protected static final int MEMBER_SECTION_LEN = 1;
    protected static final int MEMBER_SIZE_LEN = 2;
    protected static final int N_JOINTS_LEN = 2;
    protected static final int N_MEMBERS_LEN = 3;
    protected static final int SCENARIO_CODE_LEN = 10;
    protected static final int YEAR_LEN = 4;

    // ===== Core physics-facing state =====
    protected DesignConditions designConditions;
    protected final Inventory inventory = new Inventory();
    protected final ArrayList<Joint> joints = new ArrayList<Joint>();
    protected final ArrayList<Member> members = new ArrayList<Member>();

    // Optional metadata fields (kept because parseBytes reads them)
    protected String designedBy = "";
    protected String projectId = "";
    protected int iterationNumber = 1;
    protected double labelPosition = 2.0;

    // ===== Parser state =====
    private byte[] readBuf;
    private int readPtr;

    // ===== Minimal API used by Analysis / physics code =====
    public DesignConditions getDesignConditions() {
        return designConditions;
    }

    public Inventory getInventory() {
        return inventory;
    }

    public ArrayList<Joint> getJoints() {
        return joints;
    }

    public ArrayList<Member> getMembers() {
        return members;
    }

    public int getIteration() {
        return iterationNumber;
    }

    public double getLabelPosition() {
        return labelPosition;
    }

    public String getDesignedBy() {
        return designedBy;
    }

    public String getProjectId() {
        return projectId;
    }

    public void clearStructure() {
        members.clear();
        joints.clear();
    }

    public boolean isInitialized() {
        return designConditions != null;
    }

    /**
     * Physics-relevant precheck from original BridgeModel.
     */
    public boolean isPassingSlendernessCheck() {
        Iterator<Member> me = members.iterator();
        double allowableSlenderness = designConditions.getAllowableSlenderness();
        while (me.hasNext()) {
            if (me.next().getSlenderness() > allowableSlenderness) {
                return false;
            }
        }
        return true;
    }

    /**
     * Initialize empty structure with prescribed joints.
     */
    public void initialize(DesignConditions conditions, String projectId, String designedBy) {
        this.designConditions = conditions;
        if (projectId != null) this.projectId = projectId;
        if (designedBy != null) this.designedBy = designedBy;

        clearStructure();
        for (int i = 0; i < conditions.getNPrescribedJoints(); i++) {
            joints.add(conditions.getPrescribedJoint(i));
        }
    }

    // ===== CLEAR-TEXT parsing only =====

    /**
     * Parse the clear-text (unencrypted) bridge bytes.
     * This is the same logic as BridgeModel.parseBytes().
     *
     * If you only have encrypted .bdc and no key, you cannot produce these bytes.
     */
    public void parseBytes(byte[] readBuf) throws IOException {
        this.readBuf = readBuf;
        readPtr = 0;

        DraftingGrid grid = new DraftingGrid(DraftingGrid.FINE_GRID);
        clearStructure();

        if (scanUnsigned(YEAR_LEN, "bridge designer version") != version) {
            throw new IOException("bridge design file version is not " + version);
        }

        long scenarioCode = scanUnsignedLong(SCENARIO_CODE_LEN, "scenario code");
        designConditions = DesignConditions.getDesignConditions(scenarioCode);
        if (designConditions == null) {
            throw new IOException("invalid scenario " + scenarioCode);
        }

        int n_joints = scanUnsigned(N_JOINTS_LEN, "number of joints");
        int n_members = scanUnsigned(N_MEMBERS_LEN, "number of members");

        // Joints
        for (int i = 0, n = 1; i < n_joints; i++, n++) {
            int x = scanInt(JOINT_COORD_LEN, "joint " + n + " x-coordinate");
            int y = scanInt(JOINT_COORD_LEN, "joint " + n + " y-coordinate");

            if (i < designConditions.getNPrescribedJoints()) {
                Joint joint = designConditions.getPrescribedJoint(i);
                if (x != grid.worldToGridX(joint.getPointWorld().x)
                        || y != grid.worldToGridY(joint.getPointWorld().y)) {
                    throw new IOException("bad prescribed joint " + n);
                }
                joints.add(joint);
            } else {
                joints.add(new Joint(i, new Affine.Point(grid.gridToWorldX(x), grid.gridToWorldY(y))));
            }
        }

        // Members
        for (int i = 0, n = 1; i < n_members; i++, n++) {
            int jointANumber = scanUnsigned(MEMBER_JOINT_LEN, "first joint of member " + n);
            int jointBNumber = scanUnsigned(MEMBER_JOINT_LEN, "second joint of member " + n);
            int materialIndex = scanUnsigned(MEMBER_MATERIAL_LEN, "material index of member " + n);
            int sectionIndex = scanUnsigned(MEMBER_SECTION_LEN, "section index of member " + n);
            int sizeIndex = scanUnsigned(MEMBER_SIZE_LEN, "size index of member " + n);

            members.add(new Member(
                    i,
                    joints.get(jointANumber - 1),
                    joints.get(jointBNumber - 1),
                    inventory.getMaterial(materialIndex),
                    inventory.getShape(sectionIndex, sizeIndex)
            ));
        }

        // Stored ratios (not required for physics; kept to match original parse)
        Iterator<Member> e = members.iterator();
        while (e.hasNext()) {
            Member member = e.next();
            member.setCompressionForceStrengthRatio(parseRatioEncoding(scanToDelimiter("compression/strength ratio")));
            member.setTensionForceStrengthRatio(parseRatioEncoding(scanToDelimiter("tension/strength ratio")));
        }

        designedBy = scanToDelimiter("name of designer");
        projectId = scanToDelimiter("project ID");
        iterationNumber = Integer.parseInt(scanToDelimiter("iteration"));
        labelPosition = Double.parseDouble(scanToDelimiter("label position"));
    }

    protected static double parseRatioEncoding(String s) {
        try {
            return Double.parseDouble(s);
        } catch (NumberFormatException ex) {
            // "--" or locale issues -> original returns -1
            return -1;
        }
    }

    // ===== Scanner helpers (unchanged logic) =====

    private int scanInt(int width, String what) throws IOException {
        int val = 0;
        boolean negate_p = false;

        while (width > 0 && readBuf[readPtr] == ' ') {
            width--;
            readPtr++;
        }
        if (width >= 2 && readBuf[readPtr] == '-') {
            width--;
            readPtr++;
            negate_p = true;
        }
        while (width > 0) {
            if ('0' <= readBuf[readPtr] && readBuf[readPtr] <= '9') {
                val = val * 10 + (readBuf[readPtr] - '0');
                width--;
                readPtr++;
            } else {
                throw new IOException("couldn't scan " + what);
            }
        }
        return negate_p ? -val : val;
    }

    private String scanToDelimiter(String what) {
        StringBuilder buf = new StringBuilder(16);
        while (readBuf[readPtr] != DELIM) {
            buf.append((char) readBuf[readPtr]);
            readPtr++;
        }
        readPtr++; // skip delimiter
        return buf.toString();
    }

    private int scanUnsigned(int width, String what) throws IOException {
        int val = 0;

        while (width > 0 && readBuf[readPtr] == ' ') {
            width--;
            readPtr++;
        }
        while (width > 0) {
            if ('0' <= readBuf[readPtr] && readBuf[readPtr] <= '9') {
                val = val * 10 + (readBuf[readPtr] - '0');
                width--;
                readPtr++;
            } else {
                throw new IOException("couldn't scan " + what);
            }
        }
        return val;
    }

    private long scanUnsignedLong(int width, String what) throws IOException {
        long val = 0;

        while (width > 0 && readBuf[readPtr] == ' ') {
            width--;
            readPtr++;
        }
        while (width > 0) {
            if ('0' <= readBuf[readPtr] && readBuf[readPtr] <= '9') {
                val = val * 10 + (readBuf[readPtr] - '0');
                width--;
                readPtr++;
            } else {
                throw new IOException("couldn't scan " + what);
            }
        }
        return val;
    }

    public void read(File f) throws IOException {
    byte[] bytes = Files.readAllBytes(f.toPath());
    // This expects CLEAR-TEXT bytes (the output of BridgeModel.toString().getBytes("ASCII")),
    // NOT an encrypted .bdc file.
    parseBytes(bytes);
}
}


