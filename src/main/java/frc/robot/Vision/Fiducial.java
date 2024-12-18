package frc.robot.Vision;

/***
 * OLD IMPLEMENTATION -> DO NOT USE (used before april tags had the data directly in shuffleboard)
 * This used to be a class that encapsulated the fiducial data from the json files
 * I.e. from the json data we would create Fiducial classes that had all the data we needed
 * Just converting string to object data essentially
 */
@Deprecated
public class Fiducial {

  private int fID;
  private String fam;
  private double[] pts;
  private double[] skew;
  private double[] t6c_ts;
  private double[] t6r_fs;
  private double[] t6r_ts;
  private double[] t6t_cs;
  private double[] t6t_rs;
  private double ta;
  private double tx;
  private double txp;
  private double ty;
  private double typ;

  public int getFID() {
    return this.fID;
  }

  public void setFID(int fID) {
    this.fID = fID;
  }

  public String getFam() {
    return this.fam;
  }

  public void setFam(String fam) {
    this.fam = fam;
  }

  public double[] getPts() {
    return this.pts;
  }

  public void setPts(double[] pts) {
    this.pts = pts;
  }

  public double[] getSkew() {
    return this.skew;
  }

  public void setSkew(double[] skew) {
    this.skew = skew;
  }

  public double[] getT6c_ts() {
    return this.t6c_ts;
  }

  public void setT6c_ts(double[] t6c_ts) {
    this.t6c_ts = t6c_ts;
  }

  public double[] getT6r_fs() {
    return this.t6r_fs;
  }

  public void setT6r_fs(double[] t6r_fs) {
    this.t6r_fs = t6r_fs;
  }

  public double[] getT6r_ts() {
    return this.t6r_ts;
  }

  public void setT6r_ts(double[] t6r_ts) {
    this.t6r_ts = t6r_ts;
  }

  public double[] getT6t_cs() {
    return this.t6t_cs;
  }

  public void setT6t_cs(double[] t6t_cs) {
    this.t6t_cs = t6t_cs;
  }

  public double[] getT6t_rs() {
    return this.t6t_rs;
  }

  public void setT6t_rs(double[] t6t_rs) {
    this.t6t_rs = t6t_rs;
  }

  public double getTa() {
    return this.ta;
  }

  public void setTa(double ta) {
    this.ta = ta;
  }

  public double getTx() {
    return this.tx;
  }

  public void setTx(double tx) {
    this.tx = tx;
  }

  public double getTxp() {
    return this.txp;
  }

  public void setTxp(double txp) {
    this.txp = txp;
  }

  public double getTy() {
    return this.ty;
  }

  public void setTy(double ty) {
    this.ty = ty;
  }

  public double getTyp() {
    return this.typ;
  }

  public void setTyp(double typ) {
    this.typ = typ;
  }
}
