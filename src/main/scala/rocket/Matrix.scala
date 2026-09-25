// See LICENSE.Berkeley for license details.

package freechips.rocketchip.rocket

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tile._

/** Matrix-configuration support for the RISC-V vector-matrix extension (VME),
  * following the SiFive Xsfmm v0.6.6 encodings and semantics.
  *
  * The matrix unit adds tm, tk and vtwiden fields to vtype (see [[VType]]),
  * uses vl as tn, and adds three configuration instructions:
  *
  *   sf.vsettn rd, rs1   // rd = tn/vl = min(rs1, LMUL*EVE, ETE)
  *   sf.vsettm rd, rs1   // rd = tm    = min(rs1, LMUL*EVE, ETE)
  *   sf.vsettk rd, rs1   // rd = tk    = min(rs1, KMAX)
  *
  * sf.vsettnt is an assembler alias of vsetvli with vtwiden != 0.
  */
object MatrixInstructions {
  def SF_VSETTN = BitPat("b1000010_00000_?????_111_?????_1010111")
  def SF_VSETTM = BitPat("b1000010_00001_?????_111_?????_1010111")
  def SF_VSETTK = BitPat("b1000010_00010_?????_111_?????_1010111")
  def cfg = Seq(SF_VSETTN, SF_VSETTM, SF_VSETTK)
}

object MatrixConfig {
  /** Maximum fused dot-product depth (Xsfmm spec section 1.4.2). 0 = unsupported. */
  def kmax(sew: Int, twiden: Int): Int = (sew, twiden) match {
    case (8, 1)  => 4
    case (8, 2)  => 4
    case (8, 4)  => 4
    case (16, 1) => 2
    case (16, 2) => 2
    case (16, 4) => 2
    case (32, 1) => 1
    case (32, 2) => 1
    case (64, 1) => 1
    case _       => 0
  }

  /** Effective tile edge: TE for TEW < 64, TE/2 for TEW = 64. */
  def ete(sew: Int, twiden: Int, te: Int): Int = if (sew * twiden < 64) te else te / 2

  /** LMUL = min(8/KMAX, 8/TWIDEN, ceil(ETE/EVE)), as log2. */
  def lmulLog2(sew: Int, twiden: Int, vLen: Int, te: Int): Int = {
    val eve = vLen / sew
    val e = ete(sew, twiden, te)
    val lmul = Seq(8 / kmax(sew, twiden), 8 / twiden, (e + eve - 1) / eve).min max 1
    log2Ceil(lmul)
  }

  /** Largest legal tm/tn for this configuration: min(LMUL*EVE, ETE). */
  def maxTn(sew: Int, twiden: Int, vLen: Int, te: Int): Int =
    ((1 << lmulLog2(sew, twiden, vLen, te)) * (vLen / sew)) min ete(sew, twiden, te)

  def vtwidenEncoding(twiden: Int): Int = log2Ceil(twiden) + 1
  def vsewEncoding(sew: Int): Int = log2Ceil(sew / 8)

  /** Returns true if inst is one of sf.vsettn/sf.vsettm/sf.vsettk. */
  def isCfg(inst: UInt): Bool = MatrixInstructions.cfg.map(_ === inst).reduce(_ || _)

  /** Computes the new vconfig and rd value for sf.vsettn/m/k.
    * `cur` must be the vconfig as seen by this instruction (with older
    * in-flight configuration updates applied).
    */
  def update(inst: UInt, rs1: UInt, cur: VConfig)(implicit p: Parameters): (VConfig, UInt) = {
    val res = WireInit(cur)
    val info = cur.vtype.matrixInfo
    val configured = cur.vtype.matrix && !cur.vtype.vill
    val sel = inst(24, 20)
    val clampTn = Mux(rs1 > info.maxTn, info.maxTn, rs1)
    val clampK = Mux(rs1 > info.kmax, info.kmax, rs1)
    when (!configured) {
      res.vtype := 0.U.asTypeOf(new VType)
      res.vtype.vill := true.B
      res.vl := 0.U
    } .elsewhen (sel === 0.U) {
      res.vl := clampTn(res.vl.getWidth - 1, 0)
    } .elsewhen (sel === 1.U) {
      res.vtype.tm := clampTn(res.vtype.tm.getWidth - 1, 0)
    } .otherwise {
      res.vtype.tk := clampK(res.vtype.tk.getWidth - 1, 0)
    }
    val rd = Mux(!configured, 0.U, Mux(sel === 2.U, clampK, clampTn))
    (res, rd)
  }
}

/** Per-vtype matrix parameters, looked up from (vsew, vtwiden). */
class MatrixVTypeInfo(implicit p: Parameters) extends CoreBundle()(p) {
  val legal = Bool()
  val lmulMag = UInt(2.W)
  val maxTn = UInt((log2Ceil(maxVLMax) + 1).W)
  val kmax = UInt(3.W)
}
