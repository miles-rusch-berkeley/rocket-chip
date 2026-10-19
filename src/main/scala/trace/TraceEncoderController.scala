// See LICENSE.Berkeley for license details.
// See LICENSE.SiFive for license details.

package freechips.rocketchip.trace

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import org.chipsalliance.diplomacy.lazymodule._
import freechips.rocketchip.diplomacy.{AddressSet}
import freechips.rocketchip.resources.{Description, ResourceBindings, SimpleDevice}
import freechips.rocketchip.tilelink.TLRegisterNode
import freechips.rocketchip.regmapper.{RegField, RegFieldDesc}

object TraceSinkTarget {
  def width = 8
}

class TraceEncoderControlInterface() extends Bundle {
  val enable = Bool()
  val target = UInt(TraceSinkTarget.width.W)
  val bp_mode = UInt(32.W)
  // Lossy mode: on queue pressure the encoder emits Sync Pause, drops whole
  // retire groups, and emits Sync Resume once the queues drain, instead of
  // asserting stall to the core. Its own register so it is never written
  // together with enable.
  val lossy = Bool()
  // Resume watermark in queue entries (absolute, not a fraction: the quantity
  // that sets gap length is the headroom drop_threshold - watermark, which is
  // an entry count). 0 selects the encoder's elaboration default, so an
  // unprogrammed encoder is bit-identical to the previous behaviour.
  val resume_watermark = UInt(32.W)
  // Occupancy above which lossy mode starts dropping, in queue entries. 0 selects
  // the physical bound (bufferDepth - enqBound), i.e. unchanged behaviour. Lower
  // values emulate a SHALLOWER queue: in lossy mode occupancy never exceeds the
  // threshold, so the entries above it are never touched and the encoder behaves
  // exactly as a smaller build would. It can only ever emulate smaller -- the
  // physical can_accept bound remains a hard floor.
  val drop_threshold = UInt(32.W)
  // Periodic sync interval in trace bytes; 0 (reset) disables it. After at least
  // this many bytes the encoder emits a Sync Periodic checkpoint, so a trace that
  // wrapped in a ring buffer can be decoded from the first checkpoint it holds.
  val sync_interval = UInt(32.W)
}

class TraceEncoderPerformanceInterface() extends Bundle {
  // packet queues are at the high watermark this cycle. Lossless mode: the
  // core is being stalled. Lossy mode: the core would have been stalled.
  val full = Bool()
  // inside a Pause..Resume gap this cycle
  val paused = Bool()
  // a Pause packet was enqueued this cycle
  val pause_fire = Bool()
  // packets discarded this cycle (whole retire groups; up to nGroups)
  val dropped_inc = UInt(8.W)
  // instructions retired inside those discarded groups this cycle. Counts every
  // retired slot, not only the message-carrying ones dropped_inc counts, so a gap
  // that swallows nothing but sequential instructions still registers here.
  val dropped_insns_inc = UInt(8.W)
  // The watermark actually in force, after the encoder has applied its default
  // and clamped to the drop threshold. Status, not a counter: reading it back
  // lets software discover the legal maximum by writing all-ones.
  val resume_watermark_eff = UInt(32.W)
  // Effective drop threshold after default substitution and clamping.
  val drop_threshold_eff = UInt(32.W)
}

class TraceEncoderController(addr: BigInt, beatBytes: Int, hartId: Int)(implicit p: Parameters) extends LazyModule {

  val device = new SimpleDevice(s"trace-encoder-controller$hartId", Seq("ucbbar,trace")) {
    override def describe(resources: ResourceBindings): Description = {
      val Description(name, mapping) = super.describe(resources)
      val traceDmaProp = Map(
        "ucbbar,trace-dma" -> resources("ucbbar,trace-dma").map(_.value)
      ).collect { case (k, v) if v.nonEmpty => (k, v) }
      Description(name, mapping ++ traceDmaProp)
    }
  }
  val node = TLRegisterNode(
    address = Seq(AddressSet(addr, 0xFFF)),
    device = device,
    beatBytes = beatBytes
  )
  override lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    val io = IO(new Bundle {
      val control = Output(new TraceEncoderControlInterface())
      val perf = Input(new TraceEncoderPerformanceInterface())
    })

    val control_reg_write_valid = Wire(Bool())
    val control_reg_bits = RegInit(1.U(2.W))
    val enable = control_reg_bits(1) // 2 means enable, 0 means disable
    val active = control_reg_bits(0) // 1 means active, 0 means inactive
    io.control.enable := enable

    val trace_encoder_impl = RegInit(0.U(32.W))

    val trace_sink_target = RegInit(0.U(TraceSinkTarget.width.W))
    io.control.target := trace_sink_target.asUInt

    val trace_bp_mode = RegInit(0.U(32.W))
    io.control.bp_mode := trace_bp_mode

    val trace_lossy = RegInit(0.U(32.W))
    io.control.lossy := trace_lossy(0)

    val trace_resume_wm = RegInit(0.U(32.W))
    io.control.resume_watermark := trace_resume_wm

    val trace_drop_thresh = RegInit(0.U(32.W))
    io.control.drop_threshold := trace_drop_thresh

    val trace_sync_interval = RegInit(0.U(32.W))
    io.control.sync_interval := trace_sync_interval

    def traceEncoderControlRegWrite(valid: Bool, bits: UInt): Bool = {
      control_reg_write_valid := valid
      when (control_reg_write_valid) {
        control_reg_bits := bits
      }
      true.B
    }

    def traceEncoderControlRegRead(ready: Bool): (Bool, UInt) = {
      (true.B, control_reg_bits)
    }

    // Write stores the request; read returns what the encoder actually applied
    // (default substituted, clamped below the drop threshold), so software can
    // discover this build's legal maximum by writing all-ones and reading back.
    def resumeWatermarkWrite(valid: Bool, bits: UInt): Bool = {
      when (valid) { trace_resume_wm := bits }
      true.B
    }
    def resumeWatermarkRead(ready: Bool): (Bool, UInt) = {
      (true.B, io.perf.resume_watermark_eff)
    }

    def dropThresholdWrite(valid: Bool, bits: UInt): Bool = {
      when (valid) { trace_drop_thresh := bits }
      true.B
    }
    def dropThresholdRead(ready: Bool): (Bool, UInt) = {
      (true.B, io.perf.drop_threshold_eff)
    }

    // Counters are window-scoped: cleared as tracing is enabled, frozen once it
    // is disabled, so a read after disable describes exactly one traced window.
    // They previously accumulated from hardware reset, which meant a second
    // window in the same boot silently summed with the first, and the encoder's
    // post-disable drain (Pause -> Resume -> End) kept incrementing after
    // software had already stopped tracing. That drain was the dominant term in
    // the 1.6% disagreement measured between gap_cycles and the Pause..Resume
    // timestamps recovered from the trace itself.
    val counter_clear = enable && !RegNext(enable)
    def perfCounter(inc: UInt): UInt = {
      val reg = RegInit(0.U(64.W))
      when (counter_clear) {
        reg := 0.U
      } .elsewhen (enable) {
        reg := reg + inc
      }
      reg
    }
    val full            = perfCounter(io.perf.full.asUInt)
    val gap_cycles      = perfCounter(io.perf.paused.asUInt)
    val dropped_packets = perfCounter(io.perf.dropped_inc)
    val dropped_insns   = perfCounter(io.perf.dropped_insns_inc)
    val pause_count     = perfCounter(io.perf.pause_fire.asUInt)

    val regmap = node.regmap(
      Seq(
        0x00 -> Seq(
          RegField(2, traceEncoderControlRegRead(_), traceEncoderControlRegWrite(_, _),
            RegFieldDesc("control", "Control trace encoder"))
        ),
        0x04 -> Seq(
          RegField.r(32, trace_encoder_impl,
            RegFieldDesc("impl", "Trace encoder implementation"))
        ),
        0x20 -> Seq(
          RegField(8, trace_sink_target,
            RegFieldDesc("target", "Trace sink target"))
        ),
        0x24 -> Seq(
          RegField(32, trace_bp_mode,
            RegFieldDesc("bp_mode", "Trace branch predictor mode"))
        ),
        0x28 -> Seq(
          RegField.r(64, full,
            RegFieldDesc("full", "Cycles the packet queues were at the high watermark (lossless: core stalled; lossy: core would have stalled)"))
        ),
        0x30 -> Seq(
          RegField(32, trace_lossy,
            RegFieldDesc("lossy", "Lossy mode (bit 0): pause/resume instead of stalling the core"))
        ),
        0x38 -> Seq(
          RegField.r(64, gap_cycles,
            RegFieldDesc("gapCycles", "Cycles spent inside Pause..Resume gaps"))
        ),
        0x40 -> Seq(
          RegField.r(64, dropped_packets,
            RegFieldDesc("droppedPackets", "Packets discarded in lossy mode"))
        ),
        0x48 -> Seq(
          RegField.r(64, pause_count,
            RegFieldDesc("pauseCount", "Number of Pause packets emitted"))
        ),
        0x50 -> Seq(
          RegField.r(64, dropped_insns,
            RegFieldDesc("droppedInstructions",
              "Instructions retired inside groups discarded in lossy mode"))
        ),
        0x58 -> Seq(
          RegField(32, resumeWatermarkRead(_), resumeWatermarkWrite(_, _),
            RegFieldDesc("resumeWatermark",
              "Lossy resume watermark in queue entries; 0 = elaboration default. " +
              "Reads back the effective value after clamping."))
        ),
        0x5C -> Seq(
          RegField(32, dropThresholdRead(_), dropThresholdWrite(_, _),
            RegFieldDesc("dropThreshold",
              "Lossy drop threshold in queue entries; 0 = physical bound. " +
              "Lower values emulate a shallower queue. Reads back effective."))
        ),
        0x60 -> Seq(
          RegField(32, trace_sync_interval,
            RegFieldDesc("syncInterval",
              "Periodic sync interval in trace bytes; 0 = off. Write before enable."))
        )
      ):_*
    )
  }
}
