// ============================================================================
// PTB Clock Follower  (Verilog-2001)
// Blocks follow the micro-architecture diagram:
//   Stamp Capture (Follow) -> Stamp Capture (DlyRply) -> Offset Calculation
//   -> Lock Detect -> PTB Clock Counter -> Message Counters
//
//   t1 = leader TX time of Follow        (PTB_Follow header)
//   t2 = local RX time of Follow         (local time ref)
//   t3 = local TX time of delay request  (captured when Daily Request fires)
//   t4 = leader RX time of delay request (PTB_DlyRply header)
//
//   offset = ((t2 - t1) - (t4 - t3)) / 2     (follower minus leader)
//   delay  = ((t2 - t1) + (t4 - t3)) / 2
//   PTB time = local time - offset
//
// ASSUMPTIONS: 1-cycle header-valid strobes aligned with local_time,
// symmetric path delay, single clock domain, OAM header carries config.
// ============================================================================

// ----------------------------------------------------------------------------
// Stamp Capture #1 : PTB_Follow -> (t1,t2); issues Daily Request, stamps t3
// ----------------------------------------------------------------------------
module ptb_stamp_capture_follow #(
  parameter TS_W  = 64,
  parameter SEQ_W = 16
)(
  input                  clk,
  input                  rst_n,
  input  [TS_W-1:0]      local_time,
  // PTB_Follow header
  input                  follow_valid,
  input  [TS_W-1:0]      follow_t1,
  // Daily request control
  input  [47:0]          daily_period,
  // Outputs
  output reg             stamp1_valid,
  output reg [TS_W-1:0]  t1,
  output reg [TS_W-1:0]  t2,
  output reg             daily_req_valid,
  output reg [SEQ_W-1:0] daily_req_seq,
  output reg [TS_W-1:0]  t3
);
  reg [47:0] daily_cnt;
  reg        req_pending;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      daily_cnt       <= 48'd0;
      req_pending     <= 1'b1;           // request after first Follow
      stamp1_valid    <= 1'b0;
      t1              <= {TS_W{1'b0}};
      t2              <= {TS_W{1'b0}};
      t3              <= {TS_W{1'b0}};
      daily_req_valid <= 1'b0;
      daily_req_seq   <= {SEQ_W{1'b0}};
    end else begin
      stamp1_valid    <= 1'b0;
      daily_req_valid <= 1'b0;

      // Daily timer
      if (daily_cnt >= daily_period) begin
        daily_cnt   <= 48'd0;
        req_pending <= 1'b1;
      end else begin
        daily_cnt <= daily_cnt + 48'd1;
      end

      // Capture on Follow; issue the delay request when due
      if (follow_valid) begin
        t1           <= follow_t1;
        t2           <= local_time;
        stamp1_valid <= 1'b1;
        if (req_pending) begin
          daily_req_valid <= 1'b1;
          daily_req_seq   <= daily_req_seq + 1'b1;
          t3              <= local_time;
          req_pending     <= 1'b0;
        end
      end
    end
  end
endmodule

// ----------------------------------------------------------------------------
// Stamp Capture #2 : PTB_DlyRply -> t4, pairs with t1..t3
// ----------------------------------------------------------------------------
module ptb_stamp_capture_dly #(
  parameter TS_W  = 64,
  parameter SEQ_W = 16
)(
  input                  clk,
  input                  rst_n,
  // From stamp capture #1
  input                  daily_req_valid,
  input  [SEQ_W-1:0]     daily_req_seq,
  input  [TS_W-1:0]      t1_in,
  input  [TS_W-1:0]      t2_in,
  input  [TS_W-1:0]      t3_in,
  // PTB_DlyRply header
  input                  dlyrply_valid,
  input  [SEQ_W-1:0]     dlyrply_seq,
  input  [TS_W-1:0]      dlyrply_t4,
  // Outputs
  output reg             stamps_valid,
  output reg [TS_W-1:0]  t1,
  output reg [TS_W-1:0]  t2,
  output reg [TS_W-1:0]  t3,
  output reg [TS_W-1:0]  t4,
  output reg             dlyrply_drop,
  output reg             ptb_oam_dly_valid,   // PTBoamDly
  output reg [TS_W-1:0]  ptb_oam_dly          // raw t4 - t3
);
  reg                outstanding;
  reg [SEQ_W-1:0]    exp_seq;
  reg [TS_W-1:0]     t1_q, t2_q, t3_q;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      outstanding       <= 1'b0;
      exp_seq           <= {SEQ_W{1'b0}};
      t1_q              <= {TS_W{1'b0}};
      t2_q              <= {TS_W{1'b0}};
      t3_q              <= {TS_W{1'b0}};
      stamps_valid      <= 1'b0;
      t1                <= {TS_W{1'b0}};
      t2                <= {TS_W{1'b0}};
      t3                <= {TS_W{1'b0}};
      t4                <= {TS_W{1'b0}};
      dlyrply_drop      <= 1'b0;
      ptb_oam_dly_valid <= 1'b0;
      ptb_oam_dly       <= {TS_W{1'b0}};
    end else begin
      stamps_valid      <= 1'b0;
      dlyrply_drop      <= 1'b0;
      ptb_oam_dly_valid <= 1'b0;

      if (daily_req_valid) begin
        outstanding <= 1'b1;
        exp_seq     <= daily_req_seq;
        t1_q        <= t1_in;
        t2_q        <= t2_in;
        t3_q        <= t3_in;
      end

      if (dlyrply_valid) begin
        if (outstanding && (dlyrply_seq == exp_seq)) begin
          outstanding       <= 1'b0;
          t1                <= t1_q;
          t2                <= t2_q;
          t3                <= t3_q;
          t4                <= dlyrply_t4;
          stamps_valid      <= 1'b1;
          ptb_oam_dly       <= dlyrply_t4 - t3_q;
          ptb_oam_dly_valid <= 1'b1;
        end else begin
          dlyrply_drop <= 1'b1;
        end
      end
    end
  end
endmodule

// ----------------------------------------------------------------------------
// Offset Calculation
// ----------------------------------------------------------------------------
module ptb_offset_calc #(
  parameter TS_W = 64
)(
  input                        clk,
  input                        rst_n,
  input                        stamps_valid,
  input  [TS_W-1:0]            t1,
  input  [TS_W-1:0]            t2,
  input  [TS_W-1:0]            t3,
  input  [TS_W-1:0]            t4,
  input  signed [TS_W-1:0]     oam_correction,
  output reg                   calc_valid,
  output reg signed [TS_W-1:0] offset,
  output reg signed [TS_W-1:0] delay
);
  // Extra guard bits so the intermediate sums cannot overflow
  wire signed [TS_W+2:0] ms = $signed({3'b000, t2}) - $signed({3'b000, t1});
  wire signed [TS_W+2:0] sm = $signed({3'b000, t4}) - $signed({3'b000, t3});

  wire signed [TS_W+2:0] off_full = (ms - sm) >>> 1;
  wire signed [TS_W+2:0] dly_full = (ms + sm) >>> 1;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      calc_valid <= 1'b0;
      offset     <= {TS_W{1'b0}};
      delay      <= {TS_W{1'b0}};
    end else begin
      calc_valid <= stamps_valid;
      if (stamps_valid) begin
        offset <= off_full[TS_W-1:0] - oam_correction;
        delay  <= dly_full[TS_W-1:0];
      end
    end
  end
endmodule

// ----------------------------------------------------------------------------
// Lock Detect (hysteresis: N good in a row to lock, one bad to unlock)
// ----------------------------------------------------------------------------
module ptb_lock_detect #(
  parameter TS_W = 64
)(
  input                    clk,
  input                    rst_n,
  input                    calc_valid,
  input  signed [TS_W-1:0] offset,
  input  [TS_W-1:0]        lock_thresh,      // |offset| <= => good sample
  input  [TS_W-1:0]        unlock_thresh,    // |offset| >  => lose lock
  input  [7:0]             lock_count,       // consecutive good samples
  output reg               locked,
  output reg               lock_lost_pulse
);
  wire [TS_W-1:0] abs_off = offset[TS_W-1] ? (~offset + 1'b1) : offset;
  reg  [7:0]      good_cnt;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      locked          <= 1'b0;
      lock_lost_pulse <= 1'b0;
      good_cnt        <= 8'd0;
    end else begin
      lock_lost_pulse <= 1'b0;
      if (calc_valid) begin
        if (abs_off <= lock_thresh) begin
          if (good_cnt != 8'hFF)
            good_cnt <= good_cnt + 8'd1;
          if (({1'b0, good_cnt} + 9'd1) >= {1'b0, lock_count})
            locked <= 1'b1;
        end else begin
          good_cnt <= 8'd0;
          if (locked && (abs_off > unlock_thresh)) begin
            locked          <= 1'b0;
            lock_lost_pulse <= 1'b1;
          end
        end
      end
    end
  end
endmodule

// ----------------------------------------------------------------------------
// PTB Clock Counter : PTB time = local time - offset
// ----------------------------------------------------------------------------
module ptb_clock_counter #(
  parameter TS_W = 64
)(
  input                    clk,
  input                    rst_n,
  input  [TS_W-1:0]        local_time,
  input                    calc_valid,
  input  signed [TS_W-1:0] offset,
  input                    is_root,      // root keeps PTB == local time
  input                    enable,       // apply corrections
  output reg [TS_W-1:0]    ptb_time,     // PTB_clk
  output reg signed [TS_W-1:0] offset_applied  // correction currently in use
);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      offset_applied <= {TS_W{1'b0}};
      ptb_time       <= {TS_W{1'b0}};
    end else begin
      if (is_root)
        offset_applied <= {TS_W{1'b0}};
      else if (calc_valid && enable)
        offset_applied <= offset;

      ptb_time <= local_time - offset_applied;
    end
  end
endmodule

// ----------------------------------------------------------------------------
// Message Counters
//   ext_status = {follow[31:0], dlyrply[31:0], oam[31:0], req[31:0],
//                 drop[15:0], lock_loss[15:0]}   (160 bits)
// ----------------------------------------------------------------------------
module ptb_msg_counters (
  input          clk,
  input          rst_n,
  input          clear,
  input          follow_valid,
  input          dlyrply_valid,
  input          oam_valid,
  input          daily_req_valid,
  input          dlyrply_drop,
  input          lock_lost_pulse,
  output [159:0] ext_status
);
  reg [31:0] follow_cnt, dlyrply_cnt, oam_cnt, req_cnt;
  reg [15:0] drop_cnt, lock_loss_cnt;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      follow_cnt    <= 32'd0;
      dlyrply_cnt   <= 32'd0;
      oam_cnt       <= 32'd0;
      req_cnt       <= 32'd0;
      drop_cnt      <= 16'd0;
      lock_loss_cnt <= 16'd0;
    end else if (clear) begin
      follow_cnt    <= 32'd0;
      dlyrply_cnt   <= 32'd0;
      oam_cnt       <= 32'd0;
      req_cnt       <= 32'd0;
      drop_cnt      <= 16'd0;
      lock_loss_cnt <= 16'd0;
    end else begin
      if (follow_valid)    follow_cnt  <= follow_cnt  + 32'd1;
      if (dlyrply_valid)   dlyrply_cnt <= dlyrply_cnt + 32'd1;
      if (oam_valid)       oam_cnt     <= oam_cnt     + 32'd1;
      if (daily_req_valid) req_cnt     <= req_cnt     + 32'd1;
      if (dlyrply_drop    && drop_cnt      != 16'hFFFF)
        drop_cnt      <= drop_cnt + 16'd1;
      if (lock_lost_pulse && lock_loss_cnt != 16'hFFFF)
        lock_loss_cnt <= lock_loss_cnt + 16'd1;
    end
  end

  assign ext_status = {follow_cnt, dlyrply_cnt, oam_cnt, req_cnt,
                       drop_cnt, lock_loss_cnt};
endmodule

// ----------------------------------------------------------------------------
// Top : PTB Clock Follower
// ----------------------------------------------------------------------------
module ptb_clock_follower #(
  parameter TS_W  = 64,
  parameter SEQ_W = 16
)(
  input                        clk,
  input                        rst_n,

  // Local time ref
  input  [TS_W-1:0]            local_time,

  // PTB_Follow (header)
  input                        follow_valid,
  input  [TS_W-1:0]            follow_t1,

  // PTB_DlyRply (header)
  input                        dlyrply_valid,
  input  [SEQ_W-1:0]           dlyrply_seq,
  input  [TS_W-1:0]            dlyrply_t4,

  // PTB_OAM (OAM header, root + config)
  input                        oam_valid,
  input                        oam_root,
  input                        oam_enable,
  input  signed [TS_W-1:0]     oam_correction,
  input  [47:0]                oam_daily_period,
  input  [TS_W-1:0]            oam_lock_thresh,
  input  [TS_W-1:0]            oam_unlock_thresh,
  input  [7:0]                 oam_lock_count,
  input                        oam_cnt_clear,

  // Daily Request (to ReSync header)
  output                       daily_req_valid,
  output [SEQ_W-1:0]           daily_req_seq,

  // PTBoamDly
  output                       ptb_oam_dly_valid,
  output [TS_W-1:0]            ptb_oam_dly,

  // PTB_Status (lock, offset, delay)
  output                       status_lock,
  output signed [TS_W-1:0]     status_offset,
  output signed [TS_W-1:0]     status_delay,

  // PTB_clk (to local clk leaders)
  output [TS_W-1:0]            ptb_clk,

  // ExtendedPTBstatus (counter values)
  output [159:0]               ext_ptb_status
);

  // ---- Latched OAM configuration (updates only on an OAM header) ----------
  reg                    cfg_root, cfg_enable;
  reg signed [TS_W-1:0]  cfg_corr;
  reg [47:0]             cfg_period;
  reg [TS_W-1:0]         cfg_lock_th, cfg_unlock_th;
  reg [7:0]              cfg_lock_cnt;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cfg_root      <= 1'b0;
      cfg_enable    <= 1'b1;
      cfg_corr      <= {TS_W{1'b0}};
      cfg_period    <= 48'd86_400_000_000;  // placeholder: set per clk freq
      cfg_lock_th   <= 16;
      cfg_unlock_th <= 64;
      cfg_lock_cnt  <= 8'd4;
    end else if (oam_valid) begin
      cfg_root      <= oam_root;
      cfg_enable    <= oam_enable;
      cfg_corr      <= oam_correction;
      cfg_period    <= oam_daily_period;
      cfg_lock_th   <= oam_lock_thresh;
      cfg_unlock_th <= oam_unlock_thresh;
      cfg_lock_cnt  <= oam_lock_count;
    end
  end

  // ---- Stamp capture #1 ---------------------------------------------------
  wire              s1_valid;
  wire [TS_W-1:0]   t1_a, t2_a, t3_a;

  ptb_stamp_capture_follow #(.TS_W(TS_W), .SEQ_W(SEQ_W)) u_cap1 (
    .clk             (clk),
    .rst_n           (rst_n),
    .local_time      (local_time),
    .follow_valid    (follow_valid),
    .follow_t1       (follow_t1),
    .daily_period    (cfg_period),
    .stamp1_valid    (s1_valid),
    .t1              (t1_a),
    .t2              (t2_a),
    .daily_req_valid (daily_req_valid),
    .daily_req_seq   (daily_req_seq),
    .t3              (t3_a)
  );

  // ---- Stamp capture #2 ---------------------------------------------------
  wire              stamps_valid, dlyrply_drop;
  wire [TS_W-1:0]   t1, t2, t3, t4;

  ptb_stamp_capture_dly #(.TS_W(TS_W), .SEQ_W(SEQ_W)) u_cap2 (
    .clk               (clk),
    .rst_n             (rst_n),
    .daily_req_valid   (daily_req_valid),
    .daily_req_seq     (daily_req_seq),
    .t1_in             (t1_a),
    .t2_in             (t2_a),
    .t3_in             (t3_a),
    .dlyrply_valid     (dlyrply_valid),
    .dlyrply_seq       (dlyrply_seq),
    .dlyrply_t4        (dlyrply_t4),
    .stamps_valid      (stamps_valid),
    .t1                (t1),
    .t2                (t2),
    .t3                (t3),
    .t4                (t4),
    .dlyrply_drop      (dlyrply_drop),
    .ptb_oam_dly_valid (ptb_oam_dly_valid),
    .ptb_oam_dly       (ptb_oam_dly)
  );

  // ---- Offset calculation -------------------------------------------------
  wire                   calc_valid;
  wire signed [TS_W-1:0] offset, delay;

  ptb_offset_calc #(.TS_W(TS_W)) u_off (
    .clk            (clk),
    .rst_n          (rst_n),
    .stamps_valid   (stamps_valid),
    .t1             (t1),
    .t2             (t2),
    .t3             (t3),
    .t4             (t4),
    .oam_correction (cfg_corr),
    .calc_valid     (calc_valid),
    .offset         (offset),
    .delay          (delay)
  );

  // ---- Lock detect --------------------------------------------------------
  // Lock is judged on the residual error: new measurement minus the correction
  // currently applied (offset_applied still holds the old value at calc_valid).
  wire locked, lock_lost;
  wire signed [TS_W-1:0] applied;
  wire signed [TS_W-1:0] residual = offset - applied;

  ptb_lock_detect #(.TS_W(TS_W)) u_lock (
    .clk             (clk),
    .rst_n           (rst_n),
    .calc_valid      (calc_valid),
    .offset          (residual),
    .lock_thresh     (cfg_lock_th),
    .unlock_thresh   (cfg_unlock_th),
    .lock_count      (cfg_lock_cnt),
    .locked          (locked),
    .lock_lost_pulse (lock_lost)
  );

  // ---- PTB clock counter --------------------------------------------------
  ptb_clock_counter #(.TS_W(TS_W)) u_clk (
    .clk        (clk),
    .rst_n      (rst_n),
    .local_time (local_time),
    .calc_valid (calc_valid),
    .offset     (offset),
    .is_root    (cfg_root),
    .enable     (cfg_enable),
    .ptb_time   (ptb_clk),
    .offset_applied (applied)
  );

  // ---- Message counters ---------------------------------------------------
  ptb_msg_counters u_cnt (
    .clk             (clk),
    .rst_n           (rst_n),
    .clear           (oam_cnt_clear),
    .follow_valid    (follow_valid),
    .dlyrply_valid   (dlyrply_valid),
    .oam_valid       (oam_valid),
    .daily_req_valid (daily_req_valid),
    .dlyrply_drop    (dlyrply_drop),
    .lock_lost_pulse (lock_lost),
    .ext_status      (ext_ptb_status)
  );

  // ---- PTB_Status: root is always locked with zero offset/delay -----------
  assign status_lock   = cfg_root ? 1'b1 : locked;
  assign status_offset = cfg_root ? {TS_W{1'b0}} : offset;
  assign status_delay  = cfg_root ? {TS_W{1'b0}} : delay;

endmodule
