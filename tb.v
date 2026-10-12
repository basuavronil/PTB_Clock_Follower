`timescale 1ns/1ps
// ============================================================================
// Self-checking testbench for ptb_clock_follower (Verilog-2001)
//
// Leader model:
//   leader_time = local_time - leader_offset   (follower = leader + offset)
//   one-way path delay = path_delay clk cycles, symmetric
//
// Tests:
//   1. Reset state
//   2. OAM config, lock acquisition, offset/delay/PTB_clk accuracy
//   3. Message counters
//   4. Bad / unexpected DlyRply handling (drop counter, recovery)
//   5. Leader time step -> lock loss -> re-lock
//   6. Root mode (PTB == local, status forced)
//   7. Counter clear
//
// Offsets are checked with +/-TOL cycles of tolerance because the TB drives
// headers with a small fixed alignment skew relative to the DUT's sampling.
// ============================================================================
module tb_ptb_clock_follower;

  parameter TS_W       = 64;
  parameter SEQ_W      = 16;
  parameter CLK_PERIOD = 10;
  parameter TOL        = 2;     // cycles
  parameter GAP        = 80;    // idle cycles between exchanges

  // --------------------------------------------------------------- clock/rst
  reg clk   = 1'b0;
  reg rst_n = 1'b0;
  always #(CLK_PERIOD/2) clk = ~clk;

  // Free-running local time reference
  reg [TS_W-1:0] local_time;
  always @(posedge clk or negedge rst_n)
    if (!rst_n) local_time <= 64'h0000_0000_0001_0000;
    else        local_time <= local_time + 1'b1;

  // --------------------------------------------------------------- DUT I/O
  reg                    follow_valid;
  reg  [TS_W-1:0]        follow_t1;
  reg                    dlyrply_valid;
  reg  [SEQ_W-1:0]       dlyrply_seq;
  reg  [TS_W-1:0]        dlyrply_t4;
  reg                    oam_valid;
  reg                    oam_root;
  reg                    oam_enable;
  reg  signed [TS_W-1:0] oam_correction;
  reg  [47:0]            oam_daily_period;
  reg  [TS_W-1:0]        oam_lock_thresh;
  reg  [TS_W-1:0]        oam_unlock_thresh;
  reg  [7:0]             oam_lock_count;
  reg                    oam_cnt_clear;

  wire                   daily_req_valid;
  wire [SEQ_W-1:0]       daily_req_seq;
  wire                   ptb_oam_dly_valid;
  wire [TS_W-1:0]        ptb_oam_dly;
  wire                   status_lock;
  wire signed [TS_W-1:0] status_offset;
  wire signed [TS_W-1:0] status_delay;
  wire [TS_W-1:0]        ptb_clk;
  wire [159:0]           ext_ptb_status;

  ptb_clock_follower #(.TS_W(TS_W), .SEQ_W(SEQ_W)) dut (
    .clk               (clk),
    .rst_n             (rst_n),
    .local_time        (local_time),
    .follow_valid      (follow_valid),
    .follow_t1         (follow_t1),
    .dlyrply_valid     (dlyrply_valid),
    .dlyrply_seq       (dlyrply_seq),
    .dlyrply_t4        (dlyrply_t4),
    .oam_valid         (oam_valid),
    .oam_root          (oam_root),
    .oam_enable        (oam_enable),
    .oam_correction    (oam_correction),
    .oam_daily_period  (oam_daily_period),
    .oam_lock_thresh   (oam_lock_thresh),
    .oam_unlock_thresh (oam_unlock_thresh),
    .oam_lock_count    (oam_lock_count),
    .oam_cnt_clear     (oam_cnt_clear),
    .daily_req_valid   (daily_req_valid),
    .daily_req_seq     (daily_req_seq),
    .ptb_oam_dly_valid (ptb_oam_dly_valid),
    .ptb_oam_dly       (ptb_oam_dly),
    .status_lock       (status_lock),
    .status_offset     (status_offset),
    .status_delay      (status_delay),
    .ptb_clk           (ptb_clk),
    .ext_ptb_status    (ext_ptb_status)
  );

  // ExtendedPTBstatus fields
  wire [31:0] x_follow  = ext_ptb_status[159:128];
  wire [31:0] x_dlyrply = ext_ptb_status[127:96];
  wire [31:0] x_oam     = ext_ptb_status[95:64];
  wire [31:0] x_req     = ext_ptb_status[63:32];
  wire [15:0] x_drop    = ext_ptb_status[31:16];
  wire [15:0] x_lockloss= ext_ptb_status[15:0];

  // --------------------------------------------------------------- leader model
  reg signed [TS_W-1:0] leader_offset;
  integer               path_delay;

  // Monitor the DUT's Daily Request output
  reg                req_seen;
  reg [SEQ_W-1:0]    req_seq_seen;
  reg [TS_W-1:0]     req_t3;
  integer            n_req_tb;
  reg                oam_dly_seen;
  reg [TS_W-1:0]     oam_dly_val;

  always @(posedge clk) begin
    if (daily_req_valid) begin
      req_seen     = 1'b1;
      req_seq_seen = daily_req_seq;
      req_t3       = dut.t3_a;      // local TX stamp of the request
      n_req_tb     = n_req_tb + 1;
    end
    if (ptb_oam_dly_valid) begin
      oam_dly_seen = 1'b1;
      oam_dly_val  = ptb_oam_dly;
    end
  end

  // --------------------------------------------------------------- bookkeeping
  integer errors;
  integer n_follow, n_rply, n_oam, n_drop_exp, n_lockloss_exp;

  task check_eq(input [8*44-1:0] name, input [63:0] got, input [63:0] exp);
    begin
      if (got !== exp) begin
        errors = errors + 1;
        $display("[%0t] FAIL  %0s : got %0d, expected %0d", $time, name, got, exp);
      end else
        $display("[%0t] PASS  %0s : %0d", $time, name, got);
    end
  endtask

  task check_near(input [8*44-1:0] name, input signed [63:0] got,
                  input signed [63:0] exp, input signed [63:0] tol);
    reg signed [63:0] d;
    begin
      d = got - exp;
      if (d < 0) d = -d;
      if (d > tol) begin
        errors = errors + 1;
        $display("[%0t] FAIL  %0s : got %0d, expected %0d +/- %0d",
                 $time, name, got, exp, tol);
      end else
        $display("[%0t] PASS  %0s : got %0d (expected %0d +/- %0d)",
                 $time, name, got, exp, tol);
    end
  endtask

  // --------------------------------------------------------------- stimulus tasks
  // OAM header carrying configuration
  task cfg_oam(input root, input en, input signed [TS_W-1:0] corr,
               input [47:0] period, input [TS_W-1:0] lock_th,
               input [TS_W-1:0] unlock_th, input [7:0] lock_cnt);
    begin
      @(posedge clk); #1;
      oam_valid         = 1'b1;
      oam_root          = root;
      oam_enable        = en;
      oam_correction    = corr;
      oam_daily_period  = period;
      oam_lock_thresh   = lock_th;
      oam_unlock_thresh = unlock_th;
      oam_lock_count    = lock_cnt;
      @(posedge clk); #1;
      oam_valid = 1'b0;
      n_oam = n_oam + 1;
    end
  endtask

  // Leader sends Follow: t1 stamped now, header arrives path_delay later
  task send_follow;
    reg [TS_W-1:0] t1;
    begin
      t1 = local_time - leader_offset;
      repeat (path_delay) @(posedge clk);
      #1;
      follow_t1    = t1;
      follow_valid = 1'b1;
      @(posedge clk); #1;
      follow_valid = 1'b0;
      n_follow = n_follow + 1;
    end
  endtask

  // Leader answers a delay request that was transmitted at local time t3v
  task send_dlyrply(input [SEQ_W-1:0] seq, input [TS_W-1:0] t3v);
    reg [TS_W-1:0] t4;
    begin
      t4 = t3v + path_delay - leader_offset;   // leader-clock RX time
      repeat (path_delay) @(posedge clk);
      #1;
      dlyrply_seq   = seq;
      dlyrply_t4    = t4;
      dlyrply_valid = 1'b1;
      @(posedge clk); #1;
      dlyrply_valid = 1'b0;
      n_rply = n_rply + 1;
    end
  endtask

  // One full Follow / DelayReq / DlyRply exchange
  task do_exchange;
    begin
      req_seen = 1'b0;
      send_follow;
      repeat (4) @(posedge clk);
      if (req_seen)
        send_dlyrply(req_seq_seen, req_t3);
      else
        $display("[%0t] NOTE  no Daily Request issued for this Follow", $time);
      repeat (GAP) @(posedge clk);
    end
  endtask

  task run_until_lock(input integer max_n);
    integer i;
    begin
      i = 0;
      while (!status_lock && i < max_n) begin
        do_exchange;
        i = i + 1;
      end
      $display("[%0t] INFO  lock after %0d exchange(s)", $time, i);
    end
  endtask

  task check_counters;
    begin
      check_eq("ext: Follow count",   x_follow,   n_follow);
      check_eq("ext: DlyRply count",  x_dlyrply,  n_rply);
      check_eq("ext: OAM count",      x_oam,      n_oam);
      check_eq("ext: Daily Req count",x_req,      n_req_tb);
      check_eq("ext: dropped replies",x_drop,     n_drop_exp);
      check_eq("ext: lock-loss count",x_lockloss, n_lockloss_exp);
    end
  endtask

  // --------------------------------------------------------------- main test
  reg signed [TS_W-1:0] exp_ptb;

  initial begin
    $dumpfile("tb_ptb_clock_follower.vcd");
    $dumpvars(0, tb_ptb_clock_follower);

    errors = 0; n_follow = 0; n_rply = 0; n_oam = 0;
    n_drop_exp = 0; n_lockloss_exp = 0; n_req_tb = 0;
    req_seen = 0; req_seq_seen = 0; req_t3 = 0;
    oam_dly_seen = 0; oam_dly_val = 0;
    follow_valid = 0; follow_t1 = 0;
    dlyrply_valid = 0; dlyrply_seq = 0; dlyrply_t4 = 0;
    oam_valid = 0; oam_root = 0; oam_enable = 1; oam_correction = 0;
    oam_daily_period = 48'd20; oam_lock_thresh = 4; oam_unlock_thresh = 16;
    oam_lock_count = 3; oam_cnt_clear = 0;
    leader_offset = 500;
    path_delay    = 5;

    // ---- Test 1: reset ----------------------------------------------------
    $display("\n=== Test 1: reset state ===");
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (5) @(posedge clk);
    check_eq("status_lock after reset", status_lock, 0);
    check_eq("ext_ptb_status after reset", ext_ptb_status, 0);

    // ---- Test 2: lock acquisition and accuracy --------------------------------
    $display("\n=== Test 2: lock acquisition, offset 500, delay 5 ===");
    cfg_oam(1'b0, 1'b1, 0, 48'd20, 4, 16, 8'd3);
    run_until_lock(12);
    check_eq("status_lock", status_lock, 1);
    check_near("status_offset", status_offset, 500, TOL);
    check_near("status_delay",  status_delay,  5,   TOL);
    check_near("PTBoamDly (t4-t3 = delay-offset)", $signed(oam_dly_val), -495, TOL);
    do_exchange;
    @(posedge clk); #1;
    exp_ptb = $signed(local_time) - leader_offset;
    check_near("PTB_clk tracks leader time", $signed(ptb_clk), exp_ptb, TOL + 1);

    // ---- Test 3: counters ---------------------------------------------------
    $display("\n=== Test 3: message counters ===");
    check_counters;

    // ---- Test 4: bad / unexpected replies --------------------------------------
    $display("\n=== Test 4: bad and unexpected DlyRply ===");
    // (a) reply with nothing outstanding
    send_dlyrply(16'h1234, 64'd0);
    n_drop_exp = n_drop_exp + 1;
    repeat (10) @(posedge clk);
    // (b) wrong sequence number, then the correct one is still accepted
    req_seen = 1'b0;
    send_follow;
    repeat (4) @(posedge clk);
    send_dlyrply(req_seq_seen ^ 16'h00FF, req_t3);
    n_drop_exp = n_drop_exp + 1;
    repeat (10) @(posedge clk);
    send_dlyrply(req_seq_seen, req_t3);
    repeat (GAP) @(posedge clk);
    check_counters;
    check_eq("still locked after bad replies", status_lock, 1);
    check_near("offset after recovery", status_offset, 500, TOL);

    // ---- Test 5: leader time step -> lock loss -> re-lock -----------------------
    $display("\n=== Test 5: leader step +100 -> lock loss -> re-lock ===");
    leader_offset = leader_offset + 100;     // leader jumps back by 100
    do_exchange;
    n_lockloss_exp = n_lockloss_exp + 1;
    check_eq("lock lost after step", status_lock, 0);
    check_eq("ext: lock-loss count", x_lockloss, n_lockloss_exp);
    run_until_lock(12);
    check_eq("re-locked", status_lock, 1);
    check_near("offset after step", status_offset, 600, TOL);
    check_counters;

    // ---- Test 6: root mode ---------------------------------------------------
    $display("\n=== Test 6: root mode ===");
    cfg_oam(1'b1, 1'b1, 0, 48'd20, 4, 16, 8'd3);
    repeat (6) @(posedge clk); #1;
    check_eq("root: status_lock", status_lock, 1);
    check_eq("root: status_offset", status_offset, 0);
    check_eq("root: status_delay",  status_delay,  0);
    check_near("root: PTB_clk == local time", $signed(ptb_clk),
               $signed(local_time), TOL + 1);

    // ---- Test 7: counter clear ------------------------------------------------
    $display("\n=== Test 7: counter clear ===");
    @(posedge clk); #1; oam_cnt_clear = 1'b1;
    @(posedge clk); #1; oam_cnt_clear = 1'b0;
    repeat (2) @(posedge clk);
    check_eq("ext_ptb_status cleared", ext_ptb_status, 0);

    // ---- Summary ------------------------------------------------------------------
    $display("\n==============================================");
    if (errors == 0) $display(" ALL TESTS PASSED");
    else             $display(" %0d CHECK(S) FAILED", errors);
    $display("==============================================\n");
    $finish;
  end

  // Watchdog
  initial begin
    #20_000_000;
    $display("TIMEOUT: testbench did not finish");
    $finish;
  end

endmodule
