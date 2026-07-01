// 1. DESIGN - sync_fifo
module sync_fifo #(
  parameter DATA_WIDTH = 8,
  parameter DEPTH      = 16,
  parameter ADDR_WIDTH = $clog2(DEPTH)
)(
  input  logic                  clk,
  input  logic                  rst,
  input  logic                  wr_en,
  input  logic                  rd_en,
  input  logic [DATA_WIDTH-1:0] data_in,
  output logic [DATA_WIDTH-1:0] data_out,
  output logic                  full,
  output logic                  empty
);
  logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];
  logic [ADDR_WIDTH:0]   wr_ptr;
  logic [ADDR_WIDTH:0]   rd_ptr;
  always_ff @(posedge clk or posedge rst) begin
    if (rst) wr_ptr <= 0;
    else if (wr_en && !full) begin
      mem[wr_ptr[ADDR_WIDTH-1:0]] <= data_in;
      wr_ptr <= wr_ptr + 1;
    end
  end
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      rd_ptr   <= 0;
      data_out <= 0;
    end else if (rd_en && !empty) begin
      data_out <= mem[rd_ptr[ADDR_WIDTH-1:0]];
      rd_ptr   <= rd_ptr + 1;
    end
  end
  assign empty = (wr_ptr == rd_ptr);
  assign full  = (wr_ptr[ADDR_WIDTH] != rd_ptr[ADDR_WIDTH]) &&
                 (wr_ptr[ADDR_WIDTH-1:0] == rd_ptr[ADDR_WIDTH-1:0]);
endmodule
// 2. INTERFACE - fifo_if
interface fifo_if (input logic clk);
  logic       rst;
  logic       wr_en;
  logic       rd_en;
  logic [7:0] data_in;
  logic [7:0] data_out;
  logic       full;
  logic       empty;
  clocking driver_cb @(posedge clk);
    default input #1 output #1;
    output wr_en, rd_en, data_in;
    input  rst, data_out, full, empty;
  endclocking
  clocking monitor_cb @(posedge clk);
    default input #1;
    input rst, wr_en, rd_en, data_in, data_out, full, empty;
  endclocking
  // No modports: driver and monitor use plain 'virtual fifo_if'
  // so direct signal access (vif.rst, vif.wr_en) and clocking block
  // access (vif.driver_cb.wr_en) both work without restriction.
endinterface
// 3. TRANSACTION - fifo_transaction
class fifo_transaction;
  typedef enum logic [1:0] {
    WRITE = 2'b00,
    READ  = 2'b01,
    IDLE  = 2'b10
  } op_e;
  rand op_e      operation;
  rand bit [7:0] data;
  // Bias: 45% write, 45% read, 10% idle
  constraint op_dist {
    operation dist { WRITE := 45, READ := 45, IDLE := 10 };
  }
  function fifo_transaction copy();
    fifo_transaction t = new();
    t.operation = this.operation;
    t.data      = this.data;
    return t;
  endfunction
  function void display(string tag = "TXN");
    $display("[%s] op=%-5s  data=0x%02h  @%0t",
             tag, operation.name(), data, $time);
  endfunction
endclass
// 4. GENERATOR - fifo_generator
class fifo_generator;
  fifo_transaction            trans;
  mailbox #(fifo_transaction) gen2drv;
  int unsigned                num_transactions;
  event                       gen_done;   // signals completion
  function new(mailbox #(fifo_transaction) mbx,
               int unsigned n,
               ref event done);
    gen2drv          = mbx;
    num_transactions = n;
    gen_done         = done;
  endfunction
  task run();
    repeat (num_transactions) begin
      trans = new();
      if (!trans.randomize())
        $fatal(1, "[GENERATOR] Randomization failed");
      trans.display("GEN");
      gen2drv.put(trans.copy());
    end
    $display("[GENERATOR] Done - %0d transactions queued", num_transactions);
    -> gen_done;   // notify env generator is finished
  endtask
endclass
// 5. DRIVER - fifo_driver
class fifo_driver;

  virtual fifo_if             vif;
  mailbox #(fifo_transaction) gen2drv;
  function new(virtual fifo_if vif,
               mailbox #(fifo_transaction) mbx);
    this.vif     = vif;
    this.gen2drv = mbx;
  endfunction
  // FIX: drive rst directly (not through clocking block) so it
  //      asserts before the first clock edge
  task reset();
    vif.rst   = 1;      // direct assignment - combinational, immediate
    vif.wr_en = 0;
    vif.rd_en = 0;
    vif.data_in = 0;
    repeat (4) @(posedge vif.clk);  // hold reset for 4 cycles
    @(negedge vif.clk);             // deassert on falling edge (glitch-free)
    vif.rst = 0;
    $display("[DRIVER] Reset deasserted @%0t", $time);
    @(posedge vif.clk);             // one idle cycle before traffic
  endtask
  task run();
    fifo_transaction trans;
    forever begin
      if (gen2drv.num() == 0) begin
        // idle: deassert enables, wait for next transaction
        @(vif.driver_cb);
        vif.driver_cb.wr_en <= 0;
        vif.driver_cb.rd_en <= 0;
      end else begin
        gen2drv.get(trans);
        @(vif.driver_cb);
        case (trans.operation)
          fifo_transaction::WRITE: begin
            if (!vif.driver_cb.full) begin
              vif.driver_cb.wr_en   <= 1;
              vif.driver_cb.rd_en   <= 0;
              vif.driver_cb.data_in <= trans.data;
              trans.display("DRV-WR");
            end else begin
              $display("[DRIVER] FIFO full, holding @%0t", $time);
              vif.driver_cb.wr_en <= 0;
              vif.driver_cb.rd_en <= 0;
            end
          end
          fifo_transaction::READ: begin
            if (!vif.driver_cb.empty) begin
              vif.driver_cb.rd_en <= 1;
              vif.driver_cb.wr_en <= 0;
              trans.display("DRV-RD");
            end else begin
              $display("[DRIVER] FIFO empty, holding @%0t", $time);
              vif.driver_cb.wr_en <= 0;
              vif.driver_cb.rd_en <= 0;
            end
          end
          default: begin
            vif.driver_cb.wr_en <= 0;
            vif.driver_cb.rd_en <= 0;
          end
        endcase
      end
    end
  endtask
endclass
// 6. MONITOR - fifo_monitor
class fifo_monitor;
  virtual fifo_if             vif;
  mailbox #(fifo_transaction) mon2scb;
  // Pending read flag: data_out valid ONE cycle after rd_en (registered read)
  bit pending_read;
  function new(virtual fifo_if vif,
               mailbox #(fifo_transaction) mbx);
    this.vif         = vif;
    this.mon2scb     = mbx;
    this.pending_read = 0;
  endfunction
  task run();
    fifo_transaction trans;
    forever begin
      @(vif.monitor_cb);
      // ---- Capture delayed read data (from previous cycle's rd_en) ----
      if (pending_read) begin
        trans           = new();
        trans.operation = fifo_transaction::READ;
        trans.data      = vif.monitor_cb.data_out;  // valid NOW
        trans.display("MON-RD");
        mon2scb.put(trans.copy());
        pending_read = 0;
      end
      // ---- Observe WRITE (data registered on this edge) ----
      if (vif.monitor_cb.wr_en && !vif.monitor_cb.full &&
          !vif.monitor_cb.rst) begin
        trans           = new();
        trans.operation = fifo_transaction::WRITE;
        trans.data      = vif.monitor_cb.data_in;
        trans.display("MON-WR");
        mon2scb.put(trans.copy());
      end
      // ---- Observe READ strobe (set flag; capture data next cycle) ----
      if (vif.monitor_cb.rd_en && !vif.monitor_cb.empty &&
          !vif.monitor_cb.rst) begin
        pending_read = 1;
      end
    end
  endtask
endclass
// 7. SCOREBOARD - fifo_scoreboard
class fifo_scoreboard;
  mailbox #(fifo_transaction) mon2scb;
  bit [7:0]    ref_fifo[$];
  int unsigned checks_passed;
  int unsigned checks_failed;
  function new(mailbox #(fifo_transaction) mbx);
    mon2scb       = mbx;
    checks_passed = 0;
    checks_failed = 0;
  endfunction
  task run();
    fifo_transaction trans;
    forever begin
      mon2scb.get(trans);
      case (trans.operation)
        fifo_transaction::WRITE: begin
          ref_fifo.push_back(trans.data);
          $display("[SCB] PUSH 0x%02h | depth=%0d",
                   trans.data, ref_fifo.size());
        end
        fifo_transaction::READ: begin
          if (ref_fifo.size() == 0) begin
            $error("[SCB] FAIL - read from empty ref queue @%0t", $time);
            checks_failed++;
          end else begin
            bit [7:0] expected = ref_fifo.pop_front();
            if (trans.data === expected) begin
              $display("[SCB] PASS - got=0x%02h  exp=0x%02h", trans.data, expected);
              checks_passed++;
            end else begin
              $error("[SCB] FAIL - got=0x%02h  exp=0x%02h @%0t",
                     trans.data, expected, $time);
              checks_failed++;
            end
          end
        end
        default: ;
      endcase
    end
  endtask
  function void report();
    $display("==============================================");
    $display("  SCOREBOARD REPORT");
    $display("  PASSED : %0d", checks_passed);
    $display("  FAILED : %0d", checks_failed);
    if (checks_failed == 0)
      $display("  RESULT : ** ALL TESTS PASSED **");
    else
      $display("  RESULT : ** %0d TEST(S) FAILED **", checks_failed);
    $display("==============================================");
  endfunction
endclass
// 8. ENVIRONMENT - fifo_env
class fifo_env;
  fifo_generator  gen;
  fifo_driver     drv;
  fifo_monitor    mon;
  fifo_scoreboard scb;
  mailbox #(fifo_transaction) gen2drv;
  mailbox #(fifo_transaction) mon2scb;
  virtual fifo_if vif;
  event           gen_done;
  function new(virtual fifo_if vif, int unsigned num_transactions = 20);
    this.vif = vif;
    gen2drv  = new();
    mon2scb  = new();
    gen = new(gen2drv, num_transactions, gen_done);
    drv = new(vif, gen2drv);
    mon = new(vif, mon2scb);
    scb = new(mon2scb);
  endfunction
  task run();
    // FIX: reset BEFORE forking stimulus threads
    drv.reset();
    fork
      gen.run();
      drv.run();   // killed after drain below
      mon.run();   // killed after drain below
      scb.run();   // killed after drain below
    join_none
    // FIX: wait for generator to finish, then drain remaining
    //      transactions through driver -> monitor -> scoreboard
    @(gen_done);
    $display("[ENV] Generator done, draining pipeline...");
    // Wait until mailbox is empty AND extra cycles for pipeline flush
    wait (gen2drv.num() == 0);
    repeat (10) @(posedge vif.clk);  // flush registered-read pipeline
    // Wait for scoreboard to process all monitor outputs
    wait (mon2scb.num() == 0);
    repeat (5) @(posedge vif.clk);
    disable fork;  // cleanly kill forever loops
    scb.report();
  endtask
endclass
// 9. TEST - fifo_test
class fifo_test;
  fifo_env        env;
  virtual fifo_if vif;
  function new(virtual fifo_if vif, int unsigned num_transactions = 40);
    this.vif = vif;
    env = new(vif, num_transactions);
  endfunction
  task run();
    $display("============================================");
    $display("  [TEST] FIFO Verification Start");
    $display("============================================");
    env.run();
    $display("[TEST] Complete @%0t", $time);
  endtask
endclass
// 10. TESTBENCH TOP - tb_sync_fifo_top
module tb_sync_fifo_top;
  localparam DATA_WIDTH = 8;
  localparam DEPTH      = 16;
  // Clock
  logic clk;
  initial  clk = 0;
  always #5 clk = ~clk;   // 100 MHz
  // Interface
  fifo_if intf (.clk(clk));
  // DUT
  sync_fifo #(
    .DATA_WIDTH (DATA_WIDTH),
    .DEPTH      (DEPTH)
  ) dut (
    .clk      (clk),
    .rst      (intf.rst),
    .wr_en    (intf.wr_en),
    .rd_en    (intf.rd_en),
    .data_in  (intf.data_in),
    .data_out (intf.data_out),
    .full     (intf.full),
    .empty    (intf.empty)
  );
  // typedef fixes "token is ';'" error on parameterized class handle
  typedef fifo_test fifo_test_t;
  fifo_test_t test_h;
  initial begin
    test_h = new(intf, 40);   // 40 randomized transactions
    test_h.run();
    $finish;
  end
  initial begin
    $dumpfile("sync_fifo_full.vcd");
    $dumpvars(0, tb_sync_fifo_top);
  end
  // Watchdog
  initial begin
    #100000;
    $display("[WATCHDOG] Timeout - check for deadlock");
    $finish;
  end
endmodule
