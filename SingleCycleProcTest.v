`timescale 1ns / 1ps

`define STRLEN 32
`define HalfClockPeriod 60
`define ClockPeriod `HalfClockPeriod * 2

// ---------------------------------------------------------------------------
// Testbench for SingleCycleProc with a built-in reference model.
//
// On every instruction the checker (below) compares the processor against an
// independent model of what LDUR/STUR/ADD/SUB/AND/ORR/CBZ/B should do, in this
// order: PC, instruction, memory/registers from the previous instruction,
// control signals, register reads, immediate, ALU result, Zero, data memory
// read, write-back value, next PC. The first mismatch prints what was
// expected, which signal to look at, and stops the simulation.
// ---------------------------------------------------------------------------
module SingleCycleProcTest_v;

   parameter VERBOSE = 1;   // 1 = print one line per executed instruction

   initial
     begin
        $dumpfile("singlecycle.vcd");
        $dumpvars;
     end

   // These tasks are used to check if a given test has passed and
   // confirm that all tests passed.
   task passTest;
      input [63:0] actualOut, expectedOut;
      input [`STRLEN*8:0] testType;
      inout [7:0] 	  passed;

      if(actualOut == expectedOut) begin $display ("%s passed", testType); passed = passed + 1; end
      else $display ("%s failed: 0x%x should be 0x%x", testType, actualOut, expectedOut);
   endtask

   task allPassed;
      input [7:0] passed;
      input [7:0] numTests;

      if(passed == numTests) $display ("All tests passed");
      else $display("Some tests failed: %d of %d passed", passed, numTests);
   endtask

   // Inputs
   reg 		  CLK;
   reg		  Reset;
   reg [63:0] 	  startPC;
   reg [7:0] 	  passed;
   reg [15:0] 	  watchdog;

   // Outputs
   wire [63:0] 	  MemtoRegOut;
   wire [63:0] 	  currentPC;

   // Instantiate the Unit Under Test (UUT)
   SingleCycleProc uut (
		    .CLK(CLK),
		    .reset(Reset),
		    .startpc(startPC),
		    .currentpc(currentPC),
		    .MemtoRegOut(MemtoRegOut)
		    );

   // =======================================================================
   // Reference-model checker
   // =======================================================================
   reg           checking;      // set once reset is released
   reg           failed;        // set on the first mismatch
   reg [63:0]    m_pc;          // where the model says the PC should be
   reg [63:0]    m_reg [0:31];  // model register file (X = never written)
   reg           pend_st;       // a store whose result is checked next cycle
   reg [63:0]    pend_addr, pend_data;
   reg [8*8-1:0] iname;         // name of the instruction being checked
   integer       ncheck;        // instructions checked so far

   // Model register read: X31 (XZR) always reads 0
   function [63:0] mrd;
      input [4:0] r;
      begin
         if (r == 5'd31) mrd = 64'd0;
         else            mrd = m_reg[r];
      end
   endfunction

   // Read 8 bytes (big-endian) straight out of the DUT's data memory array
   function [63:0] memrd;
      input [63:0] a;
      integer ai;
      begin
         if ((^a === 1'bx) || (a > 64'd1016))
            memrd = 64'bx;
         else begin
            ai = a;
            memrd = {uut.dmem.memBank[ai],   uut.dmem.memBank[ai+1],
                     uut.dmem.memBank[ai+2], uut.dmem.memBank[ai+3],
                     uut.dmem.memBank[ai+4], uut.dmem.memBank[ai+5],
                     uut.dmem.memBank[ai+6], uut.dmem.memBank[ai+7]};
         end
      end
   endfunction

   // Report the problem and stop the simulation (only the first call acts)
   task fail;
      input [8*72-1:0] msg;
      begin
         if (!failed) begin
            failed = 1;
            $display("");
            $display("*** STOPPING at t=%0d ns ***", $time);
            $display("    instruction #%0d: PC=%h instr=%h (%0s)", ncheck, currentPC, uut.instruction, iname);
            $display("    reason: %0s", msg);
            $finish;
         end
      end
   endtask

   // 64-bit comparison with a hint about where to look
   task chk64;
      input [63:0]      act;
      input [63:0]      expv;
      input [8*72-1:0]  what;
      begin
         if (!failed && (act !== expv)) begin
            $display("*** MISMATCH");
            $display("    got      %h", act);
            $display("    expected %h", expv);
            fail(what);
         end
      end
   endtask

   // Compare up to 4 bits, ignoring bit positions where the expected value is X
   task ck_sig;
      input [3:0]      a;
      input [3:0]      e;
      input [8*12-1:0] nm;
      integer k;
      reg bad;
      begin
         bad = 0;
         for (k = 0; k < 4; k = k + 1)
            if (e[k] !== 1'bx && a[k] !== e[k]) bad = 1;
         if (bad && !failed) begin
            $display("*** control signal %0s = %b, expected %b (x = don't care)", nm, a, e);
            fail("Wrong control signal. Fix this opcode's case in SC_Control.");
         end
      end
   endtask

   // The program itself must not use a value that was never written
   task need_def;
      input [63:0]     v;
      input [8*72-1:0] msg;
      begin
         if (^v === 1'bx) fail(msg);
      end
   endtask

   task check_cycle;
      reg [31:0] ins;
      reg [10:0] op;
      reg [63:0] ra, rbr, rbt, rbsel, imm, ealu, ewb, enpc, addr, mval;
      reg        ezero, uses_a, uses_b, uses_imm, uses_alu, is_load, is_store, is_cbz, wr;
      reg [4:0]  fd;
      reg [1:0]  e_sign;
      reg [3:0]  e_aluop;
      reg        e_r2l, e_alusrc, e_m2r, e_rw, e_mr, e_mw, e_br, e_ubr;
      integer    ri;
      begin
         ins   = uut.instruction;
         op    = ins[31:21];
         iname = "?";

         // 1. PC must match the model
         if (^currentPC === 1'bx)
            fail("PC is X. Check reset/startpc and the NextPClogic output.");
         else if (currentPC !== m_pc) begin
            $display("*** PC is %h but the model says it should be %h", currentPC, m_pc);
            fail("PC differs from the model. The previous branch/NextPC was wrong.");
         end

         // 2. instruction must be a real instruction
         if (^ins === 1'bx)
            fail("Instruction is X. PC is outside the program in InstructionMemory.");

         // 3. result of the previous cycle's STUR
         if (pend_st) begin
            chk64(memrd(pend_addr), pend_data,
                  "STUR did not store the right value (MemWrite/Address/WriteData).");
            pend_st = 0;
         end

         // 4. register file must match the model after the previous instruction
         for (ri = 0; ri < 31; ri = ri + 1)
            if (!failed && (uut.regs.registers[ri] !== m_reg[ri])) begin
               $display("*** X%0d: DUT=%h model=%h", ri, uut.regs.registers[ri], m_reg[ri]);
               fail("Registers differ after the previous instruction (RegWrite/rd/MemtoReg).");
            end

         // 5. decode with the model and work out the expected values
         ra   = mrd(ins[9:5]);
         rbr  = mrd(ins[20:16]);
         rbt  = mrd(ins[4:0]);
         fd   = ins[4:0];
         enpc = m_pc + 64'd4;
         imm = 64'd0; ealu = 64'd0; ewb = 64'd0; addr = 64'd0; mval = 64'd0;
         rbsel = 64'd0; ezero = 1'b0;
         uses_a = 0; uses_b = 0; uses_imm = 0; uses_alu = 0;
         is_load = 0; is_store = 0; is_cbz = 0; wr = 0;
         e_sign = 2'b00; e_aluop = 4'b0000;
         e_r2l = 0; e_alusrc = 0; e_m2r = 0; e_rw = 0;
         e_mr = 0; e_mw = 0; e_br = 0; e_ubr = 0;

         casez (op)
            11'b11111000010: begin   // LDUR
               iname = "LDUR";
               imm  = {{55{ins[20]}}, ins[20:12]};
               ealu = ra + imm;  addr = ealu;
               mval = memrd(addr);  ewb = mval;
               uses_a = 1; uses_imm = 1; uses_alu = 1; is_load = 1; wr = 1;
               e_sign = 2'b01; e_aluop = 4'b0010; e_r2l = 1'bx; e_alusrc = 1;
               e_m2r = 1; e_rw = 1; e_mr = 1; e_mw = 0; e_br = 0; e_ubr = 0;
               need_def(ra,   "LDUR base register was never written by the program.");
               need_def(mval, "LDUR reads memory that was never initialized.");
            end
            11'b11111000000: begin   // STUR
               iname = "STUR";
               imm  = {{55{ins[20]}}, ins[20:12]};
               ealu = ra + imm;  addr = ealu;
               rbsel = rbt;
               uses_a = 1; uses_b = 1; uses_imm = 1; uses_alu = 1; is_store = 1;
               e_sign = 2'b01; e_aluop = 4'b0010; e_r2l = 1; e_alusrc = 1;
               e_m2r = 1'bx; e_rw = 0; e_mr = 0; e_mw = 1; e_br = 0; e_ubr = 0;
               need_def(ra,  "STUR base register was never written by the program.");
               need_def(rbt, "STUR source register was never written by the program.");
            end
            11'b10001011000: begin   // ADD
               iname = "ADD";
               rbsel = rbr;  ealu = ra + rbr;  ewb = ealu;
               uses_a = 1; uses_b = 1; uses_alu = 1; wr = 1;
               e_sign = 2'bxx; e_aluop = 4'b0010; e_rw = 1;
               need_def(ra,  "ADD Rn was never written by the program.");
               need_def(rbr, "ADD Rm was never written by the program.");
            end
            11'b11001011000: begin   // SUB (register)
               iname = "SUB";
               rbsel = rbr;  ealu = ra - rbr;  ewb = ealu;
               uses_a = 1; uses_b = 1; uses_alu = 1; wr = 1;
               e_sign = 2'bxx; e_aluop = 4'b0110; e_rw = 1;
               need_def(ra,  "SUB Rn was never written by the program.");
               need_def(rbr, "SUB Rm was never written by the program.");
            end
            11'b10001010000: begin   // AND
               iname = "AND";
               rbsel = rbr;  ealu = ra & rbr;  ewb = ealu;
               uses_a = 1; uses_b = 1; uses_alu = 1; wr = 1;
               e_sign = 2'bxx; e_aluop = 4'b0000; e_rw = 1;
               need_def(ra,  "AND Rn was never written by the program.");
               need_def(rbr, "AND Rm was never written by the program.");
            end
            11'b10101010000: begin   // ORR
               iname = "ORR";
               rbsel = rbr;  ealu = ra | rbr;  ewb = ealu;
               uses_a = 1; uses_b = 1; uses_alu = 1; wr = 1;
               e_sign = 2'bxx; e_aluop = 4'b0001; e_rw = 1;
               need_def(ra,  "ORR Rn was never written by the program.");
               need_def(rbr, "ORR Rm was never written by the program.");
            end
            11'b1101000100?: begin   // SUBI
               iname = "SUBI";
               imm  = {52'd0, ins[21:10]};
               ealu = ra - imm;  ewb = ealu;
               uses_a = 1; uses_imm = 1; uses_alu = 1; wr = 1;
               e_sign = 2'b00; e_aluop = 4'b0110; e_r2l = 1'bx; e_alusrc = 1; e_rw = 1;
               need_def(ra, "SUBI Rn was never written by the program.");
            end
            11'b10110100???: begin   // CBZ
               iname = "CBZ";
               imm   = {{43{ins[23]}}, ins[23:5], 2'b00};
               rbsel = rbt;  ealu = rbt;  ezero = (rbt == 64'd0);
               enpc  = ezero ? (m_pc + imm) : (m_pc + 64'd4);
               uses_b = 1; uses_imm = 1; uses_alu = 1; is_cbz = 1;
               e_sign = 2'b10; e_aluop = 4'b0111; e_r2l = 1; e_alusrc = 0;
               e_m2r = 1'bx; e_rw = 0; e_br = 1; e_ubr = 0;
               need_def(rbt, "CBZ register was never written by the program.");
            end
            11'b000101?????: begin   // B
               iname = "B";
               imm  = {{36{ins[25]}}, ins[25:0], 2'b00};
               enpc = m_pc + imm;
               uses_imm = 1;
               e_sign = 2'b11; e_aluop = 4'bxxxx; e_r2l = 1'bx; e_alusrc = 1'bx;
               e_m2r = 1'bx; e_rw = 0; e_br = 0; e_ubr = 1;
            end
            default: begin
               iname = "UNKNOWN";
               fail("Opcode is not one this model knows. Wrong PC or bad instruction.");
            end
         endcase

         // 6. control signals (X in the table = not checked)
         ck_sig({2'b00, uut.SignOp}, {2'b00, e_sign}, "SignOp");
         ck_sig(uut.ALUOp,                e_aluop,    "ALUOp");
         ck_sig({3'b000, uut.Reg2Loc},    {3'b000, e_r2l},    "Reg2Loc");
         ck_sig({3'b000, uut.ALUSrc},     {3'b000, e_alusrc}, "ALUSrc");
         ck_sig({3'b000, uut.MemtoReg},   {3'b000, e_m2r},    "MemtoReg");
         ck_sig({3'b000, uut.RegWrite},   {3'b000, e_rw},     "RegWrite");
         ck_sig({3'b000, uut.MemRead},    {3'b000, e_mr},     "MemRead");
         ck_sig({3'b000, uut.MemWrite},   {3'b000, e_mw},     "MemWrite");
         ck_sig({3'b000, uut.Branch},     {3'b000, e_br},     "Branch");
         ck_sig({3'b000, uut.Uncondbranch}, {3'b000, e_ubr},  "Uncondbranch");

         // 7. datapath values, in signal-flow order
         if (uses_a)
            chk64(uut.regoutA, ra,   "regoutA wrong. Check RegisterFile RA wiring (rm = instr[9:5]).");
         if (uses_b)
            chk64(uut.regoutB, rbsel, "regoutB wrong. Check the Reg2Loc mux / rn wiring.");
         if (uses_imm)
            chk64(uut.extimm, imm,   "extimm wrong. Check SignExtender wiring (instr[25:0]) and SignOp.");
         if (uses_alu)
            chk64(uut.aluout, ealu,  "aluout wrong. Check ALU, ALUOp, and the ALUSrc mux.");
         if (is_cbz)
            chk64({63'd0, uut.zero}, {63'd0, ezero}, "Zero flag wrong. Check ALU Zero output and CBZ ALUOp.");
         if (is_load)
            chk64(uut.dmemout, mval, "Data memory read wrong. Check DataMemory Address/MemRead wiring.");
         if (wr)
            chk64(MemtoRegOut, ewb,  "MemtoRegOut wrong. Check the MemtoReg mux and its input order.");
         chk64(uut.nextpc, enpc,     "nextpc wrong. Check NextPClogic and Branch/Uncondbranch/Zero wiring.");

         // 8. all good: advance the model
         if (VERBOSE)
            $display("ok  #%0d pc=%h instr=%h %0s", ncheck, m_pc, ins, iname);
         if (wr && fd != 5'd31) m_reg[fd] = ewb;
         if (is_store) begin
            pend_st = 1; pend_addr = addr; pend_data = rbsel;
         end
         m_pc   = enpc;
         ncheck = ncheck + 1;
      end
   endtask

   // Run the checker in the middle of every cycle, once everything has settled
   always @(negedge CLK) begin
      #5;
      if (checking && !failed) check_cycle;
   end

   // =======================================================================
   // Main test
   // =======================================================================
   integer mi;

   initial begin
      // Initialize checker state
      checking = 0; failed = 0; pend_st = 0; ncheck = 0;
      iname = "-"; m_pc = 64'd0; pend_addr = 64'd0; pend_data = 64'd0;
      for (mi = 0; mi < 32; mi = mi + 1) m_reg[mi] = 64'bx;

      // Initialize Inputs
      Reset = 0;
      startPC = 0;
      passed = 0;

      // Initialize Watchdog timer
      watchdog = 0;

      // Wait for global reset
      #(1 * `ClockPeriod);

      // Program 1
      #1
        Reset = 1; startPC = 0;
      @(posedge CLK);
      @(negedge CLK);
      @(posedge CLK);
      // Release reset on a falling edge so it cannot race with the PC register
      @(negedge CLK);
      Reset = 0;
      m_pc = startPC;
      checking = 1;

      // ***********************************************************
      // Cycle the processor until the PC reaches the final
      // instruction of the first test. The watchdog below kills the
      // simulation if the program never gets there.
      // ***********************************************************

      while (currentPC < 64'h30)
        begin
	   @(posedge CLK);
	   @(negedge CLK);
           $display("CurrentPC:%h",currentPC);
        end

      // Let the checker verify the last instruction before the result check
      #10;
      $display("Checker: %0d instructions matched the reference model", ncheck);
      passTest(MemtoRegOut, 64'hF, "Results of Program 1", passed);

      // ***********************************************************
      // Add your new tests here
      // ***********************************************************

      // Done
      allPassed(passed, 1);   // Be sure to change the one to match
      // the number of tests you add.
      $finish;
   end

   // Initialize the clock to be 0
   initial begin
      CLK = 0;
   end

   // The following is correct if clock starts at LOW level at StartTime //
   always begin
      #`HalfClockPeriod CLK = ~CLK;
      #`HalfClockPeriod CLK = ~CLK;
      watchdog = watchdog +1;
   end

   // Kill the simulation if the watchdog hits 255 cycles
   always @*
     if (watchdog == 16'hFF)
       begin
          $display("Watchdog Timer Expired. PC=%h after %0d checked instructions", currentPC, ncheck);
          $finish;
       end

endmodule
