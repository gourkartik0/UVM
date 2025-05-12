// Code your testbench here
// or browse Examples
import uvm_pkg::*;

module priority_encoder(
  input [7:0] D,
  output reg [2:0]y);
  
  always_comb begin
    casex(D)
      8'b0000_0001: y=3'b000;
      8'b0000_001x: y=3'b001;
      8'b0000_01xx: y=3'b010;
      8'b0000_1xxx: y=3'b011;
      8'b0001_xxxx: y=3'b100;
      8'b001x_xxxx: y=3'b101;
      8'b01xx_xxxx: y=3'b110;
      8'b1xxx_xxxx: y=3'b111;
      
      default: $display("INVALID INPUT");
    endcase
  end
endmodule
      
//////////////////INTERFACE//////////////////////
interface intf;
  logic [7:0] D;
  logic [2:0] y;
endinterface

/////////////////TRANACTION//////////////////////
      class pe_xtn extends uvm_sequence_item;
        randc logic[7:0]D;
        logic [2:0]y;
        
        function new(string name="pe_xtn");
          super.new(name);
        endfunction
        
        `uvm_object_utils_begin(pe_xtn)
        `uvm_field_int(D,UVM_ALL_ON)
        `uvm_field_int(y,UVM_ALL_ON)
        `uvm_object_utils_end
        
      endclass
      
/////////////////SEQUENCE////////////////////////
      class pe_seq extends uvm_sequence#(pe_xtn);
        `uvm_object_utils(pe_seq)
        
        function new(string name="pe_seq");
          super.new(name);
        endfunction
        
        task body;
          req=pe_xtn::type_id::create("req");
          
          repeat(40)begin
            
            start_item(req);
		assert(req.randomize() with {
        D inside {
          8'b0000_0001,
          8'b0000_0011,
          8'b0000_0110,
          8'b0000_1010,
          8'b0001_0110,
          8'b0010_0010,
          8'b0100_0110,
          8'b1100_0110,
          8'b0001_0001,
          8'b1010_0010
        };
        });            `uvm_info("SEQ",$sformatf(" Inputs: D=%b ",req.D),UVM_LOW);
            finish_item(req);
			get_response(req);
    #1;
            
                
          end
        endtask
        
      endclass

///////////////////////DRIVER///////////////////////////
      
		class driver extends uvm_driver#(pe_xtn);
        `uvm_component_utils(driver)
        
          virtual intf vif;
        	pe_xtn req;
        
        function new(string name="driver",uvm_component parent);
          super.new(name,parent);
        endfunction
        
        function void build_phase(uvm_phase phase);
          super.build_phase(phase);
          
          if(!uvm_config_db#(virtual intf)::get(this,"*","vif",vif))
            `uvm_fatal("[DRIVER]","unable to get interface signal")
                            
        endfunction
            
          task run_phase(uvm_phase phase);
          super.run_phase(phase);
          req=pe_xtn::type_id::create("req");
          forever begin
            seq_item_port.get_next_item(req);
            
            	vif.D<=req.D;
          		
            `uvm_info("[DRIVER]",$sformatf(" Inputs: D=%b ",req.D),UVM_LOW);
            seq_item_port.item_done(req);
          end
          endtask
        
      endclass
      
///////////////////MONITOR/////////////////////////
      class monitor extends uvm_monitor;
        `uvm_component_utils(monitor)
        virtual intf vif;
        pe_xtn req;
        
        uvm_analysis_port#(pe_xtn) port;
        
        function new(string name="monitor",uvm_component parent);
          super.new(name,parent);
          port=new("port",this);
        endfunction
        
        function void build_phase(uvm_phase phase);
          super.build_phase(phase);
          
          if(!uvm_config_db#(virtual intf)::get(this,"*","vif",vif))
            `uvm_fatal("[MONITOR]","unable to get interface signal")
                            
        endfunction
            
          task run_phase(uvm_phase phase);
          super.run_phase(phase);
          
          req=pe_xtn::type_id::create("req");
          
          forever begin
            
            #1;
            	req.D = vif.D;
            	req.y = vif.y;
            
            `uvm_info("[Monitor]",$sformatf(" Inputs: D=%b | Y=%b | ",req.D,req.y),UVM_LOW);
            
            port.write(req);
            
          end
          endtask
        
      endclass
      
///////////////////SEQUENCER//////////////////////
      
          class sequencer extends uvm_sequencer#(pe_xtn);
            `uvm_component_utils(sequencer)
            
            function new(string name="sequencer",uvm_component parent);
              super.new(name,parent);
            endfunction
            
            function void build_phase(uvm_phase phase);
              super.build_phase(phase);
                 
            endfunction
            
          endclass
       
              
      
///////////////////AGENT////////////////////////
      
          class agent extends uvm_agent;
            `uvm_component_utils(agent)
            
            sequencer seqr;
            driver drv;
            monitor mon;
            
            function new(string name="agent",uvm_component parent);
              super.new(name,parent);
            endfunction
            
            function void build_phase(uvm_phase phase);
              super.build_phase(phase);
             if(get_is_active() == UVM_ACTIVE)begin
               drv=driver::type_id::create("drv",this);
               seqr=sequencer::type_id::create("seqr",this);
        end
             mon=monitor::type_id::create("mon",this);
             
           endfunction 
            
            function void connect_phase(uvm_phase phase);
              super.connect_phase(phase);
              if(get_is_active() == UVM_ACTIVE)begin
              drv.seq_item_port.connect(seqr.seq_item_export);
              end
            endfunction
          endclass
      
/////////////////////SCOREBOARD////////////////////
      
          class scoreboard extends uvm_scoreboard;
            `uvm_component_utils(scoreboard)
            
            uvm_analysis_imp#(pe_xtn,scoreboard) ap; 
            
            pe_xtn req1[$];
            pe_xtn req2;
            
            covergroup pe_coverage;
              option.per_instance=1;
              
              D:coverpoint req2.D{bins D[] = {
          8'b0000_0001,
          8'b0000_0011,
          8'b0000_0110,
          8'b0000_1010,
          8'b0001_0110,
          8'b0010_0010,
          8'b0100_0110,
          8'b1100_0110,
          8'b0001_0001,
          8'b1010_0010
        };}
              Y:coverpoint req2.y{bins D1={[0:4]};
                                  bins D2={[5:7]};}
            endgroup
            
            function new(string name="scoreboard",uvm_component parent);
              super.new(name,parent);
              ap=new("ap",this);
              pe_coverage=new();
            endfunction        
          
            function void build_phase(uvm_phase phase);
              super.build_phase(phase);                 
          endfunction
      
            task write(pe_xtn t1);
              req1.push_back(t1);
            endtask
            
            task run_phase(uvm_phase phase);
              super.run_phase(phase);
              
              forever begin
              pe_xtn t2;
              
              wait(req1.size() !=0);
              t2=req1.pop_back();
                compare(t2);
              end
            endtask
            
            task compare(pe_xtn t);
              logic[2:0] exp;
              req2=t;
              pe_coverage.sample();
              
              casex(t.D)
                8'b0000_0001: exp=3'b000;
      			8'b0000_001x: exp=3'b001;
      			8'b0000_01xx: exp=3'b010;
     		 	8'b0000_1xxx: exp=3'b011;
      			8'b0001_xxxx: exp=3'b100;
      			8'b001x_xxxx: exp=3'b101;
      			8'b01xx_xxxx: exp=3'b110;
      			8'b1xxx_xxxx: exp=3'b111;
                
              endcase

              if(t.y != exp)begin
                `uvm_error("COMPARE",$sformatf("transaction failed D =%b | ACT=%b |  EXP =%b", t.D,t.y,exp))
          end
          else
            `uvm_info("COMPARE",$sformatf("Transaction PASSED D =%b | ACT=%b |  EXP =%b\n\n", t.D,t.y,exp),UVM_LOW)
              
            
             $display("----------------------------------------------------");
        $display("Overall Coverage %0.2f",$get_coverage);
              $display("Coverage of Gate %0.2f",pe_coverage.get_coverage());
              $display("Coverage report of INPUT D =%f",pe_coverage.D.get_coverage());
              $display("Coverage report of OUTPUT Y =%f",pe_coverage.Y.get_coverage());
        
        	$display("----------------------------------------------------");
        	$display("----------------------------------------------------");
      
     
            endtask
            
          endclass  
      
////////////////////ENVIRONMENT////////////////////
     
       class env extends uvm_env;
        `uvm_component_utils(env)
        agent agt;
        scoreboard sb;
       function new(string name="env",uvm_component parent);
        super.new(name,parent);
      endfunction
      
        function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        agt=agent::type_id::create("agt",this);
        sb=scoreboard::type_id::create("sb",this);
      endfunction
        
        function void connect_phase(uvm_phase phase);
          super.connect_phase(phase);
          agt.mon.port.connect(sb.ap);
        endfunction
        
        function void end_of_elaboration_phase(uvm_phase phase);
           super.end_of_elaboration_phase(phase);
           uvm_top.print_topology();
         endfunction
        
      endclass
          

/////////////////////// TEST///////////////////////
      
      class pe_test extends uvm_test;
        `uvm_component_utils(pe_test)
      
         env envh;
         pe_seq seq_h;
         
        function new(string name="pe_test",uvm_component parent);
        super.new(name,parent);
      endfunction
      
         function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        envh=env::type_id::create("env",this);
       
      endfunction
       
         
         task run_phase(uvm_phase phase);
           super.run_phase(phase);
           seq_h=pe_seq::type_id::create("seq_h");
           
           phase.raise_objection(this);
           
           seq_h.start(envh.agt.seqr);
           #1;
           phase.drop_objection(this);
           
           `uvm_info(get_type_name(),"END_OF_TEST_CASE",UVM_LOW);
           
         endtask     
         
       endclass
          
         
      
////////////////////////TOP//////////////////////////
              
        module top;
         
         intf vif();
         
         `include "uvm_macros.svh"

                 priority_encoder dut(.D(vif.D),.y(vif.y));
         
         initial begin
           uvm_config_db#(virtual intf)::set(null,"*","vif",vif);
           
           run_test("pe_test");
           
           #100;
           $finish;
         end
         
         
       endmodule






          /*

          UVM_INFO /apps/vcsmx/vcs/U-2023.03-SP2//etc/uvm-1.2/src/base/uvm_root.svh(402) @ 0: reporter [UVM/RELNOTES] 
----------------------------------------------------------------
UVM-1.2.Synopsys
(C) 2007-2014 Mentor Graphics Corporation
(C) 2007-2014 Cadence Design Systems, Inc.
(C) 2006-2014 Synopsys, Inc.
(C) 2011-2013 Cypress Semiconductor Corp.
(C) 2013-2014 NVIDIA Corporation
----------------------------------------------------------------

  ***********       IMPORTANT RELEASE NOTES         ************

  You are using a version of the UVM library that has been compiled
  with `UVM_NO_DEPRECATED undefined.
  See http://www.eda.org/svdb/view.php?id=3313 for more details.

  You are using a version of the UVM library that has been compiled
  with `UVM_OBJECT_DO_NOT_NEED_CONSTRUCTOR undefined.
  See http://www.eda.org/svdb/view.php?id=3770 for more details.

      (Specify +UVM_NO_RELNOTES to turn off this notice)

UVM_INFO @ 0: reporter [RNTST] Running test pe_test...
UVM_INFO /apps/vcsmx/vcs/U-2023.03-SP2//etc/uvm-1.2/src/base/uvm_root.svh(589) @ 0: reporter [UVMTOP] UVM testbench topology:
--------------------------------------------------------------
Name                       Type                    Size  Value
--------------------------------------------------------------
uvm_test_top               pe_test                 -     @341 
  env                      env                     -     @354 
    agt                    agent                   -     @363 
      drv                  driver                  -     @392 
        rsp_port           uvm_analysis_port       -     @411 
        seq_item_port      uvm_seq_item_pull_port  -     @401 
      mon                  monitor                 -     @558 
        port               uvm_analysis_port       -     @567 
      seqr                 sequencer               -     @421 
        rsp_export         uvm_analysis_export     -     @430 
        seq_item_export    uvm_seq_item_pull_imp   -     @548 
        arbitration_queue  array                   0     -    
        lock_queue         array                   0     -    
        num_last_reqs      integral                32    'd1  
        num_last_rsps      integral                32    'd1  
    sb                     scoreboard              -     @372 
      ap                   uvm_analysis_imp        -     @381 
--------------------------------------------------------------

UVM_INFO testbench.sv(74) @ 0: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00010001 
UVM_INFO testbench.sv(113) @ 0: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00010001 
UVM_INFO testbench.sv(152) @ 1: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00010001 | Y=100 | 
UVM_INFO testbench.sv(286) @ 1: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00010001 | ACT=100 |  EXP =100


----------------------------------------------------
Overall Coverage 30.00
Coverage of Gate 30.00
Coverage report of INPUT D =10.000000
Coverage report of OUTPUT Y =50.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 1: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00000011 
UVM_INFO testbench.sv(113) @ 1: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00000011 
UVM_INFO testbench.sv(152) @ 2: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00000011 | Y=001 | 
UVM_INFO testbench.sv(286) @ 2: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00000011 | ACT=001 |  EXP =001


----------------------------------------------------
Overall Coverage 35.00
Coverage of Gate 35.00
Coverage report of INPUT D =20.000000
Coverage report of OUTPUT Y =50.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 2: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=01000110 
UVM_INFO testbench.sv(113) @ 2: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=01000110 
UVM_INFO testbench.sv(152) @ 3: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=01000110 | Y=110 | 
UVM_INFO testbench.sv(286) @ 3: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =01000110 | ACT=110 |  EXP =110


----------------------------------------------------
Overall Coverage 65.00
Coverage of Gate 65.00
Coverage report of INPUT D =30.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 3: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00000001 
UVM_INFO testbench.sv(113) @ 3: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00000001 
UVM_INFO testbench.sv(152) @ 4: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00000001 | Y=000 | 
UVM_INFO testbench.sv(286) @ 4: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00000001 | ACT=000 |  EXP =000


----------------------------------------------------
Overall Coverage 70.00
Coverage of Gate 70.00
Coverage report of INPUT D =40.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 4: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00000110 
UVM_INFO testbench.sv(113) @ 4: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00000110 
UVM_INFO testbench.sv(152) @ 5: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00000110 | Y=010 | 
UVM_INFO testbench.sv(286) @ 5: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00000110 | ACT=010 |  EXP =010


----------------------------------------------------
Overall Coverage 75.00
Coverage of Gate 75.00
Coverage report of INPUT D =50.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 5: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00001010 
UVM_INFO testbench.sv(113) @ 5: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00001010 
UVM_INFO testbench.sv(152) @ 6: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00001010 | Y=011 | 
UVM_INFO testbench.sv(286) @ 6: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00001010 | ACT=011 |  EXP =011


----------------------------------------------------
Overall Coverage 80.00
Coverage of Gate 80.00
Coverage report of INPUT D =60.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 6: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00100010 
UVM_INFO testbench.sv(113) @ 6: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00100010 
UVM_INFO testbench.sv(152) @ 7: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00100010 | Y=101 | 
UVM_INFO testbench.sv(286) @ 7: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00100010 | ACT=101 |  EXP =101


----------------------------------------------------
Overall Coverage 85.00
Coverage of Gate 85.00
Coverage report of INPUT D =70.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 7: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=10100010 
UVM_INFO testbench.sv(113) @ 7: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=10100010 
UVM_INFO testbench.sv(152) @ 8: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=10100010 | Y=111 | 
UVM_INFO testbench.sv(286) @ 8: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =10100010 | ACT=111 |  EXP =111


----------------------------------------------------
Overall Coverage 90.00
Coverage of Gate 90.00
Coverage report of INPUT D =80.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 8: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=11000110 
UVM_INFO testbench.sv(113) @ 8: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=11000110 
UVM_INFO testbench.sv(152) @ 9: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=11000110 | Y=111 | 
UVM_INFO testbench.sv(286) @ 9: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =11000110 | ACT=111 |  EXP =111


----------------------------------------------------
Overall Coverage 95.00
Coverage of Gate 95.00
Coverage report of INPUT D =90.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 9: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00010110 
UVM_INFO testbench.sv(113) @ 9: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00010110 
UVM_INFO testbench.sv(152) @ 10: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00010110 | Y=100 | 
UVM_INFO testbench.sv(286) @ 10: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00010110 | ACT=100 |  EXP =100


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 10: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=11000110 
UVM_INFO testbench.sv(113) @ 10: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=11000110 
UVM_INFO testbench.sv(152) @ 11: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=11000110 | Y=111 | 
UVM_INFO testbench.sv(286) @ 11: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =11000110 | ACT=111 |  EXP =111


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 11: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00010001 
UVM_INFO testbench.sv(113) @ 11: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00010001 
UVM_INFO testbench.sv(152) @ 12: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00010001 | Y=100 | 
UVM_INFO testbench.sv(286) @ 12: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00010001 | ACT=100 |  EXP =100


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 12: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00000110 
UVM_INFO testbench.sv(113) @ 12: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00000110 
UVM_INFO testbench.sv(152) @ 13: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00000110 | Y=010 | 
UVM_INFO testbench.sv(286) @ 13: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00000110 | ACT=010 |  EXP =010


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 13: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00000001 
UVM_INFO testbench.sv(113) @ 13: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00000001 
UVM_INFO testbench.sv(152) @ 14: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00000001 | Y=000 | 
UVM_INFO testbench.sv(286) @ 14: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00000001 | ACT=000 |  EXP =000


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 14: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=01000110 
UVM_INFO testbench.sv(113) @ 14: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=01000110 
UVM_INFO testbench.sv(152) @ 15: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=01000110 | Y=110 | 
UVM_INFO testbench.sv(286) @ 15: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =01000110 | ACT=110 |  EXP =110


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 15: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00100010 
UVM_INFO testbench.sv(113) @ 15: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00100010 
UVM_INFO testbench.sv(152) @ 16: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00100010 | Y=101 | 
UVM_INFO testbench.sv(286) @ 16: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00100010 | ACT=101 |  EXP =101


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 16: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00010110 
UVM_INFO testbench.sv(113) @ 16: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00010110 
UVM_INFO testbench.sv(152) @ 17: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00010110 | Y=100 | 
UVM_INFO testbench.sv(286) @ 17: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00010110 | ACT=100 |  EXP =100


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 17: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00001010 
UVM_INFO testbench.sv(113) @ 17: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00001010 
UVM_INFO testbench.sv(152) @ 18: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00001010 | Y=011 | 
UVM_INFO testbench.sv(286) @ 18: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00001010 | ACT=011 |  EXP =011


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 18: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=10100010 
UVM_INFO testbench.sv(113) @ 18: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=10100010 
UVM_INFO testbench.sv(152) @ 19: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=10100010 | Y=111 | 
UVM_INFO testbench.sv(286) @ 19: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =10100010 | ACT=111 |  EXP =111


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 19: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00000011 
UVM_INFO testbench.sv(113) @ 19: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00000011 
UVM_INFO testbench.sv(152) @ 20: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00000011 | Y=001 | 
UVM_INFO testbench.sv(286) @ 20: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00000011 | ACT=001 |  EXP =001


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 20: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00100010 
UVM_INFO testbench.sv(113) @ 20: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00100010 
UVM_INFO testbench.sv(152) @ 21: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00100010 | Y=101 | 
UVM_INFO testbench.sv(286) @ 21: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00100010 | ACT=101 |  EXP =101


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 21: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00000110 
UVM_INFO testbench.sv(113) @ 21: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00000110 
UVM_INFO testbench.sv(152) @ 22: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00000110 | Y=010 | 
UVM_INFO testbench.sv(286) @ 22: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00000110 | ACT=010 |  EXP =010


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 22: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00010110 
UVM_INFO testbench.sv(113) @ 22: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00010110 
UVM_INFO testbench.sv(152) @ 23: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00010110 | Y=100 | 
UVM_INFO testbench.sv(286) @ 23: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00010110 | ACT=100 |  EXP =100


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 23: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00001010 
UVM_INFO testbench.sv(113) @ 23: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00001010 
UVM_INFO testbench.sv(152) @ 24: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00001010 | Y=011 | 
UVM_INFO testbench.sv(286) @ 24: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00001010 | ACT=011 |  EXP =011


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 24: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=01000110 
UVM_INFO testbench.sv(113) @ 24: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=01000110 
UVM_INFO testbench.sv(152) @ 25: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=01000110 | Y=110 | 
UVM_INFO testbench.sv(286) @ 25: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =01000110 | ACT=110 |  EXP =110


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 25: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00010001 
UVM_INFO testbench.sv(113) @ 25: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00010001 
UVM_INFO testbench.sv(152) @ 26: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00010001 | Y=100 | 
UVM_INFO testbench.sv(286) @ 26: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00010001 | ACT=100 |  EXP =100


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 26: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=10100010 
UVM_INFO testbench.sv(113) @ 26: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=10100010 
UVM_INFO testbench.sv(152) @ 27: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=10100010 | Y=111 | 
UVM_INFO testbench.sv(286) @ 27: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =10100010 | ACT=111 |  EXP =111


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 27: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=11000110 
UVM_INFO testbench.sv(113) @ 27: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=11000110 
UVM_INFO testbench.sv(152) @ 28: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=11000110 | Y=111 | 
UVM_INFO testbench.sv(286) @ 28: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =11000110 | ACT=111 |  EXP =111


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 28: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00000011 
UVM_INFO testbench.sv(113) @ 28: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00000011 
UVM_INFO testbench.sv(152) @ 29: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00000011 | Y=001 | 
UVM_INFO testbench.sv(286) @ 29: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00000011 | ACT=001 |  EXP =001


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 29: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00000001 
UVM_INFO testbench.sv(113) @ 29: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00000001 
UVM_INFO testbench.sv(152) @ 30: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00000001 | Y=000 | 
UVM_INFO testbench.sv(286) @ 30: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00000001 | ACT=000 |  EXP =000


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 30: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=11000110 
UVM_INFO testbench.sv(113) @ 30: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=11000110 
UVM_INFO testbench.sv(152) @ 31: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=11000110 | Y=111 | 
UVM_INFO testbench.sv(286) @ 31: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =11000110 | ACT=111 |  EXP =111


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 31: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00010110 
UVM_INFO testbench.sv(113) @ 31: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00010110 
UVM_INFO testbench.sv(152) @ 32: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00010110 | Y=100 | 
UVM_INFO testbench.sv(286) @ 32: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00010110 | ACT=100 |  EXP =100


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 32: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=01000110 
UVM_INFO testbench.sv(113) @ 32: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=01000110 
UVM_INFO testbench.sv(152) @ 33: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=01000110 | Y=110 | 
UVM_INFO testbench.sv(286) @ 33: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =01000110 | ACT=110 |  EXP =110


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 33: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00001010 
UVM_INFO testbench.sv(113) @ 33: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00001010 
UVM_INFO testbench.sv(152) @ 34: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00001010 | Y=011 | 
UVM_INFO testbench.sv(286) @ 34: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00001010 | ACT=011 |  EXP =011


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 34: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00000011 
UVM_INFO testbench.sv(113) @ 34: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00000011 
UVM_INFO testbench.sv(152) @ 35: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00000011 | Y=001 | 
UVM_INFO testbench.sv(286) @ 35: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00000011 | ACT=001 |  EXP =001


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 35: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00010001 
UVM_INFO testbench.sv(113) @ 35: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00010001 
UVM_INFO testbench.sv(152) @ 36: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00010001 | Y=100 | 
UVM_INFO testbench.sv(286) @ 36: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00010001 | ACT=100 |  EXP =100


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 36: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00000001 
UVM_INFO testbench.sv(113) @ 36: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00000001 
UVM_INFO testbench.sv(152) @ 37: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00000001 | Y=000 | 
UVM_INFO testbench.sv(286) @ 37: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00000001 | ACT=000 |  EXP =000


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 37: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00100010 
UVM_INFO testbench.sv(113) @ 37: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00100010 
UVM_INFO testbench.sv(152) @ 38: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00100010 | Y=101 | 
UVM_INFO testbench.sv(286) @ 38: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00100010 | ACT=101 |  EXP =101


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 38: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=00000110 
UVM_INFO testbench.sv(113) @ 38: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=00000110 
UVM_INFO testbench.sv(152) @ 39: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=00000110 | Y=010 | 
UVM_INFO testbench.sv(286) @ 39: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =00000110 | ACT=010 |  EXP =010


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(74) @ 39: uvm_test_top.env.agt.seqr@@seq_h [SEQ]  Inputs: D=10100010 
UVM_INFO testbench.sv(113) @ 39: uvm_test_top.env.agt.drv [[DRIVER]]  Inputs: D=10100010 
UVM_INFO testbench.sv(152) @ 40: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=10100010 | Y=111 | 
UVM_INFO testbench.sv(286) @ 40: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =10100010 | ACT=111 |  EXP =111


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO testbench.sv(152) @ 41: uvm_test_top.env.agt.mon [[Monitor]]  Inputs: D=10100010 | Y=111 | 
UVM_INFO testbench.sv(361) @ 41: uvm_test_top [pe_test] END_OF_TEST_CASE
UVM_INFO testbench.sv(286) @ 41: uvm_test_top.env.sb [COMPARE] Transaction PASSED D =10100010 | ACT=111 |  EXP =111


----------------------------------------------------
Overall Coverage 100.00
Coverage of Gate 100.00
Coverage report of INPUT D =100.000000
Coverage report of OUTPUT Y =100.000000
----------------------------------------------------
----------------------------------------------------
UVM_INFO /apps/vcsmx/vcs/U-2023.03-SP2//etc/uvm-1.2/src/base/uvm_objection.svh(1276) @ 41: reporter [TEST_DONE] 'run' phase is ready to proceed to the 'extract' phase
UVM_INFO /apps/vcsmx/vcs/U-2023.03-SP2//etc/uvm-1.2/src/base/uvm_report_server.svh(904) @ 41: reporter [UVM/REPORT/SERVER] 
--- UVM Report Summary ---

** Report counts by severity
UVM_INFO :  167
UVM_WARNING :    0
UVM_ERROR :    0
UVM_FATAL :    0
** Report counts by id
[COMPARE]    41
[RNTST]     1
[SEQ]    40
[TEST_DONE]     1
[UVM/RELNOTES]     1
[UVMTOP]     1
[[DRIVER]]    40
[[Monitor]]    41
[pe_test]     1

$finish called from file "/apps/vcsmx/vcs/U-2023.03-SP2//etc/uvm-1.2/src/base/uvm_root.svh", line 527.
$finish at simulation time                   41
           V C S   S i m u l a t i o n   R e p o r t 
Time: 41 ns
CPU Time:      0.470 seconds;       Data structure size:   0.3Mb
          
          */
