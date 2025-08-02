`timescale 1ns / 1ps

import noc_params::*; 

module tb_huffman;

    // Variable for Huffman
    localparam int N = 29; //Number of unique weights we are encoding/decoding
    localparam int MAX_CODE = 19; //Length of longest encoded bitstream
    localparam int MAX_PAYLOADS = (MAX_CODE * N + 31) / 32;
    bit [MAX_CODE * N - 1 : 0] bitstream; // bitstream big enough to hold N codes of up to MAX_CODE bits
    logic [31:0] huff_payloads [0:MAX_PAYLOADS-1];  // static array of 32-bit flits (worst-case)
    int total_bits;  // actual bits used
    int M;  // number of BODY flits needed
    // mode selector & BODY-flit counter
    bit   use_huffman;
    int   body_counter;
    
    // List of weights
    logic [7:0] weights [0:N-1] = '{
       121, 119, 118, 116, 108, 113, 111, 112,
       106, 105, 104, 103, 101, 100,  98,  96,
       97, 124,  99, 102, 123, 107, 110, 122,
       109, 114, 115, 117, 120
    };
    // Return the right-aligned code bits for each weight
    function automatic logic [MAX_CODE-1:0] huff_code(logic [7:0] w);
      case (w)
        8'd121: huff_code = {{MAX_CODE-2{1'b0}}, 2'b00};
        8'd119: huff_code = {{MAX_CODE-2{1'b0}}, 2'b01};
        8'd118: huff_code = {{MAX_CODE-3{1'b0}}, 3'b100};
        8'd116: huff_code = {{MAX_CODE-5{1'b0}}, 5'b10100};
        8'd108: huff_code = {{MAX_CODE-8{1'b0}}, 8'b10101000};
        8'd113: huff_code = {{MAX_CODE-8{1'b0}}, 8'b10101001};
        8'd111: huff_code = {{MAX_CODE-7{1'b0}}, 7'b1010101};
        8'd112: huff_code = {{MAX_CODE-8{1'b0}}, 8'b10101100};
        8'd106: huff_code = {{MAX_CODE-10{1'b0}},10'b1010110100};
        8'd105: huff_code = {{MAX_CODE-11{1'b0}},11'b10101101010};
        8'd104: huff_code = {{MAX_CODE-12{1'b0}},12'b101011010110};
        8'd103: huff_code = {{MAX_CODE-13{1'b0}},13'b1010110101110};
        8'd101: huff_code = {{MAX_CODE-15{1'b0}},15'b101011010111100};
        8'd100: huff_code = {{MAX_CODE-16{1'b0}},16'b1010110101111010};
        8'd98 : huff_code = {{MAX_CODE-18{1'b0}},18'b101011010111101100};
        8'd96 : huff_code = {{MAX_CODE-19{1'b0}},19'b1010110101111011010};
        8'd97 : huff_code = {{MAX_CODE-19{1'b0}},19'b1010110101111011011};
        8'd124: huff_code = {{MAX_CODE-18{1'b0}},18'b101011010111101110};
        8'd99 : huff_code = {{MAX_CODE-18{1'b0}},18'b101011010111101111};
        8'd102: huff_code = {{MAX_CODE-14{1'b0}},14'b10101101011111};
        8'd123: huff_code = {{MAX_CODE-10{1'b0}},10'b1010110110};
        8'd107: huff_code = {{MAX_CODE-10{1'b0}},10'b1010110111};
        8'd110: huff_code = {{MAX_CODE-7{1'b0}}, 7'b1010111};
        8'd122: huff_code = {{MAX_CODE-6{1'b0}}, 6'b101100};
        8'd109: huff_code = {{MAX_CODE-8{1'b0}}, 8'b10110100};
        8'd114: huff_code = {{MAX_CODE-8{1'b0}}, 8'b10110101};
        8'd115: huff_code = {{MAX_CODE-7{1'b0}}, 7'b1011011};
        8'd117: huff_code = {{MAX_CODE-5{1'b0}}, 5'b10111};
        8'd120: huff_code = {{MAX_CODE-2{1'b0}}, 2'b11};
        default: huff_code = {MAX_CODE{1'b0}};
      endcase
    endfunction

    // Return how many bits that code uses
    function automatic int huff_len(logic [7:0] w);
      case (w)
        8'd121: huff_len =  2;  8'd119: huff_len =  2;
        8'd118: huff_len =  3;  8'd116: huff_len =  5;
        8'd108: huff_len =  8;  8'd113: huff_len =  8;
        8'd111: huff_len =  7;  8'd112: huff_len =  8;
        8'd106: huff_len = 10;  8'd105: huff_len = 11;
        8'd104: huff_len = 12;  8'd103: huff_len = 13;
        8'd101: huff_len = 15;  8'd100: huff_len = 16;
        8'd98 : huff_len = 18;  8'd96 : huff_len = 19;
        8'd97 : huff_len = 19;  8'd124: huff_len = 18;
        8'd99 : huff_len = 18;  8'd102: huff_len = 14;
        8'd123: huff_len = 10;  8'd107: huff_len = 10;
        8'd110: huff_len =  7;  8'd122: huff_len =  6;
        8'd109: huff_len =  8;  8'd114: huff_len =  8;
        8'd115: huff_len =  7;  8'd117: huff_len =  5;
        8'd120: huff_len =  2;
        default: huff_len =  8;
      endcase
    endfunction

    // Testbench signals
    flit_t flit_written[PORT_NUM];
    flit_t flit_read[PORT_NUM];
    flit_t packet_queue[PORT_NUM][$];
    
    int x_curr, y_curr, num_op, timer;
    int pkt_size[$], flit_num[$], flit_to_read[$], flit_to_read_next[$], multiple_head[$], wait_time[$],x_dest[$], y_dest[$], packet_id[$];
    
    logic [PORT_SIZE-1:0] test_port_num[$];
    logic [PORT_NUM-1:0] insert_not_compl, head_done;
    logic [VC_SIZE-1:0] vc_num [$];

    logic clk;
    logic rst;
    wire [VC_NUM-1:0] error_o [PORT_NUM-1:0];

    //connections from upstream
    flit_t data_out [PORT_NUM-1:0];
    logic [PORT_NUM-1:0] valid_flit_out;
    logic [PORT_NUM-1:0] [VC_NUM-1:0] on_off_in;
    logic [PORT_NUM-1:0] [VC_NUM-1:0] is_allocatable_in;

    //connections from downstream
    flit_t data_in [PORT_NUM-1:0];
    logic valid_flit_in [PORT_NUM-1:0];
    logic [VC_NUM-1:0] on_off_out [PORT_NUM-1:0];
    logic [VC_NUM-1:0] is_allocatable_out [PORT_NUM-1:0];

    // Time stamps for latency tracking
    time t_start, t_end, prev_time, delta;
    time sum_deltas;
    int  out_count;

    //DUT Interfaces Instantiation
    router2router local_up();
    router2router north_up();
    router2router south_up();
    router2router west_up();
    router2router east_up();
    router2router local_down();
    router2router north_down();
    router2router south_down();
    router2router west_down();
    router2router east_down();

    //DUT Instantiation
    router #(
        .BUFFER_SIZE(8),
        .X_CURRENT(0),
        .Y_CURRENT(0)
    )
    router (
        .clk(clk),
        .rst(rst),
        //router2router.upstream 
        .router_if_local_up(local_up),
        .router_if_north_up(north_up),
        .router_if_south_up(south_up),
        .router_if_west_up(west_up),
        .router_if_east_up(east_up),
        //router2router.downstream
        .router_if_local_down(local_down),
        .router_if_north_down(north_down),
        .router_if_south_down(south_down),
        .router_if_west_down(west_down),
        .router_if_east_down(east_down),
        .error_o(error_o)
    );

    routers_mock routers_mock (
        .router_if_local_up(local_down),
        .router_if_north_up(north_down),
        .router_if_south_up(south_down),
        .router_if_west_up(west_down),
        .router_if_east_up(east_down),
        .router_if_local_down(local_up),
        .router_if_north_down(north_up),
        .router_if_south_down(south_up),
        .router_if_west_down(west_up),
        .router_if_east_down(east_up),
        .data_out(data_out),
        .is_valid_out(valid_flit_out),
        .is_on_off_in(on_off_in),
        .is_allocatable_in(is_allocatable_in),
        .data_in(data_in),
        .is_valid_in(valid_flit_in),
        .is_on_off_out(on_off_out),
        .is_allocatable_out(is_allocatable_out)
    );

    initial
    begin
        dump_output();
        initialize();
        clear_reset();
        
     // --- Build the Huffman bitstream ---
        total_bits = 0;
        for (int i = 0; i < N; i++) begin
          int L = huff_len(weights[i]);
          logic [MAX_CODE-1:0] code = huff_code(weights[i]);
          // copy code bits bit-by-bit
          for (int b = 0; b < L; b++) begin
            bitstream[ total_bits + b ] = code[b];
          end
          total_bits += L;
        end
        $display("Huffman bits: %0d (raw %0d)", total_bits, 8*N);
    
        // --- Slice into 32-bit payloads ---
        M = (total_bits + 31) / 32;
        $display("Will use %0d BODY flits for Huffman payload", M);
    
        // clear entire array
        for (int f = 0; f < MAX_PAYLOADS; f++)
          huff_payloads[f] = 32'b0;
    
        // fill first M entries
        for (int f = 0; f < M; f++) begin
          int base = f * 32;
          for (int b = 0; b < 32; b++) begin
            int idx = base + b;
            if (idx < total_bits)
              huff_payloads[f][b] = bitstream[idx];
            // else remains zero
          end
        end


        // --- RUN #1: Baseline (no compression) ---
          use_huffman   = 0;
          pkt_size      = '{ N + 2 };    // HEAD + N BODY + TAIL
          x_dest        = '{3};
          y_dest        = '{3};
          test_port_num = '{0};
          packet_id     = '{0};
          vc_num        = '{0};
          multiple_head = '{0};
          wait_time     = '{0};
        
          // reset latency trackers
          t_start    = $time;
          prev_time  = t_start;
          sum_deltas = 0;
          out_count  = 0;
          test();
        
          t_end = $time;
          $display("BASELINE ? total=%0t ps, sum_intervals=%0t ps", t_end-t_start, sum_deltas);
        
          // --- RUN #2: Huffman – same packet, compressed bodies ---
          use_huffman   = 1;
          pkt_size      = '{ M + 2 };    // HEAD + M BODY + TAIL
          // same coords / ports:
          x_dest        = '{3};
          y_dest        = '{3};
          test_port_num = '{0};
          packet_id     = '{0};
          vc_num        = '{0};
          multiple_head = '{0};
          wait_time     = '{0};
        
          // reset latency trackers
          t_start    = $time;
          prev_time  = t_start;
          sum_deltas = 0;
          out_count  = 0;
          body_counter = 0;  // ensure we start at huff_payloads[0]
        
          test();
        
          t_end = $time;
          $display("HUFFMAN  ? total=%0t ps, sum_intervals=%0t ps", t_end-t_start, sum_deltas);
        
          $display("[All tests PASSED]");
          #20 $finish;
    end

    // Clock update
    always #5 clk = ~clk;

    // Output dump
    task dump_output();
        $dumpfile("out.vcd");
        $dumpvars(0, tb_huffman);
    endtask

    // Initialize signals
    task initialize();
        clk     <= 0;
        rst     = 1;
    endtask
    
    // De-assert the reset signal
    task clear_reset();
        @(posedge clk);
            rst <= 0;
    endtask
    
    /*
    Create a flit to be written in both DUT and packet queue, with the given flit label and packet number in 
    the port identifier passed as port_id parameter.
    The flit to be written is created accordingly to its label, that is, HEAD and HEADTAIL flits are different
    with respect to BODY and TAIL ones.
    The last parameters, id and pkt_id, respectively refer to the identifier of the test case and the id of the packet 
    that will be inserted.
    */
    task automatic create_flit(input flit_label_t lab, input logic [PORT_SIZE-1:0] port_id, input integer id, input int pkt_id);
        flit_written[port_id].flit_label = lab;
        flit_written[port_id].vc_id      = vc_num[id];
        if(lab == HEAD | lab == HEADTAIL)
            begin
                flit_written[port_id].data.head_data.x_dest  = x_dest[id];
                flit_written[port_id].data.head_data.y_dest  = y_dest[id];
                flit_written[port_id].data.head_data.head_pl = pkt_id;
            end
        else
                flit_written[port_id].data.bt_pl = pkt_id;
    endtask
    
    /*
    Write flit into the DUT module in the proper port, given by the port identifier as input;
    while writing a flit into a port, the relative valid flag is set to 1.
    The last parameters, id and pkt_id, respectively refer to the identifier of the test case and the id of the packet 
    that will be inserted.
    Finally, the push task is called.
    */
    task automatic write_flit(input logic [PORT_SIZE-1:0] port_id, input integer pkt_id, input integer id);
        begin
            valid_flit_in[port_id]  <= 1;
            data_in[port_id]        <= flit_written[port_id];
        end
        num_op++;
        push_flit(port_id, pkt_id, id);
    endtask
    
    /*
    Push the actual flit into the proper queue only under specific conditions.
    In particular, the push operation is done if the HEAD flit hasn't been inserted yet or
    the flit to insert is not an HEAD one (i.e. multiple_head==0).
    The two last parameters, id and pkt_id, respectively refer to the identifier of the test case and the id of the packet 
    that will be inserted.
    */
    task automatic push_flit(input logic [PORT_SIZE-1:0] port_id, input integer pkt_id, input integer id);
        if( ~head_done[port_id] | int'(multiple_head[id]) == 0)
        begin
            $display("push %d, dest %d, pktid %d", $time, computeOutport(x_dest[port_id], y_dest[port_id]), pkt_id);
            packet_queue[pkt_id].push_back(flit_written[port_id]);
            $display("Pushed flit, queue size %d", packet_queue[pkt_id].size());
            flit_to_read_next[pkt_id]++;
        end
    endtask
    
    /*
    This is the main task of the testbench: after a preliminary phase of initialization, it repeatedly calls the 3 subtasks
    until there are no flits to read and the insertion of the flits of all the packets has not been completed (these two conditions
    are checked by means of a function).
    */
    task test();
        $display("\n*** NEW TEST * %d ***", $time);
        initTest();
        while(checkEndConditions()) @(posedge clk)
        begin            
            insertFlit();
            checkFlits();
            updateFlitToRead();
        end
    endtask
    
    /*
    This function updates the flit_to_read variable of all the ports in the test case vector.
    */
    function void updateFlitToRead();
        automatic int i;
        
        for(i=0; i<test_port_num.size(); i++)
            flit_to_read[packet_id[i]] = flit_to_read_next[packet_id[i]];   
    endfunction
    
    /*
    This task checks whether there are flits still to read from the queues and that the insertion of all packets into the ports has not yet completed.
    The checks is done for all ports indicated in the test_port_num list. 
    */
    function bit unsigned checkEndConditions();
        automatic int i, pid;
    
        for(i = 0; i < test_port_num.size(); i++)
        begin
            if(packet_queue[i].size()>0 | insert_not_compl[test_port_num[i]])
                return 1;
        end
        return 0;
    endfunction
    
    /*
    This task is responsible of understanding the type of the next flit that will be inserted
    and calling the proper writing task according to some conditions.
    */
task insertFlit();
    automatic int i, j, pkt_id, p_size;
    automatic logic [PORT_SIZE-1:0] port_id;

    for (i = 0; i < test_port_num.size(); i++) begin
        // recover port index
        for (j = 0; j < PORT_SIZE; j++)
            port_id[j] = test_port_num[i][j];

        pkt_id = packet_id[i];
        p_size = pkt_size[i];

        if (p_size == 1) begin
            // single-flit HEADTAIL
            flit_num[port_id]++;
            if (flit_num[port_id] == 1) begin
                create_flit(HEADTAIL, port_id, i, pkt_id);
                write_flit(port_id, pkt_id, i);
                insert_not_compl[port_id] <= 0;
            end else begin
                valid_flit_in[port_id] <= 0;
            end
        end else begin
            // multi-flit packet
            if (timer == 0
                && insert_not_compl[port_id]
                && on_off_out[port_id][vc_num[i]]) begin

                flit_num[port_id]++;

                // HEAD (or extra HEADs)
                if (flit_num[port_id] == 1 || multiple_head[i] > 0) begin
                    create_flit(HEAD, port_id, i, pkt_id);
                    write_flit(port_id, pkt_id, i);
                    multiple_head[i]--;
                    head_done[port_id] = 1;

                end else if (flit_num[port_id] == p_size) begin
                    // TAIL
                    create_flit(TAIL, port_id, i, pkt_id);
                    write_flit(port_id, pkt_id, i);
                    insert_not_compl[port_id] <= 0;

                end else begin
                    // BODY ? choose raw vs. Huffman payload
                    create_flit(BODY, port_id, i, pkt_id);
                    if (use_huffman) begin
                        flit_written[port_id].data.bt_pl = huff_payloads[ body_counter ];
                    end else begin
                        flit_written[port_id].data.bt_pl = weights[ body_counter ];
                    end
                    body_counter++;
                    write_flit(port_id, pkt_id, i);
                end

                timer = wait_time[port_id];

            end else begin
                // no new flit this cycle
                valid_flit_in[port_id] <= 0;
                if (timer > 0)
                    timer--;
            end
        end
    end
endtask

    
    /*
    This task just updates the counters that control the flits insertion and then pops out of the proper queue the next flit to be read. 
    The pkt_id refers to the identifier of the packet which is going to be read from the router and it is used to properly choose the where to 
    read from and where to put the read flit.
    */
    task automatic readFlit(input int pkt_id);
        automatic int pid;
        //$display("Read simtime %d, portnum %d, toread %d destport %d",$time, port_id,flit_to_read[port_id], dest_port_id);
        begin
            num_op++;
            flit_to_read_next[pkt_id]--;
            flit_read[pkt_id] = packet_queue[pkt_id].pop_front();
        end
    endtask
    
    /*
    Checks the correspondance between the flit extracted from the queue and the one in data_o; this check is done for all the port where
    the flit in output is valid. 
    If the check goes wrong an error message is displayed and the testbench ends.
    */
    task checkFlits();
        automatic  int i, pkt_id;
        automatic logic [PORT_SIZE-1:0] port_id;
        
        @(negedge clk)
//        $display("Check %d, port_num %d, toread %d, valid_flit_out %b",$time, port_num, flit_to_read[port_num],valid_flit_out[computeOutport(x_dest, y_dest)]); 
        begin 
            for(i=0; i<PORT_NUM; i++)
            begin
                if(valid_flit_out[i])
                begin
                    out_count++;
                    delta       = $time - prev_time;
                    sum_deltas += delta;
                    $display("Flit %0d transfer interval: %0t ps", out_count, delta);
                    prev_time   = $time;
                    if(data_out[i].flit_label == HEAD || data_out[i].flit_label == HEADTAIL)
                        pkt_id = data_out[i].data.head_data.head_pl;
                    else
                        pkt_id = data_out[i].data.bt_pl;
                        
                    readFlit( pkt_id);
                    
                    if(~checkFlitFields(flit_read[port_id],data_out[port_id]))
                    begin
                        $display("[READ] FAILED %d", $time);
                        #10 $finish;
                    end
                    else
                        $display("[READ] PASSED %d", $time);
                 end // end if
            end // end for
        end
    endtask 
    
    /*
    The function checks whether the label and the content of the two given flits are equal or not.
    Notice that the check doesn't consider the vc identifier, which is computed by the internal SA module.
    The objective in this case is only to verify that the packet exiting from the router maintains the same destionation
    address and data payload.
    */
    function bit checkFlitFields(flit_t flit_read, flit_t flit_out);
        if(flit_read.flit_label === flit_out.flit_label & 
            flit_read.data === flit_out.data)
            return 1;
        return 0;
    endfunction
    
    /*
    This task initializes to proper value all variables that are necessary for each test before it starts.
    */
    task initTest();
        automatic int i,j;
        timer       = 0;
        body_counter = 0;
        // Values reset
        for(i=0;i<PORT_NUM;i++)
        begin
            valid_flit_in[i]    = 0;
            head_done[i]        = 0;
            flit_num[i]         = 0;
            flit_to_read[i]     = 0;
            flit_to_read_next[i]= 0;
            insert_not_compl[i] = 0;
            
            for(j=0; j<VC_NUM; j++)
            begin
                is_allocatable_in[i][j] = 1;    // means that downstream router is always available
                on_off_in[i][j] = 1;            // always do "read" operation from the router          
            end
        end
        
        // Assert flag for each port in the test port list
        for(i=0; i<test_port_num.size(); i++)    
            insert_not_compl[test_port_num[i]] = 1;
    endtask
    
    /*
    Compute the outport for the current packet according to
    the position of the router into the mesh and the destionation positions.
    */
    function int computeOutport(input int xdest, input int ydest);
        automatic int x_off, y_off, res;
        x_off = xdest - x_curr;
        y_off = ydest - y_curr;
        
        if(x_off < 0)
            res = 3; //WEST
        else if (x_off > 0)
            res = 4; //EAST
        else if (y_off < 0)
            res = 1; //NORTH
        else if (y_off > 0)
            res = 2; //SOUTH
        else // x_off=0 and y_off=0
            res = 0; //LOCAL
        return res;
    endfunction
    
    /*
    This task tries to insert into the module a BODY and a TAIL
    flit without the usual leading HEAD flit. 
    A simple check is done in order to check the proper behavior of the dut.
    */ 
    task noHead();
        @(posedge clk)
        begin
            create_flit(BODY, 0, 0, 0);
            write_flit(0,0,0);
        end 
        @(posedge clk);
            valid_flit_in[0] = 0;
        @(negedge clk)
        begin
            if(~(error_o[0][0]))
                #20 $finish;
        end
        @(posedge clk)
        begin
            create_flit(TAIL, 0, 0, 0);
            write_flit(0,0,0);
        end
        @(posedge clk);
            valid_flit_in[0] = 0;
        @(negedge clk)
        begin
            if(~(error_o[0][0]))
                #20 $finish;
        end
    endtask

endmodule

/*
    ROUTERS MOCK MODULE
*/
module routers_mock #()(
    router2router.upstream router_if_local_up,
    router2router.upstream router_if_north_up,
    router2router.upstream router_if_south_up,
    router2router.upstream router_if_west_up,
    router2router.upstream router_if_east_up,
    router2router.downstream router_if_local_down,
    router2router.downstream router_if_north_down,
    router2router.downstream router_if_south_down,
    router2router.downstream router_if_west_down,
    router2router.downstream router_if_east_down,

    //ports to propagate to downstream interfaces
    output flit_t data_out [PORT_NUM-1:0],
    output logic [PORT_NUM-1:0] is_valid_out,
    input logic [PORT_NUM-1:0] [VC_NUM-1:0] is_on_off_in,
    input logic [PORT_NUM-1:0] [VC_NUM-1:0] is_allocatable_in,

    //ports to propagate to upstream interfaces
    input flit_t data_in [PORT_NUM-1:0],
    input logic is_valid_in [PORT_NUM-1:0],
    output logic [VC_NUM-1:0] is_on_off_out [PORT_NUM-1:0],
    output logic [VC_NUM-1:0] is_allocatable_out [PORT_NUM-1:0]
);

    always_comb
    begin
       
        router_if_local_up.data = data_in[LOCAL];
        router_if_north_up.data = data_in[NORTH];
        router_if_south_up.data = data_in[SOUTH];
        router_if_west_up.data  = data_in[WEST];
        router_if_east_up.data  = data_in[EAST];
        
        router_if_local_up.is_valid = is_valid_in[LOCAL];
        router_if_north_up.is_valid = is_valid_in[NORTH];
        router_if_south_up.is_valid = is_valid_in[SOUTH];
        router_if_west_up.is_valid  = is_valid_in[WEST];
        router_if_east_up.is_valid  = is_valid_in[EAST];
        
        is_on_off_out[LOCAL] = router_if_local_up.is_on_off;
        is_on_off_out[NORTH] = router_if_north_up.is_on_off;
        is_on_off_out[SOUTH] = router_if_south_up.is_on_off;
        is_on_off_out[WEST]  = router_if_west_up.is_on_off;
        is_on_off_out[EAST]  = router_if_east_up.is_on_off;
        
        is_allocatable_out[LOCAL] = router_if_local_up.is_allocatable;
        is_allocatable_out[NORTH] = router_if_north_up.is_allocatable;
        is_allocatable_out[SOUTH] = router_if_south_up.is_allocatable;
        is_allocatable_out[WEST]  = router_if_west_up.is_allocatable;
        is_allocatable_out[EAST]  = router_if_east_up.is_allocatable;
        
        data_out[LOCAL] = router_if_local_down.data;
        data_out[NORTH] = router_if_north_down.data;
        data_out[SOUTH] = router_if_south_down.data;
        data_out[WEST]  = router_if_west_down.data;
        data_out[EAST]  = router_if_east_down.data;
        
        is_valid_out[LOCAL] = router_if_local_down.is_valid;
        is_valid_out[NORTH] = router_if_north_down.is_valid;
        is_valid_out[SOUTH] = router_if_south_down.is_valid;
        is_valid_out[WEST]  = router_if_west_down.is_valid;
        is_valid_out[EAST]  = router_if_east_down.is_valid;
                
        router_if_local_down.is_on_off = is_on_off_in[LOCAL];
        router_if_north_down.is_on_off = is_on_off_in[NORTH];
        router_if_south_down.is_on_off = is_on_off_in[SOUTH];
        router_if_west_down.is_on_off  = is_on_off_in[WEST];
        router_if_east_down.is_on_off  = is_on_off_in[EAST];
        
        router_if_local_down.is_allocatable = is_allocatable_in[LOCAL];
        router_if_north_down.is_allocatable = is_allocatable_in[NORTH];
        router_if_south_down.is_allocatable = is_allocatable_in[SOUTH];
        router_if_west_down.is_allocatable  = is_allocatable_in[WEST];
        router_if_east_down.is_allocatable  = is_allocatable_in[EAST];
        
    end 
endmodule