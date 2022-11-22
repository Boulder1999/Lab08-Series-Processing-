module SP(
	// Input signals
	clk,
	rst_n,
	cg_en,
	in_valid,
	in_data,
	in_mode,
	// Output signals
	out_valid,
	out_data
);

// INPUT AND OUTPUT DECLARATION  
input		clk;
input		rst_n;
input		in_valid;
input		cg_en;
input [8:0] in_data;
input [2:0] in_mode;

output reg 		  out_valid;
output reg signed[9:0] out_data;
//////////design
reg in_valid_gray,in_valid_addsub,in_valid_sma,in_valid_sort;
wire out_valid_gray,out_valid_addsub,out_valid_sma,out_valid_sort;
reg [4:0] cnt;
reg [1:0] cnt_out;
reg [8:0] gray_in[0:8];
reg [8:0] addsub_in[0:8];
reg [8:0] sma_in[0:8];
reg [8:0] sort_in[0:8];
wire [8:0] gray_out[0:8];
wire [8:0] addsub_out[0:8];
wire [8:0] sma_out[0:8];
wire [8:0] sort_out[0:2];
parameter IDLE=3'd0,LOAD=3'd1,S_GRAY=3'd2,S_ADDSUB=3'd3,S_SMA=3'd4,S_SORT=3'd5,OUT_PRE=3'd6,OUT=3'd7;
reg [2:0] cs,ns;
reg [2:0] mode;
reg [8:0] in [0:8];
reg [8:0] out[0:2];
integer i;

always@(posedge clk or negedge rst_n) begin
	if(!rst_n) cs<=IDLE;
	else cs<=ns;
end
always@(posedge clk or negedge rst_n) begin
	if(!rst_n) mode<=0;
	else if(ns == IDLE) mode<=0;
	else if(ns == LOAD && cnt == 0) begin
		mode<=in_mode;
	end
	else mode<=mode;
end
always@(*)begin
	case(cs)
	IDLE:begin
		if(in_valid) ns = LOAD;
		else ns = IDLE;
	end
	LOAD:begin
		if(cnt < 9) ns = LOAD;
		else begin
			if( mode[0] == 1) ns = S_GRAY;
			else if( mode[1:0] == 2'b10 ) ns = S_ADDSUB;
			else if( mode[2:0] == 3'b100 ) ns =S_SMA;
			else ns = S_SORT;
		end
	end
	S_GRAY:begin
		if(out_valid_gray)begin
			if(mode[1]==1) ns = S_ADDSUB;
			else begin
				if(mode[2]==1) ns = S_SMA;
				else ns = S_SORT;
			end
		end
		else ns = S_GRAY;
	end
	S_ADDSUB:begin
		if(out_valid_addsub )begin
			if(mode[2]==1) ns = S_SMA;
			else ns = S_SORT;
		end
		else ns = S_ADDSUB;
	end
	S_SMA:begin
		if(out_valid_sma) ns = S_SORT;
		else ns = S_SMA;
	end
	S_SORT:begin
	    if(out_valid_sort) ns = OUT_PRE;
		else ns = S_SORT;
	end
	OUT_PRE:ns = OUT;
	OUT:begin
		if(cnt < 3) ns = OUT;
		else ns = IDLE;
	end
	default: ns = cs;
	endcase
end

always@(posedge clk or negedge rst_n)begin
	if(!rst_n) cnt<=0;
	else if(ns == IDLE) cnt<=0;
	else if(ns == LOAD) cnt<=cnt+1;
	else if(ns == OUT) cnt<=cnt+1;
	else cnt<=0;
end
always@(posedge clk or negedge rst_n) begin
	if(!rst_n) for(i=0 ; i< 9; i=i+1) in[i]<=0;
	else if(ns == IDLE ) for(i=0 ; i< 9; i=i+1) in[i]<=0;
	else if(ns == LOAD) begin
		if(in_valid)
			in[cnt] <= in_data;
	end
	else begin
		for(i=0;i<9;i=i+1) in[i]<=in[i];
	end
end
/////////////////////////////////////gray
always@(posedge clk or negedge rst_n) begin
	if(!rst_n) in_valid_gray<=0;
	else if(ns == S_GRAY) in_valid_gray<=1;
	else in_valid_gray<=0;
end
GRAY sub0(.clk(clk) ,.in_valid(in_valid_gray), .rst_n(rst_n), .a0(gray_in[0]), .a1(gray_in[1]) , .a2(gray_in[2]), .a3(gray_in[3]), .a4(gray_in[4]), .a5(gray_in[5]), .a6(gray_in[6]), .a7(gray_in[7]), .a8(gray_in[8])
			,.out_valid(out_valid_gray) ,.out0(gray_out[0]),.out1(gray_out[1]),.out2(gray_out[2]),.out3(gray_out[3]),.out4(gray_out[4]),.out5(gray_out[5]),.out6(gray_out[6]),.out7(gray_out[7]),.out8(gray_out[8]));
genvar gen_i;
generate
	for(gen_i=0;gen_i<9;gen_i=gen_i+1) begin
		always@(posedge clk or negedge rst_n) begin
			if(!rst_n)  gray_in[gen_i]<=0;
			else if(ns == IDLE) gray_in[gen_i]<=0;
			else if(ns == S_GRAY) gray_in[gen_i]<=in[gen_i];
			else gray_in[gen_i]<=gray_in[gen_i];
		end
	end
endgenerate
////////////////////////////////////addsub
always@(posedge clk or negedge rst_n) begin
	if(!rst_n) in_valid_addsub<=0;
	else if(ns == S_ADDSUB) in_valid_addsub<=1;
	else in_valid_addsub<=0;
end
ADDorSUB sub1(.clk(clk) ,.in_valid(in_valid_addsub), .rst_n(rst_n), .a0(addsub_in[0]), .a1(addsub_in[1]) , .a2(addsub_in[2]), .a3(addsub_in[3]), .a4(addsub_in[4]), .a5(addsub_in[5]), .a6(addsub_in[6]), .a7(addsub_in[7]), .a8(addsub_in[8])
	,.out_valid(out_valid_addsub) ,.out0(addsub_out[0]),.out1(addsub_out[1]),.out2(addsub_out[2]),.out3(addsub_out[3]),.out4(addsub_out[4]),.out5(addsub_out[5]),.out6(addsub_out[6]),.out7(addsub_out[7]),.out8(addsub_out[8]));
generate
	for(gen_i=0;gen_i<9;gen_i=gen_i+1) begin
		always@(posedge clk or negedge rst_n) begin
			if(!rst_n)  addsub_in[gen_i]<=0;
			else if(ns == IDLE) addsub_in[gen_i]<=0;
			else if(ns == S_ADDSUB) begin
				if(mode[0]==1) addsub_in[gen_i]<=gray_out[gen_i];
				else addsub_in[gen_i]<= in[gen_i];
			end
			else addsub_in[gen_i]<= addsub_in[gen_i];
		end
	end
endgenerate
///////////////////////////////sma
always@(posedge clk or negedge rst_n) begin
	if(!rst_n) in_valid_sma<=0;
	else if(ns == S_SMA) in_valid_sma<=1;
	else in_valid_sma<=0;
end
SMA sub2(.clk(clk) ,.in_valid(in_valid_sma), .rst_n(rst_n), .a0(sma_in[0]), .a1(sma_in[1]) , .a2(sma_in[2]), .a3(sma_in[3]), .a4(sma_in[4]), .a5(sma_in[5]), .a6(sma_in[6]), .a7(sma_in[7]), .a8(sma_in[8])
	,.out_valid(out_valid_sma) ,.out0(sma_out[0]),.out1(sma_out[1]),.out2(sma_out[2]),.out3(sma_out[3]),.out4(sma_out[4]),.out5(sma_out[5]),.out6(sma_out[6]),.out7(sma_out[7]),.out8(sma_out[8]));
generate
	for(gen_i=0;gen_i<9;gen_i=gen_i+1) begin
		always@(posedge clk or negedge rst_n) begin
			if(!rst_n)  sma_in[gen_i]<=0;
			else if(ns == IDLE) sma_in[gen_i]<=0;
			else if(ns == S_SMA) begin
				if(mode[1]==1)begin
					sma_in[gen_i]<=addsub_out[gen_i];
				end
				else begin//01||00
					if(mode[0]==1) sma_in[gen_i]<=gray_out[gen_i];
					else sma_in[gen_i]<=in[gen_i];
				end
			end
			else sma_in[gen_i]<=sma_in[gen_i];
		end
	end
endgenerate
///////////////////////////////sort
always@(posedge clk or negedge rst_n) begin
	if(!rst_n) in_valid_sort<=0;
	else if(ns == S_SORT) in_valid_sort<=1;
	else in_valid_sort<=0;
end

SORT_module sub3(.clk(clk),.in_valid(in_valid_sort),.rst_n(rst_n),.a0(sort_in[0]),.a1(sort_in[1]),.a2(sort_in[2]),.a3(sort_in[3]),.a4(sort_in[4]),.a5(sort_in[5]),.a6(sort_in[6]),.a7(sort_in[7]),.a8(sort_in[8]),
	.out_valid(out_valid_sort),.out0(sort_out[0]),.out4(sort_out[1]),.out8(sort_out[2]));
generate
	for(gen_i=0;gen_i<9;gen_i=gen_i+1) begin
		always@(posedge clk or negedge rst_n) begin
			if(!rst_n)  sort_in[gen_i]<=0;
			else if(ns == IDLE) sort_in[gen_i]<=0;
			else if(ns == S_SORT ) begin
				if(mode[2]==1) sort_in[gen_i]<=sma_out[gen_i];
				else begin //010 011 001 000
					if(mode[1]==1) sort_in[gen_i]<=addsub_out[gen_i];
					else begin
						if(mode[0]==1) sort_in[gen_i]<=gray_out[gen_i];
						else sort_in[gen_i]<=in[gen_i];
					end
				end
			end
			else sort_in[gen_i]<=sort_in[gen_i];
		end
	end
endgenerate
////////////out
generate
	for(gen_i=0;gen_i<3;gen_i=gen_i+1) begin
		always@(posedge clk or negedge rst_n) begin
			if(!rst_n)  out[gen_i]<=0;
			else if(ns == IDLE) out[gen_i]<=0;
			else if(ns == OUT_PRE) out[gen_i] <= sort_out[gen_i];
			else out[gen_i] <= out[gen_i];
		end
	end
endgenerate
always@(posedge clk or negedge rst_n) begin
			if(!rst_n)  out_valid<=0;
			else if(ns == IDLE) out_valid<=0;
			else if(ns == OUT) out_valid<=1;
end
always@(posedge clk or negedge rst_n)begin
	if(!rst_n) cnt_out<=2;
	else if(ns == IDLE) cnt_out<=2;
	else if(ns == OUT) cnt_out<=cnt_out-1;
	else cnt_out<=2;
end

always@(posedge clk or negedge rst_n) begin
	if(!rst_n) out_data <=0;
	else if(ns == IDLE ) out_data <=0;
	else if(ns == OUT) begin
		if(out[cnt_out][8]==1) out_data <={1'b1,out[cnt_out]} ;
		else out_data <={1'b0,out[cnt_out]} ;
	end
	else out_data <=0;
end


endmodule

//sub
module GRAY(
	clk,in_valid,rst_n,a0,a1,a2,a3,a4,a5,a6,a7,a8,
	out_valid,out0,out1,out2,out3,out4,out5,out6,out7,out8
);
input clk,rst_n,in_valid;
input [8:0] a0,a1,a2,a3,a4,a5,a6,a7,a8;
output reg out_valid;
output reg [8:0] out0,out1,out2,out3,out4,out5,out6,out7,out8;
reg [8:0] o0,o1,o2,o3,o4,o5,o6,o7,o8;
integer i;
parameter IDLE=2'd0,OUT=2'd1;
reg [1:0] cs,ns;

always@(posedge clk or negedge rst_n) begin
	if(!rst_n) cs<=IDLE;
	else cs<=ns;
end

always@(*)begin
	case(cs)
	IDLE: begin
		if(in_valid) ns = OUT;
		else ns = IDLE;
	end
	OUT:begin
		ns = IDLE;
	end
	default:ns = IDLE;
	endcase
end


always@(*)begin
	if(cs == OUT) begin
		out_valid=1;
	end
	else begin
		out_valid=0;
	end

end
always@(*)begin
	if(cs == OUT ) begin
		o8[8]=0;
		o8[7]=a8[7];
		o8[6]=o8[7]^a8[6];
		o8[5]=o8[6]^a8[5];
		o8[4]=o8[5]^a8[4];
		o8[3]=o8[4]^a8[3];
		o8[2]=o8[3]^a8[2];
		o8[1]=o8[2]^a8[1];
		o8[0]=o8[1]^a8[0];
		if(a8[8]==1)begin
			out8 = o8 * (-1);
		end
		else begin
			out8 = o8 ;
		end
	end
	else for(i=0;i<9;i=i+1) begin
		o8[i]=0;
		out8[i]=0;
	end
end
always@(*)begin
	if(cs == OUT ) begin
		o7[8]=0;
		o7[7]=a7[7];
		o7[6]=o7[7]^a7[6];
		o7[5]=o7[6]^a7[5];
		o7[4]=o7[5]^a7[4];
		o7[3]=o7[4]^a7[3];
		o7[2]=o7[3]^a7[2];
		o7[1]=o7[2]^a7[1];
		o7[0]=o7[1]^a7[0];
		if(a7[8]==1)begin
			out7 = o7 * (-1);
		end
		else begin
			out7 = o7 ;
		end
	end
	else for(i=0;i<9;i=i+1) begin
		o7[i]=0;
		out7[i]=0;
	end
end
always@(*)begin
	if(cs == OUT ) begin
		o6[8]=0;
		o6[7]=a6[7];
		o6[6]=o6[7]^a6[6];
		o6[5]=o6[6]^a6[5];
		o6[4]=o6[5]^a6[4];
		o6[3]=o6[4]^a6[3];
		o6[2]=o6[3]^a6[2];
		o6[1]=o6[2]^a6[1];
		o6[0]=o6[1]^a6[0];
		if(a6[8]==1)begin
			out6 = o6 * (-1);
		end
		else begin
			out6 = o6 ;
		end
	end
	else for(i=0;i<9;i=i+1) begin
		 o6[i]=0;
		 out6[i]=0;
	end
end
always@(*)begin
	if(cs == OUT ) begin
		o5[8]=0;
		o5[7]=a5[7];
		o5[6]=o5[7]^a5[6];
		o5[5]=o5[6]^a5[5];
		o5[4]=o5[5]^a5[4];
		o5[3]=o5[4]^a5[3];
		o5[2]=o5[3]^a5[2];
		o5[1]=o5[2]^a5[1];
		o5[0]=o5[1]^a5[0];
		if(a5[8]==1)begin
			out5 = o5 * (-1);
		end
		else begin
			out5 = o5 ;
		end
	end
	else for(i=0;i<9;i=i+1) begin
		o5[i]=0;
		out5[i]=0;
	end
end
always@(*)begin
	if(cs == OUT ) begin
		o4[8]=0;
		o4[7]=a4[7];
		o4[6]=o4[7]^a4[6];
		o4[5]=o4[6]^a4[5];
		o4[4]=o4[5]^a4[4];
		o4[3]=o4[4]^a4[3];
		o4[2]=o4[3]^a4[2];
		o4[1]=o4[2]^a4[1];
		o4[0]=o4[1]^a4[0];
		if(a4[8]==1)begin
			out4 = o4 * (-1);
		end
		else begin
			out4 = o4 ;
		end
	end
	else for(i=0;i<9;i=i+1) begin
		o4[i]=0;
		out4[i]=0;
	end
end

always@(*)begin
	if(cs == OUT ) begin
		o3[8]=0;
		o3[7]=a3[7];
		o3[6]=o3[7]^a3[6];
		o3[5]=o3[6]^a3[5];
		o3[4]=o3[5]^a3[4];
		o3[3]=o3[4]^a3[3];
		o3[2]=o3[3]^a3[2];
		o3[1]=o3[2]^a3[1];
		o3[0]=o3[1]^a3[0];
		if(a3[8]==1)begin
			out3 = o3 * (-1);
		end
		else begin
			out3 = o3 ;
		end
	end
	else for(i=0;i<9;i=i+1) begin
		 o3[i]=0;
		 out3[i]=0;
	end
end

always@(*)begin
	if(cs == OUT ) begin
		o2[8]=0;
		o2[7]=a2[7];
		o2[6]=o2[7]^a2[6];
		o2[5]=o2[6]^a2[5];
		o2[4]=o2[5]^a2[4];
		o2[3]=o2[4]^a2[3];
		o2[2]=o2[3]^a2[2];
		o2[1]=o2[2]^a2[1];
		o2[0]=o2[1]^a2[0];
		if(a2[8]==1)begin
			out2 = o2 * (-1);
		end
		else begin
			out2 = o2 ;
		end
	end
	else for(i=0;i<9;i=i+1) begin
		o2[i]=0;
		out2[i]=0;
	end
end

always@(*)begin
	if(cs == OUT ) begin
		o1[8]=0;
		o1[7]=a1[7];
		o1[6]=o1[7]^a1[6];
		o1[5]=o1[6]^a1[5];
		o1[4]=o1[5]^a1[4];
		o1[3]=o1[4]^a1[3];
		o1[2]=o1[3]^a1[2];
		o1[1]=o1[2]^a1[1];
		o1[0]=o1[1]^a1[0];
		if(a1[8]==1)begin
			out1 = o1 * (-1);
		end
		else begin
			out1 = o1 ;
		end
	end
	else for(i=0;i<9;i=i+1) begin
		o1[i]=0;
		out1[i]=0;
	end
end
always@(*)begin
	if(cs == OUT ) begin
		o0[8]=0;
		o0[7]=a0[7];
		o0[6]=o0[7]^a0[6];
		o0[5]=o0[6]^a0[5];
		o0[4]=o0[5]^a0[4];
		o0[3]=o0[4]^a0[3];
		o0[2]=o0[3]^a0[2];
		o0[1]=o0[2]^a0[1];
		o0[0]=o0[1]^a0[0];
		if(a0[8]==1)begin
			out0 = o0 * (-1);
		end
		else begin
			out0 = o0 ;
		end
	end
	else for(i=0;i<9;i=i+1) begin
		 o0[i]=0;
		 out0[i]=0;
	end
end

endmodule

module ADDorSUB(
	clk,in_valid,rst_n,a0,a1,a2,a3,a4,a5,a6,a7,a8,
	out_valid,out0,out1,out2,out3,out4,out5,out6,out7,out8
);
input clk,rst_n,in_valid;
input signed [8:0] a0,a1,a2,a3,a4,a5,a6,a7,a8;
output reg out_valid;
output reg signed  [8:0]out0,out5,out8, out1,out2,out3,out4,out6,out7;
parameter IDLE=2'd0,LOAD=2'd1,SORT=3'd2,HALF=3'd3,PROCESS=3'd4,OUT=3'd5;
reg [2:0] cs,ns;
reg signed [8:0] value[0:8];
reg signed [8:0] half,mid;
wire signed [8:0] max,min,mid_w;
integer i;
reg in_valid_sort;
wire out_valid_sort;

always@(posedge clk or negedge rst_n) begin
	if(!rst_n) cs<=IDLE;
	else cs<=ns;
end


always@(*)begin
	case(cs)
	IDLE: begin
		if(in_valid) ns = LOAD;
		else ns = IDLE;
	end
	LOAD:begin
		ns = SORT;
	end
	SORT:begin
		if(out_valid_sort) ns = HALF;
		else ns = SORT;
	end
	HALF:begin
		ns = PROCESS;
	end
	PROCESS:begin
		ns = OUT;
	end
	OUT:begin
		ns = IDLE;
	end
	default:ns = IDLE;
	endcase
end
always@(posedge clk or negedge rst_n) begin
	if(!rst_n) for(i=0;i<9;i=i+1) value[i]<=0;
	else if(ns == IDLE ) for(i=0;i<9;i=i+1) value[i]<=0;
	else if( ns == LOAD ) begin
		value[0]<=a0;
		value[1]<=a1;
		value[2]<=a2;
		value[3]<=a3;
		value[4]<=a4;
		value[5]<=a5;
		value[6]<=a6;
		value[7]<=a7;
		value[8]<=a8;
	end
end

always@(posedge clk or negedge rst_n) begin
	if(!rst_n) in_valid_sort<=0;
	else if(ns == IDLE ) in_valid_sort<=0;
	else if( ns == SORT ) in_valid_sort<=1;
end

SORT_module sub0(.clk(clk),.in_valid(in_valid_sort),.rst_n(rst_n),.a0(value[0]),.a1(value[1]),.a2(value[2]),.a3(value[3]),.a4(value[4]),.a5(value[5]),.a6(value[6]),.a7(value[7]),.a8(value[8]),
	.out_valid(out_valid_sort),.out0(min),.out4(mid_w),.out8(max));

always@(posedge clk or negedge rst_n) begin
	if(!rst_n) half<=0;
	else if(ns == IDLE ) half<=0;
	else if( ns == HALF ) begin
		half<=(max-min)/2;///notsure
	end
	else half<=half;
end

always@(posedge clk or negedge rst_n) begin
	if(!rst_n) mid<=0;
	else if(ns == IDLE ) mid<=0;
	else if( ns == HALF ) begin
		mid<=(max+min)/2;
	end
	else mid<=mid;
end

always@(posedge clk or negedge rst_n)begin
	if(!rst_n) begin
		out0<=0;
		out1<=0;
		out2<=0;
		out3<=0;
		out4<=0;
		out5<=0;
		out6<=0;
		out7<=0;
		out8<=0;
	end
	else if(ns == IDLE ) begin
		out0<=0;
		out1<=0;
		out2<=0;
		out3<=0;
		out4<=0;
		out5<=0;
		out6<=0;
		out7<=0;
		out8<=0;
	end
	else if(ns == PROCESS) begin
		if(value[0] > mid ) out0<=value[0]-half;
		else if(value[0] < mid ) out0<= value[0]+half;
		else if(value[0] == mid) out0<=value[0];

		if(value[1] > mid ) out1<=value[1]-half;
		else if(value[1] < mid ) out1<= value[1]+half;
		else if(value[1] == mid) out1<=value[1];

		if(value[2] > mid ) out2<=value[2]-half;
		else if(value[2] < mid ) out2<= value[2]+half;
		else if(value[2] == mid) out2<=value[2];

		if(value[3] > mid ) out3<=value[3]-half;
		else if(value[3] < mid ) out3<= value[3]+half;
		else if(value[3] == mid) out3<=value[3];

		if(value[4] > mid ) out4<=value[4]-half;
		else if(value[4] < mid ) out4<= value[4]+half;
		else if(value[4] == mid) out4<=value[4];

		if(value[5] > mid ) out5<=value[5]-half;
		else if(value[5] < mid ) out5<= value[5]+half;
		else if(value[5] == mid) out5<=value[5];

		if(value[6] > mid ) begin
			out6<=value[6]-half;
		end
		else if(value[6] < mid ) out6<= value[6]+half;
		else if(value[6] == mid) out6<=value[6];

		if(value[7] > mid ) out7<=value[7]-half;
		else if(value[7] < mid ) out7<= value[7]+half;
		else if(value[7] == mid) out7<=value[7];

		if(value[8] > mid ) out8<=value[8]-half;
		else if(value[8] < mid ) out8<= value[8]+half;
		else if(value[8] == mid) out8<=value[8];
	end

end
always@(posedge clk or negedge rst_n) begin
	if(!rst_n) out_valid<=0;
	else if(ns == OUT) out_valid<=1;
	else out_valid<=0;
end
endmodule

module SMA(
	clk,in_valid,rst_n,a0,a1,a2,a3,a4,a5,a6,a7,a8,
	out_valid,out0,out1,out2,out3,out4,out5,out6,out7,out8
);
input clk,rst_n,in_valid;
input signed [8:0] a0,a1,a2,a3,a4,a5,a6,a7,a8;
output reg out_valid;
output  reg signed [8:0]out0,out5,out8,out1,out2,out3,out4,out6,out7;
parameter IDLE=2'd0,LOAD=2'd1,PROCESS=3'd2,OUT=3'd3;
reg [1:0] cs,ns;
reg signed [8:0] value[0:8];
integer i;
always@(posedge clk or negedge rst_n) begin
	if(!rst_n) cs<=IDLE;
	else cs<=ns;
end

always@(*)begin
	case(cs)
	IDLE: begin
		if(in_valid) ns = LOAD;
		else ns = IDLE;
	end
	LOAD:begin
		ns = PROCESS;
	end
	PROCESS:begin
		ns = OUT;
	end
	OUT:begin
		ns = IDLE;
	end
	default:ns = IDLE;
	endcase
end
always@(posedge clk or negedge rst_n) begin
	if(!rst_n) for(i=0;i<9;i=i+1) value[i]<=0;
	else if(ns == IDLE ) for(i=0;i<9;i=i+1) value[i]<=0;
	else if( ns == LOAD ) begin
		value[0]<=a0;
		value[1]<=a1;
		value[2]<=a2;
		value[3]<=a3;
		value[4]<=a4;
		value[5]<=a5;
		value[6]<=a6;
		value[7]<=a7;
		value[8]<=a8;
	end
end
always@(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		out0<=0;
		out1<=0;
		out2<=0;
		out3<=0;
		out4<=0;
		out5<=0;
		out6<=0;
		out7<=0;
		out8<=0;
	end
	else if(ns == IDLE ) begin
		out0<=0;
		out1<=0;
		out2<=0;
		out3<=0;
		out4<=0;
		out5<=0;
		out6<=0;
		out7<=0;
		out8<=0;
	end
	else if(ns == PROCESS)begin
		out0<=(value[8]+value[0]+value[1])/3;
		out1<=(value[0]+value[1]+value[2])/3;
		out2<=(value[1]+value[2]+value[3])/3;
		out3<=(value[2]+value[3]+value[4])/3;
		out4<=(value[3]+value[4]+value[5])/3;
		out5<=(value[4]+value[5]+value[6])/3;
		out6<=(value[5]+value[6]+value[7])/3;
		out7<=(value[6]+value[7]+value[8])/3;
		out8<=(value[7]+value[8]+value[0])/3;
	end
	else begin
		out0<=out0;
		out1<=out1;
		out2<=out2;
		out3<=out3;
		out4<=out4;
		out5<=out5;
		out6<=out6;
		out7<=out7;
		out8<=out8;
	end
end

always@(posedge clk or negedge rst_n) begin
	if(!rst_n) out_valid<=0;
	else if(ns == OUT) out_valid<=1;
	else out_valid<=0;
end

endmodule



module SORT_module(
	clk,in_valid,rst_n,a0,a1,a2,a3,a4,a5,a6,a7,a8,
	out_valid,out0,out4,out8
);
input clk,rst_n,in_valid;
input signed [8:0] a0,a1,a2,a3,a4,a5,a6,a7,a8;
output reg out_valid;
output  reg signed [8:0]out0,out4,out8;
reg signed [8:0] out1,out2,out3,out5,out6,out7;


parameter IDLE=2'd0,LOAD=2'd1,PROCESS=2'd2,OUT=2'd3;
reg [1:0] cs,ns;
reg [4:0] cnt;

always@(posedge clk or negedge rst_n) begin
	if(!rst_n) cs<=IDLE;
	else cs<=ns;
end

always@(*)begin
	case(cs)
	IDLE: begin
		if(in_valid) ns = LOAD;
		else ns = IDLE;
	end
	LOAD:begin
		ns = PROCESS;
	end
	PROCESS:begin
		if(cnt > 15) ns = OUT;
		else ns = PROCESS;
	end
	OUT:begin
		ns = IDLE;
	end
	default:ns = IDLE;
	endcase
end
always@(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		cnt<=0;
	end
	else if(ns == PROCESS) cnt<=cnt+1;
	else cnt<=0;

end

always@(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		out0<=0;
		out1<=0;
		out2<=0;
		out3<=0;
		out4<=0;
		out5<=0;
		out6<=0;
		out7<=0;
		out8<=0;
	end
	else if(ns == IDLE ) begin
		out0<=0;
		out1<=0;
		out2<=0;
		out3<=0;
		out4<=0;
		out5<=0;
		out6<=0;
		out7<=0;
		out8<=0;
	end
	else if(ns == LOAD) begin
		out0<=a0;
		out1<=a1;
		out2<=a2;
		out3<=a3;
		out4<=a4;
		out5<=a5;
		out6<=a6;
		out7<=a7;
		out8<=a8;
	end
	else begin
		if(out0 > out1 && cnt[0]==0)begin
			out1<=out0;
			out0<=out1;
		end
		if(out1 > out2 && cnt[0]==1)begin
			out2<=out1;
			out1<=out2;
		end
		if(out2 > out3 && cnt[0]==0)begin
			out3<=out2;
			out2<=out3;
		end
		if(out3 > out4 && cnt[0]==1)begin
			out4<=out3;
			out3<=out4;
		end
		if(out4 > out5 && cnt[0]==0)begin
			out5<=out4;
			out4<=out5;
		end
		if(out5 > out6 && cnt[0]==1)begin
			out6<=out5;
			out5<=out6;
		end
		if(out6 > out7 && cnt[0]==0)begin
			out7<=out6;
			out6<=out7;
		end
		if(out7 > out8 && cnt[0]==1)begin
			out8<=out7;
			out7<=out8;
		end
	end

end
always@(posedge clk or negedge rst_n) begin
	if(!rst_n) out_valid<=0;
	else if(ns == OUT) out_valid<=1;
	else out_valid<=0;
end
endmodule

