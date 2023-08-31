module pratica2(
	input PClock,
	input ResetIn,
	input Run,
	output [15:0] bus,
	output [15:0] Reg0,
	output [15:0] Reg1,
	output [15:0] Reg2,
	output [15:0] Reg3,
	output [15:0] Reg4,
	output [15:0] Reg5,
	output [15:0] Reg6,
	output [15:0] Reg7
);

wire done;

wire [4:0] index;
wire [15:0] DIN;

CounterPC PC (done,ResetIn, index);
romLPM2 Memoria (index,~done,DIN);
proc processador(PClock,DIN,ResetIn,Reg0,Reg1,Reg2,Reg3,Reg4,Reg5,Reg6,Reg7, bus, done);

endmodule


module CounterPC(
    input Clock,
    input Clear,            //Habilita a execução do Counter.
    output reg [4:0]Counter //Contador para que não ocorra sobreposição de instruções.
);
	
	initial begin
		Counter = 5'b00000;
	end
	
    always @(posedge Clock) begin   //Em cada ciclo de clock:
        if(Clear)   //Se o sistema for reiniciado ou atingir o número máximo de ciclos de clock Counter recebe 0.
            Counter <= 3'b00000;
        else    //Caso não, incrementa Counter em 1.
            Counter <= Counter + 3'b00001;
    end
endmodule

module proc(
	input Clock,
	input [15:0]DIN,
	input Reset, //Reinicia o sistema.
	output [15:0] R0_output,
	output [15:0] R1_output,
	output [15:0] R2_output, 
	output [15:0] R3_output,
	output [15:0] R4_output,
	output [15:0] R5_output, 
	output [15:0] R6_output,
	output [15:0] R7_output,
	output reg[15:0] BusWires,
	output Done     //Informa o fim da instrução.
); 
	wire [2:0]Counter;
	wire [15:0]A_output, W_output, RNin;
	wire [9:0]IR_output;
	wire [10:0]IRin;
	wire incr_pc;
	wire [7:0]RNout;
	wire Ain, Gin, Gout, DINout; 
	wire [2:0]ALUop;
	//Saída dos registradores R0 a R7.
	wire [15:0] G_output, ALU_output;
	 
	wire Clear = Done | Reset;
  
    //Unidade Counter
	Counter C(Clock, Clear, Counter);
    
	wire [7:0]XXX, YYY;
	RegUpcode X(DIN[5:3], XXX);
   RegUpcode Y(DIN[2:0], YYY); 
   
	ControlUnit CU(IR_output, Counter, XXX, YYY, G_output, IRin, RNout, RNin, incr_pc, Ain, Gin, Gout, DINout, ALUop, /*ADDRin, RAMout, DOUTin, W_D,*/ Done);
	 	  
	ALU alu(A_output, BusWires, ALUop,ALU_output);
	 	 
    //Registradores
	RegN G(Clock, Gin, ALU_output, G_output);
	Reg0 R0(Clock, RNin[0], BusWires, R0_output);
	Reg1 R1(Clock, RNin[1], BusWires, R1_output);
	Reg2 R2(Clock, RNin[2], BusWires, R2_output);
	RegN R3(Clock, RNin[3], BusWires, R3_output);
	RegN R4(Clock, RNin[4], BusWires, R4_output);
	RegN R5(Clock, RNin[5], BusWires, R5_output);
	RegN R6(Clock, RNin[6], BusWires, R6_output);
	RegN R7(Clock, RNin[7], BusWires, R7_output);
	RegN A(Clock, Ain, BusWires, A_output);

	RegIR IR(Clock, IRin, DIN, IR_output);
     
	 
   // Multiplexers M(MUXop, DIN, RNout, Gout, DINout, G_output, R0_output, R1_output,R2_output, R3_output, R4_output, R5_output, R6_output, R7_output, BusWires);
	always @(RNout, Gout, DINout) begin
			  case({RNout,Gout,DINout})
					10'b0000000001: BusWires <= DIN;
					10'b0000000010: BusWires <= G_output;
					10'b0000000100: BusWires <= R0_output;
					10'b0000001000: BusWires <= R1_output;
					10'b0000010000: BusWires <= R2_output;
					10'b0000100000: BusWires <= R3_output;
					10'b0001000000: BusWires <= R4_output;
					10'b0010000000: BusWires <= R5_output;
					10'b0100000000: BusWires <= R6_output;
					10'b1000000000: BusWires = R7_output;
			  endcase     
    end
    
endmodule

//Modulo que converte um número binário de 3 bits para um número decimal.
module RegUpcode(	
   input [2:0] reg_input,
   output reg[7:0]reg_output
);

	always @(reg_input) begin
		case (reg_input)
			3'b000: reg_output = 8'b00000001;
			3'b001: reg_output = 8'b00000010;
			3'b010: reg_output = 8'b00000100;
			3'b011: reg_output = 8'b00001000;
			3'b100: reg_output = 8'b00010000;
			3'b101: reg_output = 8'b00100000;
			3'b110: reg_output = 8'b01000000;
			3'b111: reg_output = 8'b10000000;
		endcase
	end
endmodule


module ControlUnit(
	input [9:0]IR,          //Define a instrução executada.
	input [2:0]Counter,     //Contador para que não ocorra sobreposição de instruções.
	input [7:0]XXX,         //Define o reg XXX da instrução.
	input [7:0]YYY,         //Define o reg XXX da instrução.    
	input [15:0] G_output,         //Define a saída de G.    
	output reg [10:0]IRin,  //Habilita a escrita em IR.    
	output reg [7:0]RNout,  //Define se a saída dos reg R0 a R7 seram utilizados.
	output reg [7:0]RNin,   //Habilita a escrita de dados nos reg R0 a R7.
	output reg incr_pc,     //Habilita a escrita no reg R7: incremento do PC.    
	output reg Ain,         //Habilita o uso do registrador A. 
	output reg Gin,         //Habilita o uso do registrador G.
	output reg Gout,
	output reg DINout,      //Define se a próxima IR será uma instrução ou dado da chamada imediata.    
	output reg[2:0]ALUOp,   //Define a operação da ULA.    
	output reg Done     	 //Informa o término da instrução.
);  
    
	wire[2:0] ADD = 3'b000, SUB = 3'b001, OR = 3'b010, SLT = 3'b011, SLL = 3'b100, SRL = 3'b101, MV = 3'b110, MVI = 3'b111; 

	always @(Counter or IR or XXX or YYY) begin
    //Especificação de valores para todo início de execução de instrução.
	 IRin = 1'b0;
	 RNout[7:0] = 8'b00000000;
	 RNin[7:0] = 8'b00000000;
	 DINout = 1'b0;
	 Ain = 1'b0;
	 Gin = 1'b0;
	 Gout = 1'b0;
	 incr_pc = 1'b0;
	 Done = 1'b0;
	 
		case (Counter)
			3'b000: begin
				IRin = 1'b1;    //Habilita a escrita em IR.
			end
			3'b001: begin
				case (IR[8:6]) 
					ADD, SUB, OR, SLT: begin  //Instrução add, sub, OR, slt.
						RNout = XXX;    //Define o registrador de leitura XXX.
						Ain = 1'b1;     //Habilita a escrita no reg A.
					end
					SLL: begin //Instrução sll.
						RNout = XXX;    //Define o registrador de leitura XXX.
						Ain = 1'b1;     //Habilita a escrita no reg A.
						ALUOp = 3'b011; //Define a operação de sll na ULA.
					end
					SRL: begin  //Instrução srl.
						RNout = XXX;    //Define o registrador de leitura XXX.
						Ain = 1'b1;     //Habilita a escrita no reg A.
						ALUOp = 3'b111; //Define a operação de srl na ULA.
					end
               MV: begin //Instrução mv: XXX recebe o dado contido em YYY.
                  RNout = YYY;
                  RNin = XXX;     //Habilita a escrita no reg XXX.
               end		  
				endcase
			end
			3'b010: begin
				case (IR[8:6])
					ADD: begin  //Instrução add.
						RNout = YYY;    //Define o registrador de leitura YYY.
						Gin = 1'b1;     //Habilita a escrita no reg G.
						ALUOp = 3'b000; //Define a operação de add na ULA.						
					end 
					SUB: begin   //Instrução sub.
						RNout = YYY;    //Define o registrador de leitura YYY.
						Gin = 1'b1;     //Habilita a escrita no reg G.
						ALUOp = 3'b001; //Define a operação de sub na ULA.
					end
					OR: begin  //Instrução OR.
						RNout = YYY;    //Define o registrador de leitura YYY.
						Gin = 1'b1;     //Habilita a escrita no reg G.
						ALUOp = 3'b010; //Define a operação de OR na ULA.
					end
					SLT: begin  //Instrução slt.
						RNout = YYY;    //Define o registrador de leitura YYY.
						Gin = 1'b1;     //Habilita a escrita no reg G.
						ALUOp = 3'b101; //Define a operação de slt na ULA.
					end
					SLL, SRL: begin //Instrução sll e srl.
						Gin = 1'b1;
						RNout = YYY;
               end
					MV: begin
						Done = 1'b1;    //Define o término da instrução.
					end
					MVI: begin
						RNin = XXX;     //Habilita a escrita no reg XXX.
						DINout = 1'b1;
					end
				endcase
			end
			3'b011: begin		
				case (IR[8:6]) 
					ADD, SUB, OR, SLT: begin    //Instrução add, sub, OR, slt.
						Gout = 1'b1;    //Define que o dado de escrita é da saída da ULA.
						RNin = XXX;     //Habilita a escrita no reg XXX.
						Done = 1'b1;    //Define o término da instrução.
					end
					SRL, SLL: begin						
						Gout = 1'b1;    //Define que o dado de escrita é da saída da ULA.
						RNin = XXX;
						Done = 1'b1;    //Define o término da instrução.
					end
					MVI: begin
						Done = 1'b1;    //Define o término da instrução.
					end
				endcase
			end
       endcase        
    end //end always
endmodule

module ALU(
    input [15:0]A,
    input [15:0]Bus, 
    input [2:0] ALUop,
    output reg [15:0]ALU_output
);
    always @(ALUop, Bus, A) begin
        case(ALUop)
            3'b000: ALU_output <= A + Bus;      //Add.
            3'b001: ALU_output <= A - Bus;      //Sub.
            3'b010: ALU_output <= (A | Bus);     //OR
            3'b101: ALU_output <= A < Bus ? 1 : 0; //slt
            3'b011: ALU_output <= A << Bus;    //SLL.
            3'b111: ALU_output <= A >> Bus;    //SLR
        endcase  
		end  
endmodule

module Counter(
    input Clock,
    input Clear,            //Habilita a execução do Counter.
    output reg [2:0]Counter //Contador para que não ocorra sobreposição de instruções.
);
    always @(posedge Clock) begin   //Em cada ciclo de clock:
        if(Clear)   //Se o sistema for reiniciado ou atingir o número máximo de ciclos de clock Counter recebe 0.
            Counter <= 3'b000;
        else    //Caso não, incrementa Counter em 1.
            Counter <= Counter + 3'b001;
    end
endmodule

module RegN(
    input Clock,
    input Rin,  //Habilita a escrita de dados no reg RN.
    input [15:0]Bus,    //Dado de escrita em RN.
    output reg[15:0]R_output  //Dado de saída de RN.
);  
	initial begin
		R_output = 16'b0000_0000_0000_0000;
	end
	
	always @(posedge Clock) begin
			if(Rin) begin
				R_output = Bus;
			end
    end
endmodule

module Reg0(
    input Clock,
    input Rin,  //Habilita a escrita de dados no reg RN.
    input [15:0]Bus,    //Dado de escrita em RN.
    output reg[15:0]R_output  //Dado de saída de RN.
);  
	initial begin
		R_output = 16'b0000_0000_0000_0010;
	end
	
	always @(posedge Clock) begin
			if(Rin) begin
				R_output = Bus;
			end
    end
endmodule

module Reg1(
    input Clock,
    input Rin,  //Habilita a escrita de dados no reg RN.
    input [15:0]Bus,    //Dado de escrita em RN.
    output reg[15:0]R_output  //Dado de saída de RN.
);  
	initial begin
		R_output = 16'b0000_0000_0000_0011;
	end
	
	always @(posedge Clock) begin
			if(Rin) begin
				R_output = Bus;
			end
    end
endmodule

module Reg2(
    input Clock,
    input Rin,  //Habilita a escrita de dados no reg RN.
    input [15:0]Bus,    //Dado de escrita em RN.
    output reg[15:0]R_output  //Dado de saída de RN.
);  
	initial begin
		R_output = 16'b0000_0000_0000_0110;
	end
	
	always @(posedge Clock) begin
			if(Rin) begin
				R_output = Bus;
			end
    end
endmodule

module RegIR(
    input Clock,
    input Rin, 						//Habilita a escrita de dados no reg IR.
    input [15:0]Bus,    			//Dado de escrita em IR.
    output reg [9:0]R_output   	//Dado de saída de IR.
);
    always @(posedge Clock) begin
        if(Rin) begin
				R_output = Bus[9:0];
			end
    end
endmodule



