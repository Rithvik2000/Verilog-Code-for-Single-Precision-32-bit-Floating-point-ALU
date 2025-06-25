////////////// 001 = Addition
////////////// 010 = Subtraction
/////////////  011 = Multiplication
/////////////  100 = Division
////////////// 101 = Comparator  , 100(A>B) ,010 (A<B), 001(A=B)


module FP_ALU#(parameter N = 32, M = 23, P = N-M-1)(
  input[N-1:0] A,
  input[N-1:0] B,
  input[2:0] Sel,
  
   output reg [N-1:0] FINAL_OUT,
   output  reg Of,Uf
   
    );
    
    reg [N-1:0] a1,a2,m1,m2,d1,d2,c1,c2;
    wire [N-1:0] w1,w2,w3,w4,w5,w6,w7,w8,w9,w10,w11;
    wire [2:0] w12;
    
    assign w1 = a1;
    assign w2 = a2;
    assign w4 = m1;
    assign w5 = m2;
    assign w7 = d1;
    assign w8 = d2;
    assign w10 = c1;
    assign w11 = c2;
    
    Floating_Point_Multiplier #(N,M,P) M01 (w4,w5,w6,u1,o1);
    FloatAdder #(N,M,P) A01(w1,w2,w3,u2,o2);
    Floating_Point_Divider #(N,M,P) D01(w7,w8,w9,o3,u3);
    fpc #(N,M,P) C1(w10,w11,w12);
      
   always@(*) begin
   if (Sel == 1)
      begin
       a1 = A;
       a2 = B;
       FINAL_OUT = w3;
       Of = o2;
       Uf = u2;
      end
      
   else if (Sel == 2)
     begin
       a1 = A;
       a2 = {~B[N-1],B[N-2:0]};
       FINAL_OUT = w3;
       Of = o2;
       Uf = u2;
      end
   
   else if (Sel == 3)
       begin
       m1 = A;
       m2 = B;
       FINAL_OUT = w6;
       Of = o1;
       Uf = u1;
       end
   
   else if (Sel == 4)
     begin
       d1 = A;
       d2 = B;
       FINAL_OUT = w9;
       Of = o3;
       Uf = u3;
       end
   
   else if (Sel == 5)
     begin
       c1 = A;
       c2 = B;
       FINAL_OUT = {29'b0,w12};
       Of = 0;
       Uf = 0;
       end
   
   else
      FINAL_OUT = 0;
    
   end  
endmodule

/////////////////////   Floating Point Multipler ////////////

module Floating_Point_Multiplier #(parameter N = 32, M = 23, P = N-M-1)(
  input[N-1:0] A,
  input[N-1:0] B,
  
   output [N-1:0] FINAL_OUT,
   output UnderFlow,
   output OverFlow
   
    );
    
    wire[2*M+1:0] C;
    wire[1:0] C2;
    wire[N-2:0] OUT;
    
  
   //Multilying Mantissa
   Fixed_Point_Multipler M1({1'b1,A[M-1:0]},{1'b1,B[M-1:0]},C);
   
   // Shifting Operation, 2M+1 bit output from multiplier, select M-bits on MSB
   // starting after the first 1
     assign OUT[M-1:0]  = (C[2*M+1])? C[2*M:M+1]:C[2*M-1:M];
     
     
   //  Adding the Exponents
   Adder_1 A1(A[N-2:M],B[N-2:M],C[2*M+1],{C2,OUT[N-2:M]});
   
   
  // overflow and underflow
  assign OverFlow = (( ~C2[1] ) & C2[0]);
  assign UnderFlow = C2[1] ;
  
  // Sign bit Computation
  assign FINAL_OUT[N-1] = A[N-1]^B[N-1];
  
  // 3-input MUX
  assign FINAL_OUT[N-2:0] = OverFlow == 1 ? {N{1'b1}}:( UnderFlow ? {N{1'b0}}: OUT); 
     
endmodule

//M = 10,23,52
module Fixed_Point_Multipler #(parameter M = 23)(
input[M:0] A,
input[M:0] B,
output [(2*M)+1:0] OUT
);

assign OUT = A*B;
endmodule

// p =5,8,11
module Adder_1 #(parameter P = 8, Bi = -127)( //  -15,-127,-1023
input[P-1:0] A,
input[P-1:0] B,
input Shift,
output [P+1:0] OUT
);

wire[P+1:0] Bias;

assign Bias = Bi ;  
assign OUT = A+B+Bias+(Shift);
endmodule


////////////////////   Floating Point Adder  ////////////////


module FloatAdder #(parameter N = 32, M = 23, P = N-M-1)(
  input[N-1:0] A,
  input[N-1:0] B,
  
   output reg [N-1:0] FINAL_OUT,
   output reg UnderFlow_Ad,
   output reg OverFlow_Ad
    );
    
    wire[M-1:0] M1,M2;
    wire[P+1:0] E1,E2;
    reg S;
    reg[P+1:0] UnFlow1;
    reg[P+1:0] UnFlow2;
    
    reg [P-1:0] shift;
    reg [M:0] a1,b1;
     
    wire[M+2:0] in;
    reg [M+2:0] C1;
   
    reg [4:0]out;  
      
    assign M1 = A[M-1:0];
    assign M2 = B[M-1:0];
    assign E1 = A[N-2:M];
    assign E2 = B[N-2:M];
    assign S1 = A[N-1];
    assign S2 = B[N-1];
    
    
   always@(*) begin
    //COMPARE EXPONENT
      if (E1 > E2)
        begin
        shift = E1 - E2;
        a1 = {1'b1,M1};
        if (shift <= 23)
            b1 ={1'b1,M2}>>(shift);
        else
           b1 = 0;
        S = A[N-1];
        end
       
      else if (E2 > E1)
        begin
        shift = E2 - E1;
        b1 = {1'b1,M2};
        if (shift <= 23)
            a1 = {1'b1,M1}>>(shift);
        else
           a1 = 0;
        S = B[N-1];
        end  
        
        else // e1 == e2
        begin
        
        b1 = {1'b1,M2};
        a1 = {1'b1,M1};
         
      
            if (M1 > M2)
            begin
            S = A[N-1];
            end
            
          else if (M2 > M1)
            begin
            S = B[N-1];
            end
            
          else
            S = B[N-1];
        
      end
  end
    
     
     //Adding Mantissa
   Fixed_Point_Adder F1  ( {A[N-1],1'b0,a1},   {B[N-1], 1'b0,b1},  in);
  
   
    always@(*) begin 
 
    out = 5'b00000;
 
     if (in[24]) begin
        out = 5'b11000;
        
    end else if (in[23]) begin
        out = 5'b10111;
        
    end else if (in[22]) begin
        out = 5'b10110;
        
    end else if (in[21]) begin
        out = 5'b10101;
        
    end else if (in[20]) begin
        out = 5'b10100;
        
    end else if (in[19]) begin
        out = 5'b10011;
        
    end else if (in[18]) begin
        out = 5'b10010;
        
    end else if (in[17]) begin
        out = 5'b10001;
        
    end else if (in[16]) begin
        out = 5'b10000;
        
    end else if (in[15]) begin
        out = 5'b01111;
        
    end else if (in[14]) begin
        out = 5'b01110;
        
    end else if (in[13]) begin
        out = 5'b01101;
        
    end else if (in[12]) begin
        out = 5'b01100;
        
    end else if (in[11]) begin
        out = 5'b01011;
        
    end else if (in[10]) begin
        out = 5'b01010;
        
    end else if (in[9]) begin
        out = 5'b01001;
        
    end else if (in[8]) begin
        out = 5'b01000;
        
    end else if (in[7]) begin
        out = 5'b00111;
        
    end else if (in[6]) begin
        out = 5'b00110;
       
    end else if (in[5]) begin
        out = 5'b00101;
        
    end else if (in[4]) begin
        out = 5'b00100;
       
    end else if (in[3]) begin
        out = 5'b00011;
       
    end else if (in[2]) begin
        out = 5'b00010;
        
    end else if (in[1]) begin
        out = 5'b00001;
        
    end else if (in[0]) begin
        out = 5'b00000;
      
    end

      FINAL_OUT[N-1] = S;
      
      UnFlow1 = (E1 - (23-out));
      UnFlow2 = (E2 - (23-out));
      
         if ((out) == 24)
           C1 =  in >> 1;
         else
          C1 =  in << (23-out);
        
     
     //overflow and underflow
      
     
       if ((E1 >= E2) && (UnFlow1[P+1] == 0) && (UnFlow1[P] == 1))
         begin
          FINAL_OUT[N-2:0] = 31'b1111111111111111111111111111111;
          OverFlow_Ad = 1;
         end
         
       else if ((E1 >= E2)&&(UnFlow1[P+1] == 1))
         begin
          FINAL_OUT[N-2:0] = 0;
          UnderFlow_Ad = 1;
         end
         
      
     
        else if ((E2 > E1 )&& (UnFlow2[P+1] == 0) && (UnFlow2[P] == 1))
         begin
          FINAL_OUT[N-2:0] = 31'b1111111111111111111111111111111;
          OverFlow_Ad = 1;
         end
         
        else if ((E2 > E1)&&(UnFlow2[P+1] == 1))
         begin
          FINAL_OUT[N-2:0] = 0;
          UnderFlow_Ad = 1;
         end
         
        else if (E1 == 0 && E2 == 0 && M1 == 0 && M2 == 0)
        begin
          FINAL_OUT[N-2:0] = 0;
           UnderFlow_Ad = 0;
           OverFlow_Ad = 0;
        end
         
       else
         begin
         FINAL_OUT[M-1:0] = C1[M-1:0];
         FINAL_OUT[N-2:M] =(E1>=E2) ?(E1 - (23-out) ):(E2 - (23-out) );
         UnderFlow_Ad = 0;
         OverFlow_Ad = 0;
         end
      
    
end

endmodule



module Fixed_Point_Adder #(parameter M = 23)(
input[M+2:0] A,
input[M+2:0] B,
output reg [M+2:0] OUT
);
 
 reg [M+1:0] a11,b11;
 
 always@(*) begin 
  if (A[M+2] == B[M+2]) 
     OUT = A + B;
  else if (A[M+1:0] > B[M+1:0])
     OUT = A - B;
     else
     OUT = B-A;
    
 end 
endmodule




///////////////// Floating point Divider ///////////////

module Floating_Point_Divider #(parameter N = 32, M = 23,P = N-M-1)(
  input[N-1:0] A,
  input[N-1:0] B,
  
  output reg [N-1:0] OUT, 
  output reg OverFlow,
  output reg UnderFlow
  );
  
  wire[M:0] C1;
 
  reg[P+1:0] temp_exp;
  reg[P+1:0] bias = 127;
  
  reg[P+1:0] temp_Shift;
    
    Fixed_Point_Divider D1({1'b1,A[M-1:0]},{1'b1,B[M-1:0]},C1);
    
    always@(*) begin
    
        OUT[N-1] = A[N-1]^B[N-1];
        
        
        temp_Shift = -( {9'b0,(~C1[M])} + B[N-2:M] + bias +2 );
        
        temp_exp = A[N-2:M] + temp_Shift;
        
        if (temp_exp[P+1] == 0 && temp_exp[P] == 1)
          begin
             OverFlow = 1;
             UnderFlow =0;
             OUT[N-2:0] = 31'b1111111111111111111111111111111;
          end
        else if (temp_exp[P+1] == 1)
          begin
             OverFlow = 0;
             UnderFlow = 1;
             OUT[N-2:0] = 0;
          end
        else
             OverFlow = 0;
             UnderFlow = 0;
             OUT[N-2:M]= temp_exp;
             if (C1[M] == 0)
                 OUT[M-1:0] = {C1[M-2:0],1'b0};
             else
                 OUT[M-1:0] = C1[M-1:0];
        
    end
    
endmodule



//// INPUT INCLUDING 1 to MANTISSA//////

module Fixed_Point_Divider #(parameter M = 23)(
  input[M:0] A,
  input[M:0] B,
  
  output reg [M:0] Q_out 
  );
    
reg [M+1:0] temp_A;
reg [M:0] Q = 0;
reg [M+2:0] diff;

integer i;
always@(*) begin
temp_A = {1'b0,A};

 if(B ==  A)
         begin
            Q = {1'b1,{23{1'b0}}};
         end
 else begin
 for(i=24;i>=1;i=i-1)
  begin
      diff =  temp_A -  {1'b0,B} ;
     if( diff[M+1] == 1)
         begin
            Q[i-1] = 0;
           
         end
     else 
         begin
            Q[i-1] = 1;
            temp_A = (diff);
         end 
         
         
      temp_A = (temp_A << 1);
  end
  
   Q_out = Q;
end
end

endmodule


/////////////////// Floating Point Comparator //////////////
module fpc #(parameter N=32,M=23,EB=N-M-1)(
input[N-1:0] A1,B1,
output[2:0] out 
);

wire sign_A,sign_B;
wire [N-2:M] exp_A,exp_B;
wire [M-1:0] Mant_A,Mant_B;
wire [EB:0] exp_diff;
wire [M:0] Mant_diff;
reg g,l,e;


assign exp_diff = A1[N-2:M] - B1[N-2:M];
assign Mant_diff = A1[M-1:0] - B1[M-1:0];
assign sign_A = A1[N-1];
assign sign_B = B1[N-1];
assign exp_A = A1[N-2:M];
assign exp_B = B1[N-2:M];
assign Mant_A = A1[M-1:0];
assign Mant_B = B1[M-1:0];


always@(*) begin
    if((sign_A^sign_B))
        begin
            if(!sign_A && sign_B)
                begin
                g = 1'b1;
                l = 1'b0;
                e = 1'b0;
                end
            else
                begin
                g = 1'b0;
                l = 1'b1;
                e = 1'b0;
                end
        end
    else
         begin
             if(!(exp_A^exp_B))
                   begin
                     if(!(Mant_A^Mant_B))
                            begin
                            g = 1'b0;
                            l = 1'b0;
                            e = 1'b1;
                            end
                      else
                            begin
                            g = (Mant_diff[M])?1'b0:1'b1;
                            l= (Mant_diff[M])?1'b1:1'b0;
                            e = 1'b0;
                            end
                    end
            else
                begin
                g = (exp_diff[EB])?1'b0:1'b1;
                l= (exp_diff[EB])?1'b1:1'b0;
                e = 1'b0;
                end
        end
end


assign out = {g,l,e};

endmodule