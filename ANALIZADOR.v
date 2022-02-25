`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
/*
MIT License

Copyright (c) 2022 Antonio Sánchez (@TheSonders)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
//////////////////////////////////////////////////////////////////////////////////
module ANALIZADOR (
    input wire CLK50,
    input wire [15:0]GPIO,
    input wire BUTTON,
    output wire SERIAL_TX,
    output reg LED=0
    );

wire CLK120;
wire PLL_LOCKED;
wire startButton;

localparam IDLE=2'b00;
localparam SAMPLING=2'b01;
localparam SENDING=2'b10;

reg memWrite=0;
reg [13:0]memAddress=0;
reg [15:0]Sample=0;
reg [15:0]prev_GPIO=0;
reg [15:0]prev_prev_GPIO=0;
wire [31:0]memData;
reg [1:0]stm=IDLE;
reg [1:0]nbyte=0;
reg [7:0]TXData=0;
reg TXSend=0;
wire TXReady;

always @(posedge CLK120)begin
    case (stm)
        IDLE:begin
                if (startButton==1)begin
                    memAddress<=0;
                    memWrite<=1;
                    Sample<=0;
                    LED<=1;
                    stm<=SAMPLING;
                    prev_GPIO<=GPIO;
                end
            end
        SAMPLING:begin
                prev_GPIO<=GPIO;
                prev_prev_GPIO<=prev_GPIO;
                if (prev_prev_GPIO!=prev_GPIO)begin
                    if (memAddress==14'h3FFF)begin
                        stm<=SENDING;
                        memWrite<=0;
                        memAddress<=0;
                    end
                    else begin
                        Sample<=0;
                        memAddress<=memAddress+1;
                        memWrite<=1;
                    end
                end
                else begin
                    if (Sample==16'hFFFF)begin
                        if (memAddress==14'h3FFF)begin
                            stm<=SENDING;
                            memWrite<=0;
                            memAddress<=0;
                        end
                        else begin
                            Sample<=0;
                            memAddress<=memAddress+1;
                            memWrite<=1;
                        end
                    end
                    else begin
                        Sample<=Sample+1;
                    end
                end
            end
        SENDING:begin
                if (TXSend==1)TXSend<=0;
                if (TXReady==1)begin
                    nbyte<=nbyte+1;
                    case (nbyte)
                        0:begin
                            TXData<=memData[31:24];
                            TXSend<=1;
                        end
                        1:begin
                            TXData<=memData[23:16];
                            TXSend<=1;
                        end
                        2:begin
                            TXData<=memData[15:8];
                            TXSend<=1;
                        end
                        3:begin
                            TXData<=memData[7:0];
                            if (memAddress==14'h3FFF)begin
                                memAddress<=0;
                                stm<=IDLE;
                                LED<=0;
                            end
                            else begin
                                memAddress<=memAddress+1;
                                LED<=memAddress[8];
                                TXSend<=1;
                            end
                        end
                    endcase
                end
            end
    endcase
end

PLL120 PLL120 (
    .CLK50(CLK50),
    .CLK120(CLK120),
    .LOCKED(PLL_LOCKED));
    
BUTTON_DEB BUTTON_DEB(
    .clk(CLK120),
    .button_i(BUTTON),
    .button_o(startButton));
    
MEMORY MEMORY(
    .clk(CLK120),
    .we(memWrite),
    .add(memAddress),
    .data_i({prev_GPIO,Sample}),
    .data_o(memData)); 
    
SERIALTX 
    #(.CLKFREQ(120_000_000),
    .BAUDRATE(38400))
    SERIALTX (
    .clk(CLK120), 
    .TXSend(TXSend), 
    .TXData(TXData), 
    .SERIAL_TX(SERIAL_TX), 
    .TXReady(TXReady)
    );    
endmodule

module BUTTON_DEB(
    input wire clk,
    input wire button_i,
    output reg button_o=0);

localparam RISE_EDGE=4'b0011;
localparam LOW_STATE=4'b0000;
    
reg [3:0]rbutton=0;
reg released=0;

always @(posedge clk) begin
    rbutton<={rbutton[2:0],button_i};
    if (button_o==1)begin
        button_o<=0;
    end
    else begin
        if (rbutton==RISE_EDGE && released==1)begin
            button_o<=1;
            released<=0;
        end
        else if (rbutton==LOW_STATE)begin
            released<=1;
        end
    end
end
endmodule

module MEMORY(
    input wire clk,
    input wire we,
    input wire [13:0]add,
    input wire [31:0]data_i,
    output reg [31:0]data_o);
    
reg [31:0]memory[0:16383];

always @(posedge clk)begin
    if (we==1)memory[add]<=data_i;
    data_o<=memory[add];
end
endmodule

module SERIALTX
    #(parameter CLKFREQ=120_000_000,
    parameter BAUDRATE=38400)(
    input wire clk,
    input wire TXSend,
    input wire [7:0]TXData,
    output reg SERIAL_TX=1,
    output wire TXReady);
    
localparam RELOAD=(CLKFREQ/BAUDRATE)-1;
localparam IDLE=0;
localparam STARTBIT=1;
localparam STOPBIT=10;
localparam IDLEBIT=13;
localparam EVEN=0;
localparam ODD=1;

assign TXReady=(stm==IDLE && TXSend==0);
reg [$clog2(RELOAD)-1:0]prescaler=0;
reg [$clog2(IDLEBIT)-1:0]stm=IDLE;
reg [7:0]shfData=0;

always @(posedge clk)begin
    if (stm==IDLE)begin
         if (TXSend==1)begin
            shfData<=TXData;
            stm<=stm+1;
         end
    end
    else begin
        if (prescaler==0)begin
            prescaler<=RELOAD;
            case (stm)
                STARTBIT:begin
                        stm<=stm+1;
                        SERIAL_TX<=0;
                    end
                IDLEBIT:begin
                        stm<=IDLE;
                    end               
                default:begin
                        {shfData,SERIAL_TX}<={1'b1,shfData};
                        stm<=stm+1;
                    end
            endcase
        end
        else begin
            prescaler<=prescaler-1;
        end   
    end
end
endmodule
