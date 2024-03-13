//
// User core top-level
//
// Instantiated by the real top-level: apf_top
//

`default_nettype none

module core_top (

//
// physical connections
//

///////////////////////////////////////////////////
// clock inputs 74.25mhz. not phase aligned, so treat these domains as asynchronous

input 	wire 				clk_74a, // mainclk1
input 	wire 				clk_74b, // mainclk1 

input 	wire				reset_l_main,

inout		wire				bridge_spimosi,
inout		wire				bridge_spimiso,
inout		wire				bridge_spiclk,
input		wire				bridge_spiss,
inout		wire				bridge_1wire,

///////////////////////////////////////////////////
// cartridge interface
// switches between 3.3v and 5v mechanically
// output enable for multibit translators controlled by pic32

// GBA AD[15:8]
inout		wire	[7:0]		cart_tran_bank2,
output	wire				cart_tran_bank2_dir,

// GBA AD[7:0]
inout		wire	[7:0]		cart_tran_bank3,
output	wire				cart_tran_bank3_dir,

// GBA A[23:16]
inout		wire	[7:0]		cart_tran_bank1,
output	wire				cart_tran_bank1_dir,

// GBA [7] PHI#
// GBA [6] WR#
// GBA [5] RD#
// GBA [4] CS1#/CS#
//     [3:0] unwired
inout		wire	[7:4]		cart_tran_bank0,
output	wire				cart_tran_bank0_dir,

// GBA CS2#/RES#
inout		wire				cart_tran_pin30,
output	wire				cart_tran_pin30_dir,
// when GBC cart is inserted, this signal when low or weak will pull GBC /RES low with a special circuit
// the goal is that when unconfigured, the FPGA weak pullups won't interfere.
// thus, if GBC cart is inserted, FPGA must drive this high in order to let the level translators
// and general IO drive this pin.
output	wire				cart_pin30_pwroff_reset,

// GBA IRQ/DRQ
inout	wire					cart_tran_pin31,
output	wire				cart_tran_pin31_dir,

// infrared
input		wire				port_ir_rx,
output	wire				port_ir_tx,
output	wire				port_ir_rx_disable, 

// GBA link port
inout		wire				port_tran_si,
output	wire				port_tran_si_dir,
inout		wire				port_tran_so,
output	wire				port_tran_so_dir,
inout		wire				port_tran_sck,
output	wire				port_tran_sck_dir,
inout		wire				port_tran_sd,
output	wire				port_tran_sd_dir,
 
///////////////////////////////////////////////////
// cellular psram 0 and 1, two chips (64mbit x2 dual die per chip)

output	wire	[21:16]	cram0_a,
inout		wire	[15:0]	cram0_dq,
input		wire				cram0_wait,
output	wire				cram0_clk,
output	wire				cram0_adv_n,
output	wire				cram0_cre,
output	wire				cram0_ce0_n,
output	wire				cram0_ce1_n,
output	wire				cram0_oe_n,
output	wire				cram0_we_n,
output	wire				cram0_ub_n,
output	wire				cram0_lb_n,

output	wire	[21:16]	cram1_a,
inout		wire	[15:0]	cram1_dq,
input		wire				cram1_wait,
output	wire				cram1_clk,
output	wire				cram1_adv_n,
output	wire				cram1_cre,
output	wire				cram1_ce0_n,
output	wire				cram1_ce1_n,
output	wire				cram1_oe_n,
output	wire				cram1_we_n,
output	wire				cram1_ub_n,
output	wire				cram1_lb_n,

///////////////////////////////////////////////////
// sdram, 512mbit 16bit

output	wire	[12:0]	dram_a,
output	wire	[1:0]		dram_ba,
inout		wire	[15:0]	dram_dq,
output	wire	[1:0]		dram_dqm,
output	wire				dram_clk,
output	wire				dram_cke,
output	wire				dram_ras_n,
output	wire				dram_cas_n,
output	wire				dram_we_n,

///////////////////////////////////////////////////
// sram, 1mbit 16bit

output	wire	[16:0]	sram_a,
inout		wire	[15:0]	sram_dq,
output	wire				sram_oe_n,
output	wire				sram_we_n,
output	wire				sram_ub_n,
output	wire				sram_lb_n,

///////////////////////////////////////////////////
// vblank driven by dock for sync in a certain mode

input	wire					vblank,

///////////////////////////////////////////////////
// i/o to 6515D breakout usb uart

output	wire				dbg_tx,
input	wire					dbg_rx,

///////////////////////////////////////////////////
// i/o pads near jtag connector user can solder to

output	wire				user1,
input	wire					user2,

///////////////////////////////////////////////////
// RFU internal i2c bus 

inout	wire					aux_sda,
output	wire				aux_scl,

///////////////////////////////////////////////////
// RFU, do not use
output	wire				vpll_feed,

//
// logical connections
//

///////////////////////////////////////////////////
// video, audio output to scaler
output	wire	[23:0]	video_rgb,
output	wire				video_rgb_clock,
output	wire				video_rgb_clock_90,
output	wire				video_de,
output	wire				video_skip,
output	wire				video_vs,
output	wire				video_hs,
	
output	wire				audio_mclk,
input		wire				audio_adc,
output	wire				audio_dac,
output	wire				audio_lrck

);

// not using the IR port, so turn off both the LED, and
// disable the receive circuit to save power
assign port_ir_tx = 0;
assign port_ir_rx_disable = 1;

// cart is unused, so set all level translators accordingly
// directions are 0:IN, 1:OUT
// assign cart_tran_bank3 		= 8'hzz;
// assign cart_tran_bank3_dir = 1'b0;
// assign cart_tran_bank2 		= 8'hzz;
// assign cart_tran_bank2_dir = 1'b0;
// assign cart_tran_bank1 		= 8'hzz;
// assign cart_tran_bank1_dir = 1'b0;
// assign cart_tran_bank0 		= 4'hf;
// assign cart_tran_bank0_dir = 1'b1;
// assign cart_tran_pin30 		= 1'b0;		// reset or cs2, we let the hw control it by itself
// assign cart_tran_pin30_dir = 1'bz;
// assign cart_pin30_pwroff_reset = 1'b0;	// hardware can control this
// assign cart_tran_pin31 		= 1'bz;		// input
// assign cart_tran_pin31_dir = 1'b0;	// input

// link port is input only
assign port_tran_so 			= 1'bz;
assign port_tran_so_dir 	= 1'b0;		// SO is output only
assign port_tran_si 			= 1'bz;
assign port_tran_si_dir 	= 1'b0;		// SI is input only
assign port_tran_sck 		= 1'bz;
assign port_tran_sck_dir 	= 1'b0;	// clock direction can change
assign port_tran_sd 			= 1'bz;
assign port_tran_sd_dir 	= 1'b0;		// SD is input and not used

// Audio system

wire clk_audio;
wire [15:0] audio_l;
wire [15:0] audio_r;
wire audio_s;
wire audio_mix;

i2s i2s (
.clk_74a			(clk_74a),
.left_audio		(audio_l),
.right_audio	(audio_r),

.audio_mclk		(audio_mclk),
.audio_dac		(audio_dac),
.audio_lrck		(audio_lrck)

);

wire debug_led, debug_button;

emu Neogeo
(
	.clk_74a					(clk_74a),
	.reset_l_main			(reset_l_main),
	
	.debug_led				(debug_led),
	.debug_button			(debug_button),
	
	.bridge_1wire			( bridge_1wire ),
	
	.bridge_spimosi		( bridge_spimosi ),
	.bridge_spimiso		( bridge_spimiso ),
	.bridge_spiclk			( bridge_spiclk ),
	.bridge_spiss			( bridge_spiss ),

	.VGA_R					(video_rgb[23:16]),
	.VGA_G					(video_rgb[15: 8]),
	.VGA_B					(video_rgb[ 7: 0]),
	.VGA_HS					(video_hs),
	.VGA_VS					(video_vs),
	.VGA_DE					(video_de),
	
	.CLK_VIDEO				(video_rgb_clock),
	.CLK_VIDEO_90			(video_rgb_clock_90),

	.AUDIO_L					( audio_l ),
	.AUDIO_R					( audio_r ),
	.AUDIO_S					( audio_s ),

	.cram0_a					( cram0_a ),
	.cram0_dq				( cram0_dq ),
	.cram0_wait				( cram0_wait ),
	.cram0_clk				( cram0_clk ),
	.cram0_adv_n			( cram0_adv_n ),
	.cram0_cre				( cram0_cre ),
	.cram0_ce0_n			( cram0_ce0_n ),
	.cram0_ce1_n			( cram0_ce1_n ),
	.cram0_oe_n				( cram0_oe_n ),
	.cram0_we_n				( cram0_we_n ),
	.cram0_ub_n				( cram0_ub_n ),
	.cram0_lb_n				( cram0_lb_n ),
	
	.cram1_a					( cram1_a ),
	.cram1_dq				( cram1_dq ),
	.cram1_wait				( cram1_wait ),
	.cram1_clk				( cram1_clk ),
	.cram1_adv_n			( cram1_adv_n ),
	.cram1_cre				( cram1_cre ),
	.cram1_ce0_n			( cram1_ce0_n ),
	.cram1_ce1_n			( cram1_ce1_n ),
	.cram1_oe_n				( cram1_oe_n ),
	.cram1_we_n				( cram1_we_n ),
	.cram1_ub_n				( cram1_ub_n ),
	.cram1_lb_n				( cram1_lb_n ),
	
	.SDRAM_DQ				( dram_dq ),
	.SDRAM_A					( dram_a ),
	.SDRAM_DQML				( dram_dqm[0] ),
	.SDRAM_DQMH				( dram_dqm[1] ),
	.SDRAM_BA				( dram_ba ),
	.SDRAM_nWE				( dram_we_n ),
	.SDRAM_nRAS				( dram_ras_n ),
	.SDRAM_nCAS				( dram_cas_n ),
	.SDRAM_CLK				( dram_clk ),
	.SDRAM_CKE				( dram_cke ),
	
	.sram_a					( sram_a ),
	.sram_dq					( sram_dq ),
	.sram_oe_n				( sram_oe_n ),
	.sram_we_n				( sram_we_n ),
	.sram_ub_n				( sram_ub_n ),
	.sram_lb_n				( sram_lb_n ),

	//Analogizer inputs
	.snac_p1 (PLAYER1 ),
	.snac_p2 (PLAYER2 ),
	.core_hsync(core_hsync),
	.core_vsync(core_vsync)
);

//Analogizer

    //! --- SNAC ---
    wire [15:0] JOY_DB1,JOY_DB2;
    wire GAME_INPUTS_CLK, GAME_INPUTS_LAT, GAME_INPUTS_DAT;
    
    joy_db15_2p joyinputs
    (
     .clk(clk_sys),      //Reloj de Entrada sobre 48-50Mhz
     .JOY_CLK(GAME_INPUTS_CLK),
     .JOY_LOAD(GAME_INPUTS_LAT),  
     .JOY_DATA(GAME_INPUTS_DAT), 
     .joystick1 ( JOY_DB1 ),
     .joystick2 ( JOY_DB2 )
    );


//custom_joy1:13 UP,12 DOWN,11 RIGHT,10 LEFT,9 H,8 G,7 F,6 E,5 D,4 C,3 B,2 A,1 START1,0 COIN
//db15:        //    11 L, 10 S, 9 F, 8 E, 7 D, 6 C, 5 B, 4 A, 3 U, 2 D, 1 L, 0 R
//    10 9876543210
//----LS FEDCBAUDLR

wire [5:0] core_r = video_rgb[23:18];
wire [5:0] core_g = video_rgb[15:10];
wire [5:0] core_g = video_rgb[7:2];
wire clk_vid = video_rgb_clock;
wire core_hsync, core_vsync;
//VGA_DE  ~(VBlank | HBlank)

	 //Pocket controls
	 wire [15:0] PLAYER1;
	 wire [15:0] PLAYER2;
	 assign PLAYER1 = {4'b0,JOY_DB1[11],JOY_DB1[10],JOY_DB1[7:4],JOY_DB1[3],JOY_DB1[2],JOY_DB1[1],JOY_DB1[0]};
	 assign PLAYER2 = {4'b0,JOY_DB2[11],JOY_DB2[10],JOY_DB2[7:4],JOY_DB2[3],JOY_DB2[2],JOY_DB2[1],JOY_DB2[0]};
	
	//Using ADV/GM7123 RGB DAC 666 mode, SNAC interface:
    assign cart_tran_bank3_dir = 1'b1; //output
	                           //R[5] R[4] R[3] R[2] R[1] R[0] HSYNC VSYNC
    assign cart_tran_bank3     ={core_r[7],core_r[6],core_r[5],core_r[4],core_r[3],core_r[2],core_hsync,core_vsync};
    
	 assign cart_tran_bank2_dir = 1'b1; //output
	                            //B[0]     /BLANK G[5] G[4] G[3] G[2] G[1] G[0]
    assign cart_tran_bank2     ={core_b[2],video_de  ,core_g[7],core_g[6],core_g[5],core_g[4],core_g[3],core_g[2]};
    
	 assign cart_tran_bank1_dir = 1'b1; //output
	                            //SNAC_OUT2(D+)  SNAC_OUT1(D-)   VID_CLK B[5]      B[4]      B[3]      B[2]      B[1]
    assign cart_tran_bank1     ={GAME_INPUTS_LAT,GAME_INPUTS_CLK,clk_vid,core_b[7],core_b[6],core_b[5],core_b[4],core_b[3]};
	 
	assign cart_pin30_pwroff_reset = 1'b1;  // normal GPIO
	assign cart_tran_pin30_dir     = 1'b0; //input
	assign cart_tran_pin30         = 1'bZ;
    assign cart_tran_pin31         = 1'bZ;  // input
     assign cart_tran_pin31_dir    = 1'b0;  // input

     //GAME INPUTS DAT, Conf. A: cart_tran_pin31
	assign GAME_INPUTS_DAT = cart_tran_pin30; //Conf. A //SNAC_IO3
    //assign GAME_INPUTS_DAT = cart_tran_pin31; //Conf. A //SNAC_IO6

	 //Audio I2S
//	 assign cart_tran_bank0_dir = 1'b;
//	 assign cart_tran_bank0 = {audio_mclk,audio_lrck,audio_sclk,audio_dac};

    assign cart_tran_bank0_dir = 1'b0; //input
    assign cart_tran_bank0 = {4'bZZZZ};
endmodule
