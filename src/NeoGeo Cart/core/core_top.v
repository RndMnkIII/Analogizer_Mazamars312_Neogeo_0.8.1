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

/*[ANALOGIZER_HOOK_BEGIN]*/
wire clk_sys_h;
wire [23:0] video_rgb2;
/*[ANALOGIZER_HOOK_END]*/
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

	.VGA_R					(video_rgb2[23:16]),
	.VGA_G					(video_rgb2[15: 8]),
	.VGA_B					(video_rgb2[ 7: 0]),
	.VGA_HS					(video_hs),
	.VGA_VS					(video_vs),
	.VGA_DE					(video_de),
	
	.CLK_VIDEO				(video_rgb_clock), //vid
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

	/*[ANALOGIZER_HOOK_BEGIN]*/
	.SYSCLK(clk_sys_h),
	.snac_p1 (PLAYER1 ),
	.snac_p2 (PLAYER2 ),
	.analog_video_type(analog_video_type),
	.core_hsync(core_hsync),
	.core_vsync(core_vsync),
	.analogizer_game_controller_type(game_cont_type),
    .analogizer_game_cont_sample_rate(game_cont_sample_rate),
	.blank_pocket_screen(blank_pocket_screen)
	/*[ANALOGIZER_HOOK_END]*/
);

/*[ANALOGIZER_HOOK_BEGIN]*/
wire [7:0] neo_r = video_rgb2[23:16];
wire [7:0] neo_g = video_rgb2[15: 8];
wire [7:0] neo_b = video_rgb2[ 7: 0];
wire clk_vid = video_rgb_clock_90; //video_rgb_clock; //Fixed one bit shift error on RGB channels.
wire core_hsync, core_vsync;
wire  SYNC = ~^{core_hsync, core_vsync};

//NeoGeo controls
	wire [15:0] PLAYER1;
	wire [15:0] PLAYER2;

//Pocket Screen Blanking Control
assign video_rgb = blank_pocket_screen ? 24'h000 : video_rgb2;
//*** Analogizer Interface V1.0 ***
wire analogizer_ena;
wire [3:0] analog_video_type;
wire [4:0] game_cont_type /* synthesis keep */;
wire [2:0] game_cont_sample_rate /* synthesis keep */;
wire p1_interface /* synthesis keep */;
wire p2_interface /* synthesis keep */;
wire blank_pocket_screen;
// wire BtnCasAplusSEL = 0;
// wire PauseAsSelplusStart = 0;
// wire ShowTestPattern = 0;

wire [15:0] p1_btn;
wire [15:0] p2_btn;
assign PLAYER1 = {6'b000000,p1_btn[14], p1_btn[15], p1_btn[7:4], p1_btn[0], p1_btn[1], p1_btn[2], p1_btn[3]}; // Xbox Controller/Snes controller
assign PLAYER2 = {6'b000000,p2_btn[14], p2_btn[15], p2_btn[7:4], p2_btn[0], p2_btn[1], p2_btn[2], p2_btn[3]};


openFPGA_Pocket_Analogizer #(.MASTER_CLK_FREQ(96_000_000)) analogizer (
	.i_clk(clk_sys_h),
	.i_rst(~reset_l_main), //i_rst is active high
	.i_ena(1'b1),
	//Video interface
	.analog_video_type(analog_video_type),
	.R(neo_r),
	.G(neo_g),
	.B(neo_b),
	.BLANKn(video_de),
	.Hsync(SYNC), //composite SYNC on HSync.
	.Vsync(1'b1),
	.video_clk(clk_vid),
	//SNAC interface
	.conf_AB((game_cont_type >= 5'd16)),              //0 conf. A(default), 1 conf. B (see graph above)
	.game_cont_type(game_cont_type), //0-15 Conf. A, 16-31 Conf. B
	.game_cont_sample_rate(game_cont_sample_rate), //0 compatibility mode (slowest), 1 normal mode, 2 fast mode, 3 superfast mode
	//.game_cont_sample_rate(2'b01), //0 compatibility mode (slowest), 1 normal mode, 2 fast mode, 3 superfast mode
	.p1_btn_state(p1_btn),
	.p2_btn_state(p2_btn),
	.busy(),   
	//Pocket Analogizer IO interface to the Pocket cartridge port
	.cart_tran_bank2(cart_tran_bank2),
	.cart_tran_bank2_dir(cart_tran_bank2_dir),
	.cart_tran_bank3(cart_tran_bank3),
	.cart_tran_bank3_dir(cart_tran_bank3_dir),
	.cart_tran_bank1(cart_tran_bank1),
	.cart_tran_bank1_dir(cart_tran_bank1_dir),
	.cart_tran_bank0(cart_tran_bank0),
	.cart_tran_bank0_dir(cart_tran_bank0_dir),
	.cart_tran_pin30(cart_tran_pin30),
	.cart_tran_pin30_dir(cart_tran_pin30_dir),
	.cart_pin30_pwroff_reset(cart_pin30_pwroff_reset),
	.cart_tran_pin31(cart_tran_pin31),
	.cart_tran_pin31_dir(cart_tran_pin31_dir),
	//debug
	.o_stb()
);
/*[ANALOGIZER_HOOK_END]*/
endmodule
