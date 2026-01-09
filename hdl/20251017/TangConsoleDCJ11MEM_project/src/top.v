//---------------------------------------------------------------------------
// TangConsoleDCJ11MEM
// Memory system and peripherals on Tang Console 138K for DEC DCJ11 (PDP11)
//
// version 20251017
//---------------------------------------------------------------------------
`define VERSION_ID 32'h251017_01
`define USE_ACCESSLOG            // enable disk access log for debug
`define USE_MT_WRITE_PROTECT     // write protect magnetic tape on sd2

//`define USE_RT11V4_WORKAROUND  // uncomment this line to boot RT-11 v4
  
//---------------------------------------------------------------------------
// Implemented devices
// - KL11 TTY
// - KW11-L line time clock
// - KE11-A extended arithmetic unit
// - PC11 Paper Tape read/punch
// - RF11(drum) controller
// - RK11(disk) controller
// - RP11 controller with RP03 disk pack
// - TM11 Magnetic Tape (simple read/write function only)
//
// 2025/09/02: - Ported from Tang Nano 20K to Tang Console 138K
//             - Use CLK2 as a system clock
//             - Physical memory space extended from 16bit to 18bit
//
// 2025/09/07: - Some debug tool installed
// 2025/09/11: - WS2812 installed
// 2025/09/17: - Bugfix for using without SD memory
// 2025/09/19: - PC-11(Paper-Tape Reader/Punch) emulator implemented
//             - The punch function is disabled to prevent overwriting
//             - the SD card.
// 2025/09/22: - sw2_toggle and sw2_count installed for debug
// 2025/09/25: - Onboard sw's function changed
// 2025/09/26: - Invoke IRQ_ttyo at 60Hz (dirty workaround to boot RT-11)
// 2025/10/01: - Bugfix for disks
// 2025/10/02: - RP11 controller (with RP03) emulator implemented
// 2025/10/06: - TM11/TU10 magnetic tape emulator implemented
//             - Reset by GP (RESET instruction)  implemented
//             - Boot address changed (see rom.v)
// 2025/10/08: - Disk and magtape bugfix
// 2025/10/12: - SD_MEM_FRQ changed from 400kHz to 800kHz
//             - Make read MTSC1(772440) invoke bus error (for uv7 tape boot)
// 2025/10/13: - TM11: Space forward implemented
// 2025/10/14: - TM11: interrupt on 'write IE and no GO' implemented
// 2025/10/16: - Image size for RP03 changed (for 2.9BSD's sys/conf/ioconf.c)
//             - RT-11 causes MTSC1 buserror, so compile flag
//             - 'USE_RT11V4_WORKAROUND' added.
//             - '60Hz IRQ_ttyo' disabled, 'USE_RT11V4_WORKAROUND' enables it.
//             - Compile flag 'USE_MT_WRITE_PROTECT' to avoid overwrite tape
// 2025/10/17: - SD_MEM_FRQ changed from 800kHz to 1MHz
//---------------------------------------------------------------------------
// TangNanoDCJ11MEM 
// Memory system and peripherals on TangNano20K for DEC DCJ11 (PDP11)
//
// version 20240730.v6.beta
// test version for UNIX V6 experiments
//
// by Ryo Mukai (https://github.com/ryomuk)
//
// The emulatior of the following peripherals are implemented
// for running the UNIX first edition (UNIX V1).
// - KL11 TTY
// - RF11 drum
// - RK11 disk
// - KW11-L line time clock
// - KE11-A extended arithmetic unit
//
// 2024/06/09: - Hard disk (RF11 and RK11) emulator (sdhd.v) implemented
// 2024/06/10: - KW11-L emulator implemented
// 2024/06/11: - KE11-A emulator implemented
// 2024/06/24: - initial version (very unstable)
// 2024/07/10: - separate command reception for RF11 and RK11
// 2024/07/13: - IRQ bug fixed
// 2024/07/14: - debug register address changed
// 2024/07/15: - Stretched cycle installed for RF, RK, RE11
//               - CONT_n is assigned to LED3.
//             - IRQ2 and IRQ3 are removed. LED2_n is free.
//             - Memory sytem modified to single port RAM
// 2024/07/19: - Relatively stable compared to previous versions.
// 2024/07/27: - Stabilized disk ready logic for IRQ_RFRK (dirty workaround)
//             - Bufferd TX (uart.v) implemented
//             - Some features for UNIX V6 Experiments
//               - ABORT_n installed (pin is LED[2]_n)
//                 write to 160000-167777 or read 177700 causes bus error
//               - Make RAM 28KW (160000-177777 is a ROM area)
//               - boot from RK0 disk (174000g)
// 2024/07/28: - read 160000-160077 causes bus error
// 2024/07/29: - ABORT_n changed to tri-state
//             - bus error condition changed
//                 read 170000-170077 or 177700 causes bus error
// 2024/07/30: - multiple RK disks supported
//---------------------------------------------------------------------------

module top(
    input	  sw1_n,
    input	  sw2_n,
    input	  sys_clk50, // 50MHz system clock

    input	  USB_RX,
    output	  USB_TX,

    output	  GPIO_TX,
    input	  GPIO_RX,
    output	  LOG_TX,    // disk access log for debug

    output	  sd_clk,
    output	  sd_mosi, 
    input	  sd_miso,
    output	  sd_cs_n,
    input	  sd_det_n,  //  seems not working
	   
    output	  sd2_clk,
    output	  sd2_mosi, 
    input	  sd2_miso,
    output	  sd2_cs_n,
    input	  sd2_det_n,
	   
    inout [15:0]  DAL,
    input [17:16] DALH,	     // DAL[17:16] is input (not inout)
    input [3:0]	  AIO,
//    input [1:0]	  BS,
    output	  IRQ0,
    output	  IRQ1,
    output	  IRQ2,
    inout	  ABORT_n,
    output	  EVENT_n,
    output	  INIT_n,    // CPU reset signal
    input	  BUFCTL_n,
    input	  ALE_n,
    input	  SCTL_n,
    output	  CONT_n,
    output	  HALT,
    input	  CLK2,
	   
    input	  INIT_SW,
    input	  HALT_SW,

    output	  led_b,
    output	  led_r,
    output	  LED_RGB

//    output [3:0]  dbg
    );
//---------------------------------------------------------------------------
// some configurations
//---------------------------------------------------------------------------
`ifdef USE_RT11V4_WORKAROUND
  wire flag_rt11_workaround = 1'b1;
`else
  wire flag_rt11_workaround = 0;
`endif

//---------------------------------------------------------------------------
// Clock signals
//---------------------------------------------------------------------------
  // CLK2 as a system clock
  parameter	 SYS_CLK_FRQ  = 18_000_000; //Hz
  wire		 sys_clk      = CLK2;

  // CPU clock stops while INIT, 
  // so another clock is required for the initialization
  parameter	 INIT_CLK_FRQ = 50_000_000; // Hz
  wire		 init_clk     = sys_clk50;  // system clock for reset

  reg		 sys_clk25 = 0;
  always@(posedge sys_clk50)
    sys_clk25 <= ~sys_clk25;
  
  // clock for sdhd and sdtape
  parameter	 SD_SYS_FRQ   = SYS_CLK_FRQ;
  parameter	 SD_MEM_FRQ   =1000_000;
//  parameter	 SD_MEM_FRQ   = 800_000;
//  parameter	 SD_MEM_FRQ   = 400_000;
  wire		 sd_sys_clk   = sys_clk;

  // clock for ws2812 RGB LED
  parameter      WS2812_CLK_FRQ = 50_000_000; // Hz
  wire		 ws2812_clk     = sys_clk50;

//---------------------------------------------------------------------------
// UART
// There are three UART ports
//
// (1) USB_TX, USB_RX:
//     - UART via USB of Tang Console (shared with JTAG) (for TTY)
//
// (2) LOG_TX:
//     - pmod1[0] of Tang Console (for debug log)
//
// (3) GPIO_TX, GPIO_RX:
//     - UART on the test board (for TTY or debug log)
//---------------------------------------------------------------------------

// RL11 TTY console
  wire		 uart_tx;
  wire		 uart_rx;

// output ports
  assign USB_TX  = uart_tx;

  assign GPIO_TX = uart_tx; // for duplicate USB_TX
//  assign GPIO_TX = dbg_tx; // for duplicate LOG_TX
  
  assign LOG_TX  = dbg_tx;

// input port
  assign uart_rx = USB_RX & GPIO_RX; // for duplicate TTY input
//  assign uart_rx = USB_RX;

//---------------------------------------------------------------------------
// UART BPS
//---------------------------------------------------------------------------
//  parameter	 UART_BPS    =        110; //Hz (needs appropriate serial IF)
//  parameter	 UART_BPS    =        300; //Hz (minimum speed of FT232)
//  parameter	 UART_BPS    =       1200; //Hz (minimum speed of TangNano USB)
//  parameter	 UART_BPS    =       9600; //Hz
//  parameter	 UART_BPS    =      38400; //Hz
  parameter	 UART_BPS    =     115200; //Hz

//---------------------------------------------------------------------------
// RF11 (drum) disk controller
//---------------------------------------------------------------------------
  parameter ADRS_RF_DCS  = 18'o777460; // Disk Control Status Register
    parameter RF_DCS_GO          = 1'b1;  // [0]
    parameter RF_DCS_NOP         = 2'b00; // [2:1]
    parameter RF_DCS_WRITE       = 2'b01; // [2:1]
    parameter RF_DCS_READ        = 2'b10; // [2:1]
    parameter RF_DCS_WRITE_CHECK = 2'b11; // [2:1]
  parameter ADRS_RF_WC   = 18'o777462; // Word Count Register
  parameter ADRS_RF_CMA  = 18'o777464; // Current Memory Address Register
  parameter ADRS_RF_DAR  = 18'o777466; // Disk Address Register
  parameter ADRS_RF_DAE  = 18'o777470; // Disk Address Ext & Error Register
  parameter ADRS_RF_DBR  = 18'o777472; // Disk Buffer Register
  parameter ADRS_RF_MAR  = 18'o777474; // Maintenance Register
  parameter ADRS_RF_ADS  = 18'o777476; // Address of Disk Segment Register

  reg [15:0] REG_RF_DCS;
  reg [15:0] REG_RF_WC;
  reg [15:0] REG_RF_CMA;
  reg [15:0] REG_RF_DAR;
  reg [15:0] REG_RF_DAE;
  reg [15:0] REG_RF_ADS;

  reg [15:0] REG_RF_WC_BAK;
  reg [15:0] REG_RF_CMA_BAK;
  reg [15:0] REG_RF_DAR_BAK;
  reg [15:0] REG_RF_DAE_BAK;
  reg [15:0] REG_RF_DCS_BAK;
  
//---------------------------------------------------------------------------
// RK11 disk controller
//---------------------------------------------------------------------------
  parameter ADRS_RKDS = 18'o777400; // Disk Control Status Register
  parameter ADRS_RKER = 18'o777402; // Error Register
  parameter ADRS_RKCS = 18'o777404; // Control Status Register
    parameter RKCS_GO  = 1'b1;      // [0]
    parameter RKCS_CRESET      = 3'b000;  // [3:1] 'b0001 = 'o001
    parameter RKCS_WRITE       = 3'b001;  // [3:1] 'b0011 = 'o003
    parameter RKCS_READ        = 3'b010;  // [3:1] 'b0101 = 'o005
    parameter RKCS_WRITE_CHECK = 3'b011;  // [3:1] 'b0111 = 'o007
    parameter RKCS_SEEK        = 3'b100;  // [3:1] 'b1001 = 'o011
    parameter RKCS_READ_CHECK  = 3'b101;  // [3:1] 'b1011 = 'o013
    parameter RKCS_DRESET      = 3'b110;  // [3:1] 'b1101 = 'o015
    parameter RKCS_WRITE_LOCK  = 3'b111;  // [3:1] 'b1111 = 'o017
  parameter ADRS_RKWC = 18'o777406; // Word Count Register
  parameter ADRS_RKBA = 18'o777410; // Current Bus Address Register
  parameter ADRS_RKDA = 18'o777412; // Disk Address Register
  parameter ADRS_RKMR = 18'o777414; // Maintenance Register
  parameter ADRS_RKDB = 18'o777416; // Disk Buffer Register

  reg [15:0] REG_RKCS;
  reg [15:0] REG_RKWC;
  reg [15:0] REG_RKBA;
  reg [15:0] REG_RKDA;
  
  reg [15:0] REG_RKWC_BAK;
  reg [15:0] REG_RKBA_BAK;
  reg [15:0] REG_RKCS_BAK;

  reg [15:0] REG_RPWC_BAK;
  reg [15:0] REG_RPBA_BAK;
  reg [15:0] REG_RPCS_BAK;

//---------------------------------------------------------------------------
// RP11 disk controller with RP03 disk pack
//---------------------------------------------------------------------------
  parameter ADRS_RPDS = 18'o776710; // Device Status Register
  parameter ADRS_RPER = 18'o776712; // Error Register
  parameter ADRS_RPCS = 18'o776714; // Control Status Register
    parameter RPCS_GO  = 1'b1;      // [0]
    parameter RPCS_IDLE         = 3'd0;
    parameter RPCS_WRITE        = 3'd1;
    parameter RPCS_READ         = 3'd2;
    parameter RPCS_WRITE_CHECK  = 3'd3;
    parameter RPCS_SEEK         = 3'd4;
    parameter RPCS_WRITE_NOSEEK = 3'd5;
    parameter RPCS_HOME_SEEK    = 3'd6;
    parameter RPCS_READ_NOSEEK  = 3'd7;
  parameter ADRS_RPWC = 18'o776716; // Word Count
  parameter ADRS_RPBA = 18'o776720; // Bus Address
  parameter ADRS_RPCA = 18'o776722; // Cylinder Address
  parameter ADRS_RPDA = 18'o776724; // Disk Address
  parameter ADRS_RPM1 = 18'o776726; // (same address as RPDT of RP04)

  reg [15:0] REG_RPCS;
  reg [15:0] REG_RPWC;
  reg [15:0] REG_RPBA;
  reg [15:0] REG_RPCA;
  reg [15:0] REG_RPDA;
  reg [15:0] REG_RPM1 = 16'o22; // (RPDT=022, to avoid error on booting 2.9BSD)
  
  reg [15:0] REG_PKWC_BAK;
  reg [15:0] REG_PKBA_BAK;
  reg [15:0] REG_PKCA_BAK;
  reg [15:0] REG_PKDA_BAK;
  reg [15:0] REG_PKCS_BAK;

//---------------------------------------------------------------------------
// RH11 disk controller (dummy)
//---------------------------------------------------------------------------
  parameter ADRS_RPCS1 = 18'o776700;
  reg [15:0] REG_RPCS1 = 0;

  always @(posedge sys_clk)
    if(negedge_SCTL_n & (address == ADRS_RPCS1) & bus_write)
      REG_RPCS1 <= DAL;
	   
//---------------------------------------------------------------------------
// TM11 magnetic tape controller
//---------------------------------------------------------------------------
  parameter ADRS_MTS = 18'o772520; // Status Register
  parameter ADRS_MTC = 18'o772522; // Control Status Register
    parameter MTC_GO = 1'b1;      // [0]
    parameter MTC_OFFLINE   = 3'd0;
    parameter MTC_READ      = 3'd1;
    parameter MTC_WRITE     = 3'd2;
    parameter MTC_WRITE_EOF = 3'd3;
    parameter MTC_SPACE_F   = 3'd4; // Space Forward
    parameter MTC_SPACE_R   = 3'd5; // Space Reverse
    parameter MTC_WRITE_EXT = 3'd6; // Write with extended IRG
    parameter MTC_REWIND    = 3'd7;
  parameter ADRS_MTBRC = 18'o772524; // Byte Record Counter
  parameter ADRS_MTCMA = 18'o772526; // Current Memory Address Register
  parameter ADRS_MTD   = 18'o772530; // Data Buffer Register
  parameter ADRS_MTRD  = 18'o772532; // TU10 Read Lines

  wire	     MTS_EOF   = sw2;
  wire	     MTC_ERR   = (mt_error != 0);
  reg [15:0] REG_MTC;
  reg [15:0] REG_MTBRC;
  reg [15:0] REG_MTCMA;
  reg [15:0] REG_MTD;
  reg [15:0] REG_MTRD;

  reg [15:0] REG_MTC_BAK;
  reg [15:0] REG_MTBRC_BAK;
  reg [15:0] REG_MTCMA_BAK;

//---------------------------------------------------------------------------
// KE11 Extented Arithmetic Element
//---------------------------------------------------------------------------
  parameter  ADRS_KE_DIV = 18'o777300; // Divide
  parameter  ADRS_KE_AC  = 18'o777302; // Accumulator
  parameter  ADRS_KE_MQ  = 18'o777304; // Multiplier-Quotient
  parameter  ADRS_KE_MUL = 18'o777306; // Multiply
  parameter  ADRS_KE_SC  = 18'o777310; // Step Counter
  parameter  ADRS_KE_SR  = 18'o777311; // Status Register
  parameter  ADRS_KE_NOR = 18'o777312; // Normalization
  parameter  ADRS_KE_LSH = 18'o777314; // Logical Shift
  parameter  ADRS_KE_ASH = 18'o777326; // Arithmetic Shift

  reg [15:0] REG_KE_AC;
  reg [15:0] REG_KE_AC_OUT;
  reg [15:0] REG_KE_MQ;
  reg [15:0] REG_KE_MQ_OUT;
  reg [15:0] REG_KE_X;
  reg [7:0]  REG_KE_SC;
  reg [7:0]  REG_KE_SC_OUT;
  wire [7:0] REG_KE_SR;
  reg	     REG_KE_SR0;  // SR[0]

//---------------------------------------------------------------------------
// Console / KL11 registers
//---------------------------------------------------------------------------
  parameter ADRS_RCSR = 18'o777560; // Console read status (aka TKS)
  parameter ADRS_RBUF = 18'o777562; // Console read buffer (aka TKB)
  parameter ADRS_XCSR = 18'o777564; // Console send status (aka TPS)
  parameter ADRS_XBUF = 18'o777566; // Console send buffer (aka TPB)
  parameter ADRS_SWR  = 18'o777570; // Console Switch Register
  parameter ADRS_SMR  = 18'o777750; // KDJ-11 system maintenance register

  reg		 RCSR_ID;                    // bit6 (Interrupt Enabe on DONE)
  wire		 RCSR_DONE  = rx_data_ready; // bit7
  reg		 XCSR_ID;                    // bit6 (Interrupt Enabe on DONE)
  wire		 XCSR_READY = tx_ready ;     // bit7
  wire [7:0]	 RBUF       = rx_data;       // DATA(=bit7..0)
  wire [7:0]	 XBUF       = tx_data;       // DATA(=bit7..0)

  reg [15:0]	 REG_SWR;
  
//---------------------------------------------------------------------------
// Papertape Reader/Puncher registers
//---------------------------------------------------------------------------
  parameter ADRS_PRS  = 18'o777550; // Papertape Reader Status Register
  parameter ADRS_PRB  = 18'o777552; // Papertape Reader Buffer
  parameter ADRS_PPS  = 18'o777554; // Papertape Punch  Status
  parameter ADRS_PPB  = 18'o777556; // Papertape Punch  Buffer

//---------------------------------------------------------------------------
// AIO codes
//---------------------------------------------------------------------------
  parameter AIO_NONIO        = 4'b1111; // Non-I/O
  parameter AIO_GPREAD       = 4'b1110; // General-Purpose Read
  parameter AIO_INTACK       = 4'b1101; // Interrupt ack and vector read
  parameter AIO_IREADRQ      = 4'b1100; // Instruction stream request read
  parameter AIO_RMWNBL       = 4'b1011; // Read-Modify-Write, no bus lock
  parameter AIO_RMWBL        = 4'b1010; // Read-Modify-Write, bus lock
  parameter AIO_DREAD        = 4'b1001; // Data stream read
  parameter AIO_IREADDM      = 4'b1000; // Instruction stream demand read
  parameter AIO_GPWRITE      = 4'b0101; // General-Purpose Write
  parameter AIO_BUSBYTEWRITE = 4'b0011; // Bus byte write
  parameter AIO_BUSWORDWRITE = 4'b0001; // Bus word write

//---------------------------------------------------------------------------
// GP codes
//---------------------------------------------------------------------------
// GP Read
  parameter GP_PUP            = 8'o000; // Reads the power-up mode
  parameter GP_FPA            = 8'o001; // Reads FPA data
  parameter GP_PUP2           = 8'o002; // Reads the power-up mode, clear FPS
// GP Write
  parameter GP_BUSRESET       = 8'o014; // Asserts bus reset signal
  parameter GP_EXIT_ODT       = 8'o034; // Signals exit from console ODT
  parameter GP_ACK_EVENT      = 8'o100; // Acknowledges EVENT
  parameter GP_NEG_BUSRESET   = 8'o214; // Negates bus reset signal
  parameter GP_TEST1          = 8'o220; // Microdiagnostic test 1 passed
  parameter GP_TEST2          = 8'o224; // Microdiagnostic test 2 passed
  parameter GP_TEST3          = 8'o230; // Microdiagnostic test 3 passed
  parameter GP_ENTRY_ODT      = 8'o234; // Signals entry into console ODT
  
//---------------------------------------------------------------------------
// Aliases
//---------------------------------------------------------------------------
  wire [17:0] address  = DAL_latched[17:0];
  wire [7:0]  gpcode   = DAL_latched[7:0];
  wire [3:0]  DAL_iack = DAL_latched[3:0];
  wire [2:0]  iack_level; // IRQ level acknowledged

  assign iack_level = (DAL_iack == 4'b0001) ? 3'd4: // IRQ0
		      (DAL_iack == 4'b0010) ? 3'd5: // IRQ1
		      (DAL_iack == 4'b0100) ? 3'd6: // IRQ2
		      (DAL_iack == 4'b1000) ? 3'd7: // IRQ3
		      0;
			 
  wire bus_read           = (aio_read  && (BUFCTL_n == 1'b0));
  wire bus_write          = (aio_write && (BUFCTL_n == 1'b1));
  wire vec_read           = (aio_code == AIO_INTACK) & (BUFCTL_n == 1'b0);
  
  wire [3:0] aio_code     = AIO_latched;
  wire aio_write          = (aio_code == AIO_BUSBYTEWRITE) |
                            (aio_code == AIO_BUSWORDWRITE);
  wire aio_write_lowbyte  = (aio_code == AIO_BUSBYTEWRITE) & ~address[0];
  wire aio_write_highbyte = (aio_code == AIO_BUSBYTEWRITE) & address[0];
  wire aio_read           = (aio_code == AIO_IREADRQ) |
                            (aio_code == AIO_INTACK)  |
                            (aio_code == AIO_RMWNBL)  |
                            (aio_code == AIO_RMWBL)   |
                            (aio_code == AIO_DREAD)   |
                            (aio_code == AIO_IREADDM);
  wire aio_iread          = (aio_code == AIO_IREADRQ) |
                            (aio_code == AIO_IREADDM);
  wire aio_iread_dm       = (aio_code == AIO_IREADDM);

//---------------------------------------------------------------------------
// make toggle switch sw1_toggle, sw2_toggle
// and 3-bit counters sw1_count, sw2_count
// from sw1_n and sw2_n
//---------------------------------------------------------------------------
  reg        sw1_toggle = 0; // toggle sw
  reg [2:0]  sw1_count  = 0; // 3-bit counter
  reg        sw1_wait   = 0; // flag to wait for release the button
  reg [27:0] sw1_cnt    = 0; // counter for debounce
  wire	     sw1        = sw1_wait; // used as debounced sw1
  parameter  sw1_WIDTH  = (INIT_CLK_FRQ / 1000) * 100; // 100ms
  always @(posedge init_clk)
    if( sw1_n ) // sw1_n is off
      {sw1_cnt, sw1_wait}  <= 0;
    else if (sw1_cnt != sw1_WIDTH)
      sw1_cnt <= sw1_cnt + 1'd1;
    else if( ~sw1_wait ) begin
       sw1_wait   <= 1'b1;
       sw1_toggle <= ~sw1_toggle;
       sw1_count  <= sw1_count + 1'd1;
    end

  reg        sw2_toggle = 0; // toggle sw
  reg [2:0]  sw2_count  = 0; // 3-bit counter
  reg        sw2_wait   = 0; // flag to wait for release the button
  reg [27:0] sw2_cnt    = 0; // counter for debounce
  wire	     sw2        = sw2_wait; // used as debounced sw2
  parameter  sw2_WIDTH  = (INIT_CLK_FRQ / 1000) * 100; // 100ms
  always @(posedge init_clk)
    if( sw2_n ) // sw2_n is off
      {sw2_cnt, sw2_wait}  <= 0;
    else if (sw2_cnt != sw2_WIDTH)
      sw2_cnt <= sw2_cnt + 1'd1;
    else if( ~sw2_wait ) begin
       sw2_wait   <= 1'b1;
       sw2_toggle <= ~sw2_toggle;
       sw2_count  <= sw2_count + 1'd1;
    end
  
//---------------------------------------------------------------------------
// reset button and power on reset
//---------------------------------------------------------------------------
// reset for CPU
  reg reg_INIT_n;

  assign INIT_n = reg_INIT_n;
  reg [27:0]	 init_cnt = 0;
  parameter	 INIT_WIDTH = (INIT_CLK_FRQ / 1000) * 250; // 250ms
  always @(posedge init_clk)
    if( INIT_SW )
      {reg_INIT_n, init_cnt} <= 0;
    else if (init_cnt != INIT_WIDTH) begin
       reg_INIT_n <= 0;
       init_cnt <= init_cnt + 1'd1;
    end
    else
      reg_INIT_n <= 1;
       
// reset for SD Card
  wire SD_RESET_n = INIT_n;

//---------------------------------------------------------------------------
// Bus Reset
// reset for peripheral registers
// BUS_RESET is asserted and negetaed by GP code '014' and '214'.
// These codes are written by RESET instruction or initialization sequence.
//---------------------------------------------------------------------------
  wire RESET_n = ~(BUS_RESET | ~INIT_n);

  reg BUS_RESET;
  always @(negedge SCTL_n)
    if (aio_code == AIO_GPWRITE)
      if(gpcode == GP_BUSRESET)
	BUS_RESET <= 1'b1;
      else if (gpcode == GP_NEG_BUSRESET)
	BUS_RESET <= 0;

//---------------------------------------------------------------------------
// Power-up Configurations
//---------------------------------------------------------------------------
// see "DCJ11 Microprocessor User's Guide, 8.3.3 Power-Up Configuration"
//
// Power-Up Configuration Register
// [15:9] Bit<15:9> of the boot address (<8:0> are zeros.)
// [8]    FPA Here
// [7:4]  Unused
// [3]    Halt Option
//          0: Enters console ODT on HALT
//          1: Trap to 4
// [2:1]  Power-up Mode
//          01: Enter console ODT (PS=0)
//          10: Power-up to 17773000(173000) (PS=340)
//          11: Power-up to the user-defined address([15:9])(PS=340)
// [0]    POK (1: power OK)

  // Enter console ODT
  parameter PUP_ODT    = 16'b0000000_0_0000_0_01_1;

  // Power-up to 173000
  parameter PUP_173000 = 16'b0000000_0_0000_0_10_1;

  // Power-up to User Program 
  parameter PUP_USER_ADRS = 16'o160_000; //(xxx000, lower 9bits must be 0)
  parameter PUP_USER      = PUP_USER_ADRS | 9'b0_0000_0_11_1;

  wire [15:0] PUP_CONF = PUP_ODT;
//  wire [15:0] PUP_CONF = PUP_173000;
//  wire [15:0] PUP_CONF = PUP_USER;
//  wire [15:0] PUP_CONF = (~sw2_n) ? PUP_USER: PUP_ODT;

//---------------------------------------------------------------------------
// Microdiagnostic test on the power-up sequence
//---------------------------------------------------------------------------
  reg diag_test1 = 0;
  reg diag_test2 = 0;
  reg diag_test3 = 0;
  wire diag_test_passed = diag_test1 & diag_test2 & diag_test3;
  wire diag_test_failed = ~diag_test_passed;
  always @(negedge SCTL_n)
    if (aio_code == AIO_GPWRITE)
      if(gpcode == GP_TEST1)
	diag_test1 <= 1'b1;
      else if (gpcode == GP_TEST2)
	diag_test2 <= 1'b1;
      else if (gpcode == GP_TEST3)
	diag_test3 <= 1'b1;

//---------------------------------------------------------------------------
// Memory and IO
//---------------------------------------------------------------------------
  assign DAL = BUFCTL_n ? 16'bzzzz_zzzz_zzzz_zzzz :
       (address == ADRS_RCSR) ? {8'b0, RCSR_DONE, RCSR_ID, 6'b0}:
       (address == ADRS_RBUF) ? {8'b0, RBUF}:
       (address == ADRS_XCSR) ? {8'b0, XCSR_READY, XCSR_ID, 6'b0}:
       (address == ADRS_XBUF) ? {8'b0, XBUF}:

       (address == ADRS_SWR)  ? REG_SWR:
       (address == ADRS_SMR)  ? 16'b0000000000010000:
	       // RF
       (address == ADRS_RF_DCS) ? {8'b0, RF_READY, REG_RF_DCS[6:1], 1'b0}:
       (address == ADRS_RF_WC)  ? REG_RF_WC:
       (address == ADRS_RF_CMA) ? REG_RF_CMA:
       (address == ADRS_RF_DAR) ? REG_RF_DAR:
       (address == ADRS_RF_DAE) ? REG_RF_DAE:
       (address == ADRS_RF_ADS) ? REG_RF_ADS:
	       // RK
       (address == ADRS_RKDS) ? {8'b000_01001,
				 RK_READY, RK_READY,
				 2'b01, REG_RKDA[3:0]}:
       (address == ADRS_RKER) ? 16'b0: // error register not implemented
       (address == ADRS_RKCS) ? {2'b00, REG_RKCS[13:8],
				 RK_READY, REG_RKCS[6:0]}:
       (address == ADRS_RKWC) ? REG_RKWC:
       (address == ADRS_RKBA) ? REG_RKBA:
       (address == ADRS_RKDA) ? REG_RKDA:
	       // RP03
       (address == ADRS_RPDS) ? {RP_READY, RP_READY, 
				 1'b1, // SURP03 (selected drive is an RP03)
				 5'b00000,
				 8'b00000000}:
       (address == ADRS_RPER) ? 16'b0: // error register not implemented
       (address == ADRS_RPCS) ? {2'b00, REG_RPCS[13:8],
				 RP_READY, REG_RPCS[6:0]}:
       (address == ADRS_RPWC) ? REG_RPWC:
       (address == ADRS_RPBA) ? REG_RPBA:
       (address == ADRS_RPCA) ? REG_RPCA:
       (address == ADRS_RPDA) ? REG_RPDA:
       (address == ADRS_RPM1) ? REG_RPM1: // =ADRS_RPDT
	       // RH11 (RP04) (dummy)
       (address == ADRS_RPCS1) ? REG_RPCS1:
	       // MT
       (address == ADRS_MTS) ? {1'b0, MTS_EOF, 7'b0, MTS_SELR,
				5'b00000, MTS_TUR}:
       (address == ADRS_MTC) ? {MTC_ERR, REG_MTC[14:13], 1'b0, REG_MTC[11:8],
				MTC_CURDY, REG_MTC[6:0]}:
       (address == ADRS_MTBRC) ? REG_MTBRC:
       (address == ADRS_MTCMA) ? REG_MTCMA:
       (address == ADRS_MTD)   ? REG_MTD:
       (address == ADRS_MTRD)  ? REG_MTRD:
	       // KW
       (address == ADRS_KW11L)  ? REG_KW11L:
	       // KE
       (address == ADRS_KE_DIV) ? 16'b0:
       (address == ADRS_KE_AC ) ? REG_KE_AC_OUT:
       (address == ADRS_KE_MQ ) ? REG_KE_MQ_OUT:
       (address == ADRS_KE_MUL) ? 16'b0:
       (address == ADRS_KE_SC ) ? {REG_KE_SR, REG_KE_SC_OUT}:
       (address == ADRS_KE_SR ) ? {REG_KE_SR, REG_KE_SC_OUT}:
       (address == ADRS_KE_NOR) ? REG_KE_SC_OUT: // read NOR returns SC
       (address == ADRS_KE_LSH) ? 16'b0:
       (address == ADRS_KE_ASH) ? 16'b0:
	       // PC
       (address == ADRS_PRS)  ? {PRS_ERR, 3'b0, PRS_BUSY,
				 3'b0, PRS_DONE, PRS_IE, 6'b0}:
       (address == ADRS_PRB)  ? {8'b0, PRB_DATA}:
       (address == ADRS_PPS)  ? {PPS_ERR, 7'b0, PPS_READY,
				 PPS_IE, 6'b0}:
       (address == ADRS_PPB)  ? {8'b0, PPB_DATA}:

       (aio_code == AIO_GPREAD && gpcode == GP_PUP)  ? PUP_CONF :
       (aio_code == AIO_GPREAD && gpcode == GP_PUP2) ? PUP_CONF :
       (aio_code == AIO_INTACK && iack_level == LV_IRQ0) ? VA_IRQ0:
       (aio_code == AIO_INTACK && iack_level == LV_IRQ1) ? VA_IRQ1:
//       (aio_code == AIO_INTACK && iack_level == LV_IRQ2) ? VA_IRQ2:
//       (aio_code == AIO_INTACK && iack_level == LV_IRQ3) ? VA_IRQ3:
       (address == ADRS_DBG0) ? REG_DBG0:
       (address == ADRS_DBG1) ? REG_DBG1:
       (address == ADRS_DBG2) ? REG_DBG2:
       (address == ADRS_TRACE0) ? REG_TRACE[0]:
       (address == ADRS_TRACE1) ? REG_TRACE[1]:
       (address == ADRS_TRACE2) ? REG_TRACE[2]:
       (address == ADRS_TRACE3) ? REG_TRACE[3]:
       (address == ADRS_TRACE4) ? REG_TRACE[4]:
       (address == ADRS_TRACE5) ? REG_TRACE[5]:
       (address == ADRS_TRACE6) ? REG_TRACE[6]:
       (address == ADRS_TRACE7) ? REG_TRACE[7]:
       (address == ADRS_TRACE8) ? REG_TRACE[8]:
       (address == ADRS_TRACE9) ? REG_TRACE[9]:
       (address == ADRS_TRACE10) ? REG_TRACE[10]:
       (address == ADRS_TRACE11) ? REG_TRACE[11]:
       (address == ADRS_TRACE12) ? REG_TRACE[12]:
       (address == ADRS_TRACE13) ? REG_TRACE[13]:
       (address == ADRS_TRACE14) ? REG_TRACE[14]:
       (address == ADRS_TRACE15) ? REG_TRACE[15]:
       (address == ADRS_TRACE16) ? REG_TRACE[16]:
       (address == ADRS_TRACE17) ? REG_TRACE[17]:
       (address == ADRS_TRACE18) ? REG_TRACE[18]:
       (address == ADRS_TRACE19) ? REG_TRACE[19]:
       (address == ADRS_TRACE20) ? REG_TRACE[20]:
       (address == ADRS_TRACE21) ? REG_TRACE[21]:
       (address == ADRS_TRACE22) ? REG_TRACE[22]:
       (address == ADRS_TRACE23) ? REG_TRACE[23]:
       d_ram_to_cpu;
  
//---------------------------------------------------------------------------
// Synchronize ALE_n and SCTL_n to sys_clk(negedge)
//---------------------------------------------------------------------------
  reg [1:0] ALE_ns;
  wire	    negedge_ALE_n = ALE_ns[1] & ~ALE_ns[0];
  wire	    posedge_ALE_n = ALE_ns[0] & ~ALE_ns[1];
  always @(negedge sys_clk )
    ALE_ns[1:0]  <= {ALE_ns[0], ALE_n};

  reg [1:0] SCTL_ns;
  wire    negedge_SCTL_n = SCTL_ns[1] & ~SCTL_ns[0];
  always @(negedge sys_clk )
    SCTL_ns[1:0] <= {SCTL_ns[0], SCTL_n};
  
//---------------------------------------------------------------------------
// Bus error
// read 760000-760077   (for unix v6)
// read 777700          (for Microdiagnostic test 2)
// read 772440 (MTSC1)  (for unix v7 tape boot)
//---------------------------------------------------------------------------
  assign ABORT_n = bus_error ? 1'b0 : 1'bz; // simulate open collector output

  wire bus_error =
       ((address       == 18'o777700) & bus_read) | // Microdiagnostic test 2
       ((address[17:6] == 12'o7600)   & bus_read) | // read 760000-760077
       ((address       == 18'o772440) & bus_read 
	& ~flag_rt11_workaround) ; // MTSC1 (TU16)

//---------------------------------------------------------------------------
// Memory
//---------------------------------------------------------------------------
  reg [17:0]	 DAL_latched; // latched DAL[15:0]
  reg [3:0]	 AIO_latched; // latched AIO[3:0]
//  reg [1:0]	 BS_latched;  // latched BS[1:0]
  
// The leading edge of ALE is typically used by external logic
// to latch addresses, AIO codes, BS codes and the MAP control signals.
// (user's manual 2.4.1)
  always @(negedge ALE_n) begin // latch DAL and AIO
     DAL_latched <= {DALH[17:16], DAL[15:0]};

// for 16bit mode
//     DAL_latched <= {(DAL[15:12] == 4'o17) ? 2'b11: 2'b00, DAL[15:0]};

     AIO_latched <= AIO;
//     BS_latched  <= BS;
  end
  
  wire write_memory_hi = ~SCTL_n &
//       (BS_latched == 2'b00) &
       (( aio_code == AIO_BUSWORDWRITE) |
	( aio_code == AIO_BUSBYTEWRITE) & address[0]);
  wire write_memory_lo = ~SCTL_n &
//       (BS_latched == 2'b00) &
       (( aio_code == AIO_BUSWORDWRITE) |
	( aio_code == AIO_BUSBYTEWRITE) & ~address[0]);
  
  // DMA is activated during disk or mt is busy but disabled while reset
  wire DMA = RESET_n &
             ((DMA_dev == DEV_MT) ? (mt_busy & (mt_error == 0)) :
                                    (disk_busy & (sd_error == 0)));

//---------------------------------------------------------------------------
// stretched cycle control signals
//---------------------------------------------------------------------------
  assign CONT_n = (DMA | stretch_trig | stretch_start | KE11_stretch ) ?
		  1'b1 : SCTL_n;
//  assign CONT_n = SCTL_n;
  
  wire stretch_trig = RF_go | RK_go | RP_go | TM_go;
  reg  stretch_start;
  always @(posedge sys_clk or negedge INIT_n)
    if( ~INIT_n )
      stretch_start <= 0;
    else if(stretch_trig)
      stretch_start <= 1'b1;
    else if ( posedge_disk_busy | posedge_mt_busy)
      stretch_start <= 0;

  reg [1:0] disk_busys;
  wire	    posedge_disk_busy = disk_busys[0] & ~disk_busys[1];
  always @(posedge sys_clk)
    disk_busys <= {disk_busys[0], disk_busy};

  reg [1:0] mt_busys;
  wire	    posedge_mt_busy = mt_busys[0] & ~mt_busys[1];
  always @(posedge sys_clk)
    mt_busys <= {mt_busys[0], mt_busy};
    
//---------------------------------------------------------------------------
// 124KW RAM 
//   - 000000-757777: RAM
//   - 760000-760077: No memory (read/write causes bus error)
//   - 760100-767777: ROM (write causes bus error)
//   - 770000-777777: ROM and Memory mapped I/O
//---------------------------------------------------------------------------
// mem_hi and mem_lo have 128KW capacity for fail safe
  reg [7:0] mem_hi[131071:0]; // higher 8bit (odd byte address)
  reg [7:0] mem_lo[131071:0]; // lower  8bit (even byte address)

  reg [15:0] d_cpu_to_ram;
  always @(negedge SCTL_n) // write data from cpu is latched at negedge SCTL_n
    d_cpu_to_ram <= DAL;

  // address or data of memory should be latched to infer BSRAM
  reg [16:0] wa;  // word address for RAM
  always @(negedge sys_clk)
    wa <= DMA ? dma_address[17:1] : address[17:1];

  wire [15:0] d_ram_to_cpu = {mem_hi[wa], mem_lo[wa]};
  wire [7:0]  d_ram_to_dma = dma_address[0] ? mem_hi[wa]: mem_lo[wa];

  wire	we_hi = DMA ? (dma_write &   dma_address[0])  : write_memory_hi;
  wire	we_lo = DMA ? (dma_write & (~dma_address[0])) : write_memory_lo;

  // 760000-777777 is ROM (RAM=124KW)
  wire	ram_area = (wa[16:12] != 5'b111_11);

  always @(posedge sys_clk)
    if( ram_area ) begin 
       if(we_lo) // dma data is 8bit 
	 mem_lo[wa] <= DMA ? d_dma_to_ram[7:0] : d_cpu_to_ram[7:0];
       if(we_hi)
	 mem_hi[wa] <= DMA ? d_dma_to_ram[7:0] : d_cpu_to_ram[15:8];
    end
  
//---------------------------------------------------------------------------
// ROM DATA
//---------------------------------------------------------------------------
`include "rom.v"

//---------------------------------------------------------------------------
// KE11 Arithmetic unit
//---------------------------------------------------------------------------
  assign REG_KE_SR[7:0] =
		{1'b0, // not implemented
		 REG_KE_AC_OUT[15], // not implemented preperly
		 REG_KE_AC_OUT == 16'o177777,
		 REG_KE_AC_OUT == 16'o0,
		 REG_KE_MQ_OUT == 16'o0,
		 (REG_KE_AC_OUT == 16'o0) && (REG_KE_MQ_OUT == 16'o0),
		 REG_KE_AC_OUT == (REG_KE_MQ_OUT[15] ? 16'o177777: 16'o0),
		 REG_KE_SR0
		 };

  wire div_clk = sys_clk;
//  wire div_clk = sys_clk50;
  wire [31:0] quotient;  //= dividend[31:0] / divisor[15:0];
  wire [15:0] remainder; //= dividend[31:0] % divisor[15:0];
  
  wire [31:0] dividend = {REG_KE_AC[15:0], REG_KE_MQ[15:0]};
  wire [15:0] divisor  = REG_KE_X[15:0];
//-------------------
// divider
//-------------------
// Gowin IP
  Integer_Division_Top integer_division(
		.clk(div_clk), //input clk
		.rstn(RESET_n), //input rstn
		.dividend(dividend), //input [31:0] dividend
		.divisor(divisor), //input [15:0] divisor
		.remainder(remainder), //output [15:0] remainder
		.quotient(quotient) //output [31:0] quotient
	);

//-------------------
// multiplier
//-------------------
  wire	[15:0]  abs_multiplicand = REG_KE_MQ[15] ?
		((~REG_KE_MQ[15:0]) + 1'b1) :
		REG_KE_MQ[15:0];
  wire [15:0]	abs_multiplier   = REG_KE_X[15] ? 
		((~REG_KE_X[15:0]) + 1'b1) :
		REG_KE_X[15:0];
  wire		mul_sign = REG_KE_MQ[15] ^ REG_KE_X[15];
  wire [31:0]	abs_product = abs_multiplicand[15:0] * abs_multiplier[15:0];
  wire [31:0]	product = mul_sign ?
		((~abs_product[30:0])+1'b1) :
		{1'b0, abs_product[30:0]};
  
//-------------------
// logical shifter
//-------------------
  wire KE_SC_SIGN = REG_KE_SC[5];

  wire [32:0] lsh_left =  {REG_KE_SR0, REG_KE_AC, REG_KE_MQ}
                        << REG_KE_SC[4:0];
  wire [32:0] lsh_right = {REG_KE_AC, REG_KE_MQ, REG_KE_SR0}
                        >> ((~REG_KE_SC[4:0])+1'b1);

//-------------------
// arithmetic shifter
//-------------------
  wire [31:0] ash_left = {REG_KE_SR0, REG_KE_AC[14:0], REG_KE_MQ}
	                << REG_KE_SC[4:0];
  wire [31:0] ash_right = {REG_KE_AC[14:0], REG_KE_MQ, REG_KE_SR0}
	                >> ((~REG_KE_SC[4:0])+1'b1);

//-------------------
// normalizer
//-------------------
  reg [7:0]  KE_nor_cnt;
  reg [31:0] KE_nor_ACMQ;
  wire	     KE_nor_start = (KE_operation == KE_OP_NOR);
  reg	     last_KE_nor_start;
  reg	     posedge_KE_nor_start;
  always @(posedge sys_clk ) begin
     last_KE_nor_start <= KE_nor_start;
     posedge_KE_nor_start <= KE_nor_start & ~last_KE_nor_start;
  end

  always @(posedge sys_clk )
    if( posedge_KE_nor_start) begin
       KE_nor_ACMQ <= {REG_KE_AC, REG_KE_MQ};
       KE_nor_cnt <= 0;
    end
    else if((KE_nor_ACMQ[31] == KE_nor_ACMQ[30]) &
	    (KE_nor_ACMQ != 32'b11000000_00000000_00000000_00000000) &
	    (KE_nor_cnt != 8'd31)) begin
       KE_nor_ACMQ[30:0] <= {KE_nor_ACMQ[29:0], 1'b0};
       KE_nor_cnt <= KE_nor_cnt + 1'd1;
    end
  
//-------------------
// Assignment of results
//-------------------
  parameter	KE_OP_NOP     = 4'd0;
  parameter	KE_OP_DIV     = 4'd1;
  parameter	KE_OP_MUL     = 4'd2;
  parameter	KE_OP_LOAD_AC = 4'd3;
  parameter	KE_OP_LOAD_MQ = 4'd4;
  parameter	KE_OP_LOAD_SC = 4'd5;
  parameter	KE_OP_ASH     = 4'd6;
  parameter	KE_OP_LSH     = 4'd7;
  parameter	KE_OP_NOR     = 4'd8;
  reg [3:0]	KE_operation;
  always @(posedge sys_clk or negedge RESET_n)
    if( ~RESET_n)
      {REG_KE_AC_OUT, REG_KE_MQ_OUT, REG_KE_SR0} <= 0;
    else if( negedge_ALE_n )
      case (KE_operation)
	KE_OP_NOP: ; // do nothing
	KE_OP_DIV: begin
	   REG_KE_MQ_OUT <= quotient[15:0];
	   REG_KE_AC_OUT <= remainder[15:0];
	   REG_KE_SR0    <= 1'b0;  
	end
	KE_OP_MUL: begin
	   {REG_KE_AC_OUT, REG_KE_MQ_OUT} <= product;
	   REG_KE_SR0 <= 1'b0;
	end
	KE_OP_LOAD_AC:
	  REG_KE_AC_OUT <= REG_KE_AC;
	
	KE_OP_LOAD_MQ: begin
	   REG_KE_MQ_OUT <= REG_KE_MQ;
	   REG_KE_AC_OUT <= REG_KE_AC;
	end
	KE_OP_LOAD_SC:
	  REG_KE_SC_OUT <= REG_KE_SC;

	KE_OP_LSH: begin
	   if(~KE_SC_SIGN)
	     {REG_KE_SR0, REG_KE_AC_OUT, REG_KE_MQ_OUT} <= lsh_left;
	   else
	     {REG_KE_AC_OUT, REG_KE_MQ_OUT, REG_KE_SR0} <= lsh_right;
	   REG_KE_SC_OUT <= 0;
	end
	
	KE_OP_ASH: begin
	  if(~KE_SC_SIGN)
	    {REG_KE_SR0, REG_KE_AC_OUT[14:0], REG_KE_MQ_OUT} <= ash_left;
	  else
	    {REG_KE_AC_OUT[14:0], REG_KE_MQ_OUT, REG_KE_SR0} <= ash_right;
	   REG_KE_SC_OUT <= 0;
	end

	KE_OP_NOR: begin
	   REG_KE_SR0 <= 1'b0;
	   REG_KE_SC_OUT <= KE_nor_cnt;
	   {REG_KE_AC_OUT, REG_KE_MQ_OUT} <= KE_nor_ACMQ;
	end
	default:;
      endcase
  
//-------------------
// Bus access 
//-------------------
  wire [15:0] DAL_sign_extended = aio_write_lowbyte ? 
	      {(DAL[7] ? 8'hFF: 8'h00), DAL[7:0]} // extend sign
	      : DAL;

  wire	      KE11_stretch = (KE11_stretch_cnt != 0);
  
  parameter   KE11_stretch_cnt_DIV = 8'd40;
  parameter   KE11_stretch_cnt_NOR = 8'd32;
  reg [7:0]   KE11_stretch_cnt;
  always @(posedge sys_clk or negedge RESET_n)
    if( ~RESET_n ) begin
       {REG_KE_AC, REG_KE_MQ, REG_KE_SC} <= 0;
       KE_operation <= KE_OP_NOP;
       KE11_stretch_cnt <= 0;
    end
    else if(KE11_stretch_cnt != 0)
      KE11_stretch_cnt <= KE11_stretch_cnt - 1'd1;
    else if( negedge_SCTL_n & bus_write )
      case (address)
	ADRS_KE_AC: begin
	   KE_operation <= KE_OP_LOAD_AC;
	   REG_KE_AC <= DAL_sign_extended;
	end
	ADRS_KE_MQ: begin
	   KE_operation <= KE_OP_LOAD_MQ;
	   REG_KE_MQ <= DAL_sign_extended;
	   REG_KE_AC <= DAL_sign_extended[15] ? 16'hFFFF: 16'h0000;
	end
	ADRS_KE_SC:  begin
	   KE_operation <= KE_OP_LOAD_SC;
	   REG_KE_SC <= DAL[7:0];
	end
	ADRS_KE_DIV: begin
	   KE_operation <= KE_OP_DIV;
	   REG_KE_X <= DAL_sign_extended;
	   REG_KE_AC <= REG_KE_AC_OUT;
	   REG_KE_MQ <= REG_KE_MQ_OUT;
	   KE11_stretch_cnt <= KE11_stretch_cnt_DIV;
	end
	ADRS_KE_MUL: begin
	   KE_operation <= KE_OP_MUL;
	   REG_KE_X <= DAL_sign_extended;
	   REG_KE_AC <= REG_KE_AC_OUT;
	   REG_KE_MQ <= REG_KE_MQ_OUT;
	end
	ADRS_KE_NOR: begin
	   KE_operation <= KE_OP_NOR;
	   REG_KE_AC <= REG_KE_AC_OUT;
	   REG_KE_MQ <= REG_KE_MQ_OUT;
	   KE11_stretch_cnt <= KE11_stretch_cnt_NOR;
	end
	ADRS_KE_LSH: begin
	   KE_operation <= KE_OP_LSH;
	   REG_KE_SC <= DAL[7:0];
	   REG_KE_AC <= REG_KE_AC_OUT;
	   REG_KE_MQ <= REG_KE_MQ_OUT;
	end
	ADRS_KE_ASH: begin
	   KE_operation <= KE_OP_ASH;
	   REG_KE_SC <= DAL[7:0];
	   REG_KE_AC <= REG_KE_AC_OUT;
	   REG_KE_MQ <= REG_KE_MQ_OUT;
	end
	default:;
      endcase
  
//---------------------------------------------------------------------------
// RL11 TTY console
//---------------------------------------------------------------------------
  reg [7:0]  tx_data;
  wire	     tx_ready;
  reg	     tx_send = 0;
  wire [7:0] rx_data;
  wire	     rx_data_ready;
  reg	     rx_clear;
  
  always @(posedge sys_clk)
    if( ~rx_data_ready )
      rx_clear <= 0;
    else if( (address == ADRS_RBUF) & bus_read)
      rx_clear <= 1;
  
  always @(posedge sys_clk)
    if( ~tx_ready )
      tx_send <= 1'b0;
    else if( negedge_SCTL_n & bus_write)
      if(address == ADRS_XBUF )
	{tx_data[7:0], tx_send} <= {DAL[7:0], 1'b1};
  
  always @(posedge sys_clk)
    if(negedge_SCTL_n & bus_write)
      if(address == ADRS_RCSR )
	RCSR_ID <= DAL[6];
      else if( address == ADRS_XCSR)
	XCSR_ID <= DAL[6];
      else if(address == ADRS_SWR)
	REG_SWR <= DAL[15:0];
       
//---------------------------------------------------------------------------
// PC-11(Paper-Tape Reader/Punch)
//---------------------------------------------------------------------------
  wire		 PRS_ERR;   // bit15
  wire		 PRS_BUSY;  // bit11
  wire		 PRS_DONE;  // bit7
  reg		 PRS_IE;    // bit6
  wire [7:0]	 PRB_DATA;  // bit7..0

  wire		 PPS_ERR;   // bit15
  wire		 PPS_READY; // bit7
  reg		 PPS_IE;    // bit6
  reg [7:0]	 PPB_DATA;  // bit7..0

  reg		 pt_read;
  reg		 pt_punch;
  reg		 pt_clear_done;
  wire		 pt_flush = 1'b0; // punch tape flush is disabled

  assign PRS_ERR = (pt_error != 0);
  assign PPS_ERR = (pt_error != 0);

  assign PRS_BUSY  = pt_read_busy;
  assign PRS_DONE  = pt_read_done;
  assign PPS_READY = pt_write_ready;
  
  always @(negedge SCTL_n or negedge RESET_n) // write to PRS
    if( ~RESET_n)
      {pt_read, PRS_IE} <= 0;
    else if((address == ADRS_PRS) & bus_write) begin
       if( ~PRS_BUSY )
	 pt_read <= DAL[0];  // set reader enable and update PRB_DATA
       PRS_IE  <= DAL[6];    // Reader Interrupt Enable
    end
    else if( PRS_BUSY | PRS_DONE )
      pt_read <= 1'b0;
  
  always @(posedge sys_clk) // read tape buffer
    if( (address[17:1] == (ADRS_PRB>>1)) & bus_read )
      pt_clear_done <= 1'b1;
    else if( ~PRS_DONE )
      pt_clear_done <= 1'b0;

  always @(negedge SCTL_n or negedge RESET_n) // write to PPS
    if( ~RESET_n)
      PPS_IE <= 0;
    else if((address == ADRS_PPS) & bus_write)
      PPS_IE     <= DAL[6]; // Punch Interrupt Enable

  always @(negedge SCTL_n) // write tape buffer
    if((address == ADRS_PPB) & bus_write) begin
       PPB_DATA <= DAL[7:0];
       if( PPS_READY )  // tape punch ready
	 pt_punch <= 1'b1;
    end
    else if( ~PPS_READY )
      pt_punch <= 1'b0;

//---------------------------------------------------------------------------
// RF11 (drum), RK11 (disk), RP11 (with RP03 disk pack) controllers
// and TM11 (magtape)
// RF:  1024 block                             ( 512KB x 1)
// RK:  6144 block (=256cyl *  2sur *12sect) 
//     (4872 block)(=203cyl *  2sur *12sect) (2.4 MB x 8)
// RP: 84000 block (=420cyl * 20tra *10sect)
//     (81200-83000 block) (=406-416cyl * 20tra *10sect) (40MB x 4)
// RP03's cylinder is 406, but 2.9BSD says 416.
//
// 1 block = 256word = 512B = 01000B
// SD memory block
//        0-  1023: RF  (00000000)
//     1024-  7167: RK0 (00002000)
//     7168- 13311: RK1 (00016000)
//    13312- 19455: RK2 (00032000)
//    19456- 25599: RK3 (00046000)
//    25600- 31742: RK4 (00062000)
//    31744- 37887: RK5 (00076000)
//    37888- 44031: RK6 (00112000)
//    44032- 50175: RK7 (00126000)
//    50176-134175: RP0 (00142000)
//   134176-218175: RP1 (00406040)
//   218176-302175: RP2 (00652100)
//   302176-386175: RP3 (01116140)
//
// # sample for making a sd image from unix disk drive images
// dd if=rf0 of=sd.dsk bs=512 
// (if no rf0 file,  dd if=/dev/zero of=sd.dsk bs=512 count=1024)
// dd if=rk0 of=sd.dsk seek=1024  conv=notrunc
// dd if=rk1 of=sd.dsk seek=7168  conv=notrunc
// dd if=rk2 of=sd.dsk seek=13312 conv=notrunc
// dd if=rk3 of=sd.dsk seek=19456 conv=notrunc
// ...
// dd if=rp0 of=sd.dsk seek=50176 conv=notrunc
// ...
//---------------------------------------------------------------------------
  parameter  RK_block_start = 20'd1024;  // following RF disk
  parameter  RP_block_start = 20'd50176; // following RF and RK disks
                                         // = 1024 + (6144 * 8)
  wire [19:0] RF_block_address;
  wire [19:0] RK_block_address;
  wire [19:0] RP_block_address;
  wire [19:0] TM_block_address;
  assign RF_block_address = {10'b0, REG_RF_DAE[1:0], REG_RF_DAR[15:8]};

//  assign RK_block_address = {5'b00000, REG_RKDA[15:4], 3'b000}
//			    + {2'b00, REG_RKDA[15:4], 2'b00}
//			    + {12'b00000000,   REG_RKDA[3:0]}
//			    + RK_block_start;

  assign RK_block_address = REG_RKDA[15:4] * 4'd12
                            + REG_RKDA[3:0]
                            + RK_block_start;

  assign RP_block_address
    = REG_RPCS[10:8]   * 17'd84000 // 10 * 20 * 420 cylinder/unit
      + REG_RPCA[8:0]  *  8'd200   // 10 * 20 track/cylinder
      + REG_RPDA[12:8] *  4'd10    // 10 track/sector
      + REG_RPDA[3:0]              // sector
      + RP_block_start;

//--------------------------------------------------------------------------
// TM11
// maximum file block = 65536 block = 32MB
//      0-  65535: file0 (00000000)
//  65536- 131071: file1 (00200000)
// 131072- 196607: file2 (00400000)
// 196608- 262143: file3 (00600000)
// 262144- 327679: file4 (01000000)
// 327680- 393215: file5 (01200000)
// 393216- 458751: file6 (01400000)
// 458752- 524287: file7 (01600000)
// ...
// 983040-1048575: file15 (03600000)
//
// # sample for making a sd image from unix disk drive images
// dd if=file0 of=sdtape.dsk
// dd if=file1 of=sdtape.dsk seek=65536 conv=notrunc
// dd if=file2 of=sdtape.dsk seek=131072 conv=notrunc
// dd if=file3 of=sdtape.dsk seek=196608 conv=notrunc
// dd if=file4 of=sdtape.dsk seek=262144 conv=notrunc
// dd if=file5 of=sdtape.dsk seek=327680 conv=notrunc
// dd if=file6 of=sdtape.dsk seek=393216 conv=notrunc
// dd if=file7 of=sdtape.dsk seek=458752 conv=notrunc
// ...
// -----------------------------------------------------------------------
  parameter  TM_FILEMAG = 16; // size of a file = 2^16 = 65536 block 

  assign TM_block_address = (TM_FILENUM << TM_FILEMAG) + TM_POS;
  
//--------------------------------------------------------------------------
// RF_READY, RK_READY, RP_READY, TM_READY
//--------------------------------------------------------------------------
  reg [2:0] disk_readys;
  reg [2:0] mt_readys;
  reg [2:0] DMA_dev0;
  reg [2:0] DMA_dev1;
  reg [2:0] DMA_dev2;
  wire	    posedge_disk_ready = disk_readys[1] & ~disk_readys[2];
  wire	    posedge_mt_ready = mt_readys[1] & ~mt_readys[2];
  wire [2:0] current_dev = DMA_dev2;
  always @(posedge sys_clk ) begin
     disk_readys[2:0] <= {disk_readys[1:0], disk_ready};
     mt_readys[2:0] <= {mt_readys[1:0], mt_ready};
     DMA_dev0 <= DMA_dev;
     DMA_dev1 <= DMA_dev0;
     DMA_dev2 <= DMA_dev1;
  end

  reg  RF_READY;
  always @(posedge sys_clk or negedge RESET_n)
    if( ~RESET_n )
      RF_READY <= 1'b1;
    else if( RF_go )
      RF_READY <= 0;
    else if(posedge_disk_ready & (current_dev == DEV_RF))
      RF_READY <= 1'b1;

  reg  RK_READY;
  always @(posedge sys_clk or negedge RESET_n)
    if( ~RESET_n )
      RK_READY <= 1'b1;
    else if( RK_go )
      RK_READY <= 0;
    else if(posedge_disk_ready & (current_dev == DEV_RK))
      RK_READY <= 1'b1;

  reg  RP_READY;
  always @(posedge sys_clk or negedge RESET_n)
    if( ~RESET_n )
      RP_READY <= 1'b1;
    else if( RP_go )
      RP_READY <= 0;
    else if(posedge_disk_ready & (current_dev == DEV_RP))
      RP_READY <= 1'b1;
  
  wire MTS_SELR  = ~mt_det_n;
  wire MTS_TUR   = mt_ready;
  wire MTC_CURDY = mt_ready;
  reg  TM_READY; // 'TM' not 'mt'. 'mt' is used for sd memory's ready signal
  always @(posedge sys_clk or negedge RESET_n)
    if( ~RESET_n )
      TM_READY <= 1'b1;
    else if( TM_go )
      TM_READY <= 0;
    else if(posedge_mt_ready & (current_dev == DEV_MT))
      TM_READY <= 1'b1;
  
//--------------------------------------------------------------------------
// device using DMA and SD2
//--------------------------------------------------------------------------
  reg [2:0] DMA_dev;
  reg [2:0] sd2_dev;

  parameter DEV_NONE = 3'd0;
  parameter DEV_RF   = 3'd1;
  parameter DEV_RK   = 3'd2;
  parameter DEV_RP   = 3'd3;
  parameter DEV_MT   = 3'd4;
  parameter DEV_PT   = 3'd5;
  
  always @( posedge sys_clk or negedge RESET_n)
    if( ~RESET_n )
      DMA_dev = DEV_NONE;
    else if( RF_go )
      DMA_dev <= DEV_RF;
    else if( RK_go )
      DMA_dev <= DEV_RK;
    else if( RP_go )
      DMA_dev <= DEV_RP;
    else if( TM_go )
      DMA_dev <= DEV_MT;

  always @( posedge sys_clk or negedge RESET_n)
    if( ~RESET_n )
      sd2_dev <= DEV_NONE;
    else if(mt_read | mt_write | mt_nop)
      sd2_dev <= DEV_MT;
    else if(pt_read | pt_punch | pt_flush)
      sd2_dev <= DEV_PT;
      
//--------------------------------------------------------------------------
// setup addresses on RF_go, RK_go, RP_go for sd1
//--------------------------------------------------------------------------
  always @(posedge sys_clk or negedge RESET_n)
    if( ~RESET_n ) begin
       {disk_read, disk_write, disk_nop} <= 0;
       {RF_go_clear, RK_go_clear, RP_go_clear} <= 0;
    end
    else if( disk_busy ) begin
       {disk_read, disk_write, disk_nop} <= 0;
       {RF_go_clear, RK_go_clear, RP_go_clear} <= 0;
    end
    else if( RF_go ) begin
       RF_go_clear <= 1'b1;
       disk_block_address <= {4'b0000, RF_disk_block_address};
       dma_start_address  <= RF_dma_start_address;
       dma_wordcount      <= RF_dma_wordcount;
       case(RF_command)
	 RF_DCS_NOP:         disk_nop   <= 1'b1; // not implemented
	 RF_DCS_READ:        disk_read  <= 1'b1;
	 RF_DCS_WRITE:       disk_write <= 1'b1;
	 RF_DCS_WRITE_CHECK: disk_nop   <= 1'b1; // not implemented
	 default:            disk_nop   <= 1'b1; // cannot reach here
       endcase
    end
    else if( RK_go ) begin
       RK_go_clear <= 1'b1;
       disk_block_address <= {4'b0000, RK_disk_block_address};
       dma_start_address  <= RK_dma_start_address;
       dma_wordcount      <= RK_dma_wordcount;
       case ( RK_command )
	 RKCS_CRESET:      disk_nop   <= 1'b1;
	 RKCS_READ:        disk_read  <= 1'b1;
	 RKCS_WRITE:       disk_write <= 1'b1;
	 RKCS_WRITE_CHECK: disk_nop   <= 1'b1; // not implemented
	 RKCS_SEEK:        disk_nop   <= 1'b1; // not implemented
	 RKCS_DRESET:      disk_nop   <= 1'b1;
	 RKCS_READ_CHECK:  disk_read  <= 1'b1;
	 RKCS_WRITE_LOCK:  disk_nop   <= 1'b1; // not implemented
	 default:          disk_nop   <= 1'b1; // cannot reach here
       endcase
    end
    else if( RP_go ) begin
       RP_go_clear <= 1'b1;
       disk_block_address <= {4'b0000, RP_disk_block_address};
       dma_start_address  <= RP_dma_start_address;
       dma_wordcount      <= RP_dma_wordcount;
       case ( RP_command )
	 RPCS_IDLE:         disk_nop   <= 1'b1;
	 RPCS_WRITE:        disk_write <= 1'b1;
	 RPCS_READ:         disk_read  <= 1'b1;
	 RPCS_WRITE_CHECK:  disk_nop   <= 1'b1; // not implemented
	 RPCS_SEEK:         disk_nop   <= 1'b1; // not implemented
	 RPCS_WRITE_NOSEEK: disk_nop   <= 1'b1; // not implemented
	 RPCS_HOME_SEEK:    disk_nop   <= 1'b1; // not implemented
	 RPCS_READ_NOSEEK:  disk_nop   <= 1'b1; // not implemented
	 default:           disk_nop   <= 1'b1; // cannot reach here
       endcase
    end

//--------------------------------------------------------------------------
// TM_go for sd2
//--------------------------------------------------------------------------
  always @(posedge sys_clk or negedge RESET_n)
    if( ~RESET_n )
      {mt_read, mt_write, mt_nop, TM_go_clear} <= 0;
    else if( mt_busy )
      {mt_read, mt_write, mt_nop, TM_go_clear} <= 0;
    else if( TM_go ) begin
       TM_go_clear <= 1'b1;
       sd2_disk_block_address <= {4'b0000, TM_disk_block_address};
       sd2_dma_start_address  <= TM_dma_start_address;
       sd2_dma_wordcount      <= TM_dma_wordcount;
       case ( TM_command )
	 MTC_OFFLINE:   mt_nop   <= 1'b1;
	 MTC_READ:      mt_read  <= 1'b1;
`ifdef USE_MT_WRITE_PROTECT
	 MTC_WRITE:     mt_nop <= 1'b1;
`else
	 MTC_WRITE:     mt_write <= 1'b1;
`endif
	 MTC_WRITE_EOF: mt_nop   <= 1'b1; // not implemented
	 MTC_SPACE_F:   mt_nop   <= 1'b1;
	 MTC_SPACE_R:   mt_nop   <= 1'b1;
	 MTC_WRITE_EXT: mt_nop   <= 1'b1; // not implemented
	 MTC_REWIND:    mt_nop   <= 1'b1;
	 default:; // cannot reach here
       endcase
    end

//---------------------------------------------------------------------------
// RF11 (drum) controller
//---------------------------------------------------------------------------
  reg [19:0]  RF_disk_block_address;
  reg [17:0]  RF_dma_start_address;
  reg [15:0]  RF_dma_wordcount;
  reg	      RF_go;	      
  reg	      RF_go_clear;
  reg [1:0]   RF_command;
  
  always @(posedge sys_clk or negedge RESET_n)
    if( ~RESET_n ) begin
       {REG_RF_DCS, REG_RF_WC,  REG_RF_CMA}     <= 0;
       {REG_RF_DAR, REG_RF_DAE, REG_RF_ADS}     <= 0;
       RF_go <= 0;
    end
    else if( RF_go_clear )
      RF_go <= 0;
    else if( negedge_SCTL_n & bus_write )
      if( address == ADRS_RF_DCS ) begin
	 REG_RF_DCS <= DAL;
	 if( DAL[8] ) begin // RF disk clear 
	    {REG_RF_DCS, REG_RF_WC,  REG_RF_CMA} <= 0;
	    {REG_RF_DAR, REG_RF_DAE, REG_RF_ADS} <= 0;
	 end
	 else if((DAL[0] == RF_DCS_GO) & RF_READY) begin
	    RF_go                       <= 1'b1;
	    RF_command                  <=DAL[2:1]; 
	    RF_disk_block_address       <= RF_block_address;
	    RF_dma_start_address        <= {DAL[5:4], REG_RF_CMA};
	    RF_dma_wordcount            <= REG_RF_WC;
	    
	    // update registers (this is not correct implementation)
	    REG_RF_WC  <= 0;
	    {REG_RF_DCS[5:4], REG_RF_CMA}  // modified for 18bit address
	      <= {DAL[5:4], REG_RF_CMA}
		 + (((~{2'b11,REG_RF_WC[15:0]}) + 1'b1)<<1);
//	    REG_RF_CMA <= REG_RF_CMA + (((~REG_RF_WC[15:0]) + 1'b1)<<1);
	    {REG_RF_DAE[1:0], REG_RF_DAR} <= {REG_RF_DAE[1:0], REG_RF_DAR} +
					     {(~REG_RF_WC[15:0]) + 1'b1};
	    
	    // for debug
	    REG_RF_DCS_BAK <= DAL;
	    REG_RF_WC_BAK  <= REG_RF_WC;
	    REG_RF_CMA_BAK <= REG_RF_CMA;
	    REG_RF_DAE_BAK <= REG_RF_DAE;
	    REG_RF_DAR_BAK <= REG_RF_DAR;
	    
	 end
      end
      else
	case (address)
	  ADRS_RF_WC:  REG_RF_WC  <= DAL;
	  ADRS_RF_CMA: REG_RF_CMA <= DAL;
	  ADRS_RF_DAR: REG_RF_DAR <= DAL;
	  ADRS_RF_DAE: REG_RF_DAE <= DAL;
	  ADRS_RF_ADS: REG_RF_ADS <= DAL;
	  default:;
	endcase	
  
//---------------------------------------------------------------------------
// RK11 (disk) controller
//---------------------------------------------------------------------------
  reg [19:0]  RK_disk_block_address;
  reg [17:0]  RK_dma_start_address;
  reg [15:0]  RK_dma_wordcount;
  reg	      RK_go;
  reg	      RK_go_clear;
  reg [2:0]   RK_command;

  always @(posedge sys_clk or negedge RESET_n)
    if( ~RESET_n ) begin
       {REG_RKCS, REG_RKWC, REG_RKBA, REG_RKDA} <= 0;
       RK_go <= 0;
    end
    else if( RK_go_clear )
      RK_go <= 0;
    else if( negedge_SCTL_n & bus_write )
      if(address == ADRS_RKCS ) begin
	 REG_RKCS   <= DAL;
	 if((DAL[0] == RKCS_GO) & RK_READY) begin
	    RK_go       <= 1'b1;
	    RK_command  <= DAL[3:1];
	    if( DAL[3:1] == RKCS_READ || DAL[3:1] == RKCS_WRITE) begin
	       RK_disk_block_address <= RK_block_address;
	       RK_dma_start_address <= {DAL[5:4], REG_RKBA};
	       RK_dma_wordcount     <= REG_RKWC;
	       // update registers (this is not correct implementation)
	       REG_RKWC <= 0;
	       if( ~REG_RKCS[11] ) // Inhibit incrementing the RKBA (IBA)
		 {REG_RKCS[5:4], REG_RKBA}  // for 18bit address
		   <= {DAL[5:4], REG_RKBA}
		      + (((~{2'b11,REG_RKWC[15:0]}) + 1'b1)<<1);
	       // REG_RKBA <= REG_RKBA + (((~REG_RKWC[15:0]) + 1'b1)<<1);
	       // for debug
	       REG_RKCS_BAK <= DAL;
	       REG_RKWC_BAK <= REG_RKWC;
	       REG_RKBA_BAK <= REG_RKBA;
	    end
	    case ( DAL[3:1] ) // REG_RKCS[13] is SCP(Search complete) bit
	      RKCS_CRESET: {REG_RKCS, REG_RKWC, REG_RKBA, REG_RKDA} <= 0;
	      RKCS_DRESET: REG_RKCS[13] <= 1'b1;
	      RKCS_SEEK:   REG_RKCS[13] <= 1'b1;
	      default:     REG_RKCS[13] <= 0;
	    endcase
	 end
      end
      else
	case (address)
	  ADRS_RKWC:   REG_RKWC   <= DAL;
	  ADRS_RKBA:   REG_RKBA   <= DAL;
	  ADRS_RKDA:   REG_RKDA   <= DAL;
	  default:;
	endcase	
  
//---------------------------------------------------------------------------
// RP11 disk controller
//---------------------------------------------------------------------------
  reg [19:0]  RP_disk_block_address;
  reg [17:0]  RP_dma_start_address;
  reg [15:0]  RP_dma_wordcount;
  reg	      RP_go;
  reg	      RP_go_clear;
  reg [2:0]   RP_command;

  always @(posedge sys_clk or negedge RESET_n)
    if( ~RESET_n ) begin
       {REG_RPCS, REG_RPWC, REG_RPBA, REG_RPCA, REG_RPDA} <= 0;
       RP_go <= 0;
    end
    else if( RP_go_clear )
      RP_go <= 0;
    else if( negedge_SCTL_n & bus_write )
      if(address == ADRS_RPCS ) begin
	 REG_RPCS   <= DAL;
	 if((DAL[0] == RPCS_GO) & RP_READY) begin
	    RP_go       <= 1'b1;
	    RP_command  <= DAL[3:1];
	    if( DAL[3:1] == RPCS_READ || DAL[3:1] == RPCS_WRITE) begin
	       RP_disk_block_address <= RP_block_address;
 	       RP_dma_start_address <= {DAL[5:4], REG_RPBA};
	       RP_dma_wordcount     <= REG_RPWC;
	       // update registers (this is not correct implementation)
	       REG_RPWC <= 0;
//	       if( ~REG_RPCS[11] ) // Inhibit incrementing the RPBA (IBA)
//		 {REG_RPCS[5:4], REG_RPBA}  // for 18bit address
//		   <= {DAL[5:4], REG_RPBA}
//		      + (((~{2'b11,REG_RPWC[15:0]}) + 1'b1)<<1);
	       // REG_RPBA <= REG_RPBA + (((~REG_RPWC[15:0]) + 1'b1)<<1);
	       // for debug
	       REG_RPCS_BAK <= DAL;
	       REG_RPWC_BAK <= REG_RPWC;
	       REG_RPBA_BAK <= REG_RPBA;
	    end
	    case ( DAL[3:1] )
	      RPCS_IDLE: {REG_RPCS, REG_RPWC, REG_RPBA,
			  REG_RPCA, REG_RPDA} <= 0;
	      default:;
	    endcase
	 end
      end
      else
	case (address)
	  ADRS_RPWC:   REG_RPWC   <= DAL;
	  ADRS_RPBA:   REG_RPBA   <= DAL;
	  ADRS_RPCA:   REG_RPCA   <= DAL;
	  ADRS_RPDA:   REG_RPDA   <= DAL;
	  ADRS_RPM1:   REG_RPM1   <= DAL;
	  default:;
	endcase	

//---------------------------------------------------------------------------
// TM11 magnetic tape controller (MT)
//---------------------------------------------------------------------------
  reg [19:0]  TM_disk_block_address;
  reg [17:0]  TM_dma_start_address;
  reg [15:0]  TM_dma_wordcount;
  reg	      TM_go;
  reg	      TM_go_clear;
  reg [2:0]   TM_command;
  reg [19:0]  TM_POS = 0;
  reg [3:0]   TM_FILENUM = 0;
  parameter   TM_MAXFILES = 4'd15;
  
  always @(posedge sys_clk or negedge RESET_n)
    if( ~RESET_n ) begin
       {REG_MTC, REG_MTBRC, REG_MTCMA, REG_MTD, REG_MTRD} <= 0;
       TM_go <= 0;
       TM_POS <= 0;
    end
    else if( TM_go_clear ) begin
       TM_go <= 0;
       REG_MTC[0] <= 1'b0;
    end
    else if( negedge_SCTL_n & bus_write )
      if(address == ADRS_MTC ) begin
	 REG_MTC   <= DAL;
	 if((DAL[0] == MTC_GO) & TM_READY) begin
	    TM_go       <= 1'b1;
	    TM_command  <= DAL[3:1];
	    // for debug
	    REG_MTC_BAK   <= DAL;
	    REG_MTBRC_BAK <= REG_MTBRC;
	    REG_MTCMA_BAK <= REG_MTCMA;
	    if( DAL[3:1] == MTC_READ || DAL[3:1] == MTC_WRITE) begin
	       TM_disk_block_address <= TM_block_address;
 	       TM_dma_start_address  <= {DAL[5:4], REG_MTCMA};
	       TM_dma_wordcount      <= {1'b1, REG_MTBRC[15:1]};
	       // update registers (this is not correct implementation)
	       TM_POS      <= TM_POS
			      + (((~REG_MTBRC+1'b1))>>9 & 7'b111_1111);
	       REG_MTBRC <= 0;
	       {REG_MTC[5:4], REG_MTCMA}  // for 18bit address
		 <= {DAL[5:4], REG_MTCMA}
		    + (((~{3'b111,REG_MTBRC[15:1]}) + 1'b1)<<1);
	       // REG_MTCMA <= REG_MTCMA + (((~REG_MTBRC[15:0]) + 1'b1)<<1);
	    end
	    case ( DAL[3:1] )
	      MTC_OFFLINE: begin
		 {REG_MTC, REG_MTBRC, REG_MTCMA, REG_MTD, REG_MTRD} <= 0;
		 {TM_FILENUM, TM_POS} <= 0;
	      end
	      MTC_SPACE_F: begin
		 // I'm not sure this is correct,
		 // but it works for unix v7 'mt -t /dev/nrmt0 fsf n'
		 // which calls SPACE_F with MTBRC='o100010
		 TM_FILENUM <= (TM_FILENUM - REG_MTBRC[2:0] + 1'd1) 
		   & TM_MAXFILES;
//		 TM_FILENUM <= (TM_FILENUM - REG_MTBRC) & TM_MAXFILES;
		 TM_POS <= 0;
		 REG_MTBRC <= 0;
	      end
	      MTC_SPACE_R: begin
		 if( (TM_POS == 0) & (TM_FILENUM != 0))
		   TM_FILENUM <= (TM_FILENUM + REG_MTBRC[2:0]) & TM_MAXFILES;
		 TM_POS <= 0;
		 REG_MTBRC <= 0;
	      end
	      MTC_REWIND:
		{TM_FILENUM, TM_POS} <= 0;
	      default:;
	    endcase
	 end
      end
      else // address != ADRS_MTC
	case (address)
	  ADRS_MTBRC:   REG_MTBRC  <= DAL;
	  ADRS_MTCMA:   REG_MTCMA  <= DAL;
	  ADRS_MTD:     REG_MTD    <= DAL;
	  ADRS_MTRD:    REG_MTRD   <= DAL;
	  default:;
	endcase	
    else if( sd2_nocard ) // when card removed
      {TM_FILENUM, TM_POS} <= 0;

  reg TM_IE_NOGO;
  always @( posedge sys_clk )
    if(negedge_SCTL_n & (address == ADRS_MTC) & bus_write
       & DAL[6] & ~DAL[0]) // set INT ENABLE but not set GO
      TM_IE_NOGO <= 1'b1;
    else
      TM_IE_NOGO <= 0;

//---------------------------------------------------------------------------
// Interrupt
//---------------------------------------------------------------------------
  // interrupt levels  
  parameter LV_IRQ0 = 3'd4;
  parameter LV_IRQ1 = 3'd5;
//  parameter LV_IRQ2 = 3'd6;
//  parameter LV_IRQ3 = 3'd7;

//---------------------------------------------------------------------------
// unimplemented IRQs
//---------------------------------------------------------------------------
  assign IRQ2  = 0;
//  assign IRQ3  = 0;

// vector address for unimplemented IRQs
//  wire [15:0]	 VA_IRQ2 = 16'o0;
//  wire [15:0]	 VA_IRQ3 = 16'o0;

//---------------------------------------------------------------------------
// Interrupt by TTY I/O or Paper Tape Read/Punch
// IRQ0 (IRQ level=4)
//---------------------------------------------------------------------------
  reg IRQ_ttyi;
  reg IRQ_ttyo;
  reg IRQ_ppti;
  reg IRQ_ppto;
  assign      IRQ0 = IRQ_ttyi | IRQ_ttyo | IRQ_ppti | IRQ_ppto;

 // Interrupt vector addresses
  parameter   VA_ttyi  = 16'o000060;
  parameter   VA_ttyo  = 16'o000064;
  parameter   VA_ppti  = 16'o000070;
  parameter   VA_ppto  = 16'o000074;
  reg [15:0]  VA_IRQ0;

  always @(posedge sys_clk)
    if(negedge_ALE_n)
      VA_IRQ0 <= IRQ_ttyi ? VA_ttyi :
		 IRQ_ttyo ? VA_ttyo :
		 IRQ_ppti ? VA_ppti :
		 VA_ppto;

  wire	 ack_IRQ0 = (aio_code == AIO_INTACK && iack_level == LV_IRQ0);

  reg [2:0] ack_IRQ0s;
  wire	 posedge_ack_IRQ0 = ack_IRQ0s[1] & ~ack_IRQ0s[2];
  always @( negedge sys_clk )
    ack_IRQ0s[2:0] <= {ack_IRQ0s[1:0], ack_IRQ0};

  reg [2:0] rx_data_readys;
  wire posedge_rx_data_ready = rx_data_readys[1] & ~rx_data_readys[2];
  always @( posedge sys_clk )
    rx_data_readys[2:0] <= {rx_data_readys[1:0], rx_data_ready};

  reg [2:0] tx_readys;
  wire posedge_tx_ready = tx_readys[1] & ~tx_readys[2];
  always @( posedge sys_clk )
    tx_readys[2:0] <= {tx_readys[1:0], tx_ready};

  reg [2:0] PRS_DONEs;
  wire posedge_PRS_DONE = PRS_DONEs[1] & ~PRS_DONEs[2];
  always @( posedge sys_clk )
    PRS_DONEs[2:0] <= {PRS_DONEs[1:0], PRS_DONE};
  
  reg [2:0] PPS_READYs;
  wire posedge_PPS_READY = PPS_READYs[1] & ~PPS_READYs[2];
  always @( posedge sys_clk )
    PPS_READYs[2:0] <= {PPS_READYs[1:0], PPS_READY};
  
  always @( posedge sys_clk or negedge RESET_n)
    if( ~RESET_n)
      IRQ_ttyi <= 0;
    else if( posedge_rx_data_ready)
      IRQ_ttyi <= RCSR_ID;
    else if( posedge_ack_IRQ0 & (VA_IRQ0 == VA_ttyi))
      IRQ_ttyi <= 0;
    else if( ~rx_data_ready ) // additional dirty clear logic
      IRQ_ttyi <= 0;          // to clear IRQ when exiting console ODT

  always @( posedge sys_clk or negedge RESET_n)
    if( ~RESET_n)
      IRQ_ttyo <= 0;
    else if( posedge_tx_ready |
	     // This is a dirty workaround for RT-11 v4
	     (tx_ready & negedge_clk_60Hz & flag_rt11_workaround)
	     ) 
      IRQ_ttyo <= XCSR_ID;
    else if( posedge_ack_IRQ0 & (VA_IRQ0 == VA_ttyo))
      IRQ_ttyo <= 0;
// additional dirty clear logics
//    else if( (address == ADRS_XCSR) || (address == ADRS_XBUF))
//      IRQ_ttyo <= 0;   // clear by access tor tx registers
//    else if( ~tx_ready)
//      IRQ_ttyo <= 0;   // clear when tx is not ready

  always @( posedge sys_clk or negedge RESET_n)
    if( ~RESET_n)
      IRQ_ppti <= 0;
    else if( posedge_PRS_DONE)
      IRQ_ppti <= PRS_IE;
    else if( posedge_ack_IRQ0 & (VA_IRQ0 == VA_ppti))
      IRQ_ppti <= 0;
  
  always @( posedge sys_clk or negedge RESET_n)
    if( ~RESET_n)
      IRQ_ppto <= 0;
    else if( posedge_PPS_READY)
      IRQ_ppto <= PPS_IE;
    else if( posedge_ack_IRQ0 & (VA_IRQ0 == VA_ppto))
      IRQ_ppto <= 0;

//---------------------------------------------------------------------------
// Interrupt by RF(drum)/RK(disk)/RP ready/MT ready
// IRQ1 (IRQ level=5)
//---------------------------------------------------------------------------
  reg		 IRQ_RF;
  reg		 IRQ_RK;
  reg		 IRQ_RP;
  reg		 IRQ_MT;

  assign IRQ1           = IRQ_RF | IRQ_RK | IRQ_RP | IRQ_MT;

  parameter	 VA_RF     = 16'o000204; // RF(drum)
  parameter	 VA_RK     = 16'o000220; // RK(disk)
  parameter	 VA_RP     = 16'o000254; // RP(disk pack)
  parameter	 VA_MT     = 16'o000224; // MT(magnetic tape)

  reg [15:0]  VA_IRQ1;
  always @(posedge sys_clk)
    if(negedge_ALE_n)
      VA_IRQ1 <= IRQ_RF ? VA_RF :
		 IRQ_RK ? VA_RK :
		 IRQ_RP ? VA_RP :
		 VA_MT;

  wire	 ack_IRQ1 = (aio_code == AIO_INTACK && iack_level == LV_IRQ1);

  reg  [2:0] ack_IRQ1S;
  wire	     posedge_ack_IRQ1 = ack_IRQ1S[0] & ~ack_IRQ1S[2];
  always @( negedge sys_clk ) 
    ack_IRQ1S[2:0] <= {ack_IRQ1S[1:0], ack_IRQ1};
  
  reg [2:0] RF_READYS;
  wire	    posedge_RF_READY = RF_READYS[0] & ~RF_READYS[2];
  always @( negedge sys_clk )
    RF_READYS[2:0] <= {RF_READYS[1:0], RF_READY};
  
  reg [2:0] RK_READYS;
  wire	 posedge_RK_READY = RK_READYS[0] & ~RK_READYS[2];
  always @( negedge sys_clk )
    RK_READYS[2:0] <= {RK_READYS[1:0], RK_READY};

  reg [2:0] RP_READYS;
  wire	 posedge_RP_READY = RP_READYS[0] & ~RP_READYS[2];
  always @( negedge sys_clk )
    RP_READYS[2:0] <= {RP_READYS[1:0], RP_READY};

  reg [2:0] TM_READYS;
  wire	 posedge_TM_READY = TM_READYS[0] & ~TM_READYS[2];
  always @( negedge sys_clk )
    TM_READYS[2:0] <= {TM_READYS[1:0], TM_READY};

  wire	 RF_INT_ENABLE = REG_RF_DCS[6];
  wire	 RK_INT_ENABLE = REG_RKCS[6];
  wire	 RP_INT_ENABLE = REG_RPCS[6];
  wire	 TM_INT_ENABLE = REG_MTC[6];

  always @( posedge sys_clk )
    if( ~RESET_n )
      IRQ_RF <= 0;
    else if( posedge_RF_READY )
      IRQ_RF <= RF_INT_ENABLE;
    else if( posedge_ack_IRQ1 & (VA_IRQ1 == VA_RF))
      IRQ_RF <= 0;
  
  always @( posedge sys_clk )
    if( ~RESET_n )
      IRQ_RK <= 0;
    else if( posedge_RK_READY )
      IRQ_RK <= RK_INT_ENABLE;
    else if( posedge_ack_IRQ1 & (VA_IRQ1 == VA_RK))
      IRQ_RK <= 0;
  
  always @( posedge sys_clk )
    if( ~RESET_n )
      IRQ_RP <= 0;
    else if( posedge_RP_READY )
      IRQ_RP <= RP_INT_ENABLE;
    else if( posedge_ack_IRQ1 & (VA_IRQ1 == VA_RP))
      IRQ_RP <= 0;

  always @( posedge sys_clk )
    if( ~RESET_n )
      IRQ_MT <= 0;
    else if( posedge_TM_READY | TM_IE_NOGO)
      IRQ_MT <= TM_INT_ENABLE;
    else if( posedge_ack_IRQ1 & (VA_IRQ1 == VA_MT))
      IRQ_MT <= 0;
  
//---------------------------------------------------------------------------
// EVENT (timer) 
// IRQ level=6
//---------------------------------------------------------------------------
  assign EVENT_n = ~IRQ_timer; // EVENT_n(level 6, address=0100)
  wire EVENT_ACK = (aio_code == AIO_GPWRITE) & (gpcode == GP_ACK_EVENT);

//---------------------------------------------------------------------------
// KW11-L line time clock
//---------------------------------------------------------------------------
  parameter   ADRS_KW11L  = 18'o777546;
  reg	      REG_KW11L_INT_ENABLE;  // bit 6
  reg	      REG_KW11L_INT_MONITOR; // bit 7
  wire [15:0] REG_KW11L = {8'b0, 
			   REG_KW11L_INT_MONITOR, REG_KW11L_INT_ENABLE,
			   6'b0};
  wire	      IRQ_timer = REG_KW11L_INT_ENABLE & REG_KW11L_INT_MONITOR;

  reg	     clk_60Hz = 0;
  reg [19:0] cnt_8333us;
  reg	     negedge_clk_60Hz;
  always @(posedge sys_clk)
    if(cnt_8333us == SYS_CLK_FRQ/120) begin
       cnt_8333us       <= 0;
       clk_60Hz         <= ~clk_60Hz;
       negedge_clk_60Hz <= clk_60Hz; // == clk_60Hz ? 1'b1: 1'b0;
    end
    else begin
       cnt_8333us <= cnt_8333us + 1'b1;
       negedge_clk_60Hz <= 0;
    end

  reg EVENT_ACK0;
  reg EVENT_ACK1;
  wire posedge_EVENT_ACK = EVENT_ACK0 & ~EVENT_ACK1;
  always @(negedge sys_clk) begin
     EVENT_ACK0 <= EVENT_ACK;
     EVENT_ACK1 <= EVENT_ACK0;
  end
  
  always @( posedge sys_clk or negedge RESET_n)
    if( ~RESET_n ) begin
       REG_KW11L_INT_ENABLE <= 0;
       REG_KW11L_INT_MONITOR <= 1'b1; // set on processor INIT
    end
    else if( posedge_EVENT_ACK )
      REG_KW11L_INT_MONITOR <= 0;
    else if( negedge_SCTL_n & bus_write & (address == ADRS_KW11L)) begin
       REG_KW11L_INT_ENABLE  <= DAL[6];
       REG_KW11L_INT_MONITOR <= 0;
    end
    else if( negedge_clk_60Hz )
      REG_KW11L_INT_MONITOR <= 1'b1; // set interrupt flag
  
//---------------------------------------------------------------------------
// UART
//---------------------------------------------------------------------------
  uart_rx#
    ( .CLK_FRQ(SYS_CLK_FRQ),
      .BAUD_RATE(UART_BPS)
      ) uart_rx_inst
      (.clk           (sys_clk      ),
       .reset_n       (RESET_n      ),
       .rx_data       (rx_data      ),
       .rx_data_ready (rx_data_ready),
       .rx_clear      (rx_clear),
       .rx_in         (uart_rx      )
       );

  uart_tx#
    (.CLK_FRQ(SYS_CLK_FRQ),
     .BAUD_RATE(UART_BPS)
     ) uart_tx_inst
      (.clk           (sys_clk),
       .reset_n       (RESET_n),
       .tx_data       (tx_data),
       .tx_send       (tx_send),
       .tx_ready      (tx_ready),
       .tx_out        (uart_tx)
       );

//---------------------------------------------------------------------------
// DMA from HDD and MT
//---------------------------------------------------------------------------
  reg [23:0]  disk_block_address;
  reg [17:0]  dma_start_address;
  reg [15:0]  dma_wordcount;

  wire [7:0]  d_dma_to_ram_hd;
  wire [7:0]  d_dma_to_ram_mt;

  wire [17:0] dma_address_hd;
  wire [17:0] dma_address_mt;
  wire [17:0] dma_address = (DMA_dev == DEV_MT) ? dma_address_mt:
	                                         dma_address_hd;
  
  wire [7:0] d_dma_to_ram = (DMA_dev == DEV_MT) ? d_dma_to_ram_mt:
                                                  d_dma_to_ram_hd;

  wire	      dma_write   = (DMA_dev == DEV_MT) ? dma_write_mt:
	                                         dma_write_hd;

//---------------------------------------------------------------------------
// SD memory Hard disk emulator
//---------------------------------------------------------------------------
  reg	      disk_read;
  reg	      disk_write;
  reg	      disk_nop;
  wire	      disk_ready;
  wire	      disk_busy  = ~disk_ready;
  wire [4:0]  sd_state;
  wire [3:0]  sd_error;

  sdhd #( .SYS_FRQ(SD_SYS_FRQ),
	  .MEM_FRQ(SD_MEM_FRQ)
	  ) sdhd_inst
  (.i_clk                (sd_sys_clk),
   .i_reset_n            (SD_RESET_n),
   .i_sd_det_n           (1'b0),  // Tang Console's sd_det_n seems not working
   //   .i_sd_det_n           (sd_det_n),
   .i_sd_miso            (sd_miso),
   .o_sd_mosi            (sd_mosi),
   .o_sd_cs_n            (sd_cs_n),
   .o_sd_clk             (sd_clk),
   .o_disk_ready         (disk_ready),
   .i_disk_read          (disk_read),
   .i_disk_write         (disk_write),
   .i_disk_nop           (disk_nop),
   .i_disk_block_address (disk_block_address),
   .o_dma_address        (dma_address_hd),
   .i_dma_start_address  (dma_start_address),
   .i_dma_wordcount      (dma_wordcount),
   .i_dma_data           (d_ram_to_dma),
   .o_dma_data           (d_dma_to_ram_hd),
   .o_dma_write          (dma_write_hd),
   .o_sd_state           (sd_state),
   .o_sd_error           (sd_error)
   );

//---------------------------------------------------------------------------
// Secondary SD memory Card
//---------------------------------------------------------------------------
  reg [23:0]  sd2_disk_block_address;
  reg [17:0]  sd2_dma_start_address;
  reg [15:0]  sd2_dma_wordcount;

  assign {sd2_clk, sd2_mosi, sd2_cs_n}
    = (sd2_dev == DEV_MT) ? {mt_clk, mt_mosi, mt_cs_n}:
                            {pt_clk, pt_mosi, pt_cs_n};

  wire sd2_nocard = sd2_det_n;
  
//---------------------------------------------------------------------------
// SD memory tape emulator
//---------------------------------------------------------------------------
  wire pt_clk;
  wire pt_mosi;
  wire pt_miso = sd2_miso;
  wire pt_cs_n;
  wire pt_det_n = sd2_det_n;

  wire pt_read_busy;
  wire pt_read_done;
  wire pt_write_ready;

  wire [3:0]  pt_error;
  wire [4:0]  pt_state;
  wire [30:0] pt_address;
  wire [30:0] pt_address_write;
   
  sdtape #( .SYS_FRQ(SD_SYS_FRQ),
	    .MEM_FRQ(SD_MEM_FRQ)
	    ) sdtape_inst
    (.i_clk              (sd_sys_clk),
     .i_reset_n          (SD_RESET_n),
     .i_sd_det_n         (pt_det_n),
     .i_sd_miso          (pt_miso),
     .o_sd_mosi          (pt_mosi),
     .o_sd_cs_n          (pt_cs_n),
     .o_sd_clk           (pt_clk),
     .o_tape_read_busy   (pt_read_busy),
     .o_tape_read_done   (pt_read_done),
     .o_tape_punch_ready (pt_write_ready),
     .i_tape_read        (pt_read),
     .i_tape_punch       (pt_punch),
     .i_tape_clear_done  (pt_clear_done),
     .i_tape_punch_data  (PPB_DATA),
     .o_tape_read_data   (PRB_DATA),
     .i_tape_flush       (pt_flush),
     .o_sd_state         (pt_state),
     .o_sd_error         (pt_error),
     .o_sd_tape_address  (pt_address),
     .o_sd_tape_address_write (pt_address_write)
     );
  
//---------------------------------------------------------------------------
// Magnetic Tape emulater (using Hard disk emulator)
//---------------------------------------------------------------------------
  wire mt_clk;
  wire mt_mosi;
  wire mt_miso = sd2_miso;
  wire mt_cs_n;
  wire mt_det_n = sd2_det_n;

  reg	      mt_read;
  reg	      mt_write;
  reg	      mt_nop;
  wire	      mt_ready;
  wire	      mt_busy = ~mt_ready;
  wire [4:0]  mt_state;
  wire [3:0]  mt_error;

  sdhd #(.SYS_FRQ(SD_SYS_FRQ),
	 .MEM_FRQ(SD_MEM_FRQ)
	 ) sdhd_mt_inst
    (.i_clk                (sd_sys_clk),
     .i_reset_n            (SD_RESET_n),
     .i_sd_det_n           (mt_det_n),
     .i_sd_miso            (mt_miso),
     .o_sd_mosi            (mt_mosi),
     .o_sd_cs_n            (mt_cs_n),
     .o_sd_clk             (mt_clk),
     .o_disk_ready         (mt_ready),
     .i_disk_read          (mt_read),
     .i_disk_write         (mt_write),
     .i_disk_nop           (mt_nop),
     .i_disk_block_address (sd2_disk_block_address),
     .o_dma_address        (dma_address_mt),
     .i_dma_start_address  (sd2_dma_start_address),
     .i_dma_wordcount      (sd2_dma_wordcount),
     .i_dma_data           (d_ram_to_dma),
     .o_dma_data           (d_dma_to_ram_mt),
     .o_dma_write          (dma_write_mt),
     .o_sd_state           (mt_state),
     .o_sd_error           (mt_error)
     );


//---------------------------------------------------------------------------
// for debug
//---------------------------------------------------------------------------
  reg reg_HALT_SW;
  reg [27:0]	 halt_cnt = 0;
  parameter	 HALT_WIDTH = (INIT_CLK_FRQ / 1000) * 50; // 50ms
  always @(posedge init_clk)
    if(HALT_SW)
      if (halt_cnt == HALT_WIDTH)
	reg_HALT_SW <= 1'b1;
      else
	halt_cnt <= halt_cnt + 1'd1;
    else
      {reg_HALT_SW, halt_cnt} <= 0;

  reg	    dbg_trg;
  assign HALT = reg_HALT_SW | dbg_trg;

  reg [15:0] REG_TRACE[23:0];
  parameter  ADRS_TRACE23 = 18'o777000;
  parameter  ADRS_TRACE22 = 18'o777002;
  parameter  ADRS_TRACE21 = 18'o777004;
  parameter  ADRS_TRACE20 = 18'o777006;
  parameter  ADRS_TRACE19 = 18'o777010;
  parameter  ADRS_TRACE18 = 18'o777012;
  parameter  ADRS_TRACE17 = 18'o777014;
  parameter  ADRS_TRACE16 = 18'o777016;
  parameter  ADRS_TRACE15 = 18'o777020;
  parameter  ADRS_TRACE14 = 18'o777022;
  parameter  ADRS_TRACE13 = 18'o777024;
  parameter  ADRS_TRACE12 = 18'o777026;
  parameter  ADRS_TRACE11 = 18'o777030;
  parameter  ADRS_TRACE10 = 18'o777032;
  parameter  ADRS_TRACE9  = 18'o777034;
  parameter  ADRS_TRACE8  = 18'o777036;
  parameter  ADRS_TRACE7  = 18'o777040;
  parameter  ADRS_TRACE6  = 18'o777042;
  parameter  ADRS_TRACE5  = 18'o777044;
  parameter  ADRS_TRACE4  = 18'o777046;
  parameter  ADRS_TRACE3  = 18'o777050;
  parameter  ADRS_TRACE2  = 18'o777052;
  parameter  ADRS_TRACE1  = 18'o777054;
  parameter  ADRS_TRACE0  = 18'o777056;

  always @(posedge sys_clk or negedge RESET_n)
    if( ~RESET_n ) begin
       {REG_TRACE[3], REG_TRACE[2], REG_TRACE[1], REG_TRACE[0]} <= 0;
       {REG_TRACE[7], REG_TRACE[6], REG_TRACE[5], REG_TRACE[4]} <= 0;
       {REG_TRACE[11], REG_TRACE[10], REG_TRACE[9], REG_TRACE[8]} <= 0;
       {REG_TRACE[15], REG_TRACE[14], REG_TRACE[13], REG_TRACE[12]} <= 0;
       {REG_TRACE[19], REG_TRACE[18], REG_TRACE[17], REG_TRACE[16]} <= 0;
       {REG_TRACE[23], REG_TRACE[22], REG_TRACE[21], REG_TRACE[20]} <= 0;
    end
//    else if(posedge_ALE_n & (aio_read | aio_write) & 
//	    (address[15:12]==4'o17) &
//	    (address[15:0] != 16'o177776) &
//	    (address[15:3] != 13'o17756) &
//	    (address[15:6] != 10'o1774)  &
//	    (address[15:6] != 10'o1770)
//	    ) begin
    else if(negedge_ALE_n & aio_iread) begin
       // record @posedge_ALE_n as address is latched @negedge ALE_n
       {REG_TRACE[3] , REG_TRACE[2] , REG_TRACE[1] , REG_TRACE[0]}  <=
       {REG_TRACE[2] , REG_TRACE[1] , REG_TRACE[0] , address[15:0]};
       {REG_TRACE[7] , REG_TRACE[6] , REG_TRACE[5] , REG_TRACE[4]}  <=
       {REG_TRACE[6] , REG_TRACE[5] , REG_TRACE[4] , REG_TRACE[3]};
       {REG_TRACE[11], REG_TRACE[10], REG_TRACE[9] , REG_TRACE[8]}  <=
       {REG_TRACE[10], REG_TRACE[9] , REG_TRACE[8] , REG_TRACE[7]};
       {REG_TRACE[15], REG_TRACE[14], REG_TRACE[13], REG_TRACE[12]} <=
       {REG_TRACE[14], REG_TRACE[13], REG_TRACE[12], REG_TRACE[11]};
       {REG_TRACE[19], REG_TRACE[18], REG_TRACE[17], REG_TRACE[16]} <=
       {REG_TRACE[18], REG_TRACE[17], REG_TRACE[16], REG_TRACE[15]};
       {REG_TRACE[23], REG_TRACE[22], REG_TRACE[21], REG_TRACE[20]} <=
       {REG_TRACE[22], REG_TRACE[21], REG_TRACE[20], REG_TRACE[19]};
    end

  reg [15:0] REG_DBG0;
  reg [15:0] REG_DBG1;
  reg [15:0] REG_DBG2;
  reg	     REG_DBG_CP;
  parameter  ADRS_DBG0   = 18'o777100;
  parameter  ADRS_DBG1   = 18'o777102;
  parameter  ADRS_DBG2   = 18'o777104;
  always @(posedge sys_clk or negedge INIT_n)
    if( ~INIT_n ) begin // set dummy addresses
       REG_DBG0 <= 16'o177775;
       REG_DBG1 <= 16'o177775;
       REG_DBG2 <= 16'o177775;
    end
    else if(negedge_SCTL_n & bus_write)
      case (address)
	ADRS_DBG0: REG_DBG0 <= DAL;
	ADRS_DBG1: REG_DBG1 <= DAL;
	ADRS_DBG2: REG_DBG2 <= DAL;
	default:;
      endcase
  
  always @(posedge sys_clk or negedge RESET_n)
    if( ~RESET_n )
      {REG_DBG_CP, dbg_trg} <= 0;
    else if( address == ADRS_XCSR) // negate HALT when console ODT starts
      dbg_trg <= 0; 
    else if( (address[15:0] == REG_DBG0) & aio_iread )
      dbg_trg <= 1'b1; 
    else if( (address[15:0] == REG_DBG1) & aio_iread)
      REG_DBG_CP <= 1'b1;
    else if( (address[15:0] == REG_DBG2) & aio_iread  & REG_DBG_CP)
      dbg_trg <= 1'b1;
//    else if( (address == 16'o001040) & aio_iread ) // trap at 'panic:'
//      dbg_trg <= 1'b1;
//  else if( (dpwa == (16'o25246 >> 1)) & (we0_lo | we0_hi |we1_lo | we1_hi))
//    dbg_trg <= 1'b1;
//  else if( (dpwa == (16'o1256 >> 1)) & bus_write)
//  else if ((REG_RF_DAR_BAK == 16'o117400) & disk_write & (DMA_dev == DEV_RF))
//  else if ( REG_RKCS[11] ) // Inhibit incrementing RKBA
//    else if ( (address == REG_MTC) & bus_write & negedge_SCTL_n & 
//	      DAL[3:0] == 4'o11 )
//    else if ( (address == 16'h22124) & bus_write & negedge_SCTL_n)
//      dbg_trg <= 1'b1;
//    else if ( (address == ADRS_MTC) & bus_write & negedge_SCTL_n & 
//	      DAL[3:0] == 4'o11 )
//      dbg_trg <= 1'b1;

  function [40*8-1:0] bitmap2stream(input [39:0] x);
     bitmap2stream[40*8-1:0]
       = { x[ 0] ? 8'h10: 8'h00,
	   x[ 1] ? 8'h10: 8'h00,
	   x[ 2] ? 8'h10: 8'h00,
	   x[ 3] ? 8'h10: 8'h00,
	   x[ 4] ? 8'h10: 8'h00,
	   x[ 5] ? 8'h10: 8'h00,
	   x[ 6] ? 8'h10: 8'h00,
	   x[ 7] ? 8'h10: 8'h00,
	   x[ 8] ? 8'h10: 8'h00,
	   x[ 9] ? 8'h10: 8'h00,
	   x[10] ? 8'h10: 8'h00,
	   x[11] ? 8'h10: 8'h00,
	   x[12] ? 8'h10: 8'h00,
	   x[13] ? 8'h10: 8'h00,
	   x[14] ? 8'h10: 8'h00,
	   x[15] ? 8'h10: 8'h00,
	   x[16] ? 8'h10: 8'h00,
	   x[17] ? 8'h10: 8'h00,
	   x[18] ? 8'h10: 8'h00,
	   x[19] ? 8'h10: 8'h00,
	   x[20] ? 8'h10: 8'h00,
	   x[21] ? 8'h10: 8'h00,
	   x[22] ? 8'h10: 8'h00,
	   x[23] ? 8'h10: 8'h00,
	   x[24] ? 8'h10: 8'h00,
	   x[25] ? 8'h10: 8'h00,
	   x[26] ? 8'h10: 8'h00,
	   x[27] ? 8'h10: 8'h00,
	   x[28] ? 8'h10: 8'h00,
	   x[29] ? 8'h10: 8'h00,
	   x[30] ? 8'h10: 8'h00,
	   x[31] ? 8'h10: 8'h00,
	   x[32] ? 8'h10: 8'h00,
	   x[33] ? 8'h10: 8'h00,
	   x[34] ? 8'h10: 8'h00,
	   x[35] ? 8'h10: 8'h00,
	   x[36] ? 8'h10: 8'h00,
	   x[37] ? 8'h10: 8'h00,
	   x[38] ? 8'h10: 8'h00,
	   x[39] ? 8'h10: 8'h00};
  endfunction
     
  parameter LEDS = 40;
  reg [LEDS-1:0] led_array_r = 0;
  reg [LEDS-1:0] led_array_g = 0;
  reg [LEDS-1:0] led_array_b = 0;

  wire [LEDS*8-1:0] led_stream_r;
  wire [LEDS*8-1:0] led_stream_g;
  wire [LEDS*8-1:0] led_stream_b;

  assign led_stream_r= bitmap2stream(led_array_r);
  assign led_stream_g= bitmap2stream(led_array_g);
  assign led_stream_b= bitmap2stream(led_array_b);
  
  ws2812
    #(.CLK_FRQ(WS2812_CLK_FRQ),
      .LEDS(LEDS)
      ) ws2812_inst
      (
       .clk(ws2812_clk),
       .sout(LED_RGB),
       .r(led_stream_r),
       .g(led_stream_g),
       .b(led_stream_b)
       );
  
  reg [25:0]		cnt_500ms;
  reg			clk_1Hz;
  always @(posedge sys_clk)
    if(cnt_500ms == SYS_CLK_FRQ/2) begin
       cnt_500ms <= 0;
       clk_1Hz <= ~clk_1Hz;
    end else 
      cnt_500ms <= cnt_500ms + 1'b1;

  reg [5:0] event_count;
  reg	    event_monitor;
  always @(posedge sys_clk or negedge RESET_n)
    if( ~RESET_n )
      {event_count, event_monitor} <= 0;
    else if(posedge_EVENT_ACK)
      if(event_count == 6'd30) begin
	 event_count <= 0;
	 event_monitor <= ~event_monitor;
      end
      else
	event_count <= event_count + 1'd1;
  
  assign led_r = rx_data_ready | (~sd_mosi);
  assign led_b = event_monitor | (~tx_ready) | (~sd_miso);

//---------------------------------------------------------------------------
// LED array for debug
//---------------------------------------------------------------------------
  reg [25:0]		cnt_showID;
  reg			flg_showID;
  always @(posedge sys_clk or negedge INIT_n)
    if( ~INIT_n ) begin
       cnt_showID <= 0;
       flg_showID <= 1'b1;
    end
    else if(cnt_showID != (SYS_CLK_FRQ * 2))
      cnt_showID <= cnt_showID + 1'b1;
    else
      flg_showID <= 0;
  
  always @(posedge sys_clk) begin
     led_array_r[39:37] <= {diag_test_failed, HALT,    ~INIT_n};
     led_array_g[39:37] <= {rx_data_ready, ~tx_ready, BUS_RESET};
     led_array_b[39:37] <= {CLK_monitor, ALE_monitor, event_monitor};

     led_array_r[36:34]
       <= {(sd_error !=0)  | ~sd_mosi,
	   (mt_error !=0 ) | ~mt_mosi,
	   (pt_error !=0 ) | ~pt_mosi};
     led_array_g[36:34] <= 0;
     led_array_b[36:34]
       <= {~sd_miso,
	   ~mt_miso & (sd2_dev == DEV_MT),
	   ~pt_miso & (sd2_dev == DEV_PT)};
	   
     {led_array_r[33], led_array_g[33], led_array_b[33]}
       <= {IRQ2, IRQ1, IRQ0};
     
     {led_array_r[32], led_array_g[32], led_array_b[32]} <= sw1_count;

     case ( (flg_showID ? 7 : sw1_count) & 3'd7)
       0: begin
	  led_array_r[31:16]  <= address[15:0];
	  led_array_r[15:0]   <= d_cpu_to_ram;
	  {led_array_g[31:0], led_array_b[31:0]} <= 0;
       end
       1: begin
	  led_array_r[31:24]  <= {sd_det_n, sd2_det_n, 6'b000000};
	  led_array_r[23:16]  <= sd_error;
	  led_array_r[15:8]   <= mt_error;
	  led_array_r[7:0]    <= pt_error;

	  led_array_g[31:0]   <= TM_block_address;

	  led_array_b[31:24]  <= {2'b00, DMA_dev[2:0], sd2_dev[2:0]};
	  led_array_b[23:16]  <= sd_state;
	  led_array_b[15:8]   <= mt_state;
	  led_array_b[7:0]    <= pt_state;
       end
       2: begin
	  led_array_r[31:0]   <= RF_disk_block_address;
	  led_array_g[31:0]   <= RK_disk_block_address;
	  led_array_b[31:0]   <= RP_disk_block_address;
       end
       3: begin
	  led_array_r[31:0]   <= pt_address_write;
	  led_array_g[31:0]   <= 0;
	  led_array_b[31:0]   <= pt_address;
       end
       4: begin
	  led_array_r[31:16]  <= REG_DBG0;
	  led_array_r[15:0]   <= 0;
	  {led_array_g[31:0], led_array_b[31:0]} <= 0;
       end
       5: begin
	  led_array_r[31:16]  <= REG_DBG1;
	  led_array_r[15:0]   <= REG_DBG2;
	  {led_array_g[31:0], led_array_b[31:0]} <= 0;
       end
       6: begin
	  led_array_r[31:16]  <= {SCTL_n, CONT_n, 14'b0};
	  led_array_r[15:0]   
	    <= {DMA, stretch_trig, stretch_start, KE11_stretch,
		RF_go, RK_go, RP_go, TM_go,
		8'b0};
	  {led_array_g[31:0], led_array_b[31:0]} <= 0;
       end
       7: begin
 	  led_array_r[31:0]  <= `VERSION_ID;
 	  {led_array_g[31:0], led_array_b[31:0]} <= 0;
       end
       default:; // can not reach hore
     endcase
  end

// monitor sys_clk and ALE_n
  reg [24:0] CLK_count = 0;
  reg CLK_monitor;
  always @(posedge sys_clk)
    if(CLK_count == SYS_CLK_FRQ/2) begin
       CLK_count <= 0;
       CLK_monitor <= ~CLK_monitor;
    end
    else
      CLK_count <= CLK_count + 1'd1;
  
  reg [24:0] ALE_count = 0;
  reg ALE_monitor;
  always @(posedge ALE_n)
    if(ALE_count == SYS_CLK_FRQ/4) begin
       ALE_count <= 0;
       ALE_monitor <= ~ALE_monitor;
    end
    else
      ALE_count <= ALE_count + 1'd1;
  
  reg [7:0] memaccess;
  always @(posedge sys_clk)
    if(negedge_ALE_n) begin
       memaccess[0] <= (address[17:15] == 3'b000);
       memaccess[1] <= (address[17:15] == 3'b001);
       memaccess[2] <= (address[17:15] == 3'b010);
       memaccess[3] <= (address[17:15] == 3'b011);
       memaccess[4] <= (address[17:15] == 3'b100);
       memaccess[5] <= (address[17:15] == 3'b101);
       memaccess[6] <= (address[17:15] == 3'b110);
       memaccess[7] <= (address[17:15] == 3'b111);
    end

//---------------------------------------------------------------------------
// Hard disk access log to LOG_TX
//---------------------------------------------------------------------------
`ifdef USE_ACCESSLOG
  assign LOG_TX = dbg_tx;
`else
  assign LOG_TX = uart_tx;
`endif
  
`ifdef USE_ACCESSLOG
  parameter	 UART_BPS_DBG    =       115_200; // (for TeraTerm)
// the followings are for oscilloscope
//  parameter	 UART_BPS_DBG    =     1_700_000; // (27_000_000 / 10)
//  parameter	 UART_BPS_DBG    =     2_700_000; // (27_000_000 / 10)
//  parameter	 UART_BPS_DBG    =     6_750_000; // (27_000_000 / 4)
//  parameter	 UART_BPS_DBG    =    13_500_000; // (27_000_000 / 2)

// uart tx module for log
  reg [7:0]	 dbg_tx_data;
  reg		 dbg_tx_send;
  wire		 dbg_tx_ready;
  wire		 dbg_tx;
  uart_tx#
    (
     .CLK_FRQ(SYS_CLK_FRQ),
     .BAUD_RATE(UART_BPS_DBG)
     ) uart_tx_inst_dbg
      (
       .clk           (sys_clk),
       .reset_n       (RESET_n),
       .tx_data       (dbg_tx_data),
       .tx_send       (dbg_tx_send),
       .tx_ready      (dbg_tx_ready),
       .tx_out        (dbg_tx)
       );

//---------------------------------------------------------------------------
// debug_print
// print dbg_regw:dbg_reg0,dbg_reg1,dbg_reg2,dbg_reg3,dbg_reg4
//---------------------------------------------------------------------------
  function [7:0] itoh(input [3:0] x);
     case (x)
       4'h0: itoh="0"; 4'h1: itoh="1"; 4'h2: itoh="2"; 4'h3: itoh="3";
       4'h4: itoh="4"; 4'h5: itoh="5"; 4'h6: itoh="6"; 4'h7: itoh="7";
       4'h8: itoh="8"; 4'h9: itoh="9"; 4'ha: itoh="a"; 4'hb: itoh="b";
       4'hc: itoh="c"; 4'hd: itoh="d"; 4'he: itoh="e"; 4'hf: itoh="f";
     endcase
  endfunction
  function [7:0] itoh0(input [15:0] x); itoh0 = itoh(x[3:0]);   endfunction
  function [7:0] itoh1(input [15:0] x); itoh1 = itoh(x[7:4]);   endfunction
  function [7:0] itoh2(input [15:0] x); itoh2 = itoh(x[11:8]);  endfunction
  function [7:0] itoh3(input [15:0] x); itoh3 = itoh(x[15:12]); endfunction

  function [7:0] itoo(input [2:0] x);
     case (x)
       4'h0: itoo="0"; 4'h1: itoo="1"; 4'h2: itoo="2"; 4'h3: itoo="3";
       4'h4: itoo="4"; 4'h5: itoo="5"; 4'h6: itoo="6"; 4'h7: itoo="7";
     endcase
  endfunction
  function [7:0] itoo0(input [17:0] x); itoo0 = itoo(x[2:0]);  endfunction
  function [7:0] itoo1(input [17:0] x); itoo1 = itoo(x[5:3]);  endfunction
  function [7:0] itoo2(input [17:0] x); itoo2 = itoo(x[8:6]);  endfunction
  function [7:0] itoo3(input [17:0] x); itoo3 = itoo(x[11:9]); endfunction
  function [7:0] itoo4(input [17:0] x); itoo4 = itoo(x[14:12]);endfunction
  function [7:0] itoo5(input [17:0] x); itoo5 = itoo(x[17:15]);endfunction

  reg [15:0] dbg_regt;
  reg [15:0] dbg_regw;
  reg [17:0] dbg_reg0;
  reg [17:0] dbg_reg1;
  reg [17:0] dbg_reg2;
  reg [17:0] dbg_reg3;
  reg [17:0] dbg_reg4;
  reg [7:0]  dbg_pstate;
  reg [7:0]  dbg_pbuf[255:0];
  reg [7:0]  dbg_pcnt;
  parameter  DBG_PSTATE_IDLE  = 8'd255;
  parameter  DBG_PSTATE_PRINT = 8'd254;
  parameter  DBG_PSTATE_WAIT  = 8'd253;
  parameter  DBG_PSTATE_CLEAR = 8'd252;
  reg	     dbg_print;
  reg	     dbg_clear; // for hand shake
  always @(posedge sys_clk or negedge RESET_n)
    if( ~RESET_n ) begin
       dbg_pstate <= DBG_PSTATE_IDLE;
       dbg_clear <= 0;
    end
    else
      case (dbg_pstate)
	DBG_PSTATE_IDLE:
	  if( dbg_print ) begin
	     dbg_pstate <= 8'd0;
	     dbg_pcnt   <= 8'd0;
	  end
	  else
	    dbg_pstate <= DBG_PSTATE_IDLE;
	8'd0 : {dbg_pbuf[0 ], dbg_pstate} <= {itoh3(dbg_regt), 8'd1 };
	8'd1 : {dbg_pbuf[1 ], dbg_pstate} <= {itoh2(dbg_regt), 8'd2 };
	8'd2 : {dbg_pbuf[2 ], dbg_pstate} <= {itoh1(dbg_regt), 8'd3 };
	8'd3 : {dbg_pbuf[3 ], dbg_pstate} <= {itoh0(dbg_regt), 8'd4 };
	8'd4 : {dbg_pbuf[4 ], dbg_pstate} <= {" ",             8'd5 };
	8'd5 : {dbg_pbuf[5 ], dbg_pstate} <= {dbg_regw[15:8],  8'd6 };
	8'd6 : {dbg_pbuf[6 ], dbg_pstate} <= {dbg_regw[7:0],   8'd7 };
	8'd7 : {dbg_pbuf[7 ], dbg_pstate} <= {",",             8'd8 };
	8'd8 : {dbg_pbuf[8 ], dbg_pstate} <= {itoo5(dbg_reg0), 8'd9 };
	8'd9 : {dbg_pbuf[9 ], dbg_pstate} <= {itoo4(dbg_reg0), 8'd10};
	8'd10: {dbg_pbuf[10], dbg_pstate} <= {itoo3(dbg_reg0), 8'd11};
	8'd11: {dbg_pbuf[11], dbg_pstate} <= {itoo2(dbg_reg0), 8'd12};
	8'd12: {dbg_pbuf[12], dbg_pstate} <= {itoo1(dbg_reg0), 8'd13};
	8'd13: {dbg_pbuf[13], dbg_pstate} <= {itoo0(dbg_reg0), 8'd14};
	8'd14: {dbg_pbuf[14], dbg_pstate} <= {",",             8'd15};
	8'd15: {dbg_pbuf[15], dbg_pstate} <= {itoo5(dbg_reg1), 8'd16};
	8'd16: {dbg_pbuf[16], dbg_pstate} <= {itoo4(dbg_reg1), 8'd17};
	8'd17: {dbg_pbuf[17], dbg_pstate} <= {itoo3(dbg_reg1), 8'd18};
	8'd18: {dbg_pbuf[18], dbg_pstate} <= {itoo2(dbg_reg1), 8'd19};
	8'd19: {dbg_pbuf[19], dbg_pstate} <= {itoo1(dbg_reg1), 8'd20};
	8'd20: {dbg_pbuf[20], dbg_pstate} <= {itoo0(dbg_reg1), 8'd21};
	8'd21: {dbg_pbuf[21], dbg_pstate} <= {",",             8'd22};
	8'd22: {dbg_pbuf[22], dbg_pstate} <= {itoo5(dbg_reg2), 8'd23};
	8'd23: {dbg_pbuf[23], dbg_pstate} <= {itoo4(dbg_reg2), 8'd24};
	8'd24: {dbg_pbuf[24], dbg_pstate} <= {itoo3(dbg_reg2), 8'd25};
	8'd25: {dbg_pbuf[25], dbg_pstate} <= {itoo2(dbg_reg2), 8'd26};
	8'd26: {dbg_pbuf[26], dbg_pstate} <= {itoo1(dbg_reg2), 8'd27};
	8'd27: {dbg_pbuf[27], dbg_pstate} <= {itoo0(dbg_reg2), 8'd28};
	8'd28: {dbg_pbuf[28], dbg_pstate} <= {",",             8'd29};
	8'd29: {dbg_pbuf[29], dbg_pstate} <= {itoo5(dbg_reg3), 8'd30};
	8'd30: {dbg_pbuf[30], dbg_pstate} <= {itoo4(dbg_reg3), 8'd31};
	8'd31: {dbg_pbuf[31], dbg_pstate} <= {itoo3(dbg_reg3), 8'd32};
	8'd32: {dbg_pbuf[32], dbg_pstate} <= {itoo2(dbg_reg3), 8'd33};
	8'd33: {dbg_pbuf[33], dbg_pstate} <= {itoo1(dbg_reg3), 8'd34};
	8'd34: {dbg_pbuf[34], dbg_pstate} <= {itoo0(dbg_reg3), 8'd35};
	8'd35: {dbg_pbuf[35], dbg_pstate} <= {",",             8'd36};
	8'd36: {dbg_pbuf[36], dbg_pstate} <= {itoo5(dbg_reg4), 8'd37};
	8'd37: {dbg_pbuf[37], dbg_pstate} <= {itoo4(dbg_reg4), 8'd38};
	8'd38: {dbg_pbuf[38], dbg_pstate} <= {itoo3(dbg_reg4), 8'd39};
	8'd39: {dbg_pbuf[39], dbg_pstate} <= {itoo2(dbg_reg4), 8'd40};
	8'd40: {dbg_pbuf[40], dbg_pstate} <= {itoo1(dbg_reg4), 8'd41};
	8'd41: {dbg_pbuf[41], dbg_pstate} <= {itoo0(dbg_reg4), 8'd42};
	8'd42: {dbg_pbuf[42], dbg_pstate} <= {8'h0d,           8'd43}; // \r
	8'd43: {dbg_pbuf[43], dbg_pstate} <= {8'h0a,           8'd44}; // \n
	8'd44: {dbg_pbuf[44], dbg_pstate} <= {8'b0, DBG_PSTATE_CLEAR};
	DBG_PSTATE_CLEAR:
	  if( dbg_print )
	    dbg_clear <= 1'b1;
	  else if( ~dbg_print ) begin
	     dbg_clear <= 0;
	     dbg_pstate <= DBG_PSTATE_PRINT;
	  end
	DBG_PSTATE_PRINT:
	  if( dbg_pbuf[dbg_pcnt] == 8'b0)
	    dbg_pstate <= DBG_PSTATE_IDLE;
	  else if( dbg_tx_ready ) begin
	     dbg_tx_data <= dbg_pbuf[dbg_pcnt];
	     dbg_tx_send <= 1;
	     dbg_pcnt <= dbg_pcnt + 1'd1;
	     dbg_pstate <= DBG_PSTATE_WAIT;
	  end
	DBG_PSTATE_WAIT:
	  if( ~dbg_tx_ready ) begin
	     dbg_tx_send <= 0;
	     dbg_pstate <= DBG_PSTATE_PRINT;
	  end

	// dummy to avoid warning
	default: dbg_pbuf[dbg_pstate] <= 0;
      endcase
  
  reg [11:0] cnt_100us;
  reg [15:0] dbg_time;
  always @(posedge sys_clk or negedge RESET_n)
    if( ~RESET_n )
      {cnt_100us, dbg_time} <= 0;
    else if(cnt_100us == (SYS_CLK_FRQ / 1000 / 1000)*100 -1) begin // 100us
       cnt_100us <= 0;
       dbg_time <= dbg_time + 1'd1;
    end
    else
      cnt_100us <= cnt_100us + 1'd1;

  wire [15:0] REG_RF_DCS_R ={8'b0, RF_READY, REG_RF_DCS[6:1], 1'b0};
  wire [15:0] REG_RKCS_R   ={2'b00, REG_RKCS[13:8], RK_READY, REG_RKCS[6:0]};
  wire [15:0] REG_RKDS_R   ={8'b000_01001, RK_READY, RK_READY,
			     2'b01, REG_RKDA[3:0]};
  
  wire [15:0] dbg_disk_address       = disk_block_address[15:0];
  wire [15:0] dbg_disk_address_l     = disk_block_address[14:0];
  wire [15:0] dbg_disk_address_h     = {7'b0, disk_block_address[23:15]};

  wire [15:0] sd2_dbg_disk_address_l = sd2_disk_block_address[14:0];
  wire [15:0] sd2_dbg_disk_address_h = {TM_FILENUM[3:0], 3'b0,
					sd2_disk_block_address[23:15]};

  reg [15:0] dbg_ppti_cnt  = 0;
  parameter  DBG_PPTI_SKIP = 255;

  always @(posedge sys_clk)
    dbg_regt <= dbg_time;

  wire disk_access = disk_read | disk_write | disk_nop;
  wire mt_access   = mt_read   | mt_write   | mt_nop;
  always @(posedge sys_clk)
    if ( dbg_clear )
      dbg_print <= 0;
    else if (dbg_print == 1'b0)
      if ((address == 18'o773000) & aio_iread ) begin
	 dbg_regw <= "bt";
	 dbg_reg0 <= address;
	 dbg_reg1 <= 0;
	 dbg_reg2 <= 0;
	 dbg_reg3 <= 0;
	 dbg_reg4 <= 0;
	 dbg_print<= 1'b1;
      end
      else if ((DAL == VA_RF) & vec_read ) begin
	 dbg_regw <= "Fi";
	 dbg_reg0 <= REG_RF_DCS_R;
	 dbg_reg1 <= REG_TRACE[3];
	 dbg_reg2 <= REG_TRACE[2];
	 dbg_reg3 <= REG_TRACE[1];
	 dbg_reg4 <= REG_TRACE[0];
	 dbg_print<= 1'b1;
      end
      else if ((DAL == VA_RK) & vec_read ) begin
	 dbg_regw <= "Ki";
	 dbg_reg0 <= REG_RKCS_R;
	 dbg_reg1 <= REG_TRACE[3];
	 dbg_reg2 <= REG_TRACE[2];
	 dbg_reg3 <= REG_TRACE[1];
	 dbg_reg4 <= REG_TRACE[0];
	 dbg_print<= 1'b1;
      end
      else if ( bus_error ) begin
	 dbg_regw <= "be"; // bus error
//      else if (~ABORT_n & (address != 0) & aio_iread) begin
//	 dbg_regw <= "ab"; // Abort
//      else if ((address == 16'o000320) & aio_iread ) begin
//	 dbg_regw <= "tr"; // trap (UNIX V6)
	 dbg_reg0 <= address;
	 dbg_reg1 <= REG_TRACE[3];
	 dbg_reg2 <= REG_TRACE[2];
	 dbg_reg3 <= REG_TRACE[1];
	 dbg_reg4 <= REG_TRACE[0];
	 dbg_print<= 1'b1;
      end
      else if ( BUS_RESET ) begin
	 dbg_regw <= "re"; // bus error
	 dbg_reg0 <= address;
	 dbg_reg1 <= REG_TRACE[3];
	 dbg_reg2 <= REG_TRACE[2];
	 dbg_reg3 <= REG_TRACE[1];
	 dbg_reg4 <= REG_TRACE[0];
	 dbg_print<= 1'b1;
      end
      else if ((DMA_dev == DEV_RF) & disk_access) begin
	 if(disk_read)
	   dbg_regw <= "FR";
	 else if(disk_write)
	   dbg_regw <= "FW";
	 else
	   dbg_regw <= "FN";
	 dbg_reg0 <= REG_RF_DCS;
	 dbg_reg1 <= REG_RF_WC_BAK;
	 dbg_reg2 <= {REG_RF_DCS_BAK[5:4], REG_RF_CMA_BAK};
	 dbg_reg3 <= REG_RF_DAR_BAK;
	 dbg_reg4 <= dbg_disk_address;
	 dbg_print<= 1'b1;
      end
      else if ((DMA_dev == DEV_RK) & disk_access) begin
	 if(disk_read)
	   dbg_regw <= "KR";
	 else if(disk_write)
	   dbg_regw <= "KW";
	 else
	   dbg_regw <= "KN";
	 dbg_reg0 <= REG_RKCS;
	 dbg_reg1 <= REG_RKWC_BAK;
	 dbg_reg2 <= {REG_RKCS_BAK[5:4], REG_RKBA_BAK};
	 dbg_reg3 <= REG_RKDA;
	 dbg_reg4 <= dbg_disk_address;
	 dbg_print<= 1'b1;
      end
      else if ((DMA_dev == DEV_RP) & disk_access) begin
	 if(disk_read)
	   dbg_regw <= "PR";
	 else if(disk_write)
	   dbg_regw <= "PW";
	 else 
	   dbg_regw <= "PN";
	 dbg_reg0 <= REG_RPCS;
	 dbg_reg1 <= REG_RPWC_BAK;
	 dbg_reg2 <= {REG_RPCS_BAK[5:4], REG_RPBA_BAK};
	 dbg_reg3 <= dbg_disk_address_h;
	 dbg_reg4 <= dbg_disk_address_l;
	 dbg_print<= 1'b1;
      end
      else if (DMA_dev == DEV_MT & mt_access) begin
	 if(mt_read)
	   dbg_regw <= "MR";
	 else if(mt_write)
	   dbg_regw <= "MW";
	 else
	   dbg_regw <= "MN";
	 dbg_reg0 <= REG_MTC;
	 dbg_reg1 <= REG_MTBRC_BAK;
	 dbg_reg2 <= {REG_MTC_BAK[5:4], REG_MTCMA_BAK};
	 dbg_reg3 <= sd2_dbg_disk_address_h;
	 dbg_reg4 <= sd2_dbg_disk_address_l;
	 dbg_print<= 1'b1;
      end
//      else if (negedge_SCTL_n & (address == ADRS_MTBRC) & aio_write) begin
//	 dbg_regw <= "bc";
//	 dbg_reg0 <= address;
//	 dbg_reg1 <= DAL;
//	 dbg_reg2 <= 0;
//	 dbg_reg3 <= 0;
//	 dbg_reg4 <= 0;
//	 dbg_print<= 1'b1;
//      end
      else if((DAL == VA_ttyi) & vec_read) begin
	 dbg_regw <= "ti";
	 dbg_reg0 <= REG_TRACE[4];
	 dbg_reg1 <= REG_TRACE[3];
	 dbg_reg2 <= REG_TRACE[2];
	 dbg_reg3 <= REG_TRACE[1];
	 dbg_reg4 <= REG_TRACE[0];
	 dbg_print<= 1'b1;
      end
//      else if((DAL == VA_ttyo) & vec_read) begin
//	 dbg_regw <= "to";
//	 dbg_reg0 <= REG_TRACE[4];
//	 dbg_reg1 <= REG_TRACE[3];
//	 dbg_reg2 <= REG_TRACE[2];
//	 dbg_reg3 <= REG_TRACE[1];
//	 dbg_reg4 <= REG_TRACE[0];
//	 dbg_print<= 1'b1;
//      end
      else if((DAL == VA_ppti) & vec_read) begin
	 dbg_regw <= "pi";
	 dbg_reg0 <= REG_TRACE[4];
	 dbg_reg1 <= REG_TRACE[3];
	 dbg_reg2 <= REG_TRACE[2];
	 dbg_reg3 <= REG_TRACE[1];
	 dbg_reg4 <= REG_TRACE[0];
	 dbg_print<= (dbg_ppti_cnt == 0);
	 if(dbg_ppti_cnt == DBG_PPTI_SKIP)
	   dbg_ppti_cnt <= 0;
	 else 
	   dbg_ppti_cnt <= dbg_ppti_cnt + 1'd1;
      end
`endif // USE_ACCESSLOG
  
endmodule
