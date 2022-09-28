#define SFE_KX13X_MAN_ID 0x00//      Retuns "KION" in ASCII
typedef struct 
{
	uint8_t man_id : 8; 
} sfe_kx13x_man_id_t

#define SFE_KX13X_PART_ID 0x01//      Retuns "KION" in ASCII
typedef struct 
{
	uint8_t part_id : 8; 
} sfe_kx13x_part_id_t; 

#define SFE_KX13X_XADP_L 0x02
typedef struct
{
	uint8_t xadp_l : 8;
} sfe_kx13x_xadp_l_t;

#define SFE_KX13X_XADP_H 0x03
typedef struct
{
	uint8_t xadp_h : 8;
} sfe_kx13x_xadp_h_t;

#define SFE_KX13X_YADP_L 0x04
typedef struct
{
	uint8_t yadp_l : 8;
} sfe_kx13x_yadp_l_t;

#define SFE_KX13X_YADP_H 0x05
typedef struct
{
	uint8_t yadp_l : 8;
} sfe_kx13x_yadp_h_t;

#define SFE_KX13X_ZADP_L 0x06
typedef struct
{
	uint8_t yadp_l : 8;
} sfe_kx13x_zadp_l_t;

#define SFE_KX13X_ZADP_H 0x07
typedef struct
{
	uint8_t zadp_h : 8;
} sfe_kx13x_zadp_h_t;

#define SFE_KX13X_XOUT_L 0x08
typedef struct
{
	uint8_t xout_l : 8;
} sfe_kx13x_xout_l_t;

#define SFE_KX13X_XOUT_H 0x09
typedef struct
{
	uint8_t xout_h : 8;
} sfe_kx13x_xout_h_t;

#define SFE_KX13X_YOUT_L 0x0A
typedef struct
{
	uint8_t yout_l : 8;
} sfe_kx13x_yout_l_t;

#define SFE_KX13X_YOUT_H 0x0B
typedef struct
{
	uint8_t yout_h : 8;
} sfe_kx13x_yout_h_t;

#define SFE_KX13X_ZOUT_L 0x0C
typedef struct
{
	uint8_t zout_l : 8;
} sfe_kx13x_zout_l_t;

#define SFE_KX13X_ZOUT_H 0x0D   --------------^^--------------------------
typedef struct
{
	uint8_t zout_h : 8;
} sfe_kx13x_zout_h_t;

#define SFE_KX13X_COTR 0x12 
// Command Test Response 
// Verifies the proper integrated circuit intergrity
typedef struct
{
	uint8_t dc_str : 8; //Default 0b01010101 : 0x55
} sfe_kx13x_cotr_t;

#define SFE_KX13X_WHO_AM_I 0x13 
//Identifies the accelerometer being used: 0x3D - KX132 and 0x46 - KX135
typedef struct
{
	uint8_t wai : 8; 
} sfe_kx13x_wai_t;

#define SFE_KXI3X_TSCP 0x14
//Current Tilt Position Register reports current position data
typedef struct
{
	uint8_t reserved_one       :  1;
	uint8_t reserved_two       :  1;
	uint8_t left_state         :  1;  //X-
	uint8_t right_state        :  1;  //X+
	uint8_t down_state         :  1;  //Y-
	uint8_t up_state           :  1;  //Y+
	uint8_t face_down_state    :  1;	//Z-
	uint8_t face_up_down_state :  1;  //Z+
} sfe_kx13x_tscp_t;

#define SFE_KX13X_TSPP 0x15
//Previous Tilt Position Register reports previous position data
typedef struct
{
	uint8_t reserved_one       :  1;
	uint8_t reserved_two       :  1;
	uint8_t left_state         :  1;  //X-
	uint8_t right_state        :  1;  //X+
	uint8_t down_state         :  1;  //Y-
	uint8_t up_state           :  1;  //Y+
	uint8_t face_down_state    :  1;	//Z-
	uint8_t face_up_down_state :  1;  //Z+
} sfe_kx13x_tspp_t;

#define SFE_KX13X_INS1 0x16 
// Reports Tap and Double Tap interrupts according to the bit. Bit
// is cleared when the interrupt latch release register (INT_REL) is read. 
typedef struct
{
	uint8_t reserved_one  :  1;
	uint8_t reserved_two  :  1;
	uint8_t tap_left      :  1;  //X-
	uint8_t tap_right     :  1;  //X+
	uint8_t tap_down      :  1;  //Y-
	uint8_t tap_up        :  1;  //Y+
	uint8_t tap_face_down :  1;	 //Z-
	uint8_t tap_face_up   :  1;  //Z+
} sfe_kx13x_ins1_t;

#define SFE_KX13X_INS2 0x17
// Reports which function caused an interrupt
typedef struct
{
	uint8_t ffs      :  1; // Free fall
	uint8_t bfi      :  1; // Buffer full
	uint8_t wmi      :  1; // Watermark
	uint8_t drdy     :  1; // Data Ready 
	uint8_t tdts     :  2; // Tap/Double Tap  
	uint8_t reserved :  1;  
	uint8_t tps      :  1; // Tilt Position  
} sfe_kx13x_ins2_t;

#define SFE_KX13X_INS3 0x18
// Reports which axis and direction of detected motion triggered the wakeup interrupt
typedef struct
{
	uint8_t wufs :  1;
	uint8_t bts  :  1;
	uint8_t xnwu :  1;  //X-
	uint8_t xpwu :  1;  //X+
	uint8_t ynwu :  1;  //Y-
	uint8_t ypwu :  1;  //Y+
	uint8_t znwu :  1;	//Z-
	uint8_t zpwu :  1;  //Z+
} sfe_kx13x_ins3_t;

#define SFE_KX13X_STATUS_REG 0x19 
// Reports the combined status of the interrupts and the wake/back to sleep state
typedef struct
{
	uint8_t reserved_one :  3;
	uint8_t combined_int :  1;
	uint8_t reserved_two :  3;
	uint8_t wake         :  1;
} sfe_kx13x_status_reg_t;

#define SFE_KX13X_INT_REL 0x1A 
// Interrupt latch release: when an interrupt is flipped read this status to clear
// interrupt. 
typedef struct
{
	uint8_t latch_release : 8;
} sfe_kx13x_int_rel_t;

#define SFE_KX13X_CNTL1 0x1B
// Read/write control register that controls the "main feature" set.
// To change these values PC1 (bit 8) must be set to zero (stand-by mode).
typedef struct
{
	uint8_t pc1          :  1;
	uint8_t res          :  1;
	uint8_t drdye        :  1;
	uint8_t gsel         :  2;
	uint8_t tdte         :  1;
	uint8_t reserved_one :  1;
	uint8_t tpe          :  1;
} sfe_kx13x_cntl1_t;

#define SFE_KX13X_CNTL2 0x1C
// Read/Write control register that controls tilt position state enabling
// Default value: 0b00111111
typedef struct
{
	uint8_t srst :  1; //Software reset
	uint8_t cotc :  1; //Command Test Control bit
										 // The following bits control when an interrupt is generated
										 // for tilt: 1 = enabled
	uint8_t lem  :  1; // Left state (X-)
	uint8_t rim  :  2; // Right state (X+)
	uint8_t dom  :  1; // Down state (Y-)
	uint8_t upm  :  1; // Up state (Y+)
	uint8_t fdm  :  1; // Face-Down state (Z-)
	uint8_t fum  :  1; // Face-Up state (Z+)
} sfe_kx13x_cntl2_t;

#define SFE_KX13X_CNTL3 0x1D
// Read/Write control register that provides control of the Output Data Rate (ODR)
// for tilt, tap, wake-up registers.
typedef struct
{
	uint8_t opt  :  2; // ODR tilt position
	uint8_t otdt :  3; // ODR tap/double-tap
	uint8_t owuf :  3; // ORR wake-up
} sfe_kx13x_cntl3_t;

#define SFE_KX13X_CNTL4 0x1E
// Read/write control register that provides more feature set control.
// To changes these settings make sure IC is in "stand-by" mode: PC1 bit in CNTL1.
typedef struct
{
	uint8_t c_mode  :  1; 
	uint8_t th_mode :  1; 
	uint8_t wufe    :  1; 
	uint8_t btse    :  1;
	uint8_t pr_mode :  1;
	uint8_t obts    :  3; 
} sfe_kx13x_cntl4_t;

#define SFE_KX13X_CNTL5 0x1F
// Read/Write control register that providers more feature set control. 
// These settings can be changed on the fly - no need to put IC in stand-by.
typedef struct
{
	uint8_t reserved_one :  3;
	uint8_t adpe         :  1;
	uint8_t reserved_two :  2;
	uint8_t man_wake     :  1;
	uint8_t man_sleep    :  1;
} sfe_kx13x_cntl5_t;

#define SFE_KX13X_CNTL6 0x20
// Read/Write control register that providers more feature set control. 
// These settings can be changed on the fly - no need to put IC in stand-by.
typedef struct
{
	uint8_t i2c_ale :  1; 
	uint8_t reserved_one :  5;
	uint8_t i2c_alc     :  2;
} sfe_kx13x_cntl6_t;

#define SFE_KX13X_ODCNTL 0x21
// Control registers that contorls acceleration outputs. 
// To changes these settings make sure IC is in "stand-by" mode: PC1 bit in CNTL1.
typedef struct
{
	uint8_t iir_bypass   :  1;
	uint8_t lpro         :  1;
	uint8_t fstup        :  1;
	uint8_t reserved_one :  1;
	uint8_t osa          :  4;
} sfe_kx13x_odcntl_t;

#define SFE_KX13X_INC1 0x22//Controls settings for INT1
#define SFE_KX13X_INC2 0x23//Defines behavior for Wake-Up Function and Back To Sleep
#define SFE_KX13X_INC3 0x24//Defines which axes can cause a tap based interrupt
#define SFE_KX13X_INC4 0x25 //Controls which function triggers INT1
#define SFE_KX13X_INC5 0x26
#define SFE_KX13X_INC6 0x27//Controls which function triggers INT2
// 0x28 Reserved
#define SFE_KX13X_TILT_TIMER 0x29 
#define SFE_KX13X_TDTRC 0x2A // Tap Control Regs ----- 
#define SFE_KX13X_TDTC 0x2B
#define SFE_KX13X_TTH 0x2C
#define SFE_KX13X_TTL 0x2D
#define SFE_KX13X_FTD 0x2E
#define SFE_KX13X_STD 0x2F
#define SFE_KX13X_TLT 0x30
#define SFE_KX13X_TWS 0x31
#define SFE_KX13X_FFTH 0x32
#define SFE_KX13X_FFC 0x33
#define SFE_KX13X_FFCNTL 0x34
// 0x35 - 0x36 Reserved
#define SFE_KX13X_TILT_ANGLE_LL 0x37
#define SFE_KX13X_TILT_ANGLE_HL 0x38
#define SFE_KX13X_HYST_SET 0x39
#define SFE_KX13X_LP_CNTL1 0x3A
#define SFE_KX13X_LP_CNTL2 0x3B
// 0x3C - 0x48 Reserved
#define SFE_KX13X_WUFTH 0x49
#define SFE_KX13X_BTSWUFTH 0x4A
#define SFE_KX13X_BTSTH 0x4B
#define SFE_KX13X_BTSC 0x4C
#define SFE_KX13X_WUFC 0x4D
// 0x4E - 0x5C Reserved
#define SFE_KX13X_SELF_TEST 0x5D
#define SFE_KX13X_BUF_CNTL1 0x5E
#define SFE_KX13X_BUF_CNTL2 0x5F
#define SFE_KX13X_BUF_STATUS_1 0x60
#define SFE_KX13X_BUF_STATUS_2 0x61
#define SFE_KX13X_BUF_CLEAR 0x62
#define SFE_KX13X_BUF_READ 0x63
#define SFE_KX13X_ADP_CNTL1 0x64
#define SFE_KX13X_ADP_CNTL2 0x65
#define SFE_KX13X_ADP_CNTL3 0x66
#define SFE_KX13X_ADP_CNTL4 0x67
#define SFE_KX13X_ADP_CNTL5 0x68
#define SFE_KX13X_ADP_CNTL6 0x69
#define SFE_KX13X_ADP_CNTL7 0x6A
#define SFE_KX13X_ADP_CNTL8 0x6B
#define SFE_KX13X_ADP_CNTL9 0x6C
#define SFE_KX13X_ADP_CNTL10 0x6D
#define SFE_KX13X_ADP_CNTL11 0x6E
#define SFE_KX13X_ADP_CNTL12 0x6F
#define SFE_KX13X_ADP_CNTL13 0x70
#define SFE_KX13X_ADP_CNTL14 0x71
#define SFE_KX13X_ADP_CNTL15 0x72
#define SFE_KX13X_ADP_CNTL16 0x73
#define SFE_KX13X_ADP_CNTL17 0x74
#define SFE_KX13X_ADP_CNTL18 0x75
#define SFE_KX13X_ADP_CNTL19 0x76
//Reserved 0x77 - 0x7F

typedef struct
{
  uint8_t SFE_KX13X_SUCCESS  = 0x00,
  uint8_t SFE_KX13X_GENERAL_ERROR = 0x01,
  uint8_t SFE_KX13X_I2C_ERROR = 0x02
} sfe_error_code_t;

#define  SFE_HI_TILT_POSITION 0x01
#define  SFE_HI_WAKE_UP  0x02 
#define  SFE_HI_TAP_DOUBLE_TAP 0x04
#define  SFE_HI_BACK_TO_SLEEP 0x08
#define  SFE_HI_DATA_READY 0x10
#define  SFE_HI_WATERMARK 0x20
#define  SFE_HI_BUFFER_FULL 0x40
#define  SFE_HI_FREEFALL 0x80

