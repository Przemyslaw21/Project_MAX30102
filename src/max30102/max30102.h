#ifndef MAX30102_H_
#define MAX30102_H_

#define SYS_TIME_SLEEP_MS 10

#define MAX30102_MEASUREMENT_SECONDS 5
#define MAX30102_SAMPLES_PER_SECOND	3200 // 50, 100, 200, 400, 800, 100, 1600, 3200 sample rating
#define MAX30102_FIFO_ALMOST_FULL_SAMPLES 17 // used for IRQ

#define OMIT_FIFO 1  // enable = 1   disable = 0 
#define RESET_DURING_INIT 0  // enable = 1   disable = 0 
// #define MAX30102_BUFFER_LENGTH	((MAX30102_MEASUREMENT_SECONDS+1)*MAX30102_SAMPLES_PER_SECOND)


#define MAX30102_ADDRESS                   0x57 // Address MAX30102
#define MAX30102_IRQ_STATUS_1              0x00
#define MAX30102_IRQ_STATUS_2              0x01
#define MAX30102_IRQ_ENABLE_1              0x02
#define MAX30102_IRQ_ENABLE_2              0x03
#define MAX30102_FIFO_WRITE_PTR            0x04
#define MAX30102_OVERFLOW_COUNT            0x05
#define MAX30102_FIFO_READ_PTR             0x06
#define MAX30102_REG_FIFO_DATA             0x07
#define MAX30102_FIFO_CONFIG               0x08
#define MAX30102_MODE_CONFIG               0x09
#define MAX30102_SPO2_CONFIG               0x0A
#define MAX30102_LED1_PULSE_AMPLITUDE      0x0C
#define MAX30102_LED2_PULSE_AMPLITUDE      0x0D
#define MAX30102_PARTID                    0xFF//0x15
#define MAX30102_RESET                     0x40


#define FIFO_CONF_SMP_AVE_BIT 			    7 // FIFO Configuration (0x08)   
#define FIFO_CONF_SMP_AVE_LENGHT 		    3
#define FIFO_CONF_FIFO_ROLLOVER_EN_BIT 	    4
#define FIFO_CONF_FIFO_A_FULL_BIT 		    3
#define FIFO_CONF_FIFO_A_FULL_LENGHT 	    4

#define FIFO_SMP_AVE_1	                    0
#define FIFO_SMP_AVE_2	                    1
#define FIFO_SMP_AVE_4	                    2
#define FIFO_SMP_AVE_8	                    3
#define FIFO_SMP_AVE_16	                    4
#define FIFO_SMP_AVE_32	                    5


                                              // Mode Config (0x09)
#define MODE_HEART_RATE						2 // red only	
#define MODE_SpO2		                	3 // red & IR
#define MODE_Multi_LED		 				7 // red & IR


#define SPO2_CONF_LED_PW_BIT		        1 //SpO2 Config (0x0A)
#define SPO2_CONF_LED_PW_LENGTH		        2
#define SPO2_CONF_SR_LENGTH			        3
#define SPO2_CONF_SR_BIT			        4
#define SPO2_CONF_ADC_RGE_LENGTH	        2
#define SPO2_CONF_ADC_RGE_BIT		        6


#define	SPO2_ADC_RGE_2048	                0
#define	SPO2_ADC_RGE_4096	                1
#define	SPO2_ADC_RGE_8192	                2
#define	SPO2_ADC_RGE_16384	                3

#define MODE_HEART_RATE	                    2
#define MODE_SPO2		                    3
#define MODE_MULTI  	                    7

#define	PULSE_WIDTH_69			            0
#define	PULSE_WIDTH_118		                1
#define	PULSE_WIDTH_215		                2
#define	PULSE_WIDTH_411		                3

#define IR_LED_CURRENT_LOW		            0x07
#define RED_LED_CURRENT_LOW	                0x07
#define IR_LED_CURRENT_HIGH	                0xAF
#define RED_LED_CURRENT_HIGH	            0xAF

#define TEMP_INTEGER                        0x1F
#define TEMP_FRACTION                       0x20 //TFRAC[3:0]
#define TEMP_ENABLE                         0x21
#define TEMP_FRACTION_STEP                  0.0625

// //
// //	Status enum
// //
// typedef enum MAX30102_STATUS{
// 	MAX30102_OK 	= 0,
// 	MAX30102_ERROR 	= 1
// } MAX30102_STATUS;

#endif /* MAX30102_H_ */