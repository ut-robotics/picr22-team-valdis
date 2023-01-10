# picr22-team-valdis

Pinout for STM32G441KBT6

PA0 - M1_PWM_A

PA1 - M2_PWM_A

PA2 - M3_PWM_A

PA3 - TRW_PWM

PA5 - M1_DIR_A

PA6 - M2_DIR_A

PA7 - M3_DIR_A

PA8 - M1_ENC_A

PA9 - M1_ENC_B

PA11 - USB_N

PA12 - USB_P

PA13 - SWDIO

PA14 - SWCLK

PF0-OSC_IN - LED

PG10-NRST - NRST

PB3 - SLEEP_n

PB4 - M2_ENC_A

PB5 - M2_ENC_B

PB6 - M3_ENC_B

PB7 - M3_ENC_A

Pins 1,17,15,16,32,14 are used for power. Rest of unmentioned pins were not used


For motor encoder inputs we used 2 channel inputs for each motor. Depicted as ENC_A and ENC_B connections. 
For output we used PWM and DIR (direction) where PWM transmitted speed and DIR chose which way the wheel spun.
The thrower was initially planned to use D-SHOT but due to time limitations we used simple PWM output.
Sleep_n is used to wake up motors.
