# LedDim
ledstrip dimmer ww+cw+y+r Warm White to Work White (Cold white)

Uses STM32F030, 4 N ch fets (IRLML6244 20V/6A) to drive WW+CW ledstrip, Red ledstrip, Amber (Yellow) ledstrip.
There is a simple pushbutton connected to PA0 (Pin 6 on TSSOP20), a potentiometer to PA5 (Pin 11 on TSSOP20) (As a voltage divider of the 3v3). PWM outputs are PA4/TIM14CH1 for WW (Pin 10 on TSSOP20), PA6/TIM3CH1 for Yellow (Pin 12 on TSSOP20), PA7/TIM3CH2 for CW (Pin 13 on TSSOP20), PA10/TIM1CH3 (Pin 18 on TSSOP20) for Red.

Switch chooses dim mode, one per colour and a 2 colour mixing modes: one cw/ww mixing, one from warm to cold mixing in Red and Yellow on the warm end. Potentiometer sets dim level/mixing ratio. Short press on the switch switches mode, long press turns on WW on max.
This might be improved upon in the future.
