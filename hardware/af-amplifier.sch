EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 10
Title "HBR/CW by R2AUK ::: https://eax.me/hbr-cw-pcb/"
Date "2022-04-02"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:CP1 C11
U 1 1 61F49A1B
P 1100 1600
F 0 "C11" H 1215 1646 50  0000 L CNN
F 1 "10u" H 1215 1555 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_4x5.4" H 1100 1600 50  0001 C CNN
F 3 "~" H 1100 1600 50  0001 C CNN
	1    1100 1600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C10
U 1 1 61F4A414
P 2050 1650
F 0 "C10" H 2165 1696 50  0000 L CNN
F 1 "0.1u" H 2165 1605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2088 1500 50  0001 C CNN
F 3 "~" H 2050 1650 50  0001 C CNN
	1    2050 1650
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR043
U 1 1 61F4AE53
P 1100 950
F 0 "#PWR043" H 1100 800 50  0001 C CNN
F 1 "VCC" H 1115 1123 50  0000 C CNN
F 2 "" H 1100 950 50  0001 C CNN
F 3 "" H 1100 950 50  0001 C CNN
	1    1100 950 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR048
U 1 1 61F4B6DB
P 1600 2050
F 0 "#PWR048" H 1600 1800 50  0001 C CNN
F 1 "GND" H 1605 1877 50  0001 C CNN
F 2 "" H 1600 2050 50  0001 C CNN
F 3 "" H 1600 2050 50  0001 C CNN
	1    1600 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 950  2050 1250
Wire Wire Line
	1300 1250 1100 1250
Wire Wire Line
	1100 1250 1100 950 
Wire Wire Line
	1900 1250 2050 1250
Connection ~ 2050 1250
Wire Wire Line
	2050 1250 2050 1500
Wire Wire Line
	1600 1550 1600 1950
Wire Wire Line
	1600 1950 2050 1950
Wire Wire Line
	2050 1950 2050 1800
Connection ~ 1600 1950
Wire Wire Line
	1600 1950 1600 2050
Wire Wire Line
	1600 1950 1100 1950
$Comp
L Amplifier_Audio:TDA2003 U4
U 1 1 61F4D726
P 7900 2350
F 0 "U4" H 7950 2550 50  0000 L CNN
F 1 "TDA2003" H 7950 2150 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-5_P3.4x3.7mm_StaggerOdd_Lead3.8mm_Vertical" H 7900 2350 50  0001 C CIN
F 3 "http://www.st.com/resource/en/datasheet/cd00000123.pdf" H 7900 2350 50  0001 C CNN
	1    7900 2350
	1    0    0    -1  
$EndComp
Text GLabel 10650 2350 2    50   Output ~ 0
PHONES_A
Text GLabel 9750 2750 2    50   Output ~ 0
PHONES_B
$Comp
L power:+9V #PWR044
U 1 1 61F4C20A
P 2050 950
F 0 "#PWR044" H 2050 800 50  0001 C CNN
F 1 "+9V" H 2065 1123 50  0000 C CNN
F 2 "" H 2050 950 50  0001 C CNN
F 3 "" H 2050 950 50  0001 C CNN
	1    2050 950 
	1    0    0    -1  
$EndComp
$Comp
L power:+9V #PWR045
U 1 1 61F4F3E3
P 7800 1200
F 0 "#PWR045" H 7800 1050 50  0001 C CNN
F 1 "+9V" H 7815 1373 50  0000 C CNN
F 2 "" H 7800 1200 50  0001 C CNN
F 3 "" H 7800 1200 50  0001 C CNN
	1    7800 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 61F504F8
P 8100 1550
F 0 "C8" H 8215 1596 50  0000 L CNN
F 1 "0.1u" H 8215 1505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8138 1400 50  0001 C CNN
F 3 "~" H 8100 1550 50  0001 C CNN
	1    8100 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C9
U 1 1 61F51024
P 8600 1550
F 0 "C9" H 8715 1596 50  0000 L CNN
F 1 "100u" H 8715 1505 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x7.7" H 8600 1550 50  0001 C CNN
F 3 "~" H 8600 1550 50  0001 C CNN
	1    8600 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 2050 7800 1300
$Comp
L power:GND #PWR046
U 1 1 61F51A40
P 8100 1850
F 0 "#PWR046" H 8100 1600 50  0001 C CNN
F 1 "GND" H 8105 1677 50  0001 C CNN
F 2 "" H 8100 1850 50  0001 C CNN
F 3 "" H 8100 1850 50  0001 C CNN
	1    8100 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR047
U 1 1 61F51FAA
P 8600 1850
F 0 "#PWR047" H 8600 1600 50  0001 C CNN
F 1 "GND" H 8605 1677 50  0001 C CNN
F 2 "" H 8600 1850 50  0001 C CNN
F 3 "" H 8600 1850 50  0001 C CNN
	1    8600 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 1300 8100 1300
Wire Wire Line
	8600 1300 8600 1400
Connection ~ 7800 1300
Wire Wire Line
	7800 1300 7800 1200
Wire Wire Line
	8600 1700 8600 1850
Wire Wire Line
	8100 1850 8100 1700
Wire Wire Line
	8100 1400 8100 1300
Connection ~ 8100 1300
Wire Wire Line
	8100 1300 8600 1300
$Comp
L power:GND #PWR051
U 1 1 61F52BE8
P 7800 2800
F 0 "#PWR051" H 7800 2550 50  0001 C CNN
F 1 "GND" H 7805 2627 50  0001 C CNN
F 2 "" H 7800 2800 50  0001 C CNN
F 3 "" H 7800 2800 50  0001 C CNN
	1    7800 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 2800 7800 2650
$Comp
L Device:CP1 C12
U 1 1 61F53396
P 7250 2250
F 0 "C12" V 6998 2250 50  0000 C CNN
F 1 "1u" V 7089 2250 50  0000 C CNN
F 2 "Capacitor_SMD:CP_Elec_4x5.4" H 7250 2250 50  0001 C CNN
F 3 "~" H 7250 2250 50  0001 C CNN
	1    7250 2250
	0    1    1    0   
$EndComp
Text Notes 6400 2850 0    50   ~ 0
Volume control
$Comp
L power:GND #PWR050
U 1 1 61F54E97
P 6650 2600
F 0 "#PWR050" H 6650 2350 50  0001 C CNN
F 1 "GND" H 6655 2427 50  0001 C CNN
F 2 "" H 6650 2600 50  0001 C CNN
F 3 "" H 6650 2600 50  0001 C CNN
	1    6650 2600
	1    0    0    -1  
$EndComp
Text GLabel 6500 1700 0    50   Input ~ 0
AF_AMP_IN
Wire Wire Line
	6500 1700 6650 1700
Wire Wire Line
	7400 2250 7600 2250
$Comp
L Device:R R4
U 1 1 61F56516
P 7800 3200
F 0 "R4" V 7593 3200 50  0000 C CNN
F 1 "220" V 7684 3200 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7730 3200 50  0001 C CNN
F 3 "~" H 7800 3200 50  0001 C CNN
	1    7800 3200
	0    1    1    0   
$EndComp
$Comp
L Device:C C18
U 1 1 61F5703F
P 8250 3200
F 0 "C18" V 7998 3200 50  0000 C CNN
F 1 "47n" V 8089 3200 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8288 3050 50  0001 C CNN
F 3 "~" H 8250 3200 50  0001 C CNN
	1    8250 3200
	0    1    1    0   
$EndComp
Wire Wire Line
	7600 2450 7300 2450
Wire Wire Line
	7300 2450 7300 3200
Wire Wire Line
	7300 3200 7650 3200
Wire Wire Line
	7950 3200 8100 3200
Wire Wire Line
	8200 2350 8600 2350
Wire Wire Line
	8600 2350 8600 3200
Wire Wire Line
	8600 3200 8400 3200
$Comp
L Device:R R5
U 1 1 61F58C0C
P 8950 3200
F 0 "R5" H 8880 3154 50  0000 R CNN
F 1 "680" H 8880 3245 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 8880 3200 50  0001 C CNN
F 3 "~" H 8950 3200 50  0001 C CNN
	1    8950 3200
	-1   0    0    1   
$EndComp
$Comp
L Device:R R12
U 1 1 61F5935E
P 8950 3850
F 0 "R12" H 8880 3804 50  0000 R CNN
F 1 "10" H 8880 3895 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 8880 3850 50  0001 C CNN
F 3 "~" H 8950 3850 50  0001 C CNN
	1    8950 3850
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR055
U 1 1 61F597C7
P 8950 4150
F 0 "#PWR055" H 8950 3900 50  0001 C CNN
F 1 "GND" H 8955 3977 50  0001 C CNN
F 2 "" H 8950 4150 50  0001 C CNN
F 3 "" H 8950 4150 50  0001 C CNN
	1    8950 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 4150 8950 4000
Wire Wire Line
	8950 3050 8950 2750
Wire Wire Line
	8950 2350 8600 2350
Connection ~ 8600 2350
$Comp
L Device:CP1 C20
U 1 1 61F5AD45
P 8000 3600
F 0 "C20" V 8252 3600 50  0000 C CNN
F 1 "220u" V 8161 3600 50  0000 C CNN
F 2 "Capacitor_SMD:CP_Elec_8x10.5" H 8000 3600 50  0001 C CNN
F 3 "~" H 8000 3600 50  0001 C CNN
	1    8000 3600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8950 3350 8950 3600
Wire Wire Line
	8950 3600 8150 3600
Connection ~ 8950 3600
Wire Wire Line
	8950 3600 8950 3700
Wire Wire Line
	7850 3600 7300 3600
Wire Wire Line
	7300 3600 7300 3200
Connection ~ 7300 3200
$Comp
L Device:CP1 C14
U 1 1 61F5D791
P 9350 2750
F 0 "C14" V 9602 2750 50  0000 C CNN
F 1 "220u" V 9511 2750 50  0000 C CNN
F 2 "Capacitor_SMD:CP_Elec_8x10.5" H 9350 2750 50  0001 C CNN
F 3 "~" H 9350 2750 50  0001 C CNN
	1    9350 2750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9200 2750 8950 2750
Connection ~ 8950 2750
Wire Wire Line
	8950 2750 8950 2350
$Comp
L Device:C C17
U 1 1 61F5EC9B
P 9600 3050
F 0 "C17" H 9715 3096 50  0000 L CNN
F 1 "0.1u" H 9715 3005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9638 2900 50  0001 C CNN
F 3 "~" H 9600 3050 50  0001 C CNN
	1    9600 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 61F5F331
P 9600 3550
F 0 "R9" H 9530 3504 50  0000 R CNN
F 1 "10" H 9530 3595 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9530 3550 50  0001 C CNN
F 3 "~" H 9600 3550 50  0001 C CNN
	1    9600 3550
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR054
U 1 1 61F5F842
P 9600 3850
F 0 "#PWR054" H 9600 3600 50  0001 C CNN
F 1 "GND" H 9605 3677 50  0001 C CNN
F 2 "" H 9600 3850 50  0001 C CNN
F 3 "" H 9600 3850 50  0001 C CNN
	1    9600 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 2750 9600 2750
Wire Wire Line
	9600 2750 9600 2900
Connection ~ 9600 2750
Wire Wire Line
	9600 2750 9750 2750
Wire Wire Line
	9600 3200 9600 3400
Wire Wire Line
	9600 3700 9600 3850
$Comp
L Device:CP1 C13
U 1 1 61F61FBF
P 9750 2350
F 0 "C13" V 10002 2350 50  0000 C CNN
F 1 "220u" V 9911 2350 50  0000 C CNN
F 2 "Capacitor_SMD:CP_Elec_8x10.5" H 9750 2350 50  0001 C CNN
F 3 "~" H 9750 2350 50  0001 C CNN
	1    9750 2350
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C15
U 1 1 61F62CA8
P 10400 2750
F 0 "C15" H 10515 2796 50  0000 L CNN
F 1 "0.1u" H 10515 2705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 10438 2600 50  0001 C CNN
F 3 "~" H 10400 2750 50  0001 C CNN
	1    10400 2750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 61F62CB2
P 10400 3250
F 0 "R6" H 10330 3204 50  0000 R CNN
F 1 "10" H 10330 3295 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 10330 3250 50  0001 C CNN
F 3 "~" H 10400 3250 50  0001 C CNN
	1    10400 3250
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR053
U 1 1 61F62CBC
P 10400 3550
F 0 "#PWR053" H 10400 3300 50  0001 C CNN
F 1 "GND" H 10405 3377 50  0001 C CNN
F 2 "" H 10400 3550 50  0001 C CNN
F 3 "" H 10400 3550 50  0001 C CNN
	1    10400 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	10400 2900 10400 3100
Wire Wire Line
	10400 3400 10400 3550
Wire Wire Line
	10650 2350 10400 2350
Wire Wire Line
	9600 2350 8950 2350
Connection ~ 8950 2350
Wire Wire Line
	10400 2350 10400 2600
Connection ~ 10400 2350
Wire Wire Line
	10400 2350 9900 2350
Text Notes 7950 2700 0    50   ~ 0
w/ heat sink
Text GLabel 6450 1500 0    50   Input ~ 0
CW_TONE
Wire Wire Line
	6450 1500 6950 1500
Wire Wire Line
	6950 1500 6950 2250
Connection ~ 6950 2250
Wire Wire Line
	6950 2250 7100 2250
Text Notes 6050 1400 0    50   ~ 0
see /Misc/
Text GLabel 8850 5500 2    50   Output ~ 0
AF_AMP_IN
$Comp
L Device:R R?
U 1 1 624B0E23
P 8000 5800
AR Path="/6212C0AC/624B0E23" Ref="R?"  Part="1" 
AR Path="/61F4800D/624B0E23" Ref="R17"  Part="1" 
AR Path="/6225668B/624B0E23" Ref="R?"  Part="1" 
F 0 "R17" H 8070 5846 50  0000 L CNN
F 1 "10K" H 8070 5755 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7930 5800 50  0001 C CNN
F 3 "~" H 8000 5800 50  0001 C CNN
	1    8000 5800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 624B0E29
P 8000 6050
AR Path="/6212C0AC/624B0E29" Ref="#PWR?"  Part="1" 
AR Path="/61F4800D/624B0E29" Ref="#PWR064"  Part="1" 
AR Path="/6225668B/624B0E29" Ref="#PWR?"  Part="1" 
F 0 "#PWR064" H 8000 5800 50  0001 C CNN
F 1 "GND" H 8005 5877 50  0001 C CNN
F 2 "" H 8000 6050 50  0001 C CNN
F 3 "" H 8000 6050 50  0001 C CNN
	1    8000 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 6050 8000 5950
Wire Wire Line
	8700 5500 8850 5500
Text GLabel 6550 4650 0    50   Input ~ 0
ENABLE_RX
Wire Wire Line
	7800 5500 8000 5500
Wire Wire Line
	8000 5650 8000 5500
Connection ~ 8000 5500
Wire Wire Line
	7200 5500 7400 5500
Text GLabel 7200 5500 0    50   Input ~ 0
AF_PREAMP_OUT
$Comp
L Device:CP1 C27
U 1 1 624B0E37
P 7200 5000
AR Path="/61F4800D/624B0E37" Ref="C27"  Part="1" 
AR Path="/6225668B/624B0E37" Ref="C?"  Part="1" 
F 0 "C27" H 7315 5046 50  0000 L CNN
F 1 "47u" H 7315 4955 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x5.8" H 7200 5000 50  0001 C CNN
F 3 "~" H 7200 5000 50  0001 C CNN
	1    7200 5000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 624B0E3D
P 7200 5250
AR Path="/6212C0AC/624B0E3D" Ref="#PWR?"  Part="1" 
AR Path="/61F4800D/624B0E3D" Ref="#PWR063"  Part="1" 
AR Path="/6225668B/624B0E3D" Ref="#PWR?"  Part="1" 
F 0 "#PWR063" H 7200 5000 50  0001 C CNN
F 1 "GND" H 7205 5077 50  0001 C CNN
F 2 "" H 7200 5250 50  0001 C CNN
F 3 "" H 7200 5250 50  0001 C CNN
	1    7200 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 5250 7200 5150
Wire Wire Line
	6550 4650 6650 4650
Wire Wire Line
	6950 4650 7200 4650
Wire Wire Line
	7600 4650 7600 5200
Wire Wire Line
	8500 4650 8500 5200
Wire Wire Line
	7200 4650 7200 4850
Connection ~ 7200 4650
Wire Wire Line
	7200 4650 7600 4650
$Comp
L Device:R R16
U 1 1 624B0E4B
P 6800 4650
AR Path="/61F4800D/624B0E4B" Ref="R16"  Part="1" 
AR Path="/6225668B/624B0E4B" Ref="R?"  Part="1" 
F 0 "R16" V 7007 4650 50  0000 C CNN
F 1 "2.2K" V 6916 4650 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 6730 4650 50  0001 C CNN
F 3 "~" H 6800 4650 50  0001 C CNN
	1    6800 4650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8000 5500 8300 5500
Wire Wire Line
	7600 4650 8500 4650
Connection ~ 7600 4650
Text GLabel 1450 3850 0    50   Input ~ 0
MIX1_IF
Text GLabel 5400 3850 2    50   Output ~ 0
AF_PREAMP_OUT
$Comp
L Device:C C?
U 1 1 624C42FF
P 1650 4100
AR Path="/6225668B/624C42FF" Ref="C?"  Part="1" 
AR Path="/61F4800D/624C42FF" Ref="C23"  Part="1" 
F 0 "C23" H 1765 4146 50  0000 L CNN
F 1 "0.01u" H 1765 4055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1688 3950 50  0001 C CNN
F 3 "~" H 1650 4100 50  0001 C CNN
	1    1650 4100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 624C4305
P 1650 4500
AR Path="/6225668B/624C4305" Ref="R?"  Part="1" 
AR Path="/61F4800D/624C4305" Ref="R15"  Part="1" 
F 0 "R15" H 1720 4546 50  0000 L CNN
F 1 "51" H 1720 4455 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1580 4500 50  0001 C CNN
F 3 "~" H 1650 4500 50  0001 C CNN
	1    1650 4500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 624C430B
P 1650 4750
AR Path="/6225668B/624C430B" Ref="#PWR?"  Part="1" 
AR Path="/61F4800D/624C430B" Ref="#PWR058"  Part="1" 
F 0 "#PWR058" H 1650 4500 50  0001 C CNN
F 1 "GND" H 1655 4577 50  0001 C CNN
F 2 "" H 1650 4750 50  0001 C CNN
F 3 "" H 1650 4750 50  0001 C CNN
	1    1650 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 624C4311
P 1950 3850
AR Path="/6225668B/624C4311" Ref="R?"  Part="1" 
AR Path="/61F4800D/624C4311" Ref="R10"  Part="1" 
F 0 "R10" V 1743 3850 50  0000 C CNN
F 1 "2.2K" V 1834 3850 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1880 3850 50  0001 C CNN
F 3 "~" H 1950 3850 50  0001 C CNN
	1    1950 3850
	0    1    1    0   
$EndComp
$Comp
L Device:C C?
U 1 1 624C4317
P 2300 4150
AR Path="/6225668B/624C4317" Ref="C?"  Part="1" 
AR Path="/61F4800D/624C4317" Ref="C25"  Part="1" 
F 0 "C25" H 2415 4196 50  0000 L CNN
F 1 "47n" H 2415 4105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2338 4000 50  0001 C CNN
F 3 "~" H 2300 4150 50  0001 C CNN
	1    2300 4150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 624C431D
P 2300 4750
AR Path="/6225668B/624C431D" Ref="#PWR?"  Part="1" 
AR Path="/61F4800D/624C431D" Ref="#PWR059"  Part="1" 
F 0 "#PWR059" H 2300 4500 50  0001 C CNN
F 1 "GND" H 2305 4577 50  0001 C CNN
F 2 "" H 2300 4750 50  0001 C CNN
F 3 "" H 2300 4750 50  0001 C CNN
	1    2300 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C?
U 1 1 624C4323
P 2600 3850
AR Path="/6225668B/624C4323" Ref="C?"  Part="1" 
AR Path="/61F4800D/624C4323" Ref="C21"  Part="1" 
F 0 "C21" V 2852 3850 50  0000 C CNN
F 1 "10u" V 2761 3850 50  0000 C CNN
F 2 "Capacitor_SMD:CP_Elec_4x5.4" H 2600 3850 50  0001 C CNN
F 3 "~" H 2600 3850 50  0001 C CNN
	1    2600 3850
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R?
U 1 1 624C4329
P 2900 4250
AR Path="/6225668B/624C4329" Ref="R?"  Part="1" 
AR Path="/61F4800D/624C4329" Ref="R13"  Part="1" 
F 0 "R13" H 2970 4296 50  0000 L CNN
F 1 "47K" H 2970 4205 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2830 4250 50  0001 C CNN
F 3 "~" H 2900 4250 50  0001 C CNN
	1    2900 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 624C432F
P 2900 3400
AR Path="/6225668B/624C432F" Ref="R?"  Part="1" 
AR Path="/61F4800D/624C432F" Ref="R8"  Part="1" 
F 0 "R8" H 2970 3446 50  0000 L CNN
F 1 "100K" H 2970 3355 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2830 3400 50  0001 C CNN
F 3 "~" H 2900 3400 50  0001 C CNN
	1    2900 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 624C433B
P 3350 4450
AR Path="/6225668B/624C433B" Ref="R?"  Part="1" 
AR Path="/61F4800D/624C433B" Ref="R14"  Part="1" 
F 0 "R14" H 3420 4496 50  0000 L CNN
F 1 "1K" H 3420 4405 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3280 4450 50  0001 C CNN
F 3 "~" H 3350 4450 50  0001 C CNN
	1    3350 4450
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C?
U 1 1 624C4341
P 3750 4450
AR Path="/6225668B/624C4341" Ref="C?"  Part="1" 
AR Path="/61F4800D/624C4341" Ref="C26"  Part="1" 
F 0 "C26" H 3865 4496 50  0000 L CNN
F 1 "470u" H 3865 4405 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_10x10.5" H 3750 4450 50  0001 C CNN
F 3 "~" H 3750 4450 50  0001 C CNN
	1    3750 4450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 624C4347
P 2900 4750
AR Path="/6225668B/624C4347" Ref="#PWR?"  Part="1" 
AR Path="/61F4800D/624C4347" Ref="#PWR060"  Part="1" 
F 0 "#PWR060" H 2900 4500 50  0001 C CNN
F 1 "GND" H 2905 4577 50  0001 C CNN
F 2 "" H 2900 4750 50  0001 C CNN
F 3 "" H 2900 4750 50  0001 C CNN
	1    2900 4750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 624C434D
P 3350 4750
AR Path="/6225668B/624C434D" Ref="#PWR?"  Part="1" 
AR Path="/61F4800D/624C434D" Ref="#PWR061"  Part="1" 
F 0 "#PWR061" H 3350 4500 50  0001 C CNN
F 1 "GND" H 3355 4577 50  0001 C CNN
F 2 "" H 3350 4750 50  0001 C CNN
F 3 "" H 3350 4750 50  0001 C CNN
	1    3350 4750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 624C4353
P 3750 4750
AR Path="/6225668B/624C4353" Ref="#PWR?"  Part="1" 
AR Path="/61F4800D/624C4353" Ref="#PWR062"  Part="1" 
F 0 "#PWR062" H 3750 4500 50  0001 C CNN
F 1 "GND" H 3755 4577 50  0001 C CNN
F 2 "" H 3750 4750 50  0001 C CNN
F 3 "" H 3750 4750 50  0001 C CNN
	1    3750 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 624C4359
P 3350 3300
AR Path="/6225668B/624C4359" Ref="R?"  Part="1" 
AR Path="/61F4800D/624C4359" Ref="R7"  Part="1" 
F 0 "R7" H 3420 3346 50  0000 L CNN
F 1 "2.2K" H 3420 3255 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3280 3300 50  0001 C CNN
F 3 "~" H 3350 3300 50  0001 C CNN
	1    3350 3300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 624C435F
P 3350 2700
AR Path="/6225668B/624C435F" Ref="R?"  Part="1" 
AR Path="/61F4800D/624C435F" Ref="R3"  Part="1" 
F 0 "R3" H 3420 2746 50  0000 L CNN
F 1 "100" H 3420 2655 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3280 2700 50  0001 C CNN
F 3 "~" H 3350 2700 50  0001 C CNN
	1    3350 2700
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C?
U 1 1 624C4365
P 3850 3550
AR Path="/6225668B/624C4365" Ref="C?"  Part="1" 
AR Path="/61F4800D/624C4365" Ref="C19"  Part="1" 
F 0 "C19" V 4102 3550 50  0000 C CNN
F 1 "22u" V 4011 3550 50  0000 C CNN
F 2 "Capacitor_SMD:CP_Elec_5x5.4" H 3850 3550 50  0001 C CNN
F 3 "~" H 3850 3550 50  0001 C CNN
	1    3850 3550
	0    -1   -1   0   
$EndComp
$Comp
L Device:CP1 C?
U 1 1 624C436B
P 3850 3000
AR Path="/6225668B/624C436B" Ref="C?"  Part="1" 
AR Path="/61F4800D/624C436B" Ref="C16"  Part="1" 
F 0 "C16" V 4102 3000 50  0000 C CNN
F 1 "100u" V 4011 3000 50  0000 C CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x7.7" H 3850 3000 50  0001 C CNN
F 3 "~" H 3850 3000 50  0001 C CNN
	1    3850 3000
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 624C4371
P 4150 3150
AR Path="/6225668B/624C4371" Ref="#PWR?"  Part="1" 
AR Path="/61F4800D/624C4371" Ref="#PWR052"  Part="1" 
F 0 "#PWR052" H 4150 2900 50  0001 C CNN
F 1 "GND" H 4155 2977 50  0001 C CNN
F 2 "" H 4150 3150 50  0001 C CNN
F 3 "" H 4150 3150 50  0001 C CNN
	1    4150 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:R_POT RV?
U 1 1 624C4377
P 4200 3850
AR Path="/6225668B/624C4377" Ref="RV?"  Part="1" 
AR Path="/61F4800D/624C4377" Ref="RV2"  Part="1" 
F 0 "RV2" H 4131 3896 50  0000 R CNN
F 1 "10K" H 4131 3805 50  0000 R CNN
F 2 "Potentiometer_THT:Potentiometer_Bourns_3296W_Vertical" H 4200 3850 50  0001 C CNN
F 3 "~" H 4200 3850 50  0001 C CNN
	1    4200 3850
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C?
U 1 1 624C437D
P 4600 3850
AR Path="/6225668B/624C437D" Ref="C?"  Part="1" 
AR Path="/61F4800D/624C437D" Ref="C22"  Part="1" 
F 0 "C22" V 4852 3850 50  0000 C CNN
F 1 "22u" V 4761 3850 50  0000 C CNN
F 2 "Capacitor_SMD:CP_Elec_5x5.4" H 4600 3850 50  0001 C CNN
F 3 "~" H 4600 3850 50  0001 C CNN
	1    4600 3850
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R?
U 1 1 624C4383
P 4950 3850
AR Path="/6225668B/624C4383" Ref="R?"  Part="1" 
AR Path="/61F4800D/624C4383" Ref="R11"  Part="1" 
F 0 "R11" V 4743 3850 50  0000 C CNN
F 1 "2.2K" V 4834 3850 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4880 3850 50  0001 C CNN
F 3 "~" H 4950 3850 50  0001 C CNN
	1    4950 3850
	0    1    1    0   
$EndComp
$Comp
L Device:C C?
U 1 1 624C4389
P 5250 4100
AR Path="/6225668B/624C4389" Ref="C?"  Part="1" 
AR Path="/61F4800D/624C4389" Ref="C24"  Part="1" 
F 0 "C24" H 5365 4146 50  0000 L CNN
F 1 "47n" H 5365 4055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5288 3950 50  0001 C CNN
F 3 "~" H 5250 4100 50  0001 C CNN
	1    5250 4100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 624C438F
P 5250 4350
AR Path="/6225668B/624C438F" Ref="#PWR?"  Part="1" 
AR Path="/61F4800D/624C438F" Ref="#PWR057"  Part="1" 
F 0 "#PWR057" H 5250 4100 50  0001 C CNN
F 1 "GND" H 5255 4177 50  0001 C CNN
F 2 "" H 5250 4350 50  0001 C CNN
F 3 "" H 5250 4350 50  0001 C CNN
	1    5250 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 3850 2300 3850
Wire Wire Line
	2750 3850 2900 3850
Wire Wire Line
	3350 3650 3350 3550
Wire Wire Line
	3350 3150 3350 3000
Wire Wire Line
	3350 3000 3700 3000
Connection ~ 3350 3000
Wire Wire Line
	3350 3000 3350 2850
Wire Wire Line
	4000 3000 4150 3000
Wire Wire Line
	4150 3000 4150 3150
Wire Wire Line
	3350 3550 3700 3550
Connection ~ 3350 3550
Wire Wire Line
	3350 3550 3350 3450
$Comp
L power:GND #PWR?
U 1 1 624C43A1
P 4200 4350
AR Path="/6225668B/624C43A1" Ref="#PWR?"  Part="1" 
AR Path="/61F4800D/624C43A1" Ref="#PWR056"  Part="1" 
F 0 "#PWR056" H 4200 4100 50  0001 C CNN
F 1 "GND" H 4205 4177 50  0001 C CNN
F 2 "" H 4200 4350 50  0001 C CNN
F 3 "" H 4200 4350 50  0001 C CNN
	1    4200 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 4350 4200 4000
Wire Wire Line
	4200 3700 4200 3550
Wire Wire Line
	4200 3550 4000 3550
Wire Wire Line
	5100 3850 5250 3850
Wire Wire Line
	5250 3850 5250 3950
Connection ~ 5250 3850
Wire Wire Line
	5250 3850 5400 3850
Wire Wire Line
	5250 4250 5250 4350
Wire Wire Line
	3350 4750 3350 4600
Wire Wire Line
	3350 4300 3350 4200
Wire Wire Line
	3350 4200 3750 4200
Wire Wire Line
	3750 4200 3750 4300
Connection ~ 3350 4200
Wire Wire Line
	3350 4200 3350 4050
Wire Wire Line
	3750 4600 3750 4750
Wire Wire Line
	2900 4750 2900 4400
Wire Wire Line
	2900 3850 2900 4100
Connection ~ 2900 3850
Wire Wire Line
	2900 3850 3050 3850
Wire Wire Line
	2300 4750 2300 4300
Wire Wire Line
	2300 4000 2300 3850
Connection ~ 2300 3850
Wire Wire Line
	2300 3850 2450 3850
Wire Wire Line
	1450 3850 1650 3850
Wire Wire Line
	1650 3850 1650 3950
Connection ~ 1650 3850
Wire Wire Line
	1650 3850 1800 3850
Wire Wire Line
	1650 4250 1650 4350
Wire Wire Line
	1650 4650 1650 4750
Wire Wire Line
	2900 3850 2900 3550
Wire Wire Line
	2900 3250 2900 3000
Wire Wire Line
	2900 3000 3350 3000
Wire Wire Line
	3350 2400 3350 2550
Wire Wire Line
	4450 3850 4350 3850
Wire Wire Line
	4750 3850 4800 3850
Text Notes 1450 5050 0    50   ~ 0
Diplexer and 1.5 kHz RC LPF
Text Notes 4700 5050 0    50   ~ 0
1.5 kHz RC LPF
$Comp
L Transistor_FET:MMBF170 Q3
U 1 1 627E15DA
P 8500 5400
F 0 "Q3" V 8749 5400 50  0000 C CNN
F 1 "MMBF170" V 8840 5400 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 8700 5325 50  0001 L CIN
F 3 "https://www.diodes.com/assets/Datasheets/ds30104.pdf" H 8500 5400 50  0001 L CNN
	1    8500 5400
	0    1    1    0   
$EndComp
$Comp
L Transistor_FET:MMBF170 Q2
U 1 1 627EA152
P 7600 5400
F 0 "Q2" V 7849 5400 50  0000 C CNN
F 1 "MMBF170" V 7940 5400 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 7800 5325 50  0001 L CIN
F 3 "https://www.diodes.com/assets/Datasheets/ds30104.pdf" H 7600 5400 50  0001 L CNN
	1    7600 5400
	0    -1   1    0   
$EndComp
$Comp
L Transistor_BJT:MMBT3904 Q1
U 1 1 62811FE0
P 3250 3850
F 0 "Q1" H 3441 3896 50  0000 L CNN
F 1 "MMBT3904" H 3441 3805 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3450 3775 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N3904.pdf" H 3250 3850 50  0001 L CNN
	1    3250 3850
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:L7809 U3
U 1 1 62D291A6
P 1600 1250
F 0 "U3" H 1600 1492 50  0000 C CNN
F 1 "L7809" H 1600 1401 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TO-263-2" H 1625 1100 50  0001 L CIN
F 3 "" H 1600 1200 50  0001 C CNN
	1    1600 1250
	1    0    0    -1  
$EndComp
$Comp
L power:+9V #PWR049
U 1 1 62E92810
P 3350 2400
F 0 "#PWR049" H 3350 2250 50  0001 C CNN
F 1 "+9V" H 3365 2573 50  0000 C CNN
F 2 "" H 3350 2400 50  0001 C CNN
F 3 "" H 3350 2400 50  0001 C CNN
	1    3350 2400
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J?
U 1 1 630EA6D0
P 6450 2250
AR Path="/630EA6D0" Ref="J?"  Part="1" 
AR Path="/61F4800D/630EA6D0" Ref="J29"  Part="1" 
F 0 "J29" H 6600 2350 50  0000 C CNN
F 1 "VOL" H 6600 2250 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x03_P2.54mm_Vertical" H 6368 2475 50  0001 C CNN
F 3 "~" H 6450 2250 50  0001 C CNN
	1    6450 2250
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6650 1700 6650 2150
Wire Wire Line
	6650 2250 6950 2250
Wire Wire Line
	6650 2350 6650 2600
Wire Wire Line
	1100 1750 1100 1950
Wire Wire Line
	1100 1450 1100 1250
Connection ~ 1100 1250
$EndSCHEMATC
