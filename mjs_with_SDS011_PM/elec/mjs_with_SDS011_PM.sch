EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "connecting an SDS011 PM meter to a Meetjestad node"
Date "28-8-2020"
Rev "0.1"
Comp "Author: Peter Demmer"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Connection ~ 6550 3450
Wire Wire Line
	5850 3450 6550 3450
Wire Wire Line
	6250 2900 6250 3700
Wire Wire Line
	7050 3450 7450 3450
Wire Wire Line
	7050 4400 7450 4400
Wire Wire Line
	7000 5200 7450 5200
Wire Wire Line
	5200 5200 6850 5200
Wire Wire Line
	4600 5200 5050 5200
Wire Wire Line
	5000 4400 4650 4400
Wire Wire Line
	4600 3900 5050 3900
Wire Wire Line
	4700 2900 4950 2900
Wire Wire Line
	3100 4400 3500 4400
Wire Wire Line
	3100 2900 3500 2900
Wire Wire Line
	5550 2900 5850 2900
Wire Wire Line
	5900 4400 6550 4400
Wire Wire Line
	6550 4400 6850 4400
Wire Wire Line
	6550 3450 6850 3450
Wire Wire Line
	5250 3900 5200 3900
Text Notes 2350 3900 0    50   ~ 0
VMA404
Text GLabel 2850 2900 2    50   Input ~ 10
OUT+
Wire Notes Line width 20 style solid rgb(194, 194, 194)
	2850 2400 2850 5300
Wire Notes Line width 20 style solid rgb(194, 194, 194)
	2150 2400 2150 5300
Text GLabel 2150 2900 0    50   Input ~ 10
IN+
Wire Notes Line width 20 style solid rgb(194, 194, 194)
	2850 5300 2150 5300
Text Notes 2350 3700 0    50   ~ 0
Velleman
Text GLabel 2150 4400 0    50   Input ~ 10
IN-
Wire Notes Line width 20 style solid rgb(194, 194, 194)
	2150 2400 2850 2400
Text GLabel 2850 4400 2    50   Input ~ 10
OUT-
Text GLabel 4450 5200 2    50   Input ~ 0
D6
Text GLabel 4450 2900 2    50   Input ~ 10
+BATT
Text GLabel 4450 3900 2    50   Input ~ 0
D7
Text GLabel 6850 4400 2    50   Input ~ 10
GND
Wire Notes Line width 20 style solid rgb(194, 194, 194)
	4450 2400 4450 5300
Wire Notes Line width 20 style solid rgb(194, 194, 194)
	3750 2400 3750 5300
Text GLabel 3750 2900 0    50   Input ~ 10
+BATT
Wire Notes Line width 20 style solid rgb(194, 194, 194)
	4450 5300 3750 5300
Text Notes 3800 3800 0    50   ~ 0
Meetjestad PCB
Text GLabel 3750 4400 0    50   Input ~ 10
-BATT
Wire Notes Line width 20 style solid rgb(194, 194, 194)
	3750 2400 4450 2400
Wire Notes Line width 20 style solid rgb(194, 194, 194)
	8350 2400 8350 5300
Wire Notes Line width 20 style solid rgb(194, 194, 194)
	7650 2400 7650 5300
Text GLabel 7650 2750 0    50   Input ~ 0
nc
Text GLabel 7650 3100 0    50   Input ~ 0
10um
Text GLabel 7650 3950 0    50   Input ~ 0
2.5um
Text GLabel 5200 5200 0    50   Input ~ 0
D6
Text GLabel 6850 5200 2    50   Input ~ 0
TXD
Wire Notes Line width 20 style solid rgb(194, 194, 194)
	8350 5300 7650 5300
Text GLabel 7650 5200 0    50   Input ~ 0
TXD
Text Notes 7750 3800 0    50   ~ 0
Nova SDS011
Text GLabel 7650 4800 0    50   Input ~ 0
RXD
Text GLabel 7650 4400 0    50   Input ~ 10
GND
Text GLabel 7650 3450 0    50   Input ~ 10
+5V
Wire Notes Line width 20 style solid rgb(194, 194, 194)
	7650 2400 8350 2400
Text GLabel 4450 4400 2    50   Input ~ 10
GND
Text GLabel 6850 3450 2    50   Input ~ 10
+5V
Text GLabel 5200 4400 0    50   Input ~ 10
GND
Text GLabel 5200 3900 0    50   Input ~ 0
D7
Text GLabel 5200 2900 0    50   Input ~ 10
+5.5V
Connection ~ 5550 2900
Connection ~ 6550 4400
Connection ~ 5900 4400
Wire Wire Line
	5200 4400 5900 4400
Wire Wire Line
	6550 3450 6550 3550
Wire Wire Line
	6550 3900 6550 4100
Connection ~ 6550 3900
Wire Wire Line
	6550 3850 6550 3900
Connection ~ 5900 4100
Wire Wire Line
	5900 4100 6250 4100
Wire Wire Line
	5550 4100 5900 4100
$Comp
L Device:R R3
U 1 1 5F48EE79
P 5900 4250
F 0 "R3" H 5970 4296 50  0000 L CNN
F 1 "470" H 5970 4205 50  0000 L CNN
F 2 "" V 5830 4250 50  0001 C CNN
F 3 "~" H 5900 4250 50  0001 C CNN
	1    5900 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 2900 6250 2900
Connection ~ 5850 2900
Wire Wire Line
	5850 2900 5850 3050
Wire Wire Line
	5200 2900 5550 2900
Wire Wire Line
	5550 3250 5550 3700
Connection ~ 5550 3250
Wire Wire Line
	5550 3200 5550 3250
$Comp
L Device:R R2
U 1 1 5F4A2842
P 6550 3700
F 0 "R2" H 6620 3746 50  0000 L CNN
F 1 "1k8" H 6620 3655 50  0000 L CNN
F 2 "" V 6480 3700 50  0001 C CNN
F 3 "~" H 6550 3700 50  0001 C CNN
	1    6550 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_PNP_EBC Q1
U 1 1 5F49CE7C
P 5750 3250
F 0 "Q1" H 5940 3204 50  0000 L CNN
F 1 "2N2904" H 5940 3295 50  0000 L CNN
F 2 "" H 5950 3350 50  0001 C CNN
F 3 "~" H 5750 3250 50  0001 C CNN
	1    5750 3250
	1    0    0    1   
$EndComp
$Comp
L Device:R R1
U 1 1 5F49456D
P 5550 3050
F 0 "R1" H 5620 3096 50  0000 L CNN
F 1 "6k8" H 5620 3005 50  0000 L CNN
F 2 "" V 5480 3050 50  0001 C CNN
F 3 "~" H 5550 3050 50  0001 C CNN
	1    5550 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5F4936E7
P 6550 4250
F 0 "R4" H 6620 4296 50  0000 L CNN
F 1 "3k3" H 6620 4205 50  0000 L CNN
F 2 "" V 6480 4250 50  0001 C CNN
F 3 "~" H 6550 4250 50  0001 C CNN
	1    6550 4250
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:BC547 Q3
U 1 1 5F483DBB
P 6350 3900
F 0 "Q3" H 6541 3946 50  0000 L CNN
F 1 "BC547" H 6541 3855 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 6550 3825 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/BC/BC547.pdf" H 6350 3900 50  0001 L CNN
	1    6350 3900
	-1   0    0    -1  
$EndComp
$Comp
L Transistor_BJT:BC547 Q2
U 1 1 5F47F468
P 5450 3900
F 0 "Q2" H 5641 3946 50  0000 L CNN
F 1 "BC547" H 5641 3855 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 5650 3825 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/BC/BC547.pdf" H 5450 3900 50  0001 L CNN
	1    5450 3900
	1    0    0    -1  
$EndComp
$EndSCHEMATC
