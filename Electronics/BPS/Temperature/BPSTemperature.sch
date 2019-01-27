EESchema Schematic File Version 4
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Sensor_Temperature:LTC2983 U?
U 1 1 5C4CD6FF
P 6600 3500
F 0 "U?" H 6600 5078 50  0000 C CNN
F 1 "LTC2983" H 6600 4987 50  0000 C CNN
F 2 "Package_QFP:LQFP-48_7x7mm_P0.5mm" H 5850 4700 50  0001 C CNN
F 3 "http://cds.linear.com/docs/en/datasheet/2983fc.pdf" H 6600 3500 50  0001 C CNN
	1    6600 3500
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 5C4CC3F1
P 6600 1750
F 0 "#PWR?" H 6600 1600 50  0001 C CNN
F 1 "+5V" H 6615 1923 50  0000 C CNN
F 2 "" H 6600 1750 50  0001 C CNN
F 3 "" H 6600 1750 50  0001 C CNN
	1    6600 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 1750 6600 1950
Wire Wire Line
	6500 2100 6500 1950
Wire Wire Line
	6500 1950 6600 1950
Connection ~ 6600 1950
Wire Wire Line
	6600 1950 6600 2100
Wire Wire Line
	6700 2100 6700 1950
Wire Wire Line
	6700 1950 6600 1950
Wire Wire Line
	6800 2100 6800 1950
Wire Wire Line
	6800 1950 6700 1950
Connection ~ 6700 1950
Wire Wire Line
	6400 2100 6400 1950
Wire Wire Line
	6400 1950 6500 1950
Connection ~ 6500 1950
$Comp
L Device:C C?
U 1 1 5C4CC50A
P 7100 1950
F 0 "C?" V 6848 1950 50  0000 C CNN
F 1 "0.1uF" V 6939 1950 50  0000 C CNN
F 2 "" H 7138 1800 50  0001 C CNN
F 3 "~" H 7100 1950 50  0001 C CNN
	1    7100 1950
	0    1    1    0   
$EndComp
Wire Wire Line
	6950 1950 6800 1950
Connection ~ 6800 1950
$Comp
L power:GND #PWR?
U 1 1 5C4CC677
P 7350 1950
F 0 "#PWR?" H 7350 1700 50  0001 C CNN
F 1 "GND" V 7355 1822 50  0000 R CNN
F 2 "" H 7350 1950 50  0001 C CNN
F 3 "" H 7350 1950 50  0001 C CNN
	1    7350 1950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7250 1950 7350 1950
$Comp
L Device:C C?
U 1 1 5C4CC7BB
P 8500 3300
F 0 "C?" H 8615 3346 50  0000 L CNN
F 1 "1uF" H 8615 3255 50  0000 L CNN
F 2 "" H 8538 3150 50  0001 C CNN
F 3 "~" H 8500 3300 50  0001 C CNN
	1    8500 3300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4CC817
P 7750 3650
F 0 "C?" H 7865 3696 50  0000 L CNN
F 1 "10uF" H 7865 3605 50  0000 L CNN
F 2 "" H 7788 3500 50  0001 C CNN
F 3 "~" H 7750 3650 50  0001 C CNN
	1    7750 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4CC859
P 9550 1850
F 0 "C?" H 9665 1896 50  0000 L CNN
F 1 "C" H 9665 1805 50  0000 L CNN
F 2 "" H 9588 1700 50  0001 C CNN
F 3 "~" H 9550 1850 50  0001 C CNN
	1    9550 1850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4CC88B
P 7500 2550
F 0 "C?" H 7615 2596 50  0000 L CNN
F 1 "10uF" H 7615 2505 50  0000 L CNN
F 2 "" H 7538 2400 50  0001 C CNN
F 3 "~" H 7500 2550 50  0001 C CNN
	1    7500 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4CC8ED
P 7900 2800
F 0 "C?" V 8050 2800 50  0000 C CNN
F 1 "10uF" V 7750 2800 50  0000 C CNN
F 2 "" H 7938 2650 50  0001 C CNN
F 3 "~" H 7900 2800 50  0001 C CNN
	1    7900 2800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7400 2400 7500 2400
Wire Wire Line
	7500 2700 7400 2700
$Comp
L power:GND #PWR?
U 1 1 5C4CCB2D
P 9850 1750
F 0 "#PWR?" H 9850 1500 50  0001 C CNN
F 1 "GND" H 9855 1577 50  0000 C CNN
F 2 "" H 9850 1750 50  0001 C CNN
F 3 "" H 9850 1750 50  0001 C CNN
	1    9850 1750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4CCEAB
P 8150 2800
F 0 "#PWR?" H 8150 2550 50  0001 C CNN
F 1 "GND" V 8155 2672 50  0000 R CNN
F 2 "" H 8150 2800 50  0001 C CNN
F 3 "" H 8150 2800 50  0001 C CNN
	1    8150 2800
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4CD513
P 8500 3500
F 0 "#PWR?" H 8500 3250 50  0001 C CNN
F 1 "GND" H 8505 3327 50  0000 C CNN
F 2 "" H 8500 3500 50  0001 C CNN
F 3 "" H 8500 3500 50  0001 C CNN
	1    8500 3500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4CD7F7
P 8150 3450
F 0 "C?" H 8265 3496 50  0000 L CNN
F 1 "1uF" H 8265 3405 50  0000 L CNN
F 2 "" H 8188 3300 50  0001 C CNN
F 3 "~" H 8150 3450 50  0001 C CNN
	1    8150 3450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4CD7FE
P 8150 3650
F 0 "#PWR?" H 8150 3400 50  0001 C CNN
F 1 "GND" H 8155 3477 50  0000 C CNN
F 2 "" H 8150 3650 50  0001 C CNN
F 3 "" H 8150 3650 50  0001 C CNN
	1    8150 3650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4CE797
P 7750 3850
F 0 "#PWR?" H 7750 3600 50  0001 C CNN
F 1 "GND" H 7755 3677 50  0000 C CNN
F 2 "" H 7750 3850 50  0001 C CNN
F 3 "" H 7750 3850 50  0001 C CNN
	1    7750 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 3800 7750 3850
Wire Wire Line
	7400 3500 7750 3500
$Comp
L power:GND #PWR?
U 1 1 5C4CF846
P 6500 5000
F 0 "#PWR?" H 6500 4750 50  0001 C CNN
F 1 "GND" H 6505 4827 50  0000 C CNN
F 2 "" H 6500 5000 50  0001 C CNN
F 3 "" H 6500 5000 50  0001 C CNN
	1    6500 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 5000 6500 4950
Wire Wire Line
	6200 4800 6200 4950
Wire Wire Line
	6200 4950 6500 4950
Connection ~ 6500 4950
Wire Wire Line
	6500 4950 6500 4900
Wire Wire Line
	6300 4800 6300 4900
Wire Wire Line
	6300 4900 6500 4900
Connection ~ 6500 4900
Wire Wire Line
	6500 4900 6500 4850
Wire Wire Line
	6400 4800 6400 4850
Wire Wire Line
	6400 4850 6500 4850
Connection ~ 6500 4850
Wire Wire Line
	6500 4850 6500 4800
Wire Wire Line
	6600 4800 6600 4850
Wire Wire Line
	6600 4850 6500 4850
Wire Wire Line
	6700 4800 6700 4900
Wire Wire Line
	6700 4900 6500 4900
Wire Wire Line
	6800 4800 6800 4950
Wire Wire Line
	6800 4950 6500 4950
Wire Wire Line
	6900 4800 6900 4950
Wire Wire Line
	6900 4950 6800 4950
Connection ~ 6800 4950
$Comp
L power:GND #PWR?
U 1 1 5C4D2459
P 5650 4550
F 0 "#PWR?" H 5650 4300 50  0001 C CNN
F 1 "GND" H 5655 4377 50  0000 C CNN
F 2 "" H 5650 4550 50  0001 C CNN
F 3 "" H 5650 4550 50  0001 C CNN
	1    5650 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 4500 5650 4500
Wire Wire Line
	5650 4500 5650 4550
Wire Wire Line
	8500 3000 8500 3100
Connection ~ 8500 3100
Wire Wire Line
	8500 3100 8500 3150
Wire Wire Line
	8500 3450 8500 3500
Wire Wire Line
	7400 3300 8150 3300
Wire Wire Line
	8150 3600 8150 3650
Wire Wire Line
	7400 3000 8500 3000
Wire Wire Line
	7400 3100 8500 3100
Wire Wire Line
	8050 2800 8150 2800
Wire Wire Line
	7750 2800 7400 2800
$Comp
L Device:R R?
U 1 1 5C4D96FF
P 9100 1000
F 0 "R?" H 9170 1046 50  0000 L CNN
F 1 "1k" H 9170 955 50  0000 L CNN
F 2 "" V 9030 1000 50  0001 C CNN
F 3 "~" H 9100 1000 50  0001 C CNN
	1    9100 1000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4D97A0
P 9100 1450
F 0 "C?" H 9215 1496 50  0000 L CNN
F 1 "1uF" H 9215 1405 50  0000 L CNN
F 2 "" H 9138 1300 50  0001 C CNN
F 3 "~" H 9100 1450 50  0001 C CNN
	1    9100 1450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4D9F5A
P 9100 1700
F 0 "#PWR?" H 9100 1450 50  0001 C CNN
F 1 "GND" H 9105 1527 50  0000 C CNN
F 2 "" H 9100 1700 50  0001 C CNN
F 3 "" H 9100 1700 50  0001 C CNN
	1    9100 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9100 850  8900 850 
Wire Wire Line
	9100 1700 9100 1600
Wire Wire Line
	9100 1150 9100 1200
Wire Wire Line
	9100 1200 9200 1200
Connection ~ 9100 1200
Wire Wire Line
	9100 1200 9100 1300
$Comp
L Device:R R?
U 1 1 5C4DF311
P 1500 850
F 0 "R?" H 1570 896 50  0000 L CNN
F 1 "1k" H 1570 805 50  0000 L CNN
F 2 "" V 1430 850 50  0001 C CNN
F 3 "~" H 1500 850 50  0001 C CNN
	1    1500 850 
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4DF318
P 1500 1300
F 0 "C?" H 1615 1346 50  0000 L CNN
F 1 "1uF" H 1615 1255 50  0000 L CNN
F 2 "" H 1538 1150 50  0001 C CNN
F 3 "~" H 1500 1300 50  0001 C CNN
	1    1500 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4DF31F
P 1500 1550
F 0 "#PWR?" H 1500 1300 50  0001 C CNN
F 1 "GND" H 1505 1377 50  0000 C CNN
F 2 "" H 1500 1550 50  0001 C CNN
F 3 "" H 1500 1550 50  0001 C CNN
	1    1500 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 700  1300 700 
Wire Wire Line
	1500 1550 1500 1450
Wire Wire Line
	1500 1000 1500 1050
Wire Wire Line
	1500 1050 1600 1050
Connection ~ 1500 1050
Wire Wire Line
	1500 1050 1500 1150
$Comp
L Device:R R?
U 1 1 5C4DFF30
P 1500 1950
F 0 "R?" H 1570 1996 50  0000 L CNN
F 1 "1k" H 1570 1905 50  0000 L CNN
F 2 "" V 1430 1950 50  0001 C CNN
F 3 "~" H 1500 1950 50  0001 C CNN
	1    1500 1950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4DFF37
P 1500 2400
F 0 "C?" H 1615 2446 50  0000 L CNN
F 1 "1uF" H 1615 2355 50  0000 L CNN
F 2 "" H 1538 2250 50  0001 C CNN
F 3 "~" H 1500 2400 50  0001 C CNN
	1    1500 2400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4DFF3E
P 1500 2650
F 0 "#PWR?" H 1500 2400 50  0001 C CNN
F 1 "GND" H 1505 2477 50  0000 C CNN
F 2 "" H 1500 2650 50  0001 C CNN
F 3 "" H 1500 2650 50  0001 C CNN
	1    1500 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 1800 1300 1800
Wire Wire Line
	1500 2650 1500 2550
Wire Wire Line
	1500 2100 1500 2150
Wire Wire Line
	1500 2150 1600 2150
Connection ~ 1500 2150
Wire Wire Line
	1500 2150 1500 2250
$Comp
L Device:R R?
U 1 1 5C4E0FCE
P 1500 3050
F 0 "R?" H 1570 3096 50  0000 L CNN
F 1 "1k" H 1570 3005 50  0000 L CNN
F 2 "" V 1430 3050 50  0001 C CNN
F 3 "~" H 1500 3050 50  0001 C CNN
	1    1500 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4E0FD5
P 1500 3500
F 0 "C?" H 1615 3546 50  0000 L CNN
F 1 "1uF" H 1615 3455 50  0000 L CNN
F 2 "" H 1538 3350 50  0001 C CNN
F 3 "~" H 1500 3500 50  0001 C CNN
	1    1500 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4E0FDC
P 1500 3750
F 0 "#PWR?" H 1500 3500 50  0001 C CNN
F 1 "GND" H 1505 3577 50  0000 C CNN
F 2 "" H 1500 3750 50  0001 C CNN
F 3 "" H 1500 3750 50  0001 C CNN
	1    1500 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 2900 1300 2900
Wire Wire Line
	1500 3750 1500 3650
Wire Wire Line
	1500 3200 1500 3250
Wire Wire Line
	1500 3250 1600 3250
Connection ~ 1500 3250
Wire Wire Line
	1500 3250 1500 3350
$Comp
L Device:R R?
U 1 1 5C4E23C5
P 1500 4150
F 0 "R?" H 1570 4196 50  0000 L CNN
F 1 "1k" H 1570 4105 50  0000 L CNN
F 2 "" V 1430 4150 50  0001 C CNN
F 3 "~" H 1500 4150 50  0001 C CNN
	1    1500 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4E23CC
P 1500 4600
F 0 "C?" H 1615 4646 50  0000 L CNN
F 1 "1uF" H 1615 4555 50  0000 L CNN
F 2 "" H 1538 4450 50  0001 C CNN
F 3 "~" H 1500 4600 50  0001 C CNN
	1    1500 4600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4E23D3
P 1500 4850
F 0 "#PWR?" H 1500 4600 50  0001 C CNN
F 1 "GND" H 1505 4677 50  0000 C CNN
F 2 "" H 1500 4850 50  0001 C CNN
F 3 "" H 1500 4850 50  0001 C CNN
	1    1500 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 4000 1300 4000
Wire Wire Line
	1500 4850 1500 4750
Wire Wire Line
	1500 4300 1500 4350
Wire Wire Line
	1500 4350 1600 4350
Connection ~ 1500 4350
Wire Wire Line
	1500 4350 1500 4450
$Comp
L Device:R R?
U 1 1 5C4E3B27
P 1500 5250
F 0 "R?" H 1570 5296 50  0000 L CNN
F 1 "1k" H 1570 5205 50  0000 L CNN
F 2 "" V 1430 5250 50  0001 C CNN
F 3 "~" H 1500 5250 50  0001 C CNN
	1    1500 5250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4E3B2E
P 1500 5700
F 0 "C?" H 1615 5746 50  0000 L CNN
F 1 "1uF" H 1615 5655 50  0000 L CNN
F 2 "" H 1538 5550 50  0001 C CNN
F 3 "~" H 1500 5700 50  0001 C CNN
	1    1500 5700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4E3B35
P 1500 5950
F 0 "#PWR?" H 1500 5700 50  0001 C CNN
F 1 "GND" H 1505 5777 50  0000 C CNN
F 2 "" H 1500 5950 50  0001 C CNN
F 3 "" H 1500 5950 50  0001 C CNN
	1    1500 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 5100 1300 5100
Wire Wire Line
	1500 5950 1500 5850
Wire Wire Line
	1500 5400 1500 5450
Wire Wire Line
	1500 5450 1600 5450
Connection ~ 1500 5450
Wire Wire Line
	1500 5450 1500 5550
$Comp
L Device:R R?
U 1 1 5C4E54F6
P 1500 6350
F 0 "R?" H 1570 6396 50  0000 L CNN
F 1 "1k" H 1570 6305 50  0000 L CNN
F 2 "" V 1430 6350 50  0001 C CNN
F 3 "~" H 1500 6350 50  0001 C CNN
	1    1500 6350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4E54FD
P 1500 6800
F 0 "C?" H 1615 6846 50  0000 L CNN
F 1 "1uF" H 1615 6755 50  0000 L CNN
F 2 "" H 1538 6650 50  0001 C CNN
F 3 "~" H 1500 6800 50  0001 C CNN
	1    1500 6800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4E5504
P 1500 7050
F 0 "#PWR?" H 1500 6800 50  0001 C CNN
F 1 "GND" H 1505 6877 50  0000 C CNN
F 2 "" H 1500 7050 50  0001 C CNN
F 3 "" H 1500 7050 50  0001 C CNN
	1    1500 7050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 6200 1300 6200
Wire Wire Line
	1500 7050 1500 6950
Wire Wire Line
	1500 6500 1500 6550
Wire Wire Line
	1500 6550 1600 6550
Connection ~ 1500 6550
Wire Wire Line
	1500 6550 1500 6650
$Comp
L Device:R R?
U 1 1 5C4E8430
P 2100 850
F 0 "R?" H 2170 896 50  0000 L CNN
F 1 "1k" H 2170 805 50  0000 L CNN
F 2 "" V 2030 850 50  0001 C CNN
F 3 "~" H 2100 850 50  0001 C CNN
	1    2100 850 
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4E8437
P 2100 1300
F 0 "C?" H 2215 1346 50  0000 L CNN
F 1 "1uF" H 2215 1255 50  0000 L CNN
F 2 "" H 2138 1150 50  0001 C CNN
F 3 "~" H 2100 1300 50  0001 C CNN
	1    2100 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4E843E
P 2100 1550
F 0 "#PWR?" H 2100 1300 50  0001 C CNN
F 1 "GND" H 2105 1377 50  0000 C CNN
F 2 "" H 2100 1550 50  0001 C CNN
F 3 "" H 2100 1550 50  0001 C CNN
	1    2100 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 700  1900 700 
Wire Wire Line
	2100 1550 2100 1450
Wire Wire Line
	2100 1000 2100 1050
Wire Wire Line
	2100 1050 2200 1050
Connection ~ 2100 1050
Wire Wire Line
	2100 1050 2100 1150
$Comp
L Device:R R?
U 1 1 5C4E844A
P 2100 1950
F 0 "R?" H 2170 1996 50  0000 L CNN
F 1 "1k" H 2170 1905 50  0000 L CNN
F 2 "" V 2030 1950 50  0001 C CNN
F 3 "~" H 2100 1950 50  0001 C CNN
	1    2100 1950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4E8451
P 2100 2400
F 0 "C?" H 2215 2446 50  0000 L CNN
F 1 "1uF" H 2215 2355 50  0000 L CNN
F 2 "" H 2138 2250 50  0001 C CNN
F 3 "~" H 2100 2400 50  0001 C CNN
	1    2100 2400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4E8458
P 2100 2650
F 0 "#PWR?" H 2100 2400 50  0001 C CNN
F 1 "GND" H 2105 2477 50  0000 C CNN
F 2 "" H 2100 2650 50  0001 C CNN
F 3 "" H 2100 2650 50  0001 C CNN
	1    2100 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 1800 1900 1800
Wire Wire Line
	2100 2650 2100 2550
Wire Wire Line
	2100 2100 2100 2150
Wire Wire Line
	2100 2150 2200 2150
Connection ~ 2100 2150
Wire Wire Line
	2100 2150 2100 2250
$Comp
L Device:R R?
U 1 1 5C4E8464
P 2100 3050
F 0 "R?" H 2170 3096 50  0000 L CNN
F 1 "1k" H 2170 3005 50  0000 L CNN
F 2 "" V 2030 3050 50  0001 C CNN
F 3 "~" H 2100 3050 50  0001 C CNN
	1    2100 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4E846B
P 2100 3500
F 0 "C?" H 2215 3546 50  0000 L CNN
F 1 "1uF" H 2215 3455 50  0000 L CNN
F 2 "" H 2138 3350 50  0001 C CNN
F 3 "~" H 2100 3500 50  0001 C CNN
	1    2100 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4E8472
P 2100 3750
F 0 "#PWR?" H 2100 3500 50  0001 C CNN
F 1 "GND" H 2105 3577 50  0000 C CNN
F 2 "" H 2100 3750 50  0001 C CNN
F 3 "" H 2100 3750 50  0001 C CNN
	1    2100 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 2900 1900 2900
Wire Wire Line
	2100 3750 2100 3650
Wire Wire Line
	2100 3200 2100 3250
Wire Wire Line
	2100 3250 2200 3250
Connection ~ 2100 3250
Wire Wire Line
	2100 3250 2100 3350
$Comp
L Device:R R?
U 1 1 5C4E847E
P 2100 4150
F 0 "R?" H 2170 4196 50  0000 L CNN
F 1 "1k" H 2170 4105 50  0000 L CNN
F 2 "" V 2030 4150 50  0001 C CNN
F 3 "~" H 2100 4150 50  0001 C CNN
	1    2100 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4E8485
P 2100 4600
F 0 "C?" H 2215 4646 50  0000 L CNN
F 1 "1uF" H 2215 4555 50  0000 L CNN
F 2 "" H 2138 4450 50  0001 C CNN
F 3 "~" H 2100 4600 50  0001 C CNN
	1    2100 4600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4E848C
P 2100 4850
F 0 "#PWR?" H 2100 4600 50  0001 C CNN
F 1 "GND" H 2105 4677 50  0000 C CNN
F 2 "" H 2100 4850 50  0001 C CNN
F 3 "" H 2100 4850 50  0001 C CNN
	1    2100 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 4000 1900 4000
Wire Wire Line
	2100 4850 2100 4750
Wire Wire Line
	2100 4300 2100 4350
Wire Wire Line
	2100 4350 2200 4350
Connection ~ 2100 4350
Wire Wire Line
	2100 4350 2100 4450
$Comp
L Device:R R?
U 1 1 5C4E8498
P 2100 5250
F 0 "R?" H 2170 5296 50  0000 L CNN
F 1 "1k" H 2170 5205 50  0000 L CNN
F 2 "" V 2030 5250 50  0001 C CNN
F 3 "~" H 2100 5250 50  0001 C CNN
	1    2100 5250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4E849F
P 2100 5700
F 0 "C?" H 2215 5746 50  0000 L CNN
F 1 "1uF" H 2215 5655 50  0000 L CNN
F 2 "" H 2138 5550 50  0001 C CNN
F 3 "~" H 2100 5700 50  0001 C CNN
	1    2100 5700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4E84A6
P 2100 5950
F 0 "#PWR?" H 2100 5700 50  0001 C CNN
F 1 "GND" H 2105 5777 50  0000 C CNN
F 2 "" H 2100 5950 50  0001 C CNN
F 3 "" H 2100 5950 50  0001 C CNN
	1    2100 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 5100 1900 5100
Wire Wire Line
	2100 5950 2100 5850
Wire Wire Line
	2100 5400 2100 5450
Wire Wire Line
	2100 5450 2200 5450
Connection ~ 2100 5450
Wire Wire Line
	2100 5450 2100 5550
$Comp
L Device:R R?
U 1 1 5C4E84B2
P 2100 6350
F 0 "R?" H 2170 6396 50  0000 L CNN
F 1 "1k" H 2170 6305 50  0000 L CNN
F 2 "" V 2030 6350 50  0001 C CNN
F 3 "~" H 2100 6350 50  0001 C CNN
	1    2100 6350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4E84B9
P 2100 6800
F 0 "C?" H 2215 6846 50  0000 L CNN
F 1 "1uF" H 2215 6755 50  0000 L CNN
F 2 "" H 2138 6650 50  0001 C CNN
F 3 "~" H 2100 6800 50  0001 C CNN
	1    2100 6800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4E84C0
P 2100 7050
F 0 "#PWR?" H 2100 6800 50  0001 C CNN
F 1 "GND" H 2105 6877 50  0000 C CNN
F 2 "" H 2100 7050 50  0001 C CNN
F 3 "" H 2100 7050 50  0001 C CNN
	1    2100 7050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 6200 1900 6200
Wire Wire Line
	2100 7050 2100 6950
Wire Wire Line
	2100 6500 2100 6550
Wire Wire Line
	2100 6550 2200 6550
Connection ~ 2100 6550
Wire Wire Line
	2100 6550 2100 6650
$Comp
L Device:R R?
U 1 1 5C4EBD30
P 2650 850
F 0 "R?" H 2720 896 50  0000 L CNN
F 1 "1k" H 2720 805 50  0000 L CNN
F 2 "" V 2580 850 50  0001 C CNN
F 3 "~" H 2650 850 50  0001 C CNN
	1    2650 850 
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4EBD37
P 2650 1300
F 0 "C?" H 2765 1346 50  0000 L CNN
F 1 "1uF" H 2765 1255 50  0000 L CNN
F 2 "" H 2688 1150 50  0001 C CNN
F 3 "~" H 2650 1300 50  0001 C CNN
	1    2650 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4EBD3E
P 2650 1550
F 0 "#PWR?" H 2650 1300 50  0001 C CNN
F 1 "GND" H 2655 1377 50  0000 C CNN
F 2 "" H 2650 1550 50  0001 C CNN
F 3 "" H 2650 1550 50  0001 C CNN
	1    2650 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 700  2450 700 
Wire Wire Line
	2650 1550 2650 1450
Wire Wire Line
	2650 1000 2650 1050
Wire Wire Line
	2650 1050 2750 1050
Connection ~ 2650 1050
Wire Wire Line
	2650 1050 2650 1150
$Comp
L Device:R R?
U 1 1 5C4EBD4A
P 2650 1950
F 0 "R?" H 2720 1996 50  0000 L CNN
F 1 "1k" H 2720 1905 50  0000 L CNN
F 2 "" V 2580 1950 50  0001 C CNN
F 3 "~" H 2650 1950 50  0001 C CNN
	1    2650 1950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4EBD51
P 2650 2400
F 0 "C?" H 2765 2446 50  0000 L CNN
F 1 "1uF" H 2765 2355 50  0000 L CNN
F 2 "" H 2688 2250 50  0001 C CNN
F 3 "~" H 2650 2400 50  0001 C CNN
	1    2650 2400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4EBD58
P 2650 2650
F 0 "#PWR?" H 2650 2400 50  0001 C CNN
F 1 "GND" H 2655 2477 50  0000 C CNN
F 2 "" H 2650 2650 50  0001 C CNN
F 3 "" H 2650 2650 50  0001 C CNN
	1    2650 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 1800 2450 1800
Wire Wire Line
	2650 2650 2650 2550
Wire Wire Line
	2650 2100 2650 2150
Wire Wire Line
	2650 2150 2750 2150
Connection ~ 2650 2150
Wire Wire Line
	2650 2150 2650 2250
$Comp
L Device:R R?
U 1 1 5C4EBD64
P 2650 3050
F 0 "R?" H 2720 3096 50  0000 L CNN
F 1 "1k" H 2720 3005 50  0000 L CNN
F 2 "" V 2580 3050 50  0001 C CNN
F 3 "~" H 2650 3050 50  0001 C CNN
	1    2650 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4EBD6B
P 2650 3500
F 0 "C?" H 2765 3546 50  0000 L CNN
F 1 "1uF" H 2765 3455 50  0000 L CNN
F 2 "" H 2688 3350 50  0001 C CNN
F 3 "~" H 2650 3500 50  0001 C CNN
	1    2650 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4EBD72
P 2650 3750
F 0 "#PWR?" H 2650 3500 50  0001 C CNN
F 1 "GND" H 2655 3577 50  0000 C CNN
F 2 "" H 2650 3750 50  0001 C CNN
F 3 "" H 2650 3750 50  0001 C CNN
	1    2650 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 2900 2450 2900
Wire Wire Line
	2650 3750 2650 3650
Wire Wire Line
	2650 3200 2650 3250
Wire Wire Line
	2650 3250 2750 3250
Connection ~ 2650 3250
Wire Wire Line
	2650 3250 2650 3350
$Comp
L Device:R R?
U 1 1 5C4EBD7E
P 2650 4150
F 0 "R?" H 2720 4196 50  0000 L CNN
F 1 "1k" H 2720 4105 50  0000 L CNN
F 2 "" V 2580 4150 50  0001 C CNN
F 3 "~" H 2650 4150 50  0001 C CNN
	1    2650 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4EBD85
P 2650 4600
F 0 "C?" H 2765 4646 50  0000 L CNN
F 1 "1uF" H 2765 4555 50  0000 L CNN
F 2 "" H 2688 4450 50  0001 C CNN
F 3 "~" H 2650 4600 50  0001 C CNN
	1    2650 4600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4EBD8C
P 2650 4850
F 0 "#PWR?" H 2650 4600 50  0001 C CNN
F 1 "GND" H 2655 4677 50  0000 C CNN
F 2 "" H 2650 4850 50  0001 C CNN
F 3 "" H 2650 4850 50  0001 C CNN
	1    2650 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 4000 2450 4000
Wire Wire Line
	2650 4850 2650 4750
Wire Wire Line
	2650 4300 2650 4350
Wire Wire Line
	2650 4350 2750 4350
Connection ~ 2650 4350
Wire Wire Line
	2650 4350 2650 4450
$Comp
L Device:R R?
U 1 1 5C4EBD98
P 2650 5250
F 0 "R?" H 2720 5296 50  0000 L CNN
F 1 "1k" H 2720 5205 50  0000 L CNN
F 2 "" V 2580 5250 50  0001 C CNN
F 3 "~" H 2650 5250 50  0001 C CNN
	1    2650 5250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4EBD9F
P 2650 5700
F 0 "C?" H 2765 5746 50  0000 L CNN
F 1 "1uF" H 2765 5655 50  0000 L CNN
F 2 "" H 2688 5550 50  0001 C CNN
F 3 "~" H 2650 5700 50  0001 C CNN
	1    2650 5700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4EBDA6
P 2650 5950
F 0 "#PWR?" H 2650 5700 50  0001 C CNN
F 1 "GND" H 2655 5777 50  0000 C CNN
F 2 "" H 2650 5950 50  0001 C CNN
F 3 "" H 2650 5950 50  0001 C CNN
	1    2650 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 5100 2450 5100
Wire Wire Line
	2650 5950 2650 5850
Wire Wire Line
	2650 5400 2650 5450
Wire Wire Line
	2650 5450 2750 5450
Connection ~ 2650 5450
Wire Wire Line
	2650 5450 2650 5550
$Comp
L Device:R R?
U 1 1 5C4EBDB2
P 2650 6350
F 0 "R?" H 2720 6396 50  0000 L CNN
F 1 "1k" H 2720 6305 50  0000 L CNN
F 2 "" V 2580 6350 50  0001 C CNN
F 3 "~" H 2650 6350 50  0001 C CNN
	1    2650 6350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4EBDB9
P 2650 6800
F 0 "C?" H 2765 6846 50  0000 L CNN
F 1 "1uF" H 2765 6755 50  0000 L CNN
F 2 "" H 2688 6650 50  0001 C CNN
F 3 "~" H 2650 6800 50  0001 C CNN
	1    2650 6800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4EBDC0
P 2650 7050
F 0 "#PWR?" H 2650 6800 50  0001 C CNN
F 1 "GND" H 2655 6877 50  0000 C CNN
F 2 "" H 2650 7050 50  0001 C CNN
F 3 "" H 2650 7050 50  0001 C CNN
	1    2650 7050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 6200 2450 6200
Wire Wire Line
	2650 7050 2650 6950
Wire Wire Line
	2650 6500 2650 6550
Wire Wire Line
	2650 6550 2750 6550
Connection ~ 2650 6550
Wire Wire Line
	2650 6550 2650 6650
$Comp
L Device:R R?
U 1 1 5C4F1B96
P 3200 850
F 0 "R?" H 3270 896 50  0000 L CNN
F 1 "1k" H 3270 805 50  0000 L CNN
F 2 "" V 3130 850 50  0001 C CNN
F 3 "~" H 3200 850 50  0001 C CNN
	1    3200 850 
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4F1B9D
P 3200 1300
F 0 "C?" H 3315 1346 50  0000 L CNN
F 1 "1uF" H 3315 1255 50  0000 L CNN
F 2 "" H 3238 1150 50  0001 C CNN
F 3 "~" H 3200 1300 50  0001 C CNN
	1    3200 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4F1BA4
P 3200 1550
F 0 "#PWR?" H 3200 1300 50  0001 C CNN
F 1 "GND" H 3205 1377 50  0000 C CNN
F 2 "" H 3200 1550 50  0001 C CNN
F 3 "" H 3200 1550 50  0001 C CNN
	1    3200 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 700  3000 700 
Wire Wire Line
	3200 1550 3200 1450
Wire Wire Line
	3200 1000 3200 1050
Wire Wire Line
	3200 1050 3300 1050
Connection ~ 3200 1050
Wire Wire Line
	3200 1050 3200 1150
$Comp
L Device:R R?
U 1 1 5C4F1BB0
P 3200 1950
F 0 "R?" H 3270 1996 50  0000 L CNN
F 1 "1k" H 3270 1905 50  0000 L CNN
F 2 "" V 3130 1950 50  0001 C CNN
F 3 "~" H 3200 1950 50  0001 C CNN
	1    3200 1950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C4F1BB7
P 3200 2400
F 0 "C?" H 3315 2446 50  0000 L CNN
F 1 "1uF" H 3315 2355 50  0000 L CNN
F 2 "" H 3238 2250 50  0001 C CNN
F 3 "~" H 3200 2400 50  0001 C CNN
	1    3200 2400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4F1BBE
P 3200 2650
F 0 "#PWR?" H 3200 2400 50  0001 C CNN
F 1 "GND" H 3205 2477 50  0000 C CNN
F 2 "" H 3200 2650 50  0001 C CNN
F 3 "" H 3200 2650 50  0001 C CNN
	1    3200 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 1800 3000 1800
Wire Wire Line
	3200 2650 3200 2550
Wire Wire Line
	3200 2100 3200 2150
Wire Wire Line
	3200 2150 3300 2150
Connection ~ 3200 2150
Wire Wire Line
	3200 2150 3200 2250
Text Label 1600 1050 0    50   ~ 0
CH1
Text Label 1600 2150 0    50   ~ 0
CH2
Text Label 1600 3250 0    50   ~ 0
CH3
Text Label 1600 4350 0    50   ~ 0
CH4
Text Label 1600 5450 0    50   ~ 0
CH5
Text Label 1600 6550 0    50   ~ 0
CH6
Text Label 2200 1050 0    50   ~ 0
CH7
Text Label 2200 2150 0    50   ~ 0
CH8
Text Label 2200 3250 0    50   ~ 0
CH9
Text Label 2200 4350 0    50   ~ 0
CH10
Text Label 2200 5450 0    50   ~ 0
CH11
Text Label 2200 6550 0    50   ~ 0
CH12
Text Label 2750 1050 0    50   ~ 0
CH13
Text Label 2750 2150 0    50   ~ 0
CH14
Text Label 2750 3250 0    50   ~ 0
CH15
Text Label 2750 4350 0    50   ~ 0
CH16
Text Label 2750 5450 0    50   ~ 0
CH17
Text Label 2750 6550 0    50   ~ 0
CH18
Text Label 3300 1050 0    50   ~ 0
CH19
Text Label 3300 2150 0    50   ~ 0
CH20
$Comp
L Connector_Generic:Conn_02x10_Counter_Clockwise J?
U 1 1 5C4FB2E5
P 10000 5050
F 0 "J?" H 10050 5667 50  0000 C CNN
F 1 "Conn_02x10_Counter_Clockwise" H 10050 5576 50  0000 C CNN
F 2 "" H 10000 5050 50  0001 C CNN
F 3 "~" H 10000 5050 50  0001 C CNN
	1    10000 5050
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J?
U 1 1 5C4FB7AE
P 10000 5750
F 0 "J?" H 10080 5742 50  0000 L CNN
F 1 "Conn_01x02" H 10080 5651 50  0000 L CNN
F 2 "" H 10000 5750 50  0001 C CNN
F 3 "~" H 10000 5750 50  0001 C CNN
	1    10000 5750
	1    0    0    -1  
$EndComp
Text Label 9800 5750 2    50   ~ 0
5V
Text Label 9800 5850 2    50   ~ 0
GND
Text Label 9700 4650 2    50   ~ 0
CH1_LPF
Text Label 9700 4750 2    50   ~ 0
CH2_LPF
Text Label 9700 4850 2    50   ~ 0
CH3_LPF
Text Label 9700 4950 2    50   ~ 0
CH4_LPF
Text Label 9700 5050 2    50   ~ 0
CH5_LPF
Text Label 9700 5150 2    50   ~ 0
CH6_LPF
Text Label 9700 5250 2    50   ~ 0
CH7_LPF
Text Label 9700 5350 2    50   ~ 0
CH8_LPF
Text Label 9700 5450 2    50   ~ 0
CH9_LPF
Text Label 9700 5550 2    50   ~ 0
CH10_LPF
Text Label 10400 5550 0    50   ~ 0
CH11_LPF
Text Label 10400 5450 0    50   ~ 0
CH12_LPF
Text Label 10400 5350 0    50   ~ 0
CH13_LPF
Text Label 10400 5150 0    50   ~ 0
CH15_LPF
Text Label 10400 5250 0    50   ~ 0
CH14_LPF
Text Label 10400 5050 0    50   ~ 0
CH16_LPF
Text Label 10400 4950 0    50   ~ 0
CH17_LPF
Text Label 10400 4850 0    50   ~ 0
CH18_LPF
Text Label 10400 4750 0    50   ~ 0
CH19_LPF
Text Label 10400 4650 0    50   ~ 0
CH20_LPF
$Comp
L Connector_Generic:Conn_02x06_Counter_Clockwise J?
U 1 1 5C4FF29C
P 10050 3750
F 0 "J?" H 10100 4167 50  0000 C CNN
F 1 "Conn_02x06_Counter_Clockwise" H 10100 4076 50  0000 C CNN
F 2 "" H 10050 3750 50  0001 C CNN
F 3 "~" H 10050 3750 50  0001 C CNN
	1    10050 3750
	1    0    0    -1  
$EndComp
$Comp
L power:+5VL #PWR?
U 1 1 5C5003C2
P 9450 3400
F 0 "#PWR?" H 9450 3250 50  0001 C CNN
F 1 "+5VL" H 9465 3573 50  0000 C CNN
F 2 "" H 9450 3400 50  0001 C CNN
F 3 "" H 9450 3400 50  0001 C CNN
	1    9450 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	9450 3400 9450 3550
Wire Wire Line
	9450 3550 9850 3550
Wire Wire Line
	9850 3650 9700 3650
Wire Wire Line
	9850 3750 9700 3750
Wire Wire Line
	9850 3850 9700 3850
Wire Wire Line
	9850 3950 9700 3950
Wire Wire Line
	9850 4050 9700 4050
Wire Wire Line
	9700 4050 9700 3950
Wire Wire Line
	9700 4050 9700 4100
Connection ~ 9700 4050
$Comp
L power:GND #PWR?
U 1 1 5C52D6FF
P 9700 4100
F 0 "#PWR?" H 9700 3850 50  0001 C CNN
F 1 "GND" H 9705 3927 50  0000 C CNN
F 2 "" H 9700 4100 50  0001 C CNN
F 3 "" H 9700 4100 50  0001 C CNN
	1    9700 4100
	1    0    0    -1  
$EndComp
Text Label 9700 3650 2    50   ~ 0
SCK
Text Label 9700 3750 2    50   ~ 0
MOSI
Text Label 9700 3850 2    50   ~ 0
MISO
Text Label 10450 4050 0    50   ~ 0
CS1
Text Label 10450 3950 0    50   ~ 0
CS2
Wire Wire Line
	10350 3850 10450 3850
Wire Wire Line
	10350 3750 10450 3750
Wire Wire Line
	10350 3650 10450 3650
Wire Wire Line
	10350 3550 10450 3550
Text Label 10450 3850 0    50   ~ 0
CS3
Text Label 10450 3750 0    50   ~ 0
CS4
Text Label 10450 3650 0    50   ~ 0
CS5
Text Label 10450 3550 0    50   ~ 0
CS6
Wire Wire Line
	10350 3950 10450 3950
Wire Wire Line
	10350 4050 10450 4050
Wire Wire Line
	9800 4650 9700 4650
Wire Wire Line
	9700 4750 9800 4750
Wire Wire Line
	9800 4850 9700 4850
Wire Wire Line
	9700 4950 9800 4950
Wire Wire Line
	9800 5050 9700 5050
Wire Wire Line
	9700 5150 9800 5150
Wire Wire Line
	9800 5250 9700 5250
Wire Wire Line
	9700 5350 9800 5350
Wire Wire Line
	9800 5450 9700 5450
Wire Wire Line
	9700 5550 9800 5550
Wire Wire Line
	10300 4650 10400 4650
Wire Wire Line
	10400 4750 10300 4750
Wire Wire Line
	10300 4850 10400 4850
Wire Wire Line
	10400 4950 10300 4950
Wire Wire Line
	10300 5050 10400 5050
Wire Wire Line
	10400 5150 10300 5150
Wire Wire Line
	10300 5250 10400 5250
Wire Wire Line
	10400 5350 10300 5350
Wire Wire Line
	10300 5450 10400 5450
Wire Wire Line
	10400 5550 10300 5550
Text Notes 9650 650  0    50   ~ 0
CT2196MST-ND  : 6 position dip switch
Text Label 1300 700  2    50   ~ 0
CH1_LPF
Text Label 1300 1800 2    50   ~ 0
CH2_LPF
Text Label 1300 2900 2    50   ~ 0
CH3_LPF
Text Label 1300 4000 2    50   ~ 0
CH4_LPF
Text Label 1300 5100 2    50   ~ 0
CH5_LPF
Text Label 1300 6200 2    50   ~ 0
CH6_LPF
Text Label 1900 700  2    50   ~ 0
CH7_LPF
Text Label 1900 1800 2    50   ~ 0
CH8_LPF
Text Label 1900 2900 2    50   ~ 0
CH9_LPF
Text Label 1900 4000 2    50   ~ 0
CH10_LPF
Text Label 1900 5100 2    50   ~ 0
CH11_LPF
Text Label 1900 6200 2    50   ~ 0
CH12_LPF
Text Label 2450 700  2    50   ~ 0
CH13_LPF
Text Label 2450 1800 2    50   ~ 0
CH14_LPF
Text Label 2450 2900 2    50   ~ 0
CH15_LPF
Text Label 2450 4000 2    50   ~ 0
CH16_LPF
Text Label 2450 5100 2    50   ~ 0
CH17_LPF
Text Label 2450 6200 2    50   ~ 0
CH18_LPF
Text Label 3000 700  2    50   ~ 0
CH19_LPF
Text Label 3000 1800 2    50   ~ 0
CH20_LPF
Text Label 5650 2400 2    50   ~ 0
CH1
Text Label 5650 2500 2    50   ~ 0
CH2
Text Label 5650 2600 2    50   ~ 0
CH3
Text Label 5650 2700 2    50   ~ 0
CH4
Text Label 5650 2800 2    50   ~ 0
CH5
Text Label 5650 2900 2    50   ~ 0
CH6
Text Label 5650 3000 2    50   ~ 0
CH7
Text Label 5650 3100 2    50   ~ 0
CH8
Text Label 5650 3200 2    50   ~ 0
CH9
Text Label 5650 3300 2    50   ~ 0
CH10
Text Label 5650 3400 2    50   ~ 0
CH11
Text Label 5650 3500 2    50   ~ 0
CH12
Text Label 5650 3600 2    50   ~ 0
CH13
Text Label 5650 3700 2    50   ~ 0
CH14
Text Label 5650 3800 2    50   ~ 0
CH15
Text Label 5650 3900 2    50   ~ 0
CH16
Text Label 5650 4000 2    50   ~ 0
CH17
Text Label 5650 4100 2    50   ~ 0
CH18
Text Label 5650 4200 2    50   ~ 0
CH19
Text Label 5650 4300 2    50   ~ 0
CH20
Wire Wire Line
	5650 2400 5800 2400
Wire Wire Line
	5650 2500 5800 2500
Wire Wire Line
	5800 2600 5650 2600
Wire Wire Line
	5650 2700 5800 2700
Wire Wire Line
	5800 2800 5650 2800
Wire Wire Line
	5650 2900 5800 2900
Wire Wire Line
	5800 3000 5650 3000
Wire Wire Line
	5650 3100 5800 3100
Wire Wire Line
	5800 3200 5650 3200
Wire Wire Line
	5650 3300 5800 3300
Wire Wire Line
	5800 3400 5650 3400
Wire Wire Line
	5650 3500 5800 3500
Wire Wire Line
	5650 3600 5800 3600
Wire Wire Line
	5650 3700 5800 3700
Wire Wire Line
	5650 3800 5800 3800
Wire Wire Line
	5800 3900 5650 3900
Wire Wire Line
	5650 4000 5800 4000
Wire Wire Line
	5800 4100 5650 4100
Wire Wire Line
	5650 4200 5800 4200
Wire Wire Line
	5800 4300 5650 4300
Text Label 7550 4200 0    50   ~ 0
MISO
Text Label 7550 4300 0    50   ~ 0
MOSI
Text Label 7550 4400 0    50   ~ 0
SCK
Wire Wire Line
	7400 4200 7550 4200
Wire Wire Line
	7550 4300 7400 4300
Wire Wire Line
	7400 4400 7550 4400
$EndSCHEMATC
