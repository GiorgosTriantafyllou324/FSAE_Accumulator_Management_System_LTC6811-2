#include <LTC6811-board.h>


/* ----------- Lookup table of the NTC used in the BMS: NXFS15XH103FEAB025 --------------- *
 * Includes the resistance (in kOhm) of the NTC from 0 degrees C to 100 degrees with 0.5 degree step */
const float NTC_LUT[NTC_LUT_LENGTH] = {27.2186, 26.6403, 26.0760, 25.5252, 24.9877, 24.4630, 23.9509, 23.4510, 22.9629, 22.4863,
		   	   	   	   	   	     	   22.0211, 21.5667, 21.1230, 20.6897, 20.2666, 19.8530, 19.4495, 19.0550, 18.6698, 18.2933,
									   17.9255, 17.5656, 17.2139, 16.8702, 16.5344, 16.2063, 15.8856, 15.5721, 15.2658, 14.9663,
									   14.6735, 14.3873, 14.1075, 13.8339, 13.5664, 13.3048, 13.0489, 12.7987, 12.5540, 12.3147,
									   12.0805, 11.8519, 11.6281, 11.4091, 11.1947, 10.9849, 10.7795, 10.5784, 10.3815, 10.1887,
									   10.0000,  9.8152,  9.6342,  9.4570,  9.2835,  9.1135,  8.9470,  8.7840,  8.6242,  8.4678,
									    8.3145,  8.1648,  8.0181,  7.8744,  7.7337,  7.5959,  7.4609,  7.3287,  7.1991,  7.0722,
										6.9479,  6.8261,  6.7067,  6.5897,  6.4751,  6.3627,  6.2526,  6.1447,  6.0390,  5.9353,
										5.8336,  5.7337,  5.6357,  5.5396,  5.4454,  5.3530,  5.2623,  5.1734,  5.0863,  5.0008,
										4.9169,  4.8346,  4.7539,  4.6748,  4.5971,  4.5209,  4.4461,  4.3728,  4.3008,  4.2302,
										4.1609,  4.0929,  4.0262,  3.9607,  3.8964,  3.8334,  3.7714,  3.7107,  3.6510,  3.5924,
										3.5350,  3.4785,  3.4231,  3.3687,  3.3152,  3.2628,  3.2113,  3.1607,  3.1110,  3.0622,
										3.0143,  2.9679,  2.9224,  2.8777,  2.8337,  2.7906,  2.7482,  2.7066,  2.6657,  2.6256,
										2.5861,  2.5474,  2.5093,  2.4719,  2.4351,  2.3990,  2.3635,  2.3286,  2.2943,  2.2607,
										2.2275,  2.1949,  2.1627,  2.1311,  2.1001,  2.0696,  2.0396,  2.0101,  1.9811,  1.9526,
										1.9245,  1.8970,  1.8698,  1.8432,  1.8170,  1.7912,  1.7658,  1.7409,  1.7164,  1.6923,
										1.6685,  1.6453,  1.6224,  1.5999,  1.5777,  1.5559,  1.5345,  1.5134,  1.4927,  1.4722,
										1.4521,  1.4324,  1.4129,  1.3938,  1.3749,  1.3564,  1.3381,  1.3202,  1.3025,  1.2851,
										1.2680,  1.2510,  1.2343,  1.2178,  1.2016,  1.1857,  1.1700,  1.1545,  1.1393,  1.1243,
										1.1096,  1.0950,  1.0807,  1.0666,  1.0528,  1.0391,  1.0256,  1.0124,  0.9993,  0.9865,
										0.9738};

/* Holds the expected voltage across the NTC for a given temperature, knowing the Vref and the Constant Resistor value in the voltage divider */
float NTC_voltage[NTC_LUT_LENGTH];

/* Constant array where the value of each array cell is between 1 and CELLS_NUM * SLAVES_NUM.
 * ntc_to_cell_position[i][j] contains the position of the battery cell pair (1 to 144) whose
 * temperature is being monitored by the j-th NTC (1 to NTCS_NUM) in the i-th slave (1 to SLAVES_NUM)
 * Since a single NTC is measuring the temperatures of 2 cells, (one positive pole and one negative)
 * every NTC is matched to the position of the negative pole cell.
 * If the LTC6811 has a humidity sensor instead of an NTC, the value 0xFF is written in the respective position
 * */
const uint8_t ntc_to_cell_position[SLAVES_NUM][NTCS_NUM] = {{1,   3,   4,   6,      9},  // Fisrt  slave. 5 NTCs, no  RH sensor
														    {12,  15,  16,  18,  0xFF},  // Second slave. 4 NTCs, 1   RH sensor
															{19,  21,  22,  24,    27},
															{30,  33,  34,  36,  0xFF},
															{37,  39,  40,  42,    45},
															{48,  51,  52,  54,  0xFF},
															{55,  57,  58,  60,    63},
															{66,  69,  70,  72,  0xFF},
															{73,  75,  78,  81,  0xFF},
															{84,  85,  87,  88,    90},
															{91,  93,  96,  99,  0xFF},
															{102, 103, 105, 106,  108},
															{109, 111, 114, 117, 0xFF},
															{120, 121, 123, 124,  126},
															{127, 129, 132, 135, 0xFF},
															{138, 139, 141, 142,  144}};



