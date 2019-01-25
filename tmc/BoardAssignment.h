#ifndef BOARD_ASSIGNMENT_H
#define BOARD_ASSIGNMENT_H

#include "boards/Board.h"

	typedef enum {
		FOUND_BY_NONE,
		FOUND_BY_MONOFLOP,
		FOUND_BY_EEPROM
	} IDFinder;

	typedef struct
	{
		uint8 state;          // detection state of this board
		uint8 id;             // id of board
		IDFinder detectedBy;  // Holds the method used to detect the ID (Monoflop or EEPROM)
		uint32 counter_1;     // Timer cycles elapsed on ID pulse rising edge
		uint32 counter_2;     // Timer cycles elapsed on ID pulse falling edge
		uint32 timer_1;       // Current timer value on ID pulse rising edge
		uint32 timer_2;       // Current timer value on ID pulse falling edge
	} IdStateTypeDef;         // interface for id and detection state of a board

	typedef struct
	{
		IdStateTypeDef ch1;  // interface for id and detection state for the driver board
		IdStateTypeDef ch2;  // interface for id and detection state for the motion controller board
	} IdAssignmentTypeDef;   // interface for id and detection state of driver and motion controller board

	IdAssignmentTypeDef IdState;

	int32 Board_assign(IdAssignmentTypeDef *ids);     // ids and states of assigned driver and motion controller board
	int32 Board_supported(IdAssignmentTypeDef *ids);  // ids and states of supported driver and motion controller board

	#include "boards/SelfTest.h"

	// ids for channel 0
	#define ID_ERROR           0
	#define ID_TMC5031         2
	#define ID_TMC4361         4
	#define ID_TMC5130         5
	#define ID_TMC5041         6
	#define ID_TMC5072         7
	#define ID_TMC4670         9
	#define ID_TMC4331         10
	#define ID_TMC4361A        11
	#define ID_TMC8690         12
	#define ID_TMC4671         13
	#define ID_TMC4330         15
	#define ID_TMC5160         16
	#define ID_TMC4672         17
	#define ID_TMC5161         18
	#define ID_TMC7531         19
	#define ID_TMC5062         25
	#define ID_TMC8460         8
	#define ID_TMC8461         26
	#define ID_TMC8462         27
	#define ID_TMC2130_TQFP48  0xFE
	#define ID_SELFTEST        255

	// ids for channel 1
	#define ID_TMC2660         1
	#define ID_TMC2130         3
	#define ID_TMC2100         4
	#define ID_TMC2041         5
	#define ID_TMC2208         6
	#define ID_TMC2224         7
	#define ID_TMC2209         8
	#define ID_TMCC160         9
	#define ID_TMC6200        10
	#define ID_TMC2160        11
	#define ID_TMC262_1420    12
	#define ID_TMC2590        13
	#define ID_TMCMinion      14

	// init() functions for all boards - function definitions are in the respective _eval file of a chip
	extern void TMCC160_init();
	extern void TMC2041_init();
	extern void TMC2100_init();
	extern void TMC2130_init();
	extern void TMC2160_init();
	extern void TMC2208_init();
	extern void TMC2209_init();
	extern void TMC2224_init();
	extern void TMC2660_init();
	extern void TMC262_1420_init();
	extern void TMC2590_init();
	extern void TMCMinion_init();
	extern void TMC4330_init();
	extern void TMC4331_init();
	extern void TMC4361_init();
	extern void TMC4361A_init();
	extern void TMC4670_init();
	extern void TMC4671_init();
	extern void TMC4672_init();
	extern void TMC5031_init();
	extern void TMC5041_init();
	extern void TMC5062_init();
	extern void TMC5072_init();
	extern void TMC5130_init();
	extern void TMC5160_init();
	extern void TMC5161_init();
	extern void TMC6200_init();
	extern void TMC7531_init();
	extern void TMC8461_init_ch1();
	extern void TMC8461_init_ch2();
	extern void TMC8462_init_ch1();
	extern void TMC8462_init_ch2();
	extern void TMC8690_init();
	extern void SelfTest_init();

	typedef struct {
		uint16 id;
		void (*init)(void);
	} init_assignment;

	static const init_assignment init_ch1[] =
	{
		{ .id = ID_TMC5031,     .init = TMC5031_init     },
		{ .id = ID_TMC4361,     .init = TMC4361_init     },
		{ .id = ID_TMC5130,     .init = TMC5130_init     },
		{ .id = ID_TMC5041,     .init = TMC5041_init     },
		{ .id = ID_TMC5072,     .init = TMC5072_init     },
		{ .id = ID_TMC4670,     .init = TMC4670_init     },
		{ .id = ID_TMC4331,     .init = TMC4331_init     },
		{ .id = ID_TMC4361A,    .init = TMC4361A_init    },
		{ .id = ID_TMC8690,     .init = TMC8690_init     },
		{ .id = ID_TMC4671,     .init = TMC4671_init     },
		{ .id = ID_TMC4330,     .init = TMC4330_init     },
		{ .id = ID_TMC5160,     .init = TMC5160_init     },
		{ .id = ID_TMC4672,     .init = TMC4672_init     },
		{ .id = ID_TMC5161,     .init = TMC5161_init     },
		{ .id = ID_TMC7531,     .init = TMC7531_init     },
		{ .id = ID_TMC5062,     .init = TMC5062_init     },
		{ .id = ID_TMC8461,     .init = TMC8461_init_ch1 },
		{ .id = ID_TMC8462,     .init = TMC8462_init_ch1 },
		{ .id = ID_SELFTEST,    .init = SelfTest_init    }
	};

	static const init_assignment init_ch2[] =
	{
		{ .id = ID_TMC2660,     .init = TMC2660_init     },
		{ .id = ID_TMC2130,     .init = TMC2130_init     },
		{ .id = ID_TMC2100,     .init = TMC2100_init     },
		{ .id = ID_TMC2041,     .init = TMC2041_init     },
		{ .id = ID_TMC2208,     .init = TMC2208_init     },
		{ .id = ID_TMC2224,     .init = TMC2224_init     },
		{ .id = ID_TMC2209,     .init = TMC2209_init     },
		{ .id = ID_TMCC160,     .init = TMCC160_init     },
		{ .id = ID_TMC6200,     .init = TMC6200_init     },
		{ .id = ID_TMC2160,     .init = TMC2160_init     },
		{ .id = ID_TMC262_1420, .init = TMC262_1420_init },
		{ .id = ID_TMC2590,     .init = TMC2590_init     },
		{ .id = ID_TMCMinion,   .init = TMCMinion_init   }
	};

#endif /* BOARD_ASSIGNMENT_H */