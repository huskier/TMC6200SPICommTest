
#include "boards/Board.h"
#include "hal/derivative.h"
#include "hal/HAL.h"
#include "tmc/IdDetection.h"
#include "tmc/TMCL.h"
#include "tmc/VitalSignsMonitor.h"
#include "tmc/BoardAssignment.h"

#include "TMC-API/tmc/ic/TMC6200/TMC6200.h"

#include "tmc/EEPROM.h"

const char *VersionString = MODULE_ID"V306"; // module id and version of the firmware shown in the TMCL-IDE

static SPIChannelTypeDef *EEPROM_SPIChannel;

static SPIChannelTypeDef *TMC6200_SPIChannel;


/* Keep as is! This lines are important for the update functionality. */
#if defined(Landungsbruecke)
	const uint8_t Protection[] __attribute__ ((section(".cfmconfig")))=
	{
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  //Backdoor key
		0xFF, 0xFF, 0xFF, 0xFF,                          //Flash protection (FPPROT)
		0x7E,                                            //Flash security   (FSEC) => nach Image-Generierung manuell auf 0x40 setzen im Image
		0xF9,                                            //Flash option     (FOPT) (NMI ausgeschaltet, EzPort ausgeschaltet, Normal power)
		0xFF,                                            //reserved
		0xFF                                             //reserved
	};

	__attribute__ ((section(".bldata"))) uint32_t BLMagic;
#endif


static void TMC6200_readRegister(uint8_t motor, uint8_t address, int32_t *value);


/* Check if jumping into bootloader is forced                                           */
/*                                                                                      */
/* In order to jump to bootloader e.g. because of an accidental infinite loop           */
/* in a modified firmware you may short ID_CLK and ID_CH0 pins on start up.             */
/* This will force the entrance into bootloader mode and allow to replace bad firmware. */
void shallForceBoot()
{
	// toggle each pin and see if you can read the state on the other
	// leave if not, because this means that the pins are not tied together
	HAL.IOs->config->toOutput(&HAL.IOs->pins->ID_CLK);
	HAL.IOs->config->toInput(&HAL.IOs->pins->ID_CH0);

	HAL.IOs->config->setHigh(&HAL.IOs->pins->ID_CLK);
	if(!HAL.IOs->config->isHigh(&HAL.IOs->pins->ID_CH0))
		return;

	HAL.IOs->config->setLow(&HAL.IOs->pins->ID_CLK);
	if(HAL.IOs->config->isHigh(&HAL.IOs->pins->ID_CH0))
		return;

	HAL.IOs->config->toOutput(&HAL.IOs->pins->ID_CH0);
	HAL.IOs->config->toInput(&HAL.IOs->pins->ID_CLK);

	HAL.IOs->config->setHigh(&HAL.IOs->pins->ID_CH0);
	if(!HAL.IOs->config->isHigh(&HAL.IOs->pins->ID_CLK))
		return;

	HAL.IOs->config->setLow(&HAL.IOs->pins->ID_CH0);
	if(HAL.IOs->config->isHigh(&HAL.IOs->pins->ID_CLK))
		return;

	// not returned, this means pins are tied together
	tmcl_boot();
}

/* Call all standard initialization routines. */
static void init()
{
	HAL.init();                  // Initialize Hardware Abstraction Layer
	IDDetection_init();          // Initialize board detection
	tmcl_init();                 // Initialize TMCL communication

/*
	tmcdriver_init();            // Initialize dummy driver board --> preset EvalBoards.ch2
	tmcmotioncontroller_init();  // Initialize dummy motion controller board  --> preset EvalBoards.ch1

	VitalSignsMonitor.busy = 1;  // Put state to busy
	Evalboards.driverEnable = DRIVER_ENABLE;
	Evalboards.ch1.id = 0;       // preset id for driver board to 0 --> error/not found
	Evalboards.ch2.id = 0;       // preset id for driver board to 0 --> error/not found

	// We disable the drivers before configurating anything
	HAL.IOs->config->toOutput(&HAL.IOs->pins->DIO0);
	HAL.IOs->config->setHigh(&HAL.IOs->pins->DIO0);

	IdAssignmentTypeDef ids;
	IDDetection_initialScan(&ids);  // start initial board detection
	IDDetection_initialScan(&ids);  // start second time, first time not 100% reliable, not sure why - too fast after startup?
	if(!ids.ch1.id && !ids.ch2.id)
	{
		shallForceBoot();           // only checking to force jump into bootloader if there are no boards attached
		// todo CHECK 2: Workaround: shallForceBoot() changes pin settings - change them again here, since otherwise IDDetection partially breaks (LH)
		HAL.IOs->config->toOutput(&HAL.IOs->pins->ID_CLK);
		HAL.IOs->config->toInput(&HAL.IOs->pins->ID_CH0);
	}
	Board_assign(&ids);             // assign boards with detected id

	VitalSignsMonitor.busy 	= 0;    // not busy any more!
*/
}

static void TMC6200_writeRegister(uint8_t motor, uint8_t address, int32_t value)
{
	UNUSED(motor);
	tmc6200_writeInt(0, address, value);
}

static void TMC6200_readRegister(uint8_t motor, uint8_t address, int32_t *value)
{
	UNUSED(motor);
	*value = tmc6200_readInt(0, address);
}

static void enableDriverInMain(DriverState state)
{
	UNUSED(state);
}

void TMC6200_init_InMain(void)
{
	TMC6200_SPIChannel = &HAL.SPI->ch2;
	TMC6200_SPIChannel->CSN = &HAL.IOs->pins->SPI2_CSN0;
/*
	Evalboards.ch2.config->reset        = reset;
	Evalboards.ch2.config->restore      = restore;
	Evalboards.ch2.config->state        = CONFIG_RESET;
	Evalboards.ch2.config->configIndex  = 0;

	Evalboards.ch2.rotate               = rotate;
	Evalboards.ch2.right                = right;
	Evalboards.ch2.left                 = left;
	Evalboards.ch2.stop                 = stop;
	Evalboards.ch2.GAP                  = GAP;
	Evalboards.ch2.SAP                  = SAP;
	Evalboards.ch2.moveTo               = moveTo;
	Evalboards.ch2.moveBy               = moveBy;
	Evalboards.ch2.writeRegister        = writeRegister;
	Evalboards.ch2.readRegister         = readRegister;
	Evalboards.ch2.periodicJob          = periodicJob;
	Evalboards.ch2.userFunction         = userFunction;
	Evalboards.ch2.getMeasuredSpeed     = getMeasuredSpeed;
	Evalboards.ch2.enableDriver         = enableDriver;
	Evalboards.ch2.checkErrors          = checkErrors;
	Evalboards.ch2.numberOfMotors       = TMC6200_MOTORS;
	Evalboards.ch2.VMMin                = VM_MIN;
	Evalboards.ch2.VMMax                = VM_MAX;
	Evalboards.ch2.deInit               = deInit;
*/

	// set default PWM configuration for evaluation board use with TMC467x-EVAL
	tmc6200_writeInt(0, TMC6200_GCONF, 0x0);

	enableDriverInMain(DRIVER_USE_GLOBAL_ENABLE);
}


/* main function */
int main(void)
{
	// Start all initialization routines
	init();

	int32_t value = 0;
	int32_t value1 = 0;
	uint8_t readVal;

	txTest(0x32);

	TMC6200_SPIChannel = &SPI.ch2;
	TMC6200_SPIChannel->CSN = &HAL.IOs->pins->SPI2_CSN0;

	wait(1000);


	while(1)
	{
		TMC6200_readRegister(0, 0x04, &value);
		readVal = (value & 0xFF000000) >> 24;

		wait(1000);
		txTest(readVal);
	}


/*	Testing EEPROM......

	txTest(0x35);
	//EEPROM_SPIChannel = &HAL.SPI->ch2;
	EEPROM_SPIChannel = &SPI.ch2;

	eeprom_init(EEPROM_SPIChannel);

	wait(1000);

	readVal = eeprom_check(EEPROM_SPIChannel);

	txTest(readVal);

	// Main loop
	while(1)
	{
		// Check all parameters and life signs and mark errors
		//vitalsignsmonitor_checkVitalSigns();

		eeprom_write_byte(EEPROM_SPIChannel, 1025, 0x38);
		wait(1000);
		readVal = eeprom_read_byte(EEPROM_SPIChannel, 1025);

		wait(1000);
		txTest(readVal);

		// Perodic jobs of Motion controller/Driver boards
		//Evalboards.ch1.periodicJob(systick_getTick());
		//Evalboards.ch2.periodicJob(systick_getTick());

		// Process TMCL communication
		//tmcl_process();
	}
	*/

	return 0;
}
