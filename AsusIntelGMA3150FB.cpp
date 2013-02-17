////////////////////////////////////////////////////////////////////
//          File: AsusIntelGMA3150FB.cpp
//        Author: Gerald Leung
// Creation Date: Thu Dec 27 17:15:02 PST 2012
//      Copyright 2012, Gerald Leung, All Rights Reserved
//       Purpose: AsusIntelGMA3150FB implementation
////////////////////////////////////////////////////////////////////

// Interface includes
#include	"AsusIntelGMA3150FB.h"

// Implementation includes


/* Discussion on EeePC Displays
 *
 * Displays are the second deviceID in the Intel graphics hardware pipeline
 * e.g.
 * VenID	DevID
 * 8086		a001	Intel Atom D4xx/D5xx/N4xx/N5xx GMA3150 IGPU [PCI 0:2:0]
 *					PCI device class 0x030000
 * 8086		a002	display thing [PCI 0:2:1]
 *					PCI device class 0x038000
 *
 * Device IDs for other similar Intel GPUs can be found on:
 * http://en.wikipedia.org/wiki/Comparison_of_Intel_graphics_processing_units
 *
 * Note:	It appears the pipeline thus needs two drivers:
 *			1) custom display driver
 *				(inherits from IODisplay;
 *				protypical e.g. class ApplePanelA, AppleIntelPanelA, ...
 *				in AppleOnboardDisplay.kext or AppleBacklight.kext)
 *			2) custom framebuffer driver
 *				(inherits from IOFramebuffer -> IOGraphicsDevice;
 *				prototypical e.g. class AppleIntelGMAX3100FB,
 *				AppleIntelIntegratedFramebuffer in their respective kexts)
 *
 * NOTE:	It appears that the IOFramebuffer class is the "lead" class in the
 *			in the OSX graphics driver stack, NOT the Intel915 (GMA950 series)
 *			or the Intel965 (GMAX3100 series) classes.  More precisely it is
 *			their subclasses specific to the hardware in question that have
 *			these relationships.
 *
 *	This is to say the architectural relationship is:
 *	class					parent				kext/plugin/bundle
 *	AppleIntelGMAX3100FB	IOFramebuffer		AppleIntelGMAX3100FB
 *	[somehow creates connections/relationships to..]
 *	Intel965				IOAccelerator		AppleIntelGMAX3100
 *	[static functions linked by func ptrs]		AppleIntelGMAX3100GA
 *	Intel965GLContext		???					AppleIntelGMAX3100GLDriver
 *
 *			The Framebuffer class is the first to match, connects to other
 *			classes or functions (for acceleration), and pretty much does it
 *			all, including linking somehow to the relevant IODisplay class.
 *
 *			Also, getting the attached display (for laptops, expecially) is
 *			done through the Framebuffer class's methods, i.e. getting the
 *			display's EDID info (informing of supported, and native,
 *			resolutions) is done via DDC call through the I2C interface (or
 *			is faked through code that emulates DDC to the caller--it also
 *			appears to ignore the standard _DDC methods for graphics devices
 *			in the ACPI DSDT tables, because the EDIDs for the displays
 *			Apple uses and supports appear to all be softcoded in Apple's
 *			display kext configurations or overrides).
 *
 */

// Create an IODisplay driver class for these:
// Hannstar Displays 12.1"
//class		AsusHannstarDisplay; // Hannstar HSD121PHW-A03	(EeePC 1215N)
// need EDIDs for displays (extracted using get-EDID.exe 128-byte EDID 1.3)


//////////////////////////////
// (16) Overrides	IOService
//////////////////////////////
#define		SUPER					IOService

			OSDefineMetaClassAndStructors		(AsusIntelGMA3150FB, IOService);

bool		AsusIntelGMA3150FB::requestTerminate(IOService* provider, IOOptionBits options)
{
	return	true;
}

//////////////////////////////
// Overrides	
//////////////////////////////
// log, enable PCI mem, and register
bool		AsusIntelGMA3150FB::start			(IOService *provider)
{
	IOLog("%s::start\n", getName());

	if (!SUPER::start(provider))
		return	false;

	pPCIDevice	= OSDynamicCast(IOPCIDevice, provider);
	if (!pPCIDevice)
		return	false;

	pPCIDevice->setMemoryEnable(true);

	registerService();

	return	true;
}

void		AsusIntelGMA3150FB::stop			(IOService *provider)
{
	IOLog("%s::stop\n", getName());
	//pPCIDevice->setMemoryEnable(false);
	SUPER::stop(provider);
}

IOReturn	AsusIntelGMA3150FB::open			()
{
	return	kIOReturnSuccess;
}
    
void		AsusIntelGMA3150FB::close			()
{

}

void		AsusIntelGMA3150FB::free			()
{

}

/*
IOWorkLoop*	AsusIntelGMA3150FB::getWorkLoop		() const
{
	// use these in getWorkLoop?
	//IOWorkLoop* SUPER::getGraphicsSystemWorkLoop() const;
	//IOWorkLoop* SUPER::getControllerWorkLoop() const;
	return	SUPER::getWorkLoop();
	//return	SUPER::getGraphicsSystemWorkLoop();
}


IOReturn	AsusIntelGMA3150FB::requestProbe	(IOOptionBits options)
{
	return	kIOReturnSuccess;
}

*/

IOService*	AsusIntelGMA3150FB::probe			(IOService* provider, SInt32* score)
{
	IOLog("%s::probe(%s, %d)\n", getName(), provider->getName(), *score);
	// primary hw setup/init in enableController() method, call within here?
	// interrogate for device
		// PCI config IDs
		// functional behavior?

	pPCIDevice		= OSDynamicCast(IOPCIDevice, provider);
	UInt16	venID	= pPCIDevice->configRead16(kIOPCIConfigVendorID);					// 2
	UInt16	devID	= pPCIDevice->configRead16(kIOPCIConfigDeviceID);					// 2
	UInt16	subvenID= pPCIDevice->configRead16(kIOPCIConfigSubSystemVendorID);			// 2
	UInt16	subID	= pPCIDevice->configRead16(kIOPCIConfigSubSystemID);				// 2

	IOLog("PCI dev:ven = [%04x:%04x]\n", devID, venID);
	IOLog(" sub:subven = [%04x:%04x]\n", subID, subvenID);

	printPCIConfigRegs	(pPCIDevice);
	enumerateMemRegions	(pPCIDevice);

	// failure
	// success
	score	= 0;
	return	this;
}

void		AsusIntelGMA3150FB::printPCIConfigRegs(IOPCIDevice* pPCIDevice)
{
	// output basic PCI config regs to dmesg
	UInt16	venID	= pPCIDevice->configRead16(kIOPCIConfigVendorID);					// 2
	UInt16	devID	= pPCIDevice->configRead16(kIOPCIConfigDeviceID);					// 2
	UInt16	command	= pPCIDevice->configRead16(kIOPCIConfigCommand);					// 2
	UInt16	status	= pPCIDevice->configRead16(kIOPCIConfigStatus);						// 2
	UInt8	revID	= pPCIDevice->configRead8(kIOPCIConfigRevisionID);					// 1
	//UInt32	classCode		= pPCIDevice->configRead32(kIOPCIConfigClassCode);			// 3
	////classCode &= 0xffffffff;
	UInt8	classCode0		= pPCIDevice->configRead8(kIOPCIConfigClassCode);			// 3
	UInt8	classCode1		= pPCIDevice->configRead8(kIOPCIConfigClassCode+1);			// 3
	UInt8	classCode2		= pPCIDevice->configRead8(kIOPCIConfigClassCode+2);			// 3
	UInt8	cacheLineSize	= pPCIDevice->configRead8(kIOPCIConfigCacheLineSize);		// 1
	UInt8	latencyTimer	= pPCIDevice->configRead8(kIOPCIConfigLatencyTimer);		// 1
	UInt8	headerType		= pPCIDevice->configRead8(kIOPCIConfigHeaderType);			// 1

	//		MMADR	Memory-Mapped Range Address
	UInt32	mmadr	= pPCIDevice->configRead32(kIOPCIConfigBaseAddress0);				// 4
	UInt32	mm_mba	= mmadr & 0xfff80000;	// [31:19]	memory base address
	UInt16	mm_adm	= 0x0000;				// [18: 4]	512KB address range
	UInt8	mm_prefmem = 0x0;				// [ 3]		not prefetchable
	UInt8	mm_memtyp = 0x00;				// [ 2: 1]	32-bit addr
	UInt8	mm_mios	= 0x0;					// [ 0]		memory space
	//		these are the expected hardwired values of 0s, but the actual register bits
	//		should ideally be checked against expected values for rigor
	
	//		IOBAR	Input/Output Base Address
	UInt32	iobar	= pPCIDevice->configRead32(kIOPCIConfigBaseAddress1);				// 4
	UInt32	io_mba	= iobar & 0xffff0000;	// [31:16]	memory base address
	UInt16	io_adm	= 0x0000;				// [15: 3]	512KB address range
	UInt8	io_memtyp = 0x00;				// [ 2: 1]	32-bit addr
	UInt8	io_mios	= 0x1;					// [ 1]		IO space

	//		GMADR	Graphics Memory Range Address (primary frame buffer?)
	UInt32	gmadr	= pPCIDevice->configRead32(kIOPCIConfigBaseAddress2);				// 4
	//UInt32	gm_mba	= mmadr & 0xf0000000;	// [31:29]	memory base address (RW)
	//UInt32	gm_adm	= 0x0000;				// [28: 4]	512MB range (512ADMSK)
	UInt32	gm_mba	= mmadr & 0xf0000000;	// [31:28]	memory base address (RW)
	UInt32	gm_adm	= 0x0000;				// [27: 4]	256MB range (256ADMSK)
	UInt8	gm_prefmem = 0x1;				// [ 3]		prefetchable
	UInt8	gm_memtyp = 0x00;				// [ 2: 1]	32-bit addr
	UInt8	gm_mios	= 0x0;					// [ 0]		memory space

	//		GTTADR	Graphics Translation Table Range Address
	UInt32	gttadr	= pPCIDevice->configRead32(kIOPCIConfigBaseAddress3);				// 4
	UInt16	gt_mba	= mmadr & 0xfff00000;	// [31:20]	memory base address
	UInt16	gt_adm	= 0x0000;				// [19: 4]	1MB address range
	UInt8	gt_prefmem = 0x0;				// [ 3]		not prefetchable
	UInt8	gt_memtyp = 0x00;				// [ 2: 1]	32-bit addr
	UInt8	gt_mios	= 0x0;					// [ 0]		memory space

	UInt32	cardBusCISPtr	= pPCIDevice->configRead32(kIOPCIConfigCardBusCISPtr);		// 4
	UInt32	rom				= pPCIDevice->configRead32(kIOPCIConfigExpansionROMBase);	// 4
	UInt32	capAddrLow		= pPCIDevice->configRead32(kIOPCIConfigCapabilitiesPtr);	// 4
	UInt32	capAddrHigh		= pPCIDevice->configRead32(kIOPCIConfigCapabilitiesPtr+4);	// 4
	UInt8	intLine	= pPCIDevice->configRead8(kIOPCIConfigInterruptLine);				// 1
	UInt8	intPin	= pPCIDevice->configRead8(kIOPCIConfigInterruptPin);				// 1
	UInt8	minGrant	= pPCIDevice->configRead8(kIOPCIConfigMinimumGrant);			// 1
	UInt8	maxLatency	= pPCIDevice->configRead8(kIOPCIConfigMaximumLatency);			// 1

	IOLog("PCI Config Registers\n");
	IOLog("[%04x:%04x] Revision ID      = %02x\n", devID, venID, revID);
	IOLog("[%04x:%04x] Command          = %04x\n", devID, venID, command);
	IOLog("[%04x:%04x] Status           = %04x\n", devID, venID, status);
	IOLog("[%04x:%04x] Class Code       = %02x%02x%02x\n", devID, venID, classCode2, classCode1, classCode0);
	IOLog("[%04x:%04x] Cache Line Size  = %02x\n", devID, venID, cacheLineSize);
	IOLog("[%04x:%04x] Latency Timer    = %02x\n", devID, venID, latencyTimer);
	IOLog("[%04x:%04x] Header Type      = %02x\n", devID, venID, headerType);
	IOLog("[%04x:%04x] IO Memory Region Address Registers\n", devID, venID);
	IOLog("[%04x:%04x] Memory Mapped IO =         :%08x\n", devID, venID, mmadr);
	IOLog("[%04x:%04x] IO Space         =         :%08x\n", devID, venID, iobar);
	IOLog("[%04x:%04x] Gfx Memory Range =         :%08x\n", devID, venID, gmadr);
	IOLog("[%04x:%04x] Gfx Translt Tbl  =         :%08x\n", devID, venID, gttadr);
	IOLog("[%04x:%04x] CardBus CIS Addr =         :%08x\n", devID, venID, cardBusCISPtr);
	IOLog("[%04x:%04x] Expansion ROM Adr=         :%08x\n", devID, venID, rom);
	IOLog("[%04x:%04x] Capabilities Ptr = %08x:%08x\n", devID, venID, capAddrHigh, capAddrLow);
	IOLog("[%04x:%04x] Interrupt Line   = %d\n", devID, venID, intLine);
	IOLog("[%04x:%04x] Interrupt Pin    = %d\n", devID, venID, intPin);
	IOLog("[%04x:%04x] Minimum Grant    = %d\n", devID, venID, minGrant);
	IOLog("[%04x:%04x] Maximum Latency  = %d\n", devID, venID, maxLatency);
}

void		AsusIntelGMA3150FB::enumerateMemRegions(IOPCIDevice* pPCIDevice)
{
	UInt32	nMemRegions = pPCIDevice->getDeviceMemoryCount();
	UInt16	venID	= pPCIDevice->configRead16(kIOPCIConfigVendorID);					// 2
	UInt16	devID	= pPCIDevice->configRead16(kIOPCIConfigDeviceID);					// 2

	IOLog("PCI Device Memory Regions:\n");
	for (UInt32 i = 0; i < nMemRegions; ++i)
	{
		IODeviceMemory*	memDesc = pPCIDevice->getDeviceMemoryWithIndex(i);
		if (memDesc)
		{
#ifdef	__LP64__
			UInt64	addr	= memDesc->getPhysicalAddress();
			UInt32	addrLow	= (UInt32)(addr);
			UInt32	addrHigh= (UInt32)(addr >> 32);
			IOLog("[%04x:%04x] region %u: %12llu bytes @ %08x:%08x\n",
				devID, venID, i, memDesc->getLength(), addrHigh, addrLow);
#else
			IOLog("[%04x:%04x] region %lu: %12lu bytes @ %08x\n",
				devID, venID, i, memDesc->getLength(), memDesc->getPhysicalAddress());
#endif
		}
	}
}

/*

IOReturn	AsusIntelGMA3150FB::powerStateWillChangeTo(IOPMPowerFlags capabilities,
												unsigned long stateNumber,
												IOService* whatDevice)
{

}

IOReturn	AsusIntelGMA3150FB::powerStateDidChangeTo(IOPMPowerFlags capabilities,
												unsigned long stateNumber,
												IOService* whatDevice)
{

}

IOReturn	AsusIntelGMA3150FB::setPowerState	(unsigned long powerStateOrdinal,
												IOService* device)
{

}

IOReturn	AsusIntelGMA3150FB::setAggressiveness(unsigned long type,
												unsigned long newLevel)
{

}

IOReturn	AsusIntelGMA3150FB::getAggressiveness(unsigned long type,
												unsigned long* currentLevel)
{

}

IOReturn	AsusIntelGMA3150FB::newUserClient	(task_t			owningTask,
                                 				void *			security_id,
                                 				UInt32			type,
                                 				IOUserClient**	handler )
{

}

IOReturn	AsusIntelGMA3150FB::callPlatformFunction(const OSSymbol* functionName,
                                  				bool waitForFunction,
                                  				void* p1, void* p2,
                                  				void* p3, void* p4 )
{

}

//////////////////////////////
// (5) Overrides	IOGraphicsDevice.h
//////////////////////////////
void		AsusIntelGMA3150FB::hideCursor		()
{

}

void		AsusIntelGMA3150FB::showCursor		(IOGPoint* cursorLoc, int frame)
{

}

void		AsusIntelGMA3150FB::moveCursor		(IOGPoint* cursorLoc, int frame)
{

}

//void		AsusIntelGMA3150FB::resetCursor		()

void		AsusIntelGMA3150FB::getVBLTime		(AbsoluteTime* time, AbsoluteTime* delta)
{

}

void		AsusIntelGMA3150FB::getBoundingRect	(IOGBounds** bounds)
{

}


//////////////////////////////
// (4) Overrides	IOFramebuffer, Unknown purpose/necessity
//////////////////////////////
bool		AsusIntelGMA3150FB::isConsoleDevice	()
{

}

IOReturn	AsusIntelGMA3150FB::setupForCurrentConfig()
{

}

bool		AsusIntelGMA3150FB::serializeInfo	(OSSerialize* s)
{

}

bool		AsusIntelGMA3150FB::setNumber		(OSDictionary* dict,
												const char* key, UInt32 number)
{

}


//////////////////////////////
// (12) Overrides	IOFramebuffer, Required
//////////////////////////////
IODeviceMemory* AsusIntelGMA3150FB::getApertureRange(IOPixelAperture aperture)
{

}

const
char*		AsusIntelGMA3150FB::getPixelFormats	()
{

}

IOItemCount	AsusIntelGMA3150FB::getDisplayModeCount()
{

}

IOReturn	AsusIntelGMA3150FB::getDisplayModes	(IODisplayModeID* allDisplayModes)
{

}

IOReturn	AsusIntelGMA3150FB::getInformationForDisplayMode(IODisplayModeID displayMode,
												IODisplayModeInformation* info)
{

}
{

}

UInt64		AsusIntelGMA3150FB::getPixelFormatsForDisplayMode(IODisplayModeID displayMode,
												IOIndex depth) {return	0;}
{

}

IOReturn	AsusIntelGMA3150FB::getPixelInformation(IODisplayModeID displayMode,
												IOIndex depth,
												IOPixelAperture aperture,
												IOPixelInformation* pixelInfo)
{

}

IOReturn	AsusIntelGMA3150FB::getCurrentDisplayMode(IODisplayModeID* displayMode,
												IOIndex* depth)
{

}

// hw interrupt handlers
// enable/disable installed handlers
IOReturn	AsusIntelGMA3150FB::setInterruptState(void* interruptRef, UInt32 state)
{

}

IOReturn	AsusIntelGMA3150FB::getNotificationSemaphore(IOSelect interruptType,
												semaphore_t* semaphore)
{

}

// install/register handlers for hw interrupts
IOReturn	AsusIntelGMA3150FB::registerForInterruptType(IOSelect interruptType,
												IOFBInterruptProc proc, OSObject* target,
												void* ref, void** interruptRef)
{

}

// remove handler previously installed by above
IOReturn	AsusIntelGMA3150FB::unregisterInterrupt(void* interruptRef)
{

}
// end hw interrupt handler methods

//////////////////////////////
// (12) Overrides	IOFramebuffer, Recommended
//////////////////////////////

// if device supports hardware cursors, 
bool		AsusIntelGMA3150FB::convertCursorImage(void* cursorImage,
												IOHardwareCursorDescriptor* description,
												IOHardwareCursorInfo* cursor)
{

}

IOReturn	AsusIntelGMA3150FB::setCursorImage	(void* cursorImage)
{

}

IOReturn	AsusIntelGMA3150FB::setCursorState	(SInt32 x, SInt32 y, bool visible)
{

}

// (attribute == kIOHardwareCursorAttribute), *value = true)
IOReturn	AsusIntelGMA3150FB::getAttribute	(IOSelect attribute, uintptr_t* value)
{

}

// endif hardwarecursors

// software cursors
void		AsusIntelGMA3150FB::flushCursor		()
{

}

IOReturn	AsusIntelGMA3150FB::getTimingInformationForDisplayMode(IODisplayModeID displayMode,
												IOTimingInformation* info)
{

}

IOReturn	AsusIntelGMA3150FB::setStartupDisplayMode(IODisplayModeID displayMode,
												IOIndex depth)
{

}

IOReturn	AsusIntelGMA3150FB::getStartupDisplayMode(IODisplayModeID* displayMode,
												IOIndex *depth)
{

}

IOReturn	AsusIntelGMA3150FB::setGammaTable	(UInt32 channelCount, UInt32 dataCount,
												UInt32 dataWidth, void* data)
{

}

IODeviceMemory* AsusIntelGMA3150FB::getVRAMRange()
{

}

// Should call these on certain power state changes
//IOReturn handleEvent		(IOIndex event, void* info = 0)
//IOReturn deliverFramebufferNotification(IOIndex event, void* info = 0)

// if hw support for DDC/EDID sensing
bool		AsusIntelGMA3150FB::hasDDCConnect	(IOIndex connectIndex)
{

}

IOReturn	AsusIntelGMA3150FB::getDDCBlock		(IOIndex connectIndex, UInt32 blockNumber,
												IOSelect blockType, IOOptionBits options,
												UInt8* data, IOByteCount* length)
{

}
// endif hw DDC/EDID

// if software implementation of performing I2C requests
//IOReturn	AsusIntelGMA3150FB::doI2CRequest	(UInt32 bus,
//												struct IOI2CBustiming *timing, 
//												struct IOI2CRequest *request) 
//
//void		AsusIntelGMA3150FB::setDDCClock		(IOIndex bus, UInt32 value)
//
//void		AsusIntelGMA3150FB::setDDCData		(IOIndex bus, UInt32 value)
//
//bool		AsusIntelGMA3150FB::readDDCClock	(IOIndex bus)
//
//bool		AsusIntelGMA3150FB::readDDCData		(IOIndex bus)
//
//IOReturn	AsusIntelGMA3150FB::enableDDCRaster	(bool enable)
// endif software I2C methods

// if displays have legacy Apple sensing
//IOReturn	AsusIntelGMA3150FB::getAppleSense	(IOIndex connectionIndex,
//												UInt32* senseType,
//												UInt32* primary,
//												UInt32* extended,
//												UInt32* displayType)
//
//IOReturn	AsusIntelGMA3150FB::connectFlags	(IOIndex connectIndex,
//												IODisplayModeID displayMode,
//												IOOptionBits* flags)
// endif AppleSense methods

*/

// Perform first time hw initialization setup here
IOReturn	AsusIntelGMA3150FB::enableController()
{
	return	kIOReturnSuccess;
}

/*
//////////////////////////////
// (7) Overrides	IOFramebuffer, Optional
//////////////////////////////
IOReturn	AsusIntelGMA3150FB::setDisplayMode	(IODisplayModeID displayMode, IOIndex depth)
{

}

IOReturn	AsusIntelGMA3150FB::setAttribute	(IOSelect attribute, uintptr_t value)
{

}

IOReturn	AsusIntelGMA3150FB::validateDetailedTiming(void* desc, IOByteCount descripSize)
{

}

IOReturn	AsusIntelGMA3150FB::setDetailedTimings(OSArray* array)
{

}

IOItemCount	AsusIntelGMA3150FB::getConnectionCount()
{

}

IOReturn	AsusIntelGMA3150FB::setAttributeForConnection(IOIndex connectionIndex,
												IOSelect attribute, uintptr_t value)
{

}



//IOReturn	AsusIntelGMA3150FB::setCLUTWithEntries(IOColorEntry* colors, UInt32 index,
//							UInt32 numEntries, IOOptionBits options)
{

}

// to set/enable a non-standard aperture.  The system does not call this.
//IOReturn	AsusIntelGMA3150FB::setApertureEnable(IOPixelAperture aperture,
//							IOOptionBits enable)
{

}

*/

