////////////////////////////////////////////////////////////////////
//          File: AsusIntelGMA3150FB.h
//        Author: Gerald Leung
// Creation Date: Thu Dec 27 17:13:33 PST 2012
//      Copyright 2012, Gerald Leung, All Rights Reserved
//       Purpose: AsusIntelGMA3150FB interface
////////////////////////////////////////////////////////////////////

#ifndef		ASUSINTELGMA3150FB_H
#define		ASUSINTELGMA3150FB_H
// Interface includes
#include	<IOKit/graphics/IOFramebuffer.h>

// Implementation includes
//#include	<IOKit/i2c/IOI2CInterface.h>
//#include	<IOKit/graphics/IOAccelerator.h>
#include	<IOKit/IOLib.h>
#include	<IOKit/pci/IOPCIDevice.h>


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

class		AsusIntelGMA3150FB	: public IOService
{
	OSDeclareDefaultStructors	(AsusIntelGMA3150FB);

public:
	//////////////////////////////
	// (16) Overrides	IOService
	//////////////////////////////
	virtual
	bool	requestTerminate	(IOService* provider, IOOptionBits options);

    virtual
	IOService* probe			(IOService* provider, SInt32* score);

	virtual
	void	printPCIConfigRegs	(IOPCIDevice* pPCIDevice);

	virtual
	void	enumerateMemRegions	(IOPCIDevice* pPCIDevice);

	// primary hw setup/init in enableController() method, call within here?
    virtual
	bool	start				(IOService* provider);

    virtual
	void	stop				(IOService* provider);

    virtual
	IOReturn open				();
    
    virtual
	void	close				();

    virtual
	void	free				();

    //virtual
	//IOWorkLoop* getWorkLoop		() const;

    //IOWorkLoop* getGraphicsSystemWorkLoop() const;
    //IOWorkLoop* getControllerWorkLoop() const;

    //virtual
	//IOReturn requestProbe		(IOOptionBits options);

/*
    virtual
	IOReturn powerStateWillChangeTo(IOPMPowerFlags capabilities,
								unsigned long stateNumber,
								IOService* whatDevice);

    virtual
	IOReturn powerStateDidChangeTo(IOPMPowerFlags capabilities,
								unsigned long stateNumber,
								IOService* whatDevice);
	
    virtual
	IOReturn setPowerState		(unsigned long powerStateOrdinal,
								IOService* device);
	
    virtual
	IOReturn setAggressiveness	(unsigned long type,
								unsigned long newLevel);

    virtual
	IOReturn getAggressiveness	(unsigned long type,
								unsigned long* currentLevel);

    virtual
	IOReturn newUserClient		(task_t			owningTask,
                                 void *			security_id,
                                 UInt32			type,
                                 IOUserClient**	handler );

    virtual
	IOReturn callPlatformFunction(const OSSymbol* functionName,
                                  bool waitForFunction,
                                  void* p1, void* p2,
                                  void* p3, void* p4 );

	//////////////////////////////
	// (5) Overrides	IOGraphicsDevice.h
	//////////////////////////////
    virtual
	void	hideCursor			();

    virtual
	void	showCursor			(IOGPoint* cursorLoc, int frame);

    virtual
	void	moveCursor			(IOGPoint* cursorLoc, int frame);

    //virtual
    //void resetCursor( void );

    virtual
	void	getVBLTime			(AbsoluteTime* time, AbsoluteTime* delta);

    virtual
	void	getBoundingRect		(IOGBounds** bounds);


	//////////////////////////////
	// (4) Overrides	IOFramebuffer, Unknown purpose/necessity
	//////////////////////////////
    virtual
	bool	isConsoleDevice		();

    virtual
	IOReturn setupForCurrentConfig();

    virtual
	bool	serializeInfo		(OSSerialize* s);

    virtual
	bool	setNumber			(OSDictionary* dict,
								const char* key, UInt32 number);


	//////////////////////////////
	// (12) Overrides	IOFramebuffer, Required
	//////////////////////////////
	virtual
	IODeviceMemory* getApertureRange(IOPixelAperture aperture);

	virtual
	const
	char*	getPixelFormats		();

	virtual
	IOItemCount getDisplayModeCount();

	virtual
	IOReturn getDisplayModes	(IODisplayModeID* allDisplayModes);

	virtual
	IOReturn getInformationForDisplayMode(IODisplayModeID displayMode,
								IODisplayModeInformation* info);

	virtual
	UInt64	getPixelFormatsForDisplayMode(IODisplayModeID displayMode,
								IOIndex depth) {return	0;}

	virtual
	IOReturn getPixelInformation(IODisplayModeID displayMode,
								IOIndex depth,
								IOPixelAperture aperture,
								IOPixelInformation* pixelInfo);

	virtual
	IOReturn getCurrentDisplayMode(IODisplayModeID* displayMode,
								IOIndex* depth);

	// hw interrupt handlers
	// enable/disable installed handlers
	virtual
	IOReturn setInterruptState	(void* interruptRef, UInt32 state);

	virtual
	IOReturn getNotificationSemaphore(IOSelect interruptType,
								semaphore_t* semaphore);

	// install/register handlers for hw interrupts
	virtual
	IOReturn registerForInterruptType(IOSelect interruptType,
								IOFBInterruptProc proc, OSObject* target,
								void* ref, void** interruptRef);

	// remove handler previously installed by above
	virtual
	IOReturn unregisterInterrupt(void* interruptRef);
	// end hw interrupt handler methods

	//////////////////////////////
	// (12) Overrides	IOFramebuffer, Recommended
	//////////////////////////////

	// if device supports hardware cursors, 
	virtual
	bool	convertCursorImage	(void* cursorImage,
								IOHardwareCursorDescriptor* description,
								IOHardwareCursorInfo* cursor);

	virtual
	IOReturn setCursorImage		(void* cursorImage);

	virtual
	IOReturn setCursorState		(SInt32 x, SInt32 y, bool visible);

	// (attribute == kIOHardwareCursorAttribute), *value = true)
	virtual
	IOReturn getAttribute		(IOSelect attribute, uintptr_t* value);

	// endif hardwarecursors

	// software cursors
	virtual
	void	flushCursor			();

	virtual
	IOReturn getTimingInformationForDisplayMode(IODisplayModeID displayMode,
								IOTimingInformation* info);

	virtual
	IOReturn setStartupDisplayMode(IODisplayModeID displayMode,
								IOIndex depth);

	virtual
	IOReturn getStartupDisplayMode(IODisplayModeID* displayMode,
								IOIndex *depth);

	virtual
	IOReturn setGammaTable		(UInt32 channelCount, UInt32 dataCount,
								UInt32 dataWidth, void* data);

	virtual
	IODeviceMemory* getVRAMRange();

	// Should call these on certain power state changes
	//IOReturn handleEvent		(IOIndex event, void* info = 0);
	//IOReturn deliverFramebufferNotification(IOIndex event, void* info = 0);

	// if hw support for DDC/EDID sensing
	virtual
	bool	hasDDCConnect		(IOIndex connectIndex);

	virtual
	IOReturn getDDCBlock		(IOIndex connectIndex, UInt32 blockNumber,
								IOSelect blockType, IOOptionBits options,
								UInt8* data, IOByteCount* length);
	// endif hw DDC/EDID

	// if software implementation of performing I2C requests
	//virtual
	//IOReturn	doI2CRequest	(UInt32 bus,
	//							struct IOI2CBustiming *timing, 
	//							struct IOI2CRequest *request); 
	//
	//virtual
	//void	setDDCClock			(IOIndex bus, UInt32 value);
	//
	//virtual
	//void	setDDCData			(IOIndex bus, UInt32 value);
	//
	//virtual
	//bool	readDDCClock		(IOIndex bus);
	//
	//virtual
	//bool	readDDCData			(IOIndex bus);
	//
	//virtual
	//IOReturn enableDDCRaster	(bool enable);
	// endif software I2C methods

	// if displays have legacy Apple sensing
	//virtual
	//IOReturn getAppleSense		(IOIndex connectionIndex,
	//							UInt32* senseType,
	//							UInt32* primary,
	//							UInt32* extended,
	//							UInt32* displayType);
	//
	//virtual
	//IOReturn connectFlags		(IOIndex connectIndex,
	//							IODisplayModeID displayMode,
	//							IOOptionBits* flags);
	// endif AppleSense methods
	*/

	// Perform first time hw initialization setup here
	virtual
	IOReturn enableController	();

	/*

	//////////////////////////////
	// (7) Overrides	IOFramebuffer, Optional
	//////////////////////////////
	virtual
	IOReturn setDisplayMode		(IODisplayModeID displayMode, IOIndex depth);

	virtual
	IOReturn setAttribute		(IOSelect attribute, uintptr_t value);

	virtual
	IOReturn validateDetailedTiming(void* desc, IOByteCount descripSize);

	virtual
	IOReturn setDetailedTimings	(OSArray* array);

	virtual
	IOItemCount getConnectionCount();

	virtual
	IOReturn setAttributeForConnection(IOIndex connectionIndex,
								IOSelect attribute, uintptr_t value);



	//virtual
	//IOReturn setCLUTWithEntries	(IOColorEntry* colors, UInt32 index,
	//							UInt32 numEntries, IOOptionBits options);

	// to set/enable a non-standard aperture.  The system does not call this.
	//virtual
	//IOReturn setApertureEnable	(IOPixelAperture aperture,
	//							IOOptionBits enable);

*/

private:
	IOPCIDevice*
			pPCIDevice;
};

#endif		// ASUSINTELGMA3150FB_H

