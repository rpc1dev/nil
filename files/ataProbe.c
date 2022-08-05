/*
 * ataProbe v1.0 - DOS ATA(PI) Drive detection
 * Almost completely stolen from Chris Giese's public domain cdplay.c
 * (http://my.execpc.com/~geezer/software/cdplay.c)
 *
 * Usage: ataProbe [Name]
 *
 * When optional parameter [Name] is given, ataProbe tries to 
 * do a partial match against the first device beginning with
 * the string and returns the device position as errorcode 
 * (1 = Primary Master -> 4 = Secondary Slace).
 * If no match is found, errorcode 255 is returned.
 *  
 * This code is public domain (no copyright).
 * You can do whatever you want with it.
 */

#include <string.h>	/* memset() */
#include <stdio.h>	/* printf(), putchar() */
#include <conio.h>	/* getch() */
#include <time.h>	/* time() */
#include <dos.h>

#if 0
/* extra debugging messages */
#define	DEBUG(X)	X
#else
/* no debug msgs */
#define	DEBUG(X)
#endif

#if defined(__TURBOC__)
#define	INLINE
/* nothing, Turbo C++ 'inline' works only for C++ member functions */

#elif defined(__DJGPP__)
#define	INLINE	__inline__

#define	outport(P,V)	outportw(P,V)
#define	inport(P)	inportw(P)

#else
#error Not Turbo C, not DJGPP. Sorry.
#endif

#if 1	/* little-endian machine (e.g. Intel) */
#define	LE16(X)	(X)
#else	/* big-endian (e.g. 680x0 Macs). UNTESTED. */
#define	LE16(X)	bswap16(X)
#endif

/* geezer's Portable Interrupt Macros (tm) */
#if defined(__TURBOC__)
/* getvect(), setvect() in dos.h */

#define	INTERRUPT		interrupt

#define SAVE_VECT(Num,Vec)	Vec=getvect(Num)
#define	SET_VECT(Num,Fn)	setvect(Num, Fn)
#define	RESTORE_VECT(Num,Vec)	setvect(Num, Vec)

typedef void interrupt(*vector)(void);

#elif defined(__DJGPP__)
#include <dpmi.h>	/* _go32_dpmi_... */
#include <go32.h>	/* _my_cs() */

#define	INTERRUPT	/* nothing */

#define	SAVE_VECT(Num,Vec)	\
	_go32_dpmi_get_protected_mode_interrupt_vector(Num, &Vec)
#define	SET_VECT(Num,Fn)					\
	{	_go32_dpmi_seginfo NewVector;			\
								\
		NewVector.pm_selector=_my_cs();			\
		NewVector.pm_offset=(unsigned long)Fn;		\
		_go32_dpmi_allocate_iret_wrapper(&NewVector);	\
		_go32_dpmi_set_protected_mode_interrupt_vector	\
			(Num, &NewVector); }
#define	RESTORE_VECT(Num,Vec)	\
	_go32_dpmi_set_protected_mode_interrupt_vector(Num, &Vec)

typedef _go32_dpmi_seginfo vector;

#endif

#define	min(A,B)	(((A) < (B)) ? (A) : (B))

/* a pity you can't read the IRQ-to-interrupt assignments from the 8259 */
#define	IRQ14_VECTOR	118
#define	IRQ15_VECTOR	119

/* ATA/ATAPI drive register file (offsets from 0x1F0 or 0x170) */
#define	ATA_REG_DATA	0		/* data (16-bit) */
#define	ATA_REG_FEAT	1		/* write: feature reg */
#define	ATA_REG_ERR	ATA_REG_FEAT	/* read: error */
#define	ATA_REG_CNT	2		/* ATA: sector count */
#define	ATA_REG_REASON	ATA_REG_CNT	/* ATAPI: interrupt reason */
#define	ATA_REG_SECT	3		/* sector */
#define	ATA_REG_LOCYL	4		/* ATA: LSB of cylinder */
#define	ATA_REG_LOCNT	ATA_REG_LOCYL	/* ATAPI: LSB of transfer count */
#define	ATA_REG_HICYL	5		/* ATA: MSB of cylinder */
#define	ATA_REG_HICNT	ATA_REG_HICYL	/* ATAPI: MSB of transfer count */
#define	ATA_REG_DRVHD	6		/* drive select; head */
#define	ATA_REG_CMD	7		/* write: drive command */
#define	ATA_REG_STAT	7		/* read: status and error flags */
#define	ATA_REG_SLCT	0x206		/* write: device control */
#define	ATA_REG_ALTST	0x206		/* read: alternate status/error */

/* ATA drive command bytes */
#define	ATA_CMD_RD	0x20		/* read one sector */
#define	ATA_CMD_WR	0x30		/* write one sector */
#define	ATA_CMD_PKT	0xA0		/* ATAPI packet cmd */
#define	ATA_CMD_PID	0xA1		/* ATAPI identify */
#define	ATA_CMD_RDMUL	0xC4		/* read multiple sectors */
#define	ATA_CMD_WRMUL	0xC5		/* write multiple sectors */
#define	ATA_CMD_ID	0xEC		/* ATA identify */
//#define	ATA_CMD_RDDMA	0xC8		/* read sectors w/ DMA */
//#define	ATA_CMD_WRDMA	0xCA		/* write sectors w/ DMA */
//#define	ATA_CMD_SETMULT	0xC6		/* set multiple mode */

/* ATA sector size (512 bytes) */
#define	ATA_SECTSIZE		512

/* ATAPI sector size (2048 bytes) */
#define	ATAPI_LG_SECTSIZE	11
#define	ATAPI_SECTSIZE		(1 << ATAPI_LG_SECTSIZE)

/* ATAPI data/command transfer 'phases' */
#define	ATAPI_PH_ABORT		0	/* other possible phases */
#define	ATAPI_PH_DONE		3	/* (1, 2, 11) are invalid */
#define	ATAPI_PH_DATAOUT	8
#define	ATAPI_PH_CMDOUT		9
#define	ATAPI_PH_DATAIN		10

/* ATAPI packet command bytes */
#define	ATAPI_CMD_START_STOP	0x1B	/* eject/load */
#define	ATAPI_CMD_READ10	0x28	/* read data sector(s) */
#define	ATAPI_CMD_READTOC	0x43	/* read audio table-of-contents */
#define	ATAPI_CMD_PLAY		0x47	/* play audio */
#define	ATAPI_CMD_PAUSE		0x4B	/* pause/continue audio */

/* delays from Linux ide.h (milliseconds)
WAIT_WORSTCASE -- interrupt after ATA 'identify' command */
#define	WAIT_ID		30000
/* WAIT_PIDENTIFY -- interrupt after ATAPI 'identify' command */
#define	WAIT_PID	1000
/* WAIT_CMD -- interrupt after read/read multiple command
or after command packet issued to ATAPI drive */
#define	WAIT_CMD	10000

#define	WAIT_READY	30	/* RDY asserted, use 5000 for notebook/APM */

/* 'Cmd' field of 'drivecmd' structure */
#define	DRV_CMD_RD	1
#define	DRV_CMD_WR	2

typedef unsigned char	u8;
typedef unsigned short	u16;
typedef	unsigned long	u32;

typedef enum
{	false=0, true=1 } bool;

typedef struct
{	char Description[41];
	u16 Cyls, Heads, Sects;	/* CHS geometry */
	unsigned DoesDMA : 1;
	unsigned DoesLBA : 1;
	unsigned UseLBA : 1;
	unsigned IsATAPI : 1;
	u16 MultSect;	/* number of sectors moved in multiple mode */
	u8 DrvSel;	/* ATA, ATAPI only (LUN for SCSI?) */
	u16 IOAdr; } driveinfo;

typedef struct
{	u32 Blk;	/* in SECTORS */
	u32 Count;	/* in BYTES */
	u8 Cmd, Dev, *Data; } drivecmd;

static driveinfo _Drives[4];
static volatile u16 _InterruptOccured;
/****************************************************************************
	name:	bswap16
	action:	does endian swap of 16-bit quantity
	returns:swapped value
****************************************************************************/
INLINE u16 bswap16(u16 Arg)
{	return(((Arg << 8) & 0xFF00) |
		((Arg >> 8) & 0xFF00)); }
/****************************************************************************
	name:	readLE16
	action:	reads 16-bit Little Endian quantity Data from 2-byte Buffer
	returns:value read
****************************************************************************/
INLINE u16 readLE16(u8 *Buffer)
{	return(LE16(*(u16 *)Buffer)); }
/*****************************************************************************
	name:	nsleep
	action:	delay for specified number of Nanoseconds
*****************************************************************************/
#pragma argsused
void nsleep(unsigned Nanoseconds)
{	}
/*****************************************************************************
	name:	insw
	action:	reads Count words (16-bit) from I/O port Adr to
		memory location Data
*****************************************************************************/
INLINE void insw(unsigned Adr, u16 *Data, unsigned Count)
{	for(; Count != 0; Count--)
	{	*Data=inport(Adr);
		Data++; }}
/*****************************************************************************
	name:	outsw
	action:	writes Count words (16-bit) to I/O port Adr from
		memory location Data
*****************************************************************************/
INLINE void outsw(unsigned Adr, u16 *Data, unsigned Count)
{	for(; Count != 0; Count--)
	{	outport(Adr, *Data);
		Data++; }}
/*****************************************************************************
	name:	awaitInterrupt
	action:	waits with Timeout (mS) until interrupt(s) given by bitmask
		IRQMask occur
	returns:0 if timeout
		nonzero Mask value if interrupt occured
*****************************************************************************/
int awaitInterrupt(u16 IRQMask, unsigned Timeout)
{	u16 Intr, Time;

	for(Time=Timeout; Time != 0; Time--)
	{	Intr=_InterruptOccured & IRQMask;
		if(Intr != 0)
			break;
		delay(1); }
	DEBUG(printf("[waited for %3u ms]", Timeout - Time);)
	if(Time == 0)
		return(0);
	_InterruptOccured &= ~Intr;
	return(Intr); }
/*****************************************************************************
	name:	dump
	action:	hexadecimal memory dump of Count bytes at Data
*****************************************************************************/
#define BPERL		16	/* byte/line for dump */

static void dump(u8 *Data, unsigned Count)
{	u8 Byte1, Byte2;

	while(Count)
	{	for(Byte1=0; Byte1 < BPERL; Byte1++)
		{	if(Count == 0)
				break;
			printf("%02X ", Data[Byte1]);
			Count--; }
		printf("\t");
		for(Byte2=0; Byte2 < Byte1; Byte2++)
		{	if(Data[Byte2] < ' ')
				putchar('.');
			else putchar(Data[Byte2]); }
		printf("\n");
		Data += BPERL; }}
/*****************************************************************************
	name:	irq14
	action:	IRQ14 handler
*****************************************************************************/
static void INTERRUPT irq14(void)
{	_InterruptOccured |= 0x4000;
	outportb(0xA0, 0x20);
	nsleep(1000);
	outportb(0x20, 0x20); }
/*****************************************************************************
	name:	irq15
	action:	IRQ15 handler
*****************************************************************************/
static void INTERRUPT irq15(void)
{	_InterruptOccured |= 0x8000;
	outportb(0xA0, 0x20);
	nsleep(1000);
	outportb(0x20, 0x20); }
/*****************************************************************************
	name:	atapiXferCount
	returns:ATAPI byte count, read from ATA cylinder registers

	After an interrupt from the drive, you _must_ read (or,
	for ATAPI tape drives, write) this many bytes
	to prevent the drive going comatose.
*****************************************************************************/
INLINE static u16 atapiXferCount(driveinfo *Drive)
{	u16 RetVal;

	RetVal=inportb(Drive->IOAdr + ATA_REG_HICNT);
	RetVal <<= 8;
	RetVal |= inportb(Drive->IOAdr + ATA_REG_LOCNT);
	return(RetVal); }
/*****************************************************************************
	name:	atapiReadAndDiscard
	action:	reads ATAPI byte count (Got) from drive,
		reads minimum of (Want, Got) bytes from drive to Buffer,
		then reads and discards residual bytes.
	returns:number of bytes actually read to Buffer
*****************************************************************************/
static u16 atapiReadAndDiscard(driveinfo *Drive, u8 *Buffer, u16 Want)
{	u16 Count, Got;

	Got=atapiXferCount(Drive);
	DEBUG(printf("atapiReadAndDiscard: Want %u bytes, Got %u\n",
		Want, Got);)
	Count=min(Want, Got);
	insw(Drive->IOAdr + ATA_REG_DATA, (u16 *)Buffer, Count >> 1);
	if(Got > Count)
/* read only 16-bit words where possible */
	{	for(Count=Got - Count; Count > 1; Count -= 2)
			(void)inport(Drive->IOAdr + ATA_REG_DATA);
/* if the byte count is odd, read the odd byte last */
		if(Count != 0)
			(void)inportb(Drive->IOAdr + ATA_REG_DATA); }
/* unexpectedly short read: decrease Want */
	else if(Count < Want)
		Want=Got;
	return(Want); }
/*****************************************************************************
	name:	ataIdentify
	action:	probes drive after soft reset to see if it's ATA/ATAPI,
		issues ATA or ATAPI IDENTIFY DEVICE command to get drive
		information
	returns:-1 if error
		0  if success
*****************************************************************************/
static int ataIdentify(driveinfo *Drive)
{	u8 Buffer[512], IDCmd, SwapChars;
	u8 Temp, Temp1, Temp2;
	u16 IDDelay;
	u8 no_space = 0xFF;

//	DEBUG(printf("ataIdentify:\n");)
/* sector count and sector registers both set to 1 after soft reset */
	Temp1=inportb(Drive->IOAdr + ATA_REG_CNT);
	Temp2=inportb(Drive->IOAdr + ATA_REG_SECT);
	if(Temp1 != 0x01 || Temp2 != 0x01)
	{	printf("  <NONE>\n");
		return(-1); }
/* cylinder registers set to 0x14EB for ATAPI */
	Temp1=inportb(Drive->IOAdr + ATA_REG_LOCYL);
	Temp2=inportb(Drive->IOAdr + ATA_REG_HICYL);
	Temp=inportb(Drive->IOAdr + ATA_REG_STAT);
	_InterruptOccured=0;
	if(Temp1 == 0x14 && Temp2 == 0xEB)
	{	u16 Foo;

/* I guess soft reset isn't enough to clear a wedged ATAPI CD-ROM --
you need to read the byte count from the drive and read and
discard that many bytes. */

// xxx -- this doesn't work and I don't know why
		atapiReadAndDiscard(Drive, NULL, 0);

		for(Foo=atapiXferCount(Drive); Foo != 0; Foo--)
			(void)inportb(Drive->IOAdr);

//		printf("  [ATAPI drive, ");
		IDCmd=ATA_CMD_PID;
		IDDelay=WAIT_PID;
		Drive->IsATAPI=1; }
/* cylinder registers set to 0 for ATA */
	else if(Temp1 == 0 && Temp2 == 0 && Temp != 0)
	{	//printf("  [ATA drive, ");
		IDCmd=ATA_CMD_ID;
		IDDelay=WAIT_ID; }
	else
	{	
		printf("  <NONE>\n");
		return(-1); }
/* issue ATA or ATAPI IDENTIFY DEVICE command, then get results */
	outportb(Drive->IOAdr + ATA_REG_CMD, IDCmd);
	nsleep(400);
	if(awaitInterrupt(0xC000, IDDelay) == 0)
/* could be very old drive that doesn't support IDENTIFY DEVICE.
Read geometry from partition table? Use CMOS? */
	{	printf("(IDENTIFY DEVICE failed)\n");
		return(-1); }
	insw(Drive->IOAdr + ATA_REG_DATA, (u16 *)Buffer, 256);/* xxx - read ATAPI xfer count */
/* print some info */
	SwapChars=1;
/* model name is not byte swapped for NEC, Mitsumi FX, and Pioneer CD-ROMs */
	if(IDCmd == ATA_CMD_PID)
	{	if((Buffer[54] == 'N' && Buffer[55] == 'E') ||
			(Buffer[54] == 'F' && Buffer[55] == 'X') ||
			(Buffer[54] == 'P' && Buffer[55] == 'i'))
				SwapChars=0; }
/* Copy the model name, skipping blanks */
	Drive->Description[40] = 0;
	for(Temp=39; Temp != 0xFF; Temp--)
	{	Drive->Description[Temp] = Buffer[(Temp+54) ^ SwapChars];
		if (Drive->Description[Temp] == 0x20)
		{	if (no_space)
				Drive->Description[Temp] = 0; }
		else
			no_space = 0;
 	}
	printf("  %s [%s drive]", Drive->Description,
	          (Drive->IsATAPI)?"ATAPI":"ATA");
	Drive->Cyls=readLE16(Buffer + 2);
	Drive->Heads=readLE16(Buffer + 6);
	Drive->Sects=readLE16(Buffer + 12);
	printf("\n    CHS=%u:%u:%u, ", Drive->Cyls, Drive->Heads,
		Drive->Sects);
	if((Buffer[99] & 1) != 0)
	{	printf("DMA, ");
		Drive->DoesDMA=1; }
	if((Buffer[99] & 2) != 0)
	{	printf("LBA, ");
		Drive->DoesLBA=1; }
	if(((Buffer[119] & 1) != 0) && (Buffer[118] != 0))
	{	Temp=Buffer[94];
		printf("MaxMult=%u, ", Temp); }
	else Temp=1;
	Drive->MultSect=Temp;
	printf("%uK cache\n", readLE16(Buffer + 42) >> 1);
#if 0
/* PIO and DMA transfer modes indicate how fast the drive can move data.
This is of interest only if you have an intelligent IDE controller
(e.g. EIDE VLB chip for 486 system, or Triton PCI for Pentium), where the
controller can be programmed to move data at the fastest rate possible */
if(Drive->IsATAPI)
{	printf("  max PIO mode %u\n", readLE16(Buffer + 102));
	printf("  max DMA mode %u\n", readLE16(Buffer + 104));
	printf("  single-word DMA status 0x%X\n", readLE16(Buffer + 124));
	printf("  multi-word DMA status 0x%X\n", readLE16(Buffer + 126));
	printf("  advanced PIO mode supported=%u\n", Buffer[128]);
/* bits b6:b5 indicate if the drive can generate an IRQ when DRQ goes high.
This is good for performance, since you don't have to sit and poll the
drive for 3 milliseconds. (This code always polls for DRQ.) */
	switch((Buffer[0] >> 5) & 3)
	{case 0:
		printf("  polled DRQ\n");
		break;
	case 1:
		printf("  interrupt DRQ\n");
		break;
	case 2:
		printf("  accelerated DRQ\n");
		break; }}
#endif
	return(0); }
/*****************************************************************************
	name:	ataPoll
	action:	waits up to Timeout milliseconds for Mask bits of
		Drive's status register to have desired status (Want)
	returns:-1 if timeout
		0  if success
*****************************************************************************/
static int ataPoll(driveinfo *Drive, u16 Timeout, u8 Mask, u8 Want)
{	u8 Stat=0;
	u16 Time;

	for(Time=Timeout; Time != 0; Time--)
	{	Stat=inportb(Drive->IOAdr + ATA_REG_STAT);
		if((Stat & Mask) == Want)
			break;
		delay(1); }
	DEBUG(printf("[waited for %3u ms]", Timeout - Time);)
	if(Time != 0)
		return(0);
DEBUG(
	printf("bad status (0x%02X)\n", Stat);
	if(Stat & 1)
	{	printf("error code 0x%02X\n",
			inportb(Drive->IOAdr + ATA_REG_ERR)); }
)
	return(-1); }
/*****************************************************************************
	name:	ataSelect
	action:	selects master (Drive->Dev == 0xA0) or slave
		(Drive->Dev != 0xB0) Drive on interface Drive->IOAdr
	returns:-1 if timeout
		0  if success
*****************************************************************************/
static int ataSelect(driveinfo *Drive)
{	u8 Temp;

/* return now if drive already selected */
	Temp=inportb(Drive->IOAdr + ATA_REG_DRVHD) ^ Drive->DrvSel;
	if((Temp & 0x10) == 0)
		return(0);
/* wait up to WAIT_READY milliseconds for BSY=0 */
	if(ataPoll(Drive, WAIT_READY, 0x80, 0) != 0)
		return(-1);
/* select master/slave */
	outportb(Drive->IOAdr + ATA_REG_DRVHD, Drive->DrvSel);
/* Hale Landis: ATA-4 needs delay after ATA_REG_DRVHD written */
	nsleep(400);
/* wait up to WAIT_READY milliseconds for BSY=0. According to Hale Landis,
ATA drives can also wait for RDY=1, SKC=1 */
	return(ataPoll(Drive, WAIT_READY, 0x80, 0)); }
/*****************************************************************************
	name:	ataProbe
	action:	pokes at ATA interfaces to detect devices
*****************************************************************************/
static void ataProbe(void)
{	u8 WhichDrive, Byte1, Byte2;
	driveinfo *Drive;

/* set initial values */
	_Drives[0].DrvSel=_Drives[2].DrvSel=0xA0;
	_Drives[1].DrvSel=_Drives[3].DrvSel=0xB0;
	_Drives[0].IOAdr=_Drives[1].IOAdr=0x1F0;
	_Drives[2].IOAdr=_Drives[3].IOAdr=0x170;
	for(WhichDrive=0; WhichDrive < 4; WhichDrive += 2)
	{	Drive=_Drives + WhichDrive;
/* soft reset both drives on this I/F (selects master) */
		DEBUG(printf("  doing soft reset on I/F 0x%03X", 
			Drive->IOAdr);)
		outportb(Drive->IOAdr + ATA_REG_SLCT, 0x06);
		nsleep(400);
/* release soft reset AND enable interrupts from drive */
		outportb(Drive->IOAdr + ATA_REG_SLCT, 0x00);
		nsleep(400);
/* wait up for drive status:
 * BSY=0  DRDY=1  DF=? DSC=?  DRQ=?  CORR=?  IDX=?  ERR=0 
 * Fixed: Some ATAPI drives choke on that => discard errors */
		ataPoll(Drive, 250, 0xC1, 0x40);
/* identify master */
		printf("  hd%1u (0x%03X, master): ", WhichDrive,
			Drive->IOAdr);
		ataIdentify(Drive);
/* select slave */
		if(ataSelect(Drive + 1) != 0)
/* no slave; continue */
		{	DEBUG(printf(" no slave on I/F 0x%03X\n",
				Drive->IOAdr);)
			Drive[1].IOAdr=0;
			continue; }
/* identify slave */
		printf("  hd%1u (0x%03X, slave ): ", WhichDrive + 1,
			Drive->IOAdr);
		ataIdentify(Drive + 1); }}
/*****************************************************************************
	name:	ataWriteRegs
	action:	writes 7 bytes from Regs+1 to ATA register file at
		Drive->IOAdr
*****************************************************************************/
static void ataWriteRegs(driveinfo *Drive, u8 *Regs)
{	DEBUG(printf("ataWriteRegs:\n");)
	outportb(Drive->IOAdr + ATA_REG_FEAT, Regs[1]);
	outportb(Drive->IOAdr + ATA_REG_CNT, Regs[2]);
	outportb(Drive->IOAdr + ATA_REG_SECT, Regs[3]);
	outportb(Drive->IOAdr + ATA_REG_LOCYL, Regs[4]);
	outportb(Drive->IOAdr + ATA_REG_HICYL, Regs[5]);
	outportb(Drive->IOAdr + ATA_REG_DRVHD, Regs[6]);
	nsleep(400);
	_InterruptOccured=0;
	outportb(Drive->IOAdr + ATA_REG_CMD, Regs[7]);
	nsleep(400); }
/*****************************************************************************
	name:	atapiCmd2
	action:	writes ATA register file including packet command byte,
		busy-waits until drive ready, then writes 12-byte ATAPI
		command	packet Pkt
	returns:0  if OK
		-1 if drive could not be selected

		-3 timeout after writing pkt cmd byte (0xA0)
		-4 timeout after writing cmd pkt
		-5 data shortage (premature ATAPI_PH_DONE)
		-6 drive aborted command
		-7 bad drive phase
*****************************************************************************/
/* ATA_REG_STAT & 0x08 (DRQ)	ATA_REG_REASON		"phase"
	0				0		ATAPI_PH_ABORT
	0				1		bad
	0				2		bad
	0				3		ATAPI_PH_DONE
	8				0		ATAPI_PH_DATAOUT
	8				1		ATAPI_PH_CMDOUT
	8				2		ATAPI_PH_DATAIN
	8				3		bad
b0 of ATA_REG_REASON is C/nD (0=data, 1=command)
b1 of ATA_REG_REASON is IO (0=out to drive, 1=in from drive) */
static int atapiCmd2(driveinfo *Drive, drivecmd *Cmd, u8 *Pkt)
{	u8 Regs[8]={ 0, 0, 0, 0,/* regs 0-3: not used */
		0, 0x80,	/* regs 4-5: desired transfer count
xxx - 32768 -- what is the significance of this value?
is it the "maximum/preferred amount of data to be transferred
on each DRQ"? the ATAPI limit seems to be 65534 */
		0x40,		/* reg 6: LBA bit */
		ATA_CMD_PKT };	/* reg 7: ATA cmd byte to signal ATAPI */
	u8 Phase;

	DEBUG(printf("atapiCmd2:\n");)
/* select drive */
	if(ataSelect(Drive) != 0)
	{	DEBUG(printf("  error: could not select %s drive on I/F "
			"0x%03X\n", Drive->DrvSel == 0xA0 ? "master" :
			"slave", Drive->IOAdr);)
		return(-1); }
/* write ATA register file */
	Regs[6] |= Drive->DrvSel;
	ataWriteRegs(Drive, Regs);
/* await DRQ. This can be interrupt-driven if the drive supports it.
Look at the zeroth byte returned by ATAPI "identify device".
If bits b6:b5 are not zero, i.e. ((IdData[0] >> 5) & 3) != 0,
then you can use interrupts here instead of polling

BSY=0  DRDY=?  DF=? DSC=?  DRQ=1  CORR=?  IDX=?  ERR=? */
	if(ataPoll(Drive, 500, 0x88, 0x08) != 0)
	{	DEBUG(printf("  error: drive did not accept pkt cmd byte\n");)
		return(-3); }
/* write the ATAPI packet */
	_InterruptOccured=0;
	outsw(Drive->IOAdr + ATA_REG_DATA, (u16 *)Pkt, 6);
	while(1)
	{	u16 Count;

		DEBUG(printf("  ready to read %lu byte(s)\n", Cmd->Count);)
		if(awaitInterrupt(0xC000, WAIT_CMD) == 0)
		{	DEBUG(printf("  error: pkt cmd timed out\n");)
/* read status register to clear interrupt
xxx - do this in the handler */
			(void)inportb(Drive->IOAdr + ATA_REG_STAT);
			return(-4); }
		Phase=inportb(Drive->IOAdr + ATA_REG_STAT) & 0x08;
		Phase |= (inportb(Drive->IOAdr + ATA_REG_REASON) & 3);
		if(Phase == ATAPI_PH_DONE)
/* premature DONE phase?
could mean audio CD, or no CD in the drive */
		{	if(Cmd->Count != 0)
			{	DEBUG(printf("  error: data shortage\n");)
				return(-5); }
			return(0); }
/* drive aborted the command */
		else if(Phase == ATAPI_PH_ABORT)
		{	DEBUG(printf("  error: drive aborted cmd\n");)
			return(-6); }
		else if(Phase != ATAPI_PH_DATAIN)
/* ATAPI_PH_DATAOUT or ATAPI_PH_CMDOUT or something completely bogus */
		{	DEBUG(printf("  error: bad phase %u\n", Phase);)
			return(-7); }
/* read data, advance pointers */
		Count=atapiReadAndDiscard(Drive, Cmd->Data, 0xFFFFu);
		Cmd->Data += Count;
		Cmd->Count -= Count;
/* XXX - Count had better be a multiple of 2048... */
		Count >>= ATAPI_LG_SECTSIZE;
		Cmd->Blk += Count; }}

/*****************************************************************************
	name:	main
*****************************************************************************/
int main(int argc, char* argv[])
{	vector Vector14, Vector15;
	unsigned WhichDrive;
	driveinfo *Drive;
	union REGS Regs;

	printf("\nataProbe v1.0 - ATA/ATAPI Device Enumerator\n\n");
/* make sure it's really DOS */
	Regs.x.ax=0x1600;
	int86(0x2F, &Regs, &Regs);
	if(Regs.h.al != 0 && Regs.h.al != 0x80)
	{	printf("Detected Windows version ");
		if(Regs.h.al == 0x01 || Regs.h.al == 0xFF)
			printf("2.x");
		else printf("%u.%u", Regs.h.al, Regs.h.ah);
		printf(", aborting\n");
		return(-1); }

	if ( (argc > 2) || ((argc == 2) && (argv[1][0] == '/')))
	{
		puts("  usage: ataProbe [Name]");
		puts("");
		puts("  When optional parameter 'Name' is given, ataProbe tries to"); 
 		puts("  do a partial match against the first device beginning with");
 		puts("  the string 'Name'. The device position is then returned as");
 		puts("  errorcode (1 = Primary Master -> 4 = Secondary Slave).");
 		puts("  This can later be retrieved in a batch script to automate");
 		puts("  the flashing of an IDE device for instance.");
 		puts("  If no match is found, errorcode 255 is returned.");
		exit (255);
	}

/* install our interrupt handlers */
	SAVE_VECT(IRQ14_VECTOR, Vector14);
	SAVE_VECT(IRQ15_VECTOR, Vector15);
	SET_VECT(IRQ14_VECTOR, irq14);
	SET_VECT(IRQ15_VECTOR, irq15);
/* enable IRQ14 and IRQ15 at the 2nd 8259 PIC chip */
	outportb(0xA1, inportb(0xA1) & ~0xC0);
/* seed RNG */
	srand(time(NULL));
/* identify drives */
	ataProbe();
/* restore old interrupt handlers */	
	RESTORE_VECT(IRQ14_VECTOR, Vector14);
	RESTORE_VECT(IRQ15_VECTOR, Vector15);
/* Return the IDE position of the drive passed as parameter as errorcode */
	if (argc == 2)
	{
		for(WhichDrive=0; WhichDrive < 4; WhichDrive++)
		{	Drive=_Drives + WhichDrive;
			if(Drive->IOAdr == 0)
				continue;
			if (!strncmp(argv[1],Drive->Description,strlen(argv[1])))
			{	
				printf("\nDevice %s (%s) is %s %s\n", argv[1], 
					Drive->Description, 
					(WhichDrive<2)?"Primary":"Secondary",
					(WhichDrive%2)?"Slave":"Master");
				exit(WhichDrive+1); 
			} 
		}
		printf("\nDevice %s not found!\n", argv[1]);
		exit(255); 
	}
	exit(0);
}
