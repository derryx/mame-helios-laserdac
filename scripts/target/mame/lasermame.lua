-- license:BSD-3-Clause
-- copyright-holders:MAMEdev Team

---------------------------------------------------------------------------
--
--   arcade.lua
--
--   MAME target makefile
--
---------------------------------------------------------------------------

--------------------------------------------------
-- specify available CPU cores
---------------------------------------------------

CPUS["Z80"] = true
CPUS["KC80"] = true
CPUS["Z180"] = true
CPUS["I8085"] = true
CPUS["I8089"] = true
CPUS["M6502"] = true
--CPUS["ST2XXX"] = true
--CPUS["XAVIX"] = true
--CPUS["XAVIX2000"] = true
CPUS["H6280"] = true
CPUS["I86"] = true
CPUS["I386"] = true
CPUS["NEC"] = true
CPUS["V30MZ"] = true
CPUS["V60"] = true
CPUS["MCS48"] = true
CPUS["MCS51"] = true
CPUS["MCS96"] = true
CPUS["M6800"] = true
CPUS["M6805"] = true
CPUS["HD6309"] = true
CPUS["M6809"] = true
CPUS["KONAMI"] = true
CPUS["M680X0"] = true
CPUS["T11"] = true
CPUS["S2650"] = true
CPUS["TMS340X0"] = true
CPUS["TMS9900"] = true
CPUS["TMS9995"] = true
CPUS["TMS9900L"] = true
CPUS["Z8000"] = true
CPUS["Z8001"] = true
CPUS["TMS32010"] = true
CPUS["TMS32025"] = true
CPUS["TMS32031"] = true
CPUS["TMS32051"] = true
CPUS["TMS32082"] = true
CPUS["TMS57002"] = true
CPUS["CCPU"] = true
CPUS["ADSP21XX"] = true
CPUS["ASAP"] = true
CPUS["AM29000"] = true
CPUS["UPD7810"] = true
CPUS["ARM"] = true
CPUS["ARM7"] = true
CPUS["JAGUAR"] = true
CPUS["CUBEQCPU"] = true
CPUS["ESRIP"] = true
CPUS["MIPS1"] = true
CPUS["MIPS3"] = true
CPUS["PSX"] = true
CPUS["SH"] = true
CPUS["DSP16"] = true
CPUS["DSP32C"] = true
CPUS["PIC16C5X"] = true
CPUS["PIC16C62X"] = true
CPUS["PIC17"] = true
CPUS["G65816"] = true
CPUS["SPC700"] = true
CPUS["E1"] = true
CPUS["I860"] = true
CPUS["I960"] = true
CPUS["H8"] = true
CPUS["H8500"] = true
CPUS["V810"] = true
CPUS["M37710"] = true
CPUS["POWERPC"] = true
CPUS["SE3208"] = true
CPUS["MC68HC11"] = true
CPUS["ADSP21062"] = true
CPUS["DSP56156"] = true
CPUS["RSP"] = true
CPUS["COP400"] = true
CPUS["TLCS90"] = true
CPUS["TLCS870"] = true
CPUS["TLCS900"] = true
CPUS["MB88XX"] = true
CPUS["MB86233"] = true
CPUS["MB86235"] = true
CPUS["SSP1601"] = true
CPUS["APEXC"] = true
CPUS["CP1610"] = true
CPUS["F8"] = true
CPUS["LH5801"] = true
--CPUS["PDP1"] = true
--CPUS["TX0"] = true
CPUS["SATURN"] = true
CPUS["SC61860"] = true
CPUS["LR35902"] = true
CPUS["TMS7000"] = true
CPUS["SM8500"] = true
CPUS["MINX"] = true
CPUS["SSEM"] = true
CPUS["AVR8"] = true
--CPUS["TMS1000"] = true
CPUS["MCS40"] = true
CPUS["SUPERFX"] = true
CPUS["Z8"] = true
CPUS["I8008"] = true
CPUS["SCMP"] = true
--CPUS["MN1880"] = true
CPUS["MN10200"] = true
CPUS["COSMAC"] = true
CPUS["UNSP"] = true
CPUS["HCD62121"] = true
CPUS["PPS4"] = true
--CPUS["PPS41"] = true
CPUS["UPD7725"] = true
CPUS["HD61700"] = true
CPUS["LC8670"] = true
CPUS["SCORE"] = true
CPUS["ES5510"] = true
CPUS["SCUDSP"] = true
CPUS["IE15"] = true
CPUS["8X300"] = true
CPUS["ALTO2"] = true
--CPUS["W65816"] = true
CPUS["ARC"] = true
CPUS["ARCOMPACT"] = true
--CPUS["AMIS2000"] = true
--CPUS["UCOM4"] = true
CPUS["HMCS40"] = true
--CPUS["E0C6200"] = true
--CPUS["MELPS4"] = true
--CPUS["HPHYBRID"] = true
--CPUS["SM510"] = true
CPUS["ST62XX"] = true
CPUS["DSPP"] = true
CPUS["HPC"] = true
--CPUS["RII"] = true
--CPUS["BCP"] = true
--CPUS["CR16B"] = true
CPUS["FR"] = true
CPUS["UPD78K"] = true
CPUS["KS0164"] = true
--CPUS["COPS1"] = true
CPUS["MEG"] = true

--------------------------------------------------
-- specify available sound cores
--------------------------------------------------

SOUNDS["SAMPLES"] = true
SOUNDS["DAC"] = true
SOUNDS["DMADAC"] = true
SOUNDS["SPEAKER"] = true
SOUNDS["BEEP"] = true
SOUNDS["DISCRETE"] = true
SOUNDS["AY8910"] = true
SOUNDS["YM2151"] = true
SOUNDS["YM2203"] = true
SOUNDS["YM2413"] = true
SOUNDS["YM2608"] = true
SOUNDS["YM2610"] = true
SOUNDS["YM2610B"] = true
SOUNDS["YM2612"] = true
SOUNDS["YM3438"] = true
SOUNDS["YM3812"] = true
SOUNDS["YM3526"] = true
SOUNDS["Y8950"] = true
SOUNDS["YMF262"] = true
SOUNDS["YMF271"] = true
SOUNDS["YMF278B"] = true
SOUNDS["YMZ280B"] = true
SOUNDS["SN76477"] = true
SOUNDS["SN76496"] = true
SOUNDS["POKEY"] = true
SOUNDS["TIA"] = true
SOUNDS["NES_APU"] = true
SOUNDS["PAULA_8364"] = true
SOUNDS["ASTROCADE"] = true
SOUNDS["NAMCO"] = true
SOUNDS["NAMCO_15XX"] = true
SOUNDS["NAMCO_CUS30"] = true
SOUNDS["NAMCO_52XX"] = true
SOUNDS["NAMCO_63701X"] = true
--SOUNDS["NAMCO_163"] = true
SOUNDS["T6W28"] = true
SOUNDS["SNKWAVE"] = true
SOUNDS["C140"] = true
SOUNDS["C352"] = true
SOUNDS["TMS36XX"] = true
SOUNDS["TMS3615"] = true
SOUNDS["TMS5110"] = true
SOUNDS["TMS5220"] = true
SOUNDS["VLM5030"] = true
SOUNDS["ADPCM"] = true
SOUNDS["MSM5205"] = true
SOUNDS["MSM5232"] = true
SOUNDS["OKIM6258"] = true
SOUNDS["OKIM6295"] = true
SOUNDS["OKIM6376"] = true
SOUNDS["OKIM9810"] = true
--SOUNDS["UPD7752"] = true
SOUNDS["UPD7759"] = true
SOUNDS["HC55516"] = true
SOUNDS["TC8830F"] = true
SOUNDS["K005289"] = true
SOUNDS["K007232"] = true
SOUNDS["K051649"] = true
SOUNDS["K053260"] = true
SOUNDS["K054539"] = true
SOUNDS["K056800"] = true
SOUNDS["SEGAPCM"] = true
SOUNDS["MULTIPCM"] = true
SOUNDS["SCSP"] = true
SOUNDS["AICA"] = true
SOUNDS["RF5C68"] = true
SOUNDS["RF5C400"] = true
SOUNDS["CEM3394"] = true
SOUNDS["QSOUND"] = true
SOUNDS["QS1000"] = true
SOUNDS["SAA1099"] = true
SOUNDS["IREMGA20"] = true
SOUNDS["ES5503"] = true
SOUNDS["ES5505"] = true
SOUNDS["ES5506"] = true
SOUNDS["BSMT2000"] = true
SOUNDS["GAELCO_CG1V"] = true
SOUNDS["GAELCO_GAE1"] = true
--SOUNDS["HUC6230"] = true
SOUNDS["C6280"] = true
SOUNDS["SP0250"] = true
SOUNDS["SPU"] = true
SOUNDS["CDDA"] = true
SOUNDS["ICS2115"] = true
SOUNDS["I5000_SND"] = true
SOUNDS["ST0016"] = true
SOUNDS["SETAPCM"] = true
SOUNDS["X1_010"] = true
SOUNDS["VRENDER0"] = true
SOUNDS["VOTRAX"] = true
SOUNDS["ES8712"] = true
SOUNDS["CDP1869"] = true
SOUNDS["S14001A"] = true
SOUNDS["WAVE"] = true
SOUNDS["SID6581"] = true
SOUNDS["SID8580"] = true
SOUNDS["SP0256"] = true
SOUNDS["DIGITALKER"] = true
SOUNDS["CDP1863"] = true
SOUNDS["CDP1864"] = true
SOUNDS["ZSG2"] = true
SOUNDS["MOS656X"] = true
SOUNDS["ASC"] = true
SOUNDS["MAS3507D"] = true
SOUNDS["SOCRATES"] = true
SOUNDS["TMC0285"] = true
SOUNDS["TMS5200"] = true
SOUNDS["CD2801"] = true
SOUNDS["CD2802"] = true
SOUNDS["M58817"] = true
SOUNDS["TMC0281"] = true
SOUNDS["TMS5100"] = true
SOUNDS["TMS5110A"] = true
SOUNDS["LMC1992"] = true
SOUNDS["AWACS"] = true
SOUNDS["YMZ770"] = true
SOUNDS["MPEG_AUDIO"] = true
SOUNDS["T6721A"] = true
SOUNDS["MOS7360"] = true
SOUNDS["ESQPUMP"] = true
--SOUNDS["VRC6"] = true
SOUNDS["SB0400"] = true
SOUNDS["AC97"] = true
SOUNDS["ES1373"] = true
SOUNDS["L7A1045"] = true
SOUNDS["AD1848"] = true
--SOUNDS["UPD1771"] = true
SOUNDS["MEA8000"] = true
SOUNDS["DAC76"] = true
SOUNDS["TA7630"] = true
SOUNDS["MM5837"] = true
--SOUNDS["DAVE"] = true
SOUNDS["LC7535"] = true
--SOUNDS["UPD934G"] = true
SOUNDS["S_DSP"] = true
SOUNDS["KS0164"] = true
SOUNDS["TT5665"] = true
--SOUNDS["RP2C33_SOUND"] = true
--SOUNDS["UDA1344"] = true
SOUNDS["SWP30"] = true
SOUNDS["XT446"] = true

--------------------------------------------------
-- specify available video cores
--------------------------------------------------

VIDEOS["SEGA315_5124"] = true
VIDEOS["SEGA315_5313"] = true
--VIDEOS["AM8052"] = true
VIDEOS["BUFSPRITE"] = true
VIDEOS["BT45X"] = true
VIDEOS["BT47X"] = true
--VIDEOS["CDP1861"] = true
--VIDEOS["CDP1862"] = true
VIDEOS["CESBLIT"] = true
--VIDEOS["CRT9007"] = true
--VIDEOS["CRT9021"] = true
--VIDEOS["CRT9028"] = true
--VIDEOS["CRT9212"] = true
VIDEOS["CRTC_EGA"] = true
--VIDEOS["DL1416"] = true
VIDEOS["DM9368"] = true
VIDEOS["DP8350"] = true
--VIDEOS["EF9340_1"] = true
--VIDEOS["EF9345"] = true
--VIDEOS["EF9364"] = true
--VIDEOS["EF9365"] = true
VIDEOS["EF9369"] = true
--VIDEOS["GF4500"] = true
VIDEOS["GF7600GS"] = true
VIDEOS["EPIC12"] = true
VIDEOS["FIXFREQ"] = true
--VIDEOS["HD44102"] = true
--VIDEOS["HD44352"] = true
VIDEOS["HD44780"] = true
--VIDEOS["HD61603"] = true
VIDEOS["HD61830"] = true
VIDEOS["HD63484"] = true
--VIDEOS["HD66421"] = true
--VIDEOS["HLCD0438"] = true
--VIDEOS["HLCD0488"] = true
--VIDEOS["HLCD0515"] = true
--VIDEOS["HLCD0538"] = true
VIDEOS["HUC6202"] = true
VIDEOS["HUC6260"] = true
--VIDEOS["HUC6261"] = true
VIDEOS["HUC6270"] = true
--VIDEOS["HUC6272"] = true
VIDEOS["I4100"] = true
--VIDEOS["I8244"] = true
VIDEOS["I8275"] = true
VIDEOS["JANGOU_BLITTER"] = true
--VIDEOS["LC7582"] = true
--VIDEOS["LC7985"] = true
VIDEOS["M50458"] = true
VIDEOS["MB90082"] = true
VIDEOS["MB_VCU"] = true
VIDEOS["MC6845"] = true
--VIDEOS["MC6847"] = true
--VIDEOS["MD4330B"] = true
--VIDEOS["MM5445"] = true
--VIDEOS["MSM6222B"] = true
--VIDEOS["MSM6255"] = true
--VIDEOS["MOS6566"] = true
VIDEOS["PC_VGA"] = true
--VIDEOS["PCF2100"] = true
VIDEOS["POLY"] = true
VIDEOS["PSX"] = true
VIDEOS["RAMDAC"] = true
--VIDEOS["S2636"] = true
VIDEOS["SAA5050"] = true
--VIDEOS["SDA5708"] = true
VIDEOS["SCN2674"] = true
VIDEOS["PWM_DISPLAY"] = true
--VIDEOS["SED1200"] = true
--VIDEOS["SED1330"] = true
--VIDEOS["SED1356"] = true
--VIDEOS["SED1500"] = true
--VIDEOS["SED1520"] = true
VIDEOS["SNES_PPU"] = true
VIDEOS["STVVDP"] = true
--VIDEOS["T6963C"] = true
--VIDEOS["T6A04"] = true
VIDEOS["TLC34076"] = true
VIDEOS["TMS34061"] = true
--VIDEOS["TMS3556"] = true
VIDEOS["TMS9927"] = true
VIDEOS["TMS9928A"] = true
--VIDEOS["UPD3301"] = true
VIDEOS["UPD7220"] = true
--VIDEOS["UPD7227"] = true
VIDEOS["V9938"] = true
--VIDEOS["VIC4567"] = true
VIDEOS["VOODOO"] = true
VIDEOS["VOODOO_PCI"] = true
VIDEOS["ZEUS2"] = true
VIDEOS["PPU2C0X"] = true
VIDEOS["PS2GS"] = true
VIDEOS["PS2GIF"] = true
VIDEOS["VRENDER0"] = true

--------------------------------------------------
-- specify available machine cores
--------------------------------------------------

MACHINES["ACORN_IOC"] = true
MACHINES["ACORN_MEMC"] = true
MACHINES["ACORN_VIDC"] = true
MACHINES["AKIKO"] = true
MACHINES["ALPHA_8921"] = true
--MACHINES["AM2901B"] = true
MACHINES["ARM_IOMD"] = true
MACHINES["AUTOCONFIG"] = true
MACHINES["BUSMOUSE"] = true
MACHINES["CR511B"] = true
MACHINES["DMAC"] = true
MACHINES["GAYLE"] = true
MACHINES["2812FIFO"] = true
MACHINES["6522VIA"] = true
MACHINES["TPI6525"] = true
MACHINES["RIOT6532"] = true
MACHINES["6821PIA"] = true
MACHINES["6840PTM"] = true
--MACHINES["MPCC68561"] = true
MACHINES["ACIA6850"] = true
MACHINES["68681"] = true
MACHINES["7200FIFO"] = true
--MACHINES["8530SCC"] = true
--MACHINES["TTL7400"] = true
--MACHINES["TTL7404"] = true
MACHINES["TTL74123"] = true
MACHINES["TTL74145"] = true
MACHINES["TTL74148"] = true
MACHINES["TTL74153"] = true
MACHINES["TTL74157"] = true
--MACHINES["TTL74161"] = true
--MACHINES["TTL74164"] = true
MACHINES["TTL74165"] = true
MACHINES["TTL74166"] = true
--MACHINES["TTL74175"] = true
MACHINES["TTL74181"] = true
MACHINES["TTL74259"] = true
--MACHINES["TTL74381"] = true
MACHINES["TTL7474"] = true
MACHINES["KBDC8042"] = true
MACHINES["I8257"] = true
--MACHINES["ACIA6850"] = true
MACHINES["ADC0804"] = true
MACHINES["ADC0808"] = true
MACHINES["ADC083X"] = true
MACHINES["ADC1038"] = true
MACHINES["ADC1213X"] = true
MACHINES["AICARTC"] = true
--MACHINES["AM25S55X"] = true
--MACHINES["AM2847"] = true
--MACHINES["AM2910"] = true
MACHINES["AM53CF96"] = true
MACHINES["AM79C90"] = true
--MACHINES["AM9513"] = true
MACHINES["AM9517A"] = true
MACHINES["AMIGAFDC"] = true
MACHINES["AT_KEYBC"] = true
MACHINES["AT28C16"] = true
--MACHINES["AT28C64B"] = true
MACHINES["AT29X"] = true
MACHINES["AT45DBXX"] = true
MACHINES["ATAFLASH"] = true
MACHINES["ARM_AIC"] = true
MACHINES["AY31015"] = true
MACHINES["BANKDEV"] = true
--MACHINES["BIM68153"] = true
MACHINES["CDP1852"] = true
MACHINES["CDP1871"] = true
--MACHINES["CH376"] = true
MACHINES["CHESSMACHINE"] = true
MACHINES["CMOS40105"] = true
MACHINES["CDU76S"] = true
MACHINES["COM52C50"] = true
MACHINES["COM8116"] = true
MACHINES["CR589"] = true
--MACHINES["CS4031"] = true
--MACHINES["CS8221"] = true
MACHINES["CXD1095"] = true
MACHINES["CXD1185"] = true
MACHINES["DP8390"] = true
--MACHINES["DP8573"] = true
MACHINES["DS1204"] = true
MACHINES["DS1205"] = true
MACHINES["DS1302"] = true
--MACHINES["DS1315"] = true
MACHINES["DS1386"] = true
MACHINES["DS17X85"] = true
MACHINES["DS1994"] = true
MACHINES["DS2401"] = true
MACHINES["DS2404"] = true
MACHINES["DS75160A"] = true
MACHINES["DS75161A"] = true
--MACHINES["DS8874"] = true
MACHINES["E0516"] = true
MACHINES["E05A03"] = true
MACHINES["E05A30"] = true
MACHINES["EEPROMDEV"] = true
--MACHINES["ER1400"] = true
MACHINES["ER2055"] = true
MACHINES["F3853"] = true
--MACHINES["F4702"] = true
--MACHINES["HD63450"] = true
--MACHINES["HD64610"] = true
MACHINES["I2CMEM"] = true
--MACHINES["I80130"] = true
--MACHINES["I8089"] = true
MACHINES["I8155"] = true
MACHINES["I8212"] = true
MACHINES["I8214"] = true
MACHINES["I82355"] = true
MACHINES["I8243"] = true
MACHINES["I8251"] = true
MACHINES["I8255"] = true
--MACHINES["I8257"] = true
--MACHINES["I8271"] = true
MACHINES["I8279"] = true
MACHINES["I8355"] = true
MACHINES["IBM21S850"] = true
MACHINES["IDECTRL"] = true
MACHINES["IE15"] = true
MACHINES["IM6402"] = true
MACHINES["INS8154"] = true
MACHINES["INS8250"] = true
MACHINES["INTELFLASH"] = true
MACHINES["JVS"] = true
MACHINES["K033906"] = true
MACHINES["K053252"] = true
MACHINES["K056230"] = true
--MACHINES["KB3600"] = true
--MACHINES["KBDC8042"] = true
--MACHINES["KR2376"] = true
MACHINES["LATCH8"] = true
MACHINES["LC89510"] = true
MACHINES["LDPR8210"] = true
MACHINES["LDSTUB"] = true
MACHINES["LDV1000"] = true
MACHINES["LDP1000"] = true
MACHINES["LDP1450"] = true
MACHINES["LDVP931"] = true
--MACHINES["LH5810"] = true
MACHINES["LINFLASH"] = true
--MACHINES["LOCOMO"] = true
MACHINES["LPCI"] = true
MACHINES["LSI53C810"] = true
--MACHINES["M3002"] = true
--MACHINES["M68307"] = true
--MACHINES["M68340"] = true
MACHINES["M6M80011AP"] = true
MACHINES["MB14241"] = true
MACHINES["MB3773"] = true
MACHINES["MB8421"] = true
MACHINES["MB87078"] = true
--MACHINES["MB8795"] = true
MACHINES["MB89352"] = true
MACHINES["MB89371"] = true
MACHINES["MB89374"] = true
--MACHINES["MC14411"] = true
MACHINES["MC146818"] = true
MACHINES["MC6843"] = true
MACHINES["MC6846"] = true
MACHINES["MC6852"] = true
MACHINES["MC6854"] = true
MACHINES["MC68328"] = true
MACHINES["MC68901"] = true
MACHINES["MCCS1850"] = true
MACHINES["M68307"] = true
MACHINES["M68340"] = true
--MACHINES["M950X0"] = true
MACHINES["MCF5206E"] = true
MACHINES["METERS"] = true
MACHINES["MICROTOUCH"] = true
--MACHINES["MIOT6530"] = true
--MACHINES["MM5307"] = true
--MACHINES["MM58167"] = true
MACHINES["MM58274C"] = true
MACHINES["MM74C922"] = true
MACHINES["MOS6526"] = true
MACHINES["MOS6529"] = true
MACHINES["MIOT6530"] = true
MACHINES["MOS6551"] = true
--MACHINES["MOS6702"] = true
--MACHINES["MOS8706"] = true
--MACHINES["MOS8722"] = true
--MACHINES["MOS8726"] = true
MACHINES["MPU401"] = true
MACHINES["MSM5832"] = true
MACHINES["MSM58321"] = true
MACHINES["MSM6242"] = true
MACHINES["MSM6253"] = true
MACHINES["NCR5380N"] = true
MACHINES["NCR5390"] = true
MACHINES["NCR539x"] = true
MACHINES["NETLIST"] = true
MACHINES["NCR53C7XX"] = true
MACHINES["NMC9306"] = true
--MACHINES["NSC810"] = true
MACHINES["NSCSI"] = true
MACHINES["OUTPUT_LATCH"] = true
MACHINES["PC_FDC"] = true
MACHINES["PC_LPT"] = true
--MACHINES["PCCARD"] = true
MACHINES["PCF8583"] = true
MACHINES["PCF8584"] = true
MACHINES["PCF8593"] = true
MACHINES["PCI"] = true
MACHINES["PCKEYBRD"] = true
MACHINES["PIC8259"] = true
MACHINES["PIT68230"] = true
MACHINES["PIT8253"] = true
MACHINES["PLA"] = true
--MACHINES["PROFILE"] = true
--MACHINES["PROM82S129"] = true
MACHINES["PXA255"] = true
MACHINES["R10696"] = true
MACHINES["R10788"] = true
MACHINES["RA17XX"] = true
--MACHINES["R64H156"] = true
MACHINES["RF5C296"] = true
--MACHINES["RIOT6532"] = true
MACHINES["RIPPLE_COUNTER"] = true
MACHINES["ROC10937"] = true
MACHINES["RP5C01"] = true
MACHINES["RP5C15"] = true
MACHINES["RP5H01"] = true
MACHINES["RSTBUF"] = true
MACHINES["RTC4543"] = true
MACHINES["RTC65271"] = true
MACHINES["RTC9701"] = true
MACHINES["S_SMP"] = true
MACHINES["S2636"] = true
MACHINES["S3520CF"] = true
MACHINES["S3C24XX"] = true
--MACHINES["S3C44B0"] = true
--MACHINES["SA1110"] = true
--MACHINES["SA1111"] = true
MACHINES["SATURN"] = true
MACHINES["SCC68070"] = true
MACHINES["SCN_PCI"] = true
--MACHINES["SCOOP"] = true
MACHINES["SCSI"] = true
MACHINES["SCUDSP"] = true
MACHINES["SDA2006"] = true
--MACHINES["SECFLASH"] = true
MACHINES["SENSORBOARD"] = true
MACHINES["SERFLASH"] = true
MACHINES["SMC91C9X"] = true
MACHINES["SEGA_SCU"] = true
MACHINES["SMPC"] = true
--MACHINES["SPG2XX"] = true
MACHINES["STVCD"] = true
--MACHINES["SUN4C_MMU"] = true
MACHINES["SWTPC8212"] = true
MACHINES["TASC_SB30"] = true
MACHINES["TC0091LVC"] = true
--MACHINES["TDC1008"] = true
MACHINES["TE7750"] = true
MACHINES["TICKET"] = true
MACHINES["TIMEKPR"] = true
--MACHINES["TMC208K"] = true
MACHINES["TMP68301"] = true
--MACHINES["TMS5501"] = true
MACHINES["TMS6100"] = true
MACHINES["TMS9901"] = true
MACHINES["TMS9902"] = true
--MACHINES["TPI6525"] = true
MACHINES["TSB12LV01A"] = true
--MACHINES["TTL74123"] = true
--MACHINES["TTL74145"] = true
--MACHINES["TTL74148"] = true
--MACHINES["TTL74153"] = true
--MACHINES["TTL74181"] = true
--MACHINES["TTL7474"] = true
--MACHINES["UCB1200"] = true
MACHINES["UPD1990A"] = true
MACHINES["UPD4992"] = true
MACHINES["UPD4701"] = true
MACHINES["UPD7001"] = true
MACHINES["UPD7002"] = true
MACHINES["UPD7004"] = true
MACHINES["UPD71071"] = true
MACHINES["UPD765"] = true
MACHINES["FDC_PLL"] = true
MACHINES["V3021"] = true
MACHINES["WD_FDC"] = true
--MACHINES["WD1010"] = true
MACHINES["WD11C00_17"] = true
MACHINES["WD2010"] = true
MACHINES["WD33C9X"] = true
MACHINES["X2212"] = true
MACHINES["X76F041"] = true
MACHINES["X76F100"] = true
MACHINES["Z80CTC"] = true
MACHINES["Z80SIO"] = true
MACHINES["Z80SCC"] = true
MACHINES["Z80DMA"] = true
MACHINES["Z80PIO"] = true
MACHINES["Z80STI"] = true
MACHINES["Z8536"] = true
MACHINES["SECFLASH"] = true
MACHINES["PCCARD"] = true
MACHINES["FDC37C665GT"] = true
--MACHINES["SMC92X4"] = true
--MACHINES["TI99_HD"] = true
--MACHINES["STRATA"] = true
MACHINES["STEPPERS"] = true
--MACHINES["CORVUSHD"] = true
--MACHINES["WOZFDC"] = true
--MACHINES["APPLE_FDINTF"] = true
--MACHINES["IWM"] = true
--MACHINES["SWIM1"] = true
--MACHINES["SWIM2"] = true
--MACHINES["SWIM3"] = true
--MACHINES["MAC_VIDEO_SONORA"] = true
--MACHINES["DIABLO_HD"] = true
MACHINES["PCI9050"] = true
MACHINES["TMS1024"] = true
MACHINES["GENPC"] = true
MACHINES["GEN_LATCH"] = true
MACHINES["WATCHDOG"] = true
MACHINES["SMARTMEDIA"] = true
MACHINES["I82586"] = true
MACHINES["INPUT_MERGER"] = true
MACHINES["K054321"] = true
MACHINES["ADC0844"] = true
MACHINES["GEN_FIFO"] = true
MACHINES["Z80DAISY"] = true
--MACHINES["PS2DMAC"] = true
MACHINES["PS2INTC"] = true
--MACHINES["PS2MC"] = true
--MACHINES["PS2PAD"] = true
--MACHINES["PS2SIF"] = true
--MACHINES["PS2TIMER"] = true
--MACHINES["IOPCDVD"] = true
--MACHINES["IOPDMA"] = true
--MACHINES["IOPINTC"] = true
--MACHINES["IOPSIO2"] = true
--MACHINES["IOPTIMER"] = true
MACHINES["AIC565"] = true
MACHINES["AIC580"] = true
MACHINES["AIC6250"] = true
MACHINES["VRENDER0"] = true

--------------------------------------------------
-- specify available bus cores
--------------------------------------------------

--BUSES["A1BUS"] = true
--BUSES["A2BUS"] = true
--BUSES["A7800"] = true
--BUSES["A800"] = true
--BUSES["ABCBUS"] = true
--BUSES["ABCKB"] = true
--BUSES["ADAM"] = true
--BUSES["ADAMNET"] = true
--BUSES["ADB"] = true
--BUSES["APF"] = true
-- BUSES["AMIGA_KEYBOARD"] = true
--BUSES["ARCADIA"] = true
--BUSES["ASTROCADE"] = true
-- BUSES["ATA"] = true
--BUSES["BML3"] = true
--BUSES["BW2"] = true
--BUSES["C64"] = true
--BUSES["CBM2"] = true
--BUSES["CBMIEC"] = true
-- BUSES["CENTRONICS"] = true
--BUSES["CHANNELF"] = true
--BUSES["COCO"] = true
--BUSES["COLECO_CONTROLLER"] = true
--BUSES["COLECO_CART"] = true
--BUSES["COMPUCOLOR"] = true
--BUSES["COMX35"] = true
--BUSES["CPC"] = true
--BUSES["CRVISION"] = true
--BUSES["DMV"] = true
--BUSES["ECBBUS"] = true
--BUSES["ECONET"] = true
--BUSES["EP64"] = true
--BUSES["EPSON_SIO"] = true
--BUSES["GAMEBOY"] = true
-- BUSES["GAMEGEAR"] = true
--BUSES["GBA"] = true
BUSES["GENERIC"] = true
--BUSES["IEEE488"] = true
--BUSES["IMI7000"] = true
--BUSES["INTV"] = true
--BUSES["IQ151"] = true
-- BUSES["ISA"] = true
--BUSES["ISBX"] = true
--BUSES["KC"] = true
--BUSES["LPCI"] = true
--BUSES["MACPDS"] = true
BUSES["MIDI"] = true
--BUSES["MEGADRIVE"] = true
--BUSES["MSX_SLOT"] = true
-- BUSES["NEOGEO"] = true
-- BUSES["NEOGEO_CTRL"] = true
--BUSES["NES"] = true
-- BUSES["NSCSI"] = true
--BUSES["NUBUS"] = true
--BUSES["O2"] = true
--BUSES["ORICEXT"] = true
--BUSES["PCE"] = true
BUSES["PC_JOY"] = true
BUSES["PC_KBD"] = true
--BUSES["PET"] = true
--BUSES["PLUS4"] = true
-- BUSES["PSX_CONTROLLER"] = true
--BUSES["QL"] = true
-- BUSES["RS232"] = true
--BUSES["S100"] = true
-- BUSES["SAT_CTRL"] = true
--BUSES["SATURN"] = true
-- BUSES["SCSI"] = true
--BUSES["SCV"] = true
-- BUSES["SEGA8"] = true
--BUSES["SG1000_EXP"] = true
--BUSES["SGIKBD"] = true
--BUSES["SMS_CTRL"] = true
--BUSES["SMS_EXP"] = true
--BUSES["SNES"] = true
--BUSES["SPC1000"] = true
--BUSES["SUNKBD"] = true
--BUSES["TI99PEB"] = true
--BUSES["TVC"] = true
--BUSES["VBOY"] = true
--BUSES["VC4000"] = true
--BUSES["VCS"] = true
--BUSES["VCS_CTRL"] = true
--BUSES["VECTREX"] = true
--BUSES["VIC10"] = true
--BUSES["VIC20"] = true
--BUSES["VIDBRAIN"] = true
--BUSES["VIP"] = true
--BUSES["VME"] = true
--BUSES["VSMILE"] = true
--BUSES["VTECH_IOEXP"] = true
--BUSES["VTECH_MEMEXP"] = true
--BUSES["WANGPC"] = true
--BUSES["WSWAN"] = true
--BUSES["X68K"] = true
--BUSES["Z88"] = true
--BUSES["ZORRO"] = true
--BUSES["THOMSON"] = true


--------------------------------------------------
-- specify used file formats
--------------------------------------------------

--FORMATS["APOLLO_DSK"] = true
--FORMATS["GUAB_DSK"] = true
--FORMATS["AMI_DSK"] = true
--FORMATS["SC3000_BIT"] = true
--FORMATS["WD177X_DSK"] = true
--FORMATS["NASLITE_DSK"] = true
--FORMATS["BASICDSK"] = true
--FORMATS["IBMXDF_DSK"] = true
--FORMATS["IPF_DSK"] = true

--------------------------------------------------
-- this is the list of driver libraries that
-- comprise MAME plus mamedriv.o which contains
-- the list of drivers
--------------------------------------------------

function linkProjects_mame_lasermame(_target, _subtarget)
	links {
		"atari",
	}
end

function createMAMEProjects(_target, _subtarget, _name)
	project (_name)
	targetsubdir(_target .."_" .. _subtarget)
	kind (LIBTYPE)
	uuid (os.uuid("drv-" .. _target .."_" .. _subtarget .. "_" .._name))
	addprojectflags()
	precompiledheaders_novs()

	includedirs {
		MAME_DIR .. "src/osd",
		MAME_DIR .. "src/emu",
		MAME_DIR .. "src/devices",
		MAME_DIR .. "src/mame",
		MAME_DIR .. "src/lib",
		MAME_DIR .. "src/lib/util",
		MAME_DIR .. "3rdparty",
		GEN_DIR  .. "mame/layout",
	}
	includedirs {
		ext_includedir("flac"),
		ext_includedir("glm"),
		ext_includedir("jpeg"),
		ext_includedir("rapidjson"),
	}

end

function createProjects_mame_lasermame(_target, _subtarget)
--------------------------------------------------
-- the following files are general components and
-- shared across a number of drivers
--------------------------------------------------

createMAMEProjects(_target, _subtarget, "shared")
files {
	MAME_DIR .. "src/mame/machine/bacta_datalogger.h",
	MAME_DIR .. "src/mame/machine/bacta_datalogger.cpp",
	MAME_DIR .. "src/mame/machine/nmk112.cpp",
	MAME_DIR .. "src/mame/machine/nmk112.h",
	MAME_DIR .. "src/mame/machine/pcshare.cpp",
	MAME_DIR .. "src/mame/machine/pcshare.h",
	MAME_DIR .. "src/mame/machine/segacrpt_device.cpp",
	MAME_DIR .. "src/mame/machine/segacrpt_device.h",
	MAME_DIR .. "src/mame/video/avgdvg.cpp",
	MAME_DIR .. "src/mame/video/avgdvg.h",
	MAME_DIR .. "src/mame/video/awpvid.cpp",
	MAME_DIR .. "src/mame/video/awpvid.h",
	MAME_DIR .. "src/mame/video/tmap038.cpp",
	MAME_DIR .. "src/mame/video/tmap038.h",
	MAME_DIR .. "src/mame/audio/dcs.cpp",
	MAME_DIR .. "src/mame/audio/dcs.h",
	MAME_DIR .. "src/mame/audio/decobsmt.cpp",
	MAME_DIR .. "src/mame/audio/decobsmt.h",
	MAME_DIR .. "src/mame/audio/efo_zsu.cpp",
	MAME_DIR .. "src/mame/audio/efo_zsu.h",
	MAME_DIR .. "src/mame/audio/rax.cpp",
	MAME_DIR .. "src/mame/audio/rax.h",
	MAME_DIR .. "src/mame/audio/segam1audio.cpp",
	MAME_DIR .. "src/mame/audio/segam1audio.h",
}

--------------------------------------------------
-- manufacturer-specific groupings for drivers
--------------------------------------------------

createMAMEProjects(_target, _subtarget, "atari")
files {
	MAME_DIR .. "src/mame/drivers/akkaarrh.cpp",
	MAME_DIR .. "src/mame/drivers/arcadecl.cpp",
	MAME_DIR .. "src/mame/includes/arcadecl.h",
	MAME_DIR .. "src/mame/video/arcadecl.cpp",
	MAME_DIR .. "src/mame/drivers/asteroid.cpp",
	MAME_DIR .. "src/mame/includes/asteroid.h",
	MAME_DIR .. "src/mame/machine/asteroid.cpp",
	MAME_DIR .. "src/mame/audio/asteroid.cpp",
	MAME_DIR .. "src/mame/audio/llander.cpp",
	MAME_DIR .. "src/mame/drivers/atarifb.cpp",
	MAME_DIR .. "src/mame/includes/atarifb.h",
	MAME_DIR .. "src/mame/machine/atarifb.cpp",
	MAME_DIR .. "src/mame/audio/atarifb.cpp",
	MAME_DIR .. "src/mame/video/atarifb.cpp",
	MAME_DIR .. "src/mame/drivers/atarig1.cpp",
	MAME_DIR .. "src/mame/includes/atarig1.h",
	MAME_DIR .. "src/mame/video/atarig1.cpp",
	MAME_DIR .. "src/mame/drivers/atarig42.cpp",
	MAME_DIR .. "src/mame/includes/atarig42.h",
	MAME_DIR .. "src/mame/video/atarig42.cpp",
	MAME_DIR .. "src/mame/drivers/atarigt.cpp",
	MAME_DIR .. "src/mame/includes/atarigt.h",
	MAME_DIR .. "src/mame/video/atarigt.cpp",
	MAME_DIR .. "src/mame/drivers/atarigx2.cpp",
	MAME_DIR .. "src/mame/includes/atarigx2.h",
	MAME_DIR .. "src/mame/video/atarigx2.cpp",
	MAME_DIR .. "src/mame/drivers/atarisy1.cpp",
	MAME_DIR .. "src/mame/includes/atarisy1.h",
	MAME_DIR .. "src/mame/video/atarisy1.cpp",
	MAME_DIR .. "src/mame/drivers/atarisy2.cpp",
	MAME_DIR .. "src/mame/includes/atarisy2.h",
	MAME_DIR .. "src/mame/video/atarisy2.cpp",
	MAME_DIR .. "src/mame/drivers/atarisy4.cpp",
	MAME_DIR .. "src/mame/drivers/atarittl.cpp",
	MAME_DIR .. "src/mame/machine/nl_gtrak10.cpp",
	MAME_DIR .. "src/mame/machine/nl_gtrak10.h",
	MAME_DIR .. "src/mame/machine/nl_stuntcyc.cpp",
	MAME_DIR .. "src/mame/machine/nl_stuntcyc.h",
	MAME_DIR .. "src/mame/machine/nl_tank.cpp",
	MAME_DIR .. "src/mame/machine/nl_tank.h",
	MAME_DIR .. "src/mame/drivers/atetris.cpp",
	MAME_DIR .. "src/mame/includes/atetris.h",
	MAME_DIR .. "src/mame/video/atetris.cpp",
	MAME_DIR .. "src/mame/drivers/avalnche.cpp",
	MAME_DIR .. "src/mame/includes/avalnche.h",
	MAME_DIR .. "src/mame/audio/avalnche.cpp",
	MAME_DIR .. "src/mame/drivers/badlands.cpp",
	MAME_DIR .. "src/mame/includes/badlands.h",
	MAME_DIR .. "src/mame/machine/badlands.cpp",
	MAME_DIR .. "src/mame/video/badlands.cpp",
	MAME_DIR .. "src/mame/drivers/badlandsbl.cpp",
	MAME_DIR .. "src/mame/drivers/bartop52.cpp",
	MAME_DIR .. "src/mame/drivers/batman.cpp",
	MAME_DIR .. "src/mame/includes/batman.h",
	MAME_DIR .. "src/mame/video/batman.cpp",
	MAME_DIR .. "src/mame/drivers/beathead.cpp",
	MAME_DIR .. "src/mame/includes/beathead.h",
	MAME_DIR .. "src/mame/video/beathead.cpp",
	MAME_DIR .. "src/mame/drivers/blstroid.cpp",
	MAME_DIR .. "src/mame/includes/blstroid.h",
	MAME_DIR .. "src/mame/video/blstroid.cpp",
	MAME_DIR .. "src/mame/drivers/boxer.cpp",
	MAME_DIR .. "src/mame/drivers/bsktball.cpp",
	MAME_DIR .. "src/mame/includes/bsktball.h",
	MAME_DIR .. "src/mame/machine/bsktball.cpp",
	MAME_DIR .. "src/mame/audio/bsktball.cpp",
	MAME_DIR .. "src/mame/video/bsktball.cpp",
	MAME_DIR .. "src/mame/drivers/bwidow.cpp",
	MAME_DIR .. "src/mame/includes/bwidow.h",
	MAME_DIR .. "src/mame/audio/bwidow.cpp",
	MAME_DIR .. "src/mame/drivers/bzone.cpp",
	MAME_DIR .. "src/mame/includes/bzone.h",
	MAME_DIR .. "src/mame/audio/bzone.cpp",
	MAME_DIR .. "src/mame/drivers/canyon.cpp",
	MAME_DIR .. "src/mame/includes/canyon.h",
	MAME_DIR .. "src/mame/audio/canyon.cpp",
	MAME_DIR .. "src/mame/video/canyon.cpp",
	MAME_DIR .. "src/mame/drivers/cball.cpp",
	MAME_DIR .. "src/mame/drivers/ccastles.cpp",
	MAME_DIR .. "src/mame/includes/ccastles.h",
	MAME_DIR .. "src/mame/video/ccastles.cpp",
	MAME_DIR .. "src/mame/drivers/centiped.cpp",
	MAME_DIR .. "src/mame/includes/centiped.h",
	MAME_DIR .. "src/mame/video/centiped.cpp",
	MAME_DIR .. "src/mame/drivers/cloak.cpp",
	MAME_DIR .. "src/mame/includes/cloak.h",
	MAME_DIR .. "src/mame/video/cloak.cpp",
	MAME_DIR .. "src/mame/drivers/cloud9.cpp",
	MAME_DIR .. "src/mame/includes/cloud9.h",
	MAME_DIR .. "src/mame/video/cloud9.cpp",
	MAME_DIR .. "src/mame/drivers/cmmb.cpp",
	MAME_DIR .. "src/mame/drivers/cops.cpp",
	MAME_DIR .. "src/mame/drivers/copsnrob.cpp",
	MAME_DIR .. "src/mame/includes/copsnrob.h",
	MAME_DIR .. "src/mame/audio/copsnrob.cpp",
	MAME_DIR .. "src/mame/video/copsnrob.cpp",
	MAME_DIR .. "src/mame/drivers/cyberbal.cpp",
	MAME_DIR .. "src/mame/includes/cyberbal.h",
	MAME_DIR .. "src/mame/video/cyberbal.cpp",
	MAME_DIR .. "src/mame/drivers/cybstorm.cpp",
	MAME_DIR .. "src/mame/includes/cybstorm.h",
	MAME_DIR .. "src/mame/video/cybstorm.cpp",
	MAME_DIR .. "src/mame/audio/nl_destroyr.cpp",
	MAME_DIR .. "src/mame/audio/nl_destroyr.h",
	MAME_DIR .. "src/mame/drivers/destroyr.cpp",
	MAME_DIR .. "src/mame/drivers/dragrace.cpp",
	MAME_DIR .. "src/mame/includes/dragrace.h",
	MAME_DIR .. "src/mame/audio/dragrace.cpp",
	MAME_DIR .. "src/mame/video/dragrace.cpp",
	MAME_DIR .. "src/mame/drivers/eprom.cpp",
	MAME_DIR .. "src/mame/includes/eprom.h",
	MAME_DIR .. "src/mame/video/eprom.cpp",
	MAME_DIR .. "src/mame/drivers/firefox.cpp",
	MAME_DIR .. "src/mame/drivers/firetrk.cpp",
	MAME_DIR .. "src/mame/includes/firetrk.h",
	MAME_DIR .. "src/mame/audio/firetrk.cpp",
	MAME_DIR .. "src/mame/video/firetrk.cpp",
	MAME_DIR .. "src/mame/audio/nl_flyball.cpp",
	MAME_DIR .. "src/mame/audio/nl_flyball.h",
	MAME_DIR .. "src/mame/drivers/flyball.cpp",
	MAME_DIR .. "src/mame/drivers/foodf.cpp",
	MAME_DIR .. "src/mame/includes/foodf.h",
	MAME_DIR .. "src/mame/video/foodf.cpp",
	MAME_DIR .. "src/mame/drivers/gauntlet.cpp",
	MAME_DIR .. "src/mame/includes/gauntlet.h",
	MAME_DIR .. "src/mame/video/gauntlet.cpp",
	MAME_DIR .. "src/mame/drivers/harddriv.cpp",
	MAME_DIR .. "src/mame/includes/harddriv.h",
	MAME_DIR .. "src/mame/machine/harddriv.cpp",
	MAME_DIR .. "src/mame/audio/harddriv.cpp",
	MAME_DIR .. "src/mame/video/harddriv.cpp",
	MAME_DIR .. "src/mame/drivers/irobot.cpp",
	MAME_DIR .. "src/mame/includes/irobot.h",
	MAME_DIR .. "src/mame/machine/irobot.cpp",
	MAME_DIR .. "src/mame/video/irobot.cpp",
	MAME_DIR .. "src/mame/drivers/jaguar.cpp",
	MAME_DIR .. "src/mame/includes/jaguar.h",
	MAME_DIR .. "src/mame/audio/jaguar.cpp",
	MAME_DIR .. "src/mame/video/jag_blitter.cpp",
	MAME_DIR .. "src/mame/video/jag_blitter.h",
	MAME_DIR .. "src/mame/video/jaguar.cpp",
	MAME_DIR .. "src/mame/video/jagblit.h",
	MAME_DIR .. "src/mame/video/jagblit.hxx",
	MAME_DIR .. "src/mame/video/jagobj.hxx",
	MAME_DIR .. "src/mame/drivers/jedi.cpp",
	MAME_DIR .. "src/mame/includes/jedi.h",
	MAME_DIR .. "src/mame/audio/jedi.cpp",
	MAME_DIR .. "src/mame/video/jedi.cpp",
	MAME_DIR .. "src/mame/drivers/klax.cpp",
	MAME_DIR .. "src/mame/includes/klax.h",
	MAME_DIR .. "src/mame/video/klax.cpp",
	MAME_DIR .. "src/mame/drivers/liberatr.cpp",
	MAME_DIR .. "src/mame/includes/liberatr.h",
	MAME_DIR .. "src/mame/video/liberatr.cpp",
	MAME_DIR .. "src/mame/drivers/mediagx.cpp",
	MAME_DIR .. "src/mame/drivers/metalmx.cpp",
	MAME_DIR .. "src/mame/includes/metalmx.h",
	MAME_DIR .. "src/mame/drivers/mgolf.cpp",
	MAME_DIR .. "src/mame/drivers/mhavoc.cpp",
	MAME_DIR .. "src/mame/includes/mhavoc.h",
	MAME_DIR .. "src/mame/machine/mhavoc.cpp",
	MAME_DIR .. "src/mame/drivers/missile.cpp",
	MAME_DIR .. "src/mame/drivers/nitedrvr.cpp",
	MAME_DIR .. "src/mame/includes/nitedrvr.h",
	MAME_DIR .. "src/mame/machine/nitedrvr.cpp",
	MAME_DIR .. "src/mame/audio/nitedrvr.cpp",
	MAME_DIR .. "src/mame/video/nitedrvr.cpp",
	MAME_DIR .. "src/mame/drivers/offtwall.cpp",
	MAME_DIR .. "src/mame/includes/offtwall.h",
	MAME_DIR .. "src/mame/video/offtwall.cpp",
	MAME_DIR .. "src/mame/drivers/orbit.cpp",
	MAME_DIR .. "src/mame/includes/orbit.h",
	MAME_DIR .. "src/mame/audio/orbit.cpp",
	MAME_DIR .. "src/mame/video/orbit.cpp",
	MAME_DIR .. "src/mame/drivers/pong.cpp",
	MAME_DIR .. "src/mame/machine/nl_pongf.cpp",
	MAME_DIR .. "src/mame/machine/nl_pongf.h",
	MAME_DIR .. "src/mame/machine/nl_pongdoubles.cpp",
	MAME_DIR .. "src/mame/machine/nl_pongdoubles.h",
	MAME_DIR .. "src/mame/machine/nl_breakout.cpp",
	MAME_DIR .. "src/mame/machine/nl_breakout.h",
	MAME_DIR .. "src/mame/machine/nl_rebound.cpp",
	MAME_DIR .. "src/mame/machine/nl_rebound.h",
	MAME_DIR .. "src/mame/drivers/poolshrk.cpp",
	MAME_DIR .. "src/mame/includes/poolshrk.h",
	MAME_DIR .. "src/mame/audio/poolshrk.cpp",
	MAME_DIR .. "src/mame/video/poolshrk.cpp",
	MAME_DIR .. "src/mame/drivers/quantum.cpp",
	MAME_DIR .. "src/mame/drivers/quizshow.cpp",
	MAME_DIR .. "src/mame/drivers/rampart.cpp",
	MAME_DIR .. "src/mame/includes/rampart.h",
	MAME_DIR .. "src/mame/video/rampart.cpp",
	MAME_DIR .. "src/mame/drivers/relief.cpp",
	MAME_DIR .. "src/mame/includes/relief.h",
	MAME_DIR .. "src/mame/video/relief.cpp",
	MAME_DIR .. "src/mame/drivers/runaway.cpp",
	MAME_DIR .. "src/mame/includes/runaway.h",
	MAME_DIR .. "src/mame/video/runaway.cpp",
	MAME_DIR .. "src/mame/drivers/sbrkout.cpp",
	MAME_DIR .. "src/mame/drivers/shuuz.cpp",
	MAME_DIR .. "src/mame/includes/shuuz.h",
	MAME_DIR .. "src/mame/video/shuuz.cpp",
	MAME_DIR .. "src/mame/drivers/skullxbo.cpp",
	MAME_DIR .. "src/mame/includes/skullxbo.h",
	MAME_DIR .. "src/mame/video/skullxbo.cpp",
	MAME_DIR .. "src/mame/drivers/skydiver.cpp",
	MAME_DIR .. "src/mame/includes/skydiver.h",
	MAME_DIR .. "src/mame/audio/skydiver.cpp",
	MAME_DIR .. "src/mame/video/skydiver.cpp",
	MAME_DIR .. "src/mame/drivers/skyraid.cpp",
	MAME_DIR .. "src/mame/includes/skyraid.h",
	MAME_DIR .. "src/mame/audio/skyraid.cpp",
	MAME_DIR .. "src/mame/video/skyraid.cpp",
	MAME_DIR .. "src/mame/drivers/sprint2.cpp",
	MAME_DIR .. "src/mame/includes/sprint2.h",
	MAME_DIR .. "src/mame/audio/sprint2.cpp",
	MAME_DIR .. "src/mame/video/sprint2.cpp",
	MAME_DIR .. "src/mame/drivers/sprint4.cpp",
	MAME_DIR .. "src/mame/includes/sprint4.h",
	MAME_DIR .. "src/mame/video/sprint4.cpp",
	MAME_DIR .. "src/mame/audio/sprint4.cpp",
	MAME_DIR .. "src/mame/audio/sprint4.h",
	MAME_DIR .. "src/mame/drivers/sprint8.cpp",
	MAME_DIR .. "src/mame/includes/sprint8.h",
	MAME_DIR .. "src/mame/audio/sprint8.cpp",
	MAME_DIR .. "src/mame/video/sprint8.cpp",
	MAME_DIR .. "src/mame/drivers/starshp1.cpp",
	MAME_DIR .. "src/mame/includes/starshp1.h",
	MAME_DIR .. "src/mame/audio/starshp1.cpp",
	MAME_DIR .. "src/mame/video/starshp1.cpp",
	MAME_DIR .. "src/mame/drivers/starwars.cpp",
	MAME_DIR .. "src/mame/includes/starwars.h",
	MAME_DIR .. "src/mame/machine/starwars.cpp",
	MAME_DIR .. "src/mame/audio/starwars.cpp",
	MAME_DIR .. "src/mame/drivers/subs.cpp",
	MAME_DIR .. "src/mame/includes/subs.h",
	MAME_DIR .. "src/mame/machine/subs.cpp",
	MAME_DIR .. "src/mame/audio/subs.cpp",
	MAME_DIR .. "src/mame/video/subs.cpp",
	MAME_DIR .. "src/mame/drivers/tank8.cpp",
	MAME_DIR .. "src/mame/includes/tank8.h",
	MAME_DIR .. "src/mame/audio/tank8.cpp",
	MAME_DIR .. "src/mame/video/tank8.cpp",
	MAME_DIR .. "src/mame/drivers/tempest.cpp",
	MAME_DIR .. "src/mame/drivers/thunderj.cpp",
	MAME_DIR .. "src/mame/includes/thunderj.h",
	MAME_DIR .. "src/mame/video/thunderj.cpp",
	MAME_DIR .. "src/mame/drivers/tomcat.cpp",
	MAME_DIR .. "src/mame/drivers/toobin.cpp",
	MAME_DIR .. "src/mame/includes/toobin.h",
	MAME_DIR .. "src/mame/video/toobin.cpp",
	MAME_DIR .. "src/mame/drivers/tourtabl.cpp",
	MAME_DIR .. "src/mame/video/tia.cpp",
	MAME_DIR .. "src/mame/video/tia.h",
	MAME_DIR .. "src/mame/drivers/triplhnt.cpp",
	MAME_DIR .. "src/mame/includes/triplhnt.h",
	MAME_DIR .. "src/mame/audio/triplhnt.cpp",
	MAME_DIR .. "src/mame/video/triplhnt.cpp",
	MAME_DIR .. "src/mame/drivers/tunhunt.cpp",
	MAME_DIR .. "src/mame/includes/tunhunt.h",
	MAME_DIR .. "src/mame/video/tunhunt.cpp",
	MAME_DIR .. "src/mame/drivers/ultratnk.cpp",
	MAME_DIR .. "src/mame/includes/ultratnk.h",
	MAME_DIR .. "src/mame/video/ultratnk.cpp",
	MAME_DIR .. "src/mame/drivers/videopin.cpp",
	MAME_DIR .. "src/mame/includes/videopin.h",
	MAME_DIR .. "src/mame/audio/videopin.cpp",
	MAME_DIR .. "src/mame/video/videopin.cpp",
	MAME_DIR .. "src/mame/drivers/vindictr.cpp",
	MAME_DIR .. "src/mame/includes/vindictr.h",
	MAME_DIR .. "src/mame/video/vindictr.cpp",
	MAME_DIR .. "src/mame/drivers/wolfpack.cpp",
	MAME_DIR .. "src/mame/includes/wolfpack.h",
	MAME_DIR .. "src/mame/video/wolfpack.cpp",
	MAME_DIR .. "src/mame/drivers/xybots.cpp",
	MAME_DIR .. "src/mame/includes/xybots.h",
	MAME_DIR .. "src/mame/video/xybots.cpp",
	MAME_DIR .. "src/mame/machine/asic65.cpp",
	MAME_DIR .. "src/mame/machine/asic65.h",
	MAME_DIR .. "src/mame/machine/atarigen.cpp",
	MAME_DIR .. "src/mame/machine/atarigen.h",
	MAME_DIR .. "src/mame/machine/atariscom.cpp",
	MAME_DIR .. "src/mame/machine/atariscom.h",
	MAME_DIR .. "src/mame/machine/mathbox.cpp",
	MAME_DIR .. "src/mame/machine/mathbox.h",
	MAME_DIR .. "src/mame/machine/slapstic.cpp",
	MAME_DIR .. "src/mame/machine/slapstic.h",
	MAME_DIR .. "src/mame/machine/atarixga.cpp",
	MAME_DIR .. "src/mame/machine/atarixga.h",
	MAME_DIR .. "src/mame/audio/atarijsa.cpp",
	MAME_DIR .. "src/mame/audio/atarijsa.h",
	MAME_DIR .. "src/mame/audio/atarisac.cpp",
	MAME_DIR .. "src/mame/audio/atarisac.h",
	MAME_DIR .. "src/mame/audio/cage.cpp",
	MAME_DIR .. "src/mame/audio/cage.h",
	MAME_DIR .. "src/mame/audio/redbaron.cpp",
	MAME_DIR .. "src/mame/audio/redbaron.h",
	MAME_DIR .. "src/mame/video/atarimo.cpp",
	MAME_DIR .. "src/mame/video/atarimo.h",
	MAME_DIR .. "src/mame/video/atarirle.cpp",
	MAME_DIR .. "src/mame/video/atarirle.h",
	MAME_DIR .. "src/mame/video/atarivad.cpp",
	MAME_DIR .. "src/mame/video/atarivad.h",
}
end
