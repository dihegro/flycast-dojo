#include <array>
#include "spg.h"
#include "hw/holly/holly_intc.h"
#include "hw/holly/sb.h"
#include "hw/sh4/sh4_sched.h"
#include "oslib/oslib.h"
#include "hw/maple/maple_if.h"
#include "serialize.h"
#include "network/ggpo.h"
#include "hw/pvr/Renderer_if.h"

#ifdef TEST_AUTOMATION
#include "input/gamepad_device.h"
#endif

// SPG emulation - Scanline/Raster beam registers & interrupts
// Adjusted for proper console-like synchronization

// Synchronization constants
constexpr double TARGET_SPEED = 1.0;          // 100% of original console speed
constexpr double MAX_SPEED_VARIATION = 0.02;  // 2% speed tolerance
constexpr u32 FRAME_SYNC_CYCLES = 2000000;    // Cycles between sync checks

static u32 clc_pvr_scanline;                  // Current scanline counter
static u32 pvr_numscanlines = 512;            // Total scanlines per frame
static u32 prv_cur_scanline = -1;             // Current scanline number
static u32 vblk_cnt;                          // VBlank counter

#if !defined(NDEBUG) || defined(DEBUGFAST)
static float last_fps;                         // Last FPS measurement time
#endif

// 27 MHz pixel clock (Dreamcast standard)
constexpr int PIXEL_CLOCK = 27 * 1000 * 1000;
static u32 Line_Cycles;                       // Cycles per scanline
static u32 Frame_Cycles;                      // Cycles per full frame
int render_end_schid;                         // Scheduler ID for render end
int vblank_schid;                             // Scheduler ID for VBlank

static std::array<double, 4> real_times;      // Real time measurements
static std::array<u64, 4> cpu_cycles;         // CPU cycle measurements
static u32 cpu_time_idx;                      // Current measurement index
bool SH4FastEnough;                           // Flag if SH4 is keeping up
u32 fskip;                                    // Frames skipped counter

static u32 lightgun_line = 0xffff;            // Lightgun scanline position
static u32 lightgun_hpos;                     // Lightgun horizontal position
static bool maple_int_pending;                // Maple bus interrupt flag

// Calculate video timing parameters based on current settings
void CalculateSync()
{
    // Determine pixel clock (adjusted for VCLK divider)
    u32 pixel_clock = PIXEL_CLOCK / (FB_R_CTRL.vclk_div ? 1 : 2);

    // Set total scanlines (from vertical counter)
    pvr_numscanlines = SPG_LOAD.vcount + 1;

    // Calculate cycles per scanline based on horizontal resolution
    Line_Cycles = (u32)((u64)SH4_MAIN_CLOCK * (u64)(SPG_LOAD.hcount + 1) / (u64)pixel_clock);
    
    // Halve cycles for interlaced modes
    if (SPG_CONTROL.interlace)
        Line_Cycles /= 2;

    // Total cycles per frame
    Frame_Cycles = pvr_numscanlines * Line_Cycles;
    
    // Reset scanline counters
    prv_cur_scanline = 0;
    clc_pvr_scanline = 0;

    // Schedule first VBlank interrupt
    sh4_sched_request(vblank_schid, Line_Cycles);
}

// Determine when next SPG interrupt should occur
static int getNextSpgInterrupt()
{
    // Special case for mode 2 HBlank interrupts
    if (SPG_HBLANK_INT.hblank_int_mode == 2)
        return Line_Cycles;

    u32 min_scanline = prv_cur_scanline + 1;
    u32 min_active = pvr_numscanlines;

    // Find earliest upcoming event
    if (min_scanline <= SPG_VBLANK_INT.vblank_in_interrupt_line_number)
        min_active = std::min(min_active, SPG_VBLANK_INT.vblank_in_interrupt_line_number);

    if (min_scanline <= SPG_VBLANK_INT.vblank_out_interrupt_line_number)
        min_active = std::min(min_active, SPG_VBLANK_INT.vblank_out_interrupt_line_number);

    if (min_scanline <= SPG_VBLANK.vstart)
        min_active = std::min(min_active, SPG_VBLANK.vstart);

    if (min_scanline <= SPG_VBLANK.vbend)
        min_active = std::min(min_active, SPG_VBLANK.vbend);

    if (lightgun_line != 0xffff && min_scanline <= lightgun_line)
        min_active = std::min(min_active, lightgun_line);

    if (SPG_HBLANK_INT.hblank_int_mode == 0 && min_scanline <= SPG_HBLANK_INT.line_comp_val)
        min_active = std::min(min_active, SPG_HBLANK_INT.line_comp_val);

    min_active = std::max(min_active, min_scanline);

    return (min_active - prv_cur_scanline) * Line_Cycles;
}

// Reschedule SPG events
void rescheduleSPG()
{
    sh4_sched_request(vblank_schid, getNextSpgInterrupt());
}

// Main scanline scheduler - handles timing and interrupts
static int spg_line_sched(int tag, int cycles, int jitter)
{
    static u32 last_sync = 0;  // Last synchronization point
    
    // Add elapsed cycles
    clc_pvr_scanline += cycles + jitter;

    // Speed control - only check periodically
    if (sh4_sched_now64() - last_sync > FRAME_SYNC_CYCLES)
    {
        double current_speed = ... // Calculate current emulation speed
        
        // If running too fast, insert delay
        if (current_speed > TARGET_SPEED + MAX_SPEED_VARIATION)
        {
            u32 wait = (u32)((current_speed - TARGET_SPEED) * FRAME_SYNC_CYCLES);
            sh4_sched_request(vblank_schid, wait);
            return wait;
        }
        last_sync = sh4_sched_now64();
    }

    // Process completed scanlines
    while (clc_pvr_scanline >= Line_Cycles)
    {
        prv_cur_scanline = (prv_cur_scanline + 1) % pvr_numscanlines;
        clc_pvr_scanline -= Line_Cycles;
        
        // Handle VBlank in interrupt
        if (SPG_VBLANK_INT.vblank_in_interrupt_line_number == prv_cur_scanline)
        {
            if (maple_int_pending)
            {
                maple_int_pending = false;
                SB_MDST = 0;
            }
            asic_RaiseInterrupt(holly_SCANINT1);
        }

        // Handle VBlank out interrupt
        if (SPG_VBLANK_INT.vblank_out_interrupt_line_number == prv_cur_scanline)
        {
            maple_vblank();
            asic_RaiseInterrupt(holly_SCANINT2);
        }

        // VSync start/end
        if (SPG_VBLANK.vstart == prv_cur_scanline)
            SPG_STATUS.vsync = 1;

        if (SPG_VBLANK.vbend == prv_cur_scanline)
            SPG_STATUS.vsync = 0;

        SPG_STATUS.scanline = prv_cur_scanline;
        
        // HBlank interrupts
        switch (SPG_HBLANK_INT.hblank_int_mode)
        {
        case 0:
            if (prv_cur_scanline == SPG_HBLANK_INT.line_comp_val)
                asic_RaiseInterrupt(holly_HBLank);
            break;
        case 2:
            asic_RaiseInterrupt(holly_HBLank);
            break;
        default:
            die("Unimplemented HBLANK INT mode");
            break;
        }

        // Start of new frame (scanline 0)
        if (prv_cur_scanline == 0)
        {
            // Handle interlacing
            if (SPG_CONTROL.interlace)
                SPG_STATUS.fieldnum = ~SPG_STATUS.fieldnum;
            else
                SPG_STATUS.fieldnum = 0;

            // Notify renderer of VBlank
            rend_vblank();

            // Calculate emulation speed
            double now = os_GetSeconds() * 1000000.0;
            cpu_time_idx = (cpu_time_idx + 1) % cpu_cycles.size();
            if (cpu_cycles[cpu_time_idx] != 0)
            {
                u32 cycle_span = (u32)(sh4_sched_now64() - cpu_cycles[cpu_time_idx]);
                double time_span = now - real_times[cpu_time_idx];
                double cpu_speed = ((double)cycle_span / time_span) / (SH4_MAIN_CLOCK / 100000000);
                SH4FastEnough = cpu_speed >= 85.0;
            }
            else
            {
                SH4FastEnough = false;
            }
            cpu_cycles[cpu_time_idx] = sh4_sched_now64();
            real_times[cpu_time_idx] = now;

#ifdef TEST_AUTOMATION
            replay_input();
#endif

#if !defined(NDEBUG) || defined(DEBUGFAST)
            // Performance statistics logging
            vblk_cnt++;
            if ((os_GetSeconds()-last_fps)>2)
            {
                static int Last_FC;
                double ts=os_GetSeconds()-last_fps;
                double spd_fps=(FrameCount-Last_FC)/ts;
                double spd_vbs=vblk_cnt/ts;
                double spd_cpu=spd_vbs*Frame_Cycles;
                spd_cpu/=1000000;    // Convert to MHz
                double fullvbs=(spd_vbs/spd_cpu)*200;

                Last_FC=FrameCount;
                vblk_cnt=0;

                // Determine video mode
                const char* mode=0;
                const char* res=0;

                res=SPG_CONTROL.interlace?"480i":"240p";

                if (SPG_CONTROL.NTSC==0 && SPG_CONTROL.PAL==1)
                    mode="PAL";
                else if (SPG_CONTROL.NTSC==1 && SPG_CONTROL.PAL==0)
                    mode="NTSC";
                else
                {
                    res=SPG_CONTROL.interlace?"480i":"480p";
                    mode="VGA";
                }

                // Calculate frame timing
                double frames_done=spd_cpu/2;
                double mspdf=1/frames_done*1000;

                double full_rps = spd_fps + fskip / ts;

                // Log performance data
                INFO_LOG(COMMON, "%s/%c - %4.2f - %4.2f - V: %4.2f (%.2f, %s%s%4.2f) R: %4.2f+%4.2f",
                    VER_SHORTNAME,'n',mspdf,spd_cpu*100/200,spd_vbs,
                    spd_vbs/full_rps,mode,res,fullvbs,
                    spd_fps,fskip/ts);
                
                fskip=0;
                last_fps=os_GetSeconds();
            }
#endif
        }

        // Lightgun position handling
        if (lightgun_line != 0xffff && lightgun_line == prv_cur_scanline)
        {
            maple_int_pending = false;
            SPG_TRIGGER_POS = ((lightgun_line & 0x3FF) << 16) | (lightgun_hpos & 0x3FF);
            SB_MDST = 0;
            lightgun_line = 0xffff;
        }
    }

    return getNextSpgInterrupt();
}

void read_lightgun_position(int x, int y)
{
	static u8 flip;
	maple_int_pending = true;
	if (y < 0 || y >= 480 || x < 0 || x >= 640)
	{
		// Off screen
		lightgun_line = 0xffff;
	}
	else
	{
		lightgun_line = y / (SPG_CONTROL.interlace ? 2 : 1) + SPG_VBLANK_INT.vblank_out_interrupt_line_number;
		// For some reason returning the same position twice makes it register off screen
		lightgun_hpos = (x + 286) ^ flip;
		flip ^= 1;
	}
}

bool spg_Init()
{
	render_end_schid = sh4_sched_register(0, &rend_end_render);
	vblank_schid = sh4_sched_register(0, &spg_line_sched);

	return true;
}

void spg_Term()
{
	sh4_sched_unregister(render_end_schid);
	render_end_schid = -1;
	sh4_sched_unregister(vblank_schid);
	vblank_schid = -1;
}

void spg_Reset(bool hard)
{
	CalculateSync();

	SH4FastEnough = false;
	cpu_time_idx = 0;
	cpu_cycles.fill(0);
	real_times.fill(0.0);
}

void scheduleRenderDone(TA_context *cntx)
{
	int cycles = 4096;
	if (cntx != nullptr)
	{
		if (settings.platform.isNaomi2()) {
			cycles = 1500000;
		}
		else
		{
			int size = 0;
			for (TA_context *c = cntx; c != nullptr; c = c->nextContext)
				size += c->tad.thd_data - c->tad.thd_root;
			cycles = std::min(550000 + size * 100, 1500000);
		}
	}
	sh4_sched_request(render_end_schid, cycles);
}

void spg_Serialize(Serializer& ser)
{
	ser << clc_pvr_scanline;
	ser << maple_int_pending;
	ser << pvr_numscanlines;
	ser << prv_cur_scanline;
	ser << Line_Cycles;
	ser << Frame_Cycles;
	ser << lightgun_line;
	ser << lightgun_hpos;
}
void spg_Deserialize(Deserializer& deser)
{
	if (deser.version() < Deserializer::V30)
		deser.skip<u32>(); // in_vblank
	deser >> clc_pvr_scanline;
	if (deser.version() < Deserializer::V9_LIBRETRO)
	{
		deser >> pvr_numscanlines;
		deser >> prv_cur_scanline;
		deser >> vblk_cnt;
		deser >> Line_Cycles;
		deser >> Frame_Cycles;
		deser.skip<double>();	// speed_load_mspdf
		deser.skip<u32>();		// mips_counter
		deser.skip<double>();	// full_rps
		if (deser.version() <= Deserializer::V4)
			deser.skip<u32>();	// fskip
	}
	else if (deser.version() >= Deserializer::V12)
	{
		deser >> maple_int_pending;
		if (deser.version() >= Deserializer::V14)
		{
			deser >> pvr_numscanlines;
			deser >> prv_cur_scanline;
			deser >> Line_Cycles;
			deser >> Frame_Cycles;
			deser >> lightgun_line;
			deser >> lightgun_hpos;
		}
	}
	if (deser.version() < Deserializer::V14)
		CalculateSync();
}
