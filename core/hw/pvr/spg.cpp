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

//SPG emulation; Scanline/Raster beam registers & interrupts

static u32 clc_pvr_scanline;
static u32 pvr_numscanlines = 512;
static u32 prv_cur_scanline = -1;
static u32 vblk_cnt;

#if !defined(NDEBUG) || defined(DEBUGFAST)
static float last_fps;
#endif

// 27 mhz pixel clock
constexpr int PIXEL_CLOCK = 27 * 1000 * 1000;
static u32 Line_Cycles;
static u32 Frame_Cycles;
int render_end_schid;
int vblank_schid;

static std::array<double, 4> real_times;
static std::array<u64, 4> cpu_cycles;
static u32 cpu_time_idx;
bool SH4FastEnough;
u32 fskip;

// Variáveis para controle de timing estável
static double frame_time_target = 1.0 / 60.0; // 60 FPS para NTSC, 50 para PAL
static double last_frame_time = 0.0;
static double accumulated_time = 0.0;
static bool timing_stable_mode = true; // Flag para ativar modo de timing estável
static double timing_adjustment_factor = 1.0;

static u32 lightgun_line = 0xffff;
static u32 lightgun_hpos;
static bool maple_int_pending;

void CalculateSync()
{
	u32 pixel_clock = PIXEL_CLOCK / (FB_R_CTRL.vclk_div ? 1 : 2);

	// Determinar framerate correto baseado no modo de vídeo
	if (SPG_CONTROL.NTSC == 0 && SPG_CONTROL.PAL == 1) {
		frame_time_target = 1.0 / 50.0; // PAL 50Hz
	} else {
		frame_time_target = 1.0 / 60.0; // NTSC 60Hz ou VGA
	}

	pvr_numscanlines = SPG_LOAD.vcount + 1;

	Line_Cycles = (u32)((u64)SH4_MAIN_CLOCK * (u64)(SPG_LOAD.hcount + 1) / (u64)pixel_clock);
	if (SPG_CONTROL.interlace)
		Line_Cycles /= 2;

	Frame_Cycles = pvr_numscanlines * Line_Cycles;
	prv_cur_scanline = 0;
	clc_pvr_scanline = 0;

	sh4_sched_request(vblank_schid, Line_Cycles);
}

static int getNextSpgInterrupt()
{
	if (SPG_HBLANK_INT.hblank_int_mode == 2)
		return Line_Cycles;

	u32 min_scanline = prv_cur_scanline + 1;
	u32 min_active = pvr_numscanlines;

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

	u32 cycles = (min_active - prv_cur_scanline) * Line_Cycles;
	
	// Aplicar ajuste de timing se necessário
	if (timing_stable_mode && timing_adjustment_factor != 1.0) {
		cycles = (u32)(cycles * timing_adjustment_factor);
	}

	return cycles;
}

void rescheduleSPG()
{
	sh4_sched_request(vblank_schid, getNextSpgInterrupt());
}

// Função para calcular ajuste de timing baseado na performance real
static void calculateTimingAdjustment()
{
	if (!timing_stable_mode) return;
	
	double current_time = os_GetSeconds();
	if (last_frame_time > 0.0) {
		double actual_frame_time = current_time - last_frame_time;
		double target_time = frame_time_target;
		
		// Calcular diferença entre tempo real e tempo alvo
		double time_diff = actual_frame_time - target_time;
		accumulated_time += time_diff;
		
		// Ajustar apenas se a diferença acumulada for significativa
		if (std::abs(accumulated_time) > target_time * 0.1) { // 10% de tolerância
			timing_adjustment_factor = 1.0 + (accumulated_time / target_time) * 0.1;
			timing_adjustment_factor = std::max(0.5, std::min(2.0, timing_adjustment_factor));
			accumulated_time *= 0.5; // Reduzir acúmulo gradualmente
		}
	}
	last_frame_time = current_time;
}

static int spg_line_sched(int tag, int cycles, int jitter)
{
	clc_pvr_scanline += cycles + jitter;

	while (clc_pvr_scanline >= Line_Cycles)
	{
		prv_cur_scanline = (prv_cur_scanline + 1) % pvr_numscanlines;
		clc_pvr_scanline -= Line_Cycles;
		
		if (SPG_VBLANK_INT.vblank_in_interrupt_line_number == prv_cur_scanline)
		{
			if (maple_int_pending)
			{
				maple_int_pending = false;
				SB_MDST = 0;
			}
			asic_RaiseInterrupt(holly_SCANINT1);
		}

		if (SPG_VBLANK_INT.vblank_out_interrupt_line_number == prv_cur_scanline)
		{
			maple_vblank();
			asic_RaiseInterrupt(holly_SCANINT2);
		}

		if (SPG_VBLANK.vstart == prv_cur_scanline)
			SPG_STATUS.vsync = 1;

		if (SPG_VBLANK.vbend == prv_cur_scanline)
			SPG_STATUS.vsync = 0;

		SPG_STATUS.scanline = prv_cur_scanline;
		
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

		// Vblank
		if (prv_cur_scanline == 0)
		{
			if (SPG_CONTROL.interlace)
				SPG_STATUS.fieldnum = ~SPG_STATUS.fieldnum;
			else
				SPG_STATUS.fieldnum = 0;

			// Calcular ajuste de timing antes do vblank
			calculateTimingAdjustment();

			rend_vblank();

			double now = os_GetSeconds() * 1000000.0;
			cpu_time_idx = (cpu_time_idx + 1) % cpu_cycles.size();
			if (cpu_cycles[cpu_time_idx] != 0)
			{
				u32 cycle_span = (u32)(sh4_sched_now64() - cpu_cycles[cpu_time_idx]);
				double time_span = now - real_times[cpu_time_idx];
				double cpu_speed = ((double)cycle_span / time_span) / (SH4_MAIN_CLOCK / 100000000);
				
				// Ajustar threshold baseado no modo de timing estável
				double speed_threshold = timing_stable_mode ? 75.0 : 85.0;
				SH4FastEnough = cpu_speed >= speed_threshold;
			}
			else
				SH4FastEnough = false;
			cpu_cycles[cpu_time_idx] = sh4_sched_now64();
			real_times[cpu_time_idx] = now;

#ifdef TEST_AUTOMATION
			replay_input();
#endif

#if !defined(NDEBUG) || defined(DEBUGFAST)
			vblk_cnt++;
			if ((os_GetSeconds()-last_fps)>2)
			{
				static int Last_FC;
				double ts=os_GetSeconds()-last_fps;
				double spd_fps=(FrameCount-Last_FC)/ts;
				double spd_vbs=vblk_cnt/ts;
				double spd_cpu=spd_vbs*Frame_Cycles;
				spd_cpu/=1000000;	//mrhz kthx
				double fullvbs=(spd_vbs/spd_cpu)*200;

				Last_FC=FrameCount;

				vblk_cnt=0;

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

				double frames_done=spd_cpu/2;
				double mspdf=1/frames_done*1000;

				double full_rps = spd_fps + fskip / ts;

				INFO_LOG(COMMON, "%s/%c - %4.2f - %4.2f - V: %4.2f (%.2f, %s%s%4.2f) R: %4.2f+%4.2f | TAdj: %.3f",
					VER_SHORTNAME,'n',mspdf,spd_cpu*100/200,spd_vbs,
					spd_vbs/full_rps,mode,res,fullvbs,
					spd_fps,fskip/ts,timing_adjustment_factor);
				
				fskip=0;
				last_fps=os_GetSeconds();
			}
#endif
		}
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

// Função para ativar/desativar modo de timing estável
void spg_set_stable_timing(bool enable)
{
	timing_stable_mode = enable;
	if (!enable) {
		timing_adjustment_factor = 1.0;
		accumulated_time = 0.0;
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
	
	// Reset timing control variables
	last_frame_time = 0.0;
	accumulated_time = 0.0;
	timing_adjustment_factor = 1.0;
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
	
	// Aplicar ajuste de timing ao render
	if (timing_stable_mode && timing_adjustment_factor != 1.0) {
		cycles = (int)(cycles * timing_adjustment_factor);
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
	ser << timing_stable_mode;
	ser << timing_adjustment_factor;
	ser << accumulated_time;
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
			
			// Carregar novas variáveis de timing se disponíveis
			if (deser.version() >= Deserializer::V31) // Assumindo nova versão
			{
				deser >> timing_stable_mode;
				deser >> timing_adjustment_factor;
				deser >> accumulated_time;
			}
			else
			{
				// Valores padrão para saves mais antigos
				timing_stable_mode = true;
				timing_adjustment_factor = 1.0;
				accumulated_time = 0.0;
			}
		}
	}
	if (deser.version() < Deserializer::V14)
		CalculateSync();
}
