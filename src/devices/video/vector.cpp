// license:BSD-3-Clause
// copyright-holders:Brad Oliver,Aaron Giles,Bernd Wiebelt,Allard van der Bas
/******************************************************************************
 *
 * vector.c
 *
 *        anti-alias code by Andrew Caldwell
 *        (still more to add)
 *
 * 040227 Fixed miny clip scaling which was breaking in mhavoc. AREK
 * 010903 added support for direct RGB modes MLR
 * 980611 use translucent vectors. Thanks to Peter Hirschberg
 *        and Neil Bradley for the inspiration. BW
 * 980307 added cleverer dirty handling. BW, ASG
 *        fixed antialias table .ac
 * 980221 rewrote anti-alias line draw routine
 *        added inline assembly multiply fuction for 8086 based machines
 *        beam diameter added to draw routine
 *        beam diameter is accurate in anti-alias line draw (Tcosin)
 *        flicker added .ac
 * 980203 moved LBO's routines for drawing into a buffer of vertices
 *        from avgdvg.c to this location. Scaling is now initialized
 *        by calling vector_init(...). BW
 * 980202 moved out of msdos.c ASG
 * 980124 added anti-alias line draw routine
 *        modified avgdvg.c and sega.c to support new line draw routine
 *        added two new tables Tinten and Tmerge (for 256 color support)
 *        added find_color routine to build above tables .ac
 *
 * Vector Team
 *
 *        Brad Oliver
 *        Aaron Giles
 *        Bernd Wiebelt
 *        Allard van der Bas
 *        Al Kossow (VECSIM)
 *        Hedley Rainnie (VECSIM)
 *        Eric Smith (VECSIM)
 *        Neil Bradley (technical advice)
 *        Andrew Caldwell (anti-aliasing)
 *
 **************************************************************************** */

#include "emu.h"
#include "vector.h"

#include "emuopts.h"
#include "render.h"
#include "helios-dac/HeliosDac.h"
#include <iostream>

#define VECTOR_WIDTH_DENOM 512

// 20000 is needed for mhavoc (see MT 06668) 10000 is enough for other games
#define MAX_POINTS 20000

//#define HELIOS_FLAGS (HELIOS_FLAGS_DONT_BLOCK | HELIOS_FLAGS_SINGLE_MODE | HELIOS_FLAGS_START_IMMEDIATELY)
#define HELIOS_FLAGS (HELIOS_FLAGS_SINGLE_MODE | HELIOS_FLAGS_DONT_BLOCK)
#define HELIOS_DEVICE 0
#define HELIOS_PPS 5000
// #define DEBUG_MAXXY

float vector_options::s_flicker = 0.0f;
float vector_options::s_beam_width_min = 0.0f;
float vector_options::s_beam_width_max = 0.0f;
float vector_options::s_beam_dot_size = 0.0f;
float vector_options::s_beam_intensity_weight = 0.0f;

// helios dac supports 12bit = 4095 as maximum value -> so scale to this
constexpr float helios_scale = 1.8f/32768.0f;

std::unique_ptr<HeliosPoint[]> heliosPoints;
HeliosDac heliosDac;
int numHeliosDevs;

void vector_options::init(emu_options& options)
{
	s_beam_width_min = options.beam_width_min();
	s_beam_width_max = options.beam_width_max();
	s_beam_dot_size = options.beam_dot_size();
	s_beam_intensity_weight = options.beam_intensity_weight();
	s_flicker = options.flicker();
}

// device type definition
DEFINE_DEVICE_TYPE(VECTOR, vector_device, "vector_device", "VECTOR")

vector_device::vector_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, VECTOR, tag, owner, clock),
		device_video_interface(mconfig, *this),
		m_vector_list(nullptr),
		m_min_intensity(255),
		m_max_intensity(0)
{
    numHeliosDevs =  heliosDac.OpenDevices();
}

vector_device::~vector_device() {
    heliosDac.CloseDevices();
}

void vector_device::device_start()
{
	vector_options::init(machine().options());

	m_vector_index = 0;

	/* allocate memory for tables */
	m_vector_list = std::make_unique<point[]>(MAX_POINTS);
	heliosPoints = std::make_unique<HeliosPoint[]>(HELIOS_MAX_POINTS);
}

/*
 * www.dinodini.wordpress.com/2010/04/05/normalized-tunable-sigmoid-functions/
 */
float vector_device::normalized_sigmoid(float n, float k)
{
	// valid for n and k in range of -1.0 and 1.0
	return (n - n * k) / (k - fabs(n) * 2.0f * k + 1.0f);
}


/*
 * Adds a line end point to the vertices list. The vector processor emulation
 * needs to call this.
 */
void vector_device::add_point(int x, int y, rgb_t color, int intensity)
{
	point *newpoint;

	intensity = std::max(0, std::min(255, intensity));

	m_min_intensity = intensity > 0 ? std::min(m_min_intensity, intensity) : m_min_intensity;
	m_max_intensity = intensity > 0 ? std::max(m_max_intensity, intensity) : m_max_intensity;

	if (vector_options::s_flicker && (intensity > 0))
	{
		float random = (float)(machine().rand() & 255) / 255.0f; // random value between 0.0 and 1.0

		intensity -= (int)(intensity * random * vector_options::s_flicker);

		intensity = std::max(0, std::min(255, intensity));
	}

	newpoint = &m_vector_list[m_vector_index];
	newpoint->x = x;
	newpoint->y = y;
	newpoint->col = color;
	newpoint->intensity = intensity;

	m_vector_index++;
	if (m_vector_index >= MAX_POINTS)
	{
		m_vector_index--;
		logerror("*** Warning! Vector list overflow!\n");
	}
}


/*
 * The vector CPU creates a new display list. We save the old display list,
 * but only once per refresh.
 */
void vector_device::clear_list(void)
{
	m_vector_index = 0;
}


uint32_t vector_device::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	uint32_t flags = PRIMFLAG_ANTIALIAS(1) | PRIMFLAG_BLENDMODE(BLENDMODE_ADD) | PRIMFLAG_VECTOR(1);
	const rectangle &visarea = screen.visible_area();
	float xscale = 1.0f / (65536 * visarea.width());
	float yscale = 1.0f / (65536 * visarea.height());
	float xoffs = (float)visarea.min_x;
	float yoffs = (float)visarea.min_y;

#ifdef DEBUG_MAXXY
    uint16_t minx=std::numeric_limits<uint16_t>::max();
    uint16_t maxx=std::numeric_limits<uint16_t>::min();
    uint16_t miny=std::numeric_limits<uint16_t>::max();
    uint16_t maxy=std::numeric_limits<uint16_t>::min();
    uint8_t maxi=std::numeric_limits<uint8_t>::min();
#endif

	point *curpoint;
	int lastx = 0;
	int lasty = 0;
	int lasti = 0;
	int helios_dac_index = 0;

	curpoint = m_vector_list.get();

	screen.container().empty();
	screen.container().add_rect(0.0f, 0.0f, 1.0f, 1.0f, rgb_t(0xff,0x00,0x00,0x00), PRIMFLAG_BLENDMODE(BLENDMODE_ALPHA) | PRIMFLAG_VECTORBUF(1));

	for (int i = 0; i < m_vector_index; i++)
	{
		render_bounds coords;

		float intensity = (float)curpoint->intensity / 255.0f;
		float intensity_weight = normalized_sigmoid(intensity, vector_options::s_beam_intensity_weight);

		// check for static intensity
		float beam_width = m_min_intensity == m_max_intensity
			? vector_options::s_beam_width_min
			: vector_options::s_beam_width_min + intensity_weight * (vector_options::s_beam_width_max - vector_options::s_beam_width_min);

		// normalize width
		beam_width *= 1.0f / (float)VECTOR_WIDTH_DENOM;

		// apply point scale for points
		if (lastx == curpoint->x && lasty == curpoint->y)
			beam_width *= vector_options::s_beam_dot_size;

		coords.x0 = ((float)lastx - xoffs) * xscale;
		coords.y0 = ((float)lasty - yoffs) * yscale;
		coords.x1 = ((float)curpoint->x - xoffs) * xscale;
		coords.y1 = ((float)curpoint->y - yoffs) * yscale;

		if (curpoint->intensity != 0)
		{
			screen.container().add_line(
				coords.x0, coords.y0, coords.x1, coords.y1,
				beam_width,
				(curpoint->intensity << 24) | (curpoint->col & 0xffffff),
				flags);
        }

        auto dacpoint = &heliosPoints[helios_dac_index];
        dacpoint->x = (std::uint16_t)((float)curpoint->x * helios_scale);
        dacpoint->y = (std::uint16_t)((float)curpoint->y * helios_scale);

        dacpoint->r = curpoint->intensity!=0?curpoint->col.r():0;
        dacpoint->g = curpoint->intensity!=0?curpoint->col.g():0;
        dacpoint->b = curpoint->intensity!=0?curpoint->col.b():0;
        dacpoint->i = curpoint->intensity;

#ifdef DEBUG_MAXXY
        minx = std::min(minx, dacpoint -> x);
        maxx = std::max(maxx, dacpoint -> x);
        miny = std::min(miny, dacpoint -> y);
        maxy = std::max(maxy, dacpoint -> y);
        maxi = std::max(maxi, dacpoint -> i);
#endif

        helios_dac_index++;
        if (helios_dac_index >= HELIOS_MAX_POINTS)
        {
            helios_dac_index--;
            logerror("*** Warning! DAC Point list overflow!\n");
        }

		lastx = curpoint->x;
		lasty = curpoint->y;
		lasti = curpoint->intensity;

		curpoint++;
	}

	if (numHeliosDevs && heliosDac.GetStatus(HELIOS_DEVICE)) {
        heliosDac.WriteFrame(HELIOS_DEVICE, HELIOS_PPS, HELIOS_FLAGS, heliosPoints.get(), helios_dac_index);
#ifdef DEBUG_MAXXY
        std::cout << "X-values: " << minx << "-" << maxx << " ";
        std::cout << "Y-values: " << miny << "-" << maxy << " ";
        std::cout << "I: " << (char)maxi << std::endl;
#endif
    }

	return 0;
}
