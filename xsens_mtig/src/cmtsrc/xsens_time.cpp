// Note, this function requires compiler option "-lrt" to be set when compiling with gcc

#include "xsens_time.h"
#include <sys/timeb.h>

#ifdef _WIN32
#	include <windows.h>
#else
#	include <unistd.h>
#   include <sys/time.h>
#endif
#include <math.h>

namespace xsens {

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Other  functions ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
// A platform-independent clock.
uint32_t getTimeOfDay(tm* date_, time_t* secs_)
{
#ifdef _WIN32
	static uint64_t startTimePc;
	static uint64_t lpf = 0;
	static __timeb64 startTimeB;

	LARGE_INTEGER pc;
	if (QueryPerformanceCounter(&pc))
	{
		if (!lpf)
		{
			LARGE_INTEGER tmp;
			QueryPerformanceFrequency(&tmp);
			lpf = tmp.QuadPart;
			startTimePc = pc.QuadPart;

			_ftime64_s(&startTimeB);
			// get rid of millitm part
			startTimePc -= startTimeB.millitm*lpf/1000;
			startTimeB.millitm = 0;
		}

		__timeb64 tp = startTimeB;
		//_ftime32_s(&tp);
		uint64_t dms = (pc.QuadPart-startTimePc)*1000/lpf;
		tp.time += dms/1000;
		tp.millitm = (unsigned short) (dms%1000);

		if (date_ != NULL)
		{
			__time64_t tin = tp.time;
			_localtime64_s(date_,&tin);
		}
		if (secs_ != NULL)
			secs_[0] = tp.time+(tp.dstflag*3600)-(tp.timezone*60);

		// 86400 = 24*60*60 = secs in a day, this gives us the seconds since midnight
		return (1000 * ((uint32_t) tp.time % XSENS_SEC_PER_DAY)) + tp.millitm;
	}
	else
	{
		__timeb32 tp;
		_ftime32_s(&tp);

		if (date_ != NULL)
		{
			__time32_t tin = tp.time;
			_localtime32_s(date_,&tin);
		}
		if (secs_ != NULL)
			secs_[0] = tp.time+(tp.dstflag*3600)-(tp.timezone*60);

		// 86400 = 24*60*60 = secs in a day, this gives us the seconds since midnight
		return (1000 * ((uint32_t) tp.time % XSENS_SEC_PER_DAY)) + tp.millitm;
	}
#else
	timespec tp;
	clock_gettime(CLOCK_REALTIME, &tp); // compile with -lrt

	if (date_ != NULL)
		localtime_r(&tp.tv_sec,date_);

	if (secs_ != NULL)
		secs_[0] = tp.tv_sec;

	// 86400 = 24*60*60 = secs in a day, this gives us the seconds since midnight
	return (1000 * (tp.tv_sec % XSENS_SEC_PER_DAY)) + (tp.tv_nsec/1000000);
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////
// A platform-independent sleep routine.
void msleep(uint32_t ms)
{
#ifdef _WIN32
	Sleep(ms);
#else
	clock_t end = clock() + (CLOCKS_PER_SEC/1000) * ms;
	clock_t diff;

	while ((diff = end - clock()) > 0)
	{
		diff = (1000 * diff) / CLOCKS_PER_SEC;
		if (diff > 1000)
			sleep(diff / 1000);
		else
			usleep(diff * 1000);
	}
#endif
}

TimeStamp timeStampNow(void)
{
	TimeStamp ms;
	time_t s;
	ms = (TimeStamp) getTimeOfDay(NULL,&s);
	ms = (ms % 1000) + (((TimeStamp)s)*1000);

	return ms;
}

//#define LTLOWPASS		19.0

void TimeSync::setCurrentTime(TimeStamp syncTime, TimeStamp receiveTime)
{
	if (m_timeSet)
	{
		double newOffset = double(int64_t(syncTime - receiveTime));
		double dtSync = double(int64_t(syncTime - m_tLastSyncStart));
		double dtSys = double(int64_t(receiveTime - m_tLastSysStart));

		// we require at least 100 ms between two updates
		if (dtSys >= 100.0)
		{
			// we 'only' tolerate a 25% clock speed difference
			if (fabs(dtSync/dtSys - 1.0) < 0.01) // > 750.0 && dtSync < 1250.0 && dtSys > 750.0 && dtSys < 1250.0)
			{
				if (m_timeSet == 2)
				{
					m_dtSync = m_dtSync*m_avgFactor+dtSync*(1.0-m_avgFactor);
					m_dtSys  = m_dtSys *m_avgFactor+dtSys *(1.0-m_avgFactor);
				}
				else
				{
					m_dtSync = dtSync;
					m_dtSys = dtSys;
					m_timeSet = 2;
				}
				m_timeOffset = newOffset;
				m_tSyncPerSys = m_dtSync / m_dtSys;

				m_tSysStart = receiveTime;
				m_tSyncStart = syncTime;

			}
			m_tLastSysStart = receiveTime;
			m_tLastSyncStart = syncTime;
		}
	}
	else
	{
		m_timeOffset = double(int64_t(syncTime - receiveTime));
		m_timeSet = 1;
		m_dtSync = 1000.0;
		m_dtSys = 1000.0;
		m_tSyncPerSys = 1;
		m_tSysStart = receiveTime;
		m_tSyncStart = syncTime;
		m_tLastSysStart = receiveTime;
		m_tLastSyncStart = syncTime;
	}
}

}	// end of xsens namespace
