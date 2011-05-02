#ifndef _XSENS_TIME_2006_09_12
#define _XSENS_TIME_2006_09_12

#ifndef _PSTDINT_H_INCLUDED
#	include "pstdint.h"
#endif

#include <time.h>

namespace xsens {

//! The number of seconds in a normal day
#define XSENS_SEC_PER_DAY	(60*60*24)
//! The number of milliseconds in a normal day
#define XSENS_MS_PER_DAY	(XSENS_SEC_PER_DAY*1000)

//! A real-time timestamp (ms)
typedef uint64_t TimeStamp;

/*! \brief A platform-independent clock.

	The function returns the time of day in ms since midnight. If the \c date parameter is
	non-NULL, corresponding the date is placed in the variable it points to.
*/
uint32_t getTimeOfDay(tm* date_ = NULL, time_t* secs_ = NULL);

/*! \brief A platform-independent sleep routine.

	Time is measured in ms. The function will not return until the specified
	number of ms have passed.
*/
void msleep(uint32_t ms);

TimeStamp timeStampNow(void);

class TimeSync {
private:
	int m_timeSet;
	double m_timeOffset;	//!< offset of clock in ms wrt timeStampNow
	double m_tSyncPerSys;	//!< Number of sync seconds per system second passed
	int64_t m_tSysStart;	//!< Start time from which tSyncPerSys should be computed
	int64_t m_tSyncStart;	//!< Start time for next m_tSyncPerSys computation

	int64_t m_tLastSysStart;	//!< Start time from which new sync values should be computed
	int64_t m_tLastSyncStart;	//!< Start time from which new sync values should be computed

	double m_dtSync;
	double m_dtSys;

	double m_tolerance;		//!< Tolerance between clock speeds (jitter), typically 1% ==> 0.01
	double m_avgFactor;		//!< Factor for moving average filter. new1 = (factor*old+(1-factor)*new0)

public:
	TimeSync(double tolerance, double factor) : m_timeSet(0), m_tolerance(tolerance), m_avgFactor(factor) { }

	bool isSet(void) { return m_timeSet != 0; }
	void reset(void) { m_timeSet = 0; }
	void setCurrentTime(TimeStamp syncTime, TimeStamp receiveTime);
	double getOffset(void) const { return m_timeOffset; }
	double getDrift(void) const { return m_tSyncPerSys; }

	TimeStamp getCurrentTime(int64_t ts) { if (m_timeSet) return m_tSyncStart+int64_t(double(ts-m_tSysStart)*m_tSyncPerSys); else return ts; }
	TimeStamp getInverseTime(int64_t ts) { if (m_timeSet) return m_tSysStart+int64_t(double(ts-m_tSyncStart)/m_tSyncPerSys); else return ts; }
};

}	// end of xsens namespace

#endif	// _XSENS_TIME_2006_09_12
