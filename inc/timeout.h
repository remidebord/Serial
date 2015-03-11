#ifndef _TIMEOUT_H
#define _TIMEOUT_H

#include <time.h>

class timeout
{
	private:
		
		clock_t m_time;
		int m_maxTime;
		char m_state;

	public:
		
		timeout(int time);
		void start(void);
		char end(void);
};

#endif
