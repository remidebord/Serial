#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

#include "../inc/serial.h"
#include "../inc/timeout.h"

#if defined(__WIN32__) || defined(__CYGWIN__)

	char serial::Open(char* port, int baud, char bits, parity parity, char stopbit) 
	{
		char tmp[BUFFER_SIZE];
		
		// Convert to string
		sprintf(tmp, "%s", port);
		
		// Set read non blocking (wait 20ms between two bytes)
		COMMTIMEOUTS timeouts = {MAXDWORD, 0, TIME_OUT, 0, 0};
		
		// Create file handle for port
		m_fd = CreateFile(tmp,
						  GENERIC_READ | GENERIC_WRITE, 
						  0,
						  NULL,
						  OPEN_EXISTING,
						  0,
						  NULL);
		
		// Check opening status
		if(m_fd == INVALID_HANDLE_VALUE)
	    {
	        printf("Error Opening %s Port\n", tmp);
	        return 0;
	    }
	    
		// Set input and output buffer size
		SetupComm(m_fd, 4096, 4096); 
	
		// Reset settings
		m_conf.fOutxCtsFlow = FALSE;
		m_conf.fOutxDsrFlow = FALSE;
		m_conf.fOutX = FALSE;
		m_conf.fInX = FALSE;
		m_conf.fNull = 0;
		m_conf.fAbortOnError = 0;
		
		//m_conf.fRtsControl=RTS_CONTROL_TOGGLE;
	
		// Set baud rate
		switch(baud)
		{
			case 9600  : m_conf.BaudRate = CBR_9600  ; break;
			case 19200 : m_conf.BaudRate = CBR_19200 ; break;
			case 38400 : m_conf.BaudRate = CBR_38400 ; break;
			case 57600 : m_conf.BaudRate = CBR_57600 ; break;
			case 115200: m_conf.BaudRate = CBR_115200; break;
			default	   : m_conf.BaudRate = CBR_9600;
		}
	
		// Set byte size
	    switch(bits)
	    {
			case 5	: m_conf.ByteSize = 5; break;
	    	case 6	: m_conf.ByteSize = 6; break;
	    	case 7	: m_conf.ByteSize = 7; break;
	    	case 8	: m_conf.ByteSize = 8; break;
	    	default : m_conf.ByteSize = 8; break;
	    }
	
		// Set parity
		switch(parity)
		{
			case NO	  : m_conf.Parity = NOPARITY;   break;
			case EVEN : m_conf.Parity = EVENPARITY; break;
			case ODD  : m_conf.Parity = ODDPARITY;  break;
			default	  : m_conf.Parity = NOPARITY;
		}
	
		// Set stop bit
		switch(stopbit)
		{
			case 1: m_conf.StopBits = ONESTOPBIT;  break;
			case 2: m_conf.StopBits = TWOSTOPBITS; break;
			default: m_conf.StopBits = ONESTOPBIT;
		}
	
		// Configure serial port
		if(!SetCommTimeouts(m_fd, &timeouts) || !SetCommState(m_fd, &m_conf))
		{
			printf("Error initialization %s Port\n", tmp);
	        CloseHandle(m_fd);
	        return 0;
		}
	
		// Clean errors and rx/tx buffer
		PurgeComm(m_fd, PURGE_RXABORT | PURGE_TXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR);
		
		// Display settings
		printf("%s | BaudRate: %d | Bits: %d | Parity: %d | StopBits: %d\n", tmp, baud, bits, parity, stopbit);
	
		return 1;
	}
	
	void serial::Close(void) 
	{
		CloseHandle(m_fd);
	}
	
	
	char serial::Write(char* buffer, int length) 
	{
		DWORD r;
		uint8_t *pos = (uint8_t*)buffer;
	
		// Send data
		while(length > 0)
		{
			if(!WriteFile(m_fd, pos, length, &r, NULL))
			{
				return 0;
			}
			
			if (r < 1)
			{
				return 0;
			}
	
			length -= r;
			pos += r;
		}
	
		return 1;
	}
	
	int serial::Read(char* buffer)
	{
		DWORD r = 1;
		int length = 0;
		
		//memset(buffer, 0x00, BUFFER_SIZE);
		
		while(r > 0)
		{
			// Protect buffer
			if(length >= (BUFFER_SIZE - 1)) return length;
			
			ReadFile(m_fd, (buffer + length), 1, &r, NULL);
						
			if(r > 0) length++;
		}
		
		return length;
	}

#else

	char serial::Open(char* port, int baud, char bits, parity parity, char stopbit) 
	{
		char tmp[BUFFER_SIZE];
		
		// Convert to string
		sprintf(tmp, "%s", port);
		
   		m_fd = open(tmp, O_RDWR | O_NOCTTY | O_NONBLOCK);
		
		// Check opening status 
		if (m_fd < 0)
		{
	        printf("Error Opening %s Port\n", tmp);
	        return 0;
		}
		
		// Get terminal parameters
		tcgetattr(m_fd, &m_newtio);
		tcgetattr(m_fd, &m_oldtio);

		// Flushes data received but not read		
		ioctl(m_fd, TCIFLUSH);

		// Set baud rate (in and out)
		switch(baud)
		{
			case 9600	: cfsetspeed(&m_newtio, B9600)	; break;
			case 19200	: cfsetspeed(&m_newtio, B19200)	; break;
			case 38400	: cfsetspeed(&m_newtio, B38400)	; break;
			case 57600	: cfsetspeed(&m_newtio, B57600)	; break;
			case 115200	: cfsetspeed(&m_newtio, B115200); break;
			default 	: cfsetspeed(&m_newtio, B9600)	; break;
		}

		// Set byte size
		m_newtio.c_cflag &= ~CSIZE;	

		switch(bits)
		{
			case 5	: m_newtio.c_cflag |= CS5; break;
			case 6	: m_newtio.c_cflag |= CS6; break;
			case 7	: m_newtio.c_cflag |= CS7; break;
			case 8	: m_newtio.c_cflag |= CS8; break;
			default : m_newtio.c_cflag |= CS8; break;
		}

		// Set parity
		switch(parity)
		{
			case NO	  :
					m_newtio.c_cflag &=~ PARENB;	// Disable parity
				break;
				
			case EVEN :
					m_newtio.c_cflag |= PARENB;		// Enable parity
					m_newtio.c_cflag &= ~PARODD;	// Disable odd parity
				break;
				
			case ODD  :
					m_newtio.c_cflag |= PARENB;		// Enable parity
					m_newtio.c_cflag |= PARODD;		// Enable odd parity
				break;
				
			default	  :
					m_newtio.c_cflag &=~ PARENB;	// Disable parity
		}
		
		// Set stop bit
		switch(stopbit)
		{
			case 1: m_newtio.c_cflag &=~ CSTOPB	; break;	// Disable 2 stop bits
			case 2: m_newtio.c_cflag |= CSTOPB	; break;	// Enable 2 stop bits
			default: m_newtio.c_cflag &=~ CSTOPB;
		}
	
		// Enable receiver (CREAD) and ignore modem control lines (CLOCAL)
		m_newtio.c_cflag |= (CREAD | CLOCAL); 
		
		// Disable, canonical mode (ICANON = 0), echo input character (ECHO) and signal generation (ISIG)
		m_newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
		
		// Disable input parity checking (INPCK)
		m_newtio.c_iflag &= ~INPCK; 		  
	
		// Disable XON/XOFF flow control on output and input (IXON/IXOFF), framing and parity errors (IGNPAR), and disable CR to NL translation
		m_newtio.c_iflag &= ~(IXON | IXOFF | IXANY | IGNPAR | ICRNL);
	
		// Disable implementation-defined output processing (OPOST)
		m_newtio.c_oflag &=~ OPOST;	

		// Set terminal parameters
		tcsetattr(m_fd, TCSAFLUSH, &m_newtio);
		
		// Display settings
		printf("%s | BaudRate: %d | Bits: %d | Parity: %d | StopBits: %d\n", tmp, baud, bits, parity, stopbit);
	
		return 1;
	}
	
	void serial::Close(void) 
	{
   		// Set old parameters
		tcsetattr(m_fd, TCSANOW, &m_oldtio);
   		
   		// Close serial port
		if(m_fd > 0)
		{
			close(m_fd);
		}
	}
	
	
	char serial::Write(char* buffer, int length) 
	{
		ssize_t r;
		const uint8_t *pos = (const uint8_t*)buffer;

		// Send data
		while(length > 0)
		{
			r = write(m_fd, pos, length);
			
			if(r < 1)
			{
				return 0;
			}

			length -= r;
			pos += r;
		}

		return 1;
	}
	
	int serial::Read(char* buffer)
	{
		ssize_t r = 1;
		int length = 0;

		// Set timeout between 2 bytes (20ms)
		timeout timeout(TIME_OUT);
		
		//memset(buffer, 0x00, BUFFER_SIZE);
		
		while((r > 0) || timeout.end())
		{
			// Protect buffer
			if(length >= (BUFFER_SIZE - 1)) return length;
			
			r = read(m_fd, (buffer + length), 1);
						
			if(r > 0)
			{
				length++;

				// Start timer
				timeout.start();
			}
		}
		
		return length;
	}

#endif
