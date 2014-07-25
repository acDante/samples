/* $Id: RawSound.h,v 1.2 2011-03-31 08:15:57 okamoto Exp $ */ 
#ifndef RawSound_h
#define RawSound_h

#include <stdio.h>

#ifdef WIN32
#include <windows.h>
#endif

enum {
	RAW_SOUND_LITTLE_ENDIAN = 0,
	RAW_SOUND_BIG_ENDIAN,
};

typedef unsigned short RawSoundEndian;

class RawSoundHeader
{
private:
	int 		m_channels;
	unsigned 	m_samplingRate;		// [Hz]
	unsigned 	m_bitsPerSample;	// Byte/sec
	RawSoundEndian  m_endian;
public:
	RawSoundHeader(int channels, unsigned samplingRate, unsigned bitsPerSample, RawSoundEndian endian) :
		m_channels(channels), m_samplingRate(samplingRate),
		m_bitsPerSample(bitsPerSample), m_endian(endian) {;}

	RawSoundHeader() : m_channels(0), m_samplingRate(0), m_bitsPerSample(0), m_endian(0) {}

	int		getChannelNum() { return m_channels; }
	unsigned 	getSamplingRate() { return m_samplingRate; }
	unsigned	getBitPerSample() { return m_bitsPerSample; }
	RawSoundEndian  getEndian() { return m_endian; }
};

class RawSound
{
private:
	RawSoundHeader	m_header;
	int		m_datalen;
	char *		m_data;	
public:
	RawSound(RawSoundHeader &h, int datalen)
		: m_header(h), m_datalen(datalen)
	{
#ifdef WIN32
	    m_data = (char *)HeapAlloc(GetProcessHeap(), HEAP_ZERO_MEMORY, (DWORD)datalen);
#else
		m_data = new char[datalen];
#endif
	}

	RawSound() : m_datalen(0), m_data(NULL) {}
	~RawSound()	{ freeWaveData(); }

	RawSoundHeader &getHeader() { return m_header; }

	int		getDataLen() { return m_datalen; }
	char *		getData() { return m_data; }

	bool loadWaveFile(const char *waveFile);
	bool loadWaveData(char *data, int datalen);
	bool play();
	void freeWaveData();
};

#endif // RawSound_h

