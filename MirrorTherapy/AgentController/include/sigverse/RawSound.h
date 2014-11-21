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

//! �����f�[�^�w�b�_�N���X
class RawSoundHeader
{
private:
	int 		m_channels;
	unsigned 	m_samplingRate;		// [Hz]
	unsigned 	m_bitsPerSample;	// Byte/sec
	RawSoundEndian  m_endian;
public:
	/**
	 * @brief �R���X�g���N�^
	 *
	 * @param channels     �`�����l����(���m����:1,�X�e���I:2)
	 * @param samplingRate �T���v�����O���[�g
	 * @param bitsPerSample �T���v��������r�b�g��
	 * @param endian        �f�[�^�G���f�B�A��
	 */
	RawSoundHeader(int channels, unsigned samplingRate, unsigned bitsPerSample, RawSoundEndian endian) :
		m_channels(channels), m_samplingRate(samplingRate),
		m_bitsPerSample(bitsPerSample), m_endian(endian) {;}

	//! �R���X�g���N�^
	RawSoundHeader() : m_channels(0), m_samplingRate(0), m_bitsPerSample(0), m_endian(0) {}

	//! �`�����l�����𓾂�
	int		getChannelNum() { return m_channels; }
	//! �T���v�����O���[�g�𓾂�
	unsigned 	getSamplingRate() { return m_samplingRate; }
	//! �T���v��������̃r�b�g���𓾂�
	unsigned	getBitPerSample() { return m_bitsPerSample; }
	//! �f�[�^�G���f�B�A���𓾂�
	RawSoundEndian  getEndian() { return m_endian; }
};

//! �����f�[�^�N���X
class RawSound
{
private:
	RawSoundHeader	m_header;
	int		m_datalen;
	char *		m_data;	
public:
	/**
	 * @brief �R���X�g���N�^
	 *
	 * @param h �����f�[�^�w�b�_
	 * @param datalen �����f�[�^��
	 *
	 */
	RawSound(RawSoundHeader &h, int datalen)
		: m_header(h), m_datalen(datalen)
	{
#ifdef WIN32
	    m_data = (char *)HeapAlloc(GetProcessHeap(), HEAP_ZERO_MEMORY, (DWORD)datalen);
#else
		m_data = new char[datalen];
#endif
	}

	//! �R���X�g���N�^
	RawSound() : m_datalen(0), m_data(NULL) {}
	//! �f�X�g���N�^
	~RawSound()	{ freeWaveData(); }

	//! �����f�[�^�w�b�_�̎擾
	RawSoundHeader &getHeader() { return m_header; }

	//! �����f�[�^���̎擾
	int		getDataLen() { return m_datalen; }
	//! �����f�[�^�̎擾
	char *		getData() { return m_data; }

	/**
	 * @brief WAV�`���t�@�C�������[�h����
	 * @param waveFile WAV�t�@�C����
	 */
	bool loadWaveFile(const char *waveFile);
	/**
	 * @brief WAV�`���f�[�^�����[�h����
	 * @param data    WAV�f�[�^
	 * @param datalen WAV�f�[�^��
	 */
	bool loadWaveData(char *data, int datalen);
	//! �Đ�����
	bool play();
	//! WAV�f�[�^���������������
	void freeWaveData();
};

#endif // RawSound_h
