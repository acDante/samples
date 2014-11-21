/* $Id: Value.h,v 1.3 2012-06-11 05:59:25 okamoto Exp $ */ 
#ifndef Value_h
#define Value_h

#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

enum {
	VALUE_TYPE_NOT_SET = -1,
	VALUE_TYPE_STRING,
	VALUE_TYPE_INT,
	VALUE_TYPE_UINT,
	VALUE_TYPE_LONG,
	VALUE_TYPE_ULONG,
	VALUE_TYPE_FLOAT,
	VALUE_TYPE_DOUBLE,
	VALUE_TYPE_BOOL,
	VALUE_TYPE_NUM,
};

typedef short ValueType;
/**
 * @brief �����l�N���X
 */
class Value
{
private:
	ValueType	m_type;
public:
	/**
	 * @brief �R���X�g���N�^
	 * @param t �����l�̎��
	 */
	Value(ValueType t) : m_type(t) {;}

	//! �����l�̎��
	ValueType	type() const { return m_type; }
	//! �o�C�i���������Ƃ��̃f�[�^�T�C�Y�̎擾
	virtual short	binaryLength() const = 0;
	//! �o�C�i���������f�[�^�̎擾
	virtual char *  binary()  = 0;

	//! �f�X�g���N�^
	virtual ~Value() {}

	//! �^����ꂽ�I�u�W�F�N�g�̓��e���R�s�[����
	virtual void copy(const Value &o) = 0;

	//! �����̎�ނ𕶎���Ƃ��Ď擾����
	virtual const char *getTypeString() const = 0;

	//! �����l��doule�^�̒l�Ƃ��Ď擾����
	virtual double getDouble() const = 0;

	//! double�^�̒l��ݒ肷��
	virtual void setDouble(double v) = 0;


	//! �����l��int�^�̒l�Ƃ��Ď擾����
	virtual int getInt() const = 0;
	//! int�^�̒l��ݒ肷��
	virtual void setInt(int v) = 0; 

	//! �����l�𕶎���Ƃ��Ď擾����
	virtual const char *getString() const = 0;

	//! ������l��ݒ肷��
	virtual void setString(const char *) = 0;

	//! �����l��bool�^�̒l�Ƃ��Ď擾����
	virtual bool getBool() const = 0;
	//! bool�^�̒l��ݒ肷��
	virtual void setBool(bool) = 0;

	//! �I�u�W�F�N�g�̃N���[�����쐬����
	virtual Value *clone() = 0;

	//! �o�C�i���f�[�^���瑮���l�𕜌�����
	static Value *decode(char *data);
};

/**
 * @brief bool�^�����l�N���X
 */
class BoolValue : public Value
{
private:
	typedef Value Super;
private:
	enum { DATASIZE = sizeof(ValueType) + sizeof(short), };
private:
	bool	m_value;
public:
	//! �R���X�g���N�^
	BoolValue() : Value(VALUE_TYPE_BOOL) {;}
#ifdef UNIT_TEST
	BoolValue(bool b) : Value(VALUE_TYPE_BOOL) {
		setBool(b);
	}
#endif

private:
	double getDouble() const { return (double)m_value; }
	int getInt() const { return (int)m_value; }
	const char *getString() const;
	const char *getTypeString() const;

	void setDouble(double v);
	void setInt(int v);
	
	void setString(const char *str);

	void copy(const Value &o)
	{
		m_value = o.getBool();
	}
	bool getBool() const { return m_value; }
	void setBool(bool b) { m_value = b; }

	short	binaryLength() const
	{
		return DATASIZE;
	}
	char *  binary();

	Value *clone();
};

/**
 * @brief double�^�����l�N���X
 */
class DoubleValue : public Value
{
	enum {
		DATASIZE = sizeof(double) + sizeof(ValueType),
	};
		
	typedef Value Super;
private:
	double	m_value;

public:
	//! �R���X�g���N�^
	DoubleValue() : Super(VALUE_TYPE_DOUBLE), m_value(0.0) {;}
#ifdef UNIT_TEST
	DoubleValue(double v) : Super(VALUE_TYPE_DOUBLE), m_value(v) {;}
#endif
private:
	double getDouble() const { return m_value; }
	int getInt() const { return (int)m_value; }

	const char *getString() const;
	const char *getTypeString() const;

	void setDouble(double v) { m_value = v; }
	void setInt(int v) { m_value = (double)v; }
	void setString(const char *s);

	bool getBool() const { return false; }
	void setBool(bool b) { ; }

	short	binaryLength() const {
		return DATASIZE;
	}
	char *  binary();
	void copy(const Value &o);

	Value *clone();
};

/**
 * @brief ������^�����l�N���X
 */
class StringValue : public Value
{
private:
	std::string	m_str;
	char	*m_buf;
	int	m_bufsize;
public:
	//! �R���X�g���N�^
	StringValue() : Value(VALUE_TYPE_STRING), m_buf(0), m_bufsize(0) {;}
	StringValue(const char *str) : Value(VALUE_TYPE_STRING), m_buf(0), m_bufsize(0)
	{
		setString(str);
	}
	//! �f�X�g���N�^
	~StringValue() {
		delete [] m_buf; m_buf = 0;
	}

private:
	double getDouble() const { assert(0); return 0.0; }
	int getInt() const { assert(0); return 0; }
	const char *getString() const { return m_str.c_str(); }
	const char *getTypeString() const {
		static const char *type = "string";
		return type;
	}

	void setDouble(double v) { assert(0); }
	void setInt(int v) { assert(0); }
	void setString(const char *str) { m_str = str; }

	bool getBool() const  { return false;}
	void setBool(bool b) {; }

	short	binaryLength() const;
	char *  binary();
	void copy(const Value &o);

	Value *clone();
};

#endif // Value_h
 
