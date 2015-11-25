/*
 * Modified by Yoshiaki Mizuchi on 2014-01-09
 *    Add english comment, modify indent 
 */
/* $Id: Attribute.h,v 1.2 2011-03-31 08:15:56 okamoto Exp $ */ 
#ifndef Attribute_h
#define Attribute_h

#include <string>
#include "Value.h"

/**
 * @brief Attribute Class
 *
 * represents the attribute of an agent
 */
   
class Attribute
{
	typedef std::string S;
private:
	S	m_name;
	Value * m_value;
	S	m_group;
	char  * m_buf;
	int	m_bufsize;
	int	m_voffset;
private:
	int	binarySize() const;
	void	free_();
public:
	/**
	 * @brief constructor
	 * @param n attribute name
	 * @param v value
	 */
	Attribute(S n, Value *v) : m_name(n), m_value(v), m_buf(0), m_bufsize(0) {;}
	/**
	 * @brief constructor
	 * @param n attribute name
	 * @param v value
	 * @param g attribute group name
	 */
	Attribute(S n, Value *v, S g) : m_name(n), m_value(v), m_group(g), m_buf(0), m_bufsize(0)  {;}
	/**
	 * @brief default constructor
	 */
	Attribute() : m_buf(0), m_bufsize(0), m_voffset(0) {;}
	/**
	 * @brief copy constructor
	 */
	Attribute(const Attribute &o);
	
	/**
	 * @brief destructor
	 */
	~Attribute() { free_(); }

	/**
	 * @brief returns an attribute name
	 */
	const char *name() const { return m_name.c_str(); }
	/**
	 * @brief returns an attribute group name
	 */
	const char *group() const { return m_group.c_str(); }
	/**
	 * @brief returns an attribute value
	 */
	Value & value() const { return *m_value; }

	/**
	 * @brief returns the attribute value of double data type
	 */
	double	getDouble() const { return value().getDouble(); }

	/**
	 * @brief stringize the contents of an object 
	 */
	const char *toString();

	/**
	 * @brief converts the contents of an object to binary data 
	 * @param n size of binary data
	 * @return binary data
	 */
	char *	toBinary(int &n);
	/**
	 * @brief reconverts from binary data
	 * @param data binary attribute data
	 * @param n size of binary data
	 */
	int	setBinary(char *data, int n);
	/**
	 * @brief dump method (for debug)
	 */
	void	dump();
};

#endif // Attribute_h
 

