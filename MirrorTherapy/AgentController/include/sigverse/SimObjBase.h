/* $Id: SimObjBase.h,v 1.6 2011-12-21 09:19:52 okamoto Exp $ */ 
#ifndef SimObjBase_h
#define SimObjBase_h

#include <string>
#include <map>
#include <vector>

#include "systemdef.h"
#include "Attribute.h"
#include "Vector3d.h"
#include "Logger.h"


class Parts;
class Rotation;

/**
 * @brief �V�~�����[�V�����G���e�B�e�B��{�N���X
 */
class SimObjBase 
{
public:
	//! ��O��{�N���X
	class Exception
	{
	protected:
		std::string m_msg;
	public:
		Exception() {}
		Exception(const char *msg_) : m_msg(msg_) {;}
		const char *msg() { return m_msg.c_str(); }
	};



	//! �K�v�ȑ�����ێ����Ă��Ȃ��Ƃ��ɔ��������O
	class NoAttributeException : public Exception
	{
	public:
		NoAttributeException(const char *attr) : Exception()
		{
			if (attr) {
				m_msg = "No attribute : \"";
				m_msg += attr;
				m_msg += "\"";
			}
		}
	};



	//! �ǂݍ��ݐ�p�����̒l��ݒ肵���Ƃ��ɔ��������O
	class AttributeReadOnlyException : public Exception
	{
	public:
		AttributeReadOnlyException(const char *attr) : Exception() {
			if (attr) {
				m_msg = "Read only attribute : \"";
				m_msg += attr;
				m_msg += "\"";
			}
		}
	};


	/**
	 * @brief ���i�I�u�W�F�N�g�C�e���[�^
	 *
	 * �p���N���X�Ŏg�p
	 */
	class PartsIterator {
	public:
		virtual ~PartsIterator() {}
		virtual Parts *next() = 0;
	};
#ifndef UNIT_TEST
protected:
#endif
	enum {
		OP_NOT_SET = 0x0,			//!< ����Ȃ�
		OP_SET_POSITION = 0x1,		//!< �ʒu�����̏C������
		OP_SET_ROTATION = 0x2,		//!< ��]�����̏C������
		OP_SET_FORCE = 0x4,			//!< �̓x�N�g���̐ݒ葀��
		OP_SET_ACCEL = 0x8,			//!< �����x�̐ݒ葀��
		OP_SET_TORQUE = 0x10,		//!< �g���N�̐ݒ葀��
		OP_SET_VELOCITY = 0x20,		//!< ���x�̐ݒ葀��
		OP_SET_ANGULAR_VELOCITY = 0x40,	//!< �p���x�̐ݒ葀��
	};
	//! �G���e�B�e�B����^
	typedef unsigned Operation;
public:
	//! �G���e�B�e�BID�^
	typedef unsigned Id;
protected:
	//! ������^
	typedef std::string S;
	//! �����}�b�v�^
	typedef std::map<S, Attribute*> AttrM;
protected:
	bool	m_attached;    //!< �R���g���[�������蓖�Ă��Ă��邩�H
	//Id  	m_id;          //!< �G���e�B�e�BID�B�G���e�B�e�B�Ɋ���U�����ӂȒl�B
	int m_id;
	AttrM	m_attrs;       //!< �����}�b�v
	Operation m_ops;       //!< �Ăяo���ꂽ����
	std::vector<std::string>  m_files; //!< shape�t�@�C����
	std::vector<int>  m_ids;           //!< entity�����J����ID

protected:
	//! �R���X�g���N�^
	SimObjBase();
public:
	//! �f�X�g���N�^
	virtual ~SimObjBase();

	//! �R�s�[���\�b�h
	void copy(const SimObjBase &o);

	//! �����}�b�v���擾����
	const AttrM &attrs() const { return m_attrs; }

	// shape�t�@�C����ǉ�����	
	void addFile(const char* name){ m_files.push_back(name);}

	// shape�t�@�C���̐����擾����
	int getFileNum(){ return m_files.size();}

	//! entity�����J�����̐����擾����
	int getCameraNum(){return m_ids.size();}

	// entity�����J����ID��ǉ�����
	void addCameraID(int id)
	{
	  int size = m_ids.size();
	  for(int i = 0; i < size; i++){
	    if(id == m_ids[i]) {
	      LOG_ERR(("Camera ID %d is already exist.",id));
	      return;	      
	    }
	  }
	  m_ids.push_back(id);
	}

	//! entity�����J����ID(vector)���擾����
	std::vector<int> getCameraIDs(){return m_ids;}

	//! shape�t�@�C�������擾����
	std::string getFile(int num)
	{
	  if(num > m_files.size()){
	    return NULL;
	  }
	  else{
	    return m_files[num];
	  }
	}
	
	//! ������ǉ�����
	void	push(Attribute *attr)
	{
		m_attrs[attr->name()] = attr;
	}
	/**
	 * @brief �����l�𕶎���Őݒ肷��
	 *
	 * �l�͕�����ŗ^���邪�A�����l�̌^�ɉ����ēK�؂Ƀp�[�X�����B
	 *
	 * @param name ������
	 * @param v    �����l
	 */
	void	setAttrValue(const char *name, const char *v)
	{
		getAttr(name).value().setString(v);
	}

	/**
	 * @brief �����l��ݒ肷��
	 *
	 * @param name ������
	 * @param v    �����l
	 */
	void	setAttrValue(const char *name, const Value &v)
	{
		getAttr(name).value().copy(v);
	}

	/**
	 * @brief �K�v�ȑ��������邩�m�F����
	 * �V�~�����[�V���������s����̂ɍŒ���K�v�ȑ��������邩�m�F����
	 * @retval true  �K�v�ȑ���������Ă���
	 * @retval false �K�v�ȑ����������ĂȂ�
	 */
	bool	checkAttrs();

private:
	Attribute* hasAttr(const char *name) const;
public:
	//! �w��̑����𓾂�
	Attribute & getAttr(const char *name) const;

	//! �w��̑��������݂��邩�ǂ���
	bool isAttr(const char *name);

	//! �G���e�B�e�BID�𓾂�
	Id    	id() const { return m_id; }
	//! �R���g���[�������蓖�Ă��Ă��邩�H
	bool	isAttached() const { return m_attached; }

	//!�G���e�B�e�B���𓾂�
	const char *name() const
	{
		return getAttr("name").value().getString();
	}

	//! �N���X���𓾂�
	const char *classname()
	{
		return getAttr("class").value().getString();
	}

	// added by yahara@tome (2011/02/23)
#define DEFINE_ATTR_STRING(NAME, TOKEN)					\
		const char *NAME() const { return getAttr(TOKEN).value().getString(); } \
	void  NAME(const char *v) {						\
	  getAttr(TOKEN).value().setString(v);				\
	}
	//okamoto (2010/12/7)
#define DEFINE_ATTR_DOUBLE(NAME, TOKEN)					\
        double NAME() const { return getAttr(TOKEN).value().getDouble(); } \
	void  NAME(double v) {						\
	  getAttr(TOKEN).value().setDouble(v);				\
	}
	
#define DEFINE_ATTR_BOOL(NAME, TOKEN)				       \
        bool NAME() const { return getAttr(TOKEN).value().getBool(); } \
        void NAME(bool b) { getAttr(TOKEN).value().setBool(b); }

#include "SimObjBaseAttrs.h"

#undef DEFINE_ATTR_DOUBLE
#undef DEFINE_ATTR_BOOL
	// added by yahara@tome (2011/03/03)
#undef DEFINE_ATTR_STRING

	// Operation methods must be declared as virtual.
	/**
	 * @brief �G���e�B�e�B�̈ʒu��ݒ肷��
	 * @param v 3�����ʒu
	 */
	virtual void	setPosition(const Vector3d &v);
	/**
	 * @brief �G���e�B�e�B�̈ʒu��ݒ肷��
	 * @param x_ X���W�l
	 * @param y_ Y���W�l
	 * @param z_ Z���W�l
	 */
	virtual void 	setPosition(double x_, double y_, double z_);
	/**
	 * @brief �G���e�B�e�B�ɂ�����͂�ݒ肷��
	 * @param fx X������
	 * @param fy Y������
	 * @param fz Z������
	 */
	virtual void	setForce(double fx, double fy, double fz);

	/**
	 * @brief �G���e�B�e�B�ɂ�����g���N��ݒ肷��
	 * @param x X������
	 * @param y Y������
	 * @param z Z������
	 */
	virtual void	setTorque(double x, double y, double z);

	/**
	 * @brief �G���e�B�e�B�ɂ����鑬�x��ݒ肷��
	 * @param vx_ X������
	 * @param vy_ Y������
	 * @param vz_ Z������
	 */
	virtual void setVelocity(double vx_,double vy_,double vz_);

	/**
	 * @brief �G���e�B�e�B�ɂ�����p���x��ݒ肷��
	 * @param x_ X������[rad/s]
	 * @param y_ Y������[rad/s]
	 * @param z_ Z������[rad/s]
	 */
	virtual void setAngularVelocity(double x_,double y_,double z_);

	/**
	 * @brief �G���e�B�e�B�̈ʒu���擾����
	 */
	Vector3d &  getPosition(Vector3d &v)
	{
		v.set(x(), y(), z());
		return v;
	}
		
	/**
	 * @brief �G���e�B�e�B�̌�����ݒ肷��
	 *
	 * @param ax ��]����X����
	 * @param ay ��]����Y����
	 * @param az ��]����Z����
	 * @param angle ��]�p�x
	 */
	virtual void setAxisAndAngle(double ax, double ay, double az, double angle);


        /*!
         * @brief It rotates for the specification of the relative angle.
         * @brief ���Ίp�x�w��ɂ���]���s���܂�
         * @param[in] X����]�L��(�l����������i)/x-axis rotation weather(i of quaternion complex part)
         * @param[in] Y����]�L��(�l����������j)/y-axis rotation weather(j of quaternion complex part)
         * @param[in] Z����]�L��(�l����������k)/z-axis rotation weather(k of quaternion complex part)
	 * @param[in] �w��t���O(1.0=��Ίp�x�w��,else=���Ίp�x�w��)                    
         */
	virtual void setAxisAndAngle(double ax, double ay, double az, double angle, double direct);

	/**
	 * @brief �G���e�B�e�B�̌�����ݒ肷��
	 *
	 * @param r ��]�s��
	 */
	virtual void setRotation(const Rotation &r);

private:
	void 	setQ(const dReal *q);
public:

	/**
	 * @brief �G���e�B�e�B�f�[�^���o�C�i��������
	 * @param �o�C�i���f�[�^�T�C�Y
	 * @return �o�C�i���f�[�^
	 */
	char *toBinary(int &n);

protected:
	//! �p�[�c�C�e���[�^���擾����
	virtual PartsIterator * getPartsIterator() = 0;
private:
	void	free_();

public:
	void dump();
#ifdef UNIT_TEST
	Operation ops() { return m_ops; }
#endif	
#ifdef IMPLEMENT_DRAWER
	void draw(DrawContext &c);
#endif
};

#endif // SimObjBase_h
 
