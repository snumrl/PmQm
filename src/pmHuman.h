// +-------------------------------------------------------------------------
// | pmHuman.h
// | 
// | Author: Jehee Lee
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Jehee Lee 2013
// |    See the included COPYRIGHT.txt file for further details.
// |    
// |    This file is part of the PmQm library.
// |    PmQm library is free software: you can redistribute it and/or modify
// |    it under the terms of the MIT License.
// |
// |    You should have received a copy of the MIT License
// |    along with PmQm library.  If not, see <mit-license.org>.
// +-------------------------------------------------------------------------

#ifndef __PM_HUMAN_H
#define __PM_HUMAN_H

#include <stdio.h>



class PmHuman
{
  public:

	enum { UNDEFINED=-1,
       PELVIS=0,
       SPINE_1, SPINE_2, SPINE_3, SPINE_4, CHEST, 
       NECK, HEAD,
	   RIGHT_SHOULDER, LEFT_SHOULDER,
	   RIGHT_COLLAR, LEFT_COLLAR,
       UPPER_RIGHT_ARM, UPPER_LEFT_ARM, LOWER_RIGHT_ARM, LOWER_LEFT_ARM,
       UPPER_RIGHT_LEG, UPPER_LEFT_LEG, LOWER_RIGHT_LEG, LOWER_LEFT_LEG,
       RIGHT_FOOT, LEFT_FOOT, RIGHT_TOE, LEFT_TOE,
       RIGHT_PALM, LEFT_PALM,
	   RIGHT_HEEL, LEFT_HEEL,
       RIGHT_FINGER_11, RIGHT_FINGER_12, RIGHT_FINGER_13,
       RIGHT_FINGER_21, RIGHT_FINGER_22, RIGHT_FINGER_23,
       RIGHT_FINGER_31, RIGHT_FINGER_32, RIGHT_FINGER_33,
       RIGHT_FINGER_41, RIGHT_FINGER_42, RIGHT_FINGER_43,
       RIGHT_FINGER_51, RIGHT_FINGER_52, RIGHT_FINGER_53,
       LEFT_FINGER_11, LEFT_FINGER_12, LEFT_FINGER_13,
       LEFT_FINGER_21, LEFT_FINGER_22, LEFT_FINGER_23,
       LEFT_FINGER_31, LEFT_FINGER_32, LEFT_FINGER_33,
       LEFT_FINGER_41, LEFT_FINGER_42, LEFT_FINGER_43,
       LEFT_FINGER_51, LEFT_FINGER_52, LEFT_FINGER_53
	};
 
  protected:
	PmMaskType		mask;
    int				num_links;
  	int				num_spine_links;
  	
	int				parent_list[PM_HUMAN_NUM_LINKS];
	jhm::transf			base_transf[PM_HUMAN_NUM_LINKS];

	m_real			root_height;
	m_real			ankle_height;

  public:
	QmComplex*		angle_bound[PM_HUMAN_NUM_LINKS];
	
  public:
    PmHuman();
    PmHuman( char* , double scale = 1.0);
    ~PmHuman();

    virtual void	openHuman( char*, double scale = 1.0 );
    void			setAngleBound();

	PmMaskType		getMask() const { return mask; }
	int				getNumLinks() const { return num_links; }

	static int		getLinkNumber( char* );
	static char*	getLinkName( int );

	int				getParent( int i ) const { return parent_list[i]; }
	char			isAncestor( int, int );
	QmComplex*		getAngleBound( int i ) const { return angle_bound[i]; }
	jhm::transf			getJointTransf( int i ) const { return base_transf[i]; }
	jhm::vector			getJointPosition( int i ) const { return base_transf[i].getTranslation(); }
	int				getNumSpineLinks() const { return num_spine_links; }
	m_real			getRootHeight() const { return root_height; }
	m_real			getAnkleHeight() const { return ankle_height; }

	void			reshape( jhm::vector* list );

	m_real			getMass( int ) const;
	void			setMass( int, m_real );

	static int		getSymmetricPart( int );
};

#endif
