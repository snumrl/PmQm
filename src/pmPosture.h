// +-------------------------------------------------------------------------
// | pmPosture.h
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

#ifndef __PM_POSTURE_H
#define __PM_POSTURE_H

#include "qmPolygon2D.h"



#define PM_HUMAN_NUM_LINKS   58

class PmVector;

class PmPosture
{
  private:
  	PmHuman*	body;
	PmMaskType	mask;
	m_real		scale;
	jhm::vector		trans;
	jhm::quater		rotate[PM_HUMAN_NUM_LINKS];

  public:
  	//
  	// Constructors
  	//
	PmPosture() { scale=1.0; body=NULL; mask=PM_MASK_SCALE; }
//	PmPosture() { scale=1.0; body=NULL; mask=0x10000000; }

	//
	// Set and get parameters
	//
	PmHuman*	setBody( PmHuman* h ) { body=h; return h; }
	PmHuman*	getBody() const { return body; }
	
	void		setTransq( int, jhm::transq const& );
	jhm::transq		getTransq( int ) const;

	void		setTransf( int, jhm::transf const& );
	jhm::transf		getTransf( int ) const;

	jhm::vector		getTranslation() const;
	void		setTranslation( jhm::vector const& );
	void		addTranslation( jhm::vector const& );

	jhm::quater		getRotation( int ) const;
	void		setRotation( int, jhm::quater const& );
	void		setRotation( int, jhm::vector const& );
	void		setRotation( int, jhm::matrix const& );
	void		addRotation( int, jhm::vector const& );

	void		setScale( m_real );
	m_real		getScale() const;

	jhm::transf		getBaseTransf( int ) const;
	jhm::transf		getGlobalTransf( int ) const;
	jhm::vector		getGlobalTranslation( int ) const;
	jhm::position	getGlobalPosition( int ) const;
	jhm::quater		getGlobalRotation( int ) const;

	PmMaskType	setMask( PmMaskType m ) { mask=m; return m; }
	PmMaskType	getMask() const { return mask; }
	
	int			getDOF() const;
	int			getDOF( int ) const;

	int			getNumSpineLinks() const;

	void		addDisplacement( jhm::vectorN const& );
	void		addDisplacement( PmVector const&, m_real );

	//
	// Operators
	//
	PmPosture&	operator=( PmPosture const& );
	PmPosture&	operator+=( PmVector const& );

	//
	// Physical properties
	//
	jhm::vector		getCOG( int ) const;
	jhm::vector		getCOG() const;
	void		getCOGlist( m_real*, jhm::vector* ) const;

	void		getSupportPolygon( PmConstraint const&, QmPolygon2D& ) const;
	
	//
	// Auxiliary functions
	//
	PmPosture&	interpolate( m_real, PmPosture const&, PmPosture const& );

	PmPosture&	copyOver( PmPosture const& );
	PmPosture&	copyUnder( PmPosture const& );
	PmPosture&	blend( m_real, PmPosture const& );

	PmPosture&	applyTransf( jhm::transf const& );

	jhm::vector		getRollingAxis( int, int, int, int );


	//
	// Predefined postures
	//
	static PmPosture posture_stand();
	static PmPosture posture_stand( jhm::vector const& );
	static PmPosture posture_hand_relax_right();
	static PmPosture posture_hand_relax_left();
	static PmPosture posture_hand_fist_right();
	static PmPosture posture_hand_fist_left();
	static PmPosture posture_hand_stretch_right();
	static PmPosture posture_hand_stretch_left();


	//
	// Inverse kinematic and Error correction
	//
	void ik_limb( jhm::transf const&, int, m_real roll=0.0 );

	void ik_right_hand(jhm::transf const&, m_real roll=0.0 );
	void ik_left_hand (jhm::transf const&, m_real roll=0.0 );
	void ik_right_foot(jhm::transf const&, m_real roll=0.0 );
	void ik_left_foot (jhm::transf const&, m_real roll=0.0 );

	int  ik_balance( PmConstraint const& );
	int  ik_balance( PmConstraint const&, PmMaskType );

	int  ik_body( PmConstraint const& );
	int  ik_body( PmConstraint const&, PmMaskType );

	int  ik_torso( PmConstraint const&, int optimize=TRUE );
	int  ik_torso( int, jhm::transf const&, int, jhm::transf const&,
				   int, jhm::transf const&, int, jhm::transf const&,
				   int, jhm::transf const&, int optimize=TRUE );

	void correct_313_to_322( int );

	//
	//  Debugging
	//
	void euler2quater();
	void quater2euler();
};

#endif
