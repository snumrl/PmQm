// +-------------------------------------------------------------------------
// | pmVector.h
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

#ifndef __PM_VECTOR_H
#define __PM_VECTOR_H



class PmVector
{
  private:
	PmMaskType	mask;
	jhm::vector		linear;
	jhm::vector		angular[PM_HUMAN_NUM_LINKS];

  public:
  	//
  	// Constructors
  	//
    PmVector() { mask=PM_MASK_NULL; }

	//
	// Set and get parameters
	//
	void		reset( PmMaskType, jhm::vector const& );

    void        setTransq( int, jhm::transq const& );
    jhm::transq      getTransq( int ) const;

	jhm::vector		addLinearVector( jhm::vector const& );
	jhm::vector		setLinearVector( jhm::vector const& );
	jhm::vector		getLinearVector() const;

	jhm::vector		addAngularVector( int, jhm::vector const& );
	jhm::vector		setAngularVector( int, jhm::vector const& );
	jhm::vector		getAngularVector( int ) const;

	void		getDisplacement( jhm::vectorN& ) const;
	void		setDisplacement( jhm::vectorN const& );
	void		addDisplacement( jhm::vectorN const& );

	PmMaskType	setMask( PmMaskType m ) { mask=m; return m; }
	PmMaskType	getMask() const { return mask; }
	
	int         getDOF() const;
	int         getDOF( int ) const;

	//
	// Operators
	//
    PmVector&	operator=( PmVector const& );
    PmVector&	operator+=( PmVector const& );
	PmVector&	operator*=( m_real );
	PmVector&	operator/=( m_real );

	PmVector&	interpolate( m_real, PmVector const&, PmVector const& );

	PmVector&	positionDifference( PmPosture const&, PmPosture const& );

	PmVector&	difference( PmPosture const&, PmPosture const& );
	PmVector&	difference( PmVector const&, PmVector const& );

	m_real		magnitude();
	m_real		squaredMagnitude();
};

#endif
