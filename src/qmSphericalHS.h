// +-------------------------------------------------------------------------
// | qmSphericalHS.h
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

#ifndef __QM_SPHERICAL_HS_H
#define __QM_SPHERICAL_HS_H

#include "qmHalfspace.h"



class QmSphericalHS : public QmHalfspace
{
  private:
	jhm::quater		orientation;
	jhm::interval	angle_bound;

	//
    // stream
    //
    friend std::ostream& operator<<( std::ostream&, QmSphericalHS const& );
    friend std::istream& operator>>( std::istream&, QmSphericalHS& );

  public:
    //
    // Constructors
    //
	QmSphericalHS() {}
	QmSphericalHS( jhm::quater const& q, jhm::interval const& i )
		{ orientation=q; angle_bound=i; }

    //
    // Inquiry functions
    //
    int     inclusion( jhm::quater const& );
    int     intersection( QmGeodesic const&, jhm::quater* );
    m_real	distance( jhm::quater const& );
	jhm::quater	nearest( jhm::quater const& );
	jhm::vector  gradient( jhm::quater const& );

    //
    // Set and get parameters
    //
    void		setOrientation( jhm::quater const& q ) { orientation=q; }
    void		setAngleBound( jhm::interval const& i )  { angle_bound=i; }
    jhm::quater		getOrientation() const { return orientation; }
    jhm::interval	getAngleBound()  const { return angle_bound; }

	//
	// File ouput
	//
	void		write( std::ostream& );
};

#endif
