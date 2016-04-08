// +-------------------------------------------------------------------------
// | qmComplex.h
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

#ifndef __QM_COMPLEX_H
#define __QM_COMPLEX_H

#include "qmConicHS.h"
#include "qmAxialHS.h"
#include "qmSphericalHS.h"



class QmComplex
{
  private:
	int				num;
	QmHalfspace*	root;

    //
    // stream
    //
    friend std::ostream& operator<<( std::ostream&, QmComplex const& );
    friend std::istream& operator>>( std::istream&, QmComplex& );

  public:
    //
    // Constructors
    //
	QmComplex() { num=0; root=NULL; }

	//
	// Manipulate List
	//
	QmHalfspace* addHalfspace( QmHalfspace* );
	QmHalfspace* removeHalfspace( QmHalfspace* );

    //
    // Inquiry functions
    //
    int     inclusion( jhm::quater const& );
    int     intersection( QmGeodesic const&, jhm::quater* );

	m_real	distance(jhm::quater const& );
	jhm::vector 	gradient( jhm::quater const& );

	jhm::quater	project( jhm::quater const& );
	jhm::matrix	project( jhm::matrix const& );
	jhm::transf	project( jhm::transf const& );
};

#endif

