// +-------------------------------------------------------------------------
// | qmApproximate.h
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

#ifndef __QM_APPROXIMATE_H
#define __QM_APPROXIMATE_H



class QmScatteredData
{
  public:
	int		position;
	jhm::vector	value;
};

class QmApproximate
{
  private:
	int		num;
	jhm::vector*	cp;

  public:
	QmApproximate() { num=0; cp=NULL; }

	int		getSize() const { return num; }
	void	setSize( int n );

	void 	smoothBoundary();
	jhm::vector	evaluate( m_real ) const;
	void	discretize( int, jhm::vector* );
	void	approximate( int, int, QmScatteredData *data );
	void	approximateLeastSquare( int, int, QmScatteredData *data );

	static m_real B0( m_real t ) { return (1.0-t)*(1.0-t)*(1.0-t) / 6.0; }
	static m_real B1( m_real t ) { return (3.0*t*t*t - 6.0*t*t + 4.0) / 6.0; }
	static m_real B2( m_real t ) { return (-3.0*t*t*t + 3.0*t*t + 3.0*t + 1.0) / 6.0; }
	static m_real B3( m_real t ) { return t*t*t / 6.0; }

	static m_real DB0( m_real t ) { return -(1.0-t)*(1.0-t) / 2.0; }
	static m_real DB1( m_real t ) { return (3.0*t*t - 4.0*t) / 2.0; }
	static m_real DB2( m_real t ) { return (-3.0*t*t + 2.0*t + 1.0) / 2.0; }
	static m_real DB3( m_real t ) { return t*t / 2.0; }
};

#endif

