// +-------------------------------------------------------------------------
// | qmPolygon2D.h
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

#ifndef __QM_POLYGON_2D_H
#define __QM_POLYGON_2D_H



class QmPolygon2D
{
  private:
	int			numVertices;
	int			numStorage;

  public:
	jhm::position*	vertices;

  public:
	QmPolygon2D();
	QmPolygon2D( int );
	~QmPolygon2D();

	void		setNumVertices( int );
	int			getNumVertices() const { return numVertices; }

	jhm::position	operator[]( int i ) { return vertices[i]; }
	jhm::position	getVertex( int i ) const { return vertices[i]; }
	void		setVertex( int i, jhm::position p ) { vertices[i] = p; }

	void		convexHull( int, jhm::position* );
	int			inclusion( jhm::position const& ) const;
	m_real		distance( jhm::position const& ) const;
	jhm::vector		gradient( jhm::position const& ) const;
};

#endif
