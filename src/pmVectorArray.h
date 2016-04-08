// +-------------------------------------------------------------------------
// | pmVectorArray.h
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

#ifndef __PM_VECTOR_ARRAY_H
#define __PM_VECTOR_ARRAY_H



class PmLinearMotion;

class PmVectorArray
{
  private:
	int				oSize;
	int				size;

  public:
	PmVector*		vectors;

  public:
	PmVectorArray();
	~PmVectorArray();

	PmMaskType		getMask() const;

	int				getSize() const { return size; }
	void			setSize( int );

	PmVector&		getVector( int i ) const { return vectors[i]; }
	void			setVector( int i, PmVector const& v ) { vectors[i] = v; }

	void			getLinearVectors( jhm::vector* ) const;
	void			setLinearVectors( jhm::vector* );

	void			getAngularVectors( int, jhm::vector* ) const;
	void			setAngularVectors( int, jhm::vector* );

	void			amplify( m_real, PmMaskType );

	//
	// Operators
	//
    PmVectorArray&	operator=( PmVectorArray const& );
    PmVectorArray&	operator+=( PmVectorArray const& );
	PmVectorArray&	operator*=( m_real );
	PmVectorArray&	operator*=( PmVectorArray const& );
	PmVectorArray&	operator/=( m_real );

	PmVectorArray&	difference( PmLinearMotion const&, PmLinearMotion const& );
	PmVectorArray&	difference( PmVectorArray const&, PmVectorArray const& );
};

#endif

