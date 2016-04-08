// +-------------------------------------------------------------------------
// | pmConstraint.h
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

#ifndef __PM_CONSTRAINT_H
#define __PM_CONSTRAINT_H

#define MAX_CONSTRAINT_ENTITY	10

#define PM_UNCONSTRAINED		0x00
#define PM_C_POSITION			0x01
#define PM_C_ORIENTATION		0x02
#define PM_C_TRANSF				0x03
#define PM_C_COG				0x04
#define PM_C_BALANCE			0x08

#define PM_C_AND				0x01
#define PM_C_OR					0x02

class PmHuman;

class PmConstraintEntity
{
  public:
	char	mask;
	int		link;
	jhm::transf	value;

	PmConstraintEntity()
	{
		mask = PM_UNCONSTRAINED;
		link = -1;
	}

	void	applyTransf( jhm::transf const& t ) { value *= t; }

	int		getDOC() { if (mask==PM_C_POSITION) return 3;
				  else if (mask==PM_C_ORIENTATION) return 3;
				  else if (mask==PM_C_TRANSF) return 6;
				  else if (mask==PM_C_COG) return 3;
				  else if (mask==PM_C_BALANCE) return 2;
				  else return 0; }
};

class PmConstraint
{
  private:
	int		num_entity;
	
  public:
	PmConstraintEntity entity[MAX_CONSTRAINT_ENTITY];

  public:
	PmConstraint() { num_entity = 0; }

	void	reset() { num_entity = 0; }

	int		getNumEntity() const { return num_entity; }
	char	isConstrained( int ) const;

	int		getDOC();
	
	void	push( int, char, jhm::transf );   // PM_C_TRANSF or PM_C_POSITION or PM_C_ORIENTATION
	void	push( int, jhm::transf );         // PM_C_TRANSF
	void	push( int, jhm::vector );         // PM_C_POSITION
	void	push( int, jhm::quater );         // PM_C_ORIENTATION
	void	push( int, jhm::matrix );         // PM_C_ORIENTATION
	void	push_cog( jhm::vector );          // PM_C_COG
	void	push_cog( m_real, m_real );  // PM_C_BALANCE

	void	pop() { num_entity--; }
	void	remove( int );

	void	operator=( PmConstraint const& );

	jhm::transf	get( int ) const;
	jhm::transf	getTransf( int ) const;
	jhm::vector	getPosition( int ) const;
	jhm::quater	getOrientation( int ) const;

	void	set( int, jhm::transf const& );
	void	setTransf( int, jhm::transf const& );
	void	setPosition( int, jhm::vector const& );
	void	setOrientation( int, jhm::quater const& );
	void	setOrientation( int, jhm::matrix const& );

	char	getMask( int ) const;
	void	setMask( int, char );

	PmMaskType getJointMask( PmHuman* ) const;

	void	applyTransf( jhm::transf const& );
	void	applyTransf( int, jhm::transf const& );
	void	interpolate( m_real, PmConstraint const&, PmConstraint const&, int type=PM_C_AND );
};

#endif
