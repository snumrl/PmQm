// +-------------------------------------------------------------------------
// | pmMRmotion.h
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

#ifndef __PM_MULTI_RES_MOTION_H
#define __PM_MULTI_RES_MOTION_H



#define PM_MOTION_MAX_LEVEL		20

class PmMRmotion : public PmMotion
{
  private:
	int				num_levels;
	PmMaskType		mask;
	int				num_dof;

  public:
	PmLinearMotion 	base_level;
	PmVectorArray	displace [PM_MOTION_MAX_LEVEL];
	int				length   [PM_MOTION_MAX_LEVEL];
	char*			mask_list[PM_MOTION_MAX_LEVEL];
	m_real*			knot_seqs[PM_MOTION_MAX_LEVEL];

  public:
	//
	// Constructors
	//
	PmMRmotion();
	PmMRmotion( PmHuman* );
	~PmMRmotion();

	//
	// Set and get parameters
	//
	PmMaskType	getMask() const { return mask; }
	int			getDOF() const  { return num_dof; }

	void		setBody( PmHuman* );

	PmLinearMotion&	getBaseLevel() { return base_level; }
	void		setBaseLevel( PmLinearMotion const& m ) { base_level = m; }

	int			getNumLevels() const { return num_levels; }
	int			getLevelLength( int i ) const { return length[i]; }

	void		buildHierarchy( int );
	void		buildHierarchy( PmLinearMotion const&,
								int max_level = PM_MOTION_MAX_LEVEL );

	void		linearize( PmLinearMotion& ) const;
	void		linearize( PmLinearMotion&, int, float* ) const;
	void		linearize( PmLinearMotion&, int ) const;
	void		linearize( PmLinearMotion&, m_real ) const;

	jhm::transf		stitch( PmMRmotion const&, PmMRmotion const& );

	void		reconstructDetail( int, PmMRmotion** );

	PmMRmotion&	operator=( PmMRmotion const& );
};

#endif
