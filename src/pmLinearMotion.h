// +-------------------------------------------------------------------------
// | pmLinearMotion.h
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

#ifndef __PM_LINEAR_MOTION_H
#define __PM_LINEAR_MOTION_H



class PmLinearMotion : public PmMotion
{
  public:
	PmPosture*	postures;
	m_real*		knots;

	enum PmMomentType { PM_HEEL_STRIKE=0x01, PM_TOE_OFF=0x02 };

	//
	//		Constructors
	//
	PmLinearMotion();
	PmLinearMotion( PmHuman* );
	~PmLinearMotion();

	//
	//		Set and get parameters
	//
	void		setBody( PmHuman* );
	void		setSize( int );
	void		extend( int );

	PmMaskType	getMask() const;
	int			getDOF() const;

	void        getTranslation( jhm::vector* ) const;
	void        getRotation( int, jhm::quater* ) const;

	void        setTranslation( jhm::vector* );
	void        setRotation( int, jhm::quater* );

	void        addTranslation( jhm::vector* );
	void        addRotation( int, jhm::vector* );

	void		setPosture( int, PmPosture const& );
	PmPosture&	getPosture( int ) const;
	
	void		setScale( m_real );

	void		setKnots( m_real* );
	void		getKnots( m_real* );

	m_real		getKnot( int i )
				{ assert( 0<=i && i<getSize() ); return knots[i]; }
	void		setKnot( int i, m_real k )
				{ assert( 0<=i && i<getSize() ); knots[i] = k; }

	//
	//		Manipulation & Editing
	//
	void		crop( PmLinearMotion const&, int, int );
	void		alignment();
	void		correct();

    PmLinearMotion&	operator=( PmLinearMotion const& );
	PmLinearMotion& operator+=( PmVectorArray const& );

	void		addDisplacement( PmVectorArray const&, m_real );

	void		directManipScale( int, m_real, int );
	void		adjustRootTrajectory( PmConstraint*, int, int );
	void		adjustRootTrajectory( int, PmConstraint*, int, int );
	void		adjustRootTrajectory(
						  jhm::vector[][PM_HUMAN_NUM_LINKS],
						  PmConstraint*, int, int);
	void		retarget( jhm::vector joint_list[][PM_HUMAN_NUM_LINKS],
						  PmConstraint*, int, int, char,
						  char optimize=FALSE );
	void		retarget( PmConstraint*, int, int, char,
						  char optimize=FALSE );
	void		directManip(
						  int, PmPosture const&, PmConstraint*,
						  int, int, char, char optimize=FALSE, int repeat=1 );
	void		headWarp( PmPosture const&, PmVector const& );
	void		tailWarp( PmPosture const&, PmVector const& );

	void		concat( PmLinearMotion const& );
	jhm::transf		stitch( PmLinearMotion const& );
	jhm::transf		stitchWithWarp( PmLinearMotion& );
	jhm::transf		connect( PmLinearMotion& );
	void		applyTransf( const jhm::transf& );

	void		steerLocomotion( m_real angle );
	void		steerLocomotion( m_real angle, int start, int end );
	void		updateConstraintsAfterWarp();

	void		centering();
	void		flatTune( int );

	void		mirrorReflect( PmLinearMotion const& );

	//
	//		Constraints
	//
	void		setPelvisConstraints( m_real );
	void		setHandConstraints( m_real, m_real );
	void		setFootConstraints( m_real, m_real, m_real, m_real );

	bool		isToeOff( int );
	bool		isLeftToeOff( int );
	bool		isRightToeOff( int );

	void		cleanupConstraints( int, int );

	//
	//		Evaluation & Resampling
	//
	int			evaluate( m_real, PmPosture&, PmConstraint&, int start=0 ) const;
	
	void		resampling( int, PmLinearMotion const& );
	void		resampling( int, int, int, PmLinearMotion const& );
	void		resampling( PmLinearMotion const&, PmMomentType,
							PmMaskType, int max_block=0 );

	//
	//		Physical properties
	//
	jhm::vector		getLinearVelocity( int, int ) const;
	jhm::vector		getAngularVelocity( int, int ) const;

	jhm::vector		getLinearAcceleration( int, int ) const;
	jhm::vector		getAngularAcceleration( int, int ) const;

	void		getVelocity( int, PmVector& );
	void		getAcceleration( int, PmVector& );

	void		getPositionVelocity( int, PmVector& );

	m_real		maxLinearVelocity( int );
	m_real		maxAngularVelocity( int );

	jhm::vector		getZMP( int ) const;

	m_real		getKineticEnergy( int ) const;


	//
	//		File I/O
	//
	int			openMOV( char*, m_real scale = 1.0 );
	int			saveMOV( char* );

	int			openCON( char* );
	int			saveCON( char* );

	enum { PM_AMC_BODY_BUILDER, PM_AMC_BODY_BUILDER_F,
		   PM_AMC_WOODY, PM_AMC_RACKET, PM_AMC_BASKETBALL };

	int			openAMC( char*, int, m_real scale = 1.0, int subsample = 1 );
	int			saveAMC( char*, int );

	int			openAMC_BB( char*, m_real scale = 1.0, int subsample = 1 );
	int			saveAMC_BB( char* );

	int			openAMC_BB_F( char*, m_real scale = 1.0, int subsample = 1 );
	int			saveAMC_BB_F( char* );

	int			openAMC_Woody( char*, m_real scale = 1.0, int subsample = 1 );
	int			saveAMC_Woody( char* );

	int			openAMC_Racket( char*, m_real scale = 1.0, int subsample = 1 );
	int			saveAMC_Racket( char* );

	int			openAMC_BasketBall( char*, m_real scale = 1.0, int subsample = 1 );
	int			saveAMC_BasketBall( char* );

	int			openAMC_Chaja( char*, m_real scale = 1.0, int subsample = 1 );
	int			saveAMC_Chaja( char* );

	int			openAMC_Casual04( char*, m_real scale = 1.0, int subsample = 1 );
	int			saveAMC_Casual04( char* );

	int			openHTR( char*, m_real scale = 1.0 / 30.0 );

	int			openTXT( char*, m_real scale = 1.0 );

	int			openCCA( char*, m_real scale = 1.0 );
	int			saveCCA( char* );

	//
	//		Multiresolution Signal Processing
	//
	void		FPfilter( void (*func1)(int,jhm::vector*,jhm::vector*),
						  void (*func2)(int,jhm::quater*,jhm::vector*),
						  int, PmConstraint* );

	void		LTIfilter( void (*func1)(int,jhm::vector*,int),
						   void (*func2)(int,jhm::quater*,int),
						   int, PmMaskType const& );
	void		LTIfilter( void (*func1)(int,jhm::vector*,int,int,int),
                           void (*func2)(int,jhm::quater*,int,int,int),
						   int, PmMaskType const&, int, int );

	void		LTIfilter( int, PmMaskType const&,
						   int, m_real*, int, m_real* );
	void		LTIfilter( int, PmMaskType const&,
						   int, m_real*, int, m_real*, int, int );

	void		smoothing( int, PmMaskType const&, int, int );

	void		subdivide( PmLinearMotion const&, int, char*, m_real* );
	void		subdivide( PmLinearMotion const& );

	void		downSampling( int l=1 );
	void		uniformDownSampling( PmLinearMotion const&, char* );
	void		nonuniformDownSampling( PmLinearMotion&, char*, m_real* );
};

#endif
