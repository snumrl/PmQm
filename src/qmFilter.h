// +-------------------------------------------------------------------------
// | qmFilter.h
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

#ifndef __QM_FILTER_H
#define __QM_FILTER_H



static m_real smooth_mask[] =
		{ -1.0/16.0,  4.0/16.0, 10.0/16.0,  4.0/16.0, -1.0/16.0 };

static m_real binomial_mask[] =
		{  1.0/16.0,  4.0/16.0,  6.0/16.0,  4.0/16.0,  1.0/16.0 };

static m_real sharp_mask[] =
		{ -1.0/16.0, -4.0/16.0, 26.0/16.0, -4.0/16.0, -1.0/16.0 };

static m_real smooth_mask_r[] =
		{  1.0/16.0, -3.0/16.0,  3.0/16.0, -1.0/16.0 };

static m_real binomial_mask_r[] =
		{ -1.0/16.0, -5.0/16.0,  5.0/16.0,  1.0/16.0 };

static m_real sharp_mask_r[] =
		{  1.0/16.0,  5.0/16.0, -5.0/16.0, -1.0/16.0 };

void QmAlignUnitQuaternions( int, jhm::quater* );

void QmLTIfilter( int, jhm::vector*, int, m_real*, int, int, int );
void QmLTIfilter( int, jhm::quater*, int, m_real*, int, int, int );

void QmSmoothing( int, jhm::vector*, m_real*, int, int, int );
void QmSmoothing( int, jhm::quater*, m_real*, int, int, int );

void QmSmoothing( int, jhm::quater*, jhm::vector* );
void QmSharpening( int, jhm::quater*, jhm::vector* );
void QmBinomial( int, jhm::quater*, jhm::vector* );

void QmSmoothing( int, jhm::vector*, jhm::vector* );
void QmSharpening( int, jhm::vector*, jhm::vector* );
void QmBinomial( int, jhm::vector*, jhm::vector* );

void QmSmoothing( int, jhm::quater*, int );
void QmSharpening( int, jhm::quater*, int );
void QmBinomial( int, jhm::quater*, int );

void QmSmoothing( int, jhm::vector*, int );
void QmSharpening( int, jhm::vector*, int );
void QmBinomial( int, jhm::vector*, int );

void QmSmoothing( int, jhm::quater*, int, int, int );
void QmSharpening( int, jhm::quater*, int, int, int );
void QmBinomial( int, jhm::quater*, int, int, int );

void QmSmoothing( int, jhm::vector*, int, int, int );
void QmSharpening( int, jhm::vector*, int, int, int );
void QmBinomial( int, jhm::vector*, int, int, int );

//
// Energy function
//
m_real QmSecondOrderEnergy( int, jhm::vector*, int print=FALSE );
m_real QmThirdOrderEnergy( int, jhm::vector*, int print=FALSE );
m_real QmFourthOrderEnergy( int, jhm::vector*, int print=FALSE );

m_real QmSecondOrderEnergy( int, jhm::quater*, int print=FALSE );
m_real QmThirdOrderEnergy( int, jhm::quater*, int print=FALSE );
m_real QmFourthOrderEnergy( int, jhm::quater*, int print=FALSE );

//
// Max Error
//
m_real QmSecondOrderMaxError( int, jhm::vector* );
m_real QmThirdOrderMaxError( int, jhm::vector* );
m_real QmFourthOrderMaxError( int, jhm::vector* );

m_real QmSecondOrderMaxError( int, jhm::quater* );
m_real QmThirdOrderMaxError( int, jhm::quater* );
m_real QmFourthOrderMaxError( int, jhm::quater* );

#endif
