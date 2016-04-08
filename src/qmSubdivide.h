// +-------------------------------------------------------------------------
// | qmSubdivide.h
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

#ifndef __QM_SUBDIVIDE_H
#define __QM_SUBDIVIDE_H



m_real QmCardinalFunction( int, int, m_real*, m_real );

void QmSubdivideWeights( int, m_real* );

void QmInterpolatorySubdivide( int, jhm::vector*, jhm::vector*, char* );
void QmInterpolatorySubdivide( int, jhm::quater*, jhm::quater*, char* );

void QmInterpolatorySubdivide( int, jhm::vector*, jhm::vector*, char*, m_real* );
void QmInterpolatorySubdivide( int, jhm::quater*, jhm::quater*, char*, m_real* );

void QmBsplineSubdivide( int, jhm::vector*, jhm::vector* );
void QmBsplineSubdivide( int, jhm::quater*, jhm::quater* );

#endif

