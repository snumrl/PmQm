
#include "MATHCLASS/mathclass.h"
#include "qmComplex.h"

using namespace jhm;

QmHalfspace*
QmComplex::addHalfspace( QmHalfspace* h )
{
	num++;

    h->setNext( this->root );
    if ( h->getNext() ) h->getNext()->setPrev( h );
    this->root = h;

    return h;
}

QmHalfspace*
QmComplex::removeHalfspace( QmHalfspace* h )
{
	num--;

    if ( this->root==h ) this->root = this->root->getNext();

    if ( h->getNext() ) h->getNext()->setPrev( h->getPrev() );
    if ( h->getPrev() ) h->getPrev()->setNext( h->getNext() );

    h->setNext( NULL );
    h->setPrev( NULL );

    return h;
}

int
QmComplex::inclusion( quater const& q )
{
	for( QmHalfspace *h=root; h; h=h->getNext() )
		if ( !h->inclusion(q) && !h->inclusion(-q) ) return FALSE;

	return TRUE;
}

int
QmComplex::intersection( QmGeodesic const&, quater* )
{
	return -1;
}

m_real
QmComplex::distance( quater const& q )
{
	m_real d=0.0, t;

	for( QmHalfspace *h=root; h; h=h->getNext() )
	{
		if ( !h->inclusion(q) && !h->inclusion(-q) )
		{
			t = ::distance( q, h->nearest(q) );
			d += t*t;
		}
	}

	return sqrt(d);
}

vector
QmComplex::gradient( quater const& q )
{
	vector grad(0,0,0);
	quater p;

	for( QmHalfspace *h=root; h; h=h->getNext() )
	{
		p = h->nearest( q );
		if ( p%q < 0 ) p = -p;

		grad += ln( q.inverse() * p );
	}

	return grad;
}

quater
QmComplex::project( quater const& q )
{
	int flag;
	quater p = q;

	for( int i=0; i<5; i++ )
	{
		flag = TRUE;

		for( QmHalfspace *h=root; h; h=h->getNext() )
		{
			if ( !h->inclusion(p) && !h->inclusion(-p) )
			{
				flag = FALSE;
				p = h->nearest( p );
			}
		}

		if ( flag ) break;
	}

	return p;
}

matrix
QmComplex::project( matrix const& m )
{
	return Quater2Matrix( project( Matrix2Quater(m) ) );
}

transf
QmComplex::project( transf const& t )
{
	matrix m = t.getAffine();
	m = Quater2Matrix( project( Matrix2Quater(m) ) );
	return transf( m, t.getTranslation() );
}

std::ostream&
operator<<( std::ostream& os, QmComplex const& c )
{
    os << "[ " << c.num;
    for( QmHalfspace *h=c.root; h; h=h->getNext() )
	{
		os << " ";
		(*h).write(os);
	}
	os << " ]" << std::endl;

    return os;
}

std::istream&
operator>>( std::istream& is, QmComplex& c )
{
	int n;
	char buf;
    is >> buf >> n;
	for( int i=0; i<n; i++ )
	{
		char ch;
		is >> buf >> ch;
		is.putback( ch );
		is.putback( '#' );

		QmConicHS *h1;
		QmAxialHS *h2;
		QmSphericalHS *h3;

		switch(ch)
		{
		  case 'C':
			h1 = new QmConicHS;
			is >> (*h1);
			c.addHalfspace( h1 );
			break;
		  case 'A':
			h2 = new QmAxialHS;
			is >> (*h2);
			c.addHalfspace( h2 );
			break;
		  case 'S':
			h3 = new QmSphericalHS;
			is >> (*h3);
			c.addHalfspace( h3 );
			break;
		}
	}
	is >> buf;

    return is;
}

