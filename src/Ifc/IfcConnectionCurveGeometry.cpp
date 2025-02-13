/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcConnectionCurveGeometry.h"
#include "ifcpp/Ifc/IfcCurveOrEdgeCurve.h"

// ENTITY IfcConnectionCurveGeometry 
IFC4X3::IfcConnectionCurveGeometry::IfcConnectionCurveGeometry( int tag ) { m_tag = tag; }
void IFC4X3::IfcConnectionCurveGeometry::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCCONNECTIONCURVEGEOMETRY" << "(";
	if( m_CurveOnRelatingElement ) { m_CurveOnRelatingElement->getStepParameter( stream, true ); } else { stream << "$" ; }
	stream << ",";
	if( m_CurveOnRelatedElement ) { m_CurveOnRelatedElement->getStepParameter( stream, true ); } else { stream << "$" ; }
	stream << ");";
}
void IFC4X3::IfcConnectionCurveGeometry::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcConnectionCurveGeometry::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 2 ){ std::stringstream err; err << "Wrong parameter count for entity IfcConnectionCurveGeometry, expecting 2, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_CurveOnRelatingElement = IfcCurveOrEdgeCurve::createObjectFromSTEP( args[0], map, errorStream );
	m_CurveOnRelatedElement = IfcCurveOrEdgeCurve::createObjectFromSTEP( args[1], map, errorStream );
}
void IFC4X3::IfcConnectionCurveGeometry::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcConnectionGeometry::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "CurveOnRelatingElement", m_CurveOnRelatingElement ) );
	vec_attributes.emplace_back( std::make_pair( "CurveOnRelatedElement", m_CurveOnRelatedElement ) );
}
void IFC4X3::IfcConnectionCurveGeometry::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcConnectionGeometry::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcConnectionCurveGeometry::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcConnectionGeometry::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcConnectionCurveGeometry::unlinkFromInverseCounterparts()
{
	IfcConnectionGeometry::unlinkFromInverseCounterparts();
}
