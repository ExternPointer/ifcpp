/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcCurve.h"
#include "ifcpp/Ifc/IfcCurveMeasureSelect.h"
#include "ifcpp/Ifc/IfcLengthMeasure.h"
#include "ifcpp/Ifc/IfcPointByDistanceExpression.h"
#include "ifcpp/Ifc/IfcPresentationLayerAssignment.h"
#include "ifcpp/Ifc/IfcStyledItem.h"

// ENTITY IfcPointByDistanceExpression 
IFC4X3::IfcPointByDistanceExpression::IfcPointByDistanceExpression( int tag ) { m_tag = tag; }
void IFC4X3::IfcPointByDistanceExpression::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCPOINTBYDISTANCEEXPRESSION" << "(";
	if( m_DistanceAlong ) { m_DistanceAlong->getStepParameter( stream, true ); } else { stream << "$" ; }
	stream << ",";
	if( m_OffsetLateral ) { m_OffsetLateral->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_OffsetVertical ) { m_OffsetVertical->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_OffsetLongitudinal ) { m_OffsetLongitudinal->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_BasisCurve ) { stream << "#" << m_BasisCurve->m_tag; } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcPointByDistanceExpression::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcPointByDistanceExpression::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 5 ){ std::stringstream err; err << "Wrong parameter count for entity IfcPointByDistanceExpression, expecting 5, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_DistanceAlong = IfcCurveMeasureSelect::createObjectFromSTEP( args[0], map, errorStream );
	m_OffsetLateral = IfcLengthMeasure::createObjectFromSTEP( args[1], map, errorStream );
	m_OffsetVertical = IfcLengthMeasure::createObjectFromSTEP( args[2], map, errorStream );
	m_OffsetLongitudinal = IfcLengthMeasure::createObjectFromSTEP( args[3], map, errorStream );
	readEntityReference( args[4], m_BasisCurve, map, errorStream );
}
void IFC4X3::IfcPointByDistanceExpression::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcPoint::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "DistanceAlong", m_DistanceAlong ) );
	vec_attributes.emplace_back( std::make_pair( "OffsetLateral", m_OffsetLateral ) );
	vec_attributes.emplace_back( std::make_pair( "OffsetVertical", m_OffsetVertical ) );
	vec_attributes.emplace_back( std::make_pair( "OffsetLongitudinal", m_OffsetLongitudinal ) );
	vec_attributes.emplace_back( std::make_pair( "BasisCurve", m_BasisCurve ) );
}
void IFC4X3::IfcPointByDistanceExpression::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcPoint::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcPointByDistanceExpression::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcPoint::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcPointByDistanceExpression::unlinkFromInverseCounterparts()
{
	IfcPoint::unlinkFromInverseCounterparts();
}
