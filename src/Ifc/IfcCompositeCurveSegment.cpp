/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcBoolean.h"
#include "ifcpp/Ifc/IfcCompositeCurve.h"
#include "ifcpp/Ifc/IfcCompositeCurveSegment.h"
#include "ifcpp/Ifc/IfcCurve.h"
#include "ifcpp/Ifc/IfcPresentationLayerAssignment.h"
#include "ifcpp/Ifc/IfcStyledItem.h"
#include "ifcpp/Ifc/IfcTransitionCode.h"

// ENTITY IfcCompositeCurveSegment 
IFC4X3::IfcCompositeCurveSegment::IfcCompositeCurveSegment( int tag ) { m_tag = tag; }
void IFC4X3::IfcCompositeCurveSegment::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCCOMPOSITECURVESEGMENT" << "(";
	if( m_Transition ) { m_Transition->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_SameSense ) { m_SameSense->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ParentCurve ) { stream << "#" << m_ParentCurve->m_tag; } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcCompositeCurveSegment::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcCompositeCurveSegment::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 3 ){ std::stringstream err; err << "Wrong parameter count for entity IfcCompositeCurveSegment, expecting 3, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Transition = IfcTransitionCode::createObjectFromSTEP( args[0], map, errorStream );
	m_SameSense = IfcBoolean::createObjectFromSTEP( args[1], map, errorStream );
	readEntityReference( args[2], m_ParentCurve, map, errorStream );
}
void IFC4X3::IfcCompositeCurveSegment::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcSegment::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "SameSense", m_SameSense ) );
	vec_attributes.emplace_back( std::make_pair( "ParentCurve", m_ParentCurve ) );
}
void IFC4X3::IfcCompositeCurveSegment::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcSegment::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcCompositeCurveSegment::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcSegment::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcCompositeCurveSegment::unlinkFromInverseCounterparts()
{
	IfcSegment::unlinkFromInverseCounterparts();
}
