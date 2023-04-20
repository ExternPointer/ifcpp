/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcBSplineCurve.h"
#include "ifcpp/Ifc/IfcBSplineCurveForm.h"
#include "ifcpp/Ifc/IfcCartesianPoint.h"
#include "ifcpp/Ifc/IfcInteger.h"
#include "ifcpp/Ifc/IfcLogical.h"
#include "ifcpp/Ifc/IfcPresentationLayerAssignment.h"
#include "ifcpp/Ifc/IfcStyledItem.h"

// ENTITY IfcBSplineCurve 
IFC4X3::IfcBSplineCurve::IfcBSplineCurve( int tag ) { m_tag = tag; }
void IFC4X3::IfcBSplineCurve::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCBSPLINECURVE" << "(";
	if( m_Degree ) { m_Degree->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	writeEntityList( stream, m_ControlPointsList );
	stream << ",";
	if( m_CurveForm ) { m_CurveForm->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ClosedCurve ) { m_ClosedCurve->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_SelfIntersect ) { m_SelfIntersect->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcBSplineCurve::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcBSplineCurve::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 5 ){ std::stringstream err; err << "Wrong parameter count for entity IfcBSplineCurve, expecting 5, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Degree = IfcInteger::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReferenceList( args[1], m_ControlPointsList, map, errorStream );
	m_CurveForm = IfcBSplineCurveForm::createObjectFromSTEP( args[2], map, errorStream );
	m_ClosedCurve = IfcLogical::createObjectFromSTEP( args[3], map, errorStream );
	m_SelfIntersect = IfcLogical::createObjectFromSTEP( args[4], map, errorStream );
}
void IFC4X3::IfcBSplineCurve::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcBoundedCurve::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "Degree", m_Degree ) );
	if( !m_ControlPointsList.empty() )
	{
		shared_ptr<AttributeObjectVector> ControlPointsList_vec_object( new AttributeObjectVector() );
		std::copy( m_ControlPointsList.begin(), m_ControlPointsList.end(), std::back_inserter( ControlPointsList_vec_object->m_vec ) );
		vec_attributes.emplace_back( std::make_pair( "ControlPointsList", ControlPointsList_vec_object ) );
	}
	vec_attributes.emplace_back( std::make_pair( "CurveForm", m_CurveForm ) );
	vec_attributes.emplace_back( std::make_pair( "ClosedCurve", m_ClosedCurve ) );
	vec_attributes.emplace_back( std::make_pair( "SelfIntersect", m_SelfIntersect ) );
}
void IFC4X3::IfcBSplineCurve::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcBoundedCurve::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcBSplineCurve::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcBoundedCurve::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcBSplineCurve::unlinkFromInverseCounterparts()
{
	IfcBoundedCurve::unlinkFromInverseCounterparts();
}