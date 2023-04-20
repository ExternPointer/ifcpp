/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcBSplineCurveForm.h"
#include "ifcpp/Ifc/IfcCartesianPoint.h"
#include "ifcpp/Ifc/IfcInteger.h"
#include "ifcpp/Ifc/IfcKnotType.h"
#include "ifcpp/Ifc/IfcLogical.h"
#include "ifcpp/Ifc/IfcParameterValue.h"
#include "ifcpp/Ifc/IfcPresentationLayerAssignment.h"
#include "ifcpp/Ifc/IfcRationalBSplineCurveWithKnots.h"
#include "ifcpp/Ifc/IfcReal.h"
#include "ifcpp/Ifc/IfcStyledItem.h"

// ENTITY IfcRationalBSplineCurveWithKnots 
IFC4X3::IfcRationalBSplineCurveWithKnots::IfcRationalBSplineCurveWithKnots( int tag ) { m_tag = tag; }
void IFC4X3::IfcRationalBSplineCurveWithKnots::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCRATIONALBSPLINECURVEWITHKNOTS" << "(";
	if( m_Degree ) { m_Degree->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	writeEntityList( stream, m_ControlPointsList );
	stream << ",";
	if( m_CurveForm ) { m_CurveForm->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ClosedCurve ) { m_ClosedCurve->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_SelfIntersect ) { m_SelfIntersect->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	writeTypeOfIntList( stream, m_KnotMultiplicities, false );
	stream << ",";
	writeTypeOfRealList( stream, m_Knots, false );
	stream << ",";
	if( m_KnotSpec ) { m_KnotSpec->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	writeTypeOfRealList( stream, m_WeightsData, false );
	stream << ");";
}
void IFC4X3::IfcRationalBSplineCurveWithKnots::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcRationalBSplineCurveWithKnots::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 9 ){ std::stringstream err; err << "Wrong parameter count for entity IfcRationalBSplineCurveWithKnots, expecting 9, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Degree = IfcInteger::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReferenceList( args[1], m_ControlPointsList, map, errorStream );
	m_CurveForm = IfcBSplineCurveForm::createObjectFromSTEP( args[2], map, errorStream );
	m_ClosedCurve = IfcLogical::createObjectFromSTEP( args[3], map, errorStream );
	m_SelfIntersect = IfcLogical::createObjectFromSTEP( args[4], map, errorStream );
	readTypeOfIntegerList( args[5], m_KnotMultiplicities );
	readTypeOfRealList( args[6], m_Knots );
	m_KnotSpec = IfcKnotType::createObjectFromSTEP( args[7], map, errorStream );
	readTypeOfRealList( args[8], m_WeightsData );
}
void IFC4X3::IfcRationalBSplineCurveWithKnots::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcBSplineCurveWithKnots::getAttributes( vec_attributes );
	if( !m_WeightsData.empty() )
	{
		shared_ptr<AttributeObjectVector> WeightsData_vec_object( new AttributeObjectVector() );
		std::copy( m_WeightsData.begin(), m_WeightsData.end(), std::back_inserter( WeightsData_vec_object->m_vec ) );
		vec_attributes.emplace_back( std::make_pair( "WeightsData", WeightsData_vec_object ) );
	}
}
void IFC4X3::IfcRationalBSplineCurveWithKnots::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcBSplineCurveWithKnots::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcRationalBSplineCurveWithKnots::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcBSplineCurveWithKnots::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcRationalBSplineCurveWithKnots::unlinkFromInverseCounterparts()
{
	IfcBSplineCurveWithKnots::unlinkFromInverseCounterparts();
}