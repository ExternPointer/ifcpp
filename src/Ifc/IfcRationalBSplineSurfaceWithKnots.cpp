/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcBSplineSurfaceForm.h"
#include "ifcpp/Ifc/IfcCartesianPoint.h"
#include "ifcpp/Ifc/IfcInteger.h"
#include "ifcpp/Ifc/IfcKnotType.h"
#include "ifcpp/Ifc/IfcLogical.h"
#include "ifcpp/Ifc/IfcParameterValue.h"
#include "ifcpp/Ifc/IfcPresentationLayerAssignment.h"
#include "ifcpp/Ifc/IfcRationalBSplineSurfaceWithKnots.h"
#include "ifcpp/Ifc/IfcReal.h"
#include "ifcpp/Ifc/IfcStyledItem.h"

// ENTITY IfcRationalBSplineSurfaceWithKnots 
IFC4X3::IfcRationalBSplineSurfaceWithKnots::IfcRationalBSplineSurfaceWithKnots( int tag ) { m_tag = tag; }
void IFC4X3::IfcRationalBSplineSurfaceWithKnots::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCRATIONALBSPLINESURFACEWITHKNOTS" << "(";
	if( m_UDegree ) { m_UDegree->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_VDegree ) { m_VDegree->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	writeEntityList2D( stream, m_ControlPointsList );
	stream << ",";
	if( m_SurfaceForm ) { m_SurfaceForm->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_UClosed ) { m_UClosed->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_VClosed ) { m_VClosed->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_SelfIntersect ) { m_SelfIntersect->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	writeTypeOfIntList( stream, m_UMultiplicities, false );
	stream << ",";
	writeTypeOfIntList( stream, m_VMultiplicities, false );
	stream << ",";
	writeTypeOfRealList( stream, m_UKnots, false );
	stream << ",";
	writeTypeOfRealList( stream, m_VKnots, false );
	stream << ",";
	if( m_KnotSpec ) { m_KnotSpec->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	writeTypeOfRealList2D( stream, m_WeightsData, false );
	stream << ");";
}
void IFC4X3::IfcRationalBSplineSurfaceWithKnots::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcRationalBSplineSurfaceWithKnots::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 13 ){ std::stringstream err; err << "Wrong parameter count for entity IfcRationalBSplineSurfaceWithKnots, expecting 13, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_UDegree = IfcInteger::createObjectFromSTEP( args[0], map, errorStream );
	m_VDegree = IfcInteger::createObjectFromSTEP( args[1], map, errorStream );
	readEntityReferenceList2D( args[2], m_ControlPointsList, map, errorStream );
	m_SurfaceForm = IfcBSplineSurfaceForm::createObjectFromSTEP( args[3], map, errorStream );
	m_UClosed = IfcLogical::createObjectFromSTEP( args[4], map, errorStream );
	m_VClosed = IfcLogical::createObjectFromSTEP( args[5], map, errorStream );
	m_SelfIntersect = IfcLogical::createObjectFromSTEP( args[6], map, errorStream );
	readTypeOfIntegerList( args[7], m_UMultiplicities );
	readTypeOfIntegerList( args[8], m_VMultiplicities );
	readTypeOfRealList( args[9], m_UKnots );
	readTypeOfRealList( args[10], m_VKnots );
	m_KnotSpec = IfcKnotType::createObjectFromSTEP( args[11], map, errorStream );
	readTypeOfRealList2D( args[12], m_WeightsData );
}
void IFC4X3::IfcRationalBSplineSurfaceWithKnots::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcBSplineSurfaceWithKnots::getAttributes( vec_attributes );
	if( !m_WeightsData.empty() )
	{
		shared_ptr<AttributeObjectVector> outer_vector( new AttributeObjectVector() );
		vec_attributes.emplace_back( std::make_pair( "WeightsData", outer_vector ) );
		for( size_t ii=0; ii<m_WeightsData.size(); ++ii )
		{
			const std::vector<shared_ptr<IfcReal> >& vec_ii = m_WeightsData[ii];
			shared_ptr<AttributeObjectVector> inner_vector( new AttributeObjectVector() );
			outer_vector->m_vec.push_back( inner_vector );
			std::copy(vec_ii.begin(), vec_ii.end(), std::back_inserter(inner_vector->m_vec));
		}
	}
}
void IFC4X3::IfcRationalBSplineSurfaceWithKnots::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcBSplineSurfaceWithKnots::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcRationalBSplineSurfaceWithKnots::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcBSplineSurfaceWithKnots::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcRationalBSplineSurfaceWithKnots::unlinkFromInverseCounterparts()
{
	IfcBSplineSurfaceWithKnots::unlinkFromInverseCounterparts();
}