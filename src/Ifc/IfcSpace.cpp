/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcElementCompositionEnum.h"
#include "ifcpp/Ifc/IfcGloballyUniqueId.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcLengthMeasure.h"
#include "ifcpp/Ifc/IfcObjectPlacement.h"
#include "ifcpp/Ifc/IfcOwnerHistory.h"
#include "ifcpp/Ifc/IfcProductRepresentation.h"
#include "ifcpp/Ifc/IfcRelAggregates.h"
#include "ifcpp/Ifc/IfcRelAssigns.h"
#include "ifcpp/Ifc/IfcRelAssignsToProduct.h"
#include "ifcpp/Ifc/IfcRelAssociates.h"
#include "ifcpp/Ifc/IfcRelContainedInSpatialStructure.h"
#include "ifcpp/Ifc/IfcRelCoversSpaces.h"
#include "ifcpp/Ifc/IfcRelDeclares.h"
#include "ifcpp/Ifc/IfcRelDefinesByObject.h"
#include "ifcpp/Ifc/IfcRelDefinesByProperties.h"
#include "ifcpp/Ifc/IfcRelDefinesByType.h"
#include "ifcpp/Ifc/IfcRelInterferesElements.h"
#include "ifcpp/Ifc/IfcRelNests.h"
#include "ifcpp/Ifc/IfcRelPositions.h"
#include "ifcpp/Ifc/IfcRelReferencedInSpatialStructure.h"
#include "ifcpp/Ifc/IfcRelServicesBuildings.h"
#include "ifcpp/Ifc/IfcRelSpaceBoundary.h"
#include "ifcpp/Ifc/IfcSpace.h"
#include "ifcpp/Ifc/IfcSpaceTypeEnum.h"
#include "ifcpp/Ifc/IfcText.h"

// ENTITY IfcSpace 
IFC4X3::IfcSpace::IfcSpace( int tag ) { m_tag = tag; }
void IFC4X3::IfcSpace::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCSPACE" << "(";
	if( m_GlobalId ) { m_GlobalId->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_OwnerHistory ) { stream << "#" << m_OwnerHistory->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ObjectType ) { m_ObjectType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ObjectPlacement ) { stream << "#" << m_ObjectPlacement->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Representation ) { stream << "#" << m_Representation->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_LongName ) { m_LongName->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_CompositionType ) { m_CompositionType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_PredefinedType ) { m_PredefinedType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ElevationWithFlooring ) { m_ElevationWithFlooring->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcSpace::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcSpace::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 11 ){ std::stringstream err; err << "Wrong parameter count for entity IfcSpace, expecting 11, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_GlobalId = IfcGloballyUniqueId::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReference( args[1], m_OwnerHistory, map, errorStream );
	m_Name = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[3], map, errorStream );
	m_ObjectType = IfcLabel::createObjectFromSTEP( args[4], map, errorStream );
	readEntityReference( args[5], m_ObjectPlacement, map, errorStream );
	readEntityReference( args[6], m_Representation, map, errorStream );
	m_LongName = IfcLabel::createObjectFromSTEP( args[7], map, errorStream );
	m_CompositionType = IfcElementCompositionEnum::createObjectFromSTEP( args[8], map, errorStream );
	m_PredefinedType = IfcSpaceTypeEnum::createObjectFromSTEP( args[9], map, errorStream );
	m_ElevationWithFlooring = IfcLengthMeasure::createObjectFromSTEP( args[10], map, errorStream );
}
void IFC4X3::IfcSpace::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcSpatialStructureElement::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "PredefinedType", m_PredefinedType ) );
	vec_attributes.emplace_back( std::make_pair( "ElevationWithFlooring", m_ElevationWithFlooring ) );
}
void IFC4X3::IfcSpace::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcSpatialStructureElement::getAttributesInverse( vec_attributes_inverse );
	shared_ptr<AttributeObjectVector> HasCoverings_inverse_vec_obj( new AttributeObjectVector() );
	for( size_t i=0; i<m_HasCoverings_inverse.size(); ++i )
	{
		if( !m_HasCoverings_inverse[i].expired() )
		{
			HasCoverings_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcRelCoversSpaces>( m_HasCoverings_inverse[i] ) );
		}
	}
	vec_attributes_inverse.emplace_back( std::make_pair( "HasCoverings_inverse", HasCoverings_inverse_vec_obj ) );
	shared_ptr<AttributeObjectVector> BoundedBy_inverse_vec_obj( new AttributeObjectVector() );
	for( size_t i=0; i<m_BoundedBy_inverse.size(); ++i )
	{
		if( !m_BoundedBy_inverse[i].expired() )
		{
			BoundedBy_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcRelSpaceBoundary>( m_BoundedBy_inverse[i] ) );
		}
	}
	vec_attributes_inverse.emplace_back( std::make_pair( "BoundedBy_inverse", BoundedBy_inverse_vec_obj ) );
}
void IFC4X3::IfcSpace::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcSpatialStructureElement::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcSpace::unlinkFromInverseCounterparts()
{
	IfcSpatialStructureElement::unlinkFromInverseCounterparts();
}
