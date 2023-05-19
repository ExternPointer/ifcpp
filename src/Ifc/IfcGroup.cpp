/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcGloballyUniqueId.h"
#include "ifcpp/Ifc/IfcGroup.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcOwnerHistory.h"
#include "ifcpp/Ifc/IfcRelAggregates.h"
#include "ifcpp/Ifc/IfcRelAssigns.h"
#include "ifcpp/Ifc/IfcRelAssignsToGroup.h"
#include "ifcpp/Ifc/IfcRelAssociates.h"
#include "ifcpp/Ifc/IfcRelDeclares.h"
#include "ifcpp/Ifc/IfcRelDefinesByObject.h"
#include "ifcpp/Ifc/IfcRelDefinesByProperties.h"
#include "ifcpp/Ifc/IfcRelDefinesByType.h"
#include "ifcpp/Ifc/IfcRelNests.h"
#include "ifcpp/Ifc/IfcRelReferencedInSpatialStructure.h"
#include "ifcpp/Ifc/IfcText.h"

// ENTITY IfcGroup 
IFC4X3::IfcGroup::IfcGroup( int tag ) { m_tag = tag; }
void IFC4X3::IfcGroup::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCGROUP" << "(";
	if( m_GlobalId ) { m_GlobalId->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_OwnerHistory ) { stream << "#" << m_OwnerHistory->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ObjectType ) { m_ObjectType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcGroup::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcGroup::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 5 ){ std::stringstream err; err << "Wrong parameter count for entity IfcGroup, expecting 5, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_GlobalId = IfcGloballyUniqueId::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReference( args[1], m_OwnerHistory, map, errorStream );
	m_Name = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[3], map, errorStream );
	m_ObjectType = IfcLabel::createObjectFromSTEP( args[4], map, errorStream );
}
void IFC4X3::IfcGroup::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcObject::getAttributes( vec_attributes );
}
void IFC4X3::IfcGroup::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcObject::getAttributesInverse( vec_attributes_inverse );
	shared_ptr<AttributeObjectVector> IsGroupedBy_inverse_vec_obj( new AttributeObjectVector() );
	for( size_t i=0; i<m_IsGroupedBy_inverse.size(); ++i )
	{
		if( !m_IsGroupedBy_inverse[i].expired() )
		{
			IsGroupedBy_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcRelAssignsToGroup>( m_IsGroupedBy_inverse[i] ) );
		}
	}
	vec_attributes_inverse.emplace_back( std::make_pair( "IsGroupedBy_inverse", IsGroupedBy_inverse_vec_obj ) );
	shared_ptr<AttributeObjectVector> ReferencedInStructures_inverse_vec_obj( new AttributeObjectVector() );
	for( size_t i=0; i<m_ReferencedInStructures_inverse.size(); ++i )
	{
		if( !m_ReferencedInStructures_inverse[i].expired() )
		{
			ReferencedInStructures_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcRelReferencedInSpatialStructure>( m_ReferencedInStructures_inverse[i] ) );
		}
	}
	vec_attributes_inverse.emplace_back( std::make_pair( "ReferencedInStructures_inverse", ReferencedInStructures_inverse_vec_obj ) );
}
void IFC4X3::IfcGroup::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcObject::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcGroup::unlinkFromInverseCounterparts()
{
	IfcObject::unlinkFromInverseCounterparts();
}
