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
#include "ifcpp/Ifc/IfcObjectDefinition.h"
#include "ifcpp/Ifc/IfcObjectTypeEnum.h"
#include "ifcpp/Ifc/IfcOwnerHistory.h"
#include "ifcpp/Ifc/IfcRelAssignsToGroup.h"
#include "ifcpp/Ifc/IfcText.h"

// ENTITY IfcRelAssignsToGroup 
IFC4X3::IfcRelAssignsToGroup::IfcRelAssignsToGroup( int tag ) { m_tag = tag; }
void IFC4X3::IfcRelAssignsToGroup::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCRELASSIGNSTOGROUP" << "(";
	if( m_GlobalId ) { m_GlobalId->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_OwnerHistory ) { stream << "#" << m_OwnerHistory->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	writeEntityList( stream, m_RelatedObjects );
	stream << ",";
	if( m_RelatedObjectsType ) { m_RelatedObjectsType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_RelatingGroup ) { stream << "#" << m_RelatingGroup->m_tag; } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcRelAssignsToGroup::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcRelAssignsToGroup::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 7 ){ std::stringstream err; err << "Wrong parameter count for entity IfcRelAssignsToGroup, expecting 7, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_GlobalId = IfcGloballyUniqueId::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReference( args[1], m_OwnerHistory, map, errorStream );
	m_Name = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[3], map, errorStream );
	readEntityReferenceList( args[4], m_RelatedObjects, map, errorStream );
	m_RelatedObjectsType = IfcObjectTypeEnum::createObjectFromSTEP( args[5], map, errorStream );
	readEntityReference( args[6], m_RelatingGroup, map, errorStream );
}
void IFC4X3::IfcRelAssignsToGroup::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcRelAssigns::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "RelatingGroup", m_RelatingGroup ) );
}
void IFC4X3::IfcRelAssignsToGroup::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcRelAssigns::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcRelAssignsToGroup::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcRelAssigns::setInverseCounterparts( ptr_self_entity );
	shared_ptr<IfcRelAssignsToGroup> ptr_self = dynamic_pointer_cast<IfcRelAssignsToGroup>( ptr_self_entity );
	if( !ptr_self ) { throw BuildingException( "IfcRelAssignsToGroup::setInverseCounterparts: type mismatch" ); }
	if( m_RelatingGroup )
	{
		m_RelatingGroup->m_IsGroupedBy_inverse.emplace_back( ptr_self );
	}
}
void IFC4X3::IfcRelAssignsToGroup::unlinkFromInverseCounterparts()
{
	IfcRelAssigns::unlinkFromInverseCounterparts();
	if( m_RelatingGroup )
	{
		std::vector<weak_ptr<IfcRelAssignsToGroup> >& IsGroupedBy_inverse = m_RelatingGroup->m_IsGroupedBy_inverse;
		for( auto it_IsGroupedBy_inverse = IsGroupedBy_inverse.begin(); it_IsGroupedBy_inverse != IsGroupedBy_inverse.end(); )
		{
			weak_ptr<IfcRelAssignsToGroup> self_candidate_weak = *it_IsGroupedBy_inverse;
			if( self_candidate_weak.expired() )
			{
				++it_IsGroupedBy_inverse;
				continue;
			}
			shared_ptr<IfcRelAssignsToGroup> self_candidate( *it_IsGroupedBy_inverse );
			if( self_candidate.get() == this )
			{
				it_IsGroupedBy_inverse= IsGroupedBy_inverse.erase( it_IsGroupedBy_inverse );
			}
			else
			{
				++it_IsGroupedBy_inverse;
			}
		}
	}
}
