/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcBoundaryCondition.h"
#include "ifcpp/Ifc/IfcGloballyUniqueId.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcObjectPlacement.h"
#include "ifcpp/Ifc/IfcOwnerHistory.h"
#include "ifcpp/Ifc/IfcProductRepresentation.h"
#include "ifcpp/Ifc/IfcRelAggregates.h"
#include "ifcpp/Ifc/IfcRelAssigns.h"
#include "ifcpp/Ifc/IfcRelAssignsToProduct.h"
#include "ifcpp/Ifc/IfcRelAssociates.h"
#include "ifcpp/Ifc/IfcRelConnectsStructuralActivity.h"
#include "ifcpp/Ifc/IfcRelConnectsStructuralMember.h"
#include "ifcpp/Ifc/IfcRelDeclares.h"
#include "ifcpp/Ifc/IfcRelDefinesByObject.h"
#include "ifcpp/Ifc/IfcRelDefinesByProperties.h"
#include "ifcpp/Ifc/IfcRelDefinesByType.h"
#include "ifcpp/Ifc/IfcRelNests.h"
#include "ifcpp/Ifc/IfcRelPositions.h"
#include "ifcpp/Ifc/IfcRelReferencedInSpatialStructure.h"
#include "ifcpp/Ifc/IfcStructuralConnection.h"
#include "ifcpp/Ifc/IfcText.h"

// ENTITY IfcStructuralConnection 
IFC4X3::IfcStructuralConnection::IfcStructuralConnection( int tag ) { m_tag = tag; }
void IFC4X3::IfcStructuralConnection::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCSTRUCTURALCONNECTION" << "(";
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
	if( m_AppliedCondition ) { stream << "#" << m_AppliedCondition->m_tag; } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcStructuralConnection::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcStructuralConnection::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 8 ){ std::stringstream err; err << "Wrong parameter count for entity IfcStructuralConnection, expecting 8, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_GlobalId = IfcGloballyUniqueId::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReference( args[1], m_OwnerHistory, map, errorStream );
	m_Name = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[3], map, errorStream );
	m_ObjectType = IfcLabel::createObjectFromSTEP( args[4], map, errorStream );
	readEntityReference( args[5], m_ObjectPlacement, map, errorStream );
	readEntityReference( args[6], m_Representation, map, errorStream );
	readEntityReference( args[7], m_AppliedCondition, map, errorStream );
}
void IFC4X3::IfcStructuralConnection::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcStructuralItem::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "AppliedCondition", m_AppliedCondition ) );
}
void IFC4X3::IfcStructuralConnection::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcStructuralItem::getAttributesInverse( vec_attributes_inverse );
	if( !m_ConnectsStructuralMembers_inverse.empty() )
	{
		shared_ptr<AttributeObjectVector> ConnectsStructuralMembers_inverse_vec_obj( new AttributeObjectVector() );
		for( size_t i=0; i<m_ConnectsStructuralMembers_inverse.size(); ++i )
		{
			if( !m_ConnectsStructuralMembers_inverse[i].expired() )
			{
				ConnectsStructuralMembers_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcRelConnectsStructuralMember>( m_ConnectsStructuralMembers_inverse[i] ) );
			}
		}
		vec_attributes_inverse.emplace_back( std::make_pair( "ConnectsStructuralMembers_inverse", ConnectsStructuralMembers_inverse_vec_obj ) );
	}
}
void IFC4X3::IfcStructuralConnection::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcStructuralItem::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcStructuralConnection::unlinkFromInverseCounterparts()
{
	IfcStructuralItem::unlinkFromInverseCounterparts();
}