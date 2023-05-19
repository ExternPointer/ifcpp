/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcDistributionControlElement.h"
#include "ifcpp/Ifc/IfcGloballyUniqueId.h"
#include "ifcpp/Ifc/IfcIdentifier.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcObjectPlacement.h"
#include "ifcpp/Ifc/IfcOwnerHistory.h"
#include "ifcpp/Ifc/IfcProductRepresentation.h"
#include "ifcpp/Ifc/IfcRelAdheresToElement.h"
#include "ifcpp/Ifc/IfcRelAggregates.h"
#include "ifcpp/Ifc/IfcRelAssigns.h"
#include "ifcpp/Ifc/IfcRelAssignsToProduct.h"
#include "ifcpp/Ifc/IfcRelAssociates.h"
#include "ifcpp/Ifc/IfcRelConnectsElements.h"
#include "ifcpp/Ifc/IfcRelConnectsPortToElement.h"
#include "ifcpp/Ifc/IfcRelConnectsWithRealizingElements.h"
#include "ifcpp/Ifc/IfcRelContainedInSpatialStructure.h"
#include "ifcpp/Ifc/IfcRelCoversBldgElements.h"
#include "ifcpp/Ifc/IfcRelDeclares.h"
#include "ifcpp/Ifc/IfcRelDefinesByObject.h"
#include "ifcpp/Ifc/IfcRelDefinesByProperties.h"
#include "ifcpp/Ifc/IfcRelDefinesByType.h"
#include "ifcpp/Ifc/IfcRelFillsElement.h"
#include "ifcpp/Ifc/IfcRelFlowControlElements.h"
#include "ifcpp/Ifc/IfcRelInterferesElements.h"
#include "ifcpp/Ifc/IfcRelNests.h"
#include "ifcpp/Ifc/IfcRelPositions.h"
#include "ifcpp/Ifc/IfcRelProjectsElement.h"
#include "ifcpp/Ifc/IfcRelReferencedInSpatialStructure.h"
#include "ifcpp/Ifc/IfcRelSpaceBoundary.h"
#include "ifcpp/Ifc/IfcRelVoidsElement.h"
#include "ifcpp/Ifc/IfcText.h"

// ENTITY IfcDistributionControlElement 
IFC4X3::IfcDistributionControlElement::IfcDistributionControlElement( int tag ) { m_tag = tag; }
void IFC4X3::IfcDistributionControlElement::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCDISTRIBUTIONCONTROLELEMENT" << "(";
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
	if( m_Tag ) { m_Tag->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcDistributionControlElement::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcDistributionControlElement::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 8 ){ std::stringstream err; err << "Wrong parameter count for entity IfcDistributionControlElement, expecting 8, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_GlobalId = IfcGloballyUniqueId::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReference( args[1], m_OwnerHistory, map, errorStream );
	m_Name = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[3], map, errorStream );
	m_ObjectType = IfcLabel::createObjectFromSTEP( args[4], map, errorStream );
	readEntityReference( args[5], m_ObjectPlacement, map, errorStream );
	readEntityReference( args[6], m_Representation, map, errorStream );
	m_Tag = IfcIdentifier::createObjectFromSTEP( args[7], map, errorStream );
}
void IFC4X3::IfcDistributionControlElement::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcDistributionElement::getAttributes( vec_attributes );
}
void IFC4X3::IfcDistributionControlElement::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcDistributionElement::getAttributesInverse( vec_attributes_inverse );
	shared_ptr<AttributeObjectVector> AssignedToFlowElement_inverse_vec_obj( new AttributeObjectVector() );
	for( size_t i=0; i<m_AssignedToFlowElement_inverse.size(); ++i )
	{
		if( !m_AssignedToFlowElement_inverse[i].expired() )
		{
			AssignedToFlowElement_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcRelFlowControlElements>( m_AssignedToFlowElement_inverse[i] ) );
		}
	}
	vec_attributes_inverse.emplace_back( std::make_pair( "AssignedToFlowElement_inverse", AssignedToFlowElement_inverse_vec_obj ) );
}
void IFC4X3::IfcDistributionControlElement::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcDistributionElement::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcDistributionControlElement::unlinkFromInverseCounterparts()
{
	IfcDistributionElement::unlinkFromInverseCounterparts();
}
