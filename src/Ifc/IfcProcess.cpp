/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcGloballyUniqueId.h"
#include "ifcpp/Ifc/IfcIdentifier.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcOwnerHistory.h"
#include "ifcpp/Ifc/IfcProcess.h"
#include "ifcpp/Ifc/IfcRelAggregates.h"
#include "ifcpp/Ifc/IfcRelAssigns.h"
#include "ifcpp/Ifc/IfcRelAssignsToProcess.h"
#include "ifcpp/Ifc/IfcRelAssociates.h"
#include "ifcpp/Ifc/IfcRelDeclares.h"
#include "ifcpp/Ifc/IfcRelDefinesByObject.h"
#include "ifcpp/Ifc/IfcRelDefinesByProperties.h"
#include "ifcpp/Ifc/IfcRelDefinesByType.h"
#include "ifcpp/Ifc/IfcRelNests.h"
#include "ifcpp/Ifc/IfcRelSequence.h"
#include "ifcpp/Ifc/IfcText.h"

// ENTITY IfcProcess 
IFC4X3::IfcProcess::IfcProcess( int tag ) { m_tag = tag; }
void IFC4X3::IfcProcess::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCPROCESS" << "(";
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
	if( m_Identification ) { m_Identification->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_LongDescription ) { m_LongDescription->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcProcess::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcProcess::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 7 ){ std::stringstream err; err << "Wrong parameter count for entity IfcProcess, expecting 7, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_GlobalId = IfcGloballyUniqueId::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReference( args[1], m_OwnerHistory, map, errorStream );
	m_Name = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[3], map, errorStream );
	m_ObjectType = IfcLabel::createObjectFromSTEP( args[4], map, errorStream );
	m_Identification = IfcIdentifier::createObjectFromSTEP( args[5], map, errorStream );
	m_LongDescription = IfcText::createObjectFromSTEP( args[6], map, errorStream );
}
void IFC4X3::IfcProcess::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcObject::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "Identification", m_Identification ) );
	vec_attributes.emplace_back( std::make_pair( "LongDescription", m_LongDescription ) );
}
void IFC4X3::IfcProcess::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcObject::getAttributesInverse( vec_attributes_inverse );
	shared_ptr<AttributeObjectVector> IsPredecessorTo_inverse_vec_obj( new AttributeObjectVector() );
	for( size_t i=0; i<m_IsPredecessorTo_inverse.size(); ++i )
	{
		if( !m_IsPredecessorTo_inverse[i].expired() )
		{
			IsPredecessorTo_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcRelSequence>( m_IsPredecessorTo_inverse[i] ) );
		}
	}
	vec_attributes_inverse.emplace_back( std::make_pair( "IsPredecessorTo_inverse", IsPredecessorTo_inverse_vec_obj ) );
	shared_ptr<AttributeObjectVector> IsSuccessorFrom_inverse_vec_obj( new AttributeObjectVector() );
	for( size_t i=0; i<m_IsSuccessorFrom_inverse.size(); ++i )
	{
		if( !m_IsSuccessorFrom_inverse[i].expired() )
		{
			IsSuccessorFrom_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcRelSequence>( m_IsSuccessorFrom_inverse[i] ) );
		}
	}
	vec_attributes_inverse.emplace_back( std::make_pair( "IsSuccessorFrom_inverse", IsSuccessorFrom_inverse_vec_obj ) );
	shared_ptr<AttributeObjectVector> OperatesOn_inverse_vec_obj( new AttributeObjectVector() );
	for( size_t i=0; i<m_OperatesOn_inverse.size(); ++i )
	{
		if( !m_OperatesOn_inverse[i].expired() )
		{
			OperatesOn_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcRelAssignsToProcess>( m_OperatesOn_inverse[i] ) );
		}
	}
	vec_attributes_inverse.emplace_back( std::make_pair( "OperatesOn_inverse", OperatesOn_inverse_vec_obj ) );
}
void IFC4X3::IfcProcess::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcObject::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcProcess::unlinkFromInverseCounterparts()
{
	IfcObject::unlinkFromInverseCounterparts();
}
