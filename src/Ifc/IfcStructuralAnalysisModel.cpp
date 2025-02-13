/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcAnalysisModelTypeEnum.h"
#include "ifcpp/Ifc/IfcAxis2Placement3D.h"
#include "ifcpp/Ifc/IfcGloballyUniqueId.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcObjectPlacement.h"
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
#include "ifcpp/Ifc/IfcRelServicesBuildings.h"
#include "ifcpp/Ifc/IfcStructuralAnalysisModel.h"
#include "ifcpp/Ifc/IfcStructuralLoadGroup.h"
#include "ifcpp/Ifc/IfcStructuralResultGroup.h"
#include "ifcpp/Ifc/IfcText.h"

// ENTITY IfcStructuralAnalysisModel 
IFC4X3::IfcStructuralAnalysisModel::IfcStructuralAnalysisModel( int tag ) { m_tag = tag; }
void IFC4X3::IfcStructuralAnalysisModel::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCSTRUCTURALANALYSISMODEL" << "(";
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
	if( m_PredefinedType ) { m_PredefinedType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_OrientationOf2DPlane ) { stream << "#" << m_OrientationOf2DPlane->m_tag; } else { stream << "$"; }
	stream << ",";
	writeEntityList( stream, m_LoadedBy );
	stream << ",";
	writeEntityList( stream, m_HasResults );
	stream << ",";
	if( m_SharedPlacement ) { stream << "#" << m_SharedPlacement->m_tag; } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcStructuralAnalysisModel::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcStructuralAnalysisModel::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 10 ){ std::stringstream err; err << "Wrong parameter count for entity IfcStructuralAnalysisModel, expecting 10, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_GlobalId = IfcGloballyUniqueId::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReference( args[1], m_OwnerHistory, map, errorStream );
	m_Name = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[3], map, errorStream );
	m_ObjectType = IfcLabel::createObjectFromSTEP( args[4], map, errorStream );
	m_PredefinedType = IfcAnalysisModelTypeEnum::createObjectFromSTEP( args[5], map, errorStream );
	readEntityReference( args[6], m_OrientationOf2DPlane, map, errorStream );
	readEntityReferenceList( args[7], m_LoadedBy, map, errorStream );
	readEntityReferenceList( args[8], m_HasResults, map, errorStream );
	readEntityReference( args[9], m_SharedPlacement, map, errorStream );
}
void IFC4X3::IfcStructuralAnalysisModel::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcSystem::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "PredefinedType", m_PredefinedType ) );
	vec_attributes.emplace_back( std::make_pair( "OrientationOf2DPlane", m_OrientationOf2DPlane ) );
	shared_ptr<AttributeObjectVector> LoadedBy_vec_object( new AttributeObjectVector() );
	std::copy( m_LoadedBy.begin(), m_LoadedBy.end(), std::back_inserter( LoadedBy_vec_object->m_vec ) );
	vec_attributes.emplace_back( std::make_pair( "LoadedBy", LoadedBy_vec_object ) );
	shared_ptr<AttributeObjectVector> HasResults_vec_object( new AttributeObjectVector() );
	std::copy( m_HasResults.begin(), m_HasResults.end(), std::back_inserter( HasResults_vec_object->m_vec ) );
	vec_attributes.emplace_back( std::make_pair( "HasResults", HasResults_vec_object ) );
	vec_attributes.emplace_back( std::make_pair( "SharedPlacement", m_SharedPlacement ) );
}
void IFC4X3::IfcStructuralAnalysisModel::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcSystem::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcStructuralAnalysisModel::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcSystem::setInverseCounterparts( ptr_self_entity );
	shared_ptr<IfcStructuralAnalysisModel> ptr_self = dynamic_pointer_cast<IfcStructuralAnalysisModel>( ptr_self_entity );
	if( !ptr_self ) { throw BuildingException( "IfcStructuralAnalysisModel::setInverseCounterparts: type mismatch" ); }
	for( size_t i=0; i<m_HasResults.size(); ++i )
	{
		if( m_HasResults[i] )
		{
			m_HasResults[i]->m_ResultGroupFor_inverse.emplace_back( ptr_self );
		}
	}
	for( size_t i=0; i<m_LoadedBy.size(); ++i )
	{
		if( m_LoadedBy[i] )
		{
			m_LoadedBy[i]->m_LoadGroupFor_inverse.emplace_back( ptr_self );
		}
	}
}
void IFC4X3::IfcStructuralAnalysisModel::unlinkFromInverseCounterparts()
{
	IfcSystem::unlinkFromInverseCounterparts();
	for( size_t i=0; i<m_HasResults.size(); ++i )
	{
		if( m_HasResults[i] )
		{
			std::vector<weak_ptr<IfcStructuralAnalysisModel> >& ResultGroupFor_inverse = m_HasResults[i]->m_ResultGroupFor_inverse;
			for( auto it_ResultGroupFor_inverse = ResultGroupFor_inverse.begin(); it_ResultGroupFor_inverse != ResultGroupFor_inverse.end(); )
			{
				weak_ptr<IfcStructuralAnalysisModel> self_candidate_weak = *it_ResultGroupFor_inverse;
				if( self_candidate_weak.expired() )
				{
					++it_ResultGroupFor_inverse;
					continue;
				}
				shared_ptr<IfcStructuralAnalysisModel> self_candidate( *it_ResultGroupFor_inverse );
				if( self_candidate.get() == this )
				{
					it_ResultGroupFor_inverse= ResultGroupFor_inverse.erase( it_ResultGroupFor_inverse );
				}
				else
				{
					++it_ResultGroupFor_inverse;
				}
			}
		}
	}
	for( size_t i=0; i<m_LoadedBy.size(); ++i )
	{
		if( m_LoadedBy[i] )
		{
			std::vector<weak_ptr<IfcStructuralAnalysisModel> >& LoadGroupFor_inverse = m_LoadedBy[i]->m_LoadGroupFor_inverse;
			for( auto it_LoadGroupFor_inverse = LoadGroupFor_inverse.begin(); it_LoadGroupFor_inverse != LoadGroupFor_inverse.end(); )
			{
				weak_ptr<IfcStructuralAnalysisModel> self_candidate_weak = *it_LoadGroupFor_inverse;
				if( self_candidate_weak.expired() )
				{
					++it_LoadGroupFor_inverse;
					continue;
				}
				shared_ptr<IfcStructuralAnalysisModel> self_candidate( *it_LoadGroupFor_inverse );
				if( self_candidate.get() == this )
				{
					it_LoadGroupFor_inverse= LoadGroupFor_inverse.erase( it_LoadGroupFor_inverse );
				}
				else
				{
					++it_LoadGroupFor_inverse;
				}
			}
		}
	}
}
