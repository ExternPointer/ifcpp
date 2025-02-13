/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcConnectionGeometry.h"
#include "ifcpp/Ifc/IfcElement.h"
#include "ifcpp/Ifc/IfcGloballyUniqueId.h"
#include "ifcpp/Ifc/IfcInternalOrExternalEnum.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcOwnerHistory.h"
#include "ifcpp/Ifc/IfcPhysicalOrVirtualEnum.h"
#include "ifcpp/Ifc/IfcRelSpaceBoundary1stLevel.h"
#include "ifcpp/Ifc/IfcSpaceBoundarySelect.h"
#include "ifcpp/Ifc/IfcText.h"

// ENTITY IfcRelSpaceBoundary1stLevel 
IFC4X3::IfcRelSpaceBoundary1stLevel::IfcRelSpaceBoundary1stLevel( int tag ) { m_tag = tag; }
void IFC4X3::IfcRelSpaceBoundary1stLevel::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCRELSPACEBOUNDARY1STLEVEL" << "(";
	if( m_GlobalId ) { m_GlobalId->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_OwnerHistory ) { stream << "#" << m_OwnerHistory->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_RelatingSpace ) { m_RelatingSpace->getStepParameter( stream, true ); } else { stream << "$" ; }
	stream << ",";
	if( m_RelatedBuildingElement ) { stream << "#" << m_RelatedBuildingElement->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_ConnectionGeometry ) { stream << "#" << m_ConnectionGeometry->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_PhysicalOrVirtualBoundary ) { m_PhysicalOrVirtualBoundary->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_InternalOrExternalBoundary ) { m_InternalOrExternalBoundary->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ParentBoundary ) { stream << "#" << m_ParentBoundary->m_tag; } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcRelSpaceBoundary1stLevel::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcRelSpaceBoundary1stLevel::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 10 ){ std::stringstream err; err << "Wrong parameter count for entity IfcRelSpaceBoundary1stLevel, expecting 10, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_GlobalId = IfcGloballyUniqueId::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReference( args[1], m_OwnerHistory, map, errorStream );
	m_Name = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[3], map, errorStream );
	m_RelatingSpace = IfcSpaceBoundarySelect::createObjectFromSTEP( args[4], map, errorStream );
	readEntityReference( args[5], m_RelatedBuildingElement, map, errorStream );
	readEntityReference( args[6], m_ConnectionGeometry, map, errorStream );
	m_PhysicalOrVirtualBoundary = IfcPhysicalOrVirtualEnum::createObjectFromSTEP( args[7], map, errorStream );
	m_InternalOrExternalBoundary = IfcInternalOrExternalEnum::createObjectFromSTEP( args[8], map, errorStream );
	readEntityReference( args[9], m_ParentBoundary, map, errorStream );
}
void IFC4X3::IfcRelSpaceBoundary1stLevel::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcRelSpaceBoundary::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "ParentBoundary", m_ParentBoundary ) );
}
void IFC4X3::IfcRelSpaceBoundary1stLevel::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcRelSpaceBoundary::getAttributesInverse( vec_attributes_inverse );
	shared_ptr<AttributeObjectVector> InnerBoundaries_inverse_vec_obj( new AttributeObjectVector() );
	for( size_t i=0; i<m_InnerBoundaries_inverse.size(); ++i )
	{
		if( !m_InnerBoundaries_inverse[i].expired() )
		{
			InnerBoundaries_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcRelSpaceBoundary1stLevel>( m_InnerBoundaries_inverse[i] ) );
		}
	}
	vec_attributes_inverse.emplace_back( std::make_pair( "InnerBoundaries_inverse", InnerBoundaries_inverse_vec_obj ) );
}
void IFC4X3::IfcRelSpaceBoundary1stLevel::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcRelSpaceBoundary::setInverseCounterparts( ptr_self_entity );
	shared_ptr<IfcRelSpaceBoundary1stLevel> ptr_self = dynamic_pointer_cast<IfcRelSpaceBoundary1stLevel>( ptr_self_entity );
	if( !ptr_self ) { throw BuildingException( "IfcRelSpaceBoundary1stLevel::setInverseCounterparts: type mismatch" ); }
	if( m_ParentBoundary )
	{
		m_ParentBoundary->m_InnerBoundaries_inverse.emplace_back( ptr_self );
	}
}
void IFC4X3::IfcRelSpaceBoundary1stLevel::unlinkFromInverseCounterparts()
{
	IfcRelSpaceBoundary::unlinkFromInverseCounterparts();
	if( m_ParentBoundary )
	{
		std::vector<weak_ptr<IfcRelSpaceBoundary1stLevel> >& InnerBoundaries_inverse = m_ParentBoundary->m_InnerBoundaries_inverse;
		for( auto it_InnerBoundaries_inverse = InnerBoundaries_inverse.begin(); it_InnerBoundaries_inverse != InnerBoundaries_inverse.end(); )
		{
			weak_ptr<IfcRelSpaceBoundary1stLevel> self_candidate_weak = *it_InnerBoundaries_inverse;
			if( self_candidate_weak.expired() )
			{
				++it_InnerBoundaries_inverse;
				continue;
			}
			shared_ptr<IfcRelSpaceBoundary1stLevel> self_candidate( *it_InnerBoundaries_inverse );
			if( self_candidate.get() == this )
			{
				it_InnerBoundaries_inverse= InnerBoundaries_inverse.erase( it_InnerBoundaries_inverse );
			}
			else
			{
				++it_InnerBoundaries_inverse;
			}
		}
	}
}
