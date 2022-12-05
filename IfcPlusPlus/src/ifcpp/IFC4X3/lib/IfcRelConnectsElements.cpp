/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/model/AttributeObject.h"
#include "ifcpp/model/BuildingException.h"
#include "ifcpp/model/BuildingGuid.h"
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/IFC4X3/include/IfcConnectionGeometry.h"
#include "ifcpp/IFC4X3/include/IfcElement.h"
#include "ifcpp/IFC4X3/include/IfcGloballyUniqueId.h"
#include "ifcpp/IFC4X3/include/IfcLabel.h"
#include "ifcpp/IFC4X3/include/IfcOwnerHistory.h"
#include "ifcpp/IFC4X3/include/IfcRelConnectsElements.h"
#include "ifcpp/IFC4X3/include/IfcText.h"

// ENTITY IfcRelConnectsElements 
IFC4X3::IfcRelConnectsElements::IfcRelConnectsElements( int tag ) { m_tag = tag; }
void IFC4X3::IfcRelConnectsElements::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCRELCONNECTSELEMENTS" << "(";
	if( m_GlobalId ) { m_GlobalId->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_OwnerHistory ) { stream << "#" << m_OwnerHistory->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ConnectionGeometry ) { stream << "#" << m_ConnectionGeometry->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_RelatingElement ) { stream << "#" << m_RelatingElement->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_RelatedElement ) { stream << "#" << m_RelatedElement->m_tag; } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcRelConnectsElements::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcRelConnectsElements::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 7 ){ std::stringstream err; err << "Wrong parameter count for entity IfcRelConnectsElements, expecting 7, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_GlobalId = IfcGloballyUniqueId::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReference( args[1], m_OwnerHistory, map, errorStream );
	m_Name = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[3], map, errorStream );
	readEntityReference( args[4], m_ConnectionGeometry, map, errorStream );
	readEntityReference( args[5], m_RelatingElement, map, errorStream );
	readEntityReference( args[6], m_RelatedElement, map, errorStream );
}
void IFC4X3::IfcRelConnectsElements::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcRelConnects::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "ConnectionGeometry", m_ConnectionGeometry ) );
	vec_attributes.emplace_back( std::make_pair( "RelatingElement", m_RelatingElement ) );
	vec_attributes.emplace_back( std::make_pair( "RelatedElement", m_RelatedElement ) );
}
void IFC4X3::IfcRelConnectsElements::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcRelConnects::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcRelConnectsElements::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcRelConnects::setInverseCounterparts( ptr_self_entity );
	shared_ptr<IfcRelConnectsElements> ptr_self = dynamic_pointer_cast<IfcRelConnectsElements>( ptr_self_entity );
	if( !ptr_self ) { throw BuildingException( "IfcRelConnectsElements::setInverseCounterparts: type mismatch" ); }
	if( m_RelatedElement )
	{
		m_RelatedElement->m_ConnectedFrom_inverse.emplace_back( ptr_self );
	}
	if( m_RelatingElement )
	{
		m_RelatingElement->m_ConnectedTo_inverse.emplace_back( ptr_self );
	}
}
void IFC4X3::IfcRelConnectsElements::unlinkFromInverseCounterparts()
{
	IfcRelConnects::unlinkFromInverseCounterparts();
	if( m_RelatedElement )
	{
		std::vector<weak_ptr<IfcRelConnectsElements> >& ConnectedFrom_inverse = m_RelatedElement->m_ConnectedFrom_inverse;
		for( auto it_ConnectedFrom_inverse = ConnectedFrom_inverse.begin(); it_ConnectedFrom_inverse != ConnectedFrom_inverse.end(); )
		{
			weak_ptr<IfcRelConnectsElements> self_candidate_weak = *it_ConnectedFrom_inverse;
			if( self_candidate_weak.expired() )
			{
				++it_ConnectedFrom_inverse;
				continue;
			}
			shared_ptr<IfcRelConnectsElements> self_candidate( *it_ConnectedFrom_inverse );
			if( self_candidate.get() == this )
			{
				it_ConnectedFrom_inverse= ConnectedFrom_inverse.erase( it_ConnectedFrom_inverse );
			}
			else
			{
				++it_ConnectedFrom_inverse;
			}
		}
	}
	if( m_RelatingElement )
	{
		std::vector<weak_ptr<IfcRelConnectsElements> >& ConnectedTo_inverse = m_RelatingElement->m_ConnectedTo_inverse;
		for( auto it_ConnectedTo_inverse = ConnectedTo_inverse.begin(); it_ConnectedTo_inverse != ConnectedTo_inverse.end(); )
		{
			weak_ptr<IfcRelConnectsElements> self_candidate_weak = *it_ConnectedTo_inverse;
			if( self_candidate_weak.expired() )
			{
				++it_ConnectedTo_inverse;
				continue;
			}
			shared_ptr<IfcRelConnectsElements> self_candidate( *it_ConnectedTo_inverse );
			if( self_candidate.get() == this )
			{
				it_ConnectedTo_inverse= ConnectedTo_inverse.erase( it_ConnectedTo_inverse );
			}
			else
			{
				++it_ConnectedTo_inverse;
			}
		}
	}
}
