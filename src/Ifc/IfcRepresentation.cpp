/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcPresentationLayerAssignment.h"
#include "ifcpp/Ifc/IfcProductRepresentation.h"
#include "ifcpp/Ifc/IfcRepresentation.h"
#include "ifcpp/Ifc/IfcRepresentationContext.h"
#include "ifcpp/Ifc/IfcRepresentationItem.h"
#include "ifcpp/Ifc/IfcRepresentationMap.h"

// ENTITY IfcRepresentation 
IFC4X3::IfcRepresentation::IfcRepresentation( int tag ) { m_tag = tag; }
void IFC4X3::IfcRepresentation::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCREPRESENTATION" << "(";
	if( m_ContextOfItems ) { stream << "#" << m_ContextOfItems->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_RepresentationIdentifier ) { m_RepresentationIdentifier->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_RepresentationType ) { m_RepresentationType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	writeEntityList( stream, m_Items );
	stream << ");";
}
void IFC4X3::IfcRepresentation::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcRepresentation::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 4 ){ std::stringstream err; err << "Wrong parameter count for entity IfcRepresentation, expecting 4, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	readEntityReference( args[0], m_ContextOfItems, map, errorStream );
	m_RepresentationIdentifier = IfcLabel::createObjectFromSTEP( args[1], map, errorStream );
	m_RepresentationType = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	readEntityReferenceList( args[3], m_Items, map, errorStream );
}
void IFC4X3::IfcRepresentation::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	vec_attributes.emplace_back( std::make_pair( "ContextOfItems", m_ContextOfItems ) );
	vec_attributes.emplace_back( std::make_pair( "RepresentationIdentifier", m_RepresentationIdentifier ) );
	vec_attributes.emplace_back( std::make_pair( "RepresentationType", m_RepresentationType ) );
	shared_ptr<AttributeObjectVector> Items_vec_object( new AttributeObjectVector() );
	std::copy( m_Items.begin(), m_Items.end(), std::back_inserter( Items_vec_object->m_vec ) );
	vec_attributes.emplace_back( std::make_pair( "Items", Items_vec_object ) );
}
void IFC4X3::IfcRepresentation::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	shared_ptr<AttributeObjectVector> RepresentationMap_inverse_vec_obj( new AttributeObjectVector() );
	for( size_t i=0; i<m_RepresentationMap_inverse.size(); ++i )
	{
		if( !m_RepresentationMap_inverse[i].expired() )
		{
			RepresentationMap_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcRepresentationMap>( m_RepresentationMap_inverse[i] ) );
		}
	}
	vec_attributes_inverse.emplace_back( std::make_pair( "RepresentationMap_inverse", RepresentationMap_inverse_vec_obj ) );
	shared_ptr<AttributeObjectVector> LayerAssignments_inverse_vec_obj( new AttributeObjectVector() );
	for( size_t i=0; i<m_LayerAssignments_inverse.size(); ++i )
	{
		if( !m_LayerAssignments_inverse[i].expired() )
		{
			LayerAssignments_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcPresentationLayerAssignment>( m_LayerAssignments_inverse[i] ) );
		}
	}
	vec_attributes_inverse.emplace_back( std::make_pair( "LayerAssignments_inverse", LayerAssignments_inverse_vec_obj ) );
	shared_ptr<AttributeObjectVector> OfProductRepresentation_inverse_vec_obj( new AttributeObjectVector() );
	for( size_t i=0; i<m_OfProductRepresentation_inverse.size(); ++i )
	{
		if( !m_OfProductRepresentation_inverse[i].expired() )
		{
			OfProductRepresentation_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcProductRepresentation>( m_OfProductRepresentation_inverse[i] ) );
		}
	}
	vec_attributes_inverse.emplace_back( std::make_pair( "OfProductRepresentation_inverse", OfProductRepresentation_inverse_vec_obj ) );
}
void IFC4X3::IfcRepresentation::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	shared_ptr<IfcRepresentation> ptr_self = dynamic_pointer_cast<IfcRepresentation>( ptr_self_entity );
	if( !ptr_self ) { throw BuildingException( "IfcRepresentation::setInverseCounterparts: type mismatch" ); }
	if( m_ContextOfItems )
	{
		m_ContextOfItems->m_RepresentationsInContext_inverse.emplace_back( ptr_self );
	}
}
void IFC4X3::IfcRepresentation::unlinkFromInverseCounterparts()
{
	if( m_ContextOfItems )
	{
		std::vector<weak_ptr<IfcRepresentation> >& RepresentationsInContext_inverse = m_ContextOfItems->m_RepresentationsInContext_inverse;
		for( auto it_RepresentationsInContext_inverse = RepresentationsInContext_inverse.begin(); it_RepresentationsInContext_inverse != RepresentationsInContext_inverse.end(); )
		{
			weak_ptr<IfcRepresentation> self_candidate_weak = *it_RepresentationsInContext_inverse;
			if( self_candidate_weak.expired() )
			{
				++it_RepresentationsInContext_inverse;
				continue;
			}
			shared_ptr<IfcRepresentation> self_candidate( *it_RepresentationsInContext_inverse );
			if( self_candidate.get() == this )
			{
				it_RepresentationsInContext_inverse= RepresentationsInContext_inverse.erase( it_RepresentationsInContext_inverse );
			}
			else
			{
				++it_RepresentationsInContext_inverse;
			}
		}
	}
}
