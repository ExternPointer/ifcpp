/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcDefinitionSelect.h"
#include "ifcpp/Ifc/IfcGloballyUniqueId.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcLibraryInformation.h"
#include "ifcpp/Ifc/IfcLibraryReference.h"
#include "ifcpp/Ifc/IfcLibrarySelect.h"
#include "ifcpp/Ifc/IfcOwnerHistory.h"
#include "ifcpp/Ifc/IfcRelAssociatesLibrary.h"
#include "ifcpp/Ifc/IfcText.h"

// ENTITY IfcRelAssociatesLibrary 
IFC4X3::IfcRelAssociatesLibrary::IfcRelAssociatesLibrary( int tag ) { m_tag = tag; }
void IFC4X3::IfcRelAssociatesLibrary::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCRELASSOCIATESLIBRARY" << "(";
	if( m_GlobalId ) { m_GlobalId->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_OwnerHistory ) { stream << "#" << m_OwnerHistory->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	stream << "(";
	for( size_t ii = 0; ii < m_RelatedObjects.size(); ++ii )
	{
		if( ii > 0 )
		{
			stream << ",";
		}
		const shared_ptr<IfcDefinitionSelect>& type_object = m_RelatedObjects[ii];
		if( type_object )
		{
			type_object->getStepParameter( stream, true );
		}
		else
		{
			stream << "$";
		}
	}
	stream << ")";
	stream << ",";
	if( m_RelatingLibrary ) { m_RelatingLibrary->getStepParameter( stream, true ); } else { stream << "$" ; }
	stream << ");";
}
void IFC4X3::IfcRelAssociatesLibrary::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcRelAssociatesLibrary::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 6 ){ std::stringstream err; err << "Wrong parameter count for entity IfcRelAssociatesLibrary, expecting 6, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_GlobalId = IfcGloballyUniqueId::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReference( args[1], m_OwnerHistory, map, errorStream );
	m_Name = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[3], map, errorStream );
	readSelectList( args[4], m_RelatedObjects, map, errorStream );
	m_RelatingLibrary = IfcLibrarySelect::createObjectFromSTEP( args[5], map, errorStream );
}
void IFC4X3::IfcRelAssociatesLibrary::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcRelAssociates::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "RelatingLibrary", m_RelatingLibrary ) );
}
void IFC4X3::IfcRelAssociatesLibrary::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcRelAssociates::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcRelAssociatesLibrary::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcRelAssociates::setInverseCounterparts( ptr_self_entity );
	shared_ptr<IfcRelAssociatesLibrary> ptr_self = dynamic_pointer_cast<IfcRelAssociatesLibrary>( ptr_self_entity );
	if( !ptr_self ) { throw BuildingException( "IfcRelAssociatesLibrary::setInverseCounterparts: type mismatch" ); }
	shared_ptr<IfcLibraryInformation>  RelatingLibrary_IfcLibraryInformation = dynamic_pointer_cast<IfcLibraryInformation>( m_RelatingLibrary );
	if( RelatingLibrary_IfcLibraryInformation )
	{
		RelatingLibrary_IfcLibraryInformation->m_LibraryInfoForObjects_inverse.emplace_back( ptr_self );
	}
	shared_ptr<IfcLibraryReference>  RelatingLibrary_IfcLibraryReference = dynamic_pointer_cast<IfcLibraryReference>( m_RelatingLibrary );
	if( RelatingLibrary_IfcLibraryReference )
	{
		RelatingLibrary_IfcLibraryReference->m_LibraryRefForObjects_inverse.emplace_back( ptr_self );
	}
}
void IFC4X3::IfcRelAssociatesLibrary::unlinkFromInverseCounterparts()
{
	IfcRelAssociates::unlinkFromInverseCounterparts();
	shared_ptr<IfcLibraryInformation>  RelatingLibrary_IfcLibraryInformation = dynamic_pointer_cast<IfcLibraryInformation>( m_RelatingLibrary );
	if( RelatingLibrary_IfcLibraryInformation )
	{
		std::vector<weak_ptr<IfcRelAssociatesLibrary> >& LibraryInfoForObjects_inverse = RelatingLibrary_IfcLibraryInformation->m_LibraryInfoForObjects_inverse;
		for( auto it_LibraryInfoForObjects_inverse = LibraryInfoForObjects_inverse.begin(); it_LibraryInfoForObjects_inverse != LibraryInfoForObjects_inverse.end(); )
		{
			weak_ptr<IfcRelAssociatesLibrary> self_candidate_weak = *it_LibraryInfoForObjects_inverse;
			if( self_candidate_weak.expired() )
			{
				++it_LibraryInfoForObjects_inverse;
				continue;
			}
			shared_ptr<IfcRelAssociatesLibrary> self_candidate( *it_LibraryInfoForObjects_inverse );
			if( self_candidate.get() == this )
			{
				it_LibraryInfoForObjects_inverse= LibraryInfoForObjects_inverse.erase( it_LibraryInfoForObjects_inverse );
			}
			else
			{
				++it_LibraryInfoForObjects_inverse;
			}
		}
	}
	shared_ptr<IfcLibraryReference>  RelatingLibrary_IfcLibraryReference = dynamic_pointer_cast<IfcLibraryReference>( m_RelatingLibrary );
	if( RelatingLibrary_IfcLibraryReference )
	{
		std::vector<weak_ptr<IfcRelAssociatesLibrary> >& LibraryRefForObjects_inverse = RelatingLibrary_IfcLibraryReference->m_LibraryRefForObjects_inverse;
		for( auto it_LibraryRefForObjects_inverse = LibraryRefForObjects_inverse.begin(); it_LibraryRefForObjects_inverse != LibraryRefForObjects_inverse.end(); )
		{
			weak_ptr<IfcRelAssociatesLibrary> self_candidate_weak = *it_LibraryRefForObjects_inverse;
			if( self_candidate_weak.expired() )
			{
				++it_LibraryRefForObjects_inverse;
				continue;
			}
			shared_ptr<IfcRelAssociatesLibrary> self_candidate( *it_LibraryRefForObjects_inverse );
			if( self_candidate.get() == this )
			{
				it_LibraryRefForObjects_inverse= LibraryRefForObjects_inverse.erase( it_LibraryRefForObjects_inverse );
			}
			else
			{
				++it_LibraryRefForObjects_inverse;
			}
		}
	}
}
