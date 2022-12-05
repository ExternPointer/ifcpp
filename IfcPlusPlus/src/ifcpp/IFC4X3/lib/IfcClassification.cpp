/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/model/AttributeObject.h"
#include "ifcpp/model/BuildingException.h"
#include "ifcpp/model/BuildingGuid.h"
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/IFC4X3/include/IfcClassification.h"
#include "ifcpp/IFC4X3/include/IfcClassificationReference.h"
#include "ifcpp/IFC4X3/include/IfcDate.h"
#include "ifcpp/IFC4X3/include/IfcIdentifier.h"
#include "ifcpp/IFC4X3/include/IfcLabel.h"
#include "ifcpp/IFC4X3/include/IfcRelAssociatesClassification.h"
#include "ifcpp/IFC4X3/include/IfcText.h"
#include "ifcpp/IFC4X3/include/IfcURIReference.h"

// ENTITY IfcClassification 
IFC4X3::IfcClassification::IfcClassification( int tag ) { m_tag = tag; }
void IFC4X3::IfcClassification::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCCLASSIFICATION" << "(";
	if( m_Source ) { m_Source->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Edition ) { m_Edition->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_EditionDate ) { m_EditionDate->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Specification ) { m_Specification->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ReferenceTokens.size() > 0 )
	{
		stream << "(";
		for( size_t ii = 0; ii < m_ReferenceTokens.size(); ++ii )
		{
			if( ii > 0 )
			{
				stream << ",";
			}
			const shared_ptr<IfcIdentifier>& type_object = m_ReferenceTokens[ii];
			if( type_object )
			{
				type_object->getStepParameter( stream, false );
			}
			else
			{
				stream << "$";
			}
		}
		stream << ")";
	}
	else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcClassification::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcClassification::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 7 ){ std::stringstream err; err << "Wrong parameter count for entity IfcClassification, expecting 7, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Source = IfcLabel::createObjectFromSTEP( args[0], map, errorStream );
	m_Edition = IfcLabel::createObjectFromSTEP( args[1], map, errorStream );
	m_EditionDate = IfcDate::createObjectFromSTEP( args[2], map, errorStream );
	m_Name = IfcLabel::createObjectFromSTEP( args[3], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[4], map, errorStream );
	m_Specification = IfcURIReference::createObjectFromSTEP( args[5], map, errorStream );
	readTypeOfStringList( args[6], m_ReferenceTokens );
}
void IFC4X3::IfcClassification::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcExternalInformation::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "Source", m_Source ) );
	vec_attributes.emplace_back( std::make_pair( "Edition", m_Edition ) );
	vec_attributes.emplace_back( std::make_pair( "EditionDate", m_EditionDate ) );
	vec_attributes.emplace_back( std::make_pair( "Name", m_Name ) );
	vec_attributes.emplace_back( std::make_pair( "Description", m_Description ) );
	vec_attributes.emplace_back( std::make_pair( "Specification", m_Specification ) );
	if( !m_ReferenceTokens.empty() )
	{
		shared_ptr<AttributeObjectVector> ReferenceTokens_vec_object( new AttributeObjectVector() );
		std::copy( m_ReferenceTokens.begin(), m_ReferenceTokens.end(), std::back_inserter( ReferenceTokens_vec_object->m_vec ) );
		vec_attributes.emplace_back( std::make_pair( "ReferenceTokens", ReferenceTokens_vec_object ) );
	}
}
void IFC4X3::IfcClassification::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcExternalInformation::getAttributesInverse( vec_attributes_inverse );
	if( !m_ClassificationForObjects_inverse.empty() )
	{
		shared_ptr<AttributeObjectVector> ClassificationForObjects_inverse_vec_obj( new AttributeObjectVector() );
		for( size_t i=0; i<m_ClassificationForObjects_inverse.size(); ++i )
		{
			if( !m_ClassificationForObjects_inverse[i].expired() )
			{
				ClassificationForObjects_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcRelAssociatesClassification>( m_ClassificationForObjects_inverse[i] ) );
			}
		}
		vec_attributes_inverse.emplace_back( std::make_pair( "ClassificationForObjects_inverse", ClassificationForObjects_inverse_vec_obj ) );
	}
	if( !m_HasReferences_inverse.empty() )
	{
		shared_ptr<AttributeObjectVector> HasReferences_inverse_vec_obj( new AttributeObjectVector() );
		for( size_t i=0; i<m_HasReferences_inverse.size(); ++i )
		{
			if( !m_HasReferences_inverse[i].expired() )
			{
				HasReferences_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcClassificationReference>( m_HasReferences_inverse[i] ) );
			}
		}
		vec_attributes_inverse.emplace_back( std::make_pair( "HasReferences_inverse", HasReferences_inverse_vec_obj ) );
	}
}
void IFC4X3::IfcClassification::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcExternalInformation::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcClassification::unlinkFromInverseCounterparts()
{
	IfcExternalInformation::unlinkFromInverseCounterparts();
}
