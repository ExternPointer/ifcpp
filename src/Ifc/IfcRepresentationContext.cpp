/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcRepresentation.h"
#include "ifcpp/Ifc/IfcRepresentationContext.h"

// ENTITY IfcRepresentationContext 
IFC4X3::IfcRepresentationContext::IfcRepresentationContext( int tag ) { m_tag = tag; }
void IFC4X3::IfcRepresentationContext::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCREPRESENTATIONCONTEXT" << "(";
	if( m_ContextIdentifier ) { m_ContextIdentifier->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ContextType ) { m_ContextType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcRepresentationContext::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcRepresentationContext::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 2 ){ std::stringstream err; err << "Wrong parameter count for entity IfcRepresentationContext, expecting 2, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_ContextIdentifier = IfcLabel::createObjectFromSTEP( args[0], map, errorStream );
	m_ContextType = IfcLabel::createObjectFromSTEP( args[1], map, errorStream );
}
void IFC4X3::IfcRepresentationContext::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	vec_attributes.emplace_back( std::make_pair( "ContextIdentifier", m_ContextIdentifier ) );
	vec_attributes.emplace_back( std::make_pair( "ContextType", m_ContextType ) );
}
void IFC4X3::IfcRepresentationContext::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	shared_ptr<AttributeObjectVector> RepresentationsInContext_inverse_vec_obj( new AttributeObjectVector() );
	for( size_t i=0; i<m_RepresentationsInContext_inverse.size(); ++i )
	{
		if( !m_RepresentationsInContext_inverse[i].expired() )
		{
			RepresentationsInContext_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcRepresentation>( m_RepresentationsInContext_inverse[i] ) );
		}
	}
	vec_attributes_inverse.emplace_back( std::make_pair( "RepresentationsInContext_inverse", RepresentationsInContext_inverse_vec_obj ) );
}
void IFC4X3::IfcRepresentationContext::setInverseCounterparts( shared_ptr<BuildingEntity> )
{
}
void IFC4X3::IfcRepresentationContext::unlinkFromInverseCounterparts()
{
}
