/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcBoolean.h"
#include "ifcpp/Ifc/IfcCartesianTransformationOperator2D.h"
#include "ifcpp/Ifc/IfcIdentifier.h"
#include "ifcpp/Ifc/IfcSurfaceStyleWithTextures.h"
#include "ifcpp/Ifc/IfcSurfaceTexture.h"
#include "ifcpp/Ifc/IfcTextureCoordinate.h"

// ENTITY IfcSurfaceTexture 
IFC4X3::IfcSurfaceTexture::IfcSurfaceTexture( int tag ) { m_tag = tag; }
void IFC4X3::IfcSurfaceTexture::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCSURFACETEXTURE" << "(";
	if( m_RepeatS ) { m_RepeatS->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_RepeatT ) { m_RepeatT->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Mode ) { m_Mode->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_TextureTransform ) { stream << "#" << m_TextureTransform->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Parameter.size() > 0 )
	{
		stream << "(";
		for( size_t ii = 0; ii < m_Parameter.size(); ++ii )
		{
			if( ii > 0 )
			{
				stream << ",";
			}
			const shared_ptr<IfcIdentifier>& type_object = m_Parameter[ii];
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
void IFC4X3::IfcSurfaceTexture::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcSurfaceTexture::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 5 ){ std::stringstream err; err << "Wrong parameter count for entity IfcSurfaceTexture, expecting 5, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_RepeatS = IfcBoolean::createObjectFromSTEP( args[0], map, errorStream );
	m_RepeatT = IfcBoolean::createObjectFromSTEP( args[1], map, errorStream );
	m_Mode = IfcIdentifier::createObjectFromSTEP( args[2], map, errorStream );
	readEntityReference( args[3], m_TextureTransform, map, errorStream );
	readTypeOfStringList( args[4], m_Parameter );
}
void IFC4X3::IfcSurfaceTexture::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcPresentationItem::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "RepeatS", m_RepeatS ) );
	vec_attributes.emplace_back( std::make_pair( "RepeatT", m_RepeatT ) );
	vec_attributes.emplace_back( std::make_pair( "Mode", m_Mode ) );
	vec_attributes.emplace_back( std::make_pair( "TextureTransform", m_TextureTransform ) );
	if( !m_Parameter.empty() )
	{
		shared_ptr<AttributeObjectVector> Parameter_vec_object( new AttributeObjectVector() );
		std::copy( m_Parameter.begin(), m_Parameter.end(), std::back_inserter( Parameter_vec_object->m_vec ) );
		vec_attributes.emplace_back( std::make_pair( "Parameter", Parameter_vec_object ) );
	}
}
void IFC4X3::IfcSurfaceTexture::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcPresentationItem::getAttributesInverse( vec_attributes_inverse );
	if( !m_IsMappedBy_inverse.empty() )
	{
		shared_ptr<AttributeObjectVector> IsMappedBy_inverse_vec_obj( new AttributeObjectVector() );
		for( size_t i=0; i<m_IsMappedBy_inverse.size(); ++i )
		{
			if( !m_IsMappedBy_inverse[i].expired() )
			{
				IsMappedBy_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcTextureCoordinate>( m_IsMappedBy_inverse[i] ) );
			}
		}
		vec_attributes_inverse.emplace_back( std::make_pair( "IsMappedBy_inverse", IsMappedBy_inverse_vec_obj ) );
	}
	if( !m_UsedInStyles_inverse.empty() )
	{
		shared_ptr<AttributeObjectVector> UsedInStyles_inverse_vec_obj( new AttributeObjectVector() );
		for( size_t i=0; i<m_UsedInStyles_inverse.size(); ++i )
		{
			if( !m_UsedInStyles_inverse[i].expired() )
			{
				UsedInStyles_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcSurfaceStyleWithTextures>( m_UsedInStyles_inverse[i] ) );
			}
		}
		vec_attributes_inverse.emplace_back( std::make_pair( "UsedInStyles_inverse", UsedInStyles_inverse_vec_obj ) );
	}
}
void IFC4X3::IfcSurfaceTexture::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcPresentationItem::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcSurfaceTexture::unlinkFromInverseCounterparts()
{
	IfcPresentationItem::unlinkFromInverseCounterparts();
}