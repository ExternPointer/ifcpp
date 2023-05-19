/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcIndexedPolygonalFace.h"
#include "ifcpp/Ifc/IfcPolygonalFaceSet.h"
#include "ifcpp/Ifc/IfcPositiveInteger.h"
#include "ifcpp/Ifc/IfcPresentationLayerAssignment.h"
#include "ifcpp/Ifc/IfcStyledItem.h"
#include "ifcpp/Ifc/IfcTextureCoordinateIndices.h"

// ENTITY IfcIndexedPolygonalFace 
IFC4X3::IfcIndexedPolygonalFace::IfcIndexedPolygonalFace( int tag ) { m_tag = tag; }
void IFC4X3::IfcIndexedPolygonalFace::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCINDEXEDPOLYGONALFACE" << "(";
	stream << "(";
	for( size_t ii = 0; ii < m_CoordIndex.size(); ++ii )
	{
		if( ii > 0 )
		{
			stream << ",";
		}
		const shared_ptr<IfcPositiveInteger>& type_object = m_CoordIndex[ii];
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
	stream << ");";
}
void IFC4X3::IfcIndexedPolygonalFace::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcIndexedPolygonalFace::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 1 ){ std::stringstream err; err << "Wrong parameter count for entity IfcIndexedPolygonalFace, expecting 1, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	readTypeOfIntegerList( args[0], m_CoordIndex );
}
void IFC4X3::IfcIndexedPolygonalFace::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcTessellatedItem::getAttributes( vec_attributes );
	shared_ptr<AttributeObjectVector> CoordIndex_vec_object( new AttributeObjectVector() );
	std::copy( m_CoordIndex.begin(), m_CoordIndex.end(), std::back_inserter( CoordIndex_vec_object->m_vec ) );
	vec_attributes.emplace_back( std::make_pair( "CoordIndex", CoordIndex_vec_object ) );
}
void IFC4X3::IfcIndexedPolygonalFace::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcTessellatedItem::getAttributesInverse( vec_attributes_inverse );
	shared_ptr<AttributeObjectVector> ToFaceSet_inverse_vec_obj( new AttributeObjectVector() );
	for( size_t i=0; i<m_ToFaceSet_inverse.size(); ++i )
	{
		if( !m_ToFaceSet_inverse[i].expired() )
		{
			ToFaceSet_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcPolygonalFaceSet>( m_ToFaceSet_inverse[i] ) );
		}
	}
	vec_attributes_inverse.emplace_back( std::make_pair( "ToFaceSet_inverse", ToFaceSet_inverse_vec_obj ) );
	shared_ptr<AttributeObjectVector> HasTexCoords_inverse_vec_obj( new AttributeObjectVector() );
	for( size_t i=0; i<m_HasTexCoords_inverse.size(); ++i )
	{
		if( !m_HasTexCoords_inverse[i].expired() )
		{
			HasTexCoords_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcTextureCoordinateIndices>( m_HasTexCoords_inverse[i] ) );
		}
	}
	vec_attributes_inverse.emplace_back( std::make_pair( "HasTexCoords_inverse", HasTexCoords_inverse_vec_obj ) );
}
void IFC4X3::IfcIndexedPolygonalFace::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcTessellatedItem::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcIndexedPolygonalFace::unlinkFromInverseCounterparts()
{
	IfcTessellatedItem::unlinkFromInverseCounterparts();
}
