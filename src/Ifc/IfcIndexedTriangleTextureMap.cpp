/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcIndexedTriangleTextureMap.h"
#include "ifcpp/Ifc/IfcPositiveInteger.h"
#include "ifcpp/Ifc/IfcSurfaceTexture.h"
#include "ifcpp/Ifc/IfcTessellatedFaceSet.h"
#include "ifcpp/Ifc/IfcTextureVertexList.h"

// ENTITY IfcIndexedTriangleTextureMap 
IFC4X3::IfcIndexedTriangleTextureMap::IfcIndexedTriangleTextureMap( int tag ) { m_tag = tag; }
void IFC4X3::IfcIndexedTriangleTextureMap::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCINDEXEDTRIANGLETEXTUREMAP" << "(";
	writeEntityList( stream, m_Maps );
	stream << ",";
	if( m_MappedTo ) { stream << "#" << m_MappedTo->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_TexCoords ) { stream << "#" << m_TexCoords->m_tag; } else { stream << "$"; }
	stream << ",";
	writeTypeOfIntList2D( stream, m_TexCoordIndex, true );
	stream << ");";
}
void IFC4X3::IfcIndexedTriangleTextureMap::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcIndexedTriangleTextureMap::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 4 ){ std::stringstream err; err << "Wrong parameter count for entity IfcIndexedTriangleTextureMap, expecting 4, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	readEntityReferenceList( args[0], m_Maps, map, errorStream );
	readEntityReference( args[1], m_MappedTo, map, errorStream );
	readEntityReference( args[2], m_TexCoords, map, errorStream );
	readTypeOfIntegerList2D( args[3], m_TexCoordIndex );
}
void IFC4X3::IfcIndexedTriangleTextureMap::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcIndexedTextureMap::getAttributes( vec_attributes );
	shared_ptr<AttributeObjectVector> TexCoordIndex_vector( new AttributeObjectVector() );
	vec_attributes.emplace_back( std::make_pair( "TexCoordIndex", TexCoordIndex_vector ) );
	for( size_t ii=0; ii<m_TexCoordIndex.size(); ++ii )
	{
		const std::vector<shared_ptr<IfcPositiveInteger> >& vec_ii = m_TexCoordIndex[ii];
		shared_ptr<AttributeObjectVector> inner_vector( new AttributeObjectVector() );
		TexCoordIndex_vector->m_vec.push_back( inner_vector );
		std::copy(vec_ii.begin(), vec_ii.end(), std::back_inserter(inner_vector->m_vec));
	}
}
void IFC4X3::IfcIndexedTriangleTextureMap::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcIndexedTextureMap::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcIndexedTriangleTextureMap::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcIndexedTextureMap::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcIndexedTriangleTextureMap::unlinkFromInverseCounterparts()
{
	IfcIndexedTextureMap::unlinkFromInverseCounterparts();
}
