/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcIndexedTextureMap.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcPositiveInteger;
	//ENTITY
	class IFCQUERY_EXPORT IfcIndexedTriangleTextureMap : public IfcIndexedTextureMap
	{
	public:
		IfcIndexedTriangleTextureMap() = default;
		IfcIndexedTriangleTextureMap( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 4; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 2133299955; }

		// IfcPresentationItem -----------------------------------------------------------

		// IfcTextureCoordinate -----------------------------------------------------------
		// attributes:
		//  std::vector<shared_ptr<IfcSurfaceTexture> >	m_Maps;

		// IfcIndexedTextureMap -----------------------------------------------------------
		// attributes:
		//  shared_ptr<IfcTessellatedFaceSet>			m_MappedTo;
		//  shared_ptr<IfcTextureVertexList>			m_TexCoords;

		// IfcIndexedTriangleTextureMap -----------------------------------------------------------
		// attributes:
		std::vector<std::vector<shared_ptr<IfcPositiveInteger> > >	m_TexCoordIndex;			//optional
	};
}
