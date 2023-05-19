/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcTextureCoordinate.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcTessellatedFaceSet;
	class IFCQUERY_EXPORT IfcTextureVertexList;
	//ENTITY
	class IFCQUERY_EXPORT IfcIndexedTextureMap : public IfcTextureCoordinate
	{
	public:
		IfcIndexedTextureMap() = default;
		IfcIndexedTextureMap( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 3; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 1437953363; }

		// IfcPresentationItem -----------------------------------------------------------

		// IfcTextureCoordinate -----------------------------------------------------------
		// attributes:
		//  std::vector<shared_ptr<IfcSurfaceTexture> >	m_Maps;

		// IfcIndexedTextureMap -----------------------------------------------------------
		// attributes:
		shared_ptr<IfcTessellatedFaceSet>			m_MappedTo;
		shared_ptr<IfcTextureVertexList>			m_TexCoords;
	};
}
