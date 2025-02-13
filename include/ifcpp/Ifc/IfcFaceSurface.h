/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcSurfaceOrFaceSurface.h"
#include "IfcFace.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcSurface;
	class IFCQUERY_EXPORT IfcBoolean;
	//ENTITY
	class IFCQUERY_EXPORT IfcFaceSurface : virtual public IfcSurfaceOrFaceSurface, public IfcFace
	{
	public:
		IfcFaceSurface() = default;
		IfcFaceSurface( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 3; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 3008276851; }

		// IfcRepresentationItem -----------------------------------------------------------
		// inverse attributes:
		//  std::vector<weak_ptr<IfcPresentationLayerAssignment> >	m_LayerAssignment_inverse;
		//  std::vector<weak_ptr<IfcStyledItem> >					m_StyledByItem_inverse;

		// IfcTopologicalRepresentationItem -----------------------------------------------------------

		// IfcFace -----------------------------------------------------------
		// attributes:
		//  std::vector<shared_ptr<IfcFaceBound> >					m_Bounds;
		// inverse attributes:
		//  std::vector<weak_ptr<IfcTextureMap> >					m_HasTextureMaps_inverse;

		// IfcFaceSurface -----------------------------------------------------------
		// attributes:
		shared_ptr<IfcSurface>									m_FaceSurface;
		shared_ptr<IfcBoolean>									m_SameSense;
	};
}
