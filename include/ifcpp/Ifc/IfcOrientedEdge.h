/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcEdge.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcEdge;
	class IFCQUERY_EXPORT IfcBoolean;
	//ENTITY
	class IFCQUERY_EXPORT IfcOrientedEdge : public IfcEdge
	{
	public:
		IfcOrientedEdge() = default;
		IfcOrientedEdge( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 4; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 1029017970; }

		// IfcRepresentationItem -----------------------------------------------------------
		// inverse attributes:
		//  std::vector<weak_ptr<IfcPresentationLayerAssignment> >	m_LayerAssignment_inverse;
		//  std::vector<weak_ptr<IfcStyledItem> >					m_StyledByItem_inverse;

		// IfcTopologicalRepresentationItem -----------------------------------------------------------

		// IfcEdge -----------------------------------------------------------
		// attributes:
		//  shared_ptr<IfcVertex>									m_EdgeStart;
		//  shared_ptr<IfcVertex>									m_EdgeEnd;

		// IfcOrientedEdge -----------------------------------------------------------
		// attributes:
		shared_ptr<IfcEdge>										m_EdgeElement;
		shared_ptr<IfcBoolean>									m_Orientation;
	};
}
