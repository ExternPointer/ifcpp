/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcPoint.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcSurface;
	class IFCQUERY_EXPORT IfcParameterValue;
	//ENTITY
	class IFCQUERY_EXPORT IfcPointOnSurface : public IfcPoint
	{
	public:
		IfcPointOnSurface() = default;
		IfcPointOnSurface( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 3; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 1423911732; }

		// IfcRepresentationItem -----------------------------------------------------------
		// inverse attributes:
		//  std::vector<weak_ptr<IfcPresentationLayerAssignment> >	m_LayerAssignment_inverse;
		//  std::vector<weak_ptr<IfcStyledItem> >					m_StyledByItem_inverse;

		// IfcGeometricRepresentationItem -----------------------------------------------------------

		// IfcPoint -----------------------------------------------------------

		// IfcPointOnSurface -----------------------------------------------------------
		// attributes:
		shared_ptr<IfcSurface>									m_BasisSurface;
		shared_ptr<IfcParameterValue>							m_PointParameterU;
		shared_ptr<IfcParameterValue>							m_PointParameterV;
	};
}
