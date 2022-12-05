/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/model/GlobalDefines.h"
#include "ifcpp/model/BasicTypes.h"
#include "ifcpp/model/BuildingObject.h"
#include "IfcSweptSurface.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcAxis1Placement;
	//ENTITY
	class IFCQUERY_EXPORT IfcSurfaceOfRevolution : public IfcSweptSurface
	{ 
	public:
		IfcSurfaceOfRevolution() = default;
		IfcSurfaceOfRevolution( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 3; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 4124788165; }

		// IfcRepresentationItem -----------------------------------------------------------
		// inverse attributes:
		//  std::vector<weak_ptr<IfcPresentationLayerAssignment> >	m_LayerAssignment_inverse;
		//  std::vector<weak_ptr<IfcStyledItem> >					m_StyledByItem_inverse;

		// IfcGeometricRepresentationItem -----------------------------------------------------------

		// IfcSurface -----------------------------------------------------------

		// IfcSweptSurface -----------------------------------------------------------
		// attributes:
		//  shared_ptr<IfcProfileDef>								m_SweptCurve;
		//  shared_ptr<IfcAxis2Placement3D>							m_Position;					//optional

		// IfcSurfaceOfRevolution -----------------------------------------------------------
		// attributes:
		shared_ptr<IfcAxis1Placement>							m_AxisPosition;
	};
}

