/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcFillStyleSelect.h"
#include "IfcGeometricRepresentationItem.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcCurveStyle;
	class IFCQUERY_EXPORT IfcHatchLineDistanceSelect;
	class IFCQUERY_EXPORT IfcCartesianPoint;
	class IFCQUERY_EXPORT IfcPlaneAngleMeasure;
	//ENTITY
	class IFCQUERY_EXPORT IfcFillAreaStyleHatching : virtual public IfcFillStyleSelect, public IfcGeometricRepresentationItem
	{
	public:
		IfcFillAreaStyleHatching() = default;
		IfcFillAreaStyleHatching( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 5; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 374418227; }

		// IfcRepresentationItem -----------------------------------------------------------
		// inverse attributes:
		//  std::vector<weak_ptr<IfcPresentationLayerAssignment> >	m_LayerAssignment_inverse;
		//  std::vector<weak_ptr<IfcStyledItem> >					m_StyledByItem_inverse;

		// IfcGeometricRepresentationItem -----------------------------------------------------------

		// IfcFillAreaStyleHatching -----------------------------------------------------------
		// attributes:
		shared_ptr<IfcCurveStyle>								m_HatchLineAppearance;
		shared_ptr<IfcHatchLineDistanceSelect>					m_StartOfNextHatchLine;
		shared_ptr<IfcCartesianPoint>							m_PointOfReferenceHatchLine;	//optional
		shared_ptr<IfcCartesianPoint>							m_PatternStart;				//optional
		shared_ptr<IfcPlaneAngleMeasure>						m_HatchLineAngle;
	};
}
