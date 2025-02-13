/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcLabel;
	class IFCQUERY_EXPORT IfcCurve;
	class IFCQUERY_EXPORT IfcBoolean;
	class IFCQUERY_EXPORT IfcGrid;
	class IFCQUERY_EXPORT IfcVirtualGridIntersection;
	//ENTITY
	class IFCQUERY_EXPORT IfcGridAxis : public BuildingEntity
	{
	public:
		IfcGridAxis() = default;
		IfcGridAxis( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 3; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 852622518; }

		// IfcGridAxis -----------------------------------------------------------
		// attributes:
		shared_ptr<IfcLabel>								m_AxisTag;					//optional
		shared_ptr<IfcCurve>								m_AxisCurve;
		shared_ptr<IfcBoolean>								m_SameSense;
		// inverse attributes:
		std::vector<weak_ptr<IfcGrid> >						m_PartOfW_inverse;
		std::vector<weak_ptr<IfcGrid> >						m_PartOfV_inverse;
		std::vector<weak_ptr<IfcGrid> >						m_PartOfU_inverse;
		std::vector<weak_ptr<IfcVirtualGridIntersection> >	m_HasIntersections_inverse;
	};
}
