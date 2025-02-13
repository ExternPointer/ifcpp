/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcMetricValueSelect.h"
#include "IfcObjectReferenceSelect.h"
#include "IfcResourceObjectSelect.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcLabel;
	class IFCQUERY_EXPORT IfcText;
	class IFCQUERY_EXPORT IfcAppliedValueSelect;
	class IFCQUERY_EXPORT IfcMeasureWithUnit;
	class IFCQUERY_EXPORT IfcDate;
	class IFCQUERY_EXPORT IfcArithmeticOperatorEnum;
	class IFCQUERY_EXPORT IfcAppliedValue;
	class IFCQUERY_EXPORT IfcExternalReferenceRelationship;
	//ENTITY
	class IFCQUERY_EXPORT IfcAppliedValue : virtual public IfcMetricValueSelect, virtual public IfcObjectReferenceSelect, virtual public IfcResourceObjectSelect, public BuildingEntity
	{
	public:
		IfcAppliedValue() = default;
		IfcAppliedValue( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 10; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 411424972; }

		// IfcAppliedValue -----------------------------------------------------------
		// attributes:
		shared_ptr<IfcLabel>										m_Name;						//optional
		shared_ptr<IfcText>											m_Description;				//optional
		shared_ptr<IfcAppliedValueSelect>							m_AppliedValue;				//optional
		shared_ptr<IfcMeasureWithUnit>								m_UnitBasis;				//optional
		shared_ptr<IfcDate>											m_ApplicableDate;			//optional
		shared_ptr<IfcDate>											m_FixedUntilDate;			//optional
		shared_ptr<IfcLabel>										m_Category;					//optional
		shared_ptr<IfcLabel>										m_Condition;				//optional
		shared_ptr<IfcArithmeticOperatorEnum>						m_ArithmeticOperator;		//optional
		std::vector<shared_ptr<IfcAppliedValue> >					m_Components;				//optional
		// inverse attributes:
		std::vector<weak_ptr<IfcExternalReferenceRelationship> >	m_HasExternalReference_inverse;
	};
}
