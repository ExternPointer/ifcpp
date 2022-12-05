/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/model/GlobalDefines.h"
#include "ifcpp/model/BasicTypes.h"
#include "ifcpp/model/BuildingObject.h"

namespace IFC4X3
{
	// TYPE IfcPropertySetTemplateTypeEnum = ENUMERATION OF	(PSET_MATERIALDRIVEN	,PSET_OCCURRENCEDRIVEN	,PSET_PERFORMANCEDRIVEN	,PSET_PROFILEDRIVEN	,PSET_TYPEDRIVENONLY	,PSET_TYPEDRIVENOVERRIDE	,QTO_OCCURRENCEDRIVEN	,QTO_TYPEDRIVENONLY	,QTO_TYPEDRIVENOVERRIDE	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcPropertySetTemplateTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcPropertySetTemplateTypeEnumEnum
		{
			ENUM_PSET_MATERIALDRIVEN,
			ENUM_PSET_OCCURRENCEDRIVEN,
			ENUM_PSET_PERFORMANCEDRIVEN,
			ENUM_PSET_PROFILEDRIVEN,
			ENUM_PSET_TYPEDRIVENONLY,
			ENUM_PSET_TYPEDRIVENOVERRIDE,
			ENUM_QTO_OCCURRENCEDRIVEN,
			ENUM_QTO_TYPEDRIVENONLY,
			ENUM_QTO_TYPEDRIVENOVERRIDE,
			ENUM_NOTDEFINED
		};

		IfcPropertySetTemplateTypeEnum() = default;
		IfcPropertySetTemplateTypeEnum( IfcPropertySetTemplateTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 606860825; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcPropertySetTemplateTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcPropertySetTemplateTypeEnumEnum m_enum;
	};
}

