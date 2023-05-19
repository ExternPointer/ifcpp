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
	// TYPE IfcSimplePropertyTemplateTypeEnum = ENUMERATION OF	(P_BOUNDEDVALUE	,P_ENUMERATEDVALUE	,P_LISTVALUE	,P_REFERENCEVALUE	,P_SINGLEVALUE	,P_TABLEVALUE	,Q_AREA	,Q_COUNT	,Q_LENGTH	,Q_NUMBER	,Q_TIME	,Q_VOLUME	,Q_WEIGHT);
	class IFCQUERY_EXPORT IfcSimplePropertyTemplateTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcSimplePropertyTemplateTypeEnumEnum
		{
			ENUM_P_BOUNDEDVALUE,
			ENUM_P_ENUMERATEDVALUE,
			ENUM_P_LISTVALUE,
			ENUM_P_REFERENCEVALUE,
			ENUM_P_SINGLEVALUE,
			ENUM_P_TABLEVALUE,
			ENUM_Q_AREA,
			ENUM_Q_COUNT,
			ENUM_Q_LENGTH,
			ENUM_Q_NUMBER,
			ENUM_Q_TIME,
			ENUM_Q_VOLUME,
			ENUM_Q_WEIGHT
		};

		IfcSimplePropertyTemplateTypeEnum() = default;
		IfcSimplePropertyTemplateTypeEnum( IfcSimplePropertyTemplateTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 3841475323; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcSimplePropertyTemplateTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcSimplePropertyTemplateTypeEnumEnum m_enum;
	};
}
