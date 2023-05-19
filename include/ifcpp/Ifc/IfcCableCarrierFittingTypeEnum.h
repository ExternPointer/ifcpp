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
	// TYPE IfcCableCarrierFittingTypeEnum = ENUMERATION OF	(BEND	,CONNECTOR	,CROSS	,JUNCTION	,TEE	,TRANSITION	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcCableCarrierFittingTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcCableCarrierFittingTypeEnumEnum
		{
			ENUM_BEND,
			ENUM_CONNECTOR,
			ENUM_CROSS,
			ENUM_JUNCTION,
			ENUM_TEE,
			ENUM_TRANSITION,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcCableCarrierFittingTypeEnum() = default;
		IfcCableCarrierFittingTypeEnum( IfcCableCarrierFittingTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 54623293; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcCableCarrierFittingTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcCableCarrierFittingTypeEnumEnum m_enum;
	};
}
