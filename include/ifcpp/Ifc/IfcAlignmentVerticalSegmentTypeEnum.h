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
	// TYPE IfcAlignmentVerticalSegmentTypeEnum = ENUMERATION OF	(CIRCULARARC	,CLOTHOID	,CONSTANTGRADIENT	,PARABOLICARC);
	class IFCQUERY_EXPORT IfcAlignmentVerticalSegmentTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcAlignmentVerticalSegmentTypeEnumEnum
		{
			ENUM_CIRCULARARC,
			ENUM_CLOTHOID,
			ENUM_CONSTANTGRADIENT,
			ENUM_PARABOLICARC
		};

		IfcAlignmentVerticalSegmentTypeEnum() = default;
		IfcAlignmentVerticalSegmentTypeEnum( IfcAlignmentVerticalSegmentTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 1505327130; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcAlignmentVerticalSegmentTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcAlignmentVerticalSegmentTypeEnumEnum m_enum;
	};
}
