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
	// TYPE IfcAlignmentHorizontalSegmentTypeEnum = ENUMERATION OF	(BLOSSCURVE	,CIRCULARARC	,CLOTHOID	,COSINECURVE	,CUBIC	,HELMERTCURVE	,LINE	,SINECURVE	,VIENNESEBEND);
	class IFCQUERY_EXPORT IfcAlignmentHorizontalSegmentTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcAlignmentHorizontalSegmentTypeEnumEnum
		{
			ENUM_BLOSSCURVE,
			ENUM_CIRCULARARC,
			ENUM_CLOTHOID,
			ENUM_COSINECURVE,
			ENUM_CUBIC,
			ENUM_HELMERTCURVE,
			ENUM_LINE,
			ENUM_SINECURVE,
			ENUM_VIENNESEBEND
		};

		IfcAlignmentHorizontalSegmentTypeEnum() = default;
		IfcAlignmentHorizontalSegmentTypeEnum( IfcAlignmentHorizontalSegmentTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 3194911961; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcAlignmentHorizontalSegmentTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcAlignmentHorizontalSegmentTypeEnumEnum m_enum;
	};
}
