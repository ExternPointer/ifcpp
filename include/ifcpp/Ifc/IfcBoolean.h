/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcModulusOfRotationalSubgradeReactionSelect.h"
#include "IfcModulusOfSubgradeReactionSelect.h"
#include "IfcModulusOfTranslationalSubgradeReactionSelect.h"
#include "IfcRotationalStiffnessSelect.h"
#include "IfcSimpleValue.h"
#include "IfcTranslationalStiffnessSelect.h"
#include "IfcWarpingStiffnessSelect.h"

namespace IFC4X3
{
	// TYPE IfcBoolean = BOOLEAN;
	class IFCQUERY_EXPORT IfcBoolean : public IfcModulusOfRotationalSubgradeReactionSelect, public IfcModulusOfSubgradeReactionSelect, public IfcModulusOfTranslationalSubgradeReactionSelect, public IfcRotationalStiffnessSelect, public IfcSimpleValue, public IfcTranslationalStiffnessSelect, public IfcWarpingStiffnessSelect
	{
	public:
		IfcBoolean() = default;
		IfcBoolean( bool value );
		virtual uint32_t classID() const { return 2735952531; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcBoolean> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		bool m_value;
	};
}
