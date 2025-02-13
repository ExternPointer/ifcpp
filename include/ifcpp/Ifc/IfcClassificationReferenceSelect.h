/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#pragma once
#include <iostream>
#include <sstream>
#include <map>
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"

namespace IFC4X3
{
	// TYPE IfcClassificationReferenceSelect = SELECT	(IfcClassification	,IfcClassificationReference);
	class IFCQUERY_EXPORT IfcClassificationReferenceSelect : virtual public BuildingObject
	{
	public:
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const = 0;
		static shared_ptr<IfcClassificationReferenceSelect> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
	};
}

