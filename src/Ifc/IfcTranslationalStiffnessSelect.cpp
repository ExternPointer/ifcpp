/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <map>
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Ifc/IfcBoolean.h"
#include "ifcpp/Ifc/IfcLinearStiffnessMeasure.h"
#include "ifcpp/Ifc/IfcTranslationalStiffnessSelect.h"

// TYPE IfcTranslationalStiffnessSelect = SELECT	(IfcBoolean	,IfcLinearStiffnessMeasure);
shared_ptr<IFC4X3::IfcTranslationalStiffnessSelect> IFC4X3::IfcTranslationalStiffnessSelect::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.empty() ){ return shared_ptr<IfcTranslationalStiffnessSelect>(); }
	if( arg.compare("$")==0 )
	{
		return shared_ptr<IfcTranslationalStiffnessSelect>();
	}
	if( arg.compare("*")==0 )
	{
		return shared_ptr<IfcTranslationalStiffnessSelect>();
	}
	shared_ptr<IfcTranslationalStiffnessSelect> result_object;
	readSelectType( arg, result_object, map, errorStream );
	return result_object;
}