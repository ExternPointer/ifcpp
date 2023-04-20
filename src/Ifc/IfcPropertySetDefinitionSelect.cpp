/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <map>
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Ifc/IfcPropertySetDefinitionSet.h"
#include "ifcpp/Ifc/IfcPropertySetDefinitionSelect.h"

// TYPE IfcPropertySetDefinitionSelect = SELECT	(IfcPropertySetDefinition	,IfcPropertySetDefinitionSet);
shared_ptr<IFC4X3::IfcPropertySetDefinitionSelect> IFC4X3::IfcPropertySetDefinitionSelect::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.empty() ){ return shared_ptr<IfcPropertySetDefinitionSelect>(); }
	if( arg.compare("$")==0 )
	{
		return shared_ptr<IfcPropertySetDefinitionSelect>();
	}
	if( arg.compare("*")==0 )
	{
		return shared_ptr<IfcPropertySetDefinitionSelect>();
	}
	shared_ptr<IfcPropertySetDefinitionSelect> result_object;
	readSelectType( arg, result_object, map, errorStream );
	return result_object;
}