/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <map>
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Ifc/IfcDefinitionSelect.h"

// TYPE IfcDefinitionSelect = SELECT	(IfcObjectDefinition	,IfcPropertyDefinition);
shared_ptr<IFC4X3::IfcDefinitionSelect> IFC4X3::IfcDefinitionSelect::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.empty() ){ return shared_ptr<IfcDefinitionSelect>(); }
	if( arg.compare("$")==0 )
	{
		return shared_ptr<IfcDefinitionSelect>();
	}
	if( arg.compare("*")==0 )
	{
		return shared_ptr<IfcDefinitionSelect>();
	}
	shared_ptr<IfcDefinitionSelect> result_object;
	readSelectType( arg, result_object, map, errorStream );
	return result_object;
}
