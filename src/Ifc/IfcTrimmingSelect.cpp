/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <map>
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Ifc/IfcParameterValue.h"
#include "ifcpp/Ifc/IfcTrimmingSelect.h"

// TYPE IfcTrimmingSelect = SELECT	(IfcCartesianPoint	,IfcParameterValue);
shared_ptr<IFC4X3::IfcTrimmingSelect> IFC4X3::IfcTrimmingSelect::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.empty() ){ return shared_ptr<IfcTrimmingSelect>(); }
	if( arg.compare("$")==0 )
	{
		return shared_ptr<IfcTrimmingSelect>();
	}
	if( arg.compare("*")==0 )
	{
		return shared_ptr<IfcTrimmingSelect>();
	}
	shared_ptr<IfcTrimmingSelect> result_object;
	readSelectType( arg, result_object, map, errorStream );
	return result_object;
}