/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcBuildingElementPartTypeEnum.h"

// TYPE IfcBuildingElementPartTypeEnum = ENUMERATION OF	(APRON	,ARMOURUNIT	,INSULATION	,PRECASTPANEL	,SAFETYCAGE	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcBuildingElementPartTypeEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCBUILDINGELEMENTPARTTYPEENUM("; }
	switch( m_enum )
	{
		case ENUM_APRON:	stream << ".APRON."; break;
		case ENUM_ARMOURUNIT:	stream << ".ARMOURUNIT."; break;
		case ENUM_INSULATION:	stream << ".INSULATION."; break;
		case ENUM_PRECASTPANEL:	stream << ".PRECASTPANEL."; break;
		case ENUM_SAFETYCAGE:	stream << ".SAFETYCAGE."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcBuildingElementPartTypeEnum> IFC4X3::IfcBuildingElementPartTypeEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcBuildingElementPartTypeEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcBuildingElementPartTypeEnum>(); }
	shared_ptr<IfcBuildingElementPartTypeEnum> type_object( new IfcBuildingElementPartTypeEnum() );
	if( std_iequal( arg, ".APRON." ) )
	{
		type_object->m_enum = IfcBuildingElementPartTypeEnum::ENUM_APRON;
	}
	else if( std_iequal( arg, ".ARMOURUNIT." ) )
	{
		type_object->m_enum = IfcBuildingElementPartTypeEnum::ENUM_ARMOURUNIT;
	}
	else if( std_iequal( arg, ".INSULATION." ) )
	{
		type_object->m_enum = IfcBuildingElementPartTypeEnum::ENUM_INSULATION;
	}
	else if( std_iequal( arg, ".PRECASTPANEL." ) )
	{
		type_object->m_enum = IfcBuildingElementPartTypeEnum::ENUM_PRECASTPANEL;
	}
	else if( std_iequal( arg, ".SAFETYCAGE." ) )
	{
		type_object->m_enum = IfcBuildingElementPartTypeEnum::ENUM_SAFETYCAGE;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcBuildingElementPartTypeEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcBuildingElementPartTypeEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
