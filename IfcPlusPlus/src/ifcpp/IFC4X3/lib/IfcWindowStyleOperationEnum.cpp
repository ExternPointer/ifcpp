/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/model/BasicTypes.h"
#include "ifcpp/model/BuildingException.h"
#include "ifcpp/IFC4X3/include/IfcWindowStyleOperationEnum.h"

// TYPE IfcWindowStyleOperationEnum = ENUMERATION OF	(DOUBLE_PANEL_HORIZONTAL	,DOUBLE_PANEL_VERTICAL	,SINGLE_PANEL	,TRIPLE_PANEL_BOTTOM	,TRIPLE_PANEL_HORIZONTAL	,TRIPLE_PANEL_LEFT	,TRIPLE_PANEL_RIGHT	,TRIPLE_PANEL_TOP	,TRIPLE_PANEL_VERTICAL	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcWindowStyleOperationEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCWINDOWSTYLEOPERATIONENUM("; }
	switch( m_enum )
	{
		case ENUM_DOUBLE_PANEL_HORIZONTAL:	stream << ".DOUBLE_PANEL_HORIZONTAL."; break;
		case ENUM_DOUBLE_PANEL_VERTICAL:	stream << ".DOUBLE_PANEL_VERTICAL."; break;
		case ENUM_SINGLE_PANEL:	stream << ".SINGLE_PANEL."; break;
		case ENUM_TRIPLE_PANEL_BOTTOM:	stream << ".TRIPLE_PANEL_BOTTOM."; break;
		case ENUM_TRIPLE_PANEL_HORIZONTAL:	stream << ".TRIPLE_PANEL_HORIZONTAL."; break;
		case ENUM_TRIPLE_PANEL_LEFT:	stream << ".TRIPLE_PANEL_LEFT."; break;
		case ENUM_TRIPLE_PANEL_RIGHT:	stream << ".TRIPLE_PANEL_RIGHT."; break;
		case ENUM_TRIPLE_PANEL_TOP:	stream << ".TRIPLE_PANEL_TOP."; break;
		case ENUM_TRIPLE_PANEL_VERTICAL:	stream << ".TRIPLE_PANEL_VERTICAL."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcWindowStyleOperationEnum> IFC4X3::IfcWindowStyleOperationEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcWindowStyleOperationEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcWindowStyleOperationEnum>(); }
	shared_ptr<IfcWindowStyleOperationEnum> type_object( new IfcWindowStyleOperationEnum() );
	if( std_iequal( arg, ".DOUBLE_PANEL_HORIZONTAL." ) )
	{
		type_object->m_enum = IfcWindowStyleOperationEnum::ENUM_DOUBLE_PANEL_HORIZONTAL;
	}
	else if( std_iequal( arg, ".DOUBLE_PANEL_VERTICAL." ) )
	{
		type_object->m_enum = IfcWindowStyleOperationEnum::ENUM_DOUBLE_PANEL_VERTICAL;
	}
	else if( std_iequal( arg, ".SINGLE_PANEL." ) )
	{
		type_object->m_enum = IfcWindowStyleOperationEnum::ENUM_SINGLE_PANEL;
	}
	else if( std_iequal( arg, ".TRIPLE_PANEL_BOTTOM." ) )
	{
		type_object->m_enum = IfcWindowStyleOperationEnum::ENUM_TRIPLE_PANEL_BOTTOM;
	}
	else if( std_iequal( arg, ".TRIPLE_PANEL_HORIZONTAL." ) )
	{
		type_object->m_enum = IfcWindowStyleOperationEnum::ENUM_TRIPLE_PANEL_HORIZONTAL;
	}
	else if( std_iequal( arg, ".TRIPLE_PANEL_LEFT." ) )
	{
		type_object->m_enum = IfcWindowStyleOperationEnum::ENUM_TRIPLE_PANEL_LEFT;
	}
	else if( std_iequal( arg, ".TRIPLE_PANEL_RIGHT." ) )
	{
		type_object->m_enum = IfcWindowStyleOperationEnum::ENUM_TRIPLE_PANEL_RIGHT;
	}
	else if( std_iequal( arg, ".TRIPLE_PANEL_TOP." ) )
	{
		type_object->m_enum = IfcWindowStyleOperationEnum::ENUM_TRIPLE_PANEL_TOP;
	}
	else if( std_iequal( arg, ".TRIPLE_PANEL_VERTICAL." ) )
	{
		type_object->m_enum = IfcWindowStyleOperationEnum::ENUM_TRIPLE_PANEL_VERTICAL;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcWindowStyleOperationEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcWindowStyleOperationEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
