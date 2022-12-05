/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/model/BasicTypes.h"
#include "ifcpp/model/BuildingException.h"
#include "ifcpp/IFC4X3/include/IfcWindowPanelPositionEnum.h"

// TYPE IfcWindowPanelPositionEnum = ENUMERATION OF	(BOTTOM	,LEFT	,MIDDLE	,RIGHT	,TOP	,NOTDEFINED);
void IFC4X3::IfcWindowPanelPositionEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCWINDOWPANELPOSITIONENUM("; }
	switch( m_enum )
	{
		case ENUM_BOTTOM:	stream << ".BOTTOM."; break;
		case ENUM_LEFT:	stream << ".LEFT."; break;
		case ENUM_MIDDLE:	stream << ".MIDDLE."; break;
		case ENUM_RIGHT:	stream << ".RIGHT."; break;
		case ENUM_TOP:	stream << ".TOP."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcWindowPanelPositionEnum> IFC4X3::IfcWindowPanelPositionEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcWindowPanelPositionEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcWindowPanelPositionEnum>(); }
	shared_ptr<IfcWindowPanelPositionEnum> type_object( new IfcWindowPanelPositionEnum() );
	if( std_iequal( arg, ".BOTTOM." ) )
	{
		type_object->m_enum = IfcWindowPanelPositionEnum::ENUM_BOTTOM;
	}
	else if( std_iequal( arg, ".LEFT." ) )
	{
		type_object->m_enum = IfcWindowPanelPositionEnum::ENUM_LEFT;
	}
	else if( std_iequal( arg, ".MIDDLE." ) )
	{
		type_object->m_enum = IfcWindowPanelPositionEnum::ENUM_MIDDLE;
	}
	else if( std_iequal( arg, ".RIGHT." ) )
	{
		type_object->m_enum = IfcWindowPanelPositionEnum::ENUM_RIGHT;
	}
	else if( std_iequal( arg, ".TOP." ) )
	{
		type_object->m_enum = IfcWindowPanelPositionEnum::ENUM_TOP;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcWindowPanelPositionEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
