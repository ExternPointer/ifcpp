/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcWindowPanelOperationEnum.h"

// TYPE IfcWindowPanelOperationEnum = ENUMERATION OF	(BOTTOMHUNG	,FIXEDCASEMENT	,OTHEROPERATION	,PIVOTHORIZONTAL	,PIVOTVERTICAL	,REMOVABLECASEMENT	,SIDEHUNGLEFTHAND	,SIDEHUNGRIGHTHAND	,SLIDINGHORIZONTAL	,SLIDINGVERTICAL	,TILTANDTURNLEFTHAND	,TILTANDTURNRIGHTHAND	,TOPHUNG	,NOTDEFINED);
void IFC4X3::IfcWindowPanelOperationEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCWINDOWPANELOPERATIONENUM("; }
	switch( m_enum )
	{
		case ENUM_BOTTOMHUNG:	stream << ".BOTTOMHUNG."; break;
		case ENUM_FIXEDCASEMENT:	stream << ".FIXEDCASEMENT."; break;
		case ENUM_OTHEROPERATION:	stream << ".OTHEROPERATION."; break;
		case ENUM_PIVOTHORIZONTAL:	stream << ".PIVOTHORIZONTAL."; break;
		case ENUM_PIVOTVERTICAL:	stream << ".PIVOTVERTICAL."; break;
		case ENUM_REMOVABLECASEMENT:	stream << ".REMOVABLECASEMENT."; break;
		case ENUM_SIDEHUNGLEFTHAND:	stream << ".SIDEHUNGLEFTHAND."; break;
		case ENUM_SIDEHUNGRIGHTHAND:	stream << ".SIDEHUNGRIGHTHAND."; break;
		case ENUM_SLIDINGHORIZONTAL:	stream << ".SLIDINGHORIZONTAL."; break;
		case ENUM_SLIDINGVERTICAL:	stream << ".SLIDINGVERTICAL."; break;
		case ENUM_TILTANDTURNLEFTHAND:	stream << ".TILTANDTURNLEFTHAND."; break;
		case ENUM_TILTANDTURNRIGHTHAND:	stream << ".TILTANDTURNRIGHTHAND."; break;
		case ENUM_TOPHUNG:	stream << ".TOPHUNG."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcWindowPanelOperationEnum> IFC4X3::IfcWindowPanelOperationEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcWindowPanelOperationEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcWindowPanelOperationEnum>(); }
	shared_ptr<IfcWindowPanelOperationEnum> type_object( new IfcWindowPanelOperationEnum() );
	if( std_iequal( arg, ".BOTTOMHUNG." ) )
	{
		type_object->m_enum = IfcWindowPanelOperationEnum::ENUM_BOTTOMHUNG;
	}
	else if( std_iequal( arg, ".FIXEDCASEMENT." ) )
	{
		type_object->m_enum = IfcWindowPanelOperationEnum::ENUM_FIXEDCASEMENT;
	}
	else if( std_iequal( arg, ".OTHEROPERATION." ) )
	{
		type_object->m_enum = IfcWindowPanelOperationEnum::ENUM_OTHEROPERATION;
	}
	else if( std_iequal( arg, ".PIVOTHORIZONTAL." ) )
	{
		type_object->m_enum = IfcWindowPanelOperationEnum::ENUM_PIVOTHORIZONTAL;
	}
	else if( std_iequal( arg, ".PIVOTVERTICAL." ) )
	{
		type_object->m_enum = IfcWindowPanelOperationEnum::ENUM_PIVOTVERTICAL;
	}
	else if( std_iequal( arg, ".REMOVABLECASEMENT." ) )
	{
		type_object->m_enum = IfcWindowPanelOperationEnum::ENUM_REMOVABLECASEMENT;
	}
	else if( std_iequal( arg, ".SIDEHUNGLEFTHAND." ) )
	{
		type_object->m_enum = IfcWindowPanelOperationEnum::ENUM_SIDEHUNGLEFTHAND;
	}
	else if( std_iequal( arg, ".SIDEHUNGRIGHTHAND." ) )
	{
		type_object->m_enum = IfcWindowPanelOperationEnum::ENUM_SIDEHUNGRIGHTHAND;
	}
	else if( std_iequal( arg, ".SLIDINGHORIZONTAL." ) )
	{
		type_object->m_enum = IfcWindowPanelOperationEnum::ENUM_SLIDINGHORIZONTAL;
	}
	else if( std_iequal( arg, ".SLIDINGVERTICAL." ) )
	{
		type_object->m_enum = IfcWindowPanelOperationEnum::ENUM_SLIDINGVERTICAL;
	}
	else if( std_iequal( arg, ".TILTANDTURNLEFTHAND." ) )
	{
		type_object->m_enum = IfcWindowPanelOperationEnum::ENUM_TILTANDTURNLEFTHAND;
	}
	else if( std_iequal( arg, ".TILTANDTURNRIGHTHAND." ) )
	{
		type_object->m_enum = IfcWindowPanelOperationEnum::ENUM_TILTANDTURNRIGHTHAND;
	}
	else if( std_iequal( arg, ".TOPHUNG." ) )
	{
		type_object->m_enum = IfcWindowPanelOperationEnum::ENUM_TOPHUNG;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcWindowPanelOperationEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
