/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcCostScheduleTypeEnum.h"

// TYPE IfcCostScheduleTypeEnum = ENUMERATION OF	(BUDGET	,COSTPLAN	,ESTIMATE	,PRICEDBILLOFQUANTITIES	,SCHEDULEOFRATES	,TENDER	,UNPRICEDBILLOFQUANTITIES	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcCostScheduleTypeEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCCOSTSCHEDULETYPEENUM("; }
	switch( m_enum )
	{
		case ENUM_BUDGET:	stream << ".BUDGET."; break;
		case ENUM_COSTPLAN:	stream << ".COSTPLAN."; break;
		case ENUM_ESTIMATE:	stream << ".ESTIMATE."; break;
		case ENUM_PRICEDBILLOFQUANTITIES:	stream << ".PRICEDBILLOFQUANTITIES."; break;
		case ENUM_SCHEDULEOFRATES:	stream << ".SCHEDULEOFRATES."; break;
		case ENUM_TENDER:	stream << ".TENDER."; break;
		case ENUM_UNPRICEDBILLOFQUANTITIES:	stream << ".UNPRICEDBILLOFQUANTITIES."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcCostScheduleTypeEnum> IFC4X3::IfcCostScheduleTypeEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcCostScheduleTypeEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcCostScheduleTypeEnum>(); }
	shared_ptr<IfcCostScheduleTypeEnum> type_object( new IfcCostScheduleTypeEnum() );
	if( std_iequal( arg, ".BUDGET." ) )
	{
		type_object->m_enum = IfcCostScheduleTypeEnum::ENUM_BUDGET;
	}
	else if( std_iequal( arg, ".COSTPLAN." ) )
	{
		type_object->m_enum = IfcCostScheduleTypeEnum::ENUM_COSTPLAN;
	}
	else if( std_iequal( arg, ".ESTIMATE." ) )
	{
		type_object->m_enum = IfcCostScheduleTypeEnum::ENUM_ESTIMATE;
	}
	else if( std_iequal( arg, ".PRICEDBILLOFQUANTITIES." ) )
	{
		type_object->m_enum = IfcCostScheduleTypeEnum::ENUM_PRICEDBILLOFQUANTITIES;
	}
	else if( std_iequal( arg, ".SCHEDULEOFRATES." ) )
	{
		type_object->m_enum = IfcCostScheduleTypeEnum::ENUM_SCHEDULEOFRATES;
	}
	else if( std_iequal( arg, ".TENDER." ) )
	{
		type_object->m_enum = IfcCostScheduleTypeEnum::ENUM_TENDER;
	}
	else if( std_iequal( arg, ".UNPRICEDBILLOFQUANTITIES." ) )
	{
		type_object->m_enum = IfcCostScheduleTypeEnum::ENUM_UNPRICEDBILLOFQUANTITIES;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcCostScheduleTypeEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcCostScheduleTypeEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}