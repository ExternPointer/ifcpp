/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcElectricApplianceTypeEnum.h"

// TYPE IfcElectricApplianceTypeEnum = ENUMERATION OF	(DISHWASHER	,ELECTRICCOOKER	,FREESTANDINGELECTRICHEATER	,FREESTANDINGFAN	,FREESTANDINGWATERCOOLER	,FREESTANDINGWATERHEATER	,FREEZER	,FRIDGE_FREEZER	,HANDDRYER	,KITCHENMACHINE	,MICROWAVE	,PHOTOCOPIER	,REFRIGERATOR	,TUMBLEDRYER	,VENDINGMACHINE	,WASHINGMACHINE	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcElectricApplianceTypeEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCELECTRICAPPLIANCETYPEENUM("; }
	switch( m_enum )
	{
		case ENUM_DISHWASHER:	stream << ".DISHWASHER."; break;
		case ENUM_ELECTRICCOOKER:	stream << ".ELECTRICCOOKER."; break;
		case ENUM_FREESTANDINGELECTRICHEATER:	stream << ".FREESTANDINGELECTRICHEATER."; break;
		case ENUM_FREESTANDINGFAN:	stream << ".FREESTANDINGFAN."; break;
		case ENUM_FREESTANDINGWATERCOOLER:	stream << ".FREESTANDINGWATERCOOLER."; break;
		case ENUM_FREESTANDINGWATERHEATER:	stream << ".FREESTANDINGWATERHEATER."; break;
		case ENUM_FREEZER:	stream << ".FREEZER."; break;
		case ENUM_FRIDGE_FREEZER:	stream << ".FRIDGE_FREEZER."; break;
		case ENUM_HANDDRYER:	stream << ".HANDDRYER."; break;
		case ENUM_KITCHENMACHINE:	stream << ".KITCHENMACHINE."; break;
		case ENUM_MICROWAVE:	stream << ".MICROWAVE."; break;
		case ENUM_PHOTOCOPIER:	stream << ".PHOTOCOPIER."; break;
		case ENUM_REFRIGERATOR:	stream << ".REFRIGERATOR."; break;
		case ENUM_TUMBLEDRYER:	stream << ".TUMBLEDRYER."; break;
		case ENUM_VENDINGMACHINE:	stream << ".VENDINGMACHINE."; break;
		case ENUM_WASHINGMACHINE:	stream << ".WASHINGMACHINE."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcElectricApplianceTypeEnum> IFC4X3::IfcElectricApplianceTypeEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcElectricApplianceTypeEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcElectricApplianceTypeEnum>(); }
	shared_ptr<IfcElectricApplianceTypeEnum> type_object( new IfcElectricApplianceTypeEnum() );
	if( std_iequal( arg, ".DISHWASHER." ) )
	{
		type_object->m_enum = IfcElectricApplianceTypeEnum::ENUM_DISHWASHER;
	}
	else if( std_iequal( arg, ".ELECTRICCOOKER." ) )
	{
		type_object->m_enum = IfcElectricApplianceTypeEnum::ENUM_ELECTRICCOOKER;
	}
	else if( std_iequal( arg, ".FREESTANDINGELECTRICHEATER." ) )
	{
		type_object->m_enum = IfcElectricApplianceTypeEnum::ENUM_FREESTANDINGELECTRICHEATER;
	}
	else if( std_iequal( arg, ".FREESTANDINGFAN." ) )
	{
		type_object->m_enum = IfcElectricApplianceTypeEnum::ENUM_FREESTANDINGFAN;
	}
	else if( std_iequal( arg, ".FREESTANDINGWATERCOOLER." ) )
	{
		type_object->m_enum = IfcElectricApplianceTypeEnum::ENUM_FREESTANDINGWATERCOOLER;
	}
	else if( std_iequal( arg, ".FREESTANDINGWATERHEATER." ) )
	{
		type_object->m_enum = IfcElectricApplianceTypeEnum::ENUM_FREESTANDINGWATERHEATER;
	}
	else if( std_iequal( arg, ".FREEZER." ) )
	{
		type_object->m_enum = IfcElectricApplianceTypeEnum::ENUM_FREEZER;
	}
	else if( std_iequal( arg, ".FRIDGE_FREEZER." ) )
	{
		type_object->m_enum = IfcElectricApplianceTypeEnum::ENUM_FRIDGE_FREEZER;
	}
	else if( std_iequal( arg, ".HANDDRYER." ) )
	{
		type_object->m_enum = IfcElectricApplianceTypeEnum::ENUM_HANDDRYER;
	}
	else if( std_iequal( arg, ".KITCHENMACHINE." ) )
	{
		type_object->m_enum = IfcElectricApplianceTypeEnum::ENUM_KITCHENMACHINE;
	}
	else if( std_iequal( arg, ".MICROWAVE." ) )
	{
		type_object->m_enum = IfcElectricApplianceTypeEnum::ENUM_MICROWAVE;
	}
	else if( std_iequal( arg, ".PHOTOCOPIER." ) )
	{
		type_object->m_enum = IfcElectricApplianceTypeEnum::ENUM_PHOTOCOPIER;
	}
	else if( std_iequal( arg, ".REFRIGERATOR." ) )
	{
		type_object->m_enum = IfcElectricApplianceTypeEnum::ENUM_REFRIGERATOR;
	}
	else if( std_iequal( arg, ".TUMBLEDRYER." ) )
	{
		type_object->m_enum = IfcElectricApplianceTypeEnum::ENUM_TUMBLEDRYER;
	}
	else if( std_iequal( arg, ".VENDINGMACHINE." ) )
	{
		type_object->m_enum = IfcElectricApplianceTypeEnum::ENUM_VENDINGMACHINE;
	}
	else if( std_iequal( arg, ".WASHINGMACHINE." ) )
	{
		type_object->m_enum = IfcElectricApplianceTypeEnum::ENUM_WASHINGMACHINE;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcElectricApplianceTypeEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcElectricApplianceTypeEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
