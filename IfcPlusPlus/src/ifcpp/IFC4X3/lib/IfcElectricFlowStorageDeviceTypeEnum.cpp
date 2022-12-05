/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/model/BasicTypes.h"
#include "ifcpp/model/BuildingException.h"
#include "ifcpp/IFC4X3/include/IfcElectricFlowStorageDeviceTypeEnum.h"

// TYPE IfcElectricFlowStorageDeviceTypeEnum = ENUMERATION OF	(BATTERY	,CAPACITOR	,CAPACITORBANK	,COMPENSATOR	,HARMONICFILTER	,INDUCTOR	,INDUCTORBANK	,RECHARGER	,UPS	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcElectricFlowStorageDeviceTypeEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCELECTRICFLOWSTORAGEDEVICETYPEENUM("; }
	switch( m_enum )
	{
		case ENUM_BATTERY:	stream << ".BATTERY."; break;
		case ENUM_CAPACITOR:	stream << ".CAPACITOR."; break;
		case ENUM_CAPACITORBANK:	stream << ".CAPACITORBANK."; break;
		case ENUM_COMPENSATOR:	stream << ".COMPENSATOR."; break;
		case ENUM_HARMONICFILTER:	stream << ".HARMONICFILTER."; break;
		case ENUM_INDUCTOR:	stream << ".INDUCTOR."; break;
		case ENUM_INDUCTORBANK:	stream << ".INDUCTORBANK."; break;
		case ENUM_RECHARGER:	stream << ".RECHARGER."; break;
		case ENUM_UPS:	stream << ".UPS."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcElectricFlowStorageDeviceTypeEnum> IFC4X3::IfcElectricFlowStorageDeviceTypeEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcElectricFlowStorageDeviceTypeEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcElectricFlowStorageDeviceTypeEnum>(); }
	shared_ptr<IfcElectricFlowStorageDeviceTypeEnum> type_object( new IfcElectricFlowStorageDeviceTypeEnum() );
	if( std_iequal( arg, ".BATTERY." ) )
	{
		type_object->m_enum = IfcElectricFlowStorageDeviceTypeEnum::ENUM_BATTERY;
	}
	else if( std_iequal( arg, ".CAPACITOR." ) )
	{
		type_object->m_enum = IfcElectricFlowStorageDeviceTypeEnum::ENUM_CAPACITOR;
	}
	else if( std_iequal( arg, ".CAPACITORBANK." ) )
	{
		type_object->m_enum = IfcElectricFlowStorageDeviceTypeEnum::ENUM_CAPACITORBANK;
	}
	else if( std_iequal( arg, ".COMPENSATOR." ) )
	{
		type_object->m_enum = IfcElectricFlowStorageDeviceTypeEnum::ENUM_COMPENSATOR;
	}
	else if( std_iequal( arg, ".HARMONICFILTER." ) )
	{
		type_object->m_enum = IfcElectricFlowStorageDeviceTypeEnum::ENUM_HARMONICFILTER;
	}
	else if( std_iequal( arg, ".INDUCTOR." ) )
	{
		type_object->m_enum = IfcElectricFlowStorageDeviceTypeEnum::ENUM_INDUCTOR;
	}
	else if( std_iequal( arg, ".INDUCTORBANK." ) )
	{
		type_object->m_enum = IfcElectricFlowStorageDeviceTypeEnum::ENUM_INDUCTORBANK;
	}
	else if( std_iequal( arg, ".RECHARGER." ) )
	{
		type_object->m_enum = IfcElectricFlowStorageDeviceTypeEnum::ENUM_RECHARGER;
	}
	else if( std_iequal( arg, ".UPS." ) )
	{
		type_object->m_enum = IfcElectricFlowStorageDeviceTypeEnum::ENUM_UPS;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcElectricFlowStorageDeviceTypeEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcElectricFlowStorageDeviceTypeEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
